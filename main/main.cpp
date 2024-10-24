#include <driver/gpio.h>
#include <driver/ledc.h>
#include <esp_adc/adc_continuous.h>
#include <hal/adc_types.h>
#include <hal/gpio_hal.h>

#include <cstdint>
#include <cstdio>

#include "NimBLEDevice.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "soc/soc_caps.h"

// #define DEBUG 1

#define CONFIG_SERVICE_UUID "c22fe686-2ea0-4eb3-8389-d568284e5b10"
#define DEVICE_NAME_CHARACTERISTIC_UUID "6f5e26b6-5920-4c44-ad4b-3b7893f8ef09"
#define BATTERY_AMOUNT_CHARACTERISTIC_UUID "0efa935a-9627-4f0b-9e8f-1b86084d2477"
#define WHEEL_DIAMETER_CHARACTERISTIC_UUID "63a4b587-33e2-49ba-9c8b-cf0ed8989bae"
#define WHEEL_DIVISIONS_CHARACTERISTIC_UUID "9b04682b-7eb0-449b-bd79-419064a43463"

#define OPERATION_SERVICE_UUID "0331e722-6d35-4922-8c69-f80d0f9fb9e7"
#define TURN_PULSE_CHARACTERISTIC_UUID "8fc09712-72c2-421e-865e-fd5436896cf2"

#define SENSOR_ADC_CHANNEL_A ADC_CHANNEL_3
#define SENSOR_ADC_CHANNEL_B ADC_CHANNEL_4

enum TurnDirection { BACKWARD, FORWARD, NONE };

const uint8_t turnStates[] = {
    0b00000000,
    0b00000001,
    0b00000011,
    0b00000010,
};

bool sensorAState = 0;
bool sensorBState = 0;
uint8_t currentTurnIndex = 0;
TurnDirection lastTurnDirection = TurnDirection::NONE;
uint8_t turnJitterCounter = 0;

BLEServer *btServer;
BLEService *configService;
BLECharacteristic *deviceNameCharacteristic;
BLECharacteristic *batteryAmountCharacteristic;
BLECharacteristic *wheelDiameterCharacteristic;
BLECharacteristic *wheelDivisionsCharacteristic;

BLEService *operationService;
BLECharacteristic *turnPulseCharacteristic;

class ConfigCharacteristicCallbacks : public BLECharacteristicCallbacks {
  using BLECharacteristicCallbacks::onWrite;
  void onWrite(BLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) {}
};

QueueHandle_t turnPulseNotifyingQueue;

#ifdef DEBUG
QueueHandle_t debugSensorAQueue;
QueueHandle_t debugSensorBQueue;
#endif

static bool IRAM_ATTR adcConvDoneCallback(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata,
                                          void *user_data) {
  uint32_t sensorARaw = 0;
  uint32_t sensorBRaw = 0;

  int sensorASampleCount = 0;
  int sensorBSampleCount = 0;

  for (int i = 0; i < edata->size; i += SOC_ADC_DIGI_RESULT_BYTES) {
    adc_digi_output_data_t *p = (adc_digi_output_data_t *)&edata->conv_frame_buffer[i];
    uint32_t channel = p->type2.channel;
    uint32_t data = p->type2.data;

    if (channel == SENSOR_ADC_CHANNEL_A) {
      sensorARaw += data;
      sensorASampleCount++;
    } else {
      sensorBRaw += data;
      sensorBSampleCount++;
    }
  }

  sensorARaw /= sensorASampleCount;
  sensorBRaw /= sensorBSampleCount;

  if (sensorAState && sensorARaw < 2000) {
    sensorAState = false;
  } else if (!sensorAState && sensorARaw > 3000) {
    sensorAState = true;
  }

  if (sensorBState && sensorBRaw < 600) {
    sensorBState = false;
  } else if (!sensorBState && sensorBRaw > 900) {
    sensorBState = true;
  }

#ifdef DEBUG
  uint32_t sensorADebug = sensorARaw | (sensorAState << 31);
  uint32_t sensorBDebug = sensorBRaw | (sensorBState << 31);

  xQueueSendToBackFromISR(debugSensorAQueue, &sensorADebug, nullptr);
  xQueueSendToBackFromISR(debugSensorBQueue, &sensorBDebug, nullptr);
#endif

  uint8_t currentTurnState = sensorAState | (sensorBState << 1);

  if (currentTurnState == turnStates[(currentTurnIndex + 1) & 3]) {
    currentTurnIndex++;
  } else if (currentTurnState == turnStates[(currentTurnIndex - 1) & 3]) {
    currentTurnIndex--;
  }

  TurnDirection turnDirection = TurnDirection::NONE;

  if (currentTurnIndex == 255) {
    currentTurnIndex = 3;
    turnDirection = TurnDirection::BACKWARD;
  } else if (currentTurnIndex == 4) {
    currentTurnIndex = 0;
    turnDirection = TurnDirection::FORWARD;
  }

  if (turnDirection != TurnDirection::NONE) {
    if (turnDirection != lastTurnDirection) {
      turnJitterCounter++;
    } else {
      turnJitterCounter = 0;
    }
    lastTurnDirection = turnDirection;

    // if (turnJitterCounter < 2) {
    xQueueSendToBackFromISR(turnPulseNotifyingQueue, &turnDirection, nullptr);
    //}
  }

  return true;
}

extern "C" {
void app_main() {
  //
  BLEDevice::init("Metrino v1");

  btServer = BLEDevice::createServer();

  configService = btServer->createService(CONFIG_SERVICE_UUID);

  deviceNameCharacteristic = configService->createCharacteristic(DEVICE_NAME_CHARACTERISTIC_UUID,
                                                                 NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
  batteryAmountCharacteristic =
      configService->createCharacteristic(BATTERY_AMOUNT_CHARACTERISTIC_UUID, NIMBLE_PROPERTY::READ);
  wheelDiameterCharacteristic = configService->createCharacteristic(WHEEL_DIAMETER_CHARACTERISTIC_UUID,
                                                                    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
  wheelDivisionsCharacteristic = configService->createCharacteristic(WHEEL_DIVISIONS_CHARACTERISTIC_UUID,
                                                                     NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);

  /*
  deviceNameCharacteristic->setCallbacks(new ConfigCharacteristicCallbacks());
  wheelDiameterCharacteristic->setCallbacks(
      new ConfigCharacteristicCallbacks());
  wheelDivisionsCharacteristic->setCallbacks(
      new ConfigCharacteristicCallbacks());*/

  configService->start();

  operationService = btServer->createService(OPERATION_SERVICE_UUID);
  turnPulseCharacteristic = operationService->createCharacteristic(
      TURN_PULSE_CHARACTERISTIC_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
  turnPulseCharacteristic->setValue(0);
  operationService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(CONFIG_SERVICE_UUID);
  pAdvertising->addServiceUUID(OPERATION_SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMaxPreferred(0x12);

  BLEDevice::startAdvertising();

  turnPulseNotifyingQueue = xQueueCreate(1024, 1);

#ifdef DEBUG
  debugSensorAQueue = xQueueCreate(32, 4);
  debugSensorBQueue = xQueueCreate(32, 4);
#endif

  adc_continuous_handle_t adcDriverHandle = NULL;

  adc_continuous_handle_cfg_t adc_config = {
      .max_store_buf_size = 1024,
      .conv_frame_size = 8,
  };
  ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &adcDriverHandle));

  adc_continuous_config_t dig_cfg = {
      .sample_freq_hz = 20 * 1000,
      .conv_mode = ADC_CONV_SINGLE_UNIT_1,
      .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
  };

  adc_digi_pattern_config_t adc_pattern[] = {
      {
          .atten = ADC_ATTEN_DB_0,
          .channel = SENSOR_ADC_CHANNEL_A,
          .unit = ADC_UNIT_1,
          .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH,
      },
      {
          .atten = ADC_ATTEN_DB_0,
          .channel = SENSOR_ADC_CHANNEL_B,
          .unit = ADC_UNIT_1,
          .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH,
      },
  };
  dig_cfg.pattern_num = 2;

  dig_cfg.adc_pattern = adc_pattern;
  ESP_ERROR_CHECK(adc_continuous_config(adcDriverHandle, &dig_cfg));

  adc_continuous_evt_cbs_t cbs = {
      .on_conv_done = adcConvDoneCallback,
  };
  ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(adcDriverHandle, &cbs, NULL));
  ESP_ERROR_CHECK(adc_continuous_start(adcDriverHandle));

  while (true) {
#ifdef DEBUG
    unsigned int stateA;
    unsigned int stateB;

    if (xQueueReceive(debugSensorAQueue, &stateA, 1000 / portTICK_PERIOD_MS) &&
        xQueueReceive(debugSensorBQueue, &stateB, 1000 / portTICK_PERIOD_MS)) {
      // ESP_LOGI("dsg", "%u", (stateB & 1 << 31) >> 31);
      ESP_LOGI("dsg", ", %u, %u", stateA & ~(1 << 31), /* (stateA & 1 << 31) >> 31,*/
               stateB & ~(1 << 31) /*, (stateB & 1 << 31) >> 31*/);
    }

    vTaskDelay(1);
    continue;
#endif

    uint8_t turnDirection;

    if (xQueueReceive(turnPulseNotifyingQueue, &turnDirection, 1000 / portTICK_PERIOD_MS)) {
      turnPulseCharacteristic->setValue(turnDirection);
      turnPulseCharacteristic->notify();
    }
  }
}
}