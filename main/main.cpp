#include <driver/gpio.h>
#include <driver/ledc.h>
#include <esp_adc/adc_continuous.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_sleep.h>
#include <hal/adc_types.h>
#include <hal/gpio_hal.h>
#include <math.h>

#include <cstdint>
#include <cstdio>

#include "NimBLEDevice.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "led_strip.h"
#include "soc/soc_caps.h"

// El modo DEBUG permite inspeccionar las mediciones recibidas por el ADC del encoder.
// Esto puede ser útil a la hora de calibrar el encoder, o diagnosticar problemas de conexionado.
// Para activar el modo DEBUG descomente la siguiente línea:

// #define DEBUG 1

// ------------ Monitoreo de la batería y LED indicador ------------ //
#define BATTERY_ADC_CHANNEL ADC_CHANNEL_4
#define STATUS_LED_GPIO GPIO_NUM_48

// relación del divisor de voltaje tal que R1 = 55.3 kohm, R2 = 9.89 kohm, usando la fórmula R2 / (R1 + R2)
#define BATTERY_VOLTAGE_DIVIDER_RATIO 0.151710385
#define BATTERY_TRACKED_VOLTAGE_INITIAL 4800
#define BATTERY_VOLTAGE_ALERT_LEVEL 4200
#define BATTERY_VOLTAGE_SHUTDOWN_LEVEL 4000
#define BATTERY_VOLTAGE_MAX_LEVEL 6000

led_strip_handle_t ledStripHandle;
adc_oneshot_unit_handle_t batteryAdcHandle;
adc_cali_handle_t batteryAdcCaliHandle;

// -------------------------- Encoder ------------------------------ //
#define SENSOR_ADC_CHANNEL_A ADC_CHANNEL_5
#define SENSOR_ADC_CHANNEL_B ADC_CHANNEL_6

const uint8_t turnStates[] = {
    0b00000000,
    0b00000001,
    0b00000011,
    0b00000010,
};

bool sensorAState = 0;
bool sensorBState = 0;
uint8_t currentTurnStateIndex = 0;

adc_continuous_handle_t encoderAdcHandle;

// ------------------- Parámetros editables ------------------------ //
char *deviceName = "Metrino v1";
double wheelDiameter = 0.08;
int wheelSlots = 32;

double currentMeasurement;

// ------------------------ Bluetooth ------------------------------ //
#define PARAMETER_SERVICE_UUID "c22fe686-2ea0-4eb3-8389-d568284e5b10"
#define DEVICE_NAME_CHARACTERISTIC_UUID "6f5e26b6-5920-4c44-ad4b-3b7893f8ef09"
#define BATTERY_AMOUNT_CHARACTERISTIC_UUID "0efa935a-9627-4f0b-9e8f-1b86084d2477"
#define WHEEL_DIAMETER_CHARACTERISTIC_UUID "63a4b587-33e2-49ba-9c8b-cf0ed8989bae"
#define WHEEL_DIVISIONS_CHARACTERISTIC_UUID "9b04682b-7eb0-449b-bd79-419064a43463"
#define OPERATION_SERVICE_UUID "0331e722-6d35-4922-8c69-f80d0f9fb9e7"
#define MEASUREMENT_CHARACTERISTIC_UUID "8fc09712-72c2-421e-865e-fd5436896cf2"

BLEServer *btServer;
BLEService *configService;
BLECharacteristic *deviceNameCharacteristic;
BLECharacteristic *batteryAmountCharacteristic;
BLECharacteristic *wheelDiameterCharacteristic;
BLECharacteristic *wheelSlotsCharacteristic;
BLEService *operationService;
BLECharacteristic *measurementCharacteristic;

bool bluetoothInitialized = false;
uint32_t connectedClientAmount = 0;

class ServerCallbacks : public BLEServerCallbacks {
  using BLEServerCallbacks::onConnect;
  void onConnect(NimBLEServer *pServer, ble_gap_conn_desc *desc) { connectedClientAmount++; }

  using BLEServerCallbacks::onDisconnect;
  void onDisconnect(NimBLEServer *pServer, ble_gap_conn_desc *desc) { connectedClientAmount--; }
};

class MeasurementCharacteristicCallbacks : public BLECharacteristicCallbacks {
  using BLECharacteristicCallbacks::onWrite;
  void onWrite(BLECharacteristic *pCharacteristic, ble_gap_conn_desc *desc) {
    currentMeasurement = *(double *)pCharacteristic->getValue().data();
  }
};

// ------------------------ FreeRTOS ------------------------------- //
SemaphoreHandle_t measurementUpdateSemaphore;

#ifdef DEBUG
QueueHandle_t debugSensorAQueue;
QueueHandle_t debugSensorBQueue;
#endif

// función llamada cada vez que el ADC del encoder tiene datos nuevos
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

  if (sensorAState && sensorARaw < 600) {
    sensorAState = false;
  } else if (!sensorAState && sensorARaw > 1000) {
    sensorAState = true;
  }

  if (sensorBState && sensorBRaw < 600) {
    sensorBState = false;
  } else if (!sensorBState && sensorBRaw > 1000) {
    sensorBState = true;
  }

#ifdef DEBUG
  uint32_t sensorADebug = sensorARaw | (sensorAState << 31);
  uint32_t sensorBDebug = sensorBRaw | (sensorBState << 31);

  xQueueSendToBackFromISR(debugSensorAQueue, &sensorADebug, nullptr);
  xQueueSendToBackFromISR(debugSensorBQueue, &sensorBDebug, nullptr);
#endif

  uint8_t currentTurnState = sensorAState | (sensorBState << 1);

  if (currentTurnState == turnStates[(currentTurnStateIndex + 1) & 3]) {
    currentTurnStateIndex++;
  } else if (currentTurnState == turnStates[(currentTurnStateIndex - 1) & 3]) {
    currentTurnStateIndex--;
  }

  int turnDirection = 0;

  if (currentTurnStateIndex == 255) {
    currentTurnStateIndex = 3;
    turnDirection = 1;
  } else if (currentTurnStateIndex == 4) {
    currentTurnStateIndex = 0;
    turnDirection = -1;
  }

  if (turnDirection != 0) {
    currentMeasurement += turnDirection * M_PI * wheelDiameter / wheelSlots;
    xSemaphoreGiveFromISR(measurementUpdateSemaphore, 0);
  }

  return true;
}

// hilo dedicado al monitoreo de la batería y la actualización del LED indicador de estado
void statusLedTask(void *unused) {
  double trackedVoltage = BATTERY_TRACKED_VOLTAGE_INITIAL;
  int rawMeasurement;
  int voltMeasurement;

  while (true) {
    adc_oneshot_read(batteryAdcHandle, BATTERY_ADC_CHANNEL, &rawMeasurement);
    adc_cali_raw_to_voltage(batteryAdcCaliHandle, rawMeasurement, &voltMeasurement);
    trackedVoltage += (voltMeasurement / BATTERY_VOLTAGE_DIVIDER_RATIO - trackedVoltage) * 0.5;

    if (bluetoothInitialized) {
      batteryAmountCharacteristic->setValue(trackedVoltage / 1000);
    }

    if (trackedVoltage < BATTERY_VOLTAGE_SHUTDOWN_LEVEL) {
      led_strip_set_pixel(ledStripHandle, 0, 32, 0, 0);
      led_strip_refresh(ledStripHandle);
      esp_deep_sleep_start();
    }

    if (connectedClientAmount == 0) {
      led_strip_set_pixel(ledStripHandle, 0, 0, 0, 0);
      led_strip_refresh(ledStripHandle);
      vTaskDelay(500 / portTICK_PERIOD_MS);
      led_strip_set_pixel(ledStripHandle, 0, 0, 0, 32);
      led_strip_refresh(ledStripHandle);
      vTaskDelay(500 / portTICK_PERIOD_MS);
    } else {
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    if (trackedVoltage < BATTERY_VOLTAGE_ALERT_LEVEL) {
      led_strip_set_pixel(ledStripHandle, 0, 32, 0, 0);
      led_strip_refresh(ledStripHandle);
      vTaskDelay(50 / portTICK_PERIOD_MS);
      led_strip_set_pixel(ledStripHandle, 0, 0, 0, 0);
      led_strip_refresh(ledStripHandle);
      vTaskDelay(50 / portTICK_PERIOD_MS);
      led_strip_set_pixel(ledStripHandle, 0, 32, 0, 0);
      led_strip_refresh(ledStripHandle);
      vTaskDelay(50 / portTICK_PERIOD_MS);
      led_strip_set_pixel(ledStripHandle, 0, 0, 0, 32);
      led_strip_refresh(ledStripHandle);
    }
  }
}

// inicialización del LED indicador de estado y el ADC dedicado a medir la carga de la batería
void statusLedInit() {
  led_strip_config_t ledStripConfig = {
      .strip_gpio_num = STATUS_LED_GPIO,
      .max_leds = 1,
  };
  led_strip_rmt_config_t ledStripRmtConfig = {
      .resolution_hz = 10 * 1000 * 1000,  // 10MHz
      .flags = {.with_dma = false},
  };
  ESP_ERROR_CHECK(led_strip_new_rmt_device(&ledStripConfig, &ledStripRmtConfig, &ledStripHandle));

  led_strip_clear(ledStripHandle);

  adc_cali_curve_fitting_config_t adcCalibrationConfig = {
      .unit_id = ADC_UNIT_2,
      .atten = ADC_ATTEN_DB_0,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
  };
  ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&adcCalibrationConfig, &batteryAdcCaliHandle));

  adc_oneshot_unit_init_cfg_t adcInitConfig = {
      .unit_id = ADC_UNIT_2,
      .ulp_mode = ADC_ULP_MODE_DISABLE,
  };
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&adcInitConfig, &batteryAdcHandle));

  adc_oneshot_chan_cfg_t adcChannelConfig = {
      .atten = ADC_ATTEN_DB_0,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
  };
  ESP_ERROR_CHECK(adc_oneshot_config_channel(batteryAdcHandle, BATTERY_ADC_CHANNEL, &adcChannelConfig));

  xTaskCreatePinnedToCore(&statusLedTask, "statusLedTask", 2048, NULL, 1, NULL, 0);
}

// inicialización del servicio Bluetooth
void bluetoothInit() {
  BLEDevice::init(deviceName);

  btServer = BLEDevice::createServer();

  configService = btServer->createService(PARAMETER_SERVICE_UUID);

  deviceNameCharacteristic = configService->createCharacteristic(DEVICE_NAME_CHARACTERISTIC_UUID,
                                                                 NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
  batteryAmountCharacteristic =
      configService->createCharacteristic(BATTERY_AMOUNT_CHARACTERISTIC_UUID, NIMBLE_PROPERTY::READ);
  wheelDiameterCharacteristic = configService->createCharacteristic(WHEEL_DIAMETER_CHARACTERISTIC_UUID,
                                                                    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
  wheelSlotsCharacteristic = configService->createCharacteristic(WHEEL_DIVISIONS_CHARACTERISTIC_UUID,
                                                                 NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);

  deviceNameCharacteristic->setValue((uint8_t *)deviceName, strlen(deviceName));
  wheelDiameterCharacteristic->setValue(wheelDiameter);
  wheelSlotsCharacteristic->setValue(wheelSlots);

  btServer->setCallbacks(new ServerCallbacks());

  configService->start();

  operationService = btServer->createService(OPERATION_SERVICE_UUID);

  measurementCharacteristic = operationService->createCharacteristic(
      MEASUREMENT_CHARACTERISTIC_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
  measurementCharacteristic->setValue(0);
  measurementCharacteristic->setCallbacks(new MeasurementCharacteristicCallbacks());

  operationService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(PARAMETER_SERVICE_UUID);
  pAdvertising->addServiceUUID(OPERATION_SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMaxPreferred(0x12);

  BLEDevice::startAdvertising();

  bluetoothInitialized = true;
}

// Inicialización del ADC dedicado al encoder
void encoderInit() {
  measurementUpdateSemaphore = xSemaphoreCreateBinary();

#ifdef DEBUG
  debugSensorAQueue = xQueueCreate(32, 4);
  debugSensorBQueue = xQueueCreate(32, 4);
#endif

  adc_continuous_handle_cfg_t adcConfig = {
      .max_store_buf_size = 1024,
      .conv_frame_size = 8,
  };
  ESP_ERROR_CHECK(adc_continuous_new_handle(&adcConfig, &encoderAdcHandle));

  adc_continuous_config_t digConfig = {
      .sample_freq_hz = 20 * 1000,
      .conv_mode = ADC_CONV_SINGLE_UNIT_1,
      .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
  };

  adc_digi_pattern_config_t adcPattern[] = {
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
  digConfig.pattern_num = 2;
  digConfig.adc_pattern = adcPattern;

  ESP_ERROR_CHECK(adc_continuous_config(encoderAdcHandle, &digConfig));

  adc_continuous_evt_cbs_t adcCallbacks = {
      .on_conv_done = adcConvDoneCallback,
  };
  ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(encoderAdcHandle, &adcCallbacks, NULL));

  ESP_ERROR_CHECK(adc_continuous_start(encoderAdcHandle));
}

extern "C" {
// hilo principal, encargado de inicializar los subsistemas y enviar la medición
void app_main() {
  statusLedInit();
  bluetoothInit();
  encoderInit();

  while (true) {
#ifdef DEBUG
    unsigned int stateA;
    unsigned int stateB;

    if (xQueueReceive(debugSensorAQueue, &stateA, 1000 / portTICK_PERIOD_MS) &&
        xQueueReceive(debugSensorBQueue, &stateB, 1000 / portTICK_PERIOD_MS)) {
      ESP_LOGI("metrino_debug", "%u, %u", stateA & ~(1 << 31), stateB & ~(1 << 31));
    }

    vTaskDelay(1);
    continue;
#endif

    if (xSemaphoreTake(measurementUpdateSemaphore, portMAX_DELAY)) {
      measurementCharacteristic->setValue(currentMeasurement);
      measurementCharacteristic->notify();
      vTaskDelay(16 / portTICK_PERIOD_MS);
    }
  }
}
}