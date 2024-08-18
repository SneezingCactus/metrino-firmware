#include <driver/gpio.h>
#include <driver/ledc.h>
#include <esp_adc/adc_continuous.h>
#include <hal/adc_types.h>
#include <hal/gpio_hal.h>

// #include "NimBLEDevice.h"
#include "esp_log.h"
#include "led_strip.h"
#include "soc/soc_caps.h"

#define CONFIG_SERVICE_UUID "c22fe686-2ea0-4eb3-8389-d568284e5b10"
#define DEVICE_NAME_CHARACTERISTIC_UUID "6f5e26b6-5920-4c44-ad4b-3b7893f8ef09"
#define BATTERY_AMOUNT_CHARACTERISTIC_UUID                                     \
  "0efa935a-9627-4f0b-9e8f-1b86084d2477"
#define WHEEL_DIAMETER_CHARACTERISTIC_UUID                                     \
  "63a4b587-33e2-49ba-9c8b-cf0ed8989bae"
#define WHEEL_DIVISIONS_CHARACTERISTIC_UUID                                    \
  "9b04682b-7eb0-449b-bd79-419064a43463"

#define OPERATION_SERVICE_UUID "0331e722-6d35-4922-8c69-f80d0f9fb9e7"
#define TURN_PULSE_CHARACTERISTIC_UUID "8fc09712-72c2-421e-865e-fd5436896cf2"

#define ENCODER_PIN_A GPIO_NUM_32
#define ENCODER_PIN_B GPIO_NUM_33

enum TurnState {
  BACKWARD,
  IDLE,
  FORWARD,
};

uint64_t bitches1 = 0;
uint64_t bitches2 = 0;

bool fuck1 = false;
bool fuck2 = false;

TurnState turnState = TurnState::IDLE;
/*
BLEServer *btServer;
BLEService *configService;
BLECharacteristic *deviceNameCharacteristic;
BLECharacteristic *batteryAmountCharacteristic;
BLECharacteristic *wheelDiameterCharacteristic;
BLECharacteristic *wheelDivisionsCharacteristic;

BLEService *operationService;
BLECharacteristic *turnPulseCharacteristic;

QueueHandle_t turnPulseNotifyingQueue;

class ConfigCharacteristicCallbacks : public BLECharacteristicCallbacks {
  using BLECharacteristicCallbacks::onWrite;
  void onWrite(BLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) {}
};
*/

void IRAM_ATTR encoderPinAInterrupt(void *arg) {
  /*
  bool isHigh = gpio_get_level(ENCODER_PIN_A);

  unsigned char direction = 0x11 + 0x10 * isHigh;
  xQueueSendToBackFromISR(turnPulseNotifyingQueue, &direction, nullptr);
  return;

  switch (turnState) {
    case TurnState::IDLE:
      if (!isHigh) turnState = TurnState::BACKWARD_STAGE1;
      break;
    case TurnState::FORWARD_STAGE1:
      if (!isHigh) turnState = TurnState::FORWARD_STAGE2;
      break;
    case TurnState::FORWARD_STAGE3:
      if (isHigh) {
        turnState = TurnState::IDLE;
        unsigned char direction = 1;
        xQueueSendToBackFromISR(turnPulseNotifyingQueue, &direction, nullptr);
      }
      break;
    case BACKWARD_STAGE2:
      if (isHigh) turnState = TurnState::BACKWARD_STAGE3;
      break;
    default:
      // turnState = TurnState::IDLE;
      break;
  }*/
}

void IRAM_ATTR encoderPinBInterrupt(void *arg) {
  /*
  bool isHigh = gpio_get_level(ENCODER_PIN_B);

  unsigned char direction = 0x10 + 0x10 * isHigh;
  xQueueSendToBackFromISR(turnPulseNotifyingQueue, &direction, nullptr);
  return;

  switch (turnState) {
    case TurnState::IDLE:
      if (!isHigh) turnState = TurnState::FORWARD_STAGE1;
      break;
    case TurnState::BACKWARD_STAGE1:
      if (!isHigh) turnState = TurnState::BACKWARD_STAGE2;
      break;
    case TurnState::BACKWARD_STAGE3:
      if (isHigh) {
        turnState = TurnState::IDLE;
        unsigned char direction = 0;
        xQueueSendToBackFromISR(turnPulseNotifyingQueue, &direction, nullptr);
      }
      break;
    case FORWARD_STAGE2:
      if (isHigh) turnState = TurnState::FORWARD_STAGE3;
      break;
    default:
      break;
  }*/
}

void turnPulseNotifyingTask(void *arg) {
  while (true) {
    /*bool pinAHigh = gpio_get_level(ENCODER_PIN_A);
    bool pinBHigh = gpio_get_level(ENCODER_PIN_B);

    switch (turnState) {
      case TurnState::IDLE:
        if (pinAHigh) {
          turnState = TurnState::BACKWARD_STAGE1;
        } else if (pinBHigh) {
          turnState = TurnState::FORWARD_STAGE1;
        }
        break;
      case TurnState::FORWARD_STAGE1:
        if (pinAHigh) turnState = TurnState::FORWARD_STAGE2;
        break;
      case TurnState::FORWARD_STAGE2:
        if (!pinBHigh) turnState = TurnState::FORWARD_STAGE3;
        break;
      case TurnState::FORWARD_STAGE3:
        if (!pinAHigh) {
          turnPulseCharacteristic->setValue(1);
          turnPulseCharacteristic->notify();
          turnState = TurnState::IDLE;
        }
        break;
      case TurnState::BACKWARD_STAGE1:
        if (pinBHigh) turnState = TurnState::BACKWARD_STAGE2;
        break;
      case TurnState::BACKWARD_STAGE2:
        if (!pinAHigh) turnState = TurnState::BACKWARD_STAGE3;
        break;
      case TurnState::BACKWARD_STAGE3:
        if (!pinBHigh) {
          turnPulseCharacteristic->setValue(0);
          turnPulseCharacteristic->notify();
          turnState = TurnState::IDLE;
        }
        break;
    }*/

    /*
    if (pinAHigh && turnState == TurnState::IDLE) {
      turnState = TurnState::BACKWARD_STAGE1;
    }
    */
    bool shit1 = !gpio_get_level(ENCODER_PIN_A);
    bool shit2 = !gpio_get_level(ENCODER_PIN_B);

    /*
    switch (turnState) {
      case TurnState::IDLE:
        if (shit1) {
          turnState = TurnState::BACKWARD;
        } else if (shit2) {
          turnState = TurnState::FORWARD;
        }
        break;
      case TurnState::BACKWARD:
        if (!shit1) {
          turnState = TurnState::IDLE;
          ESP_LOGI("fuck", "left");
        }
        break;
      case TurnState::FORWARD:
        if (!shit2) {
          turnState = TurnState::IDLE;
          ESP_LOGI("fuck", "right");
        }
        break;
    }*/

    if (fuck1 != shit1 || fuck2 != shit2) {
      // ESP_LOGI("fuck", "%d %d", shit1, shit2);
    }

    if (fuck1 != shit1) {
      fuck1 = shit1;
    }

    if (fuck2 != shit2) {
      fuck2 = shit2;
    }

    /*unsigned char direction;
    if (xQueueReceive(turnPulseNotifyingQueue, &direction, 1000 /
    portTICK_PERIOD_MS)) { ESP_LOGI("fuck", "%x", direction);
      turnPulseCharacteristic->setValue(direction);
      turnPulseCharacteristic->notify();
    }*/
  }
}

extern "C" {
void app_main() {
  /*
  BLEDevice::init("Long name works now");

  btServer = BLEDevice::createServer();

  configService = btServer->createService(CONFIG_SERVICE_UUID);

  deviceNameCharacteristic =
  configService->createCharacteristic(DEVICE_NAME_CHARACTERISTIC_UUID,
                                                                 NIMBLE_PROPERTY::READ
  | NIMBLE_PROPERTY::WRITE); batteryAmountCharacteristic =
      configService->createCharacteristic(BATTERY_AMOUNT_CHARACTERISTIC_UUID,
  NIMBLE_PROPERTY::READ); wheelDiameterCharacteristic =
  configService->createCharacteristic(WHEEL_DIAMETER_CHARACTERISTIC_UUID,
                                                                    NIMBLE_PROPERTY::READ
  | NIMBLE_PROPERTY::WRITE); wheelDivisionsCharacteristic =
  configService->createCharacteristic(WHEEL_DIVISIONS_CHARACTERISTIC_UUID,
                                                                     NIMBLE_PROPERTY::READ
  | NIMBLE_PROPERTY::WRITE);

  deviceNameCharacteristic->setCallbacks(new ConfigCharacteristicCallbacks());
  wheelDiameterCharacteristic->setCallbacks(new
  ConfigCharacteristicCallbacks());
  wheelDivisionsCharacteristic->setCallbacks(new
  ConfigCharacteristicCallbacks());

  configService->start();

  operationService = btServer->createService(OPERATION_SERVICE_UUID);
  turnPulseCharacteristic = operationService->createCharacteristic(
      TURN_PULSE_CHARACTERISTIC_UUID, NIMBLE_PROPERTY::READ |
  NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
  turnPulseCharacteristic->setValue(TurnState::IDLE);
  operationService->start();

  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(CONFIG_SERVICE_UUID);
  pAdvertising->addServiceUUID(OPERATION_SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone
  connections issue pAdvertising->setMaxPreferred(0x12);

  BLEDevice::startAdvertising();

  turnPulseNotifyingQueue = xQueueCreate(8, 1);
  xTaskCreatePinnedToCore(&turnPulseNotifyingTask, "turnPulseNotifying", 4096,
  NULL, 5, nullptr, 1);

  gpio_config_t io_conf = {
      .pin_bit_mask = (1ULL << ENCODER_PIN_A) | (1ULL << ENCODER_PIN_B),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_ANYEDGE,
  };
  gpio_config(&io_conf);*/

  /*
  led_strip_handle_t led_strip;

  // LED strip initialization with the GPIO and pixels number
  led_strip_config_t strip_config = {
      .strip_gpio_num = GPIO_NUM_48,             // The GPIO that connected to
  the LED strip's data line .max_leds = 1,                             // The
  number of LEDs in the strip, .led_pixel_format = LED_PIXEL_FORMAT_GRB,  //
  Pixel format of your LED strip .led_model = LED_MODEL_WS2812,             //
  LED strip model .flags =
          {
              .invert_out = false,
          },
  };

  led_strip_rmt_config_t rmt_config = {
      .clk_src = RMT_CLK_SRC_DEFAULT,     // different clock source can lead to
  different power consumption .resolution_hz = 10 * 1000 * 1000,  // 10MHz
      .flags =
          {
              .with_dma = false,
          },
  };
  ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config,
  &led_strip));

  int balls = 0;

  while (true) {
    led_strip_set_pixel_hsv(led_strip, 0, balls, 255, 255);
    led_strip_refresh(led_strip);
    balls += 2;
    if (balls == 360) balls = 0;
    vTaskDelay(1);
  }*/

  adc_continuous_handle_t adcDriverHandle = NULL;

  adc_continuous_handle_cfg_t adc_config = {
      .max_store_buf_size = 1024,
      .conv_frame_size = 256,
  };
  ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &adcDriverHandle));

  adc_continuous_config_t dig_cfg = {
      .sample_freq_hz = 20 * 1000,
      .conv_mode = ADC_CONV_SINGLE_UNIT_1,
      .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
  };

  adc_digi_pattern_config_t adc_pattern[] = {
      {
          .atten = ADC_ATTEN_DB_0,
          .channel = ADC_CHANNEL_3,
          .unit = ADC_UNIT_1,
          .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH,
      },
      {
          .atten = ADC_ATTEN_DB_0,
          .channel = ADC_CHANNEL_4,
          .unit = ADC_UNIT_1,
          .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH,
      },
  };
  dig_cfg.pattern_num = 2;

  dig_cfg.adc_pattern = adc_pattern;
  ESP_ERROR_CHECK(adc_continuous_config(adcDriverHandle, &dig_cfg));

  /*
  gpio_intr_enable(ENCODER_PIN_A);
  gpio_intr_enable(ENCODER_PIN_B);
  gpio_install_isr_service(0);
  gpio_isr_handler_add(ENCODER_PIN_A, encoderPinAInterrupt, NULL);
  gpio_isr_handler_add(ENCODER_PIN_B, encoderPinBInterrupt, NULL);*/

  /*
  adc_oneshot_unit_handle_t adc1_handle;
  adc_oneshot_unit_init_cfg_t init_config1 = {
      .unit_id = ADC_UNIT_1,
      .ulp_mode = ADC_ULP_MODE_DISABLE,
  };
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

  adc_oneshot_chan_cfg_t config = {
      .atten = ADC_ATTEN_DB_12,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
  };
  ESP_ERROR_CHECK(
      adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_3, &config));
  ESP_ERROR_CHECK(
      adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_4, &config));

  bool sensor1LastState = false;
  bool sensor2LastState = false;
  int count = 0;

  while (true) {
    int sensor1Value, sensor2Value;
    adc_oneshot_read(adc1_handle, ADC_CHANNEL_3, &sensor1Value);
    adc_oneshot_read(adc1_handle, ADC_CHANNEL_4, &sensor2Value);

    if (sensor1Value > 120 && !sensor1LastState) {
      count++;
      ESP_LOGI("fuck", "SEN1 ON!!!! %d", count);
      sensor1LastState = true;
    } else if (sensor1Value < 90 && sensor1LastState) {
      ESP_LOGI("fuck", "SEN1 Off.");
      sensor1LastState = false;
    }

    if (sensor2Value > 100 && !sensor2LastState) {
      count++;
      ESP_LOGI("fuck", "SEN2 ON!!!! %d", count);
      sensor2LastState = true;
    } else if (sensor2Value < 50 && sensor2LastState) {
      ESP_LOGI("fuck", "SEN2 Off.");
      sensor2LastState = false;
    }
  }*/
}
}