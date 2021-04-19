#include "esphome.h"

class Hpma115C0Sensor : public PollingComponent, public sensor::Sensor, public UARTDevice {
 public:
  Hpma115C0Sensor(uint32_t update_interval, UARTComponent *parent) : PollingComponent(update_interval), UARTDevice(parent) {}

  static const char MSG_CHAR_1 = 0x42;
  static const char MSG_CHAR_2 = 0x4d;

  uint8_t count = 0;
  unsigned char data[32] = {};

  void setup() override {
    // nothing to do here
  }
  void update() override {

    count = 0;
    data[32] = {};

    while (available() && count < 32) {
      data[count] = read();
      if (count == 0 && data[0] != MSG_CHAR_1) {
        ESP_LOGD("hpma115c0", "unexpected first char %c", data[0]);
        break;
      }
      if (count == 1 && data[1] != MSG_CHAR_2) {
        ESP_LOGD("hpma115c0", "unexpected second char %c", data[1]);
        break;
      }      
      count++;
    }

    ESP_LOGD("hpma115c0", "read: %s", data);
    publish_state(0);
  }
};