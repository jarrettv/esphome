#include "esphome.h"

class Hpma115C0Component : public PollingComponent, public UARTDevice {
 public:
  Hpma115C0Component(uint32_t update_interval, UARTComponent *parent) : PollingComponent(update_interval), UARTDevice(parent) {}

  float get_setup_priority() const override { return esphome::setup_priority::BUS; }

  Sensor *pm_1_sensor = new Sensor();
  Sensor *pm_2_5_sensor = new Sensor();
  Sensor *pm_4_sensor = new Sensor();
  Sensor *pm_10_sensor = new Sensor();

  static const char MSG_CHAR_1 = 0x42;
  static const char MSG_CHAR_2 = 0x4d;

  int sum = 0;
  uint8_t count = 0;
  unsigned char data[32] = {};

  void setup() override {
    // nothing to do here
  }
  void update() override {

    sum = 0;
    count = 0;
    data[32] = {};

    while (available() && count < 32) {
      data[count] = read();
      if (count == 0 && data[0] != MSG_CHAR_1) {
        ESP_LOGD("hpma115c0", "Unexpected first char %c", data[0]);
        break;
      }
      if (count == 1 && data[1] != MSG_CHAR_2) {
        ESP_LOGD("hpma115c0", "Unexpected second char %c", data[1]);
        break;
      }
      if (count < 30) sum += data[count];
      count++;
    }

    if (count < 32 || sum == 0) {
      ESP_LOGW("hpma115c0", "Unable to read packet");
      return;
    }

    int checksum = (data[30] << 8) | data[31];

    if (sum != checksum) {
      ESP_LOGW("hpma115c0", "Checksum invalid: %i <> %i", sum, checksum);
      return;
    }

    auto pm = (data[4] * 256) + data[5];
    pm_1_sensor->publish_state(pm);

    pm = (data[6] * 256) + data[7];
    pm_2_5_sensor->publish_state(pm);

    pm = (data[8] * 256) + data[9];
    pm_4_sensor->publish_state(pm);

    pm = (data[10] * 256) + data[11];
    pm_10_sensor->publish_state(pm);
  }
};