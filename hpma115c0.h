#include "esphome.h"

class Hpma115C0Component : public PollingComponent, public UARTDevice {
 public:
  Hpma115C0Component(uint32_t update_interval, UARTComponent *parent) : PollingComponent(update_interval), UARTDevice(parent) {}

  float get_setup_priority() const override { return esphome::setup_priority::BUS; }

  Sensor *pm_1_sensor = new Sensor();
  Sensor *pm_2_5_sensor = new Sensor();
  Sensor *pm_4_sensor = new Sensor();
  Sensor *pm_10_sensor = new Sensor();

  static const uint16_t STARTUP_DELAY_MS = 20000;

  bool delaying_startup = true;
  
  static const char MSG_CHAR_1 = 0x42;
  static const char MSG_CHAR_2 = 0x4d;
  bool packet_found = false;
  std::array<u_char,32> last_good_packet = {};
  
  void setup() override {
    this->set_timeout("startup_delay", STARTUP_DELAY_MS, [this]() {
      ESP_LOGI("hpma115c0", "Startup delay completed");
      this->delaying_startup = false;
    });
  }

  void update() override {

    if (delaying_startup) {
      ESP_LOGI("hpma115c0", "Startup delay in progress");
      return;
    }

    read_packets();

    publish_state(last_good_packet);
  }

  void read_packets() {
    int count = 0;
    std::array<u_char,32> data = {};

    // only start counting up when the first 2 characters are found
    while (available()) {
      data[count] = read();
      
      if ((data[0] == MSG_CHAR_1 && count == 0) || data[1] == MSG_CHAR_2) count++;
      else read_packets();

      // full packet
      if (count == 32) {
        handle_packet(data);
        read_packets();
      }
    }
  }

  void handle_packet(std::array<u_char,32> data) {

    ESP_LOGD("hpma115c0", "handle_packet");

    uint8_t sum = 0;
    for (int i = 0; i < 30; i++) sum += data[i];
    uint8_t checksum = (data[30] << 8) | data[31];

    if (sum != checksum) {
      ESP_LOGW("hpma115c0", "Checksum invalid: %i <> %i", sum, checksum);
      return;
    }

    last_good_packet = data;
    packet_found = true;
  }

  void publish_state(const std::array<u_char,32> &data) {

    if (!packet_found) return;

    // read data
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