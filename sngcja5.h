#include "esphome.h"
#include <Wire.h>
#include "esphome/core/log.h"

static const char *const TAG = "sngcja5.sensor";
static const uint8_t I2C_ADDR = 0x33; // fixed device address

class Sngcja5Component : public PollingComponent {
  public:

  Sngcja5Component(uint32_t update_interval) : PollingComponent(update_interval) {}

  Sensor *pm1_sensor = new Sensor();
  Sensor *pm2_5_sensor = new Sensor();
  Sensor *pm10_sensor = new Sensor();

  float get_setup_priority() const override { return esphome::setup_priority::DATA; }

  void setup() override {
    ESP_LOGCONFIG(TAG, "Setting up SN-GCJA5...");
    Wire.begin();
  }

  void dump_config() override {
    ESP_LOGCONFIG(TAG, "SN-GCJA5:");
    LOG_SENSOR("  ", "PM1.0", pm1_sensor);
    LOG_SENSOR("  ", "PM2.5", pm2_5_sensor);
    LOG_SENSOR("  ", "PM10.0", pm10_sensor);
  }

  void update() override {
    if (!read_measurement()) {
      ESP_LOGE(TAG, "Error reading i2c data");
      return;
    }

    delay(8);
    auto pm1 = get_pm(PM1);
    pm1_sensor->publish_state(pm1);

    auto pm2_5 = get_pm(PM2_5);
    pm2_5_sensor->publish_state(pm2_5);

    auto pm10 = get_pm(PM10);
    pm10_sensor->publish_state(pm10);
    
    ESP_LOGD(TAG, "  Particle counts: 0.5=%d 1.0=%d 2.5=%d 5.0=%d 7.5=%d 10=%d", 
      read_register16(PCOUNT_0_5),
      read_register16(PCOUNT_1),
      read_register16(PCOUNT_2_5),
      read_register16(PCOUNT_5),
      read_register16(PCOUNT_7_5),
      read_register16(PCOUNT_10));
    
    auto state = read_register8(STATE);
    ESP_LOGD(TAG, "  Sensor status %d", (state >> 6) & 0b11);
    ESP_LOGD(TAG, "  PD status %d", (state >> 4) & 0b11);
    ESP_LOGD(TAG, "  LD operational status %d", (state >> 2) & 0b11);
    ESP_LOGD(TAG, "  Fan operational status %d", state & 0b11);
  }

  private:

    enum REGISTERS {
      PM1 = 0x00,
      PM2_5 = 0x04,
      PM10 = 0x08,
      PCOUNT_0_5 = 0x0C,
      PCOUNT_1 = 0x0E,
      PCOUNT_2_5 = 0x10,
      PCOUNT_5 = 0x14,
      PCOUNT_7_5 = 0x16,
      PCOUNT_10 = 0x18,
      STATE = 0x26,
    };

    uint8_t buffer_[100]; // for data read from device

    float get_pm(uint8_t pmRegister) {
      uint32_t count = read_register32(pmRegister);
      return (count / 1000.0);
    }

    bool read_measurement() {      
      memset(buffer_, 0x0, sizeof(buffer_)); // reset
      uint8_t offset = 0;

      Wire.beginTransmission(I2C_ADDR);
      Wire.write(PM1); // this is beginning address

      if (Wire.endTransmission(false) != 0)
        return (false); //Sensor did not ACK

      // read all data available from device into buffer
      Wire.requestFrom(I2C_ADDR, (uint8_t)40);
      while (Wire.available())
        buffer_[offset++] = Wire.read();

      return (true);
    }
    
    uint8_t read_register8(uint8_t addr) {
      return(buffer_[addr]);
    }
    
    uint16_t read_register16(uint8_t addr) {
      return ((uint16_t)buffer_[addr+1] << 8 | buffer_[addr]);
    }
    
    uint32_t read_register32(uint8_t addr) {
      return (((uint32_t)buffer_[addr+3] << 24) | ((uint32_t)buffer_[addr+2] << 16) | \
        ((uint32_t)buffer_[addr+1] << 8) | ((uint32_t)buffer_[addr] << 0));
    }
};