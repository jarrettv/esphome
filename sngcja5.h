#include "esphome.h"
#include <Wire.h>
#include "esphome/core/log.h"

static const char *const TAG = "sngcja5.sensor";

class Sngcja5Component : public PollingComponent {
  public:

  Sngcja5Component(uint32_t update_interval) : PollingComponent(update_interval) {}

  Sensor *pm_1_sensor = new Sensor();
  Sensor *pm_2_5_sensor = new Sensor();
  Sensor *pm_10_sensor = new Sensor();

  float get_setup_priority() const override { return esphome::setup_priority::DATA; }

  void setup() override {
    ESP_LOGCONFIG(TAG, "Setting up SN-GCJA5...");
    Wire.begin();
  }

  void dump_config() override {
    ESP_LOGCONFIG(TAG, "SN-GCJA5:");

    LOG_SENSOR("  ", "PM1.0", pm_1_sensor);
    LOG_SENSOR("  ", "PM2.5", pm_2_5_sensor);
    LOG_SENSOR("  ", "PM10.0", pm_10_sensor);

    // ESP_LOGCONFIG(TAG, "  Sensor Status: %d", getStatusSensors());
    // ESP_LOGCONFIG(TAG, "  PD Status: %d", getStatusPD());
    // ESP_LOGCONFIG(TAG, "  LD Status: %d", getStatusLD());
    // ESP_LOGCONFIG(TAG, "  Fan Status: %d", getStatusFan());
  }

  void update() override {

    if (!readMeasurement()) {
      ESP_LOGE(TAG, "Error reading measurement");
      return;
    }

    delay(8);
    float pm1 = getPM(SNGCJA5_PM1);
    pm_1_sensor->publish_state(pm1);

    float pm2_5 = getPM(SNGCJA5_PM2_5);
    pm_2_5_sensor->publish_state(pm2_5);

    float pm10 = getPM(SNGCJA5_PM10);
    pm_10_sensor->publish_state(pm10);
    
    ESP_LOGD(TAG, "  Sensor status %d", getStatusSensors());
    ESP_LOGD(TAG, "  PD status %d", getStatusPD());
    ESP_LOGD(TAG, "  LD operational status %d", getStatusLD());
    ESP_LOGD(TAG, "  Fan operational status %d", getStatusFan());
  }

  private:
    
    uint8_t _deviceAddress = 0x33; //Default, unchangable address
    uint8_t read_buf[100];         // buffer to store data read
    uint8_t offset;                // offset in read_buf

    enum SNGCJA5_REGISTERS {
      SNGCJA5_PM1 = 0x00,
      SNGCJA5_PM2_5 = 0x04,
      SNGCJA5_PM10 = 0x08,
      SNGCJA5_PCOUNT_0_5 = 0x0C,
      SNGCJA5_PCOUNT_1 = 0x0E,
      SNGCJA5_PCOUNT_2_5 = 0x10,
      SNGCJA5_PCOUNT_5 = 0x14,
      SNGCJA5_PCOUNT_7_5 = 0x16,
      SNGCJA5_PCOUNT_10 = 0x18,
      SNGCJA5_STATE = 0x26,
    };

    float getPM(uint8_t pmRegister) {
      //Given a mass density PM register, do conversion and return mass density
      uint32_t count = readRegister32(pmRegister);
      return (count / 1000.0);
    }

    uint8_t getState() {
      return (readRegister8(SNGCJA5_STATE));
    }

    uint8_t getStatusSensors() {
      return ((getState() >> 6) & 0b11);
    }

    uint8_t getStatusPD() {
      return ((getState() >> 4) & 0b11);
    }

    uint8_t getStatusLD() {
      return ((getState() >> 2) & 0b11);
    }

    uint8_t getStatusFan() {
      return ((getState() >> 0) & 0b11);
    }

    bool readMeasurement() {
      // reset buffer
      memset (read_buf, 0x0, sizeof(read_buf));
      offset = 0;

      Wire.beginTransmission(_deviceAddress);
      Wire.write(SNGCJA5_PM1); // this is beginning address

      if (Wire.endTransmission(false) != 0)
        return (false); //Sensor did not ACK

      Wire.requestFrom(_deviceAddress, (uint8_t)40);
      while (Wire.available())
        read_buf[offset++] = Wire.read();

      return(true);
    }
    
    uint8_t readRegister8(uint8_t addr) {
      return(read_buf[addr]);
    }
    
    uint16_t readRegister16(uint8_t addr) {
      return ((uint16_t)read_buf[addr+1] << 8 | read_buf[addr]);
    }
    
    uint32_t readRegister32(uint8_t addr) {
      return (((uint32_t)read_buf[addr+3] << 24) | ((uint32_t)read_buf[addr+2] << 16) | \
        ((uint32_t)read_buf[addr+1] << 8) | ((uint32_t)read_buf[addr] << 0));
    }
};