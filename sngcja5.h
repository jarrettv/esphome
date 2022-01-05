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

    ESP_LOGCONFIG(TAG, "  Sensor Status: %d", getStatusSensors());
    ESP_LOGCONFIG(TAG, "  PD Status: %d", getStatusPD());
    ESP_LOGCONFIG(TAG, "  LD Status: %d", getStatusLD());
    ESP_LOGCONFIG(TAG, "  Fan Status: %d", getStatusFan());
  }

  void update() override {
    float pm1_0 = getPM1_0();
    pm_1_sensor->publish_state(pm1_0);

    float pm2_5 = getPM2_5();
    pm_2_5_sensor->publish_state(pm2_5);

    float pm10 = getPM10();
    pm_10_sensor->publish_state(pm10);
  }

  private:
    enum SNGCJA5_REGISTERS {
      SNGCJA5_PM1_0 = 0x00,
      SNGCJA5_PM2_5 = 0x04,
      SNGCJA5_PM10 = 0x08,
      SNGCJA5_PCOUNT_0_5 = 0x0C,
      SNGCJA5_PCOUNT_1_0 = 0x0E,
      SNGCJA5_PCOUNT_2_5 = 0x10,
      SNGCJA5_PCOUNT_5_0 = 0x14,
      SNGCJA5_PCOUNT_7_5 = 0x16,
      SNGCJA5_PCOUNT_10 = 0x18,
      SNGCJA5_STATE = 0x26,
    };
    
    uint8_t _deviceAddress = 0x33; //Default, unchangable address

    //Given a mass density PM register, do conversion and return mass density
    float getPM(uint8_t pmRegister)
    {
      uint32_t count = readRegister32(pmRegister);
      return (count / 1000.0);
    }
    float getPM1_0()
    {
      return (getPM(SNGCJA5_PM1_0));
    }
    float getPM2_5()
    {
      return (getPM(SNGCJA5_PM2_5));
    }
    float getPM10()
    {
      return (getPM(SNGCJA5_PM10));
    }

    //Particle count functions
    uint16_t getPC0_5()
    {
      return (readRegister16(SNGCJA5_PCOUNT_0_5));
    }
    uint16_t getPC1_0()
    {
      return (readRegister16(SNGCJA5_PCOUNT_1_0));
    }
    uint16_t getPC2_5()
    {
      return (readRegister16(SNGCJA5_PCOUNT_2_5));
    }
    uint16_t getPC5_0()
    {
      return (readRegister16(SNGCJA5_PCOUNT_5_0));
    }
    uint16_t getPC7_5()
    {
      return (readRegister16(SNGCJA5_PCOUNT_7_5));
    }
    uint16_t getPC10()
    {
      return (readRegister16(SNGCJA5_PCOUNT_10));
    }

    //State functions
    uint8_t getState()
    {
      return (readRegister8(SNGCJA5_STATE));
    }
    uint8_t getStatusSensors()
    {
      return ((getState() >> 6) & 0b11);
    }
    uint8_t getStatusPD()
    {
      return ((getState() >> 4) & 0b11);
    }
    uint8_t getStatusLD()
    {
      return ((getState() >> 2) & 0b11);
    }
    uint8_t getStatusFan()
    {
      return ((getState() >> 0) & 0b11);
    }

    //Low level I2C functions
    //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=


    //Reads from a given location
    //Stores the result at the provided outputPointer
    uint8_t readRegister8(uint8_t addr)
    {
      Wire.beginTransmission(_deviceAddress);
      Wire.write(addr);
      if (Wire.endTransmission(false) != 0)
        return (0); //Sensor did not ACK

      Wire.requestFrom((uint8_t)_deviceAddress, (uint8_t)1);
      if (Wire.available())
        return (Wire.read());

      return (0); //Sensor did not respond
    }

    //Reads two consecutive bytes from a given location
    uint16_t readRegister16(uint8_t addr)
    {
      Wire.beginTransmission(_deviceAddress);
      Wire.write(addr);
      if (Wire.endTransmission(false) != 0)
        return (0); //Sensor did not ACK

      Wire.requestFrom((uint8_t)_deviceAddress, (uint8_t)2);
      if (Wire.available())
      {
        uint8_t lsb = Wire.read();
        uint8_t msb = Wire.read();
        return ((uint16_t)msb << 8 | lsb);
      }
      return (0); //Sensor did not respond
    }

    //Reads four consecutive bytes from a given location
    uint32_t readRegister32(uint8_t addr)
    {
      Wire.beginTransmission(_deviceAddress);
      Wire.write(addr);
      if (Wire.endTransmission(false) != 0)
        return (0); //Sensor did not ACK

      Wire.requestFrom((uint8_t)_deviceAddress, (uint8_t)4);
      if (Wire.available())
      {
        uint8_t ll = Wire.read();
        uint8_t lh = Wire.read();
        uint8_t hl = Wire.read();
        uint8_t hh = Wire.read();

        auto result = (((uint32_t)hh << 24) | ((uint32_t)hl << 16) | ((uint32_t)lh << 8) | ((uint32_t)ll << 0));

        ESP_LOGD(TAG, "Address: %d ll=%d lh=%d hl=%d hh=%d result=%d", addr, ll, lh, hl, hh, result);
        return result;
      }
      return (0); //Sensor did not respond
    }
};