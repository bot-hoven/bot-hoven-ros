// #ifndef HARDWARE__I2C__I2C_BUS_MANAGER_H_
// #define HARDWARE__I2C__I2C_BUS_MANAGER_H_

// #include "hardware/i2c/I2CPeripheral.h"
// #include <memory>
// #include <mutex>
// #include <string>
// #include <map>

// class I2CBusManager {
// public:
//   static std::shared_ptr<hardware::I2CPeripheral> getBus(const std::string &bus_name, const std::string &default_device = "");

// private:
//   static std::mutex mutex_;
//   static std::map<std::string, std::weak_ptr<hardware::I2CPeripheral>> buses_;
// };

// #endif  // HARDWARE__I2C__I2C_BUS_MANAGER_H_


#ifndef HARDWARE__I2C__I2CBUSMANAGER_H_
#define HARDWARE__I2C__I2CBUSMANAGER_H_

#include <memory>
#include <string>
#include <mutex>
#include "hardware/i2c/I2CPeripheral.h"

namespace hardware {

class I2CBusManager {
public:
  static std::shared_ptr<I2CBusManager> getInstance(const std::string& device);

  void WriteRegisterByte(uint8_t address, uint8_t register_address, uint8_t value);
  uint8_t ReadRegisterByte(uint8_t address, uint8_t register_address);

private:
  I2CBusManager(const std::string& device);
  std::unique_ptr<I2CPeripheral> i2c_peripheral_;
  std::mutex i2c_mutex_;
};

}  // namespace hardware

#endif  // HARDWARE__I2C__I2CBUSMANAGER_H_
