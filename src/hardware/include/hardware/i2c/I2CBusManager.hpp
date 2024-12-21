#ifndef HARDWARE__I2C__I2C_BUS_MANAGER_H_
#define HARDWARE__I2C__I2C_BUS_MANAGER_H_

#include "hardware/i2c/I2CPeripheral.h"
#include <memory>
#include <mutex>
#include <string>

class I2CBusManager {
public:
  static std::shared_ptr<hardware::I2CPeripheral> getBus(const std::string &bus_name, const std::string &default_device = "");

private:
  static std::mutex mutex_;
  static std::map<std::string, std::weak_ptr<hardware::I2CPeripheral>> buses_;
};

#endif  // HARDWARE__I2C__I2C_BUS_MANAGER_H_
