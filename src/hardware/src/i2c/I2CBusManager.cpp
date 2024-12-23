// #include "hardware/i2c/I2CBusManager.hpp"
// #include <stdexcept>

// std::mutex I2CBusManager::mutex_;
// std::map<std::string, std::weak_ptr<I2CPeripheral>> I2CBusManager::buses_;

// std::shared_ptr<hardware::I2CPeripheral> I2CBusManager::getBus(const std::string &bus_name, const std::string &default_device) {
//   std::lock_guard<std::mutex> lock(mutex_);

//   auto it = buses_.find(bus_name);
//   if (it != buses_.end()) {
//     // Check if the weak pointer is still valid
//     auto bus = it->second.lock();
//     if (bus) {
//       return bus;
//     }
//   }

//   // If no bus exists, create a new one
//   if (default_device.empty()) {
//     throw std::invalid_argument("Bus not initialized, and no default device provided.");
//   }

//   auto new_bus = std::make_shared<hardware::I2CPeripheral>(default_device);
//   buses_[bus_name] = new_bus;

//   return new_bus;
// }

#include "hardware/i2c/I2CBusManager.hpp"
#include <stdexcept>

namespace hardware {

std::shared_ptr<I2CBusManager> I2CBusManager::getInstance(const std::string& device) {
  static std::shared_ptr<I2CBusManager> instance = std::make_shared<I2CBusManager>(device);
  return instance;
}

I2CBusManager::I2CBusManager(const std::string& device) {
  i2c_peripheral_ = std::make_unique<I2CPeripheral>(device, 0);  // Default address
}

void I2CBusManager::WriteRegisterByte(uint8_t address, uint8_t register_address, uint8_t value) {
  std::lock_guard<std::mutex> lock(i2c_mutex_);
  i2c_peripheral_->ConnectToPeripheral(address);
  i2c_peripheral_->WriteRegisterByte(register_address, value);
}

uint8_t I2CBusManager::ReadRegisterByte(uint8_t address, uint8_t register_address) {
  std::lock_guard<std::mutex> lock(i2c_mutex_);
  i2c_peripheral_->ConnectToPeripheral(address);
  return i2c_peripheral_->ReadRegisterByte(register_address);
}

}  // namespace hardware
