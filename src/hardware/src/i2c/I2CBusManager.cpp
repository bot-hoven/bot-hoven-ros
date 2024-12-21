#include "hardware/i2c/I2CBusManager.hpp"
#include <stdexcept>

std::mutex I2CBusManager::mutex_;
std::map<std::string, std::weak_ptr<I2CPeripheral>> I2CBusManager::buses_;

std::shared_ptr<hardware::I2CPeripheral> I2CBusManager::getBus(const std::string &bus_name, const std::string &default_device) {
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = buses_.find(bus_name);
  if (it != buses_.end()) {
    // Check if the weak pointer is still valid
    auto bus = it->second.lock();
    if (bus) {
      return bus;
    }
  }

  // If no bus exists, create a new one
  if (default_device.empty()) {
    throw std::invalid_argument("Bus not initialized, and no default device provided.");
  }

  auto new_bus = std::make_shared<hardware::I2CPeripheral>(default_device);
  buses_[bus_name] = new_bus;

  return new_bus;
}

