#ifndef HARDWARE_COMMON_HPP_
#define HARDWARE_COMMON_HPP_

#include <string>
#include <vector>
#include <memory>

// Shared headers
#include "hardware/i2c/I2CPeripheral.h"
#include "hardware/i2c/I2CBusManager.hpp"
#include "hardware/common/Constants.h"
#include "rclcpp/rclcpp.hpp"

namespace hardware {

// enum class DeviceStatus {
//     OK,
//     ERROR,
//     BUSY,
//     NOT_CONNECTED
// };

struct I2CConfig {
    std::string device_path;
    uint8_t address;
};

// inline void log_event(const std::string &component, const std::string &message) {
//     // In case we want a common log event, we might want to pass the type 
//     // as an arg (INFO, FATAL, etc.) and use a switch-case.
//     RCLCPP_INFO(rclcpp::get_logger(component), message);
// }

} // namespace hardware

#endif // HARDWARE_COMMON_HPP_
