#include "hardware/bothoven_hardware.hpp"

namespace hardware {

BothovenHardware::CallbackReturn BothovenHardware::on_init(const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  joint_positions_.resize(info.joints.size(), 0.0);
  joint_commands_.resize(info.joints.size(), 0.0);
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> BothovenHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < joint_positions_.size(); ++i) {
    state_interfaces.emplace_back(info_.joints[i].name, "position", &joint_positions_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> BothovenHardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < joint_commands_.size(); ++i) {
    command_interfaces.emplace_back(info_.joints[i].name, "position", &joint_commands_[i]);
  }
  return command_interfaces;
}

hardware_interface::return_type BothovenHardware::read(const rclcpp::Time &time, const rclcpp::Duration &period) {
  (void)time; // suppress unused parameter warning
  (void)period; // suppress unused parameter warning
  // Read joint states from hardware
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type BothovenHardware::write(const rclcpp::Time &time, const rclcpp::Duration &period) {
  (void)time; // suppress unused parameter warning
  (void)period; // suppress unused parameter warning
  // Send commands to hardware
  return hardware_interface::return_type::OK;
}

void BothovenHardware::init(){}

} // namespace hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(hardware::BothovenHardware, hardware_interface::SystemInterface)