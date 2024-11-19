#ifndef BOTHOVEN_HARDWARE_HPP
#define BOTHOVEN_HARDWARE_HPP

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

namespace bothoven_hardware {

class BothovenHardware : public hardware_interface::SystemInterface {
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
  hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  // Joint states and commands
  std::vector<double> joint_positions_;
  std::vector<double> joint_commands_;
};

} // namespace bothoven_hardware

#endif // BOTHOVEN_HARDWARE_HPP
