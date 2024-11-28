#ifndef BOTHOVEN_HARDWARE_HPP
#define BOTHOVEN_HARDWARE_HPP

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

namespace hardware {

class BothovenHardware : public hardware_interface::SystemInterface {
public:
  using return_type = hardware_interface::return_type;

  RCLCPP_SHARED_PTR_DEFINITIONS(SystemInterface)

  CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
  
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  
  hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
  hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  void init();
  
  // Joint states and commands
  std::vector<double> joint_positions_;
  std::vector<double> joint_commands_;
};

} // namespace bothoven_hardware

#endif // BOTHOVEN_HARDWARE_HPP
