#ifndef BOTHOVEN_HARDWARE_HPP
#define BOTHOVEN_HARDWARE_HPP

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "pca9685/pca9685_comm.h"

namespace hardware {

class BothovenHardware : public hardware_interface::SystemInterface {
public:
  using return_type = hardware_interface::return_type;

  RCLCPP_SHARED_PTR_DEFINITIONS(BothovenHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo &info) override;
  
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  
  hardware_interface::return_type read(
    const rclcpp::Time &time, const rclcpp::Duration &period) override;

  hardware_interface::return_type write(
    const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  void init();

  // PCA9685 methods and variables
  std::unique_ptr<pca9685_hardware_interface::PCA9685> left_hand_pca;
  std::unique_ptr<pca9685_hardware_interface::PCA9685> right_hand_pca;
  double command_to_duty_cycle(double command);
  
  // Joint states and commands
  std::vector<double> joint_positions_;
  std::vector<double> joint_commands_;
};

} // namespace bothoven_hardware

#endif // BOTHOVEN_HARDWARE_HPP
