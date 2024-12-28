// #ifndef HARDWARE__PCA9685__PCA9685_SYSTEM_HPP_
// #define HARDWARE__PCA9685__PCA9685_SYSTEM_HPP_

// #include <memory>
// #include <string>
// #include <vector>

// #include "hardware_interface/handle.hpp"
// #include "hardware_interface/hardware_info.hpp"
// #include "hardware_interface/system_interface.hpp"
// #include "hardware_interface/types/hardware_interface_return_values.hpp"
// #include "rclcpp/clock.hpp"
// #include "rclcpp/duration.hpp"
// #include "rclcpp/macros.hpp"
// #include "rclcpp/time.hpp"
// #include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
// #include "rclcpp_lifecycle/state.hpp"

// #include "hardware_common.hpp"
// #include "hardware/pca9685/visibility_control.h"
// #include <hardware/pca9685/pca9685_comm.h>


// namespace pca9685_hardware_interface
// {
// class Pca9685SystemHardware : public hardware_interface::SystemInterface
// {
// public:
//   RCLCPP_SHARED_PTR_DEFINITIONS(Pca9685SystemHardware);

//   PCA9685_HARDWARE_INTERFACE_PUBLIC
//   hardware_interface::CallbackReturn on_init(
//     const hardware_interface::HardwareInfo & info) override;

//   PCA9685_HARDWARE_INTERFACE_PUBLIC
//   std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

//   PCA9685_HARDWARE_INTERFACE_PUBLIC
//   std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

//   PCA9685_HARDWARE_INTERFACE_PUBLIC
//   hardware_interface::CallbackReturn on_configure(
//     const rclcpp_lifecycle::State & previous_state) override;

//   PCA9685_HARDWARE_INTERFACE_PUBLIC
//   hardware_interface::CallbackReturn on_cleanup(
//     const rclcpp_lifecycle::State & previous_state) override;

//   PCA9685_HARDWARE_INTERFACE_PUBLIC
//   hardware_interface::CallbackReturn on_activate(
//     const rclcpp_lifecycle::State & previous_state) override;

//   PCA9685_HARDWARE_INTERFACE_PUBLIC
//   hardware_interface::CallbackReturn on_deactivate(
//     const rclcpp_lifecycle::State & previous_state) override;

//   PCA9685_HARDWARE_INTERFACE_PUBLIC
//   hardware_interface::return_type read(
//     const rclcpp::Time & time, const rclcpp::Duration & period) override;

//   PCA9685_HARDWARE_INTERFACE_PUBLIC
//   hardware_interface::return_type write(
//     const rclcpp::Time & time, const rclcpp::Duration & period) override;

// private:
//   std::vector<double> hw_commands_;
//   pca9685_hardware_interface::PCA9685 pca;
//   double command_to_duty_cycle(double command);
// };

// }  // namespace pca9685_hardware_interface

// #endif  // HARDWARE__PCA9685__PCA9685_SYSTEM_HPP_

#ifndef HARDWARE__PCA9685__PCA9685_SYSTEM_HPP_
#define HARDWARE__PCA9685__PCA9685_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "hardware/pca9685/visibility_control.h"
#include <hardware/pca9685/pca9685_comm.h>

namespace pca9685_hardware_interface
{
class Pca9685SystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Pca9685SystemHardware);

  PCA9685_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  PCA9685_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  PCA9685_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  PCA9685_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  PCA9685_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  PCA9685_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  PCA9685_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  PCA9685_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  PCA9685_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::vector<double> hw_commands_;
  std::unique_ptr<pca9685_hardware_interface::PCA9685> pca;
  double command_to_duty_cycle(double command);
};

}  // namespace pca9685_hardware_interface

#endif  // HARDWARE__PCA9685__PCA9685_SYSTEM_HPP_