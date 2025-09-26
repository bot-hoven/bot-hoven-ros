#include <chrono>
#include <cmath>
#include <sstream>

#include "bothoven_hardware/cl42t/cl42t_system.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace cl42t_hardware_interface {

hardware_interface::CallbackReturn
Cl42tSystemHardware::on_init(const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  try {
    cfg_.spi_device_ = info_.hardware_parameters.at("spi_device");
    cfg_.bus_speed_hz_ =
        std::stoi(info_.hardware_parameters.at("bus_speed_hz"));
    cfg_.spi_mode_ = std::stoi(info_.hardware_parameters.at("spi_mode"));
    cfg_.bits_per_word_ =
        std::stoi(info_.hardware_parameters.at("bits_per_word"));
    cfg_.stepper_side_ = info_.hardware_parameters.at("stepper_side").at(0);
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("Cl42tSystemHardware"),
                 "Failed to parse CL42T parameters: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // validate interfaces
  const hardware_interface::ComponentInfo &joint = info.joints[0];
  if (joint.state_interfaces.size() != 1 ||
      joint.command_interfaces.size() != 1) {
    RCLCPP_FATAL(rclcpp::get_logger("Cl42tSystemHardware"),
                 "Invalid interface configuration on joint '%s'.",
                 joint.name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
    RCLCPP_FATAL(rclcpp::get_logger("Cl42tSystemHardware"),
                 "Incorrect interface type on joint '%s'.",
                 joint.name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  try {
    min_position_ = std::stod(joint.state_interfaces[0].min);
    max_position_ = std::stod(joint.state_interfaces[0].max);
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("Cl42tSystemHardware"),
                 "Failed to parse interface parameters: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("Cl42tSystemHardware"), "Min Position: %f",
              min_position_);
  RCLCPP_INFO(rclcpp::get_logger("Cl42tSystemHardware"), "Max Position: %f",
              max_position_);

  hw_state_ = std::numeric_limits<double>::quiet_NaN();
  hw_command_ = std::numeric_limits<double>::quiet_NaN();

  if (min_position_ > max_position_) {
    RCLCPP_FATAL(rclcpp::get_logger("Cl42tSystemHardware"),
                 "Invalid position bounds specified.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("Cl42tSystemHardware"),
              "Successfully Initialized %s.", info_.name.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Cl42tSystemHardware::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("Cl42tSystemHardware"),
              "Configuring SPI Interface...please wait...");

  // initialize command and state interfaces to zero positions
  hw_state_ = 0;
  hw_command_ = 0;

  try {
    comm_ = std::make_unique<hardware::Cl42tComm>(cfg_.spi_device_);
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("Cl42tSystemHardware"),
                 "Failed to setup SPI Object: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
Cl42tSystemHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[0].name, hardware_interface::HW_IF_POSITION, &hw_state_));
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
Cl42tSystemHardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &hw_command_));
  return command_interfaces;
}

hardware_interface::CallbackReturn Cl42tSystemHardware::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("Cl42tSystemHardware"),
              "Cleaning up ...please wait...");
  comm_->shutdown();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Cl42tSystemHardware::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("Cl42tSystemHardware"),
              "Activating ...please wait...");
  try {
    comm_->init_peripheral(cfg_.bits_per_word_, cfg_.bus_speed_hz_,
                          cfg_.spi_mode_);
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("Cl42tSystemHardware"),
                 "Error initializing SPI: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("Cl42tSystemHardware"),
              "Successfully activated %s.", info_.name.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Cl42tSystemHardware::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("Cl42tSystemHardware"),
              "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
Cl42tSystemHardware::read(const rclcpp::Time & /*time*/,
                          const rclcpp::Duration & /*period*/) {
  double current_position = comm_->get_position(cfg_.stepper_side_);
  if (std::isnan(current_position)) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("Cl42tSystemHardware"),
                       info_.name.c_str()
                           << ": Received nan position from pico.");
  }
  hw_state_ = current_position;
  // RCLCPP_INFO_STREAM(rclcpp::get_logger("Cl42tSystemHardware"),
  //                    info_.name.c_str()
  //                        << ": Received position: " << current_position);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
Cl42tSystemHardware::write(const rclcpp::Time & /*time*/,
                           const rclcpp::Duration & /*period*/) {
  // RCLCPP_INFO_STREAM(rclcpp::get_logger("Cl42tSystemHardware"),
  //                    "Commanded velocity: " << hw_command_);
  bool success =
      comm_->set_velocity(cfg_.stepper_side_, static_cast<float>(hw_command_));
  if (!success) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("Cl42tSystemHardware"),
                       info_.name.c_str()
                           << ": Failed to send target velocity to pico.");
  }
  return hardware_interface::return_type::OK;
}

} // namespace cl42t_hardware_interface

PLUGINLIB_EXPORT_CLASS(cl42t_hardware_interface::Cl42tSystemHardware,
                       hardware_interface::SystemInterface)
