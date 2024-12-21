#include "hardware/pca9685/pca9685_system.hpp"
#include "hardware_common.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"


namespace pca9685_hardware_interface
{
hardware_interface::CallbackReturn Pca9685SystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  try {
    // Dynamically create or fetch the shared I2C bus instance
    auto i2c_bus = I2CBusManager::getBus(
      info.hardware_parameters.at("i2c_bus"),
      info.hardware_parameters.at("default_i2c_device")
    );

    pca = std::make_unique<PCA9685>(i2c_bus, std::stoi(info_.hardware.parameters["i2c_address"]));

  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("Pca9685SystemHardware"), "Error initializing PCA9685: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // Pca9685System has one command interface on each output
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Pca9685SystemHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Pca9685SystemHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"), "PCA9685 initialized successfully.");
  return hardware_interface::CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> Pca9685SystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Pca9685SystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn Pca9685SystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"), "Configuring ...please wait...");
  pca.init();
  pca.set_pwm_freq(pca.frequency);

  RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Pca9685SystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"), "Cleaning up ...please wait...");

  // i2c_bus.disconnect();
  
  RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Pca9685SystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (auto i = 0u; i < hw_commands_.size(); i++)
  {
    if (std::isnan(hw_commands_[i]))
    {
      hw_commands_[i] = 0;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Pca9685SystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Pca9685SystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  return hardware_interface::return_type::OK;
}

double Pca9685SystemHardware::command_to_duty_cycle(double command){

    double min_input = -1.0;
    double max_input = 1.0;

    double clamped_command = std::clamp(command, min_input, max_input);

    // min/max were changed to match datasheet for servos
    double min_duty_cycle = 1.0;
    double max_duty_cycle = 2.0;


    double slope = (max_duty_cycle-min_duty_cycle)/(max_input-min_input);
    double offset = (max_duty_cycle+min_duty_cycle)/2;

    return slope * clamped_command + offset;

}

hardware_interface::return_type Pca9685SystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  for (auto i = 0u; i < hw_commands_.size(); i++)
  {
    double duty_cycle = command_to_duty_cycle(hw_commands_[i]);

    // RCLCPP_INFO(
    //     rclcpp::get_logger("Pca9685SystemHardware"),
    //     "Joint '%d' has command '%f', duty_cycle: '%f'.", i, hw_commands_[i], duty_cycle);

    pca.set_pwm_ms(i, duty_cycle);

  }

  return hardware_interface::return_type::OK;
}

}  // namespace pca9685_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  pca9685_hardware_interface::Pca9685SystemHardware, hardware_interface::SystemInterface)
