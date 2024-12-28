#include "hardware/bothoven_hardware.hpp"


namespace hardware {

hardware_interface::CallbackReturn BothovenHardware::on_init(const hardware_interface::HardwareInfo &info) {
  
    auto i2c_bus = std::make_shared<hardware::I2CPeripheral>(
      info_.hardware_parameters["i2c_bus"],
      0 // default address of 0 for initialization
    );

    // auto i2c_bus = std::make_unique<hardware::I2CPeripheral>();

    if (!i2c_bus) {
      RCLCPP_ERROR(rclcpp::get_logger("I2CPeripheral"), "Failed to initialize I2C bus.");
      return hardware_interface::CallbackReturn::ERROR;
    }

  try {
    left_hand_pca = std::make_unique<pca9685_hardware_interface::PCA9685>(i2c_bus, std::stoi(info_.hardware_parameters["left_hand_pca_i2c_address"]));
    right_hand_pca = std::make_unique<pca9685_hardware_interface::PCA9685>(i2c_bus, std::stoi(info_.hardware_parameters["right_hand_pca_i2c_address"]));


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

  // left_hand_pca->set_pwm_freq(td::stoi(info_.hardware_parameters["pca9685_frequency_hz"]));

  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // for (const hardware_interface::ComponentInfo & joint : info_.joints)
  // {
  //   // Pca9685System has one command interface on each output
  //   if (joint.command_interfaces.size() != 1)
  //   {
  //     RCLCPP_FATAL(
  //       rclcpp::get_logger("Pca9685SystemHardware"),
  //       "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
  //       joint.command_interfaces.size());
  //     return hardware_interface::CallbackReturn::ERROR;
  //   }

  //   if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
  //   {
  //     RCLCPP_FATAL(
  //       rclcpp::get_logger("Pca9685SystemHardware"),
  //       "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
  //       joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
  //     return hardware_interface::CallbackReturn::ERROR;
  //   }
  // }

  return hardware_interface::CallbackReturn::SUCCESS;



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

hardware_interface::CallbackReturn BothovenHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("BothovenHardware"), "Configuring ...please wait...");
  left_hand_pca.init_i2c();
  right_hand_pca.init_i2c();
  RCLCPP_INFO(rclcpp::get_logger("BothovenHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BothovenHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("BothovenHardware"), "Cleaning up ...please wait...");

  // i2c_bus.disconnect();
  
  RCLCPP_INFO(rclcpp::get_logger("BothovenHardware"), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BothovenHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("BothovenHardware"), "Activating ...please wait...");
  left_hand_pca.set_pwm_freq(std::stoi(info_.hardware_parameters["pca9685_frequency_hz"]));
  right_hand_pca.set_pwm_freq(std::stoi(info_.hardware_parameters["pca9685_frequency_hz"]));
  RCLCPP_INFO(rclcpp::get_logger("BothovenHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BothovenHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("BothovenHardware"), "Deactivating ...please wait...");

  // method here to reset all servos and solenoids to their "starting/default" positions;
  
  RCLCPP_INFO(rclcpp::get_logger("BothovenHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type BothovenHardware::read(const rclcpp::Time &time, const rclcpp::Duration &period) {
  (void)time; // suppress unused parameter warning
  (void)period; // suppress unused parameter warning
  // Read joint states from hardware
  return hardware_interface::return_type::OK;
}

double BothovenHardware::command_to_duty_cycle(double command){

    double min_input = -1.0;
    double max_input = 1.0;

    double clamped_command = std::clamp(command, min_input, max_input);

    double min_duty_cycle = 1.0;
    double max_duty_cycle = 2.0;


    double slope = (max_duty_cycle-min_duty_cycle)/(max_input-min_input);
    double offset = (max_duty_cycle+min_duty_cycle)/2;

    return slope * clamped_command + offset;

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