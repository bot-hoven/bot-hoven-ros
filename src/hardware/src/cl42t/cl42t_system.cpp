#include "hardware/cl42t/cl42t_system.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <sstream>
#include <stdexcept>
#include <string>
#include <chrono>


namespace cl42t_hardware_interface {

hardware_interface::CallbackReturn Cl42tSystemHardware::on_init(const hardware_interface::HardwareInfo &info) {
    // Call base initialization and check for success.
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    try {
        cfg_.spi_device_ = info.hardware_parameters.at("spi_device");
        cfg_.bus_speed_hz_ = std::stoi(info.hardware_parameters.at("bus_speed_hz"));
        cfg_.bits_per_word_ = std::stoi(info.hardware_parameters.at("bits_per_word"));
        cfg_.stepper_side_ = info.hardware_parameters.at("stepper_side");
    } catch (const std::exception &e) {
        RCLCPP_FATAL(rclcpp::get_logger("Cl42tSystemHardware"), "Failed to parse CL42T parameters: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Validate interfaces (simplified example).
    const hardware_interface::ComponentInfo& joint = info.joints[0];
    if (joint.state_interfaces.size() != 1 || joint.command_interfaces.size() != 1) {
        RCLCPP_FATAL(rclcpp::get_logger("Cl42tSystemHardware"), "Invalid interface configuration on joint '%s'.", joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
        RCLCPP_FATAL(rclcpp::get_logger("Cl42tSystemHardware"), "Expected position interfaces on joint '%s'.", joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }
    try {
        min_position_ = std::stod(joint.command_interfaces[0].min);
        max_position_ = std::stod(joint.command_interfaces[0].max);
        position_state_interface_name_ = joint.state_interfaces[0].name;
        position_command_interface_name_ = joint.command_interfaces[0].name;
    } catch (const std::exception &e) {
        RCLCPP_FATAL(rclcpp::get_logger("Cl42tSystemHardware"), "Failed to parse interface parameters: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }
    if (min_position_ > max_position_) {
        RCLCPP_FATAL(rclcpp::get_logger("Cl42tSystemHardware"), "Invalid position bounds specified.");
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("Cl42tSystemHardware"), "Successfully Initialized %s.", info.name.c_str());
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Cl42tSystemHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
    RCLCPP_INFO(rclcpp::get_logger("Cl42tSystemHardware"), "Configuring ...please wait...");
    try {
        // Create the SPI peripheral.
        spi_peripheral_ = new hardware::SPIPeripheral(cfg_.spi_device_);
    } catch (const std::exception &e) {
        RCLCPP_FATAL(rclcpp::get_logger("Cl42tSystemHardware"), "Failed to setup SPI Object: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    try {
        // Initialize the SPI peripheral via the communication interface.
        comm_.setup(spi_peripheral_, cfg_.bits_per_word_, cfg_.bus_speed_hz_);
        comm_.init();
    } catch (const std::exception &e) {
        RCLCPP_FATAL(rclcpp::get_logger("Cl42tSystemHardware"), "Error initializing CL42T Communicator: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Cl42tSystemHardware::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/) {
    RCLCPP_INFO(rclcpp::get_logger("Cl42tSystemHardware"), "Cleaning up ...please wait...");
    if (spi_peripheral_) {
        delete spi_peripheral_;
        spi_peripheral_ = nullptr;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Cl42tSystemHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
    last_command_time_ = std::chrono::steady_clock::now();
    try {
        // Initialize the CL42T communication interface.
        comm_.init();
    } catch (const std::exception &e) {
        RCLCPP_FATAL(rclcpp::get_logger("Cl42tSystemHardware"), "Error initializing CL42T: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("Cl42tSystemHardware"), "Successfully activated!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Cl42tSystemHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
    RCLCPP_INFO(rclcpp::get_logger("Cl42tSystemHardware"), "Successfully deactivated!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Cl42tSystemHardware::read(const rclcpp::Time & /*time*/,
                                                          const rclcpp::Duration & /*period*/) {
    // TODO: Implement reading from hardware if needed.
    command_sent_ = false;
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type Cl42tSystemHardware::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_command_time_);

    // if (elapsed.count() >= 1) {  // Check if at least 1 second has passed
        std::string command = "p l 200";
        try {
            comm_.send_command(command);
            RCLCPP_INFO(rclcpp::get_logger("Cl42tSystemHardware"), "Sent command: %s", command.c_str());
            last_command_time_ = now;  // Update the last command time
        } catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("Cl42tSystemHardware"), "Failed to send command: %s", e.what());
            return hardware_interface::return_type::ERROR;
        }
    // }

    return hardware_interface::return_type::OK;
}


} // namespace cl42t_hardware_interface

PLUGINLIB_EXPORT_CLASS(cl42t_hardware_interface::Cl42tSystemHardware, hardware_interface::SystemInterface)
