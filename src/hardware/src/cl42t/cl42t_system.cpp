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
        cfg_.spi_mode_ = std::stoi(info.hardware_parameters.at("spi_mode"));
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
        position_state_interface_name_ = joint_state_interfaces_.begin()->first;
        position_command_interface_name_ = joint_command_interfaces_.begin()->first;
    } catch (const std::exception &e) {
        RCLCPP_FATAL(rclcpp::get_logger("Cl42tSystemHardware"), "Failed to parse interface parameters: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_DEBUG(rclcpp::get_logger("Cl42tSystemHardware"), "Min Position: %f", min_position_);
    RCLCPP_DEBUG(rclcpp::get_logger("Cl42tSystemHardware"), "Max Position: %f", max_position_);
    RCLCPP_DEBUG(rclcpp::get_logger("Cl42tSystemHardware"), "position_state_interface_name_: %s", position_state_interface_name_.c_str());
    RCLCPP_DEBUG(rclcpp::get_logger("Cl42tSystemHardware"), "position_command_interface_name_: %s", position_command_interface_name_.c_str());


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
        comm_.setup(spi_peripheral_, cfg_.bits_per_word_, cfg_.bus_speed_hz_, cfg_.spi_mode_);
        comm_.init();
    } catch (const std::exception &e) {
        RCLCPP_FATAL(rclcpp::get_logger("Cl42tSystemHardware"), "Error initializing CL42T Communicator: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("Cl42tSystemHardware"), "Working so far...");
    // Reset the state interface
    set_state(position_state_interface_name_, 0.0);
    previous_position_command_ = 0.0;

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
    RCLCPP_INFO(rclcpp::get_logger("Cl42tSystemHardware"), "Activating ...please wait...");
    // last_command_time_ = std::chrono::steady_clock::now();
    // Initialize the CL42T communication interface.
    try {
        comm_.init();
    } catch (const std::exception &e) {
        RCLCPP_FATAL(rclcpp::get_logger("Cl42tSystemHardware"), "Error initializing CL42T: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // TODO: Get the stepper resolution after calibration setting to 0.01 for now
    cl42t_resolution_ = 0.1;

    RCLCPP_INFO(rclcpp::get_logger("Cl42tSystemHardware"), "Successfully activated!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Cl42tSystemHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
    RCLCPP_INFO(rclcpp::get_logger("Cl42tSystemHardware"), "Successfully deactivated!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Cl42tSystemHardware::read(const rclcpp::Time & /*time*/,
                                                          const rclcpp::Duration & /*period*/) {

    double current_position;
    double previous_position = get_state(position_state_interface_name_);
    double previous_command = get_command(position_command_interface_name_);

    // Get the current position from the CL42T driver
    // First send the read request to the CL42T driver
    if (std::abs(previous_position - previous_command) >= cl42t_resolution_) {
        try {
            current_position = comm_.read_position(cfg_.stepper_side_);
            // RCLCPP_INFO(rclcpp::get_logger("Cl42tSystemHardware"), "Sent read command to: %s", cfg_.stepper_side_.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("Cl42tSystemHardware"), "Failed to read position: %s", e.what());
            return hardware_interface::return_type::ERROR;
        }

        if (current_position != -1) {
            set_state(position_state_interface_name_, current_position);
            RCLCPP_INFO(rclcpp::get_logger("Cl42tSystemHardware"), "Received position %lf from %s", current_position, cfg_.stepper_side_.c_str());
        }
        else {
            RCLCPP_INFO(rclcpp::get_logger("Cl42tSystemHardware"), "Failed to read position from %s", cfg_.stepper_side_.c_str());
        }
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type Cl42tSystemHardware::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {

    desired_position_ = get_command(position_command_interface_name_);

    // RCLCPP_INFO(get_logger(), "command: %f", get_command(position_command_interface_name_));

    // Ensure the desired positon is within range
    if (desired_position_ < min_position_ || desired_position_ > max_position_) {
        RCLCPP_ERROR(get_logger(), "Invalid Command. Ensure commands are in [%lf, %lf].", min_position_,
                     max_position_);
        return hardware_interface::return_type::ERROR;
    }

    // Calculate position change
    // double position_change = desired_position_ - get_state(position_state_interface_name_);
    double position_change = desired_position_ - previous_position_command_;

    // Check if the position change is within the resolution
    // Kept this in here because I dont want it to keep slamming the SPI if its an impossible request
    if (std::abs(position_change) >= cl42t_resolution_) {

        // Send the command over SPI
        if (previous_position_command_ != desired_position_) {
        try {
            comm_.send_position(cfg_.stepper_side_, desired_position_);
            previous_position_command_ = desired_position_;
            RCLCPP_INFO(rclcpp::get_logger("Cl42tSystemHardware"), "Sent position: %f", desired_position_);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("Cl42tSystemHardware"), "Failed to send command: %s", e.what());
            return hardware_interface::return_type::ERROR;
        }
        }
    }

    return hardware_interface::return_type::OK;
}


} // namespace cl42t_hardware_interface

PLUGINLIB_EXPORT_CLASS(cl42t_hardware_interface::Cl42tSystemHardware, hardware_interface::SystemInterface)
