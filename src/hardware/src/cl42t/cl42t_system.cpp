#include "hardware/cl42t/cl42t_system.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

// For demonstration, include your SPI peripheral header
#include "hardware/spi/SPIPeripheral.h"
#include <memory>

namespace cl42t_hardware_interface {
    hardware_interface::CallbackReturn Cl42tSystemHardware::on_init(const hardware_interface::HardwareInfo &info) {
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Try to parse the CL42T parameters
        try {
            cfg_.spi_device_ = info_.hardware_parameters.at("spi_device");
            cfg_.chip_select_ = std::stoi(info_.hardware_parameters.at("chip_select]"));
            cfg_.bus_speed_hz_ = std::stoi(info_.hardware_parameters.at("bus_speed_hz"));
            cfg_.bits_per_word_ = std::stoi(info_.hardware_parameters.at("bits_per_word"));
            cfg_.stepper_side_ = info_.hardware_parameters.at("stepper_side");
        } catch (const std::exception &e) {
            RCLCPP_FATAL(rclcpp::get_logger("Cl42tSystemHardware"), "Failed to parse CL42T parameters: %s",
                         e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Parse the joint associated with the interface
        const hardware_interface::ComponentInfo& joint = info.joints[0];

        // Validate the state interface
        if (joint.state_interfaces.size() != 1) {
            RCLCPP_FATAL(rclcpp::get_logger("Cl42tSystemHardware"), "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
                         joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(rclcpp::get_logger("Cl42tSystemHardware"), "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
                         joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Validate the command interface
        if (joint.command_interfaces.size() != 1) {
            RCLCPP_FATAL(rclcpp::get_logger("Cl42tSystemHardware"), "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                         joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(rclcpp::get_logger("Cl42tSystemHardware"), "Joint '%s' have %s command interfaces found. '%s' expected.",
                         joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
                         hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Try parse the interface parameters
        try {
            min_position_ = std::stod(joint.command_interfaces[0].min);
            max_position_ = std::stod(joint.command_interfaces[0].max);
            position_state_interface_name_ = joint_state_interfaces_.begin()->first;
            position_command_interface_name_ = joint_command_interfaces_.begin()->first;
        } catch (const std::exception& e) {
            RCLCPP_FATAL(rclcpp::get_logger("Cl42tSystemHardware"), "Failed to parse interface parameters: %s", e.what());
            return CallbackReturn::ERROR;
        }

        // Validate position bounds
        if (min_position_ > max_position_) {
            RCLCPP_FATAL(rclcpp::get_logger("Cl42tSystemHardware"), "Invalid Position bounds specified.");
            return CallbackReturn::ERROR;
        }

        // Intialize internal class variables
        dir_ = RotateDir::CW;
        angular_resolution_ = 2 * M_PI / pulses_per_rev_;  // angle in radians per pulse;
        num_pulses_ = 0;

        RCLCPP_INFO(rclcpp::get_logger("Cl42tSystemHardware"), "Successfully Initialized %s.", info.name.c_str());
        return hardware_interface::CallbackReturn::SUCCESS;

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn Cl42tSystemHardware::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/) {
        RCLCPP_INFO(rclcpp::get_logger("Cl42tSystemHardware"), "Configuring ...please wait...");

        try {
            // Create the SPI peripheral using the configured device and parameters.
            spi_peripheral_ = new hardware::SPIPeripheral(cfg_.spi_device_);
            
        } catch (const std::exception &e) {
            RCLCPP_FATAL(rclcpp::get_logger("Cl42tSystemHardware"), "Failed to setup SPI Object: %s", e.what());
            return CallbackReturn::ERROR;
        }

        try {
            comm_.setup(spi_peripheral_, cfg_.chip_select_, cfg_.bits_per_word_, cfg_.bus_speed_hz_);
        } catch (const std::exception &e) {
            RCLCPP_FATAL(rclcpp::get_logger("Cl42tSystemHardware"), "Error initializing CL42T Communicator: %s", e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }
        return CallbackReturn::SUCCESS;

        RCLCPP_INFO(rclcpp::get_logger("Cl42tSystemHardware"), "Successfully configured!");

    }

    hardware_interface::CallbackReturn Cl42tSystemHardware::on_cleanup(
        const rclcpp_lifecycle::State & /*previous_state*/) {
        RCLCPP_INFO(rclcpp::get_logger("Cl42tSystemHardware"), "Cleaning up ...please wait...");

        // comm_.close();

        if (spi_peripheral_) {
            delete spi_peripheral_;
            spi_peripheral_ = nullptr;
        }
        return hardware_interface::CallbackReturn::SUCCESS;

        RCLCPP_INFO(rclcpp::get_logger("Cl42tSystemHardware"), "Successfully cleaned up!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn Cl42tSystemHardware::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/) {

        // Initialize the CL42T Comm object
        try {
            comm_.init();
        } catch (const std::exception &e) {
            RCLCPP_FATAL(rclcpp::get_logger("Cl42tSystemHardware"), "Error initializing CL42T: %s", e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger("Cl42tSystemHardware"), "Successfully activated!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn Cl42tSystemHardware::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/) {
        RCLCPP_INFO(rclcpp::get_logger("Cl42tSystemHardware"), "Successfully deactivated!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type Cl42tSystemHardware::read(const rclcpp::Time & /*time*/,
                                         const rclcpp::Duration & /*period*/) {
        // TODO: Implement the read function for SPI communication
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type Cl42tSystemHardware::write(const rclcpp::Time & /*time*/,
                                          const rclcpp::Duration & /*period*/) {

        // For each write cycle, send the static command \"p l 200\" via SPI.\n
        //TODO: Change this to be a real command! This static value is just for testing purposes.
        std::string command = "p l 200";
        try {
            comm_.send_command(command);
            RCLCPP_INFO(rclcpp::get_logger("CL42TSystem"), "Sent command: %s", command.c_str());
        } catch (const std::exception &e) {
            RCLCPP_ERROR(rclcpp::get_logger("CL42TSystem"), "Failed to send command: %s", e.what());
            return hardware_interface::return_type::ERROR;
        }
        return hardware_interface::return_type::OK;
    }

} // namespace cl42t_hardware_interface
 
PLUGINLIB_EXPORT_CLASS(cl42t_hardware_interface::Cl42tSystemHardware, hardware_interface::SystemInterface)
