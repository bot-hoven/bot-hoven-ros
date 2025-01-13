#include "hardware/mcp23017/mcp23017_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mcp23017_hardware_interface {
    hardware_interface::CallbackReturn Mcp23017SystemHardware::on_init(const hardware_interface::HardwareInfo &info) {
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Parse the MCP23017 parameters
        cfg_.bus_name = info_.hardware_parameters.at("i2c_device");
        cfg_.i2c_address = std::stoi(info_.hardware_parameters.at("i2c_address"));

        hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

        // Validate the command interface
        for (const hardware_interface::ComponentInfo &joint : info_.joints) {
            // MCP23017System has one command interface on each output
            if (joint.command_interfaces.size() != 1) {
                RCLCPP_FATAL(rclcpp::get_logger("Mcp23017SystemHardware"),
                             "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                             joint.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
                RCLCPP_FATAL(rclcpp::get_logger("Mcp23017SystemHardware"),
                             "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                             joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        // Try parse the interface parameters
        try {
            for (const hardware_interface::ComponentInfo &joint : info_.joints) {
                min_positions_.push_back(std::stod(joint.command_interfaces[0].min));
                max_positions_.push_back(std::stod(joint.command_interfaces[0].max));
                // position_state_interface_name_ = joint_state_interfaces_.begin()->first;
                // position_command_interface_name_ = joint_command_interfaces_.begin()->first;
        } catch (const std::exception& e) {
            RCLCPP_FATAL(get_logger(), "Failed to parse interface parameters: %s", e.what());
            return CallbackReturn::ERROR;
        }

        // Validate position bounds
        if (min_position_ > max_position_) {
            RCLCPP_FATAL(get_logger(), "Invalid Position bounds specified.");
            return CallbackReturn::ERROR;
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> Mcp23017SystemHardware::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        for (auto i = 0u; i < info_.joints.size(); i++) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> Mcp23017SystemHardware::export_command_interfaces() {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (auto i = 0u; i < info_.joints.size(); i++) {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
        }

        return command_interfaces;
    }

    hardware_interface::CallbackReturn Mcp23017SystemHardware::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/) {
        RCLCPP_INFO(rclcpp::get_logger("Mcp23017SystemHardware"), "Configuring ...please wait...");

        try {
            // Create or fetch the shared I2C bus instance
            i2c_bus_ = std::make_shared<hardware::I2CPeripheral>(cfg_.bus_name);

        } catch (const std::exception &e) {
            RCLCPP_FATAL(rclcpp::get_logger("Mcp23017SystemHardware"), "Error initializing I2C Bus: %s", e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (!i2c_bus_) {
            RCLCPP_FATAL(rclcpp::get_logger("Mcp23017SystemHardware"), "Failed to initialize I2C bus.");
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Setup and initialize the PCA object
        try {
            mcp_.setup(i2c_bus_, cfg_.i2c_address);

        } catch (const std::exception &e) {
            RCLCPP_FATAL(rclcpp::get_logger("Mcp23017SystemHardware"), "Error initializing MCP23017: %s", e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger("Mcp23017SystemHardware"), "Successfully configured!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn Mcp23017SystemHardware::on_cleanup(
        const rclcpp_lifecycle::State & /*previous_state*/) {
        RCLCPP_INFO(rclcpp::get_logger("Mcp23017SystemHardware"), "Cleaning up ...please wait...");

        RCLCPP_INFO(rclcpp::get_logger("Mcp23017SystemHardware"), "Successfully cleaned up!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn Mcp23017SystemHardware::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/) {
        for (auto i = 0u; i < hw_commands_.size(); i++) {
            if (std::isnan(hw_commands_[i])) {
                hw_commands_[i] = 0;
            }
        }

        RCLCPP_INFO(rclcpp::get_logger("Mcp23017SystemHardware"), "Successfully activated!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn Mcp23017SystemHardware::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/) {
        RCLCPP_INFO(rclcpp::get_logger("Mcp23017SystemHardware"), "Successfully deactivated!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type Mcp23017SystemHardware::read(const rclcpp::Time & /*time*/,
                                                                 const rclcpp::Duration & /*period*/) {
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type Mcp23017SystemHardware::write(const rclcpp::Time & /*time*/,
                                                                  const rclcpp::Duration & /*period*/) {
        uint8_t solenoid_values_;

        for (auto i = 0u; i < hw_commands_.size(); i++) {
            uint8_t bit_value = static_cast<uint8_t>(hw_commands_[i]);

            // TODO: Should insert a check here to ensure value is either a 0 or 1?

            solenoid_values_ |= (bit_value << i);

            RCLCPP_INFO(rclcpp::get_logger("Mcp23017SystemHardware"), "Joint '%d' has command '%d'.", i, bit_value);
        }

        mcp_.set_gpio_state(solenoid_values_);

        return hardware_interface::return_type::OK;
    }

}  // namespace mcp23017_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mcp23017_hardware_interface::Mcp23017SystemHardware, hardware_interface::SystemInterface)