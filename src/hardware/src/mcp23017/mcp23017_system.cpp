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

        // Try to parse the MCP23017 parameters
        try {
            cfg_.i2c_device = info_.hardware_parameters.at("i2c_device");
            cfg_.i2c_address = std::stoi(info_.hardware_parameters.at("i2c_address"));
        } catch (const std::exception &e) {
            RCLCPP_FATAL(rclcpp::get_logger("Mcp23017SystemHardware"), "Failed to parse MCP23017 parameters: %s",
                         e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }

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
                // position_command_interface_names_ = joint_command_interfaces_.begin()->first;
                // position_state_interface_names_ = joint_state_interfaces_.begin()->first;
            }
        } catch (const std::exception &e) {
            RCLCPP_FATAL(get_logger(), "Failed to parse interface parameters: %s", e.what());
            return CallbackReturn::ERROR;
        }

        // Validate position bounds
        for (auto i = 0u; i < info_.joints.size(); i++) {
            if (min_positions_[i] > max_positions_[i]) {
                RCLCPP_FATAL(get_logger(), "Invalid Position bounds specified.");
                return CallbackReturn::ERROR;
            }
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
            // Get the shared pointer for the I2C bus
            i2c_bus_ = hardware::I2CPeripheral::getInstance(cfg_.i2c_device);
        } catch (const std::exception &e) {
            RCLCPP_FATAL(rclcpp::get_logger("Mcp23017SystemHardware"), "Error initializing I2C Bus: %s", e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Setup the MCP23017 object
        try {
            mcp_.setup(i2c_bus_, cfg_.i2c_address);
        } catch (const std::exception &e) {
            RCLCPP_FATAL(rclcpp::get_logger("Mcp23017SystemHardware"), "Error setting initial state of MCP23017: %s",
                         e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger("Mcp23017SystemHardware"), "Successfully configured!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn Mcp23017SystemHardware::on_cleanup(
        const rclcpp_lifecycle::State & /*previous_state*/) {
        RCLCPP_INFO(rclcpp::get_logger("Mcp23017SystemHardware"), "Cleaning up ...please wait...");

        // Release the shared pointer (this will automatically close the I2C bus once the
        // last shared pointer instance is destroyed via the I2CPeripheral destructor)
        i2c_bus_.reset();

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

        // Initialize the MCP23017 object
        try {
            mcp_.connect();
            mcp_.init();
        } catch (const std::exception &e) {
            RCLCPP_FATAL(rclcpp::get_logger("Mcp23017SystemHardware"), "Error initializing MCP23017: %s", e.what());
            return hardware_interface::CallbackReturn::ERROR;
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
        uint8_t new_solenoid_values_ = 0;
        num_write_attempts_ = 0;
        write_success_ = false;

        for (auto i = 0u; i < hw_commands_.size(); i++) {
            // Round the value to the nearest integer (0 or 1)
            uint8_t bit_value = static_cast<uint8_t>(std::round(hw_commands_[i]));

            // Ensure the value is clamped to 0 or 1
            bit_value = std::min<uint8_t>(1, bit_value);

            new_solenoid_values_ |= (bit_value << i);

            // This causes more yap than Tony
            // RCLCPP_INFO(
            //     rclcpp::get_logger("Mcp23017SystemHardware"),
            //     "Joint '%d' has command '%f', rounded to '%d'.", i, hw_commands_[i], bit_value);
        }

        // Only perform the write if the new state is different from the current state
        if (new_solenoid_values_ != current_solenoid_values_) {

            // Connect to the I2C bus (if this fails, no point in continuing)
            try {
                mcp_.connect();
            } catch (const std::exception &e) {
                RCLCPP_ERROR(rclcpp::get_logger("Mcp23017SystemHardware"), "Failed to connect to I2CBus during write: %s",
                             e.what());
                return hardware_interface::return_type::ERROR;
            }

            // Try to write values to the I2C bus, attempt multiple times if necessary
            while (!success && num_write_attempts_ < MAX_WRITE_ATTEMPTS) {
                try {
                    write_solenoid_states(new_solenoid_values_)
                } catch (const std::exception &e) {
                    if (num_write_attempts_ < MAX_WRITE_ATTEMPTS) {
                        num_write_attempts_++;
                        RCLCPP_WARN(rclcpp::get_logger("Mcp23017SystemHardware"),
                                    "Failed to write to MCP23017, re-trying (attempt %d): %s", e.what(),
                                    num_write_attempts_);
                        // Wait 100 ms between re-write attempts
                        rclcpp::sleep_for(std::chrono::nanoseconds(I2C_REWRITE_DELAY_US * NS_PER_US));
                    } else {
                        RCLCPP_ERROR(rclcpp::get_logger("Mcp23017SystemHardware"),
                                     "Failed to write to MCP23017 after maximum attempts: %s", e.what());
                        return hardware_interface::return_type::ERROR;
                    }
                }
            } 

            num_write_attempts_ = 0;
        }

        /**
         * Writes the current state of the MCP23017 to the I2C bus.
         *
         * @param new_solenoid_values_ The new state to be written to the GPIOs.
         * @throws std::system_error if an error occurs during the I2C communication.
         */
        void write_solenoid_states(uint8_t new_solenoid_values_) {
            mcp_.set_gpio_state(new_solenoid_values_);  // This function may throw an exception
            current_solenoid_values_ = new_solenoid_values_;

            for (int i = 0; i < 5; ++i) {
                uint8_t bit_value = (current_solenoid_values_ >> i) & 0x01;
                RCLCPP_INFO(rclcpp::get_logger("Mcp23017SystemHardware"), "Bit %d: %d", i, bit_value);
            }
        }

        return hardware_interface::return_type::OK;
    }

}  // namespace mcp23017_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mcp23017_hardware_interface::Mcp23017SystemHardware, hardware_interface::SystemInterface)