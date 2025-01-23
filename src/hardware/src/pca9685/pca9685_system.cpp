#include "hardware/pca9685/pca9685_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pca9685_hardware_interface {
    hardware_interface::CallbackReturn Pca9685SystemHardware::on_init(const hardware_interface::HardwareInfo &info) {
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Try to parse the PCA9685 parameters
        try {
            cfg_.i2c_device = info_.hardware_parameters.at("i2c_device");
            cfg_.i2c_address = std::stoi(info_.hardware_parameters.at("i2c_address"));
            cfg_.freq_hz = std::stod(info_.hardware_parameters.at("frequency_hz"));
            cfg_.min_duty_cycle = std::stod(info_.hardware_parameters.at("min_duty_cycle_ms"));
            cfg_.max_duty_cycle = std::stod(info_.hardware_parameters.at("max_duty_cycle_ms"));
        } catch (const std::exception &e) {
            RCLCPP_FATAL(rclcpp::get_logger("Pca9685SystemHardware"), "Failed to parse PCA9685 parameters: %s",
                         e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }

        hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

        // Validate the command interface
        for (const hardware_interface::ComponentInfo &joint : info_.joints) {
            // PCA9685System has one command interface on each output
            if (joint.command_interfaces.size() != 1) {
                RCLCPP_FATAL(rclcpp::get_logger("Pca9685SystemHardware"),
                             "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                             joint.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
                RCLCPP_FATAL(rclcpp::get_logger("Pca9685SystemHardware"),
                             "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                             joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        // Try parse the interface parameters
        try {
            for (const hardware_interface::ComponentInfo &joint : info_.joints) {
                for (const auto &param : joint.parameters) {
                    RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"), "Joint [%s] Parameter: [%s] = [%s]",
                                joint.name.c_str(), param.first.c_str(), param.second.c_str());
                }

                min_positions_.push_back(std::stod(joint.command_interfaces[0].min));
                max_positions_.push_back(std::stod(joint.command_interfaces[0].max));
            }
        } catch (const std::exception &e) {
            RCLCPP_FATAL(rclcpp::get_logger("Pca9685SystemHardware"), "Failed to parse interface parameters: %s",
                         e.what());
            return CallbackReturn::ERROR;
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> Pca9685SystemHardware::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        for (auto i = 0u; i < info_.joints.size(); i++) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> Pca9685SystemHardware::export_command_interfaces() {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (auto i = 0u; i < info_.joints.size(); i++) {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
        }

        return command_interfaces;
    }

    hardware_interface::CallbackReturn Pca9685SystemHardware::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/) {
        RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"), "Configuring ...please wait...");
        try {
            // Get the shared pointer for the I2C bus
            i2c_bus_ = hardware::I2CPeripheral::getInstance(cfg_.i2c_device);
        } catch (const std::exception &e) {
            RCLCPP_FATAL(rclcpp::get_logger("Pca9685SystemHardware"), "Error initializing I2C Bus: %s", e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Setup the PCA9685 object
        try {
            pca_.setup(i2c_bus_, cfg_.i2c_address);
        } catch (const std::exception &e) {
            RCLCPP_FATAL(rclcpp::get_logger("Pca9685SystemHardware"), "Error setting initial state of PCA9685: %s",
                         e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"), "Successfully configured!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn Pca9685SystemHardware::on_cleanup(
        const rclcpp_lifecycle::State & /*previous_state*/) {
        RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"), "Cleaning up ...please wait...");

        // Release the shared pointer (this will automatically close the I2C bus once the
        // last shared pointer instance is destroyed via the I2CPeripheral destructor)
        i2c_bus_.reset();

        RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"), "Successfully cleaned up!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn Pca9685SystemHardware::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/) {
        for (auto i = 0u; i < hw_commands_.size(); i++) {
            if (std::isnan(hw_commands_[i])) {
                hw_commands_[i] = (min_positions_[i] + max_positions_[i]) / 2;  // Initialize at mid-point
            }
        }

        // Initialize the PCA9685 object
        try {
            pca_.connect();
            pca_.init();
            pca_.set_pwm_freq(cfg_.freq_hz);
        } catch (const std::exception &e) {
            RCLCPP_FATAL(rclcpp::get_logger("Pca9685SystemHardware"), "Error initializing PCA9685: %s", e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"), "Successfully activated!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn Pca9685SystemHardware::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/) {
        RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"), "Successfully deactivated!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type Pca9685SystemHardware::read(const rclcpp::Time & /*time*/,
                                                                const rclcpp::Duration & /*period*/) {
        return hardware_interface::return_type::OK;
    }

    double Pca9685SystemHardware::command_to_duty_cycle(double command, double min_input, double max_input,
                                                        double min_duty_cycle, double max_duty_cycle) {
        double clamped_command = std::clamp(command, min_input, max_input);

        return map(command, min_input, max_input, min_duty_cycle, max_duty_cycle);
    }

    hardware_interface::return_type Pca9685SystemHardware::write(const rclcpp::Time & /*time*/,
                                                                 const rclcpp::Duration & /*period*/) {
        num_write_attempts_ = 0;
        write_success_ = false;

        // Connect to the I2C bus (if this fails, no point in continuing)
        try {
            pca_.connect();
        } catch (const std::exception &e) {
            RCLCPP_ERROR(rclcpp::get_logger("Pca9685SystemHardware"), "Failed to connect to I2CBus during write: %s",
                         e.what());
            return hardware_interface::return_type::ERROR;
        }

        for (auto i = 0u; i < hw_commands_.size(); i++) {
            bool write_success_ = false;

            if (current_pwm_values_[i] != hw_commands_[i]) {
                double duty_cycle = command_to_duty_cycle(hw_commands_[i], min_positions_[i], max_positions_[i],
                                                          cfg_.min_duty_cycle, cfg_.max_duty_cycle);

                // Try to write values to the I2C bus, re-attempt up to MAX_WRITE_ATTEMPTS times
                while (!write_success_ && num_write_attempts_ < MAX_WRITE_ATTEMPTS) {
                    try {
                        write_servo_duty_cycle(i, duty_cycle);
                    } catch (const std::exception &e) {
                        if (num_write_attempts_ < MAX_WRITE_ATTEMPTS) {
                            num_write_attempts_++;
                            RCLCPP_WARN(rclcpp::get_logger("Pca9685SystemHardware"),
                                        "Failed to write to PCA9685, re-trying (attempt %d): %s", e.what(),
                                        num_write_attempts_);
                            rclcpp::sleep_for(std::chrono::nanoseconds(I2C_REWRITE_DELAY_US * NS_PER_US));
                        } else {
                            RCLCPP_ERROR(rclcpp::get_logger("Pca9685SystemHardware"),
                                         "Failed to write to PCA9685 after maximum attempts: %s", e.what());
                            return hardware_interface::return_type::ERROR;
                        }
                    }
                }
            }
        }

        return hardware_interface::return_type::OK;
    }

    /**
     * Writes the current state of the PCA9685 to the I2C bus.
     *
     * @param channel The channel on the PCA9685 you want to set the PWM for.
     * @param duty_cycle The duty cycle in ms to be sent to the channel.
     * @throws std::system_error if an error occurs during the I2C communication.
     */
    void write_servo_duty_cycle(int channel, double duty_cycle) {
        pca_.set_pwm_ms(channel, duty_cycle);  // This function may throw an exception
        current_pwm_values_[channel] = hw_commands_[channel];
    }
}  // namespace pca9685_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(pca9685_hardware_interface::Pca9685SystemHardware, hardware_interface::SystemInterface)