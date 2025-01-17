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

        // Parse the MCP23017 parameters
        cfg_.i2c_device = info_.hardware_parameters.at("i2c_device");
        cfg_.i2c_address = std::stoi(info_.hardware_parameters.at("i2c_address"));
        cfg_.freq_hz = std::stod(info_.hardware_parameters.at("frequency_hz"));

        hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        // min_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        // max_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        // min_duty_cycles_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        // max_duty_cycles_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

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
                min_positions_.push_back(std::stod(joint.command_interfaces[0].min));
                max_positions_.push_back(std::stod(joint.command_interfaces[0].max));
                min_duty_cycles_.push_back(std::stod(joint.parameters.at("min_duty_cycle")));
                max_duty_cycles_.push_back(std::stod(joint.parameters.at("max_duty_cycle")));
                // position_command_interface_names_ = joint_command_interfaces_.begin()->first;
                // position_state_interface_names_ = joint_state_interfaces_.begin()->first;
            }
        } catch (const std::exception &e) {
            RCLCPP_FATAL(rclcpp::get_logger("Pca9685SystemHardware"), "Failed to parse interface parameters: %s",
                         e.what());
            return CallbackReturn::ERROR;
        }

        // Validate position bounds
        for (auto i = 0u; i < info_.joints.size(); i++) {
            if (min_positions_[i] > max_positions_[i]) {
                RCLCPP_FATAL(rclcpp::get_logger("Pca9685SystemHardware"), "Invalid Position bounds specified.");
                return CallbackReturn::ERROR;
            }
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

        // Not sure if I need this second check
        if (!i2c_bus_) {
            RCLCPP_FATAL(rclcpp::get_logger("Pca9685SystemHardware"), "Failed to initialize I2C bus.");
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
                hw_commands_[i] = 0;
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

        double slope = (max_duty_cycle - min_duty_cycle) / (max_input - min_input);
        double offset = (max_duty_cycle + min_duty_cycle) / 2;

        return slope * clamped_command + offset;
    }

    hardware_interface::return_type Pca9685SystemHardware::write(const rclcpp::Time & /*time*/,
                                                                 const rclcpp::Duration & /*period*/) {
        for (auto i = 0u; i < hw_commands_.size(); i++) {
            double duty_cycle = command_to_duty_cycle(hw_commands_[i], min_positions_[i], max_positions_[i],
                                                      min_duty_cycles_[i], max_duty_cycles_[i]);

            RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"), "Joint '%d' has command '%f', duty_cycle: '%f'.",
                        i, hw_commands_[i], duty_cycle);

            pca_.connect();
            pca_.set_pwm_ms(i, duty_cycle);
        }

        return hardware_interface::return_type::OK;
    }

}  // namespace pca9685_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(pca9685_hardware_interface::Pca9685SystemHardware, hardware_interface::SystemInterface)