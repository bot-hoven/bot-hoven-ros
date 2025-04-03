#include "hardware/pca9685/pca9685_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pca9685_hardware_interface {
    hardware_interface::CallbackReturn Pca9685SystemHardware::on_init(const hardware_interface::HardwareInfo &info) {
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Try to parse the PCA9685 parameters
        try {
            cfg_.i2c_device = info_.hardware_parameters.at("i2c_device");
            cfg_.pca_i2c_address = std::stoi(info_.hardware_parameters.at("pca_i2c_address"));
            cfg_.pca_freq_hz = std::stod(info_.hardware_parameters.at("pca_frequency_hz"));
        } catch (const std::exception &e) {
            RCLCPP_FATAL(rclcpp::get_logger("Pca9685SystemHardware"), "Failed to parse PCA9685 parameters: %s",
                         e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }

        hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

        // Resize our parameter vectors
        servo_channels_.resize(info_.joints.size());
        pwm_min_values_.resize(info_.joints.size());
        pwm_max_values_.resize(info_.joints.size());
        pwm_slopes_.resize(info_.joints.size());
        pwm_intercepts_.resize(info_.joints.size());
        current_command_values_.resize(info_.joints.size(), 0.5);  // Default to middle position

        // Validate the command interface
        for (auto i = 0u; i < info_.joints.size(); i++) {
            const hardware_interface::ComponentInfo &joint = info_.joints[i];

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
            for (auto i = 0u; i < info_.joints.size(); i++) {
                const hardware_interface::ComponentInfo &joint = info_.joints[i];

                // Print all parameters for debugging
                // for (const auto &param : joint.parameters) {
                //     RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"), "Joint [%s] Parameter: [%s] = [%s]",
                //                 joint.name.c_str(), param.first.c_str(), param.second.c_str());
                // }

                min_positions_.push_back(std::stod(joint.command_interfaces[0].min));
                max_positions_.push_back(std::stod(joint.command_interfaces[0].max));

                // Get joint-specific servo channel and calibration data
                if (joint.parameters.count("servo_channel") > 0) {
                    servo_channels_[i] = std::stoi(joint.parameters.at("servo_channel"));
                } else {
                    servo_channels_[i] = i;  // Default to index if not specified
                    RCLCPP_WARN(rclcpp::get_logger("Pca9685SystemHardware"),
                                "No servo_channel parameter for joint %s, defaulting to %d", joint.name.c_str(), i);
                }

                // Get PWM min/max values
                if (joint.parameters.count("pwm_min") > 0 && joint.parameters.count("pwm_max") > 0) {
                    pwm_min_values_[i] = std::stoi(joint.parameters.at("pwm_min"));
                    pwm_max_values_[i] = std::stoi(joint.parameters.at("pwm_max"));
                } else {
                    // Default values if not specified
                    pwm_min_values_[i] = 500;   // 0.5ms
                    pwm_max_values_[i] = 2500;  // 2.5ms
                    RCLCPP_WARN(rclcpp::get_logger("Pca9685SystemHardware"),
                                "No pwm_min/pwm_max parameters for joint %s, defaulting to %d/%d", joint.name.c_str(),
                                pwm_min_values_[i], pwm_max_values_[i]);
                }

                // Get calibration parameters
                if (joint.parameters.count("pwm_slope") > 0 && joint.parameters.count("pwm_intercept") > 0) {
                    pwm_slopes_[i] = std::stod(joint.parameters.at("pwm_slope"));
                    pwm_intercepts_[i] = std::stod(joint.parameters.at("pwm_intercept"));
                } else {
                    // Default linear mapping if not specified
                    pwm_slopes_[i] = 1.0;
                    pwm_intercepts_[i] = 0.0;
                    RCLCPP_WARN(rclcpp::get_logger("Pca9685SystemHardware"),
                                "No pwm_slope/pwm_intercept parameters for joint %s, defaulting to %f/%f",
                                joint.name.c_str(), pwm_slopes_[i], pwm_intercepts_[i]);
                }

                // RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"),
                //            "Joint [%s]: channel=%d, min_pos=%f, max_pos=%f, pwm_min=%d, pwm_max=%d, slope=%f,
                //            intercept=%f", joint.name.c_str(), servo_channels_[i], min_positions_[i],
                //            max_positions_[i], pwm_min_values_[i], pwm_max_values_[i], pwm_slopes_[i],
                //            pwm_intercepts_[i]);
            }
        } catch (const std::exception &e) {
            RCLCPP_FATAL(rclcpp::get_logger("Pca9685SystemHardware"), "Failed to parse interface parameters: %s",
                         e.what());
            return CallbackReturn::ERROR;
        }

        // Validate position bounds
        for (auto i = 0u; i < info_.joints.size(); i++) {
            if (min_positions_[i] > max_positions_[i]) {
                RCLCPP_FATAL(get_logger(), "Invalid Position bounds specified.");
                return CallbackReturn::ERROR;
            }
        }

        // Try to parse the ADS7138 parameters
        try {
            cfg_.ads_i2c_address = std::stoi(info_.hardware_parameters.at("ads_i2c_address"));

            // Optional parameter with default
            if (info_.hardware_parameters.count("conversion_rate") > 0) {
                cfg_.ads_conversion_rate = std::stod(info_.hardware_parameters.at("ads_conversion_rate"));
            } else {
                cfg_.ads_conversion_rate = 20.0;  // Default to 20 kSPS
            }
        } catch (const std::exception &e) {
            RCLCPP_FATAL(rclcpp::get_logger("Ads7138SystemHardware"), "Failed to parse ADS7138 parameters: %s",
                         e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Resize state values vector
        hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

        // For each joint, we need to extract parameters and validate interfaces
        channel_mapping_.resize(info_.joints.size());
        min_angles_.resize(info_.joints.size());
        max_angles_.resize(info_.joints.size());
        adc_slopes_.resize(info_.joints.size());
        adc_intercepts_.resize(info_.joints.size());

        // Validate interfaces - ADS7138 only provides state interfaces, no command interfaces
        for (auto i = 0u; i < info_.joints.size(); i++) {
            const hardware_interface::ComponentInfo &joint = info_.joints[i];

            // Print all parameters for this joint
            // RCLCPP_INFO(rclcpp::get_logger("Ads7138SystemHardware"), "Joint [%s] parameters:", joint.name.c_str());
            // for (const auto &param : joint.parameters) {
            //     RCLCPP_INFO(rclcpp::get_logger("Ads7138SystemHardware"), "  %s = %s",
            //              param.first.c_str(), param.second.c_str());
            // }

            // ADS7138 should have state interfaces
            if (joint.state_interfaces.empty()) {
                RCLCPP_WARN(rclcpp::get_logger("Ads7138SystemHardware"), "Joint '%s' has no state interfaces.",
                            joint.name.c_str());
                continue;
            }

            bool has_position_interface = false;
            for (const auto &state_interface : joint.state_interfaces) {
                if (state_interface.name == hardware_interface::HW_IF_POSITION) {
                    has_position_interface = true;
                    break;
                }
            }

            if (!has_position_interface) {
                RCLCPP_WARN(rclcpp::get_logger("Ads7138SystemHardware"), "Joint '%s' has no position state interface.",
                            joint.name.c_str());
                continue;
            }

            // Try to parse the joint parameters
            try {
                // Extract channel mapping from parameters
                if (joint.parameters.count("adc_channel") > 0) {
                    int channel = std::stoi(joint.parameters.at("adc_channel"));
                    if (channel < 0 || channel > 7) {
                        throw std::out_of_range("ADS7138 channel must be between 0 and 7");
                    }
                    channel_mapping_[i] = static_cast<uint8_t>(channel);
                } else {
                    // Default to index if not specified
                    channel_mapping_[i] = i < 8 ? static_cast<uint8_t>(i) : 0;
                    RCLCPP_WARN(rclcpp::get_logger("Ads7138SystemHardware"),
                                "No adc_channel parameter for joint %s, defaulting to %d", joint.name.c_str(),
                                channel_mapping_[i]);
                }

                // Get angle range from command interface limits or parameters
                if (joint.parameters.count("min_angle") > 0 && joint.parameters.count("max_angle") > 0) {
                    min_angles_[i] = std::stod(joint.parameters.at("min_angle"));
                    max_angles_[i] = std::stod(joint.parameters.at("max_angle"));
                } else if (!joint.command_interfaces.empty()) {
                    // If no explicit angle range, try to use command interface limits
                    min_angles_[i] = std::stod(joint.command_interfaces[0].min);
                    max_angles_[i] = std::stod(joint.command_interfaces[0].max);
                } else {
                    // Default values
                    min_angles_[i] = 0.0;
                    max_angles_[i] = 90.0;
                    RCLCPP_WARN(rclcpp::get_logger("Ads7138SystemHardware"),
                                "No min_angle/max_angle parameters for joint %s, defaulting to %f/%f",
                                joint.name.c_str(), min_angles_[i], max_angles_[i]);
                }

                // Get calibration parameters
                if (joint.parameters.count("adc_slope") > 0 && joint.parameters.count("adc_intercept") > 0) {
                    adc_slopes_[i] = std::stod(joint.parameters.at("adc_slope"));
                    adc_intercepts_[i] = std::stod(joint.parameters.at("adc_intercept"));
                } else {
                    // Default to linear mapping if not specified
                    adc_slopes_[i] = 1.0;
                    adc_intercepts_[i] = 0.0;
                    RCLCPP_WARN(rclcpp::get_logger("Ads7138SystemHardware"),
                                "No adc_slope/adc_intercept parameters for joint %s, defaulting to %f/%f",
                                joint.name.c_str(), adc_slopes_[i], adc_intercepts_[i]);
                }

                // RCLCPP_INFO(rclcpp::get_logger("Ads7138SystemHardware"),
                //          "Joint [%s] mapped to ADS7138 channel [%d], angle range [%f, %f], calibration [%f, %f]",
                //          joint.name.c_str(), channel_mapping_[i], min_angles_[i], max_angles_[i],
                //          adc_slopes_[i], adc_intercepts_[i]);

            } catch (const std::exception &e) {
                RCLCPP_FATAL(rclcpp::get_logger("Ads7138SystemHardware"),
                             "Failed to parse interface parameters for joint '%s': %s", joint.name.c_str(), e.what());
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> Pca9685SystemHardware::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        for (auto i = 0u; i < info_.joints.size(); i++) {
            // Only export state interfaces if they have the adc_channel parameter
            // This allows us to work with joints that already have state interfaces from other components
            if (info_.joints[i].parameters.count("adc_channel") > 0) {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));

                RCLCPP_INFO(rclcpp::get_logger("Ads7138SystemHardware"),
                            "Exporting state interface for joint [%s] mapped to ADS7138 channel [%d]",
                            info_.joints[i].name.c_str(), channel_mapping_[i]);
            }
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
            pca_.setup(i2c_bus_, cfg_.pca_i2c_address);
        } catch (const std::exception &e) {
            RCLCPP_FATAL(rclcpp::get_logger("Pca9685SystemHardware"), "Error setting initial state of PCA9685: %s",
                         e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"), "Successfully configured!");

        // try {
        //     // Get the shared pointer for the I2C bus
        //     i2c_bus_ = hardware::I2CPeripheral::getInstance(cfg_.i2c_device);
        // } catch (const std::exception &e) {
        //     RCLCPP_FATAL(rclcpp::get_logger("Ads7138SystemHardware"), "Error initializing I2C Bus: %s", e.what());
        //     return hardware_interface::CallbackReturn::ERROR;
        // }

        // Setup the ADS7138 object
        try {
            ads_.setup(i2c_bus_, cfg_.ads_i2c_address);
        } catch (const std::exception &e) {
            RCLCPP_FATAL(rclcpp::get_logger("Ads7138SystemHardware"), "Error setting up ADS7138: %s", e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger("Ads7138SystemHardware"), "Successfully configured!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn Pca9685SystemHardware::on_cleanup(
        const rclcpp_lifecycle::State & /*previous_state*/) {
        RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"), "Cleaning up ...please wait...");

        // Release the shared pointer (this will automatically close the I2C bus once the
        // last shared pointer instance is destroyed via the I2CPeripheral destructor)
        i2c_bus_.reset();

        RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"), "Successfully cleaned up!");

        RCLCPP_INFO(rclcpp::get_logger("Ads7138SystemHardware"), "Cleaning up ...please wait...");

        // Release the shared pointer (this will automatically close the I2C bus once the
        // last shared pointer instance is destroyed via the I2CPeripheral destructor)
        i2c_bus_.reset();

        RCLCPP_INFO(rclcpp::get_logger("Ads7138SystemHardware"), "Successfully cleaned up!");

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
            pca_.init();  // This will enable auto-increment
            pca_.set_pwm_freq(cfg_.pca_freq_hz);

            // Explicitly ensure auto-increment is enabled
            pca_.enable_auto_increment();

        } catch (const std::exception &e) {
            RCLCPP_FATAL(rclcpp::get_logger("Pca9685SystemHardware"), "Error initializing PCA9685: %s", e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"), "Successfully activated!");

        RCLCPP_INFO(rclcpp::get_logger("Ads7138SystemHardware"), "Activating ...please wait...");

        // Initialize hw_states_ with default values
        for (auto i = 0u; i < hw_states_.size(); i++) {
            if (std::isnan(hw_states_[i])) {
                hw_states_[i] = 0;  // Default to middle position
            }
        }

        // Initialize the ADS7138 object
        try {
            ads_.connect();
            ads_.init();
            rclcpp::sleep_for(std::chrono::milliseconds(5));  // Wait for the device to stabilize

            // Set conversion rate based on configuration
            uint8_t rate_setting;
            if (cfg_.ads_conversion_rate <= 1.0)
                rate_setting = ads7138_hardware_interface::ADS7138_CONV_RATE_1K;
            else if (cfg_.ads_conversion_rate <= 2.0)
                rate_setting = ads7138_hardware_interface::ADS7138_CONV_RATE_2K;
            else if (cfg_.ads_conversion_rate <= 5.0)
                rate_setting = ads7138_hardware_interface::ADS7138_CONV_RATE_5K;
            else if (cfg_.ads_conversion_rate <= 10.0)
                rate_setting = ads7138_hardware_interface::ADS7138_CONV_RATE_10K;
            else if (cfg_.ads_conversion_rate <= 20.0)
                rate_setting = ads7138_hardware_interface::ADS7138_CONV_RATE_20K;
            else if (cfg_.ads_conversion_rate <= 50.0)
                rate_setting = ads7138_hardware_interface::ADS7138_CONV_RATE_50K;
            else if (cfg_.ads_conversion_rate <= 100.0)
                rate_setting = ads7138_hardware_interface::ADS7138_CONV_RATE_100K;
            else
                rate_setting = ads7138_hardware_interface::ADS7138_CONV_RATE_200K;

            ads_.set_conversion_rate(rate_setting);
            rclcpp::sleep_for(std::chrono::milliseconds(5));  // Wait for the device to stabilize

        } catch (const std::exception &e) {
            RCLCPP_FATAL(rclcpp::get_logger("Ads7138SystemHardware"), "Error initializing ADS7138: %s", e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn Pca9685SystemHardware::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/) {
        RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"), "Successfully deactivated!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    double Pca9685SystemHardware::adc_value_to_angle(uint16_t adc_value, size_t joint_idx) {
        // Convert ADC value to angle using calibration data
        if (joint_idx >= adc_slopes_.size() || joint_idx >= adc_intercepts_.size()) {
            RCLCPP_ERROR(rclcpp::get_logger("Ads7138SystemHardware"), "Invalid joint index %zu for calibration",
                         joint_idx);
            return 0.0;
        }

        // Apply calibration formula: angle = slope * adc_value + intercept
        double angle = adc_slopes_[joint_idx] * adc_value + adc_intercepts_[joint_idx];

        // Clamp to range
        return std::clamp(angle, min_angles_[joint_idx], max_angles_[joint_idx]);
    }

    hardware_interface::return_type Pca9685SystemHardware::read(const rclcpp::Time & /*time*/,
                                                                const rclcpp::Duration & /*period*/) {
        // Collect channels to read
        std::vector<uint8_t> channels_to_read;
        std::vector<size_t> indices_map;  // Maps channel to joint index

        for (auto i = 0u; i < info_.joints.size(); i++) {
            if (info_.joints[i].parameters.count("adc_channel") > 0) {
                channels_to_read.push_back(channel_mapping_[i]);
                indices_map.push_back(i);
            }
        }

        if (channels_to_read.empty()) {
            return hardware_interface::return_type::OK;
        }

        // Read each channel individually instead of using auto-sequence
        bool any_success = false;

        for (size_t i = 0; i < channels_to_read.size(); i++) {
            uint8_t channel = channels_to_read[i];
            size_t joint_idx = indices_map[i];

            int channel_attempts = 0;
            bool channel_success = false;

            while (!channel_success && channel_attempts < MAX_READ_ATTEMPTS) {
                try {
                    // Ensure we're connected
                    ads_.connect();

                    // Use simpler single-channel read - avoid auto-sequence
                    uint16_t adc_value = ads_.read_channel(channel);

                    // Convert to angle
                    hw_states_[joint_idx] = adc_value_to_angle(adc_value, joint_idx);

                    channel_success = true;
                    any_success = true;

                    RCLCPP_DEBUG(rclcpp::get_logger("Pca9685SystemHardware"),
                                 "Joint '%s' (channel %d): ADC=%d, Angle=%f", info_.joints[joint_idx].name.c_str(),
                                 channel, adc_value, hw_states_[joint_idx]);

                } catch (const std::exception &e) {
                    channel_attempts++;

                    RCLCPP_WARN(rclcpp::get_logger("Pca9685SystemHardware"),
                                "Failed to read channel %d (attempt %d/%d): %s", channel, channel_attempts,
                                MAX_READ_ATTEMPTS, e.what());

                    // Add delay between attempts
                    rclcpp::sleep_for(std::chrono::microseconds(500 * channel_attempts)); // Progressive delay
                }
            }

            // If we couldn't read this channel, log but continue with others
            if (!channel_success) {
                RCLCPP_ERROR(rclcpp::get_logger("Pca9685SystemHardware"), "Failed to read channel %d after %d attempts",
                             channel, MAX_READ_ATTEMPTS);
            }
        }

        // As long as we read at least one channel successfully, return OK
        return any_success ? hardware_interface::return_type::OK : hardware_interface::return_type::ERROR;
    }

    // Convert commanded position to PWM duty cycle using calibration data
    double Pca9685SystemHardware::angle_to_duty_cycle(double angle, int joint_index) {
        // Ensure joint_index is valid
        if (joint_index < 0 || joint_index >= static_cast<int>(info_.joints.size())) {
            RCLCPP_ERROR(get_logger(), "Invalid joint index: %d", joint_index);
            return 1.5;  // Default to neutral position (1.5ms)
        }
    
        // Calculate PWM in microseconds using the linear calibration
        // Directly apply slope and intercept to the command value
        int pwm_us = static_cast<int>((pwm_slopes_[joint_index] * (angle)) + pwm_intercepts_[joint_index]);
        
        // Constrain PWM to safe limits
        int min_us = pwm_min_values_[joint_index];
        int max_us = pwm_max_values_[joint_index];
        
        if (pwm_us < min_us) pwm_us = min_us;
        if (pwm_us > max_us) pwm_us = max_us;
        
        // Convert microseconds to milliseconds for set_pwm_ms
        return pwm_us / 1000.0;
    }

    hardware_interface::return_type Pca9685SystemHardware::write(const rclcpp::Time & /*time*/,
                                                                 const rclcpp::Duration & /*period*/) {
        // Collect joints that need updating for batch write
        std::vector<int> channels_to_update;
        std::vector<double> duty_cycles_ms;

        for (auto i = 0u; i < hw_commands_.size(); i++) {
            if (current_command_values_[i] != hw_commands_[i]) {
                // Calculate duty cycle using calibration data
                double angle = (hw_commands_[i] * 180 / M_PI) + 90;
                double duty_cycle_ms = angle_to_duty_cycle(angle, i);

                channels_to_update.push_back(servo_channels_[i]);
                duty_cycles_ms.push_back(duty_cycle_ms);

                // Update the current value so we don't keep sending the same command
                current_command_values_[i] = hw_commands_[i];
                
                RCLCPP_DEBUG(rclcpp::get_logger("Pca9685SystemHardware"),
                "Servo: %d (channel %d), Command: %f, Duty Cycle: %f ms", i, servo_channels_[i],
                hw_commands_[i], duty_cycle_ms);
            }
        }
        for (auto i = 0u; i < channels_to_update.size(); i++) {

            pca_.set_pwm_ms(channels_to_update[i], duty_cycles_ms[i]);
        }


        // // If nothing to update, return OK
        // if (channels_to_update.empty()) {
        //     return hardware_interface::return_type::OK;
        // }

        // // Try to write values to the I2C bus, re-attempt up to MAX_WRITE_ATTEMPTS times
        // num_write_attempts_ = 0;
        // write_success_ = false;

        // while (!write_success_ && num_write_attempts_ < MAX_WRITE_ATTEMPTS) {
        //     try {
        //         pca_.connect();  // This function may throw an exception

        //         // Convert milliseconds to microseconds for better precision
        //         std::vector<uint16_t> us_values;
        //         for (double ms : duty_cycles_ms) {
        //             us_values.push_back(static_cast<uint16_t>(ms * 1000.0));
        //         }

        //         // Use batch write if multiple servos need updating
        //         if (channels_to_update.size() > 1) {
        //             // Use batch write function with auto-increment
        //             pca_.write_microseconds_batch(channels_to_update, us_values);
                    

        //             RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"), "Updated %zu servos in batch mode",
        //                         channels_to_update.size());
        //         } else {
        //             // Single servo update
        //             pca_.write_microseconds(channels_to_update[0], us_values[0]);

        //             RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"), "Updated servo at channel %d with %d us",
        //                         channels_to_update[0], us_values[0]);
        //         }

        //         write_success_ = true;
        //     } catch (const std::exception &e) {
        //         num_write_attempts_++;
        //         RCLCPP_WARN(rclcpp::get_logger("Pca9685SystemHardware"),
        //                     "Failed to write to PCA9685, re-trying (attempt %d): %s", num_write_attempts_, e.what());
        //         rclcpp::sleep_for(std::chrono::nanoseconds(WRITE_ATTEMP_DELAY_US * 1000));
        //     }
        // }

        // if (num_write_attempts_ == MAX_WRITE_ATTEMPTS) {
        //     RCLCPP_ERROR(rclcpp::get_logger("Pca9685SystemHardware"),
        //                  "Failed to write to PCA9685 after maximum attempts");
        //     return hardware_interface::return_type::ERROR;
        // }

        return hardware_interface::return_type::OK;
    }
}  // namespace pca9685_hardware_interface

PLUGINLIB_EXPORT_CLASS(pca9685_hardware_interface::Pca9685SystemHardware, hardware_interface::SystemInterface)