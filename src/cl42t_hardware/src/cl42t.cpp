#include "cl42t_hardware/cl42t_hardware/cl42t.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace cl42t_hardware {

    hardware_interface::CallbackReturn CL42T::on_init(const hardware_interface::HardwareInfo& info) {
        if (hardware_interface::ActuatorInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Try initialize hardware parameters from the config
        try {
            pulses_per_rev_ = std::stoi(info.hardware_parameters.at("pulses_per_rev"));
            chip_name_ = info.hardware_parameters.at("gpio.chip_name");

            int i = 0;
            std::string prefix = "gpio.pins." + std::to_string(i);
            // Iterate over all defined gpio pins (assumes pins are parameterized last in config)
            while (info.hardware_parameters.find(prefix + ".pin_number") != info.hardware_parameters.end()) {
                GPIOPin pin;
                pin.pin_number = std::stoi(info.hardware_parameters.at(prefix + ".pin_number"));
                pin.descriptor = info.hardware_parameters.at(prefix + ".descriptor");
                pin.direction = info.hardware_parameters.at(prefix + ".direction");
                pin.init_value = std::stoi(info.hardware_parameters.at(prefix + ".init_value"));
                gpio_pins_.push_back(pin);

                prefix = "gpio.pins." + std::to_string(++i);
            }
        } catch (const std::exception& e) {
            RCLCPP_FATAL(get_logger(), "Failed to parse hardware parameters: %s", e.what());
            return CallbackReturn::ERROR;
        }

        // Validate pulse_per_rev corresponds to a valid setting
        if (std::find(ValidPulsesPerRev.begin(), ValidPulsesPerRev.end(), pulses_per_rev_) == ValidPulsesPerRev.end()) {
            RCLCPP_FATAL(
                get_logger(),
                "Config specified an invalid setting of %d pulses per revolution. Consult the CL42T-V4.1 manual for "
                "possible settings and ensure the physical hardware is set accordingly.",
                pulses_per_rev_);
            return CallbackReturn::ERROR;
        }

        // Parse the joint associated with the interface
        const hardware_interface::ComponentInfo& joint = info.joints[0];

        // Validate the state interface
        if (joint.state_interfaces.size() != 1) {
            RCLCPP_FATAL(get_logger(), "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
                         joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(get_logger(), "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
                         joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Validate the command interface
        if (joint.command_interfaces.size() != 1) {
            RCLCPP_FATAL(get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                         joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
                         joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
                         hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Try initialize the joint and interface parameters
        try {
            min_position_ = std::stod(joint.parameters.at("min_position"));
            max_position_ = std::stod(joint.parameters.at("max_position"));
            position_state_interface_name_ = joint_state_interfaces_.begin()->first;
            position_command_interface_name_ = joint_command_interfaces_.begin()->first;
        } catch (const std::exception& e) {
            RCLCPP_FATAL(get_logger(), "Failed to parse joint parameters: %s", e.what());
            return CallbackReturn::ERROR;
        }

        // validate posoition bounds

        // Intialize internal class variables
        dir_ = RotateDir::CW;
        angular_resolution_ = 2 * M_PI / pulses_per_rev_;  // angle in radians per pulse;
        num_pulses_ = 0;

        RCLCPP_INFO(get_logger(), "Successfully Initialized %s.", info.name.c_str());
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn CL42T::on_configure(const rclcpp_lifecycle::State& previous_state) {
        (void)previous_state;

        // Try request each GPIO pin with the corresponding initial value and descriptor
        try {
            gpiod::chip chip(chip_name_);

            for (size_t i = 0; i < gpio_pins_.size(); i++) {
                gpiod::line line = chip.get_line(gpio_pins_[i].pin_number);
                int request_dir = gpio_pins_[i].direction.compare("output") == 0 ? gpiod::line_request::DIRECTION_OUTPUT
                                                                                 : gpiod::line_request::DIRECTION_INPUT;
                gpiod::line_request config = {gpio_pins_[i].descriptor, request_dir, 0};
                line.request(config, gpio_pins_[i].init_value);
                gpio_lines_.push_back(std::move(line));
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Failed to request GPIO communication: %s", e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Reset the state interface
        set_state(position_state_interface_name_, 0.0);

        RCLCPP_INFO(get_logger(), "Successfully configured GPIO communication.");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn CL42T::on_cleanup(const rclcpp_lifecycle::State& previous_state) {
        (void)previous_state;
        for (const auto& line : gpio_lines_) {
            line.release();
        }

        RCLCPP_INFO(get_logger(), "Successfully cleaned up GPIO communication.");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type CL42T::read(const rclcpp::Time& time, const rclcpp::Duration& period) {
        (void)time;    // suppress unused parameter warning
        (void)period;  // suppress unused parameter warning

        // update the state based on the executed signal
        double new_position = get_state(position_state_interface_name_);
        double position_change = num_pulses_ * angular_resolution_;
        new_position += (dir_ == RotateDir::CCW) ? position_change : -position_change;
        set_state(position_state_interface_name_, new_position);

        // reset num_pulses_
        num_pulses_ = 0;

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type CL42T::write(const rclcpp::Time& time, const rclcpp::Duration& period) {
        (void)time;    // suppress unused parameter warning
        (void)period;  // suppress unused parameter warning

        double desired_position = get_command(position_command_interface_name_);

        // Ensure the desired positon is within range
        if (desired_position < min_position_ || desired_position > max_position_) {
            RCLCPP_ERROR(get_logger(), "Invalid Command. Ensure commands are in [%lf, %lf].", min_position_,
                         max_position_);
            return hardware_interface::return_type::ERROR;
        }

        // Calculate the angle to rotate
        double position_change = desired_position - get_state(position_state_interface_name_);

        // Process the command if the difference exceeds the resolution of the stepper motor
        if (std::abs(position_change) >= angular_resolution_) {
            // Determine the rotation direction and if the direction changed
            int dir = position_change > 0 ? RotateDir::CCW : RotateDir::CW;
            if (dir_ != dir) {
                gpio_lines_[1].set_value(dir);
                rclcpp::Rate(rclcpp::Duration(std::chrono::nanoseconds(MinDirTimeUsec * NsecPerUsec))).sleep();
                dir_ = dir;
            }

            // Determine the number of pulses needed to achieve the desired position
            if (desired_position >= (max_position_ - (angular_resolution_ / 2)) ||
                desired_position <= (min_position_ + (angular_resolution_ / 2))) {
                // Cast the result if rounding can produce out of bounds positions
                num_pulses_ = static_cast<int>(std::abs(position_change) / angular_resolution_);
            } else {
                num_pulses_ = std::round(std::abs(position_change) / angular_resolution_);
            }

            // Ensure the number of pulses can be executed prior to the next command otherwise adjust
            // the number and operate as close as possible given the constraints of the hardware
            if (num_pulses_ * MinPulseTimeUsec > period.nanoseconds() / NsecPerUsec) {
                num_pulses_ = static_cast<int>(period.nanoseconds() / (MinPulseTimeUsec * NsecPerUsec));
                RCLCPP_WARN(get_logger(), "The desired position cannot be reached prior to the next command.");
            }

            // Determine the cycle period based on the command period and the number of pulses needed
            // integer cast  ensures time constraints are satisfied
            int cycle_period_us = static_cast<int>(period.nanoseconds() / (NsecPerUsec * num_pulses_));

            // Generate pulses
            try {
                generate_pulses(gpio_lines_[0], num_pulses_, cycle_period_us, DutyCycle);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(get_logger(), "Failed to generate pulses: %s", e.what());
                return hardware_interface::return_type::ERROR;
            }
        }

        return hardware_interface::return_type::OK;
    }

    /**
     * Generate num_pulses at the specified duty cycle, each lasting cycle_period_us.
     */
    void generate_pulses(gpiod::line& pul_line, int num_pulses, int cycle_period_us, float duty_cycle) {
        // Ensure duty cycle is between 0 and 1
        if (duty_cycle <= 0.0 || duty_cycle > 1.0) {
            throw std::invalid_argument("Invalid Duty cycle. Ensure the Duty cycle is in (0, 1].");
        }

        // Determine the high and low times based on the duty cycle
        int high_time_us = static_cast<int>(cycle_period_us * duty_cycle);
        int low_time_us = cycle_period_us - high_time_us;

        // Ensure high & low times satisfy the driver constraints, operate with the closest specifications if invalid
        if (high_time_us < MinPulseTimeUsec) {
            RCLCPP_WARN(rclcpp::get_logger("cl42t_hardware"),
                        "Desired high time of %d usec does not satisfy driver time constraints. Resorting to minimum "
                        "allowable high time.",
                        high_time_us);
            high_time_us = MinPulseTimeUsec;
        }
        if (low_time_us < MinPulseTimeUsec) {
            RCLCPP_WARN(rclcpp::get_logger("cl42t_hardware"),
                        "Desired low time of %d usec does not satisfy driver time constraints. Resorting to minimum "
                        "allowable low time.",
                        low_time_us);
            low_time_us = MinPulseTimeUsec;
        }

        // Generate pulses at the duty cycle
        for (int i = 0; i < num_pulses; i++) {
            pul_line.set_value(LogicalHigh);
            rclcpp::Rate(rclcpp::Duration(std::chrono::nanoseconds(high_time_us * NsecPerUsec))).sleep();
            pul_line.set_value(LogicalLow);
            rclcpp::Rate(rclcpp::Duration(std::chrono::nanoseconds(low_time_us * NsecPerUsec))).sleep();
        }
    }

}  // namespace cl42t_hardware

// Register the plugin
PLUGINLIB_EXPORT_CLASS(cl42t_hardware::CL42T, hardware_interface::ActuatorInterface)