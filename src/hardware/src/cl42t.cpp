#include "hardware/cl42t.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace cl42t_hardware {

    hardware_interface::CallbackReturn CL42T::on_init(const hardware_interface::HardwareInfo& info) {
        if (hardware_interface::ActuatorInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Try parse the hardware parameters
        try {
            pulses_per_rev_ = std::stoi(info.hardware_parameters.at("pulses_per_rev"));
        } catch (const std::exception& e) {
            RCLCPP_FATAL(get_logger(), "Failed to parse hardware parameters: %s", e.what());
            return CallbackReturn::ERROR;
        }

        // Validate pulse_per_rev corresponds to a valid setting
        if (std::find(ValidPulsesPerRev.begin(), ValidPulsesPerRev.end(), pulses_per_rev_) == ValidPulsesPerRev.end()) {
            RCLCPP_FATAL(get_logger(),
                         "Invalid setting of %d pulses per revolution specified. Consult the CL42T-V4.1 manual for "
                         "possible settings and ensure the physical hardware is set accordingly.",
                         pulses_per_rev_);
            return CallbackReturn::ERROR;
        }

        // Try parse the GPIO parameters
        try {
            for (const hardware_interface::ComponentInfo& gpio_component : info.gpios) {
                GPIOPin pin;
                pin.chip_name = gpio_component.parameters.at("chip_name");
                pin.pin_number = std::stoi(gpio_component.parameters.at("pin_number"));
                pin.descriptor = gpio_component.parameters.at("descriptor");
                pin.direction = gpio_component.parameters.at("direction");
                pin.init_value = std::stoi(gpio_component.parameters.at("init_value"));
                gpio_pins_.push_back(pin);
            }
        } catch (const std::exception& e) {
            RCLCPP_FATAL(get_logger(), "Failed to parse GPIO parameters: %s", e.what());
            return CallbackReturn::ERROR;
        }

        // Validate GPIO paramaters
        // TODO: update this validation with ValidPinDescriptors.size() once remaining pin functionality is implemented
        if (info_.gpios.size() != 2) {
            RCLCPP_FATAL(get_logger(), "Interface has '%ld' GPIO components. '%d' expected.", info_.gpios.size(), 2);
            return hardware_interface::CallbackReturn::ERROR;
        }
        for (size_t i = 0; i < gpio_pins_.size(); i++) {
            // Validate GPIO pin descriptor corresponds to a valid setting
            if (std::find(ValidPinDescriptors.begin(), ValidPinDescriptors.end(), gpio_pins_[i].descriptor) ==
                ValidPinDescriptors.end()) {
                RCLCPP_FATAL(get_logger(),
                             "Unsupported Descriptor \" %s \" specified. Consult the README.md for supported values.",
                             gpio_pins_[i].descriptor.c_str());
                return CallbackReturn::ERROR;
            }
            // Validate GPIO pin ordering
            if (gpio_pins_[i].descriptor.compare(ValidPinDescriptors[i]) != 0) {
                RCLCPP_FATAL(get_logger(),
                             "The %zuth pin descriptor is \" %s \". \" %s \" expected. Consult the README.md for "
                             "expected ordering ",
                             i, gpio_pins_[i].descriptor.c_str(), ValidPinDescriptors[i].c_str());
                return CallbackReturn::ERROR;
            }
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

        // Try parse the interface parameters
        try {
            min_position_ = std::stod(joint.command_interfaces[0].min);
            max_position_ = std::stod(joint.command_interfaces[0].max);
            position_state_interface_name_ = joint_state_interfaces_.begin()->first;
            position_command_interface_name_ = joint_command_interfaces_.begin()->first;
            hardware_name_ = info.name;
        } catch (const std::exception& e) {
            RCLCPP_FATAL(get_logger(), "Failed to parse interface parameters: %s", e.what());
            return CallbackReturn::ERROR;
        }

        // Validate position bounds
        if (min_position_ > max_position_) {
            RCLCPP_FATAL(get_logger(), "Invalid Position bounds specified.");
            return CallbackReturn::ERROR;
        }

        // Intialize internal class variables
        dir_ = RotateDir::CW;
        angular_resolution_ = 2 * M_PI / pulses_per_rev_;  // angle in radians per pulse;
        num_pulses_ = 0;

        RCLCPP_INFO(get_logger(), "Successfully Initialized %s.", hardware_name_.c_str());
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn CL42T::on_configure(const rclcpp_lifecycle::State& previous_state) {
        (void)previous_state;

        // Try request each GPIO pin with the corresponding initial value and descriptor
        try {
            for (const GPIOPin& pin : gpio_pins_) {
                gpiod::chip chip(pin.chip_name);
                gpiod::line line = chip.get_line(pin.pin_number);
                int request_dir = pin.direction.compare("output") == 0 ? gpiod::line_request::DIRECTION_OUTPUT
                                                                       : gpiod::line_request::DIRECTION_INPUT;
                gpiod::line_request config = {pin.descriptor, request_dir, 0};
                line.request(config, pin.init_value);
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

    hardware_interface::CallbackReturn CL42T::on_activate(const rclcpp_lifecycle::State& previous_state) {
        (void)previous_state;  // suppress unused parameter warning

        // Set the direction pin and wait the minimum setup time.
        gpio_lines_[1].set_value(dir_);
        rclcpp::sleep_for(std::chrono::nanoseconds(MinDirTimeUsec * NsecPerUsec));

        RCLCPP_INFO(get_logger(), "Successfully Activated.");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn CL42T::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
        (void)previous_state;  // suppress unused parameter warning

        RCLCPP_INFO(get_logger(), "Successfully Deactivated.");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn CL42T::on_cleanup(const rclcpp_lifecycle::State& previous_state) {
        (void)previous_state;  // suppress unused parameter warning

        for (const gpiod::line& line : gpio_lines_) {
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
        new_position += (dir_ == RotateDir::CW) ? position_change : -position_change;
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
            int dir = position_change > 0 ? RotateDir::CW : RotateDir::CCW;
            if (dir_ != dir) {
                gpio_lines_[1].set_value(dir);
                rclcpp::sleep_for(std::chrono::nanoseconds(MinDirTimeUsec * NsecPerUsec));
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
            generate_pulses(gpio_lines_[0], num_pulses_, cycle_period_us);
        }

        return hardware_interface::return_type::OK;
    }

    /**
     * Generate num_pulses at the recommended duty cycle of 50%, each lasting cycle_period_us.
     */
    void CL42T::generate_pulses(gpiod::line& pul_line, int num_pulses, int cycle_period_us) {
        // Determine the pulse width from the cycle period
        int pulse_width_us = static_cast<int>(cycle_period_us * DutyCycle);

        // Ensure the pulse width satisfies the driver constraints, operate with the closest specifications if invalid
        if (pulse_width_us < MinPulseTimeUsec) {
            RCLCPP_WARN(rclcpp::get_logger(hardware_name_),
                        "Desired pulse width of %d usec does not satisfy driver time constraints. Resorting to the "
                        "driver minimum allowable pulse width.",
                        pulse_width_us);
            pulse_width_us = MinPulseTimeUsec;
        }

        // Generate pulses at the recommended duty cycle
        rclcpp::Rate toggle_rate =
            rclcpp::Rate(rclcpp::Duration(std::chrono::nanoseconds(pulse_width_us * NsecPerUsec)));
        for (int i = 0; i < num_pulses; i++) {
            pul_line.set_value(LogicalHigh);
            toggle_rate.sleep();
            pul_line.set_value(LogicalLow);
            toggle_rate.sleep();
        }
    }

}  // namespace cl42t_hardware

// Register the plugin
PLUGINLIB_EXPORT_CLASS(cl42t_hardware::CL42T, hardware_interface::ActuatorInterface)