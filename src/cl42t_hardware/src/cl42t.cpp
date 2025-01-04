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
            chip_name_ = std::stoi(info.hardware_parameters.at("gpio.chip_name"));

            int i = 0;
            std::string prefix = "gpio.pins." + std::to_string(i);
            // Iterate over all defined gpio pins (assumes pins are parameterized last)
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
                "Config specified and invalid setting of %d pulses per revolution. Consult the CL42T-V4.1 manual for "
                "possible settings.",
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

        // Try initialize the joint parameters
        try {
            min_position_ = std::stod(joint.parameters.at("min_position"));
            max_position_ = std::stod(joint.parameters.at("max_position"));
        } catch (const std::exception& e) {
            RCLCPP_FATAL(get_logger(), "Failed to parse joint parameters: %s", e.what());
            return CallbackReturn::ERROR;
        }

        RCLCPP_INFO(get_logger(), "Successfully Initialized %s.", info.name);
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn CL42T::on_configure(const rclcpp_lifecycle::State& previous_state) {
        (void)previous_state;
        try {
            gpiod::chip chip(chip_name_);

            // initialize each GPIO pin with the corresponding initial value and descriptor
            for (size_t i = 0; i < gpio_pins_.size(); i++) {
                gpiod::line line = chip.get_line(gpio_pins_[i].pin_number);
                int request_dir = gpio_pins_[i].direction.compare("output") == 0 ? gpiod::line_request::DIRECTION_OUTPUT
                                                                                 : gpiod::line_request::DIRECTION_INPUT;
                gpiod::line_request config = {gpio_pins_[i].descriptor, request_dir, 0};
                line.request(config, gpio_pins_[i].init_value);
                gpio_lines_.push_back(std::move(line));
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Failed to initialize GPIO communication: %s", e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }

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
        // TODO: Add code to read position from stepper motor/encoder

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type CL42T::write(const rclcpp::Time& time, const rclcpp::Duration& period) {
        (void)time;    // suppress unused parameter warning
        (void)period;  // suppress unused parameter warning
        // TODO: Write commanded position to command stepper motor

        return hardware_interface::return_type::OK;
    }

}  // namespace cl42t_hardware

// Register the plugin
PLUGINLIB_EXPORT_CLASS(cl42t_hardware::CL42T, hardware_interface::ActuatorInterface)