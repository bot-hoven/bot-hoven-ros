#include "cl42t_hardware/cl42t_hardware/cl42t.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace cl42t_hardware {

    hardware_interface::CallbackReturn CL42T::on_init(const hardware_interface::HardwareInfo& info) {
        if (hardware_interface::ActuatorInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // // Initialize parameters from URDF/config
        // try {
        //     pul_pin_ = std::stoi(info.hardware_parameters["pul_pin"]);
        //     dir_pin_ = std::stoi(info.hardware_parameters["dir_pin"]);
        //     ena_pin_ = std::stoi(info.hardware_parameters["ena_pin"]);
        //     pulses_per_rev_ = std::stoi(info.hardware_parameters["pulses_per_rev"]);
        // } catch (const std::exception& e) {
        //     RCLCPP_ERROR(rclcpp::get_logger(), "Failed to parse hardware parameters: %s", e.what());
        //     return hardware_interface::CallbackReturn::ERROR;
        // }

        const hardware_interface::ComponentInfo& joint = info.joints[0];

        // Validate state interface
        if (joint.state_interfaces.size() != 1) {
            RCLCPP_FATAL(get_logger(), "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
                         joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Validate command interface
        if (joint.command_interfaces.size() != 1) {
            RCLCPP_FATAL(get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                         joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn CL42T::on_configure(const rclcpp_lifecycle::State& previous_state) {
        (void)previous_state;
        // TODO: Add hardware initialization. Set up GPIO pins, initialize communication, etc.

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn CL42T::on_cleanup(const rclcpp_lifecycle::State& previous_state) {
        (void)previous_state;
        // TODO: Add cleanup code. Release GPIO pins, close communication, etc.

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