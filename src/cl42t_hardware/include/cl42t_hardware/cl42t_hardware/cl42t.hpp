#ifndef CL42T_HPP
#define CL42T_HPP

#include <hardware_interface/actuator_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

namespace cl42t_hardware {
    class CL42T : public hardware_interface::ActuatorInterface {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(ActuatorInterface)

        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

        hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

        hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

        hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    private:
        // Hardware parameters
        uint8_t pul_pin_;
        uint8_t dir_pin_;
        uint8_t ena_pin_;
        uint32_t pulses_per_rev_;

        // Interface variables
        double position_;
        double position_command_;
    };

}  // namespace cl42t_hardware

#endif  // CL42T_HPP