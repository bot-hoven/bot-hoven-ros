#ifndef CL42T_HPP
#define CL42T_HPP

#include <algorithm>
#include <cmath>
#include <gpiod.hpp>
#include <hardware_interface/actuator_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <vector>

namespace cl42t_hardware {

    // CL42T-V4.1 interface constants, see documentation for further details:
    // https://www.omc-stepperonline.com/index.php?route=product/product/get_file&file=3272/CL42T-V41%20Manual.pdf
    constexpr uint8_t LogicalHigh = 1;
    constexpr uint8_t LogicalLow = 0;
    constexpr uint8_t MinPulseTimeUsec = 1;
    constexpr uint8_t MinDirTimeUsec = 2;
    constexpr uint32_t MinEnaTimeUsec = 200000;
    constexpr uint32_t UsecPerSec = 1000000;
    constexpr uint32_t NsecPerUsec = 1000;
    constexpr float DutyCycle = 0.5;
    const std::vector<int> ValidPulsesPerRev = {200,  800,  1600, 3200, 6400, 12800, 25600, 51200,
                                                1000, 2000, 4000, 5000, 8000, 10000, 20000, 40000};

    enum RotateDir { CW = 0, CCW = 1 };

    struct GPIOPin {
        uint8_t pin_number;      // GPIO pin number
        std::string descriptor;  // Pin description
        std::string direction;   // Input or output
        int init_value;          // Initial value to set (only relevant for output pins)
    };

    class CL42T : public hardware_interface::ActuatorInterface {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(ActuatorInterface)

        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

        hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

        hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

        hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    private:
        // Auxiliary function prototypes
        void generate_pulses(gpiod::line& pul_line, int num_pulses, int cycle_period_us, float duty_cycle);

        // Hardware parameters
        int pulses_per_rev_;
        std::string chip_name_;
        std::vector<GPIOPin> gpio_pins_;
        std::vector<gpiod::line> gpio_lines_;

        // Joint parameters
        double min_position_;
        double max_position_;

        // Interface variables
        std::string position_state_interface_name_;
        std::string position_command_interface_name_;

        // Internal variables
        int dir_;
        double angular_resolution_;
        int num_pulses_;
    };

}  // namespace cl42t_hardware

#endif  // CL42T_HPP