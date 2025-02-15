#ifndef HARDWARE__CL42T__CL42T_COMM_HPP
#define HARDWARE__CL42T__CL42T_COMM_HPP

#include <hardware/cl42t/cl42t_comm.h>
#include "hardware/cl42t/visibility_control.h"
#include <iostream>
#include <gpiod.hpp>
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace cl42t_hardware_interface {

    // CL42T-V4.1 constants
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
    const std::vector<std::string> ValidPinDescriptors = {"PUL+", "DIR+", "ENA+", "ALM"};

    enum RotateDir { CW = 0, CCW = 1 };

    struct GPIOPin {
        std::string chip_name;
        int pin_number;
        std::string descriptor;
        std::string direction;
        int init_value;
    };

    struct Config {
        std::string spi_device_;
        int chip_select_;
        int bus_speed_hz_;
        int bits_per_word_;
        std::string stepper_side_;
    };

    class Cl42tSystemHardware : public hardware_interface::SystemInterface {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(Cl42tSystemHardware)

        CL42T_HARDWARE_INTERFACE_PUBLIC
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

        // CL42T_HARDWARE_INTERFACE_PUBLIC
        // std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        // CL42T_HARDWARE_INTERFACE_PUBLIC
        // std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        CL42T_HARDWARE_INTERFACE_PUBLIC
        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

        CL42T_HARDWARE_INTERFACE_PUBLIC
        hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

        CL42T_HARDWARE_INTERFACE_PUBLIC
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

        CL42T_HARDWARE_INTERFACE_PUBLIC
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

        CL42T_HARDWARE_INTERFACE_PUBLIC
        hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

        CL42T_HARDWARE_INTERFACE_PUBLIC
        hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    private:
        // Auxiliary function prototypes
        void generate_pulses(gpiod::line& pul_line, int num_pulses, int cycle_period_us);

        // Hardware parameters
        int pulses_per_rev_;

        // Communication parameters
        Config cfg_; 
        cl42t_hardware_interface::CL42TComm comm_;
        
        hardware::SPIPeripheral * spi_peripheral_;

        // GPIO parameters
        std::vector<GPIOPin> gpio_pins_;
        std::vector<gpiod::line> gpio_lines_;

        // Interface parameters
        double min_position_;
        double max_position_;
        std::string position_state_interface_name_;
        std::string position_command_interface_name_;

        // Internal variables
        int dir_;
        double angular_resolution_;
        int num_pulses_;
    };

}  // namespace cl42t_hardware_interface

#endif  // HARDWARE__CL42T__CL42T_COMM_HPP