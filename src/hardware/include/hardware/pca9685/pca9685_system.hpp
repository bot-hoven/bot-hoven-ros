#ifndef HARDWARE__PCA9685__PCA9685_SYSTEM_HPP
#define HARDWARE__PCA9685__PCA9685_SYSTEM_HPP

#include <hardware/pca9685/pca9685_comm.h>

#include <memory>
#include <string>
#include <vector>

#include "hardware/pca9685/visibility_control.h"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace pca9685_hardware_interface {

    struct Config {
        std::string i2c_device;
        int i2c_address;
        double freq_hz;
        double min_duty_cycle;
        double max_duty_cycle; 
    };

    class Pca9685SystemHardware : public hardware_interface::SystemInterface {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(Pca9685SystemHardware)

        PCA9685_HARDWARE_INTERFACE_PUBLIC
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

        PCA9685_HARDWARE_INTERFACE_PUBLIC
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        PCA9685_HARDWARE_INTERFACE_PUBLIC
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        PCA9685_HARDWARE_INTERFACE_PUBLIC
        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

        PCA9685_HARDWARE_INTERFACE_PUBLIC
        hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

        PCA9685_HARDWARE_INTERFACE_PUBLIC
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

        PCA9685_HARDWARE_INTERFACE_PUBLIC
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

        PCA9685_HARDWARE_INTERFACE_PUBLIC
        hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

        PCA9685_HARDWARE_INTERFACE_PUBLIC
        hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    private:
        // Auxiliary function prototypes
        double command_to_duty_cycle(double command, double min_input, double max_input, double min_duty_cycle,
                                     double max_duty_cycle);

        // I2C parameters
        std::shared_ptr<hardware::I2CPeripheral> i2c_bus_ = nullptr;

        // Device parameters
        Config cfg_;
        pca9685_hardware_interface::PCA9685 pca_;

        // Interface parameters
        std::vector<double> min_positions_;
        std::vector<double> max_positions_;
        std::vector<double> hw_commands_;

        // Internal variables
        std::vector<double> current_pwm_values_ = {0.5, 0.5, 0.5, 0.5, 0.5};
        std::mutex instance_mutex_;
    };

}  // namespace pca9685_hardware_interface

#endif  // HARDWARE__PCA9685__PCA9685_SYSTEM_HPP