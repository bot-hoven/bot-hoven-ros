#ifndef HARDWARE__PCA9685__PCA9685_SYSTEM_HPP
#define HARDWARE__PCA9685__PCA9685_SYSTEM_HPP

#include <hardware/ads7138/ads7138_comm.h>
#include <hardware/pca9685/pca9685_comm.h>

#include <memory>
#include <string>
#include <vector>

#include "hardware/ads7138/visibility_control.h"
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

    // Constants
    constexpr int MAX_WRITE_ATTEMPTS = 3;
    constexpr int WRITE_ATTEMP_DELAY_US = 100;
    constexpr int MAX_READ_ATTEMPTS = 3;
    constexpr int READ_ATTEMPT_DELAY_US = 100;
    constexpr uint32_t I2C_REREAD_DELAY_US = 100;

    struct Config {
        std::string i2c_device;
        int pca_i2c_address;
        double pca_freq_hz;
        int ads_i2c_address;
        double ads_conversion_rate;
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
        double angle_to_duty_cycle(double command, int joint_index);
        double adc_value_to_angle(uint16_t adc_value, size_t joint_index);

        // I2C parameters
        std::shared_ptr<hardware::I2CPeripheral> i2c_bus_ = nullptr;

        // Device parameters
        Config cfg_;
        pca9685_hardware_interface::PCA9685 pca_;
        ads7138_hardware_interface::ADS7138 ads_;

        // Interface parameters
        std::vector<double> min_positions_;
        std::vector<double> max_positions_;
        std::vector<double> hw_commands_;

        // Channel and calibration parameters
        std::vector<int> servo_channels_;
        std::vector<uint16_t> pwm_min_values_;
        std::vector<uint16_t> pwm_max_values_;
        std::vector<double> pwm_slopes_;
        std::vector<double> pwm_intercepts_;

        // Interface parameters
        std::vector<double> min_angles_;        // Min angle for each joint
        std::vector<double> max_angles_;        // Max angle for each joint
        std::vector<uint8_t> channel_mapping_;  // Maps joint index to ADS7138 channel
        std::vector<double> adc_slopes_;        // Calibration slope for each joint
        std::vector<double> adc_intercepts_;    // Calibration intercept for each joint
        std::vector<double> hw_states_;         // Position values for state interfaces

        // Internal variables
        std::vector<double> current_command_values_ = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
        std::mutex instance_mutex_;
        int num_write_attempts_ = 0;
        bool write_success_ = false;
        int num_read_attempts_ = 0;
        bool read_success_ = false;
    };

}  // namespace pca9685_hardware_interface

#endif  // HARDWARE__PCA9685__PCA9685_SYSTEM_HPP