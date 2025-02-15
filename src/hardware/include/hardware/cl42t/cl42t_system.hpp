#ifndef HARDWARE__CL42T__CL42T_SYSTEM_HPP
#define HARDWARE__CL42T__CL42T_SYSTEM_HPP

#include "hardware/cl42t/cl42t_comm.h"
#include "hardware/cl42t/visibility_control.h"
#include "hardware/spi/SPIPeripheral.h"

#include <memory>
#include <string>
#include <vector>

// Include only headers needed for declarations.
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

/**
 * @brief ROS2 control hardware interface for the CL42T stepper motor driver.
 *
 * This class implements the SystemInterface to manage SPI communication with 
 * the CL42T stepper motor driver.
 */
namespace cl42t_hardware_interface {

struct Config {
    std::string spi_device_;
    int bus_speed_hz_;
    int bits_per_word_;
    std::string stepper_side_;
};

class Cl42tSystemHardware : public hardware_interface::SystemInterface {
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(Cl42tSystemHardware)

    /**
     * @brief Initializes the hardware interface.
     *
     * Parses configuration parameters and validates the hardware interfaces.
     *
     * @param info Hardware configuration information.
     * @return CallbackReturn::SUCCESS if successful.
     */
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

    /**
     * @brief Configures the hardware.
     *
     * Sets up the SPI peripheral and communication interface.
     *
     * @param previous_state Previous lifecycle state.
     * @return CallbackReturn::SUCCESS if successful.
     */
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

    /**
     * @brief Cleans up the hardware interface.
     *
     * Releases allocated resources.
     *
     * @param previous_state Previous lifecycle state.
     * @return CallbackReturn::SUCCESS if successful.
     */
    hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

    /**
     * @brief Activates the hardware interface.
     *
     * Prepares the system for operation.
     *
     * @param previous_state Previous lifecycle state.
     * @return CallbackReturn::SUCCESS if successful.
     */
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

    /**
     * @brief Deactivates the hardware interface.
     *
     * Stops hardware operations.
     *
     * @param previous_state Previous lifecycle state.
     * @return CallbackReturn::SUCCESS if successful.
     */
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    /**
     * @brief Reads the hardware state.
     *
     * @param time Current time.
     * @param period Duration of the cycle.
     * @return return_type::OK if reading is successful.
     */
    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    /**
     * @brief Writes commands to the hardware.
     *
     * Sends commands via SPI.
     *
     * @param time Current time.
     * @param period Duration of the cycle.
     * @return return_type::OK if writing is successful.
     */
    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    // Helper function if needed (e.g., generate pulses)
    void generate_pulses(/* parameters if required */);

    int pulses_per_rev_;
    Config cfg_;
    CL42TComm comm_;
    hardware::SPIPeripheral* spi_peripheral_;

    // Interface parameters (simplified example)
    double min_position_;
    double max_position_;
    std::string position_state_interface_name_;
    std::string position_command_interface_name_;

    int dir_;
    double angular_resolution_;
    int num_pulses_;

    bool command_sent_ = false;
    std::chrono::steady_clock::time_point last_command_time_;
};

}  // namespace cl42t_hardware_interface

#endif  // HARDWARE__CL42T__CL42T_SYSTEM_HPP
