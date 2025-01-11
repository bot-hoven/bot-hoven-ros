#ifndef HARDWARE__MCP23017__MCP23017_SYSTEM_HPP
#define HARDWARE__MCP23017__MCP23017_SYSTEM_HPP

#include <hardware/mcp23017/mcp23017_comm.h>

#include <memory>
#include <string>
#include <vector>

#include "hardware/mcp23017/visibility_control.h"
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

namespace mcp23017_hardware_interface {
    class Mcp23017SystemHardware : public hardware_interface::SystemInterface {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(Mcp23017SystemHardware);

        MCP23017_HARDWARE_INTERFACE_PUBLIC
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

        MCP23017_HARDWARE_INTERFACE_PUBLIC
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        MCP23017_HARDWARE_INTERFACE_PUBLIC
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        MCP23017_HARDWARE_INTERFACE_PUBLIC
        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

        MCP23017_HARDWARE_INTERFACE_PUBLIC
        hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

        MCP23017_HARDWARE_INTERFACE_PUBLIC
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

        MCP23017_HARDWARE_INTERFACE_PUBLIC
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

        MCP23017_HARDWARE_INTERFACE_PUBLIC
        hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

        MCP23017_HARDWARE_INTERFACE_PUBLIC
        hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    private:
        std::vector<double> hw_commands_;
        std::shared_ptr<hardware::I2CPeripheral> i2c_bus;
        std::unique_ptr<mcp23017_hardware_interface::MCP23017> mcp;
    };

}  // namespace mcp23017_hardware_interface

#endif  // HARDWARE__MCP23017__MCP23017_SYSTEM_HPP