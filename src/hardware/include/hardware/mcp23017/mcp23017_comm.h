#ifndef HARDWARE__MCP23017__MCP23017_COMM_H
#define HARDWARE__MCP23017__MCP23017_COMM_H

#include <memory>
#include <string>

#include "hardware/i2c/I2CPeripheral.h"
#include "hardware/mcp23017/mcp23017_constants.h"

namespace mcp23017_hardware_interface {

    class MCP23017 {
    public:
        MCP23017() = default;
        ~MCP23017();

        void setup(std::shared_ptr<hardware::I2CPeripheral> i2c_bus, const int i2c_address);

        void connect();

        void disconnect();

        bool connected() const;

        void init();

        void set_gpio_state(uint8_t gpio_value_);

    private:
        bool is_connected = false;
        std::shared_ptr<hardware::I2CPeripheral> i2c_dev;
        int address = 0x20;
    };

}  // namespace mcp23017_hardware_interface

#endif  // HARDWARE__MCP23017__MCP23017_COMM_H