#ifndef HARDWARE__I2C__I2CPERIPHERAL_H
#define HARDWARE__I2C__I2CPERIPHERAL_H

#include <cstdint>
#include <string>

namespace hardware {

    class I2CPeripheral {
    public:
        I2CPeripheral(const std::string& device);
        ~I2CPeripheral();

        void WriteRegisterByte(const uint8_t register_address, const uint8_t value);

        uint8_t ReadRegisterByte(const uint8_t register_address);

        void ConnectToPeripheral(const uint8_t address);

        int GetCurrentI2CAddress();

    private:
        int bus_fd;
        int current_i2c_address;

        void OpenBus(const std::string& device);
        void CloseBus();
    };

}  // namespace hardware

#endif  // HARDWARE__I2C__I2CPERIPHERAL_H