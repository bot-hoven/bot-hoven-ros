#include "hardware/i2c/I2CPeripheral.h"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <iostream>
extern "C" {
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
}
#include <system_error>

namespace hardware {
    std::shared_ptr<I2CPeripheral> I2CPeripheral::instance_ = nullptr;

    std::shared_ptr<I2CPeripheral> I2CPeripheral::getInstance(const std::string& device) {
        if (!instance_) {
            instance_ = std::shared_ptr<I2CPeripheral>(new I2CPeripheral(device));
        }
        return instance_;
    }

    I2CPeripheral::I2CPeripheral(const std::string& device) {
        OpenBus(device);
        // ConnectToPeripheral(address);
    }

    I2CPeripheral::~I2CPeripheral() {
        try {
            CloseBus();
        } catch (const std::exception& e) {
            std::cerr << "Error while closing I2C bus: " << e.what() << std::endl;
        }
    }

    void I2CPeripheral::WriteRegisterByte(const uint8_t register_address, const uint8_t value) {
        i2c_smbus_data data;
        data.byte = value;
        const auto err = i2c_smbus_access(bus_fd, I2C_SMBUS_WRITE, register_address, I2C_SMBUS_BYTE_DATA, &data);
        if (err) {
            const auto msg =
                "Could not write value (" + std::to_string(value) + ") to register " + std::to_string(register_address);
            throw std::system_error(errno, std::system_category(), msg);
        }
    }

    uint8_t I2CPeripheral::ReadRegisterByte(const uint8_t register_address) {
        i2c_smbus_data data;
        const auto err = i2c_smbus_access(bus_fd, I2C_SMBUS_READ, register_address, I2C_SMBUS_BYTE_DATA, &data);
        if (err) {
            const auto msg = "Could not read value at register " + std::to_string(register_address);
            throw std::system_error(-err, std::system_category(), msg);
        }
        return data.byte & 0xFF;
    }

    void I2CPeripheral::OpenBus(const std::string& device) {
        bus_fd = open(device.c_str(), O_RDWR);
        if (bus_fd < 0) {
            throw std::system_error(errno, std::system_category(), "Could not open i2c bus.");
        }
    }

    void I2CPeripheral::CloseBus() {
        if (bus_fd >= 0) {
            if (close(bus_fd) < 0) {
                throw std::system_error(errno, std::system_category(), "Could not close i2c bus.");
            }
            bus_fd = -1; // Reset bus_fd to indicate it's closed
        }
    }


    void I2CPeripheral::ConnectToPeripheral(const uint8_t address) {
        if (ioctl(bus_fd, I2C_SLAVE, address) < 0) {
            throw std::system_error(errno, std::system_category(), "Could not set peripheral address.");
        }
        current_i2c_address = address;
    }

    int I2CPeripheral::GetCurrentI2CAddress() {
        return current_i2c_address;
    }

}  // namespace hardware