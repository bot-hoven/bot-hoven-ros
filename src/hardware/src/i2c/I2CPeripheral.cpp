#include "hardware/i2c/I2CPeripheral.h"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <sstream>
extern "C" {
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
}
#include <system_error>

namespace hardware {
    std::shared_ptr<I2CPeripheral> I2CPeripheral::instance_ = nullptr;
    std::mutex I2CPeripheral::instance_mutex_;

    std::shared_ptr<I2CPeripheral> I2CPeripheral::getInstance(const std::string& device) {
        if (!instance_) {
            instance_ = createInstance(device);
        }
        return instance_;
    }

    std::shared_ptr<I2CPeripheral> I2CPeripheral::createInstance(const std::string& device) {
        return std::shared_ptr<I2CPeripheral>(new I2CPeripheral(device));
    }

    /**
     * @throw std::system_error if the I2C device cannot be opened.
     */
    I2CPeripheral::I2CPeripheral(const std::string& device) {
        std::lock_guard<std::mutex> lock(instance_mutex_);
        OpenBus(device);
    }

    /**
     * @throw std::system_error if the I2C device cannot be closed.
     */
    I2CPeripheral::~I2CPeripheral() {
        // Acquire a lock to make the following operations atomic (ie. thread-safe)
        std::lock_guard<std::mutex> lock(instance_mutex_);
        instance_.reset();  // Reset the shared pointer to indicate that the instance is destroyed
        CloseBus();
    }

    void I2CPeripheral::WriteRegisterByte(const uint8_t register_address, const uint8_t value) {
        i2c_smbus_data data;
        data.byte = value;
        const auto err = i2c_smbus_access(bus_fd_, I2C_SMBUS_WRITE, register_address, I2C_SMBUS_BYTE_DATA, &data);
        if (err) {
            const auto msg =
                "Could not write value (" + std::to_string(value) + ") to register " + std::to_string(register_address);
            throw std::system_error(errno, std::system_category(), msg);
        }
    }

    uint8_t I2CPeripheral::ReadRegisterByte(const uint8_t register_address) {
        i2c_smbus_data data;
        const auto err = i2c_smbus_access(bus_fd_, I2C_SMBUS_READ, register_address, I2C_SMBUS_BYTE_DATA, &data);
        if (err) {
            const auto msg = "Could not read value at register " + std::to_string(register_address);
            throw std::system_error(-err, std::system_category(), msg);
        }
        return data.byte & 0xFF;
    }

    void I2CPeripheral::OpenBus(const std::string& device) {
        bus_fd_ = open(device.c_str(), O_RDWR);
        if (bus_fd_ < 0) {
            std::ostringstream error_message;
            error_message << "Error opening I2C device: " << device_;
            throw std::system_error(errno, std::system_category(), error_message.str());
        }
    }

    void I2CPeripheral::CloseBus() {
        // Check that the bus is open before trying to close it
        if (bus_fd_ >= 0) {
            // Confirm that the bus is closed successfully
            if (close(bus_fd_) < 0) {
                std::ostringstream error_message;
                error_message << "Error closing I2C device: " << device_;
                throw std::system_error(errno, std::system_category(), error_message.str());
            }
            bus_fd_ = -1;  // Reset bus_fd to indicate it's closed
        }
    }

    /**
     * @throw std::system_error if the I2C peripheral cannot be connected to.
     */
    void I2CPeripheral::ConnectToPeripheral(const uint8_t address) {
        if (ioctl(bus_fd_, I2C_SLAVE, address) < 0) {
            std::ostringstream error_message;
            error_message << "Could not connect to I2C Peripheral with address 0x" << std::hex
                          << static_cast<int>(address);
            throw std::system_error(errno, std::system_category(), error_message.str());
        }
        current_i2c_address_ = address;
    }

    int I2CPeripheral::GetCurrentI2CAddress() { return current_i2c_address_; }

}  // namespace hardware