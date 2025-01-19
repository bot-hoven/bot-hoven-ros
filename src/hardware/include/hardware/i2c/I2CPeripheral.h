#ifndef HARDWARE__I2C__I2CPERIPHERAL_H
#define HARDWARE__I2C__I2CPERIPHERAL_H

#include <cstdint>
#include <memory>
#include <string>
#include <mutex>

namespace hardware {

    class I2CPeripheral {
    public:
        // Singleton instance accessor
        static std::shared_ptr<I2CPeripheral> getInstance(const std::string& device);

        // Destructor
        ~I2CPeripheral();

        // Public methods for I2C operations
        void WriteRegisterByte(const uint8_t register_address, const uint8_t value);
        uint8_t ReadRegisterByte(const uint8_t register_address);
        void ConnectToPeripheral(const uint8_t address);

        // Getters and setters
        int GetCurrentI2CAddress();

    private:
        // Private constructor for singleton pattern
        I2CPeripheral(const std::string& device);

        // Factory method to create an instance
        static std::shared_ptr<I2CPeripheral> createInstance(const std::string& device);

        // // Allow std::make_shared to access the private constructor
        // friend std::shared_ptr<I2CPeripheral> std::make_shared<I2CPeripheral>(const std::string&);

        // Disable copy constructor and assignment operator
        I2CPeripheral(const I2CPeripheral&) = delete;
        I2CPeripheral& operator=(const I2CPeripheral&) = delete;

        // Methods to open and close the I2C bus
        void OpenBus(const std::string& device);
        void CloseBus();

        // Singleton instance and mutex
        static std::shared_ptr<I2CPeripheral> instance_;
        static std::mutex instance_mutex_;

        // Parameters for the I2C bus
        int bus_fd;
        int current_i2c_address;
    };

}  // namespace hardware

#endif  // HARDWARE__I2C__I2CPERIPHERAL_H