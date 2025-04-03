#ifndef HARDWARE__I2C__I2CPERIPHERAL_H
#define HARDWARE__I2C__I2CPERIPHERAL_H

#include <cstdint>
#include <memory>
#include <string>
#include <mutex>
#include <vector>

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
        void WriteRegisterBlock(const uint8_t starting_register, const std::vector<uint8_t>& values);
        std::vector<uint8_t> ReadRegisterBlock(const uint8_t starting_register, size_t count);
        void WriteRegisterBlockAlternative(const uint8_t starting_register, const std::vector<uint8_t>& values);
        
        // SMBus operations
        void WriteSMBusByte(const uint8_t register_address, const uint8_t value);
        uint8_t ReadSMBusByte(const uint8_t register_address);
        void WriteSMBusBlock(const uint8_t register_address, const std::vector<uint8_t>& values);
        std::vector<uint8_t> ReadSMBusBlock(const uint8_t register_address, size_t count);

        // Getters and setters
        int GetCurrentI2CAddress();
        int GetFileDescriptor() const;  // New method to access file descriptor

    private:
        // Private constructor for singleton pattern
        I2CPeripheral(const std::string& device);

        // Factory method to create an instance
        static std::shared_ptr<I2CPeripheral> createInstance(const std::string& device);

        // Disable copy constructor and assignment operator
        I2CPeripheral(const I2CPeripheral&) = delete;
        I2CPeripheral& operator=(const I2CPeripheral&) = delete;

        // Methods to open and close the I2C bus
        void OpenBus(const std::string& device);
        void CloseBus();
        void RecoverBus();

        // Singleton instance and mutex
        static std::shared_ptr<I2CPeripheral> instance_;
        static std::mutex instance_mutex_;
        static std::mutex bus_mutex_;

        // Parameters for the I2C bus
        std::string device_;
        int bus_fd_;
        int current_i2c_address_;
    };

}  // namespace hardware

#endif  // HARDWARE__I2C__I2CPERIPHERAL_H