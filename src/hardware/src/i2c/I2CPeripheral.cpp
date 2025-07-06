#include "hardware/i2c/I2CPeripheral.h"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <sstream>
#include <vector>
extern "C" {
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
}
#include <system_error>

namespace hardware {
    std::shared_ptr<I2CPeripheral> I2CPeripheral::instance_ = nullptr;
    std::mutex I2CPeripheral::instance_mutex_;
    std::mutex I2CPeripheral::bus_mutex_;

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

    void I2CPeripheral::WriteRegisterBlock(const uint8_t starting_register, const std::vector<uint8_t>& values) {
        std::lock_guard<std::mutex> lock(bus_mutex_);
        if (values.empty()) {
            return;  // Nothing to write
        }

        // Maximum bytes to write at once (excluding register address)
        const size_t MAX_CHUNK_SIZE = 16;

        for (size_t offset = 0; offset < values.size(); offset += MAX_CHUNK_SIZE) {
            // Calculate chunk size for this iteration
            size_t chunk_size = std::min(MAX_CHUNK_SIZE, values.size() - offset);

            // Create buffer for this chunk
            std::vector<uint8_t> buffer;
            buffer.reserve(chunk_size + 1);

            // If this is the first chunk, use starting_register
            // For subsequent chunks, calculate the appropriate register address
            uint8_t reg_addr = starting_register + offset;
            buffer.push_back(reg_addr);

            // Add data for this chunk
            buffer.insert(buffer.end(), values.begin() + offset, values.begin() + offset + chunk_size);

            // Write this chunk
            if (write(bus_fd_, buffer.data(), buffer.size()) != static_cast<ssize_t>(buffer.size())) {
                std::ostringstream error_message;
                error_message << "Failed to write " << chunk_size << " bytes to register "
                              << static_cast<int>(reg_addr);
                throw std::system_error(errno, std::system_category(), error_message.str());
            }

            // Add a small delay between chunks
            usleep(1000);  // 1ms delay
        }
    }

    std::vector<uint8_t> I2CPeripheral::ReadRegisterBlock(const uint8_t starting_register, size_t count) {
        std::lock_guard<std::mutex> lock(bus_mutex_);
        if (count == 0) {
            return {};  // Nothing to read
        }

        // First, set the register pointer
        if (write(bus_fd_, &starting_register, 1) != 1) {
            std::ostringstream error_message;
            error_message << "Failed to set register pointer to " << static_cast<int>(starting_register);
            throw std::system_error(errno, std::system_category(), error_message.str());
        }

        // Now read the data
        std::vector<uint8_t> buffer(count);
        if (read(bus_fd_, buffer.data(), count) != static_cast<ssize_t>(count)) {
            std::ostringstream error_message;
            error_message << "Failed to read " << count << " bytes from register "
                          << static_cast<int>(starting_register);
            throw std::system_error(errno, std::system_category(), error_message.str());
        }

        return buffer;
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
     * @brief Attempts to recover the I2C bus from a hung state
     * 
     * This method implements multiple recovery strategies:
     * 1. Close and reopen the bus
     * 2. Use ioctl I2C_BUS_RESET if supported by the driver
     * 3. Send SMBus quick commands to each known device
     * 
     * @return true if recovery was successful, false otherwise
     */
    bool I2CPeripheral::RecoverBus() {
        std::lock_guard<std::mutex> lock(bus_mutex_);
        
        bool recovery_successful = false;
        
        // Step 1: Try SMBus quick commands to unstick devices
        if (bus_fd_ >= 0) {
            std::vector<uint8_t> known_addresses = {0x40, 0x41, 0x20, 0x21, 0x10, 0x11, 0x12, 0x13};
            
            for (auto addr : known_addresses) {
                if (ioctl(bus_fd_, I2C_SLAVE, addr) >= 0) {
                    // Try SMBus quick command (sends a single bit)
                    i2c_smbus_write_quick(bus_fd_, I2C_SMBUS_WRITE);
                    // No need to check result, just attempt for each device
                    usleep(1000);  // Small delay between attempts
                }
            }
        }
        
        // Step 2: Try ioctl I2C_BUS_RESET if supported
        if (bus_fd_ >= 0) {
            // Define I2C_BUS_RESET if not available in system headers
            #ifndef I2C_BUS_RESET
            #define I2C_BUS_RESET 0x0702
            #endif
            
            recovery_successful = (ioctl(bus_fd_, I2C_BUS_RESET, 0) >= 0);
        }
        
        // Step 3: Close and reopen the bus
        try {
            if (bus_fd_ >= 0) {
                close(bus_fd_);
                bus_fd_ = -1;
            }
            
            // Wait before reopening
            usleep(10000);  // 10ms
            
            OpenBus(device_);
            
            // Reconnect to the current device if needed
            if (current_i2c_address_ > 0) {
                ConnectToPeripheral(current_i2c_address_);
            }
            
            // Test the bus by reading a device we know exists
            // Try with one device from each PCB
            bool bus_working = false;
            
            // Test left PCB device
            if (ioctl(bus_fd_, I2C_SLAVE, 0x20) >= 0) {  // MCP23017 left
                try {
                    // Try to read IODIRA register
                    i2c_smbus_read_byte_data(bus_fd_, 0x00);
                    bus_working = true;
                } catch (...) {
                    // Ignore errors
                }
            }
            
            // If left failed, try right PCB device
            if (!bus_working && ioctl(bus_fd_, I2C_SLAVE, 0x21) >= 0) {  // MCP23017 right
                try {
                    // Try to read IODIRA register
                    i2c_smbus_read_byte_data(bus_fd_, 0x00);
                    bus_working = true;
                } catch (...) {
                    // Ignore errors
                }
            }
            
            recovery_successful = bus_working;
            
        } catch (const std::exception&) {
            recovery_successful = false;
        }
        
        return recovery_successful;
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

    void I2CPeripheral::WriteSMBusByte(const uint8_t register_address, const uint8_t value) {
        // This is actually identical to our existing implementation
        i2c_smbus_data data;
        data.byte = value;
        const auto err = i2c_smbus_access(bus_fd_, I2C_SMBUS_WRITE, register_address, I2C_SMBUS_BYTE_DATA, &data);
        if (err) {
            const auto msg =
                "Could not write SMBus value (" + std::to_string(value) + ") to register " + std::to_string(register_address);
            throw std::system_error(errno, std::system_category(), msg);
        }
    }
    
    uint8_t I2CPeripheral::ReadSMBusByte(const uint8_t register_address) {
        // This is actually identical to our existing implementation
        i2c_smbus_data data;
        const auto err = i2c_smbus_access(bus_fd_, I2C_SMBUS_READ, register_address, I2C_SMBUS_BYTE_DATA, &data);
        if (err) {
            const auto msg = "Could not read SMBus value at register " + std::to_string(register_address);
            throw std::system_error(-err, std::system_category(), msg);
        }
        return data.byte & 0xFF;
    }
    
    void I2CPeripheral::WriteSMBusBlock(const uint8_t register_address, const std::vector<uint8_t>& values) {
        if (values.empty() || values.size() > 32) {  // SMBus spec limits block transfers to 32 bytes
            const auto msg = "SMBus block write requires 1-32 bytes, got " + std::to_string(values.size());
            throw std::invalid_argument(msg);
        }
        
        // Create a C-style array for the SMBus functions
        uint8_t data[32];
        for (size_t i = 0; i < values.size(); i++) {
            data[i] = values[i];
        }
        
        // The i2c_smbus_write_block_data function handles appending the length byte
        int result = i2c_smbus_write_i2c_block_data(bus_fd_, register_address, values.size(), data);
        
        if (result < 0) {
            const auto msg = "SMBus block write failed for register " + std::to_string(register_address);
            throw std::system_error(errno, std::system_category(), msg);
        }
    }

    int I2CPeripheral::GetFileDescriptor() const { return bus_fd_; }
    
    std::vector<uint8_t> I2CPeripheral::ReadSMBusBlock(const uint8_t register_address, size_t count) {
        if (count == 0 || count > 32) {  // SMBus spec limits block transfers to 32 bytes
            const auto msg = "SMBus block read requires 1-32 bytes, got " + std::to_string(count);
            throw std::invalid_argument(msg);
        }
        
        uint8_t data[32];
        int result = i2c_smbus_read_i2c_block_data(bus_fd_, register_address, count, data);
        
        if (result < 0) {
            const auto msg = "SMBus block read failed for register " + std::to_string(register_address);
            throw std::system_error(errno, std::system_category(), msg);
        }
        
        // Copy the data to a vector
        std::vector<uint8_t> values(data, data + count);
        return values;
    }

    void I2CPeripheral::WriteRegisterBlockAlternative(const uint8_t starting_register, const std::vector<uint8_t>& values) {
        if (values.empty()) {
            return; // Nothing to write
        }
    
        // Use i2c_smbus_write_i2c_block_data which is specifically designed for block operations
        // This function requires external library linkage, so let's use a different approach:
        
        // Set up the message structures
        struct i2c_msg messages[1];
        struct i2c_rdwr_ioctl_data packets;
        
        // Create a buffer with register address + all values
        std::vector<uint8_t> buffer;
        buffer.reserve(values.size() + 1);
        buffer.push_back(starting_register);
        buffer.insert(buffer.end(), values.begin(), values.end());
        
        // Configure the message
        messages[0].addr = current_i2c_address_;
        messages[0].flags = 0;  // Write
        messages[0].len = buffer.size();
        messages[0].buf = buffer.data();
        
        // Configure the packet
        packets.msgs = messages;
        packets.nmsgs = 1;
        
        // Send the message
        if (ioctl(bus_fd_, I2C_RDWR, &packets) < 0) {
            std::ostringstream error_message;
            error_message << "Failed to write " << values.size() << " bytes to register " 
                          << static_cast<int>(starting_register);
            throw std::system_error(errno, std::system_category(), error_message.str());
        }
    }

}  // namespace hardware