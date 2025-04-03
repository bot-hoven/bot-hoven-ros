#include "hardware/ads7138/ads7138_comm.h"

#include <unistd.h>

#include <chrono>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace ads7138_hardware_interface {

    ADS7138::~ADS7138() = default;

    void ADS7138::setup(std::shared_ptr<hardware::I2CPeripheral> i2c_bus, const int i2c_address) {
        i2c_dev = i2c_bus;
        address = i2c_address;
    }

    /**
     * @throw std::system_error if the I2C peripheral cannot be connected to.
     */
    void ADS7138::connect() {
        if (i2c_dev->GetCurrentI2CAddress() != address) {
            i2c_dev->ConnectToPeripheral(address);
        }
    }

    /**
     * @throw std::system_error if an error occurs during the I2C communication.
     */
    void ADS7138::init() {
        // // Reset the device for a clean start
        // reset();
        // usleep(5000);  // Extended delay after reset (5ms)
        
        // Try to read device ID (should be 0x10)
        uint8_t id = i2c_dev->ReadRegisterByte(ADS7138_REG_DEVICE_ID);
    
    }

    /**
     * @throw std::system_error if an error occurs during the I2C communication.
     */
    void ADS7138::reset() { i2c_dev->WriteRegisterByte(ADS7138_REG_RESET, ADS7138_RESET_MASK); }

    /**
     * @throw std::system_error if an error occurs during the I2C communication.
     * @throw std::out_of_range if the channel is invalid.
     */
    uint16_t ADS7138::read_channel(uint8_t channel) {
        if (channel > 7) {
            throw std::out_of_range("ADS7138 channel must be between 0 and 7");
        }
        
        // Select the channel to read
        i2c_dev->WriteRegisterByte(ADS7138_REG_MANUAL_CH_SEL, channel);
        usleep(500);  // Increased delay (500μs) for conversion
        
        // Read MSB register first (0x10)
        uint8_t msb = i2c_dev->ReadRegisterByte(ADS7138_REG_MANUAL_DATA_0);
        usleep(100);  // Small delay between reads
        
        // Then read LSB register (0x11)
        uint8_t lsb = i2c_dev->ReadRegisterByte(ADS7138_REG_MANUAL_DATA_1);
        
        // Combine into 16-bit value (12-bit ADC result in the upper 12 bits)
        uint16_t result = (static_cast<uint16_t>(msb) << 8) | lsb;
        
        return result;
    }

    /**
     * @throw std::system_error if an error occurs during the I2C communication.
     * @throw std::out_of_range if any channel is invalid.
     */
    std::vector<uint16_t> ADS7138::read_channels(const std::vector<uint8_t>& channels) {
        std::vector<uint16_t> results;
        results.reserve(channels.size());

        for (auto channel : channels) {
            results.push_back(read_channel(channel));
        }

        return results;
    }

    /**
     * @throw std::system_error if an error occurs during the I2C communication.
     */
    void ADS7138::config_auto_sequence(uint8_t channel_mask, bool continuous) {
        // Set which channels to include in the sequence
        i2c_dev->WriteRegisterByte(ADS7138_REG_AUTO_SEQ_CH, channel_mask);

        // Configure auto-sequence mode (with or without continuous conversion)
        uint8_t as_ctrl = continuous ? ADS7138_AS_CONT_MASK : 0;
        i2c_dev->WriteRegisterByte(ADS7138_REG_AUTO_SEQ_CTRL, as_ctrl);
    }

    /**
     * @throw std::system_error if an error occurs during the I2C communication.
     */
    void ADS7138::start_auto_sequence() {
        // Read current control register value
        uint8_t ctrl_val = i2c_dev->ReadRegisterByte(ADS7138_REG_AUTO_SEQ_CTRL);

        // Set the START bit while preserving other bits
        ctrl_val |= ADS7138_AS_START_MASK;

        // Write back to start the sequence
        i2c_dev->WriteRegisterByte(ADS7138_REG_AUTO_SEQ_CTRL, ctrl_val);
    }

    /**
     * @throw std::system_error if an error occurs during the I2C communication.
     */
    std::array<uint16_t, 8> ADS7138::read_auto_sequence_results() {
        std::array<uint16_t, 8> results;

        // Read results for all 8 channels from their data registers
        for (int i = 0; i < 8; i++) {
            uint8_t msb_reg = ADS7138_REG_CH0_DATA_0 + (i * 2);
            results[i] = read_16bit_value(msb_reg);
        }

        return results;
    }

    /**
     * @throw std::system_error if an error occurs during the I2C communication.
     */
    void ADS7138::set_conversion_rate(uint8_t rate) {
        if (rate > ADS7138_CONV_RATE_200K) {
            throw std::out_of_range("Invalid conversion rate");
        }

        i2c_dev->WriteRegisterByte(ADS7138_REG_CONV_RATE, rate);
    }

    // /**
    //  * @throw std::system_error if an error occurs during the I2C communication.
    //  */
    // uint8_t ADS7138::read_device_id() { return i2c_dev->ReadRegisterByte(ADS7138_REG_DEVICE_ID); }

    /**
     * @brief Read a 16-bit value from two consecutive registers.
     * @throw std::system_error if an error occurs during the I2C communication.
     */
    uint16_t ADS7138::read_16bit_value(uint8_t msb_reg) {
        // Read MSB and LSB from consecutive registers
        uint8_t msb = i2c_dev->ReadRegisterByte(msb_reg);
        uint8_t lsb = i2c_dev->ReadRegisterByte(msb_reg + 1);

        // Combine into 16-bit value (12-bit ADC result in the upper 12 bits)
        return (static_cast<uint16_t>(msb) << 8) | lsb;
    }

    /**
     * @brief Read multiple channels more efficiently using block reading
     *
     * @param channels Vector of channels to read (0-7)
     * @return Vector of 12-bit ADC values corresponding to each channel
     * @throw std::system_error if an error occurs during the I2C communication
     * @throw std::out_of_range if any channel is invalid
     */
    std::vector<uint16_t> ADS7138::read_channels_block(const std::vector<uint8_t>& channels) {
        std::vector<uint16_t> results;
        results.reserve(channels.size());

        if (channels.empty()) {
            return results;
        }

        // Validate all channels
        for (auto channel : channels) {
            if (channel > 7) {
                throw std::out_of_range("ADS7138 channel must be between 0 and 7");
            }
        }

        try {
            // Approach 1: Set up auto-sequence mode for all channels at once
            uint8_t channel_mask = 0;
            for (auto channel : channels) {
                channel_mask |= (1 << channel);
            }

            // Configure auto-sequence
            config_auto_sequence(channel_mask, false);  // false = single sequence, not continuous
            start_auto_sequence();

            // Read all results
            auto all_results = read_auto_sequence_results();

            // Extract the values for our requested channels
            for (auto channel : channels) {
                results.push_back(all_results[channel]);
            }
        } catch (const std::exception& e) {
            std::ostringstream error_message;
            error_message << "Failed to perform block read: " << e.what();
            throw std::runtime_error(error_message.str());
        }

        return results;
    }

    /**
     * @brief Set up and read all channels in auto-sequence mode
     *
     * @param mask Bit mask of channels to include (bit 0 for CH0, bit 1 for CH1, etc.)
     * @return Vector of ADC values for all enabled channels
     * @throw std::system_error if an error occurs during the I2C communication
     */
    std::vector<uint16_t> ADS7138::read_auto_sequence(uint8_t mask) {
        std::vector<uint16_t> results;
        results.reserve(8);  // Maximum possible channels

        try {
            // Configure auto-sequence
            config_auto_sequence(mask, false);  // false = single sequence, not continuous
            start_auto_sequence();

            // Read all results
            auto all_results = read_auto_sequence_results();

            // Extract only the enabled channels
            for (int i = 0; i < 8; i++) {
                if (mask & (1 << i)) {
                    results.push_back(all_results[i]);
                }
            }
        } catch (const std::exception& e) {
            std::ostringstream error_message;
            error_message << "Failed to read auto-sequence: " << e.what();
            throw std::runtime_error(error_message.str());
        }

        return results;
    }

}  // namespace ads7138_hardware_interface