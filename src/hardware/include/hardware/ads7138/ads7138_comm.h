#ifndef HARDWARE__ADS7138__ADS7138_COMM_H
#define HARDWARE__ADS7138__ADS7138_COMM_H

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "hardware/ads7138/ads7138_constants.h"
#include "hardware/i2c/I2CPeripheral.h"

namespace ads7138_hardware_interface {

    /**
     * @brief Communication interface for the ADS7138 12-bit 8-channel ADC.
     *
     * This class provides methods to interact with the ADS7138 ADC via I2C.
     * It supports both manual channel reading and auto-sequence mode.
     */
    class ADS7138 {
    public:
        /**
         * @brief Default constructor.
         */
        ADS7138() = default;

        /**
         * @brief Destructor.
         */
        ~ADS7138();

        /**
         * @brief Set up the communication interface using an I2C peripheral.
         *
         * @param i2c_bus Shared pointer to the I2C peripheral.
         * @param i2c_address The I2C address of the ADS7138 device.
         */
        void setup(std::shared_ptr<hardware::I2CPeripheral> i2c_bus, const int i2c_address = ADS7138_ADDR_GND);

        /**
         * @brief Connect to the ADS7138 device.
         *
         * @throw std::system_error if the I2C peripheral cannot be connected to.
         */
        void connect();

        /**
         * @brief Initialize the ADS7138 device.
         *
         * This method configures the device for operation in manual mode by default.
         * It sets up the conversion rate and other default parameters.
         *
         * @throw std::system_error if an error occurs during the I2C communication.
         */
        void init();

        /**
         * @brief Reset the ADS7138 device.
         *
         * @throw std::system_error if an error occurs during the I2C communication.
         */
        void reset();

        /**
         * @brief Read a single channel in manual mode.
         *
         * @param channel The channel to read (0-7).
         * @return The 12-bit ADC value.
         * @throw std::system_error if an error occurs during the I2C communication.
         * @throw std::out_of_range if the channel is invalid.
         */
        uint16_t read_channel(uint8_t channel);

        /**
         * @brief Read multiple channels in manual mode.
         *
         * @param channels Vector of channels to read (0-7).
         * @return Vector of 12-bit ADC values corresponding to each channel.
         * @throw std::system_error if an error occurs during the I2C communication.
         * @throw std::out_of_range if any channel is invalid.
         */
        std::vector<uint16_t> read_channels(const std::vector<uint8_t>& channels);

        /**
         * @brief Configure auto-sequence mode.
         *
         * @param channel_mask Bit mask of channels to include in auto-sequence (bit 0 for CH0, bit 1 for CH1, etc.).
         * @param continuous If true, continuously convert all enabled channels; if false, perform a single sequence.
         * @throw std::system_error if an error occurs during the I2C communication.
         */
        void config_auto_sequence(uint8_t channel_mask, bool continuous = true);

        /**
         * @brief Start auto-sequence conversion.
         *
         * @throw std::system_error if an error occurs during the I2C communication.
         */
        void start_auto_sequence();

        /**
         * @brief Read results from auto-sequence mode.
         *
         * @return Array of 8 ADC values, with values for enabled channels.
         * @throw std::system_error if an error occurs during the I2C communication.
         */
        std::array<uint16_t, 8> read_auto_sequence_results();

        /**
         * @brief Set the conversion rate.
         *
         * @param rate Conversion rate code (see ADS7138_CONV_RATE_* constants).
         * @throw std::system_error if an error occurs during the I2C communication.
         */
        void set_conversion_rate(uint8_t rate);

        /**
         * @brief Read the device ID.
         *
         * @return The device ID value.
         * @throw std::system_error if an error occurs during the I2C communication.
         */
        // uint8_t read_device_id();
        std::vector<uint16_t> read_channels_block(const std::vector<uint8_t>& channels);
        std::vector<uint16_t> read_auto_sequence(uint8_t mask);

    private:
        std::shared_ptr<hardware::I2CPeripheral> i2c_dev;
        int address = ADS7138_ADDR_GND;

        /**
         * @brief Read a 16-bit value from two consecutive registers.
         *
         * @param msb_reg The register address for the MSB.
         * @return The 16-bit value.
         * @throw std::system_error if an error occurs during the I2C communication.
         */
        uint16_t read_16bit_value(uint8_t msb_reg);
    };

}  // namespace ads7138_hardware_interface

#endif  // HARDWARE__ADS7138__ADS7138_COMM_H