#include "hardware/pca9685/pca9685_comm.h"

#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <thread>

namespace pca9685_hardware_interface {

    PCA9685::~PCA9685() = default;

    void PCA9685::setup(std::shared_ptr<hardware::I2CPeripheral> i2c_bus, const int i2c_address) {
        i2c_dev = i2c_bus;
        address = i2c_address;
    }

    /**
     * @throw std::system_error if the I2C peripheral cannot be connected to.
     */
    void PCA9685::connect() {
        if (i2c_dev->GetCurrentI2CAddress() != address) {
            i2c_dev->ConnectToPeripheral(address);
        }
    }

    /**
     * @throw std::system_error if an error occurs during the I2C communication.
     */
    // void PCA9685::init() {
    //     set_all_pwm(0, 0);

    //     // Set component to totem pole structure and enable auto-increment
    //     i2c_dev->WriteRegisterByte(MODE2, OUTDRV | OCH);

    //     // PCA9685 responds to LED All Call I2C-bus address (0x70) and enable auto-increment
    //     i2c_dev->WriteRegisterByte(MODE1, ALLCALL | AI);
    //     usleep(INIT_SLEEP_DELAY_US);

    //     auto mode1_val = i2c_dev->ReadRegisterByte(MODE1);

    //     // After the oscillator is stable, the SLEEP bit can be set to 0, component starts in sleep mode by default
    //     mode1_val &= ~SLEEP;

    //     i2c_dev->WriteRegisterByte(MODE1, mode1_val);
    //     usleep(INIT_SLEEP_DELAY_US);
    // }

    void PCA9685::init() {
        // Simple reset first
        try {
            // Set all outputs off
            i2c_dev->WriteRegisterByte(ALL_LED_OFF_H, 0x10);

            // Reset MODE1 register to defaults
            i2c_dev->WriteRegisterByte(MODE1, 0x00);
            usleep(5000);

            // Now continue with normal init
            set_all_pwm(0, 0);
            i2c_dev->WriteRegisterByte(MODE2, OUTDRV | OCH);
            i2c_dev->WriteRegisterByte(MODE1, ALLCALL);
            usleep(INIT_SLEEP_DELAY_US);

            auto mode1_val = i2c_dev->ReadRegisterByte(MODE1);
            mode1_val &= ~SLEEP;
            i2c_dev->WriteRegisterByte(MODE1, mode1_val);
            usleep(INIT_SLEEP_DELAY_US);
        } catch (const std::exception& e) {
            // std::cerr << "Error in PCA9685 init: " << e.what() << std::endl;
            throw;
        }
    }

    /**
     * This sets the frequency of the PWM output modulation.
     * From the datasheet the formula for determining the prescale value is
     * prescale_value = round(osc_clock / (4096 * frequency)) - 1
     *
     * @throw std::system_error if an error occurs during the I2C communication.
     */
    // void PCA9685::set_pwm_freq(const double freq_hz) {
    //     frequency = freq_hz;

    //     auto prescaleval = OSCILLATOR_CLOCK_SPEED;
    //     prescaleval /= BIT_RESOLUTION * 1.0;
    //     prescaleval /= freq_hz;
    //     prescaleval -= 1.0;

    //     auto prescale = static_cast<int>(std::round(prescaleval));

    //     // Get the current mode1 register value
    //     const auto oldmode = i2c_dev->ReadRegisterByte(MODE1);

    //     // Ensure the RESTART bit is set to 0 and SLEEP is set to 1
    //     // as prescale can only be updated in sleep mode
    //     auto newmode = (oldmode & 0x7F) | SLEEP;

    //     // Put the PCA in sleep mode
    //     i2c_dev->WriteRegisterByte(MODE1, newmode);

    //     // Write the prescale value
    //     i2c_dev->WriteRegisterByte(PRE_SCALE, prescale);

    //     // Restore the original state of mode1 register
    //     i2c_dev->WriteRegisterByte(MODE1, oldmode);

    //     // Wait for oscillator to stabilize before restarting the PCA
    //     usleep(INIT_SLEEP_DELAY_US);
    //     i2c_dev->WriteRegisterByte(MODE1, oldmode | RESTART);
    // }

    void PCA9685::set_pwm_freq(const double freq_hz) {
        // First ensure we're connected
        connect();

        frequency = freq_hz;

        // std::cout << "Setting PWM frequency to " << freq_hz << " Hz" << std::endl;

        // Calculate prescale value
        auto prescaleval = OSCILLATOR_CLOCK_SPEED;
        prescaleval /= BIT_RESOLUTION * 1.0;
        prescaleval /= freq_hz;
        prescaleval -= 1.0;

        auto prescale = static_cast<uint8_t>(std::round(prescaleval));
        // std::cout << "Calculated prescale value: " << static_cast<int>(prescale) << std::endl;

        try {
            // Get the current mode1 register value
            // std::cout << "Reading MODE1 register..." << std::endl;
            const auto oldmode = i2c_dev->ReadRegisterByte(MODE1);
            // std::cout << "Current MODE1 value: 0x" << std::hex << static_cast<int>(oldmode) << std::dec << std::endl;

            // Prepare sleep mode: SLEEP bit set to 1
            auto newmode = (oldmode & 0x7F) | SLEEP;
            // std::cout << "Setting device to sleep mode..." << std::endl;

            // Put the PCA in sleep mode
            i2c_dev->WriteRegisterByte(MODE1, newmode);
            // std::cout << "Sleep mode set, MODE1: 0x" << std::hex << static_cast<int>(newmode) << std::dec << std::endl;

            // Add a small delay to ensure the device is in sleep mode
            usleep(5000);

            // Write the prescale value
            // std::cout << "Writing prescale value " << static_cast<int>(prescale) << " to register 0x" << std::hex
                    //   << static_cast<int>(PRESCALE) << std::dec << "..." << std::endl;
            i2c_dev->WriteRegisterByte(PRESCALE, prescale);

            // Restore the original state of mode1 register
            // std::cout << "Restoring original MODE1 value..." << std::endl;
            i2c_dev->WriteRegisterByte(MODE1, oldmode);

            // Wait for oscillator to stabilize
            usleep(INIT_SLEEP_DELAY_US);

            // Restart
            // std::cout << "Restarting with RESTART bit..." << std::endl;
            i2c_dev->WriteRegisterByte(MODE1, oldmode | RESTART);

            // std::cout << "PWM frequency set successfully!" << std::endl;
        } catch (const std::exception& e) {
            // std::cerr << "Exception in set_pwm_freq: " << e.what() << std::endl;
            throw;  // Re-throw to be caught by the retry mechanism
        }
    }

    /**
     * The PCA is a 12-bit device, so the on and off registers are separated into high and low bytes, 8-bits for the low
     * and 4-bits for the high. The on register is the point in the PWM cycle when the signal transitions from low to
     * high. The off register is the point in the PWM cycle when the signal transitions from high to low. Therefore,
     * each channel has 4 registers that need to be written to set the PWM signal.
     *
     * @throw std::system_error if an error occurs during the I2C communication.
     * @throw std::out_of_range if an error occurs during the I2C communication.
     */
    void PCA9685::set_pwm(const int channel, const uint16_t on, const uint16_t off) {
        if (channel < 0 || channel > 15) {
            throw std::out_of_range("Channel must be between 0 and 15");
        }
        const auto channel_offset = 4 * channel;
        i2c_dev->WriteRegisterByte(LED0_ON_L + channel_offset, on & 0xFF);
        i2c_dev->WriteRegisterByte(LED0_ON_H + channel_offset, on >> 8);
        i2c_dev->WriteRegisterByte(LED0_OFF_L + channel_offset, off & 0xFF);
        i2c_dev->WriteRegisterByte(LED0_OFF_H + channel_offset, off >> 8);
    }

    /**
     * The PCA9685 has a special register that allows for setting all PWM channels at once. This is useful for setting
     * all channels to the same value.
     *
     * @throw std::system_error if an error occurs during the I2C communication.
     */
    void PCA9685::set_all_pwm(const uint16_t on, const uint16_t off) {
        i2c_dev->WriteRegisterByte(ALL_LED_ON_L, on & 0xFF);
        i2c_dev->WriteRegisterByte(ALL_LED_ON_H, on >> 8);
        i2c_dev->WriteRegisterByte(ALL_LED_OFF_L, off & 0xFF);
        i2c_dev->WriteRegisterByte(ALL_LED_OFF_H, off >> 8);
    }

    /**
     * Get the PWM value for a specific channel
     */
    uint16_t PCA9685::get_pwm(const int channel, bool off) {
        if (channel < 0 || channel > 15) {
            throw std::out_of_range("Channel must be between 0 and 15");
        }

        // Calculate register addresses for this channel
        const auto channel_offset = 4 * channel;

        // Determine which register to read (ON or OFF)
        uint8_t reg_addr = LED0_ON_L + channel_offset;
        if (off) {
            reg_addr = LED0_OFF_L + channel_offset;
        }

        // Read low and high bytes
        uint8_t low_byte = i2c_dev->ReadRegisterByte(reg_addr);
        uint8_t high_byte = i2c_dev->ReadRegisterByte(reg_addr + 1);

        // Combine into 16-bit value
        return (static_cast<uint16_t>(high_byte) << 8) | low_byte;
    }

    /**
     * Read the prescale value from the PCA9685
     */
    uint8_t PCA9685::read_prescale() {
        connect();  // Ensure we're connected to the correct I2C address
        return i2c_dev->ReadRegisterByte(PRESCALE);
    }
    /**
     * Set the PWM value directly in microseconds
     */
    // void PCA9685::write_microseconds(const int channel, uint16_t microseconds) {
    //     if (channel < 0 || channel > 15) {
    //         throw std::out_of_range("Channel must be between 0 and 15");
    //     }

    //     // Calculate the pulse length based on the prescale value and oscillator frequency
    //     double pulse = microseconds;
    //     double pulse_length_us = 1000000.0;  // 1,000,000 us per second

    //     // Read the prescale value
    //     uint16_t prescale = read_prescale();

    //     // Calculate pulse length (in us per bit) according to the datasheet formula
    //     prescale += 1;
    //     pulse_length_us *= prescale;
    //     pulse_length_us /= OSCILLATOR_CLOCK_SPEED;

    //     // Convert microseconds to PWM value
    //     pulse /= pulse_length_us;

    //     // Set the PWM value
    //     set_pwm(channel, 0, static_cast<uint16_t>(pulse));
    // }

    void PCA9685::write_microseconds(const int channel, uint16_t microseconds) {
        if (channel < 0 || channel > 15) {
            throw std::out_of_range("Channel must be between 0 and 15");
        }

        connect();  // Ensure we're connected to the device

        // Calculate the pulse length
        double pulse_length_us = 1000000.0;  // 1,000,000 us per second

        // Read prescale value for accurate timing
        uint8_t prescale = read_prescale();
        double adjusted_prescale = prescale + 1.0;

        // Calculate pulse length according to datasheet formula
        pulse_length_us *= adjusted_prescale;
        pulse_length_us /= OSCILLATOR_CLOCK_SPEED;

        // Convert microseconds to PWM value
        double pulse = microseconds / pulse_length_us;

        // Limit to 12-bit resolution
        if (pulse > 4095)
            pulse = 4095;

        uint16_t pwm_value = static_cast<uint16_t>(pulse);

        // Calculate register addresses for this channel
        const uint8_t channel_offset = 4 * channel;

        // Use direct register writes with sequential delays
        i2c_dev->WriteRegisterByte(LED0_ON_L + channel_offset, 0);  // ON low byte
        usleep(1000);                                               // 1ms delay

        i2c_dev->WriteRegisterByte(LED0_ON_H + channel_offset, 0);  // ON high byte
        usleep(1000);                                               // 1ms delay

        i2c_dev->WriteRegisterByte(LED0_OFF_L + channel_offset, pwm_value & 0xFF);  // OFF low byte
        usleep(1000);                                                               // 1ms delay

        i2c_dev->WriteRegisterByte(LED0_OFF_H + channel_offset, (pwm_value >> 8) & 0xFF);  // OFF high byte
        usleep(1000);                                                                      // 1ms delay
    }

    /**
     * @throw std::system_error if an error occurs during the I2C communication.
     */
    void PCA9685::set_pwm_ms(const int channel, const double ms) {
        // Convert Hz to ms
        auto period_ms = 1000.0 / frequency;

        // Get the number of bits per ms
        auto bits_per_ms = BIT_RESOLUTION / period_ms;

        // Calculate the number of bits for the given ms
        auto bits = ms * bits_per_ms;

        set_pwm(channel, 0, bits);
    }

    /**
     * Read the current PWM values for a specific channel
     *
     * @throw std::system_error if an error occurs during the I2C communication
     * @throw std::out_of_range if the channel is invalid
     */
    std::pair<uint16_t, uint16_t> PCA9685::read_pwm(const int channel) {
        if (channel < 0 || channel > 15) {
            throw std::out_of_range("Channel must be between 0 and 15");
        }

        // Calculate register addresses for this channel
        const auto channel_offset = 4 * channel;

        // Read the ON and OFF values from the PCA9685 registers
        uint8_t on_low = i2c_dev->ReadRegisterByte(LED0_ON_L + channel_offset);
        uint8_t on_high = i2c_dev->ReadRegisterByte(LED0_ON_H + channel_offset);
        uint8_t off_low = i2c_dev->ReadRegisterByte(LED0_OFF_L + channel_offset);
        uint8_t off_high = i2c_dev->ReadRegisterByte(LED0_OFF_H + channel_offset);

        // Combine bytes into 16-bit values
        uint16_t on_value = (on_high << 8) | on_low;
        uint16_t off_value = (off_high << 8) | off_low;

        return std::make_pair(on_value, off_value);
    }

    /**
     * Get the current PWM value in microseconds for a channel
     *
     * @param channel One of the PWM output pins (0-15)
     * @return The pulse width in microseconds
     */
    uint16_t PCA9685::get_pulse_width_microseconds(const int channel) {
        if (channel < 0 || channel > 15) {
            throw std::out_of_range("Channel must be between 0 and 15");
        }

        // Read the PWM OFF value for this channel
        uint16_t pwm_value = get_pwm(channel, true);

        // Read the prescale value (needed for microsecond calculations)
        uint8_t prescale = read_prescale();

        // Calculate pulse length in microseconds per bit
        double pulse_length_us = 1000000.0 * (prescale + 1) / OSCILLATOR_CLOCK_SPEED;

        // Convert PWM value to microseconds
        return static_cast<uint16_t>(pwm_value * pulse_length_us);
    }

    void PCA9685::set_pwm_batch(const std::vector<int>& channels, const std::vector<uint16_t>& on_values,
                                const std::vector<uint16_t>& off_values) {
        // Validate inputs
        if (channels.size() != on_values.size() || channels.size() != off_values.size()) {
            throw std::invalid_argument("Channel, on_values, and off_values vectors must have the same size");
        }

        if (channels.empty()) {
            return;  // Nothing to do
        }

        connect();  // Ensure we're connected to the correct I2C address

        // Process each channel
        for (size_t i = 0; i < channels.size(); ++i) {
            int channel = channels[i];
            uint16_t on = on_values[i];
            uint16_t off = off_values[i];

            // Validate channel
            if (channel < 0 || channel > 15) {
                throw std::out_of_range("Channel must be between 0 and 15");
            }

            // For each channel, we need to write 4 bytes (on_L, on_H, off_L, off_H)
            const auto channel_offset = LED0_ON_L + (4 * channel);

            // Prepare register data
            std::vector<uint8_t> data = {
                static_cast<uint8_t>(on & 0xFF),         // on_L
                static_cast<uint8_t>((on >> 8) & 0xFF),  // on_H
                static_cast<uint8_t>(off & 0xFF),        // off_L
                static_cast<uint8_t>((off >> 8) & 0xFF)  // off_H
            };

            // Write block of data
            i2c_dev->WriteRegisterBlockAlternative(channel_offset, data);
        }
    }

    void PCA9685::write_microseconds_batch(const std::vector<int>& channels,
                                           const std::vector<uint16_t>& microseconds) {
        // Validate inputs
        if (channels.size() != microseconds.size()) {
            throw std::invalid_argument("Channel and microseconds vectors must have the same size");
        }

        if (channels.empty()) {
            return;  // Nothing to do
        }

        connect();  // Ensure we're connected to the correct I2C address

        // Read prescale value once for all calculations
        uint8_t prescale_value = read_prescale();
        double adjusted_prescale = prescale_value + 1.0;
        double pulse_length_us = 1000000.0;  // 1,000,000 us per second
        pulse_length_us *= adjusted_prescale;
        pulse_length_us /= OSCILLATOR_CLOCK_SPEED;

        // Maximum number of channels to process in one batch
        const size_t MAX_BATCH_SIZE = 2;  // Start with small batches

        // Process channels in smaller batches
        for (size_t batch_start = 0; batch_start < channels.size(); batch_start += MAX_BATCH_SIZE) {
            // Calculate batch size for this iteration
            size_t batch_size = std::min(MAX_BATCH_SIZE, channels.size() - batch_start);

            // Create temporary vectors for this batch
            std::vector<int> batch_channels;
            std::vector<uint16_t> batch_on_values;
            std::vector<uint16_t> batch_off_values;

            for (size_t i = 0; i < batch_size; i++) {
                int channel_idx = batch_start + i;
                int channel = channels[channel_idx];

                // Validate channel
                if (channel < 0 || channel > 15) {
                    throw std::out_of_range("Channel must be between 0 and 15");
                }

                // Calculate PWM value from microseconds
                double pulse = microseconds[channel_idx];
                pulse /= pulse_length_us;

                // Limit to 12-bit resolution
                if (pulse > 4095)
                    pulse = 4095;

                uint16_t off_value = static_cast<uint16_t>(pulse);

                batch_channels.push_back(channel);
                batch_on_values.push_back(0);  // All 'on' values are 0
                batch_off_values.push_back(off_value);
            }

            // Use the batch method for this small group
            set_pwm_batch(batch_channels, batch_on_values, batch_off_values);

            // Add a small delay between batches
            usleep(5000);  // 5ms delay
        }
    }

    std::vector<uint16_t> PCA9685::read_pwm_batch(uint8_t start_channel, uint8_t num_channels) {
        // Validate input
        if (start_channel > 15) {
            throw std::out_of_range("Start channel must be between 0 and 15");
        }

        if (num_channels == 0) {
            return {};  // Empty result
        }

        if (start_channel + num_channels > 16) {
            throw std::out_of_range("Channel range exceeds valid channels (0-15)");
        }

        connect();  // Ensure we're connected to the correct I2C address

        // Calculate the starting register address
        const uint8_t start_register = LED0_ON_L + (4 * start_channel);

        // Calculate total registers to read (4 registers per channel)
        const uint16_t total_registers = 4 * num_channels;

        // Read all registers in a single transaction
        std::vector<uint8_t> register_data = i2c_dev->ReadRegisterBlock(start_register, total_registers);

        // Process the registers into on/off values
        std::vector<uint16_t> pwm_values;
        pwm_values.reserve(num_channels);

        for (uint8_t i = 0; i < num_channels; i++) {
            // Extract the 4 registers for this channel
            const uint16_t base_idx = i * 4;

            // Combine low and high bytes to get on and off values
            uint16_t on_value = register_data[base_idx] | (static_cast<uint16_t>(register_data[base_idx + 1]) << 8);
            uint16_t off_value =
                register_data[base_idx + 2] | (static_cast<uint16_t>(register_data[base_idx + 3]) << 8);

            // Calculate the pulse width
            uint16_t pulse_width;

            // Check if fully on or fully off
            if ((on_value & 0x1000) != 0) {
                // Fully on
                pulse_width = 4096;
            } else if ((off_value & 0x1000) != 0) {
                // Fully off
                pulse_width = 0;
            } else {
                // Calculate pulse width (off - on), handling wrap-around
                pulse_width = (off_value >= on_value) ? (off_value - on_value) : (4096 + off_value - on_value);
            }

            pwm_values.push_back(pulse_width);
        }

        return pwm_values;
    }

    void PCA9685::set_pwm_batch_smbus(const std::vector<int>& channels, const std::vector<uint16_t>& on_values,
                                      const std::vector<uint16_t>& off_values) {
        // Validate inputs
        if (channels.size() != on_values.size() || channels.size() != off_values.size()) {
            throw std::invalid_argument("Channel, on_values, and off_values vectors must have the same size");
        }

        if (channels.empty()) {
            return;  // Nothing to do
        }

        connect();  // Ensure we're connected to the correct I2C address

        // SMBus block transfer is limited to 32 bytes, so we need to process channels in chunks
        // Each channel requires 4 bytes (ON_L, ON_H, OFF_L, OFF_H)
        const size_t MAX_CHANNELS_PER_TRANSFER = 8;  // 8 channels * 4 bytes = 32 bytes

        // Process channels in chunks
        for (size_t base_idx = 0; base_idx < channels.size(); base_idx += MAX_CHANNELS_PER_TRANSFER) {
            // Determine how many channels to process in this chunk
            size_t chunk_size = std::min(MAX_CHANNELS_PER_TRANSFER, channels.size() - base_idx);

            // Process each channel in this chunk individually
            for (size_t i = 0; i < chunk_size; i++) {
                size_t idx = base_idx + i;
                int channel = channels[idx];

                // Validate channel
                if (channel < 0 || channel > 15) {
                    throw std::out_of_range("Channel must be between 0 and 15");
                }

                // Calculate register addresses for this channel
                uint8_t base_register = LED0_ON_L + (4 * channel);

                // Prepare the 4 bytes of data for this channel
                std::vector<uint8_t> data = {
                    static_cast<uint8_t>(on_values[idx] & 0xFF),         // on_L
                    static_cast<uint8_t>((on_values[idx] >> 8) & 0xFF),  // on_H
                    static_cast<uint8_t>(off_values[idx] & 0xFF),        // off_L
                    static_cast<uint8_t>((off_values[idx] >> 8) & 0xFF)  // off_H
                };

                // Write using SMBus block operation
                try {
                    i2c_dev->WriteSMBusBlock(base_register, data);
                } catch (const std::exception& e) {
                    // Fall back to individual byte writes if block write fails
                    std::cerr << "SMBus block write failed for channel " << channel << ": " << e.what() << std::endl;
                    std::cerr << "Falling back to individual byte writes..." << std::endl;

                    for (size_t j = 0; j < 4; j++) {
                        i2c_dev->WriteSMBusByte(base_register + j, data[j]);
                        usleep(1000);  // 1ms delay between writes
                    }
                }

                // Small delay between channels
                usleep(2000);  // 2ms delay
            }
        }
    }

    void PCA9685::write_microseconds_batch_smbus(const std::vector<int>& channels,
                                                 const std::vector<uint16_t>& microseconds) {
        // Validate inputs
        if (channels.size() != microseconds.size()) {
            throw std::invalid_argument("Channel and microseconds vectors must have the same size");
        }

        if (channels.empty()) {
            return;  // Nothing to do
        }

        connect();  // Ensure we're connected to the correct I2C address

        // Read prescale value once for all calculations
        uint8_t prescale_value = read_prescale();

        // Calculate pulse length (in us per bit) according to the datasheet formula
        double adjusted_prescale = prescale_value + 1.0;
        double pulse_length_us = 1000000.0;  // 1,000,000 us per second
        pulse_length_us *= adjusted_prescale;
        pulse_length_us /= OSCILLATOR_CLOCK_SPEED;

        // Prepare on/off values for all channels
        std::vector<uint16_t> on_values(channels.size(), 0);  // All 'on' values are 0
        std::vector<uint16_t> off_values(channels.size());

        // Calculate 'off' values based on microseconds
        for (size_t i = 0; i < channels.size(); ++i) {
            double pulse = microseconds[i];
            pulse /= pulse_length_us;

            // Limit to 12-bit resolution
            if (pulse > 4095)
                pulse = 4095;

            off_values[i] = static_cast<uint16_t>(pulse);
        }

        // Use our new SMBus method
        set_pwm_batch_smbus(channels, on_values, off_values);
    }

    // In pca9685_comm.cpp

    void PCA9685::enable_auto_increment() {
        connect();

        // Read current MODE1 register value
        uint8_t mode1 = i2c_dev->ReadRegisterByte(MODE1);
        std::cout << "Current MODE1 value: 0x" << std::hex << static_cast<int>(mode1) << std::dec << std::endl;

        // Set bit 5 (AI) to enable auto-increment
        if ((mode1 & 0x20) == 0) {
            std::cout << "Auto-increment bit was not set. Enabling..." << std::endl;
            mode1 |= 0x20;  // Set bit 5 (AI)
            i2c_dev->WriteRegisterByte(MODE1, mode1);

            // Verify the change
            uint8_t new_mode1 = i2c_dev->ReadRegisterByte(MODE1);
            std::cout << "New MODE1 value: 0x" << std::hex << static_cast<int>(new_mode1) << std::dec << std::endl;
        } else {
            std::cout << "Auto-increment bit is already set" << std::endl;
        }
    }

    // Simplified manual register write that uses the existing WriteRegisterBlock
    void PCA9685::write_registers_manual(uint8_t start_register, const std::vector<uint8_t>& values) {
        if (values.empty()) {
            return;
        }

        connect();

        // First ensure auto-increment is enabled
        enable_auto_increment();

        // Use the existing WriteRegisterBlock method
        std::cout << "Writing " << values.size() << " bytes to registers starting at 0x" << std::hex
                  << static_cast<int>(start_register) << std::dec << std::endl;

        try {
            i2c_dev->WriteRegisterBlock(start_register, values);
            std::cout << "Block write succeeded" << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "Block write failed: " << e.what() << std::endl;

            // Fall back to individual register writes
            std::cout << "Falling back to individual writes..." << std::endl;

            for (size_t i = 0; i < values.size(); i++) {
                try {
                    i2c_dev->WriteRegisterByte(start_register + i, values[i]);
                    std::cout << "  Wrote 0x" << std::hex << static_cast<int>(values[i]) << " to register 0x"
                              << static_cast<int>(start_register + i) << std::dec << std::endl;
                } catch (const std::exception& e) {
                    std::cerr << "  Individual write failed: " << e.what() << std::endl;
                }

                // Small delay between writes
                usleep(1000);  // 1ms delay
            }
        }
    }

    void PCA9685::write_microseconds_batch_manual(const std::vector<int>& channels,
                                                  const std::vector<uint16_t>& microseconds) {
        if (channels.size() != microseconds.size()) {
            throw std::invalid_argument("Channel and microseconds vectors must have the same size");
        }

        if (channels.empty()) {
            return;
        }

        connect();

        // Calculate pulse lengths
        uint8_t prescale_value = read_prescale();
        double adjusted_prescale = prescale_value + 1.0;
        double pulse_length_us = 1000000.0;  // 1,000,000 us per second
        pulse_length_us *= adjusted_prescale;
        pulse_length_us /= OSCILLATOR_CLOCK_SPEED;

        // Process channels in pairs to keep message sizes small
        for (size_t i = 0; i < channels.size(); i += 2) {
            size_t chunk_size = std::min(size_t(2), channels.size() - i);

            // Prepare data for this chunk
            std::vector<uint8_t> chunk_data;

            for (size_t j = 0; j < chunk_size; j++) {
                int channel = channels[i + j];

                // Validate channel
                if (channel < 0 || channel > 15) {
                    throw std::out_of_range("Channel must be between 0 and 15");
                }

                // Calculate PWM values
                double pulse = microseconds[i + j] / pulse_length_us;
                if (pulse > 4095)
                    pulse = 4095;

                uint16_t off_value = static_cast<uint16_t>(pulse);

                // ON value is always 0
                chunk_data.push_back(0);  // ON_L
                chunk_data.push_back(0);  // ON_H

                // OFF value based on microseconds
                chunk_data.push_back(off_value & 0xFF);         // OFF_L
                chunk_data.push_back((off_value >> 8) & 0xFF);  // OFF_H
            }

            // Determine starting register
            uint8_t start_register = LED0_ON_L + (4 * channels[i]);

            try {
                write_registers_manual(start_register, chunk_data);
                std::cout << "Manual write successful for channel(s): ";
                for (size_t j = 0; j < chunk_size; j++) {
                    std::cout << channels[i + j];
                    if (j < chunk_size - 1)
                        std::cout << ", ";
                }
                std::cout << std::endl;
            } catch (const std::exception& e) {
                std::cerr << "Error in manual write: " << e.what() << std::endl;

                // Fall back to individual writes
                std::cout << "Falling back to individual writes..." << std::endl;

                for (size_t j = 0; j < chunk_size; j++) {
                    try {
                        write_microseconds(channels[i + j], microseconds[i + j]);
                        std::cout << "Individual write for channel " << channels[i + j] << " successful" << std::endl;
                    } catch (const std::exception& e) {
                        std::cerr << "Individual write failed for channel " << channels[i + j] << ": " << e.what()
                                  << std::endl;
                    }
                }
            }

            // Small delay between chunks
            usleep(5000);  // 5ms delay
        }
    }

    std::vector<uint16_t> PCA9685::read_microseconds_batch(uint8_t start_channel, uint8_t num_channels) {
        // Get raw PWM values
        std::vector<uint16_t> pwm_values = read_pwm_batch(start_channel, num_channels);

        // If empty, return empty result
        if (pwm_values.empty()) {
            return {};
        }

        // Read prescale value once for all conversions
        uint8_t prescale = read_prescale();

        // Calculate pulse length (in us per bit) according to the datasheet formula
        double adjusted_prescale = prescale + 1.0;
        double pulse_length_us = 1000000.0;  // 1,000,000 us per second
        pulse_length_us *= adjusted_prescale;
        pulse_length_us /= OSCILLATOR_CLOCK_SPEED;

        // Convert PWM values to microseconds
        std::vector<uint16_t> microseconds;
        microseconds.reserve(pwm_values.size());

        for (uint16_t pulse_width : pwm_values) {
            // Convert to microseconds
            uint16_t us = pulse_width * pulse_length_us;
            microseconds.push_back(us);
        }

        return microseconds;
    }

}  // namespace pca9685_hardware_interface