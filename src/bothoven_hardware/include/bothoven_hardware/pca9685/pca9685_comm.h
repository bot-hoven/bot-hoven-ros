#ifndef HARDWARE__PCA9685__PCA9685_COMM_H
#define HARDWARE__PCA9685__PCA9685_COMM_H

#include <memory>
#include <string>
#include <vector>

#include "bothoven_hardware/i2c/I2CPeripheral.h"
#include "bothoven_hardware/pca9685/pca9685_constants.h"

namespace pca9685_hardware_interface {

    class PCA9685 {
    public:
        PCA9685() = default;
        ~PCA9685();

        void setup(std::shared_ptr<hardware::I2CPeripheral> i2c_bus, const int i2c_address);

        void connect();

        void init();

        void set_pwm_freq(const double freq_hz);

        void set_pwm(const int channel, const uint16_t on, const uint16_t off);

        void set_all_pwm(const uint16_t on, const uint16_t off);

        void set_pwm_ms(const int channel, const double ms);

        uint16_t get_pwm(const int channel, bool off = true);

        void write_microseconds(const int channel, uint16_t microseconds);

        uint8_t read_prescale();

        std::pair<uint16_t, uint16_t> read_pwm(const int channel);

        uint16_t get_pulse_width_microseconds(const int channel);

        void set_pwm_batch(const std::vector<int>& channels, const std::vector<uint16_t>& on_values,
                           const std::vector<uint16_t>& off_values);

        void write_microseconds_batch(const std::vector<int>& channels, const std::vector<uint16_t>& microseconds);

        std::vector<uint16_t> read_pwm_batch(uint8_t start_channel, uint8_t num_channels);

        std::vector<uint16_t> read_microseconds_batch(uint8_t start_channel, uint8_t num_channels);

        void write_microseconds_batch_smbus(const std::vector<int>& channels,
                                            const std::vector<uint16_t>& microseconds);

        void set_pwm_batch_smbus(const std::vector<int>& channels, const std::vector<uint16_t>& on_values,
                                 const std::vector<uint16_t>& off_values);

        void enable_auto_increment();
        void write_microseconds_batch_manual(const std::vector<int>& channels,
                                             const std::vector<uint16_t>& microseconds);
        void write_registers_manual(uint8_t start_register, const std::vector<uint8_t>& values);

    private:
        std::shared_ptr<hardware::I2CPeripheral> i2c_dev;

        // Default frequency pulled from PCA9685 datasheet.
        double frequency = 200.0;
        int address = 0x40;
    };

}  // namespace pca9685_hardware_interface

#endif  // HARDWARE__PCA9685__PCA9685_COMM_H