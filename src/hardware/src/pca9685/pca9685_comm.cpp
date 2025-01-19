#include "hardware/pca9685/pca9685_comm.h"

#include <unistd.h>
#include <system_error>
#include <cmath>

namespace pca9685_hardware_interface {

    PCA9685::~PCA9685() = default;

    void PCA9685::setup(std::shared_ptr<hardware::I2CPeripheral> i2c_bus, const int i2c_address){
        i2c_dev = i2c_bus;
        address = i2c_address;
    }

    void PCA9685::connect(){
        try {
            if (i2c_dev->GetCurrentI2CAddress() != address) {
                i2c_dev->ConnectToPeripheral(address);
            }
        } catch (const std::exception& e) {
            throw std::runtime_error(
                "Error: PCA9685 with I2C address " + std::to_string(address) + 
                " failed to connect to the I2C bus: " + std::string(e.what())
            );
        }
    }

    void PCA9685::init() {
        set_all_pwm(0, 0);
        i2c_dev->WriteRegisterByte(MODE2, OUTDRV);
        i2c_dev->WriteRegisterByte(MODE1, ALLCALL);
        usleep(5'000);
        auto mode1_val = i2c_dev->ReadRegisterByte(MODE1);
        mode1_val &= ~SLEEP;
        i2c_dev->WriteRegisterByte(MODE1, mode1_val);
        usleep(5'000);
    }

    void PCA9685::set_pwm_freq(const double freq_hz) {
        frequency = freq_hz;

        auto prescaleval = 2.5e7;  //    # 25MHz
        prescaleval /= 4096.0;     //       # 12-bit
        prescaleval /= freq_hz;
        prescaleval -= 1.0;

        auto prescale = static_cast<int>(std::round(prescaleval));

        const auto oldmode = i2c_dev->ReadRegisterByte(MODE1);

        auto newmode = (oldmode & 0x7F) | SLEEP;

        // i2c_dev->ConnectToPeripheral(address);
        i2c_dev->WriteRegisterByte(MODE1, newmode);
        i2c_dev->WriteRegisterByte(PRESCALE, prescale);
        i2c_dev->WriteRegisterByte(MODE1, oldmode);
        usleep(5'000);
        i2c_dev->WriteRegisterByte(MODE1, oldmode | RESTART);
    }

    void PCA9685::set_pwm(const int channel, const uint16_t on, const uint16_t off) {
        const auto channel_offset = 4 * channel;
        // i2c_dev->ConnectToPeripheral(address);
        i2c_dev->WriteRegisterByte(LED0_ON_L + channel_offset, on & 0xFF);
        i2c_dev->WriteRegisterByte(LED0_ON_H + channel_offset, on >> 8);
        i2c_dev->WriteRegisterByte(LED0_OFF_L + channel_offset, off & 0xFF);
        i2c_dev->WriteRegisterByte(LED0_OFF_H + channel_offset, off >> 8);
    }

    void PCA9685::set_all_pwm(const uint16_t on, const uint16_t off) {
        // i2c_dev->ConnectToPeripheral(address);
        i2c_dev->WriteRegisterByte(ALL_LED_ON_L, on & 0xFF);
        i2c_dev->WriteRegisterByte(ALL_LED_ON_H, on >> 8);
        i2c_dev->WriteRegisterByte(ALL_LED_OFF_L, off & 0xFF);
        i2c_dev->WriteRegisterByte(ALL_LED_OFF_H, off >> 8);
    }

    void PCA9685::set_pwm_ms(const int channel, const double ms) {
        auto period_ms = 1000.0 / frequency;
        auto bits_per_ms = 4096 / period_ms;
        auto bits = ms * bits_per_ms;
        set_pwm(channel, 0, bits);
    }

}  // namespace pca9685_hardware_interface