#ifndef HARDWARE__PCA9685__PCA9685_H
#define HARDWARE__PCA9685__PCA9685_H

#include <string>
#include <memory>

namespace PiPCA9685 {

class I2CPeripheral;

class PCA9685 {
public:
    int frequency = 50;

    PCA9685() = default;
    ~PCA9685();

    void init();
    void disconnect();

    void set_pwm_freq(const double freq_hz);

    void set_pwm(const int channel, const uint16_t on, const uint16_t off);

    void set_all_pwm(const uint16_t on, const uint16_t off);

    void set_pwm_ms(const int channel, const double ms);

    void set_device(const std::string &device_path) { device = device_path; }
    void set_address(int addr) { address = addr; }
    // void set_frequency(int freq) { frequency = freq; }

private:
    std::shared_ptr<I2CPeripheral> i2c_dev;

    std::string device = "/dev/i2c-1";
    int address = 0x40;
};

}  // namespace PiPCA9685

#endif //HARDWARE__PCA9685__PCA9685_H
