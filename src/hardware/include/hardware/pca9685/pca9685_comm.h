#ifndef HARDWARE__PCA9685__PCA9685_COMM_H_
#define HARDWARE__PCA9685__PCA9685_COMM_H_

#include <string>
#include <memory>

#include "pca9685/Constants.h"
#include "i2c/I2CPeripheral.h"

namespace pca9685_hardware_interface {

class PCA9685 {
public:
    PCA9685(std::shared_ptr<hardware::I2CPeripheral> i2c_device, int address);
    ~PCA9685();

    void init_i2c();

    void set_pwm_freq(const double freq_hz);

    void set_pwm(const int channel, const uint16_t on, const uint16_t off);

    void set_all_pwm(const uint16_t on, const uint16_t off);

    void set_pwm_ms(const int channel, const double ms);

private:
    std::shared_ptr<hardware::I2CPeripheral> i2c_dev;
    
    // Default frequency pulled from PCA9685 datasheet.
    double frequency = 200.0;
    int address = 0x40;
    

};

}  // namespace pca9685_hardware_interface

#endif //HARDWARE__PCA9685__PCA9685_COMM_H_