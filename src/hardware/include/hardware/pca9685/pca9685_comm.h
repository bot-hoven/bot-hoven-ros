// // #ifndef HARDWARE__PCA9685__PCA9685_H
// // #define HARDWARE__PCA9685__PCA9685_H

// // #include "hardware_common.hpp"

// // #include <string>
// // #include <memory>

// // namespace pca9685_hardware_interface {

// // // class I2CBusManager;

// // class PCA9685 {
// // public:
// //     int frequency = 50;

// //     PCA9685(std::shared_ptr<hardware::I2CBusManager> i2c_device, int device_address);
// //     ~PCA9685();

// //     void init();
// //     void disconnect();

// //     void set_pwm_freq(const double freq_hz);

// //     void set_pwm(const int channel, const uint16_t on, const uint16_t off);

// //     void set_all_pwm(const uint16_t on, const uint16_t off);

// //     void set_pwm_ms(const int channel, const double ms);

// //     void set_device(const std::string &device_path) { device = device_path; }
// //     void set_address(int addr) { address = addr; }
// //     // void set_frequency(int freq) { frequency = freq; }

// // private:
// //     std::shared_ptr<hardware::I2CPeripheral> i2c_dev;

// //     std::string device = "/dev/i2c-1";
// //     int address = 0x40;
// // };

// // }  // namespace pca9685_hardware_interface

// // #endif //HARDWARE__PCA9685__PCA9685_H


#ifndef HARDWARE__PCA9685__PCA9685_COMM_H_
#define HARDWARE__PCA9685__PCA9685_COMM_H_

#include <string>
#include <memory>

#include "hardware/common/Constants.h"
#include "hardware/i2c/I2CPeripheral.h"

namespace pca9685_hardware_interface {

class PCA9685 {
public:
    PCA9685(std::shared_ptr<hardware::I2CPeripheral> i2c_device, int address);
    ~PCA9685();

    // void connect_to_i2c(const int address);

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