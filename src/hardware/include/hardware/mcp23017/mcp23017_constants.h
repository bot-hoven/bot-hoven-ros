#ifndef HARDWARE__PCA9685__PCA9685_CONSTANTS_H
#define HARDWARE__PCA9685__PCA9685_CONSTANTS_H
#include <cstdint>

namespace mcp23017_hardware_interface {

    // Registers/etc:
    constexpr uint8_t IODIRA = 0x00;
    constexpr uint8_t IODIRB = 0x01;
    constexpr uint8_t IPOLA = 0x02;
    constexpr uint8_t IPOLB = 0x03;
    constexpr uint8_t GPINTENA = 0x04;
    constexpr uint8_t GPINTENB = 0x05;
    constexpr uint8_t GPPUA = 0x0C;
    constexpr uint8_t GPPUB = 0x0D;
    constexpr uint8_t GPIOA = 0x12;
    constexpr uint8_t GPIOB = 0x13;
    constexpr uint8_t OLATA = 0x14;
    constexpr uint8_t OLATB = 0x15;

    // Other Constants:
    constexpr uint32_t NS_PER_US = 1000;
    constexpr uint32_t I2C_REWRITE_DELAY_US = 100;

}  // namespace mcp23017_hardware_interface

#endif  // HARDWARE__PCA9685__PCA9685_CONSTANTS_H
