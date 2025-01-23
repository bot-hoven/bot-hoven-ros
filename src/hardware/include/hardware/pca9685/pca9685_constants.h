#ifndef HARDWARE__PCA9685__PCA9685_CONSTANTS_H
#define HARDWARE__PCA9685__PCA9685_CONSTANTS_H
#include <cstdint>

namespace pca9685_hardware_interface {

    // Registers/etc:
    constexpr uint8_t MODE1 = 0x00;
    constexpr uint8_t MODE2 = 0x01;
    constexpr uint8_t SUBADR1 = 0x02;
    constexpr uint8_t SUBADR2 = 0x03;
    constexpr uint8_t SUBADR3 = 0x04;
    constexpr uint8_t PRESCALE = 0xFE;
    constexpr uint8_t LED0_ON_L = 0x06;
    constexpr uint8_t LED0_ON_H = 0x07;
    constexpr uint8_t LED0_OFF_L = 0x08;
    constexpr uint8_t LED0_OFF_H = 0x09;
    constexpr uint8_t ALL_LED_ON_L = 0xFA;
    constexpr uint8_t ALL_LED_ON_H = 0xFB;
    constexpr uint8_t ALL_LED_OFF_L = 0xFC;
    constexpr uint8_t ALL_LED_OFF_H = 0xFD;

    // Bits:
    constexpr uint8_t RESTART = 0x80;
    constexpr uint8_t SLEEP = 0x10;
    constexpr uint8_t ALLCALL = 0x01;
    constexpr uint8_t INVRT = 0x10;
    constexpr uint8_t OUTDRV = 0x04;
    constexpr uint8_t OCH = 0x08;

    // Datasheet Constants:
    constexpr useconds_t INIT_SLEEP_DELAY_US = 5'000;
    constexpr uint32_t OSCILLATOR_CLOCK_SPEED = 25'000'000;  // 25MHz
    constexpr uint16_t BIT_RESOLUTION = 4'096; // 12 bit device

    // Other Constants:
    constexpr uint32_t NS_PER_US = 1000;
    constexpr uint32_t I2C_REWRITE_DELAY_US = 100;

}  // namespace pca9685_hardware_interface

#endif  // HARDWARE__PCA9685__PCA9685_CONSTANTS_H
