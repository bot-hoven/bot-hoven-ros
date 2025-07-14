#ifndef HARDWARE__ADS7138__ADS7138_CONSTANTS_H
#define HARDWARE__ADS7138__ADS7138_CONSTANTS_H

#include <cstdint>

namespace ads7138_hardware_interface {

    // Device address options (A0 pin configuration)
    constexpr uint8_t ADS7138_ADDR_GND = 0x10;  // A0 pin connected to GND (default)
    constexpr uint8_t ADS7138_ADDR_VDD = 0x11;  // A0 pin connected to VDD
    constexpr uint8_t ADS7138_ADDR_SDA = 0x12;  // A0 pin connected to SDA
    constexpr uint8_t ADS7138_ADDR_SCL = 0x13;  // A0 pin connected to SCL

    // Register addresses
    constexpr uint8_t ADS7138_REG_DEVICE_ID      = 0x00;  // Device ID register
    constexpr uint8_t ADS7138_REG_DEVICE_STATUS  = 0x01;  // Device status register
    constexpr uint8_t ADS7138_REG_SYS_STATUS     = 0x02;  // System status register
    constexpr uint8_t ADS7138_REG_ALERT_STATUS   = 0x03;  // Alert status register
    constexpr uint8_t ADS7138_REG_ALERT_MASK     = 0x04;  // Alert mask register
    constexpr uint8_t ADS7138_REG_ALERT_PIN      = 0x05;  // Alert pin control register
    constexpr uint8_t ADS7138_REG_AUTO_SEQ_CH    = 0x06;  // Auto-sequence channel enable
    constexpr uint8_t ADS7138_REG_SEQ_STATUS     = 0x07;  // Sequence status register
    constexpr uint8_t ADS7138_REG_GPO_CONFIG     = 0x08;  // GPO configuration register
    constexpr uint8_t ADS7138_REG_GPO_DRIVE      = 0x09;  // GPO drive register
    constexpr uint8_t ADS7138_REG_GPO_VALUE      = 0x0A;  // GPO value register
    constexpr uint8_t ADS7138_REG_GPI_VALUE      = 0x0B;  // GPI value register
    constexpr uint8_t ADS7138_REG_RESET          = 0x0C;  // Reset control register
    constexpr uint8_t ADS7138_REG_MANUAL_CH_SEL  = 0x0D;  // Manual channel select register
    constexpr uint8_t ADS7138_REG_AUTO_SEQ_CTRL  = 0x0E;  // Auto-sequence control register
    constexpr uint8_t ADS7138_REG_CONV_RATE      = 0x0F;  // Conversion rate register

    // Channel data registers (manual mode)
    constexpr uint8_t ADS7138_REG_MANUAL_DATA_0  = 0x10;  // Manual conversion data - MSB
    constexpr uint8_t ADS7138_REG_MANUAL_DATA_1  = 0x11;  // Manual conversion data - LSB

    // Channel data registers (auto sequence mode)
    constexpr uint8_t ADS7138_REG_CH0_DATA_0     = 0x12;  // Channel 0 data - MSB
    constexpr uint8_t ADS7138_REG_CH0_DATA_1     = 0x13;  // Channel 0 data - LSB
    constexpr uint8_t ADS7138_REG_CH1_DATA_0     = 0x14;  // Channel 1 data - MSB
    constexpr uint8_t ADS7138_REG_CH1_DATA_1     = 0x15;  // Channel 1 data - LSB
    constexpr uint8_t ADS7138_REG_CH2_DATA_0     = 0x16;  // Channel 2 data - MSB
    constexpr uint8_t ADS7138_REG_CH2_DATA_1     = 0x17;  // Channel 2 data - LSB
    constexpr uint8_t ADS7138_REG_CH3_DATA_0     = 0x18;  // Channel 3 data - MSB
    constexpr uint8_t ADS7138_REG_CH3_DATA_1     = 0x19;  // Channel 3 data - LSB
    constexpr uint8_t ADS7138_REG_CH4_DATA_0     = 0x1A;  // Channel 4 data - MSB
    constexpr uint8_t ADS7138_REG_CH4_DATA_1     = 0x1B;  // Channel 4 data - LSB
    constexpr uint8_t ADS7138_REG_CH5_DATA_0     = 0x1C;  // Channel 5 data - MSB
    constexpr uint8_t ADS7138_REG_CH5_DATA_1     = 0x1D;  // Channel 5 data - LSB
    constexpr uint8_t ADS7138_REG_CH6_DATA_0     = 0x1E;  // Channel 6 data - MSB
    constexpr uint8_t ADS7138_REG_CH6_DATA_1     = 0x1F;  // Channel 6 data - LSB
    constexpr uint8_t ADS7138_REG_CH7_DATA_0     = 0x20;  // Channel 7 data - MSB
    constexpr uint8_t ADS7138_REG_CH7_DATA_1     = 0x21;  // Channel 7 data - LSB

    // Channel lower threshold registers
    constexpr uint8_t ADS7138_REG_CH0_LTH_0      = 0x22;  // Channel 0 lower threshold - MSB
    constexpr uint8_t ADS7138_REG_CH0_LTH_1      = 0x23;  // Channel 0 lower threshold - LSB
    // ... similar for other channels

    // Channel upper threshold registers
    constexpr uint8_t ADS7138_REG_CH0_UTH_0      = 0x32;  // Channel 0 upper threshold - MSB
    constexpr uint8_t ADS7138_REG_CH0_UTH_1      = 0x33;  // Channel 0 upper threshold - LSB
    // ... similar for other channels

    // Channel hysteresis registers
    constexpr uint8_t ADS7138_REG_CH0_HYS        = 0x42;  // Channel 0 hysteresis
    // ... similar for other channels

    // Device ID values
    constexpr uint8_t ADS7138_ID                 = 0x10;  // Device ID for ADS7138

    // Bit masks and values for various registers
    constexpr uint8_t ADS7138_RESET_MASK         = 0x01;  // Reset bit in RESET register
    constexpr uint8_t ADS7138_AS_START_MASK      = 0x01;  // Start bit in AUTO_SEQ_CTRL
    constexpr uint8_t ADS7138_AS_CONT_MASK       = 0x02;  // Continuous mode bit in AUTO_SEQ_CTRL
    
    // Manual channel select values
    constexpr uint8_t ADS7138_MANUAL_CH0         = 0x00;  // Select channel 0
    constexpr uint8_t ADS7138_MANUAL_CH1         = 0x01;  // Select channel 1
    constexpr uint8_t ADS7138_MANUAL_CH2         = 0x02;  // Select channel 2
    constexpr uint8_t ADS7138_MANUAL_CH3         = 0x03;  // Select channel 3
    constexpr uint8_t ADS7138_MANUAL_CH4         = 0x04;  // Select channel 4
    constexpr uint8_t ADS7138_MANUAL_CH5         = 0x05;  // Select channel 5
    constexpr uint8_t ADS7138_MANUAL_CH6         = 0x06;  // Select channel 6
    constexpr uint8_t ADS7138_MANUAL_CH7         = 0x07;  // Select channel 7

    // Conversion rates
    constexpr uint8_t ADS7138_CONV_RATE_1K       = 0x00;  // 1 kSPS
    constexpr uint8_t ADS7138_CONV_RATE_2K       = 0x01;  // 2 kSPS
    constexpr uint8_t ADS7138_CONV_RATE_5K       = 0x02;  // 5 kSPS
    constexpr uint8_t ADS7138_CONV_RATE_10K      = 0x03;  // 10 kSPS
    constexpr uint8_t ADS7138_CONV_RATE_20K      = 0x04;  // 20 kSPS
    constexpr uint8_t ADS7138_CONV_RATE_50K      = 0x05;  // 50 kSPS
    constexpr uint8_t ADS7138_CONV_RATE_100K     = 0x06;  // 100 kSPS
    constexpr uint8_t ADS7138_CONV_RATE_200K     = 0x07;  // 200 kSPS

    // Other constants
    constexpr uint16_t ADS7138_RESOLUTION        = 4096;  // 12-bit resolution (2^12)
    constexpr double ADS7138_VREF                = 2.5;   // Reference voltage in volts
    constexpr uint32_t I2C_REWRITE_DELAY_US      = 100;   // Delay between I2C operations

} // namespace ads7138_hardware_interface

#endif // HARDWARE__ADS7138__ADS7138_CONSTANTS_H