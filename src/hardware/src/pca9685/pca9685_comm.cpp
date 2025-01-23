#include "hardware/pca9685/pca9685_comm.h"

#include <unistd.h>

#include <chrono>
#include <cmath>
#include <sstream>
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
    int PCA9685::connect() {
        if (i2c_dev->GetCurrentI2CAddress() != address) {
            i2c_dev->ConnectToPeripheral(address);
        }
    }

     /**
     * @throw std::system_error if an error occurs during the I2C communication.
     */
    void PCA9685::init() {
        set_all_pwm(0, 0);

        // Set component to totem pole structure
        // Set PWM outputs to only change on last ACK of all 4 writes for a PWM channel
        i2c_dev->WriteRegisterByte(MODE2, OUTDRV & OCH);

        // PCA9685 responds to LED All Call I2C-bus address (0x70)
        i2c_dev->WriteRegisterByte(MODE1, ALLCALL);
        usleep(INIT_SLEEP_DELAY_US);
        auto mode1_val = i2c_dev->ReadRegisterByte(MODE1);

        // After the oscillator is stable, the SLEEP bit can be set to 0, component starts in sleep mode by default
        mode1_val &= ~SLEEP;
        i2c_dev->WriteRegisterByte(MODE1, mode1_val);
        usleep(INIT_SLEEP_DELAY_US);
    }

    /**
     * This sets the frequency of the PWM output modulation.
     * From the datasheet the formula for determining the prescale value is
     * prescale_value = round(osc_clock / (4096 * frequency)) - 1
     *
     * @throw std::system_error if an error occurs during the I2C communication.
     */
    void PCA9685::set_pwm_freq(const double freq_hz) {
        frequency = freq_hz;

        auto prescaleval = OSCILLATOR_CLOCK_SPEED;
        prescaleval /= BIT_RESOLUTION * 1.0;
        prescaleval /= freq_hz;
        prescaleval -= 1.0;

        auto prescale = static_cast<int>(std::round(prescaleval));

        // Get the current mode1 register value
        const auto oldmode = i2c_dev->ReadRegisterByte(MODE1);

        // Ensure the RESTART bit is set to 0 and SLEEP is set to 1
        // as prescale can only be updated in sleep mode
        auto newmode = (oldmode & 0x7F) | SLEEP;

        // Put the PCA in sleep mode
        i2c_dev->WriteRegisterByte(MODE1, newmode);

        // Write the prescale value
        i2c_dev->WriteRegisterByte(PRESCALE, prescale);

        // Restore the original state of mode1 register
        i2c_dev->WriteRegisterByte(MODE1, oldmode);

        // Wait for oscillator to stabilize before restarting the PCA
        usleep(INIT_SLEEP_DELAY_US);
        i2c_dev->WriteRegisterByte(MODE1, oldmode | RESTART);
    }

    /**
     * The PCA is a 12-bit device, so the on and off registers are separated into high and low bytes, 8-bits for the low
     * and 4-bits for the high. The on register is the point in the PWM cycle when the signal transitions from low to
     * high. The off register is the point in the PWM cycle when the signal transitions from high to low. Therefore,
     * each channel has 4 registers that need to be written to set the PWM signal.
     * 
     * @throw std::system_error if an error occurs during the I2C communication.
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

}  // namespace pca9685_hardware_interface