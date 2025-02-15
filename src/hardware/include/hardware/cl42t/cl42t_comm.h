#ifndef HARDWARE__CL42T__CL42T_COMM_H
#define HARDWARE__CL42T__CL42T_COMM_H

#include <cstdint>
#include <string>
#include <stdexcept>
#include <memory>

// Include only the declaration of the SPI peripheral.
#include "hardware/spi/SPIPeripheral.h"
// Include necessary hardware_interface types.
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"

/**
 * @brief Communication interface for the CL42T stepper motor driver.
 *
 * This class encapsulates SPI communication routines used for sending commands 
 * to the CL42T driver.
 */
namespace cl42t_hardware_interface {

class CL42TComm {
public:
    /**
     * @brief Constructor.
     */
    CL42TComm();

    /**
     * @brief Destructor.
     */
    ~CL42TComm();

    /**
     * @brief Set up the communication interface using an SPI peripheral.
     *
     * This function takes ownership of the provided SPI peripheral pointer and stores 
     * configuration parameters.
     *
     * @param spi_dev Pointer to the SPI peripheral object.
     * @param bits_per_word Number of bits per word.
     * @param bus_speed_hz SPI bus speed in Hz.
     */
    void setup(hardware::SPIPeripheral* spi_dev, uint8_t bits_per_word, uint32_t bus_speed_hz);

    /**
     * @brief Initialize the SPI peripheral with the stored configuration.
     *
     * @throw std::runtime_error if the SPI peripheral is not properly initialized.
     */
    void init();

    /**
     * @brief Send a command string via SPI.
     *
     * The command should be a null-terminated string (e.g., "p l 200\0").
     *
     * @param command The command string to send.
     * @throw std::runtime_error if the SPI write fails.
     */
    void send_command(const std::string &command);

private:
    std::unique_ptr<hardware::SPIPeripheral> spi_dev_;
    uint8_t bits_per_word_;   ///< Number of bits per word.
    uint32_t bus_speed_hz_;   ///< SPI bus speed in Hz.
};

} // namespace cl42t_hardware_interface

#endif // HARDWARE__CL42T__CL42T_COMM_H
