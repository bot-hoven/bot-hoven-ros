#ifndef HARDWARE__CL42T__CL42T_COMM_H
#define HARDWARE__CL42T__CL42T_COMM_H

#include <cstdint>
#include <string>
#include <stdexcept>
#include <memory>

#include "hardware/spi/SPIPeripheral.h"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"

namespace cl42t_hardware_interface {

class CL42TComm {
public:
    CL42TComm();
    ~CL42TComm();

    /**
     * Set up the communication interface using an SPI peripheral.
     * @param spi_dev A pointer to the SPI peripheral object.
     */
    void setup(hardware::SPIPeripheral* spi_dev, uint8_t chip_select, uint8_t bits_per_word, uint32_t bus_speed_hz);
    void init();
    // void CL42TComm::close();


    /**
     * Send a command string via SPI.
     * The command is expected to be a null-terminated string, e.g. "p l 200\\0".
     * @param command The command string to send.
     * @throw std::runtime_error if the SPI write fails.
     */
    void send_command(const std::string &command);

private:
    // Unique pointer to manage the SPI peripheral.
    // This unique_ptr takes ownership of the raw pointer passed in setup().
    std::unique_ptr<hardware::SPIPeripheral> spi_dev_;
    uint8_t chip_select_;  // For custom chip select handling

    uint8_t mode_;
    uint8_t bits_per_word_;
    uint32_t bus_speed_hz_;
};

} // namespace cl42t_hardware_interface

#endif // HARDWARE__CL42T__CL42T_COMM_H
