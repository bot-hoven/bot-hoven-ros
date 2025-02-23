#ifndef SPI_CONSOLE_H
#define SPI_CONSOLE_H

#include "SPIPeripheral.h"
#include <string>
#include <vector>

constexpr uint32_t SPI_WRITE_DELAY = 100; // microseconds

namespace spi_console {

/**
 * @brief Prints hexadecimal representation of a string
 * @param s The input string to print in hex format
 */
void print_hex(const std::string& s);

/**
 * @brief Sends a command over SPI with proper formatting
 * @param spi SPI peripheral instance
 * @param command The command string to send
 */
void send_command(hardware::SPIPeripheral& spi, const std::string& command);

/**
 * @brief Reads response from SPI device
 * @param spi SPI peripheral instance
 * @param max_response_size Maximum allowed response size
 * @return Received response as string
 */
std::string read_response(hardware::SPIPeripheral& spi, size_t max_response_size = 32);

/**
 * @brief Main application loop
 * @param spi Initialized SPI peripheral instance
 */
void run_application(hardware::SPIPeripheral& spi);

} // namespace spi_console

#endif // SPI_CONSOLE_H