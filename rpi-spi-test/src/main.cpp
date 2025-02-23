#include "spi_console.h"
#include <iostream>

constexpr uint32_t BAUDRATE = 500 * 1000;
constexpr uint8_t SPI_MODE = 1;

int main() {
    try {
        hardware::SPIPeripheral spi("/dev/spidev0.0");
        spi.InitPeripheral(8, BAUDRATE, SPI_MODE);
        spi_console::run_application(spi);
    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << '\n';
        return 1;
    }
    return 0;
}