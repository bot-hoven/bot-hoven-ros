#include "hardware/cl42t/cl42t_comm.h"
#include "hardware/spi/SPIPeripheral.h"
#include <stdexcept>
#include <string>

namespace cl42t_hardware_interface {

    CL42TComm::CL42TComm() : spi_dev_(nullptr) {}

    CL42TComm::~CL42TComm() {}


    void CL42TComm::setup(hardware::SPIPeripheral* spi_dev, uint8_t chip_select, uint8_t bits_per_word, uint32_t bus_speed_hz) {
        if (spi_dev == nullptr) {
            throw std::runtime_error("Invalid SPI peripheral pointer.");
        }

        // Transfer ownership of spi_dev to our unique_ptr.
        spi_dev_.reset(spi_dev);

        chip_select_ = chip_select;  // For custom chip select handling
        bits_per_word_ = bits_per_word;
        bus_speed_hz_ = bus_speed_hz;
    }

    void CL42TComm::init() {
        if (!spi_dev_) {
            throw std::runtime_error("SPI peripheral not initialized in CL42TComm.");
        }
        spi_dev_->InitPeripheral(chip_select_, bits_per_word_, bus_speed_hz_);
    }

    // void CL42TComm::close() {
    //     spi_dev_.reset();
    // }

    void CL42TComm::send_command(const std::string &command) {
        if (!spi_dev_) {
            throw std::runtime_error("SPI peripheral not initialized in CL42TComm.");
        }
        // Ensure the command string is null-terminated.
        std::string cmd = command;
        if (cmd.empty() || cmd.back() != '\0') {
            cmd.push_back('\0');
        }
        // Write the command via SPI.
        spi_dev_->WriteData(reinterpret_cast<const uint8_t*>(cmd.c_str()), cmd.size());
    }

} // namespace cl42t_hardware_interface
