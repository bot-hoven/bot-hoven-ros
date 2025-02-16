#include "hardware/cl42t/cl42t_comm.h"

// System includes for SPI operations.
#include <stdexcept>
#include <string>
#include <sstream>
#include <cstring>
#include <vector>

namespace cl42t_hardware_interface {

    CL42TComm::CL42TComm() 
        : spi_dev_(nullptr), bits_per_word_(0), bus_speed_hz_(0) 
    {}

    CL42TComm::~CL42TComm() {}

    void CL42TComm::setup(hardware::SPIPeripheral* spi_dev, uint8_t bits_per_word, uint32_t bus_speed_hz) {
        if (spi_dev == nullptr) {
            throw std::runtime_error("Invalid SPI peripheral pointer.");
        }
        // Transfer ownership of spi_dev.
        spi_dev_.reset(spi_dev);
        bits_per_word_ = bits_per_word;
        bus_speed_hz_ = bus_speed_hz;
    }

    void CL42TComm::init() {
        if (!spi_dev_) {
            throw std::runtime_error("SPI peripheral not initialized in CL42TComm.");
        }
        // Initialize the SPI peripheral with the stored parameters.
        spi_dev_->InitPeripheral(bits_per_word_, bus_speed_hz_);
    }

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

    // double CL42TComm::read_position(const std::string &stepper_side) {
    //     if (!spi_dev_) {
    //         throw std::runtime_error("SPI peripheral not initialized in CL42TComm.");
    //     }

    //     std::vector<char> buffer;
    //     char byte = 0;

    //     // Build the read command.
    //     std::ostringstream oss;
    //     oss << "r " << stepper_side;
    //     std::string command = oss.str();

    //     // Ensure the command string is null-terminated.
    //     if (command.empty() || command.back() != '\0') {
    //         command.push_back('\0');
    //     }

    //     // Write the command via SPI.
    //     spi_dev_->WriteData(reinterpret_cast<const uint8_t*>(command.c_str()), command.size());
        
    //     // Read bytes one by one until null terminator is found
    //     do {
    //         spi_dev_->ReadData(reinterpret_cast<uint8_t*>(&byte), 1);
    //         buffer.push_back(byte);
    //     } while (byte != '\0');

    //     // Construct a string from the buffer
    //     std::string response(buffer.data());

    //     // Convert the string to a double
    //     char* end;
    //     double position = std::strtod(response.c_str(), &end);

    //     // Check if the conversion was successful
    //     if (end == response.c_str()) {
    //         throw std::runtime_error("Failed to convert SPI response to double.");
    //     }

    //     return position;

    double CL42TComm::read_position(const std::string &stepper_side) {
        if (!spi_dev_) {
            throw std::runtime_error("SPI peripheral not initialized.");
        }

        // Build the command (e.g., "r l\0")
        std::string command = "r " + stepper_side + '\0';

        // Send command + read response in one transaction
        const size_t MAX_RESPONSE_LEN = 32;
        std::vector<uint8_t> tx_data(command.begin(), command.end());
        tx_data.resize(MAX_RESPONSE_LEN, 0xFF);  // Pad with dummy bytes

        std::vector<uint8_t> rx_data(tx_data.size());
        spi_dev_->Transfer(tx_data.data(), rx_data.data(), tx_data.size());

        // Extract null-terminated string
        std::string response;
        for (uint8_t byte : rx_data) {
            if (byte == '\0') break;
            response += static_cast<char>(byte);
        }

        // Convert to double
        char* end;
        double position = std::strtod(response.c_str(), &end);
        if (end == response.c_str()) {
            throw std::runtime_error("Failed to convert response to double.");
        }

        // // Pi (receive binary):
        // float pos_float;
        // memcpy(&pos_float, rx_data.data(), sizeof(float));
        // double position = pos_float;

        return position;
    }

} // namespace cl42t_hardware_interface
