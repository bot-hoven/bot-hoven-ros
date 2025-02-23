#include "hardware/cl42t/cl42t_comm.h"

// System includes for SPI operations.
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <thread>
#include <chrono>

namespace cl42t_hardware_interface {

    CL42TComm::CL42TComm() : spi_dev_(nullptr), bits_per_word_(0), bus_speed_hz_(0), mode_(0) {}

    CL42TComm::~CL42TComm() {}

    void CL42TComm::setup(hardware::SPIPeripheral* spi_dev, uint8_t bits_per_word, uint32_t bus_speed_hz, int mode) {
        if (spi_dev == nullptr) {
            throw std::runtime_error("Invalid SPI peripheral pointer.");
        }
        // Transfer ownership of spi_dev.
        spi_dev_.reset(spi_dev);
        bits_per_word_ = bits_per_word;
        bus_speed_hz_ = bus_speed_hz;
        mode_ = mode;
    }

    void CL42TComm::init() {
        if (!spi_dev_) {
            throw std::runtime_error("SPI peripheral not initialized in CL42TComm.");
        }
        // Initialize the SPI peripheral with the stored parameters.
        spi_dev_->InitPeripheral(bits_per_word_, bus_speed_hz_, mode_);
    }

    void CL42TComm::send_command(const std::string& stepper_side, double position) {
        if (!spi_dev_) {
            throw std::runtime_error("SPI peripheral not initialized in CL42TComm.");
        }

        // Build the output message
        std::ostringstream oss;
        oss << "p" << stepper_side << position;
        std::string command = oss.str();

        // Ensure the command string is null-terminated.
        std::string cmd = command;
        if (cmd.empty() || cmd.back() != '\0') {
            cmd.push_back('\0');
        }
        // Write the command via SPI.
        spi_dev_->WriteData(reinterpret_cast<const uint8_t*>(cmd.c_str()), cmd.size());
    }

    double CL42TComm::read_position(const std::string& stepper_side) {
        const size_t MAX_RESPONSE_LEN = 32;
        double position = -1;
        std::vector<uint8_t> rx_buffer;
        std::vector<uint8_t> tx_dummy(1, 0xFF);
        if (!spi_dev_) {
            throw std::runtime_error("SPI peripheral not initialized.");
        }

        // Build the command (e.g., "rl\0")
        std::string command = "r" + stepper_side;
        // Ensure the command string is null-terminated.
        if (command.empty() || command.back() != '\0') {
            command.push_back('\0');
        }

        try {
            // Send read command for stepper side
            spi_dev_->Transfer(reinterpret_cast<const uint8_t*>(command.c_str()), nullptr, command.size());

            // Allow response preparation time
            std::this_thread::sleep_for(std::chrono::microseconds(100));

            uint8_t rx_byte;
            do {
                spi_dev_->Transfer(tx_dummy.data(), &rx_byte, 1);
                rx_buffer.push_back(rx_byte);
            } while (rx_byte != '\0' && rx_buffer.size() < MAX_RESPONSE_LEN);

            // Convert to string (may contain embedded nulls)
            std::string response(reinterpret_cast<char*>(rx_buffer.data()), rx_buffer.size());

            // Trim the response: remove any characters that are not digits, a decimal point, or a sign.
            std::string trimmed_response;
            for (char c : response) {
                if ((c >= '0' && c <= '9') || c == '.' || c == '-' || c == '+') {
                    trimmed_response.push_back(c);
                }
            }

            // Convert the trimmed string to a double.
            char* end;
            position = std::strtod(trimmed_response.c_str(), &end);
            if (end == trimmed_response.c_str()) {
                std::ostringstream error_message;
                error_message << "Failed to convert response to double: " << trimmed_response;
                throw std::runtime_error(error_message.str());
            }
        } catch (const std::exception& e) {
            // std::cerr << "Error: " << e.what() << std::endl;
            return -1;
        }

        // // Send command + read response in one transaction.

        // std::vector<uint8_t> tx_data(command.begin(), command.end());
        // tx_data.resize(MAX_RESPONSE_LEN, 0xFF);  // Pad with dummy bytes

        // std::vector<uint8_t> rx_data(tx_data.size());
        // spi_dev_->Transfer(tx_data.data(), rx_data.data(), tx_data.size());

        // // Extract a null-terminated string from the received data.
        // std::string response;
        // for (uint8_t byte : rx_data) {
        //     if (byte == '\0') break;
        //     response.push_back(static_cast<char>(byte));
        // }

        // if (response.empty()) {
        //     // throw std::runtime_error("Received an empty response from SPI.");
        //     response = "-1\0";
        // }

        // // Debug: Print the raw response
        // // RCLCPP_INFO(get_logger(), "Raw response: %s", response.c_str());

        // // Trim the response: remove any characters that are not digits, a decimal point, or a sign.
        // std::string trimmed_response;
        // for (char c : response) {
        //     if ((c >= '0' && c <= '9') || c == '.' || c == '-' || c == '+') {
        //         trimmed_response.push_back(c);
        //     }
        // }

        // // Debug: Print the trimmed response
        // // RCLCPP_INFO(get_logger(), "Trimmed response: %s", trimmed_response.c_str());

        // // Convert the trimmed string to a double.
        // char* end;
        // double position = std::strtod(trimmed_response.c_str(), &end);
        // if (end == trimmed_response.c_str()) {
        //     std::ostringstream error_message;
        //     error_message << "Failed to convert response to double: " << trimmed_response;
        //     throw std::runtime_error(error_message.str());
        // }

        return position;
    }

    // double CL42TComm::read_position(const std::string &stepper_side) {
    //     if (!spi_dev_) {
    //         throw std::runtime_error("SPI peripheral not initialized.");
    //     }

    //     // Build the command (e.g., "r l\0")
    //     std::string command = "r " + stepper_side + '\0';

    //     // Send command + read response in one transaction
    //     const size_t MAX_RESPONSE_LEN = 32;
    //     std::vector<uint8_t> tx_data(command.begin(), command.end());
    //     tx_data.resize(MAX_RESPONSE_LEN, 0xFF);  // Pad with dummy bytes

    //     std::vector<uint8_t> rx_data(tx_data.size());
    //     spi_dev_->Transfer(tx_data.data(), rx_data.data(), tx_data.size());

    //     // Extract null-terminated string
    //     std::string response;
    //     for (uint8_t byte : rx_data) {
    //         if (byte == '\0') break;
    //         response += static_cast<char>(byte);
    //     }

    //     if (response.empty()) {
    //         return -1;
    //     }

    //     // Convert to double
    //     char* end;
    //     double position = std::strtod(response.c_str(), &end);
    //     if (end == response.c_str()) {
    //         std::ostringstream error_message;
    //         error_message << "Failed to convert response to double: " << response;
    //         throw std::runtime_error(error_message.str());
    //         // throw std::runtime_error("Failed to convert response to double.");
    //     }

    //     // // Pi (receive binary):
    //     // float pos_float;
    //     // memcpy(&pos_float, rx_data.data(), sizeof(float));
    //     // double position = pos_float;

    //     return position;
    // }

}  // namespace cl42t_hardware_interface
