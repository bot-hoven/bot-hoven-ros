#include "spi_console.h"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>

namespace spi_console {

    using namespace std::chrono_literals;

    void print_hex(const std::string& s) {
        for (size_t i = 0; i < s.size(); i++) {
            std::cout << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
                      << static_cast<int>(static_cast<unsigned char>(s[i])) << " ";
        }
    }

    void send_command(hardware::SPIPeripheral& spi, const std::string& command) {
        try {
            std::string cmd = command;
            if (cmd.empty() || cmd.back() != '\0') {
                cmd.push_back('\0');
            }

            const uint8_t* data_ptr = reinterpret_cast<const uint8_t*>(cmd.data());
            for (size_t i = 0; i < cmd.size(); ++i) {
                spi.Transfer(data_ptr + i, nullptr, 1);
            }

            std::cout << "Sent: " << cmd << " (";
            print_hex(cmd);
            std::cout << ")\n";
        } catch (const std::exception& e) {
            std::cerr << "Error sending command: " << e.what() << '\n';
            throw;
        }
    }

    std::string read_response(hardware::SPIPeripheral& spi, size_t max_response_size) {
        std::vector<uint8_t> rx_buffer;
        std::vector<uint8_t> tx_dummy(1, 0xFF);

        try {
            uint8_t rx_byte;
            do {
                spi.Transfer(tx_dummy.data(), &rx_byte, 1);
                rx_buffer.push_back(rx_byte);
            } while (rx_byte != '\0' && rx_buffer.size() < max_response_size);

            std::cout << "Received raw bytes: ";
            for (auto b : rx_buffer) {
                std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(b) << " ";
            }
            std::cout << std::dec << "\n";

            // Convert to string (may contain embedded nulls)
            std::string response(reinterpret_cast<char*>(rx_buffer.data()), rx_buffer.size());

            // Print with escaped nulls
            std::cout << "As string: \"";
            for (char c : response) {
                if (c == '\0')
                    std::cout << "\\0";
                else if (isprint(c))
                    std::cout << c;
                else
                    std::cout << "\\x" << std::hex << static_cast<int>(c);
            }
            std::cout << "\"\n";

            return std::string(reinterpret_cast<char*>(rx_buffer.data()), rx_buffer.size());
        } catch (const std::exception& e) {
            std::cerr << "Read error: " << e.what() << '\n';
            throw;
        }
    }

    void run_application(hardware::SPIPeripheral& spi) {
        // send_command(spi, ""); // Initial sync

        while (true) {
            std::cout << "Enter message (or 'exit'):\n";
            std::string input;
            std::getline(std::cin, input);

            if (input.empty())
                continue;
            if (input == "exit")
                break;

            std::string processed_cmd = input;
            if (processed_cmd.back() != '\0') {
                processed_cmd.push_back('\0');
            }

            std::cout << "Input Command: '" << processed_cmd << "' (";
            print_hex(processed_cmd);
            std::cout << ")\n" << std::dec;

            if (input == "rl" || input == "rr") {
                send_command(spi, processed_cmd);
                // std::this_thread::sleep_for(std::chrono::microseconds(SPI_WRITE_DELAY));
                read_response(spi);
            } else {
                send_command(spi, processed_cmd);
            }
        }
    }

} 
