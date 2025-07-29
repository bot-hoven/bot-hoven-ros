#include "bothoven_hardware/spi/SPIPeripheral.h"

namespace hardware {

    SPIPeripheral::SPIPeripheral(const std::string &device)
        : device_(device)
    {
        open_bus(device_);
    }

    void SPIPeripheral::open_bus(const std::string &device) {
        fd_ = open(device.c_str(), O_RDWR);
        if (fd_ < 0) {
            std::ostringstream error_message;
            error_message << "Error opening SPI device: " << device;
            throw std::runtime_error(error_message.str());
        }
    }

    void SPIPeripheral::close_bus() {
        if (fd_ >= 0) {
            close(fd_);
            fd_ = -1;
        }
    }

    void SPIPeripheral::init_peripheral(uint8_t bits, uint32_t speed_hz, uint8_t mode) {
        bits_per_word_ = bits;
        bus_speed_hz_ = speed_hz;

        switch (mode) {
        case 0:mode_ = SPI_MODE_0; break;
        case 1:mode_ = SPI_MODE_1; break;
        case 2:mode_ = SPI_MODE_2; break;
        case 3: mode_ = SPI_MODE_3; break;
        default:mode_ = SPI_MODE_0; break;
        }

        SetMode(mode_);
        SetBitsPerWord(bits);
        SetSpeed(speed_hz);

        // set MSB first
        uint8_t lsb = 0;
        if (ioctl(spi_fd, SPI_IOC_WR_LSB_FIRST, &lsb) < 0) {
            close(spi_fd);
            throw std::runtime_error("Failed to set bit order");
        }
    }

    void SPIPeripheral::send_command(const std::string& command) {
        std::string cmd_with_null = command;
        if (cmd_with_null.back() != '\0') {
            cmd_with_null += '\0';
        }
        std::vector<uint8_t> tx_data(cmd_with_null.begin(), cmd_with_null.end());
        std::vector<uint8_t> rx_data(tx_data.size());
        transfer_data(tx_data, rx_data);

        if (tx_data.empty()) return;
        
        rx_data.resize(tx_data.size());
        
        struct spi_ioc_transfer transfer = {};
        transfer.tx_buf = reinterpret_cast<uintptr_t>(tx_data.data());
        transfer.rx_buf = reinterpret_cast<uintptr_t>(rx_data.data());
        transfer.len = tx_data.size();
        transfer.delay_usecs = delay;
        transfer.speed_hz = speed;
        transfer.bits_per_word = bits;
        
        if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &transfer) < 0) {
            throw std::runtime_error("SPI transfer failed");
        }
    }

    std::string read_response(int max_response_size) {
        std::vector<uint8_t> tx_dummy(max_response_size, 0xFF);
        std::vector<uint8_t> rx_response(max_response_size);
        
        struct spi_ioc_transfer resp_transfer = {};
        resp_transfer.tx_buf = reinterpret_cast<uintptr_t>(tx_dummy.data());
        resp_transfer.rx_buf = reinterpret_cast<uintptr_t>(rx_response.data());
        resp_transfer.len = max_response_size;
        resp_transfer.delay_usecs = 0;
        resp_transfer.speed_hz = speed;
        resp_transfer.bits_per_word = bits;
        
        if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &resp_transfer) < 0) {
            throw std::runtime_error("SPI response transfer failed");
        }
        
        std::string response_str;
        for (uint8_t byte : rx_response) {
            if (byte == 0) break;  // null terminator
            if (byte >= 32 && byte <= 126) {  // printable ASCII
                response_str += static_cast<char>(byte);
            }
        }
        return response_str;
    }

    double get_position(char motor) {        
        std::string pos_req = "r"; // read
        pos_req += motor;
        
        std::string response;
        try {
            send_command(pos_req);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            response = read_response(32, verbose);
        } catch (const std::exception& e) {
            if (verbose) {
                std::cerr << "Error reading position for motor " << motor << ": " << e.what() << std::endl;
            }
            response = ""; 
        }
        
        if (response.empty()) {
            return std::numeric_limits<double>::quiet_NaN();
        }
    
        try {
            return std::stod(pos_str);
        } catch (const std::exception&) {
            return std::numeric_limits<double>::quiet_NaN();
        }
    }

    void SPIPeripheral::set_mode(uint8_t mode) {
        mode_ = mode;
        if (ioctl(fd_, SPI_IOC_WR_MODE, &mode_) < 0) {
            throw std::runtime_error("Failed to set SPI mode");
        }
    }

    void SPIPeripheral::set_speed(uint32_t speed_hz) {
        bus_speed_hz_ = speed_hz;
        if (ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &bus_speed_hz_) < 0) {
            throw std::runtime_error("Failed to set SPI speed");
        }
    }

    void SPIPeripheral::set_bits_per_word(uint8_t bits) {
        bits_per_word_ = bits;
        if (ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word_) < 0) {
            throw std::runtime_error("Failed to set SPI bits per word");
        }
    }

}  // namespace hardware
