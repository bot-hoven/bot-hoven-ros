#include "hardware/spi/SPIPeripheral.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sstream>
#include <linux/spi/spidev.h>
#include <cstring>  // for memset

namespace hardware {

SPIPeripheral::SPIPeripheral(const std::string &device)
    : device_(device)
{
    OpenBus(device_);
}

SPIPeripheral::~SPIPeripheral() {
    CloseBus();
}

void SPIPeripheral::OpenBus(const std::string &device) {
    fd_ = open(device.c_str(), O_RDWR);
    if (fd_ < 0) {
        std::ostringstream error_message;
        error_message << "Error opening SPI device: " << device;
        throw std::runtime_error(error_message.str());
    }
}

void SPIPeripheral::CloseBus() {
    if (fd_ >= 0) {
        close(fd_);
        fd_ = -1;
    }
}

void SPIPeripheral::InitPeripheral(uint8_t chip_select, uint8_t bits, uint32_t speed_hz) {
    // Set the peripheral configuration parameters
    chip_select_ = chip_select; // Not used automatically by spidev; managed by device file normally.
    mode_ = SPI_MODE_0;
    bits_per_word_ = bits;
    bus_speed_hz_ = speed_hz;

    // Configure the SPI peripheral using the provided setters.
    SetMode(SPI_MODE_0);
    SetBitsPerWord(bits);
    SetSpeed(speed_hz);
}

void SPIPeripheral::WriteData(const uint8_t* data, size_t length) {
    ssize_t ret = write(fd_, data, length);
    if (ret != static_cast<ssize_t>(length)) {
        throw std::runtime_error("Failed to write SPI data");
    }
}

void SPIPeripheral::ReadData(uint8_t* data, size_t length) {
    ssize_t ret = read(fd_, data, length);
    if (ret != static_cast<ssize_t>(length)) {
        throw std::runtime_error("Failed to read SPI data");
    }
}

void SPIPeripheral::Transfer(const uint8_t* tx_data, uint8_t* rx_data, size_t length) {
    struct spi_ioc_transfer transfer;
    std::memset(&transfer, 0, sizeof(transfer));
    
    transfer.tx_buf = reinterpret_cast<unsigned long>(tx_data);
    transfer.rx_buf = reinterpret_cast<unsigned long>(rx_data);
    transfer.len = length;
    transfer.speed_hz = bus_speed_hz_;
    transfer.bits_per_word = bits_per_word_;
    transfer.delay_usecs = 0;
    
    int ret = ioctl(fd_, SPI_IOC_MESSAGE(1), &transfer);
    if (ret < 1) {
        throw std::runtime_error("Failed to transfer SPI message");
    }
}

void SPIPeripheral::SetMode(uint8_t mode) {
    mode_ = mode;
    if (ioctl(fd_, SPI_IOC_WR_MODE, &mode_) < 0) {
        throw std::runtime_error("Failed to set SPI mode");
    }
}

void SPIPeripheral::SetSpeed(uint32_t speed_hz) {
    bus_speed_hz_ = speed_hz;
    if (ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &bus_speed_hz_) < 0) {
        throw std::runtime_error("Failed to set SPI speed");
    }
}

void SPIPeripheral::SetBitsPerWord(uint8_t bits) {
    bits_per_word_ = bits;
    if (ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word_) < 0) {
        throw std::runtime_error("Failed to set SPI bits per word");
    }
}

} // namespace hardware
