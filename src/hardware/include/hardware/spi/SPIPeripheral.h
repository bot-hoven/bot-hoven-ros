#ifndef HARDWARE__SPI__SPIPERIPHERAL_H
#define HARDWARE__SPI__SPIPERIPHERAL_H

#include <cstdint>
#include <string>
#include <stdexcept>

namespace hardware {

class SPIPeripheral {
public:
    /**
     * Constructor.
     * @param device The SPI device file (e.g., "/dev/spidev0.0").
     */
    SPIPeripheral(const std::string &device);
    
    ~SPIPeripheral();

    /**
     * Initialize the SPI peripheral.
     * @param chip_select An optional chip select value.
     * @param bits Bits per word (e.g., 8).
     * @param speed_hz SPI bus speed in Hz (e.g., 500000).
     * @throw std::runtime_error if the SPI device cannot be configured.
     */
    void InitPeripheral(uint8_t chip_select = 0, uint8_t bits = 8, uint32_t speed_hz = 500000);

    /**
     * Write a block of data to the SPI bus.
     * @param data Pointer to the data buffer.
     * @param length Number of bytes to write.
     * @throw std::runtime_error if the write fails.
     */
    void WriteData(const uint8_t* data, size_t length);

    /**
     * Read a block of data from the SPI bus.
     * @param data Pointer to the buffer where data will be stored.
     * @param length Number of bytes to read.
     * @throw std::runtime_error if the read fails.
     */
    void ReadData(uint8_t* data, size_t length);

    /**
     * Transfer data over SPI (simultaneously send tx_data and receive into rx_data).
     * @param tx_data Pointer to data to send.
     * @param rx_data Pointer to buffer for received data.
     * @param length Number of bytes to transfer.
     * @throw std::runtime_error if the transfer fails.
     */
    void Transfer(const uint8_t* tx_data, uint8_t* rx_data, size_t length);

    // Optional setters for SPI parameters
    void SetMode(uint8_t mode);
    void SetSpeed(uint32_t speed_hz);
    void SetBitsPerWord(uint8_t bits);

private:
    int fd_;
    std::string device_;
    uint8_t chip_select_;  // For custom chip select handling

    uint8_t mode_;
    uint8_t bits_per_word_;
    uint32_t bus_speed_hz_;

    void OpenBus(const std::string &device);
    void CloseBus();
};

} // namespace hardware

#endif // HARDWARE__SPI__SPIPERIPHERAL_H
