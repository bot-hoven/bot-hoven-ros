#ifndef HARDWARE__SPI__SPIPERIPHERAL_H
#define HARDWARE__SPI__SPIPERIPHERAL_H

#include <cstdint>
#include <string>
#include <stdexcept>

namespace hardware {

/**
 * @brief A class for SPI peripheral communication.
 *
 * This class provides functions to open an SPI bus, configure its parameters,
 * and perform read, write, and transfer operations.
 */
class SPIPeripheral {
public:
    /**
     * @brief Constructor that opens the SPI device.
     * @param device The SPI device file (e.g., "/dev/spidev0.0").
     */
    SPIPeripheral(const std::string &device);
    
    /**
     * @brief Destructor that closes the SPI device.
     */
    ~SPIPeripheral();

    /**
     * @brief Initializes the SPI peripheral with configuration parameters.
     * @param bits Number of bits per word.
     * @param speed_hz SPI bus speed in Hz.
     * @throw std::runtime_error if configuration fails.
     */
    void InitPeripheral(uint8_t bits = 8, uint32_t speed_hz = 500000);

    /**
     * @brief Writes a block of data to the SPI bus.
     * @param data Pointer to the data buffer.
     * @param length Number of bytes to write.
     * @throw std::runtime_error if the write operation fails.
     */
    void WriteData(const uint8_t* data, size_t length);

    /**
     * @brief Reads a block of data from the SPI bus.
     * @param data Pointer to the buffer where data will be stored.
     * @param length Number of bytes to read.
     * @throw std::runtime_error if the read operation fails.
     */
    void ReadData(uint8_t* data, size_t length);

    /**
     * @brief Transfers data over SPI by simultaneously sending and receiving.
     * @param tx_data Pointer to the data to send.
     * @param rx_data Pointer to the buffer for received data.
     * @param length Number of bytes to transfer.
     * @throw std::runtime_error if the transfer operation fails.
     */
    void Transfer(const uint8_t* tx_data, uint8_t* rx_data, size_t length);

    /**
     * @brief Sets the SPI mode.
     * @param mode The SPI mode (e.g., SPI_MODE_0).
     * @throw std::runtime_error if setting the mode fails.
     */
    void SetMode(uint8_t mode);

    /**
     * @brief Sets the SPI bus speed.
     * @param speed_hz The desired speed in Hz.
     * @throw std::runtime_error if setting the speed fails.
     */
    void SetSpeed(uint32_t speed_hz);

    /**
     * @brief Sets the number of bits per word for SPI communication.
     * @param bits Number of bits per word.
     * @throw std::runtime_error if setting bits per word fails.
     */
    void SetBitsPerWord(uint8_t bits);

private:
    int fd_;
    std::string device_;
    uint8_t chip_select_;
    uint8_t mode_;
    uint8_t bits_per_word_;
    uint32_t bus_speed_hz_;

    // Internal functions to manage the SPI bus.
    void OpenBus(const std::string &device);
    void CloseBus();
};

} // namespace hardware

#endif // HARDWARE__SPI__SPIPERIPHERAL_H
