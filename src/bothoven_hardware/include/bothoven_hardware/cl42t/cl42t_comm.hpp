#ifndef HARDWARE__CL42T__CL42T_COMM_HPP
#define HARDWARE__CL42T__CL42T_COMM_HPP

#include <cstdint>
#include <string>
#include <vector>

namespace hardware {

/**
 * @brief A class for SPI peripheral communication.
 *
 * This class provides functions to open an SPI bus, configure its parameters,
 * and perform read, write, and transfer operations.
 */
class Cl42tComm {
public:
  /**
   * @brief Constructor that opens the SPI device.
   * @param device The SPI device file (e.g., "/dev/spidev0.0").
   */
  Cl42tComm(const std::string &device);

  /**
   * @brief Close the SPI device.
   */
  void shutdown();

  /**
   * @brief Initializes the SPI peripheral with configuration parameters.
   * @param bits Number of bits per word.
   * @param speed_hz SPI bus speed in Hz.
   * @throw std::runtime_error if configuration fails.
   */
  void init_peripheral(uint8_t bits = 8, uint32_t speed_hz = 500000,
                       uint8_t mode = 1);

  /**
   * @brief Writes a block of data to the SPI bus.
   * @param data Pointer to the data buffer.
   * @param length Number of bytes to write.
   * @throw std::runtime_error if the write operation fails.
   */
  void write_data(const uint8_t *data, size_t length);

  /**
   * @brief Reads a block of data from the SPI bus.
   * @param data Pointer to the buffer where data will be stored.
   * @param length Number of bytes to read.
   * @throw std::runtime_error if the read operation fails.
   */
  void read_data(uint8_t *data, size_t length);

  /**
   * @brief Interface for a bulk simultaneous read/write.
   * @param command String to send over SPI.
   * @return bool indicating send success
   */
  bool send_command(const std::string &command);

  /**
   * @brief Read an SPI response. Sends dummy bytes on Tx.
   * @param response_size Size (in bytes) of response.
   * @return std::vector<uint8_t> containing response bytes.
   */
  std::vector<uint8_t> read_response(size_t response_size = 32);

  /**
   * @brief Perform a position request over SPI to the pico.
   * @param motor Which motor ('l' or 'r') position the pico should send back.
   */
  float get_position(char motor);

  /**
   * @brief Send motor velocity to the pico.
   * @param motor Which motor ('l' or 'r') position the pico should send back.
   * @param vel_mps Motor velocity output by the PID in meters/sec.
   */
  bool set_velocity(char motor, float vel_mps);

  /**
   * @brief Sets the SPI mode.
   * @param mode The SPI mode (e.g., SPI_MODE_0).
   * @throw std::runtime_error if setting the mode fails.
   */
  void set_mode(uint8_t mode);

  /**
   * @brief Sets the SPI bus speed.
   * @param speed_hz The desired speed in Hz.
   * @throw std::runtime_error if setting the speed fails.
   */
  void set_speed(uint32_t speed_hz);

  /**
   * @brief Sets the number of bits per word for SPI communication.
   * @param bits Number of bits per word.
   * @throw std::runtime_error if setting bits per word fails.
   */
  void set_bits_per_word(uint8_t bits);

  /**
   * Check if SPI communication is active.
   */
  inline bool is_open() { return spi_fd_ >= 0; }

private:
  int spi_fd_;
  std::string device_;
  uint8_t chip_select_;
  uint8_t mode_;
  uint8_t bits_per_word_;
  uint32_t bus_speed_hz_;

  void open_bus(const std::string &device);
};

} // namespace hardware

#endif // HARDWARE__CL42T__CL42T_COMM_HPP
