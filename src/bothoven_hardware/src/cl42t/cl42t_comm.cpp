#include "bothoven_hardware/cl42t/cl42t_comm.hpp"
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <unistd.h>

namespace hardware {

Cl42tComm::Cl42tComm(const std::string &device) : device_(device) {
  open_bus(device_);
}

void Cl42tComm::open_bus(const std::string &device) {
  spi_fd_ = open(device.c_str(), O_RDWR);
  if (spi_fd_ < 0) {
    std::ostringstream error_message;
    error_message << "Error opening SPI device: " << device;
    throw std::runtime_error(error_message.str());
  }
}

void Cl42tComm::shutdown() {
  if (spi_fd_ >= 0) {
    close(spi_fd_);
    spi_fd_ = -1;
  }
}

void Cl42tComm::init_peripheral(uint8_t bits, uint32_t speed_hz, uint8_t mode) {
  bits_per_word_ = bits;
  bus_speed_hz_ = speed_hz;

  switch (mode) {
  case 0:
    mode_ = SPI_MODE_0;
    break;
  case 1:
    mode_ = SPI_MODE_1;
    break;
  case 2:
    mode_ = SPI_MODE_2;
    break;
  case 3:
    mode_ = SPI_MODE_3;
    break;
  default:
    mode_ = SPI_MODE_0;
    break;
  }

  set_mode(mode_);
  set_bits_per_word(bits);
  set_speed(speed_hz);

  // set MSB first
  uint8_t lsb = 0;
  if (ioctl(spi_fd_, SPI_IOC_WR_LSB_FIRST, &lsb) < 0) {
    close(spi_fd_);
    throw std::runtime_error("Failed to set bit order");
  }
}

bool Cl42tComm::send_command(const std::string &command) {
  std::vector<uint8_t> tx_data(command.begin(), command.end());
  std::vector<uint8_t> rx_data(tx_data.size());

  if (tx_data.empty())
    return false;

  struct spi_ioc_transfer transfer = {};
  transfer.tx_buf = reinterpret_cast<uintptr_t>(tx_data.data());
  transfer.rx_buf = reinterpret_cast<uintptr_t>(rx_data.data());
  transfer.len = tx_data.size();
  transfer.delay_usecs = 0;
  transfer.speed_hz = bus_speed_hz_;
  transfer.bits_per_word = bits_per_word_;

  if (ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &transfer) < 0) {
    throw std::runtime_error("SPI transfer failed");
  }
  return true;
}

std::vector<uint8_t> Cl42tComm::read_response(size_t response_size) {
  std::vector<uint8_t> tx_dummy(response_size, 0xFF);
  std::vector<uint8_t> rx_data(response_size);

  struct spi_ioc_transfer transfer {};
  transfer.tx_buf = reinterpret_cast<uintptr_t>(tx_dummy.data());
  transfer.rx_buf = reinterpret_cast<uintptr_t>(rx_data.data());
  transfer.len = response_size;
  transfer.delay_usecs = 0;
  transfer.speed_hz = bus_speed_hz_;
  transfer.bits_per_word = bits_per_word_;

  if (ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &transfer) < 0) {
    throw std::runtime_error("SPI transfer failed");
  }

  // std::ostringstream oss;
  // oss << "Received [" << response_size << " bytes]:";
  // for (uint8_t b : rx_data) {
  //   oss << ' ' << std::hex << std::uppercase << std::setw(2)
  //       << std::setfill('0') << static_cast<int>(b);
  // }
  // oss << std::dec;
  // RCLCPP_INFO_STREAM(rclcpp::get_logger("Cl42tSystemHardware"), oss.str());

  return rx_data;
}

float Cl42tComm::get_position(char motor) {
  std::string pos_req = "g"; // get
  pos_req += motor;
  pos_req.resize(6, '\0'); // pico expects 6 byte request messages
  bool success = send_command(pos_req);
  if (!success) {
    return std::numeric_limits<float>::quiet_NaN();
  }

  rclcpp::sleep_for(std::chrono::milliseconds(
      1)); // give pico time to load position into MISO

  std::vector<uint8_t> pos_res;
  try {
    pos_res = read_response(4);
  } catch (std::exception &e) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("Cl42tSystemHardware"),
                       "Error reading position for motor " << motor << ": "
                                                           << e.what());
    return std::numeric_limits<float>::quiet_NaN();
  }

  float pos_value;
  std::memcpy(&pos_value, pos_res.data(), sizeof(pos_value));
  return pos_value;
}

bool Cl42tComm::set_velocity(char motor, float vel_mps) {
  std::string vel_req;
  vel_req.resize(6, '\0');
  vel_req[0] = 's';
  vel_req[1] = motor;

  std::memcpy(&vel_req[2], &vel_mps, sizeof(vel_mps));

  RCLCPP_INFO_STREAM(rclcpp::get_logger("Cl42tSystemHardware"),
                     "Sending: " << vel_mps);

  bool success;
  try {
    success = send_command(vel_req);
  } catch (std::exception &e) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("Cl42tSystemHardware"),
                       "Error sending velocity to pico " << motor << ": "
                                                         << e.what());
    return std::numeric_limits<float>::quiet_NaN();
  }
  return success;
}

void Cl42tComm::set_mode(uint8_t mode) {
  mode_ = mode;
  if (ioctl(spi_fd_, SPI_IOC_WR_MODE, &mode_) < 0) {
    throw std::runtime_error("Failed to set SPI mode");
  }
}

void Cl42tComm::set_speed(uint32_t speed_hz) {
  bus_speed_hz_ = speed_hz;
  if (ioctl(spi_fd_, SPI_IOC_WR_MAX_SPEED_HZ, &bus_speed_hz_) < 0) {
    throw std::runtime_error("Failed to set SPI speed");
  }
}

void Cl42tComm::set_bits_per_word(uint8_t bits) {
  bits_per_word_ = bits;
  if (ioctl(spi_fd_, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word_) < 0) {
    throw std::runtime_error("Failed to set SPI bits per word");
  }
}

} // namespace hardware
