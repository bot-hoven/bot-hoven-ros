#ifndef HARDWARE__I2C__I2CPERIPHERAL_H_
#define HARDWARE__I2C__I2CPERIPHERAL_H_

#include <cstdint>
#include <string>

namespace hardware {

class I2CPeripheral {
public:
  I2CPeripheral(const std::string& device, const uint8_t address);
  ~I2CPeripheral();

  void WriteRegisterByte(const uint8_t register_address, const uint8_t value);

  uint8_t ReadRegisterByte(const uint8_t register_address);

private:
  int bus_fd;

  void OpenBus(const std::string& device);
  void ConnectToPeripheral(const uint8_t address);

};

}  // namespace hardware

#endif  // HARDWARE__I2C__I2CPERIPHERAL_H_