# Hardware Interfaces

## Adding a New Hardware Interface
Follow these steps to define a new hardware interface within the `src/hardware/` directory:
1) Define the C++ header file in `include/hardware/` and the C++ source file in `src/` for the interface you are implementing. Ensure you override the necessary functions from one of the ros2_control `hardware_interface` base classes, and that you are exporting your class as a plugin.

2) Add the interface class to `hardware.xml`. Include any additional dependencies in the `package.xml` and project DockerFile. If these dependancies require certain compile or linker flags, modify `CMakeList.txt` accordingly.

3) Associate your interface to particular joints within `ros2_control.xacro` and specify the necessary parameters.


## CL42TV4.1 Closed Loop Stepper Motor Driver

### Product Manual
https://www.omc-stepperonline.com/index.php?route=product/product/get_file&file=3272/CL42T-V41%20Manual.pdf

### Description
An actuator interface designed for position control of a single joint.

**Hardware parameters:** confgurable values of the CLT42TV4.1 driver. Note that values are assumed to match the physical configuration. Hardware Parameters include
- `pulses_per_rev`: The Pulse/Rev setting determined by the physical configuration of SW1-4.

**GPIO parameters:** Parameters corresponding to the remaining hardware system (PCBs and microcontroller) interact with the CLT42TV4.1 through GPIO. In accordance with our PCB design, each stepper motor can be configured with GPIO pins for the PUL+, DIR+, ENA+, and ALM connections, and this interface requires pins be specified in this order. The GPIO parameters are based on the requirements of the `libgpiod-dev` library, and are specified within gpio tags in the xacro file for clarity. GPIO parameters include:
- `chip_name`: The name of the device managing the pin
- `pin_number`: The GPIO pin number
- `descriptor`: The pin description (valid options include: "PUL+", "DIR+", "ENA+", or "ALM")
- `direction`: The IO direction (valid options include:"ouput" or "input")
- `init_value`: The initial value to set (only relevant for output pins)

### Limitations
Currently, this interface supports the motor operation in single pulse mode using only the PUL+ and DIR+ pins. Inclusion of the remaining GPIO pins can be added later, and will likely require overiding additionally methods of the `ActuatorInterface` class, depending on how these signals impact the lifecycle of the interface. Furthermore, the consideration of hardware parameters specifying other switch settings needs to be added.
