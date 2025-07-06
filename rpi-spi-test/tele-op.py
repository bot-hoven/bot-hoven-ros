#!/usr/bin/env python3

"""
Robot Teleoperator

A command-line interface for controlling stepper motors, servos, and solenoids
for a robotic system using SPI and I2C communication.
Modified to support both left and right hands with different I2C addresses.
"""

import spidev
import time
import sys
import signal
import threading
import argparse
import os
import re
from datetime import datetime
from smbus2 import SMBus

# SPI configuration
SPI_BUS = 0
SPI_DEVICE = 0
SPI_SPEED = 500000  # 500 kHz
SPI_MODE = 1        # SPI mode (CPOL=0, CPHA=1)

# I2C configuration for left hand
LEFT_I2C_BUS = 1     # Standard I2C bus on Raspberry Pi
LEFT_PCA9685_ADDR = 0x40  # Servo controller
LEFT_MCP23017_ADDR = 0x20  # GPIO extender for solenoids
LEFT_ADS7138_ADDR = 0x10  # ADC

# I2C configuration for right hand
RIGHT_I2C_BUS = 1    # Standard I2C bus on Raspberry Pi
RIGHT_PCA9685_ADDR = 0x41  # Servo controller
RIGHT_MCP23017_ADDR = 0x21  # GPIO extender for solenoids
RIGHT_ADS7138_ADDR = 0x12  # ADC

# Component constants
NUM_SERVOS_PER_HAND = 5      # Number of servo channels per hand (0-4)
NUM_SOLENOIDS_PER_HAND = 5   # Number of solenoid channels per hand (0-4)

# Servo calibration data
# Format: [SLOPE, INTERCEPT] for each servo
SERVO_PWM_CALIBRATION = [
    # Left Hand (0-4)
    [12.14, 519.92],  # Servo 1 (channel 0)
    [12.84, 287.44],  # Servo 2 (channel 1)
    [12.33, 306.41],  # Servo 3 (channel 2)
    [13.20, 319.56],  # Servo 4 (channel 3)
    [13.10, 339.41],  # Servo 5 (channel 4)
    # Right Hand (5-9)
    [13.05, 352.17],  # Servo 6 (channel 5)
    [12.78, 357.69],  # Servo 7 (channel 6)
    [12.96, 411.49],  # Servo 8 (channel 7)
    [12.51, 443.14],  # Servo 9 (channel 8)
    [12.87, 372.07]   # Servo 10 (channel 9)
]

SERVO_ADC_CALIBRATION = [
    # Left Hand (0-4)
    [1.79, 105.23],  # Servo 1 (channel 0)
    [2.39, 103.98],  # Servo 2 (channel 1)
    [2.19, 91.01],   # Servo 3 (channel 2)
    [2.21, 99.48],   # Servo 4 (channel 3)
    [2.23, 102.81],  # Servo 5 (channel 4)
    # Right Hand (5-9)
    [2.23, 103.63],  # Servo 6 (channel 5)
    [2.41, 111.39],  # Servo 7 (channel 6)
    [2.19, 115.01],  # Servo 8 (channel 7)
    [2.18, 117.06],  # Servo 9 (channel 8)
    [2.21, 105.57]   # Servo 10 (channel 9)
]

# Servo constraints
PWM_MIN = 500       # Minimum pulse width in microseconds
PWM_MAX = 2500      # Maximum pulse width in microseconds

# PCA9685 registers
PCA9685_MODE1 = 0x00
PCA9685_MODE2 = 0x01
PCA9685_SUBADR1 = 0x02
PCA9685_SUBADR2 = 0x03
PCA9685_SUBADR3 = 0x04
PCA9685_PRESCALE = 0xFE
PCA9685_LED0_ON_L = 0x06
PCA9685_LED0_ON_H = 0x07
PCA9685_LED0_OFF_L = 0x08
PCA9685_LED0_OFF_H = 0x09
PCA9685_ALL_LED_ON_L = 0xFA
PCA9685_ALL_LED_ON_H = 0xFB
PCA9685_ALL_LED_OFF_L = 0xFC
PCA9685_ALL_LED_OFF_H = 0xFD

# PCA9685 mode bits
PCA9685_RESTART = 0x80
PCA9685_SLEEP = 0x10
PCA9685_ALLCALL = 0x01
PCA9685_INVRT = 0x10
PCA9685_OUTDRV = 0x04
PCA9685_OCH = 0x08

# PCA9685 constants
PCA9685_OSC_CLOCK = 25000000  # 25MHz
PCA9685_RESOLUTION = 4096     # 12-bit resolution

# MCP23017 registers
MCP23017_IODIRA = 0x00
MCP23017_IODIRB = 0x01
MCP23017_GPIOA = 0x12
MCP23017_OLATA = 0x14

# Terminal colors and formatting
class Colors:
    HEADER = '\033[95m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    
    @staticmethod
    def disable():
        """Disable colors for non-terminal environments"""
        Colors.HEADER = ''
        Colors.BLUE = ''
        Colors.CYAN = ''
        Colors.GREEN = ''
        Colors.YELLOW = ''
        Colors.RED = ''
        Colors.ENDC = ''
        Colors.BOLD = ''
        Colors.UNDERLINE = ''

# Check if we're in a terminal that supports colors
if not sys.stdout.isatty():
    Colors.disable()


class RobotTeleoperator:
    """Main controller class for communicating with the robot hardware"""
    
    def __init__(self, spi_bus=SPI_BUS, spi_device=SPI_DEVICE, spi_speed=SPI_SPEED, 
                 spi_mode=SPI_MODE, i2c_bus=LEFT_I2C_BUS):
        """Initialize the communication interfaces to the robot hardware."""
        # Initialize SPI for stepper motors
        self.spi = spidev.SpiDev()
        self.spi.open(spi_bus, spi_device)
        self.spi.max_speed_hz = spi_speed
        self.spi.mode = spi_mode
        self.spi.bits_per_word = 8
        self.spi.lsbfirst = False
        
        # Initialize I2C for servos and solenoids
        self.i2c = SMBus(i2c_bus)
        
        # State tracking
        self.monitoring = False
        self.monitor_thread = None
        self.left_position = None
        self.right_position = None
        
        # Combined for both hands - first 5 are left, second 5 are right
        self.servo_angles = [90] * (NUM_SERVOS_PER_HAND * 2)  # Default servo angles (degrees)
        self.solenoid_states = [0] * (NUM_SOLENOIDS_PER_HAND * 2)  # Default solenoid states (0=OFF)
        
        # Capture Ctrl+C for clean shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        
        # Initialize hardware
        self.init_hardware()
        
        # Print connection info
        self.last_command_time = datetime.now()
        self.print_banner()
        
    def init_hardware(self):
        """Initialize all hardware components for both hands"""
        try:
            # Initialize Left Hand PCA9685 (servo controller)
            self.init_pca9685(LEFT_PCA9685_ADDR)
            print(f"{Colors.GREEN}Left hand servo controller (PCA9685 @ 0x{LEFT_PCA9685_ADDR:02X}) initialized{Colors.ENDC}")
            
            # # Initialize Left Hand MCP23017 (solenoid controller)
            self.init_mcp23017(LEFT_MCP23017_ADDR)
            print(f"{Colors.GREEN}Left hand solenoid controller (MCP23017 @ 0x{LEFT_MCP23017_ADDR:02X}) initialized{Colors.ENDC}")
            
            # Initialize Right Hand PCA9685 (servo controller)
            self.init_pca9685(RIGHT_PCA9685_ADDR)
            print(f"{Colors.GREEN}Right hand servo controller (PCA9685 @ 0x{RIGHT_PCA9685_ADDR:02X}) initialized{Colors.ENDC}")
            
            # Initialize Right Hand MCP23017 (solenoid controller)
            self.init_mcp23017(RIGHT_MCP23017_ADDR)
            print(f"{Colors.GREEN}Right hand solenoid controller (MCP23017 @ 0x{RIGHT_MCP23017_ADDR:02X}) initialized{Colors.ENDC}")
            
            # Set all servos to default positions
            for i in range(NUM_SERVOS_PER_HAND * 2):
                self.set_servo_angle(i, self.servo_angles[i])
                
            # Set all solenoids to OFF
            self.set_all_solenoids(0)
            
        except Exception as e:
            print(f"{Colors.RED}Error initializing hardware: {str(e)}{Colors.ENDC}")
            raise
    
    def init_pca9685(self, address):
        """Initialize a PCA9685 servo controller"""
        # Reset the device
        self.i2c.write_byte_data(address, PCA9685_MODE1, 0x00)
        time.sleep(0.01)  # Wait for reset
        
        # Set totem pole structure (MODE2 register)
        self.i2c.write_byte_data(address, PCA9685_MODE2, PCA9685_OUTDRV | PCA9685_OCH)
        
        # Enable responding to LED All Call I2C-bus address
        self.i2c.write_byte_data(address, PCA9685_MODE1, PCA9685_ALLCALL)
        time.sleep(0.005)  # Wait for oscillator
        
        # Wake up from sleep
        mode1 = self.i2c.read_byte_data(address, PCA9685_MODE1)
        mode1 = mode1 & ~PCA9685_SLEEP  # Clear sleep bit
        self.i2c.write_byte_data(address, PCA9685_MODE1, mode1)
        time.sleep(0.005)  # Wait for oscillator
        
        # Set PWM frequency to 50Hz (standard for servos)
        self.set_pwm_freq(address, 50)
    
    def init_mcp23017(self, address):
        """Initialize an MCP23017 GPIO expander for solenoids"""
        # Set GPIOA pins as outputs
        self.i2c.write_byte_data(address, MCP23017_IODIRA, 0x00)
        
        # Initialize all outputs to 0
        self.i2c.write_byte_data(address, MCP23017_OLATA, 0x00)
    
    def set_pwm_freq(self, address, freq_hz):
        """Set PWM frequency for the PCA9685"""
        # Calculate prescale value
        prescaleval = PCA9685_OSC_CLOCK  # 25MHz
        prescaleval /= PCA9685_RESOLUTION  # 12-bit
        prescaleval /= freq_hz
        prescaleval -= 1.0
        prescale = int(round(prescaleval))
        
        # Read current mode
        oldmode = self.i2c.read_byte_data(address, PCA9685_MODE1)
        
        # Go to sleep mode
        newmode = (oldmode & 0x7F) | PCA9685_SLEEP
        self.i2c.write_byte_data(address, PCA9685_MODE1, newmode)
        
        # Set prescale
        self.i2c.write_byte_data(address, PCA9685_PRESCALE, prescale)
        
        # Restore old mode
        self.i2c.write_byte_data(address, PCA9685_MODE1, oldmode)
        time.sleep(0.005)  # Wait for oscillator
        
        # Set RESTART bit
        self.i2c.write_byte_data(address, PCA9685_MODE1, oldmode | PCA9685_RESTART)
    
    def set_pwm(self, channel, on, off, is_left=True):
        """Set PWM values for a channel"""
        if channel < 0 or channel > 15:
            raise ValueError("Channel must be between 0 and 15")
            
        # Select the correct address
        address = LEFT_PCA9685_ADDR if is_left else RIGHT_PCA9685_ADDR
            
        # Calculate register addresses
        led_on_l = PCA9685_LED0_ON_L + (channel * 4)
        led_on_h = PCA9685_LED0_ON_H + (channel * 4)
        led_off_l = PCA9685_LED0_OFF_L + (channel * 4)
        led_off_h = PCA9685_LED0_OFF_H + (channel * 4)
        
        # Write the PWM values
        self.i2c.write_byte_data(address, led_on_l, on & 0xFF)
        self.i2c.write_byte_data(address, led_on_h, on >> 8)
        self.i2c.write_byte_data(address, led_off_l, off & 0xFF)
        self.i2c.write_byte_data(address, led_off_h, off >> 8)
    
    def set_servo_angle(self, channel, angle):
        """Set servo to specified angle using calibration data"""
        if channel < 0 or channel >= (NUM_SERVOS_PER_HAND * 2):
            raise ValueError(f"Servo channel must be between 0 and {(NUM_SERVOS_PER_HAND * 2)-1}")
            
        # Determine if this is a left or right hand servo
        is_left = channel < NUM_SERVOS_PER_HAND
        
        # Apply offset based on hand
        adjusted_angle = angle
        # if is_left:
        #     adjusted_angle = angle + 90  # Add 90 for left servos
        # else:
        #     adjusted_angle = angle - 90  # Subtract 90 for right servos
            
        # Calculate local channel (0-4) for the appropriate hand
        local_channel = channel if is_left else (channel - NUM_SERVOS_PER_HAND)
                    
        # Get calibration parameters for this servo
        slope = SERVO_PWM_CALIBRATION[channel][0]
        intercept = SERVO_PWM_CALIBRATION[channel][1]
        
        # Calculate PWM in microseconds using the linear calibration
        pwm_us = int((slope * adjusted_angle) + intercept)
        
        # Constrain PWM to safe limits
        pwm_us = max(PWM_MIN, min(PWM_MAX, pwm_us))
        
        # Convert microseconds to PWM values
        # For 50Hz (20ms period), 1ms = 205 counts in 12-bit resolution
        pwm_value = int(pwm_us * 4096 / 20000)
        
        # Update internal state
        self.servo_angles[channel] = angle
        
        # Set the PWM value (always start at 0)
        self.set_pwm(local_channel, 0, pwm_value, is_left)
        
        # Log the conversion
        hand_str = "Left" if is_left else "Right"
        print(f"{Colors.GREEN}Set {hand_str} servo {local_channel} to {angle}° (adjusted: {adjusted_angle}°, pwm: {pwm_us}µs){Colors.ENDC}")
        
        return True
    
    def set_solenoid(self, channel, state):
        """Set a solenoid state"""
        if channel < 0 or channel >= (NUM_SOLENOIDS_PER_HAND * 2):
            raise ValueError(f"Solenoid channel must be between 0 and {(NUM_SOLENOIDS_PER_HAND * 2)-1}")
            
        # Determine if this is left or right hand
        is_left = channel < NUM_SOLENOIDS_PER_HAND
        address = LEFT_MCP23017_ADDR if is_left else RIGHT_MCP23017_ADDR
        
        # Calculate local channel (0-4) for the appropriate hand
        local_channel = channel if is_left else (channel - NUM_SOLENOIDS_PER_HAND)
            
        # Read current state
        current_state = self.i2c.read_byte_data(address, MCP23017_GPIOA)
        
        # Modify the bit for this channel
        if state:
            new_state = current_state | (1 << local_channel)  # Set bit
        else:
            new_state = current_state & ~(1 << local_channel)  # Clear bit
            
        # Write new state
        self.i2c.write_byte_data(address, MCP23017_GPIOA, new_state)
        
        # Update internal state
        self.solenoid_states[channel] = 1 if state else 0
        
        # Log the action
        hand_str = "Left" if is_left else "Right"
        print(f"{Colors.GREEN}Set {hand_str} solenoid {local_channel} to {'ON' if state else 'OFF'}{Colors.ENDC}")
        
        return True
    
    def set_all_solenoids(self, state):
        """Set all solenoids to the same state"""
        value = 0
        if state:
            value = (1 << NUM_SOLENOIDS_PER_HAND) - 1  # All bits set
            
        # Write to GPIO register for left hand
        self.i2c.write_byte_data(LEFT_MCP23017_ADDR, MCP23017_GPIOA, value)
        
        # Write to GPIO register for right hand
        self.i2c.write_byte_data(RIGHT_MCP23017_ADDR, MCP23017_GPIOA, value)
        
        # Update internal state
        self.solenoid_states = [1 if state else 0] * (NUM_SOLENOIDS_PER_HAND * 2)
        
        return True
    
    def print_banner(self):
        """Print application banner with connection information"""
        banner = f"""
{Colors.HEADER}{Colors.BOLD}╔═══════════════════════════════════════════════════════════╗
║ Robot Teleoperator Control System - Dual Hand Support       ║
╚═══════════════════════════════════════════════════════════╝{Colors.ENDC}
{Colors.CYAN}• SPI Connection (Steppers): Bus {SPI_BUS}, Device {SPI_DEVICE}, {SPI_SPEED/1000:.1f} kHz, Mode {SPI_MODE}
• Left Hand I2C Devices:
  - PCA9685 (Servos): 0x{LEFT_PCA9685_ADDR:02X}
  - MCP23017 (Solenoids): 0x{LEFT_MCP23017_ADDR:02X}
  - ADS7138 (ADC): 0x{LEFT_ADS7138_ADDR:02X}
• Right Hand I2C Devices:
  - PCA9685 (Servos): 0x{RIGHT_PCA9685_ADDR:02X}
  - MCP23017 (Solenoids): 0x{RIGHT_MCP23017_ADDR:02X}
  - ADS7138 (ADC): 0x{RIGHT_ADS7138_ADDR:02X}
• Components: {NUM_SERVOS_PER_HAND * 2} Servos, {NUM_SOLENOIDS_PER_HAND * 2} Solenoids, 2 Stepper Motors{Colors.ENDC}

{Colors.GREEN}Enter a command or type 'help' for available options.{Colors.ENDC}
"""
        print(banner)

    def signal_handler(self, sig, frame):
        """Handle Ctrl+C gracefully to ensure proper resource cleanup"""
        print(f"\n{Colors.YELLOW}Interrupt received, shutting down...{Colors.ENDC}")
        self.cleanup()
        sys.exit(0)

    def cleanup(self):
        """Clean up resources before exiting"""
        if self.monitoring:
            self.monitoring = False
            if self.monitor_thread:
                self.monitor_thread.join(timeout=1.0)
                
        # Set all outputs to safe state
        try:
            # Turn off all solenoids
            self.set_all_solenoids(0)
            
            # Move servos to default positions
            for i in range(NUM_SERVOS_PER_HAND * 2):
                self.set_servo_angle(i, 90)
        except Exception as e:
            print(f"{Colors.YELLOW}Warning during cleanup: {str(e)}{Colors.ENDC}")
                
        # Close connections
        try:
            self.spi.close()
            self.i2c.close()
            print(f"{Colors.GREEN}All connections closed. Goodbye!{Colors.ENDC}")
        except:
            pass

    # === Motor Commands ===
    
    def send_command(self, command, wait_for_response=True):
        """Send a command to the Pico over SPI without expecting a response."""
        # Record command time for monitoring
        self.last_command_time = datetime.now()
        
        # Ensure command ends with null terminator
        if not command.endswith('\0'):
            command += '\0'
            
        # Log the command
        cmd_str = command.rstrip('\0')
        print(f"{Colors.BLUE}▶ Sending: '{cmd_str}'{Colors.ENDC}")
        
        # Show hex representation for debugging
        hex_cmd = ' '.join([f'{ord(c):02X}' for c in command])
        print(f"{Colors.CYAN}  Hex: {hex_cmd}{Colors.ENDC}")
        
        # Send each byte individually
        for byte in command:
            self.spi.xfer2([ord(byte)])
            time.sleep(0.0001)  # Small delay between bytes
            
        if wait_for_response:
            print(f"{Colors.GREEN}Command sent successfully{Colors.ENDC}")

    def read_position(self, motor):
        """Read position from a specific motor."""
        # Ensure we have a valid motor code
        motor_code = self.get_motor_code(motor)
        if not motor_code:
            return None
            
        # Build the position query command
        command = f"r{motor_code}"
        
        # Ensure command ends with null terminator
        if not command.endswith('\0'):
            command += '\0'
            
        # Log the command
        cmd_str = command.rstrip('\0')
        print(f"{Colors.BLUE}▶ Sending position query: '{cmd_str}'{Colors.ENDC}")
        
        # Show hex representation for debugging
        hex_cmd = ' '.join([f'{ord(c):02X}' for c in command])
        print(f"{Colors.CYAN}  Hex: {hex_cmd}{Colors.ENDC}")
        
        # Send each byte individually
        for byte in command:
            self.spi.xfer2([ord(byte)])
            time.sleep(0.0001)  # Small delay between bytes
            
        # Wait for Pico to process
        time.sleep(0.01)
        
        # Read response
        response_bytes = []
        valid_bytes = []
        
        # Read bytes until null terminator or max length
        max_response_size = 32
        for _ in range(max_response_size):
            rx_byte = self.spi.xfer2([0xFF])[0]
            response_bytes.append(rx_byte)
            # Skip the first byte which is usually FF or garbage
            if len(response_bytes) > 1 and rx_byte != 0xFF:
                valid_bytes.append(rx_byte)
            if rx_byte == 0:
                break
                
        # Display raw bytes for debugging
        hex_response = ' '.join([f'{b:02X}' for b in response_bytes[:min(16, len(response_bytes))]])
        if len(response_bytes) > 16:
            hex_response += f" ... ({len(response_bytes)} bytes total)"
        print(f"{Colors.CYAN}  Response bytes: {hex_response}{Colors.ENDC}")
        
        # Convert to string, skipping the first byte if it's FF
        try:
            # Find position of null terminator if present
            if 0 in valid_bytes:
                valid_bytes = valid_bytes[:valid_bytes.index(0)]
                
            # Convert bytes to ASCII, filtering out any non-printable characters
            response_str = ''.join(chr(b) for b in valid_bytes if b >= 32 and b <= 126)
            
            # Show response
            if response_str:
                print(f"{Colors.GREEN}◀ Received: '{response_str}'{Colors.ENDC}")
                
                # Try to parse as float
                try:
                    position = float(response_str)
                    
                    # Update stored position
                    if motor_code == 'l':
                        self.left_position = position
                    else:
                        self.right_position = position
                        
                    return position
                except ValueError:
                    print(f"{Colors.RED}Error: Could not parse position from '{response_str}'{Colors.ENDC}")
                    return None
            else:
                print(f"{Colors.YELLOW}◀ Empty response{Colors.ENDC}")
                return None
            
        except Exception as e:
            print(f"{Colors.RED}◀ Error parsing response: {str(e)}{Colors.ENDC}")
            # As a fallback, try to display the raw bytes as ASCII
            fallback_str = ''.join(chr(b) if 32 <= b <= 126 else '.' for b in valid_bytes)
            print(f"{Colors.YELLOW}  Raw ASCII: {fallback_str}{Colors.ENDC}")
            return None

    def calibrate_motors(self):
        """Send the calibration command to both stepper motors with periodic polling"""
        print(f"{Colors.YELLOW}Initiating motor calibration...{Colors.ENDC}")
        print(f"{Colors.YELLOW}This will move both motors to their limits. Ensure path is clear!{Colors.ENDC}")
        
        # Setup for calibration polling
        timeout_seconds = 30  # Increased timeout for full calibration
        poll_interval = 1.0
        start_time = time.time()
        last_poll_time = 0
        
        print(f"{Colors.YELLOW}Waiting for calibration to complete (timeout: {timeout_seconds} seconds)...{Colors.ENDC}")
        
        while True:
            # Check if we need to poll again
            current_time = time.time()
            time_elapsed = current_time - start_time
            time_since_last_poll = current_time - last_poll_time
            
            # If it's time to poll again
            if time_since_last_poll >= poll_interval:
                # Send calibration command 'c' using existing function
                self.send_command('c', wait_for_response=False)
                print(f"{Colors.BLUE}Polling for calibration status... [{time_elapsed:.1f}s elapsed]{Colors.ENDC}")
                
                # Wait for Pico to process
                time.sleep(0.01)
                
                # Check for response
                response_bytes = []
                for _ in range(2):  # Expect 'c' and null terminator
                    rx_byte = self.spi.xfer2([0xFF])[0]
                    response_bytes.append(rx_byte)
                    if rx_byte == 0x00:  # Stop at null terminator
                        break
                
                # Check if we got 'c' (0x63)
                if 0x63 in response_bytes:
                    print(f"{Colors.GREEN}◀ Calibration complete!{Colors.ENDC}")
                    
                    # Add a synchronization sequence after completion
                    print(f"{Colors.CYAN}Synchronizing SPI communication...{Colors.ENDC}")
                    # Send a few dummy bytes to resync
                    for _ in range(3):
                        self.spi.xfer2([0xFF])
                        time.sleep(0.005)
                    
                    return True
                
                # Update last poll time
                last_poll_time = time.time()
            
            # Check for timeout, ensuring we waited at least poll_interval since the last message
            if time_elapsed > timeout_seconds and time_since_last_poll >= poll_interval:
                print(f"{Colors.RED}Calibration timed out after {timeout_seconds} seconds.{Colors.ENDC}")
                return False
            
            # Small sleep to prevent CPU hogging
            time.sleep(0.1)

    def move_motor(self, motor, position):
        """Move a stepper motor to the specified position."""
        # Validate motor parameter
        motor_code = self.get_motor_code(motor)
        if not motor_code:
            return
            
        # Send position command: p<motor_code><position>
        command = f"p{motor_code}{position}"
        self.send_command(command)
        
        print(f"{Colors.GREEN}Command sent to move {motor} motor to {position} meters{Colors.ENDC}")

    def get_position(self, motor):
        """Get the current position of a stepper motor"""
        # Get position using the read_position method
        position = self.read_position(motor)
        
        if position is not None:
            print(f"{Colors.GREEN}{motor.capitalize()} motor position: {position:.6f} meters{Colors.ENDC}")
            return position
        return None

    def home_motors(self):
        """Send the home command to both stepper motors"""
        print(f"{Colors.YELLOW}Sending stepper motors to home position...{Colors.ENDC}")
        
        # Send home command 'h'
        self.send_command('h')
        print(f"{Colors.YELLOW}Home command sent.{Colors.ENDC}")

    def monitor_positions(self):
        """Start continuous monitoring of robot state"""
        if self.monitoring:
            print(f"{Colors.YELLOW}Monitoring is already active{Colors.ENDC}")
            return
            
        self.monitoring = True
    def monitor_positions(self):
        """Start continuous monitoring of robot state"""
        if self.monitoring:
            print(f"{Colors.YELLOW}Monitoring is already active{Colors.ENDC}")
            return
            
        self.monitoring = True
    
    def monitor_loop():
        """Background thread function for monitoring"""
        print(f"{Colors.GREEN}Starting robot monitoring (press Ctrl+C or type 'stop' to end){Colors.ENDC}")
        
        try:
            while self.monitoring:
                # Only query if we haven't received a command recently
                time_since_command = (datetime.now() - self.last_command_time).total_seconds()
                if time_since_command > 0.5:  # Don't interfere with manual commands
                    l_pos = self.read_position('left')
                    r_pos = self.read_position('right')
                    
                    # Build status display
                    status = f"\r{Colors.CYAN}Steppers: "
                    if l_pos is not None and r_pos is not None:
                        status += f"L={l_pos:.3f}m, R={r_pos:.3f}m | "
                    else:
                        status += "Unknown | "
                        
                    # Add servo angles - left hand
                    status += f"Left Servos: "
                    for i in range(NUM_SERVOS_PER_HAND):
                        status += f"{i}:{self.servo_angles[i]}° "
                    
                    # Add servo angles - right hand
                    status += f"| Right Servos: "
                    for i in range(NUM_SERVOS_PER_HAND):
                        status += f"{i}:{self.servo_angles[i + NUM_SERVOS_PER_HAND]}° "
                    
                    # Add solenoid states - left hand
                    status += f"| Left Solenoids: "
                    for i in range(NUM_SOLENOIDS_PER_HAND):
                        status += f"{i}:{self.solenoid_states[i]} "
                        
                    # Add solenoid states - right hand
                    status += f"| Right Solenoids: "
                    for i in range(NUM_SOLENOIDS_PER_HAND):
                        status += f"{i}:{self.solenoid_states[i + NUM_SOLENOIDS_PER_HAND]} "
                        
                    status += f"{Colors.ENDC}"
                    
                    # Display status
                    print(status, end='')
                    sys.stdout.flush()
                        
                time.sleep(0.5)  # 2Hz update rate
                
        except Exception as e:
            print(f"\n{Colors.RED}Monitoring error: {str(e)}{Colors.ENDC}")
        finally:
            self.monitoring = False
            print(f"\n{Colors.YELLOW}Monitoring stopped{Colors.ENDC}")
            
        # Start monitoring thread
        self.monitor_thread = threading.Thread(target=monitor_loop)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()

    def stop_monitoring(self):
        """Stop monitoring if active"""
        if not self.monitoring:
            print(f"{Colors.YELLOW}Monitoring is not active{Colors.ENDC}")
            return
            
        self.monitoring = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=1.0)
            print(f"{Colors.GREEN}Monitoring stopped{Colors.ENDC}")
            
    # === Utility Methods ===
    
    def get_motor_code(self, motor):
        """Convert motor name to code used in protocol."""
        motor = motor.lower()
        if motor in ['left', 'l']:
            return 'l'
        elif motor in ['right', 'r']:
            return 'r'
        else:
            print(f"{Colors.RED}Error: Invalid motor '{motor}'. Use 'left' or 'right'{Colors.ENDC}")
            return None
            
    def print_status(self):
        """Print current status of all robot components"""
        # Print stepper positions
        print(f"{Colors.CYAN}=== Robot Status ==={Colors.ENDC}")
        print(f"{Colors.CYAN}Stepper Positions:{Colors.ENDC}")
        print(f"  Left: {self.left_position if self.left_position is not None else 'Unknown'}")
        print(f"  Right: {self.right_position if self.right_position is not None else 'Unknown'}")
        
        # Print servo angles - left hand
        print(f"{Colors.CYAN}Left Hand Servo Angles (degrees):{Colors.ENDC}")
        for i in range(NUM_SERVOS_PER_HAND):
            print(f"  Servo {i}: {self.servo_angles[i]}°")
            
        # Print servo angles - right hand
        print(f"{Colors.CYAN}Right Hand Servo Angles (degrees):{Colors.ENDC}")
        for i in range(NUM_SERVOS_PER_HAND):
            print(f"  Servo {i}: {self.servo_angles[i + NUM_SERVOS_PER_HAND]}°")
            
        # Print solenoid states - left hand
        print(f"{Colors.CYAN}Left Hand Solenoid States:{Colors.ENDC}")
        for i in range(NUM_SOLENOIDS_PER_HAND):
            print(f"  Solenoid {i}: {'ON' if self.solenoid_states[i] else 'OFF'}")
            
        # Print solenoid states - right hand
        print(f"{Colors.CYAN}Right Hand Solenoid States:{Colors.ENDC}")
        for i in range(NUM_SOLENOIDS_PER_HAND):
            print(f"  Solenoid {i}: {'ON' if self.solenoid_states[i + NUM_SOLENOIDS_PER_HAND] else 'OFF'}")
    
    # Combined command to set all components at once
    def set_all_components(self, stepper_pos, servo_angles, solenoid_states):
        """Set all robot components at once"""
        success = True
        
        # Validate inputs
        if len(servo_angles) != (NUM_SERVOS_PER_HAND * 2):
            print(f"{Colors.RED}Error: Expected {NUM_SERVOS_PER_HAND * 2} servo angles{Colors.ENDC}")
            return False
            
        if len(solenoid_states) != (NUM_SOLENOIDS_PER_HAND * 2):
            print(f"{Colors.RED}Error: Expected {NUM_SOLENOIDS_PER_HAND * 2} solenoid states{Colors.ENDC}")
            return False
        
        # Set stepper position
        try:
            self.move_motor('left', stepper_pos)
        except Exception as e:
            print(f"{Colors.RED}Error setting stepper: {str(e)}{Colors.ENDC}")
            success = False
        
        # Set servo angles
        for i, angle in enumerate(servo_angles):
            try:
                self.set_servo_angle(i, angle)
            except Exception as e:
                print(f"{Colors.RED}Error setting servo {i}: {str(e)}{Colors.ENDC}")
                success = False
        
        # Set solenoid states
        for i, state in enumerate(solenoid_states):
            try:
                self.set_solenoid(i, state)
            except Exception as e:
                print(f"{Colors.RED}Error setting solenoid {i}: {str(e)}{Colors.ENDC}")
                success = False
                
        return success
        
    def run_cli(self):
        """Run the interactive command-line interface"""
        try:
            while True:
                # Get user input with custom prompt
                command = input(f"{Colors.BOLD}{Colors.BLUE}robot> {Colors.ENDC}").strip()
                
                # Skip empty commands
                if not command:
                    continue
                    
                # Process command
                self.process_command(command)
                
        except KeyboardInterrupt:
            print(f"\n{Colors.YELLOW}Interrupted by user{Colors.ENDC}")
        except Exception as e:
            print(f"\n{Colors.RED}Error: {str(e)}{Colors.ENDC}")
        finally:
            self.cleanup()
            
    def process_command(self, command):
        """Process a command entered by the user"""
        # Convert to lowercase
        cmd_lower = command.lower()
        
        # Exit commands
        if cmd_lower in ["exit", "quit", "q"]:
            print(f"{Colors.GREEN}Exiting...{Colors.ENDC}")
            raise KeyboardInterrupt
            
        # Help command
        elif cmd_lower == "help":
            self.print_help()
            
        # Clear screen
        elif cmd_lower in ["clear", "cls"]:
            os.system('cls' if os.name == 'nt' else 'clear')
            self.print_banner()
            
        # Status command
        elif cmd_lower == "status":
            self.print_status()
            
        # Calibration
        elif cmd_lower in ["calibrate", "cal"]:
            success = self.calibrate_motors()
            if success:
                print(f"{Colors.GREEN}Calibration completed successfully.{Colors.ENDC}")
            else:
                print(f"{Colors.YELLOW}Calibration did not complete. Motors may be in an unknown state.{Colors.ENDC}")
            
        # Position query
        elif cmd_lower.startswith("position ") or cmd_lower.startswith("p "):
            parts = cmd_lower.split(maxsplit=1)
            if len(parts) == 2:
                motor = parts[1]
                self.get_position(motor)
            else:
                print(f"{Colors.RED}Error: Missing motor specification. Use 'position <motor>'{Colors.ENDC}")
                
        # Stepper move command
        elif cmd_lower.startswith("move ") or cmd_lower.startswith("m "):
            parts = re.split(r'\s+', cmd_lower, maxsplit=2)
            if len(parts) >= 3:
                motor = parts[1]
                try:
                    position = float(parts[2])
                    self.move_motor(motor, position)
                except ValueError:
                    print(f"{Colors.RED}Error: Invalid position '{parts[2]}'. Must be a number{Colors.ENDC}")
            else:
                print(f"{Colors.RED}Error: Invalid format. Use 'move <motor> <position>'{Colors.ENDC}")

        # Home command
        elif cmd_lower == "home":
            self.home_motors()
            
        # Servo command with hand specification
        elif cmd_lower.startswith("servo ") or cmd_lower.startswith("s "):
            parts = re.split(r'\s+', cmd_lower, maxsplit=3)
            if len(parts) >= 3:
                try:
                    hand = parts[1].lower()
                    channel = int(parts[2])
                    
                    # Check if we have angle specified
                    if len(parts) < 4:
                        print(f"{Colors.RED}Error: Missing angle. Use 'servo <hand> <channel> <angle>'{Colors.ENDC}")
                        return
                        
                    angle = float(parts[3])
                    
                    # Calculate global channel based on hand
                    if hand in ['left', 'l']:
                        if channel < 0 or channel >= NUM_SERVOS_PER_HAND:
                            print(f"{Colors.RED}Error: Invalid servo channel. Must be 0-{NUM_SERVOS_PER_HAND-1}{Colors.ENDC}")
                            return
                        global_channel = channel
                    elif hand in ['right', 'r']:
                        if channel < 0 or channel >= NUM_SERVOS_PER_HAND:
                            print(f"{Colors.RED}Error: Invalid servo channel. Must be 0-{NUM_SERVOS_PER_HAND-1}{Colors.ENDC}")
                            return
                        global_channel = channel + NUM_SERVOS_PER_HAND
                    else:
                        print(f"{Colors.RED}Error: Invalid hand '{hand}'. Use 'left' or 'right'{Colors.ENDC}")
                        return
                        
                    self.set_servo_angle(global_channel, angle)
                except ValueError:
                    print(f"{Colors.RED}Error: Invalid servo parameters. Use numbers for channel and angle{Colors.ENDC}")
                except Exception as e:
                    print(f"{Colors.RED}Error: {str(e)}{Colors.ENDC}")
            else:
                print(f"{Colors.RED}Error: Invalid format. Use 'servo <hand> <channel> <angle>'{Colors.ENDC}")
                
        # Servos status
        elif cmd_lower == "servos":
            print(f"{Colors.CYAN}Current servo angles:{Colors.ENDC}")
            
            # Left hand servos
            print(f"{Colors.CYAN}Left Hand:{Colors.ENDC}")
            for i in range(NUM_SERVOS_PER_HAND):
                print(f"  Servo {i}: {self.servo_angles[i]}°")
                
            # Right hand servos
            print(f"{Colors.CYAN}Right Hand:{Colors.ENDC}")
            for i in range(NUM_SERVOS_PER_HAND):
                print(f"  Servo {i}: {self.servo_angles[i + NUM_SERVOS_PER_HAND]}°")
                
        # Solenoid command with hand specification
        elif cmd_lower.startswith("solenoid ") or cmd_lower.startswith("sol "):
            parts = re.split(r'\s+', cmd_lower, maxsplit=3)
            if len(parts) >= 3:
                try:
                    hand = parts[1].lower()
                    channel = int(parts[2])
                    
                    # Check if we have state specified
                    if len(parts) < 4:
                        print(f"{Colors.RED}Error: Missing state. Use 'solenoid <hand> <channel> <state>'{Colors.ENDC}")
                        return
                        
                    state = int(parts[3])
                    
                    # Calculate global channel based on hand
                    if hand in ['left', 'l']:
                        if channel < 0 or channel >= NUM_SOLENOIDS_PER_HAND:
                            print(f"{Colors.RED}Error: Invalid solenoid channel. Must be 0-{NUM_SOLENOIDS_PER_HAND-1}{Colors.ENDC}")
                            return
                        global_channel = channel
                    elif hand in ['right', 'r']:
                        if channel < 0 or channel >= NUM_SOLENOIDS_PER_HAND:
                            print(f"{Colors.RED}Error: Invalid solenoid channel. Must be 0-{NUM_SOLENOIDS_PER_HAND-1}{Colors.ENDC}")
                            return
                        global_channel = channel + NUM_SOLENOIDS_PER_HAND
                    else:
                        print(f"{Colors.RED}Error: Invalid hand '{hand}'. Use 'left' or 'right'{Colors.ENDC}")
                        return
                        
                    self.set_solenoid(global_channel, state)
                except ValueError:
                    print(f"{Colors.RED}Error: Invalid solenoid parameters. Use numbers for channel and state{Colors.ENDC}")
                except Exception as e:
                    print(f"{Colors.RED}Error: {str(e)}{Colors.ENDC}")
            else:
                print(f"{Colors.RED}Error: Invalid format. Use 'solenoid <hand> <channel> <state>'{Colors.ENDC}")
                
        # Set all solenoids for a specific hand
        elif cmd_lower.startswith("solenoids "):
            parts = re.split(r'\s+', cmd_lower, maxsplit=2)
            if len(parts) >= 3:
                try:
                    hand = parts[1].lower()
                    state = int(parts[2])
                    
                    if hand in ['left', 'l']:
                        # Set all left hand solenoids
                        for i in range(NUM_SOLENOIDS_PER_HAND):
                            self.set_solenoid(i, state)
                        print(f"{Colors.GREEN}Set all left hand solenoids to {'ON' if state else 'OFF'}{Colors.ENDC}")
                    elif hand in ['right', 'r']:
                        # Set all right hand solenoids
                        for i in range(NUM_SOLENOIDS_PER_HAND):
                            self.set_solenoid(i + NUM_SOLENOIDS_PER_HAND, state)
                        print(f"{Colors.GREEN}Set all right hand solenoids to {'ON' if state else 'OFF'}{Colors.ENDC}")
                    elif hand in ['all', 'both', 'a', 'b']:
                        # Set all solenoids on both hands
                        self.set_all_solenoids(state)
                        print(f"{Colors.GREEN}Set all solenoids to {'ON' if state else 'OFF'}{Colors.ENDC}")
                    else:
                        print(f"{Colors.RED}Error: Invalid hand '{hand}'. Use 'left', 'right', or 'all'{Colors.ENDC}")
                except ValueError:
                    print(f"{Colors.RED}Error: Invalid state value. Use 0 (OFF) or 1 (ON){Colors.ENDC}")
            else:
                print(f"{Colors.RED}Error: Invalid format. Use 'solenoids <hand> <state>'{Colors.ENDC}")
                
        # Combined command for all components
        elif cmd_lower.startswith("setall "):
            try:
                # Parse command: setall <stepper_pos> <l_s0> <l_s1> <l_s2> <l_s3> <l_s4> <r_s0> <r_s1> <r_s2> <r_s3> <r_s4> <l_sol0> <l_sol1> <l_sol2> <l_sol3> <l_sol4> <r_sol0> <r_sol1> <r_sol2> <r_sol3> <r_sol4>
                parts = re.split(r'\s+', cmd_lower)[1:]
                total_servos = NUM_SERVOS_PER_HAND * 2
                total_solenoids = NUM_SOLENOIDS_PER_HAND * 2
                
                if len(parts) != 1 + total_servos + total_solenoids:
                    print(f"{Colors.RED}Error: Expected format is 'setall <stepper_pos> <l_s0> <l_s1> <l_s2> <l_s3> <l_s4> <r_s0> <r_s1> <r_s2> <r_s3> <r_s4> <l_sol0> <l_sol1> <l_sol2> <l_sol3> <l_sol4> <r_sol0> <r_sol1> <r_sol2> <r_sol3> <r_sol4>'{Colors.ENDC}")
                    return
                
                # Parse values
                stepper_pos = float(parts[0])
                servo_angles = [float(parts[i+1]) for i in range(total_servos)]
                solenoid_states = [int(parts[i+1+total_servos]) for i in range(total_solenoids)]
                
                # Execute command
                self.set_all_components(stepper_pos, servo_angles, solenoid_states)
                
            except ValueError as e:
                print(f"{Colors.RED}Error parsing values: {str(e)}{Colors.ENDC}")
                print(f"{Colors.YELLOW}All values must be numbers (stepper=float, servos=float, solenoids=int){Colors.ENDC}")
            except Exception as e:
                print(f"{Colors.RED}Error executing command: {str(e)}{Colors.ENDC}")
                
        # Monitor positions
        elif cmd_lower == "monitor":
            self.monitor_positions()
            
        # Stop monitoring
        elif cmd_lower == "stop":
            self.stop_monitoring()
            
        # Raw command
        elif cmd_lower.startswith("raw "):
            raw_cmd = command[4:].strip()
            if raw_cmd:
                # For raw commands starting with 'r', we need to read response
                if raw_cmd.startswith('r') and len(raw_cmd) == 2:
                    motor_code = raw_cmd[1].lower()
                    if motor_code in ['l', 'r']:
                        self.read_position(motor_code)
                    else:
                        print(f"{Colors.RED}Invalid motor code in raw command{Colors.ENDC}")
                else:
                    # Just send the command without expecting response
                    self.send_command(raw_cmd)
            else:
                print(f"{Colors.RED}Error: No command specified after 'raw'{Colors.ENDC}")
                
        # Unknown command
        else:
            print(f"{Colors.RED}Unknown command: '{command}'. Type 'help' for available commands{Colors.ENDC}")
    
    def print_help(self):
        """Display detailed help information"""
        help_text = f"""
{Colors.HEADER}{Colors.BOLD}╔═══════════════════════════════════════════════════════════╗
║ Robot Teleoperator - Dual Hand Command Reference            ║
╚═══════════════════════════════════════════════════════════╝{Colors.ENDC}

{Colors.BOLD}Basic Commands:{Colors.ENDC}
  {Colors.CYAN}help{Colors.ENDC}                    Show this help message
  {Colors.CYAN}status{Colors.ENDC}                  Show current status of all components
  {Colors.CYAN}exit, quit, q{Colors.ENDC}           Exit the program
  {Colors.CYAN}clear, cls{Colors.ENDC}              Clear the screen

{Colors.BOLD}Stepper Motor Control:{Colors.ENDC}
  {Colors.CYAN}calibrate, cal{Colors.ENDC}          Initialize calibration sequence for both motors
                            {Colors.YELLOW}This moves the motors to their limits to establish the range{Colors.ENDC}

  {Colors.CYAN}move <motor> <pos>{Colors.ENDC}      Move specified stepper to position (in meters)
  {Colors.CYAN}m <motor> <pos>{Colors.ENDC}         {Colors.YELLOW}Example: move left 0.5  (moves left motor to 0.5m){Colors.ENDC}

  {Colors.CYAN}position <motor>{Colors.ENDC}        Get current position of specified stepper motor
  {Colors.CYAN}p <motor>{Colors.ENDC}               {Colors.YELLOW}Example: position right  (shows right motor position){Colors.ENDC}

  {Colors.CYAN}home{Colors.ENDC}                    Move both stepper motors to their home positions
                            {Colors.YELLOW}This returns motors to center position after calibration{Colors.ENDC}

{Colors.BOLD}Servo Control:{Colors.ENDC}
  {Colors.CYAN}servo <hand> <channel> <angle>{Colors.ENDC} Set servo angle (0-180 degrees)
  {Colors.CYAN}s <hand> <channel> <angle>{Colors.ENDC}     {Colors.YELLOW}Example: servo left 2 90  (sets left hand servo 2 to 90 degrees){Colors.ENDC}

  {Colors.CYAN}servos{Colors.ENDC}                  Show current angles of all servos

{Colors.BOLD}Solenoid Control:{Colors.ENDC}
  {Colors.CYAN}solenoid <hand> <channel> <state>{Colors.ENDC} Set solenoid state (0=OFF, 1=ON)
  {Colors.CYAN}sol <hand> <channel> <state>{Colors.ENDC}   {Colors.YELLOW}Example: solenoid right 1 1  (turns ON right hand solenoid 1){Colors.ENDC}

  {Colors.CYAN}solenoids <hand> <state>{Colors.ENDC}       Set all solenoids on specified hand to the same state
                            {Colors.YELLOW}Example: solenoids left 0  (turns OFF all left hand solenoids){Colors.ENDC}
                            {Colors.YELLOW}Example: solenoids all 1  (turns ON all solenoids on both hands){Colors.ENDC}

{Colors.BOLD}Combined Control:{Colors.ENDC}
  {Colors.CYAN}setall <stepper> <l_s0> <l_s1> <l_s2> <l_s3> <l_s4> <r_s0> <r_s1> <r_s2> <r_s3> <r_s4> <l_sol0> <l_sol1> <l_sol2> <l_sol3> <l_sol4> <r_sol0> <r_sol1> <r_sol2> <r_sol3> <r_sol4>{Colors.ENDC}
                            {Colors.YELLOW}Control all components at once with a single command{Colors.ENDC}
                            {Colors.YELLOW}Example: setall 0.5 90 100 80 95 90 90 95 85 90 90 1 0 0 1 0 0 1 0 0 1{Colors.ENDC}

{Colors.BOLD}Monitoring:{Colors.ENDC}
  {Colors.CYAN}monitor{Colors.ENDC}                 Start continuous monitoring of all components
  {Colors.CYAN}stop{Colors.ENDC}                    Stop monitoring

{Colors.BOLD}Raw Commands:{Colors.ENDC}
  {Colors.CYAN}raw <command>{Colors.ENDC}           Send a raw command to the motor controller
                            {Colors.YELLOW}Example: raw pl0.5  (move left motor to 0.5m){Colors.ENDC}

{Colors.GREEN}All commands are case-insensitive.{Colors.ENDC}
"""
        print(help_text)


def main():
    """Main entry point for the application"""
    # Declare globals at the beginning of the function
    global LEFT_PCA9685_ADDR, LEFT_MCP23017_ADDR, LEFT_ADS7138_ADDR
    global RIGHT_PCA9685_ADDR, RIGHT_MCP23017_ADDR, RIGHT_ADS7138_ADDR
    
    parser = argparse.ArgumentParser(description="Robot Teleoperator Control System - Dual Hand Support")
    
    # SPI options
    spi_group = parser.add_argument_group('SPI Options (Stepper Motors)')
    spi_group.add_argument("--spi-bus", type=int, default=SPI_BUS, help="SPI bus number (default: 0)")
    spi_group.add_argument("--spi-device", type=int, default=SPI_DEVICE, help="SPI device number (default: 0)")
    spi_group.add_argument("--spi-speed", type=int, default=SPI_SPEED, help="SPI clock speed in Hz (default: 500000)")
    spi_group.add_argument("--spi-mode", type=int, default=SPI_MODE, help="SPI mode (0-3) (default: 1)")
    
    # I2C options
    i2c_group = parser.add_argument_group('I2C Options')
    i2c_group.add_argument("--i2c-bus", type=int, default=LEFT_I2C_BUS, help="I2C bus number (default: 1)")
    
    # Left hand options
    left_group = parser.add_argument_group('Left Hand Options')
    left_group.add_argument("--left-pca", type=lambda x: int(x, 0), default=LEFT_PCA9685_ADDR, 
                           help=f"Left hand PCA9685 I2C address (default: 0x{LEFT_PCA9685_ADDR:02X})")
    left_group.add_argument("--left-mcp", type=lambda x: int(x, 0), default=LEFT_MCP23017_ADDR, 
                           help=f"Left hand MCP23017 I2C address (default: 0x{LEFT_MCP23017_ADDR:02X})")
    left_group.add_argument("--left-adc", type=lambda x: int(x, 0), default=LEFT_ADS7138_ADDR, 
                           help=f"Left hand ADS7138 I2C address (default: 0x{LEFT_ADS7138_ADDR:02X})")
    
    # Right hand options
    right_group = parser.add_argument_group('Right Hand Options')
    right_group.add_argument("--right-pca", type=lambda x: int(x, 0), default=RIGHT_PCA9685_ADDR, 
                            help=f"Right hand PCA9685 I2C address (default: 0x{RIGHT_PCA9685_ADDR:02X})")
    right_group.add_argument("--right-mcp", type=lambda x: int(x, 0), default=RIGHT_MCP23017_ADDR, 
                            help=f"Right hand MCP23017 I2C address (default: 0x{RIGHT_MCP23017_ADDR:02X})")
    right_group.add_argument("--right-adc", type=lambda x: int(x, 0), default=RIGHT_ADS7138_ADDR, 
                            help=f"Right hand ADS7138 I2C address (default: 0x{RIGHT_ADS7138_ADDR:02X})")
    
    # Other options
    parser.add_argument("--no-color", action="store_true", help="Disable colored output")
    parser.add_argument("--disable-servos", action="store_true", help="Skip servo initialization")
    parser.add_argument("--disable-solenoids", action="store_true", help="Skip solenoid initialization")
    
    args = parser.parse_args()
    
    # Update I2C addresses if provided via command line
    LEFT_PCA9685_ADDR = args.left_pca
    LEFT_MCP23017_ADDR = args.left_mcp
    LEFT_ADS7138_ADDR = args.left_adc
    RIGHT_PCA9685_ADDR = args.right_pca
    RIGHT_MCP23017_ADDR = args.right_mcp
    RIGHT_ADS7138_ADDR = args.right_adc
    
    # Disable colors if requested
    if args.no_color:
        Colors.disable()
    
    try:
        print(f"{Colors.CYAN}Initializing robot teleoperator with dual hand support...{Colors.ENDC}")
        controller = RobotTeleoperator(
            spi_bus=args.spi_bus, 
            spi_device=args.spi_device, 
            spi_speed=args.spi_speed, 
            spi_mode=args.spi_mode,
            i2c_bus=args.i2c_bus
        )
        controller.run_cli()
    except Exception as e:
        print(f"{Colors.RED}Fatal error: {str(e)}{Colors.ENDC}")
        return 1
        
    return 0


if __name__ == "__main__":
    sys.exit(main())


# """
# Robot Teleoperator

# A command-line interface for controlling stepper motors, servos, and solenoids
# for a robotic system using SPI and I2C communication.
# """

# import spidev
# import time
# import sys
# import signal
# import threading
# import argparse
# import os
# import re
# from datetime import datetime
# from smbus2 import SMBus

# # SPI configuration
# SPI_BUS = 0
# SPI_DEVICE = 0
# SPI_SPEED = 500000  # 500 kHz
# SPI_MODE = 1        # SPI mode (CPOL=0, CPHA=1)

# # I2C configuration
# I2C_BUS = 1         # Standard I2C bus on Raspberry Pi
# PCA9685_ADDR = 0x41  # Servo controller
# MCP23017_ADDR = 0x21  # GPIO extender for solenoids
# ADS7138_ADDR = 0x12  # ADC (not implemented in this version)

# # Component constants
# NUM_SERVOS = 5      # Number of servo channels (0-4)
# NUM_SOLENOIDS = 5   # Number of solenoid channels (0-4)

# # Servo calibration data (from C++ code)
# # Format: [SLOPE, INTERCEPT] for each servo
# SERVO_PWM_CALIBRATION = [
#     [11.85, 547.02],  // Servo 1 (channel 0)
#     [12.03, 373.43],  // Servo 2 (channel 1)
#     [11.91, 351.75],  // Servo 3 (channel 2)
#     [12.35, 406.23],  // Servo 4 (channel 3)
#     [12.37, 417.43],  // Servo 5 (channel 4)
#     [12.33, 426.43],  // Servo 6 (channel 5)
#     [12.30, 412.15],  // Servo 7 (channel 6)
#     [12.36, 476.37],  // Servo 8 (channel 7)
#     [12.02, 496.32],  // Servo 9 (channel 8)
#     [12.28, 430.31]   // Servo 10 (channel 9)
# ]

# SERVO_ADC_CALIBRATION = [
#     [1.79, 105.23],  // Servo 1 (channel 0)
#     [2.39, 103.98],  // Servo 2 (channel 1)
#     [2.19, 91.01],   // Servo 3 (channel 2)
#     [2.21, 99.48],   // Servo 4 (channel 3)
#     [2.23, 102.81],  // Servo 5 (channel 4)
#     [2.23, 103.63],  // Servo 6 (channel 5)
#     [2.41, 111.39],  // Servo 7 (channel 6)
#     [2.19, 115.01],  // Servo 8 (channel 7)
#     [2.18, 117.06],  // Servo 9 (channel 8)
#     [2.21, 105.57]   // Servo 10 (channel 9)
# ]

# # Servo constraints
# PWM_MIN = 500       # Minimum pulse width in microseconds
# PWM_MAX = 2500      # Maximum pulse width in microseconds

# # PCA9685 registers
# PCA9685_MODE1 = 0x00
# PCA9685_MODE2 = 0x01
# PCA9685_SUBADR1 = 0x02
# PCA9685_SUBADR2 = 0x03
# PCA9685_SUBADR3 = 0x04
# PCA9685_PRESCALE = 0xFE
# PCA9685_LED0_ON_L = 0x06
# PCA9685_LED0_ON_H = 0x07
# PCA9685_LED0_OFF_L = 0x08
# PCA9685_LED0_OFF_H = 0x09
# PCA9685_ALL_LED_ON_L = 0xFA
# PCA9685_ALL_LED_ON_H = 0xFB
# PCA9685_ALL_LED_OFF_L = 0xFC
# PCA9685_ALL_LED_OFF_H = 0xFD

# # PCA9685 mode bits
# PCA9685_RESTART = 0x80
# PCA9685_SLEEP = 0x10
# PCA9685_ALLCALL = 0x01
# PCA9685_INVRT = 0x10
# PCA9685_OUTDRV = 0x04
# PCA9685_OCH = 0x08

# # PCA9685 constants
# PCA9685_OSC_CLOCK = 25000000  # 25MHz
# PCA9685_RESOLUTION = 4096     # 12-bit resolution

# # MCP23017 registers
# MCP23017_IODIRA = 0x00
# MCP23017_IODIRB = 0x01
# MCP23017_GPIOA = 0x12
# MCP23017_OLATA = 0x14

# # Terminal colors and formatting
# class Colors:
#     HEADER = '\033[95m'
#     BLUE = '\033[94m'
#     CYAN = '\033[96m'
#     GREEN = '\033[92m'
#     YELLOW = '\033[93m'
#     RED = '\033[91m'
#     ENDC = '\033[0m'
#     BOLD = '\033[1m'
#     UNDERLINE = '\033[4m'
    
#     @staticmethod
#     def disable():
#         """Disable colors for non-terminal environments"""
#         Colors.HEADER = ''
#         Colors.BLUE = ''
#         Colors.CYAN = ''
#         Colors.GREEN = ''
#         Colors.YELLOW = ''
#         Colors.RED = ''
#         Colors.ENDC = ''
#         Colors.BOLD = ''
#         Colors.UNDERLINE = ''

# # Check if we're in a terminal that supports colors
# if not sys.stdout.isatty():
#     Colors.disable()


# class RobotTeleoperator:
#     """Main controller class for communicating with the robot hardware"""
    
#     def __init__(self, spi_bus=SPI_BUS, spi_device=SPI_DEVICE, spi_speed=SPI_SPEED, 
#                  spi_mode=SPI_MODE, i2c_bus=I2C_BUS):
#         """Initialize the communication interfaces to the robot hardware."""
#         # Initialize SPI for stepper motors
#         self.spi = spidev.SpiDev()
#         self.spi.open(spi_bus, spi_device)
#         self.spi.max_speed_hz = spi_speed
#         self.spi.mode = spi_mode
#         self.spi.bits_per_word = 8
#         self.spi.lsbfirst = False
        
#         # Initialize I2C for servos and solenoids
#         self.i2c = SMBus(i2c_bus)
        
#         # State tracking
#         self.monitoring = False
#         self.monitor_thread = None
#         self.left_position = None
#         self.right_position = None
#         self.servo_angles = [90] * NUM_SERVOS  # Default servo angles (degrees)
#         self.solenoid_states = [0] * NUM_SOLENOIDS  # Default solenoid states (0=OFF)
        
#         # Capture Ctrl+C for clean shutdown
#         signal.signal(signal.SIGINT, self.signal_handler)
        
#         # Initialize hardware
#         self.init_hardware()
        
#         # Print connection info
#         self.last_command_time = datetime.now()
#         self.print_banner()
        
#     def init_hardware(self):
#         """Initialize all hardware components"""
#         try:
#             # Initialize PCA9685 (servo controller)
#             self.init_pca9685()
#             print(f"{Colors.GREEN}Servo controller (PCA9685) initialized{Colors.ENDC}")
            
#             # Initialize MCP23017 (solenoid controller)
#             self.init_mcp23017()
#             print(f"{Colors.GREEN}Solenoid controller (MCP23017) initialized{Colors.ENDC}")
            
#             # Set all servos to default positions
#             for i in range(NUM_SERVOS):
#                 self.set_servo_angle(i, self.servo_angles[i])
                
#             # Set all solenoids to OFF
#             self.set_all_solenoids(0)
            
#         except Exception as e:
#             print(f"{Colors.RED}Error initializing hardware: {str(e)}{Colors.ENDC}")
#             raise
    
#     def init_pca9685(self):
#         """Initialize the PCA9685 servo controller"""
#         # Reset the device
#         self.i2c.write_byte_data(PCA9685_ADDR, PCA9685_MODE1, 0x00)
#         time.sleep(0.01)  # Wait for reset
        
#         # Set totem pole structure (MODE2 register)
#         self.i2c.write_byte_data(PCA9685_ADDR, PCA9685_MODE2, PCA9685_OUTDRV | PCA9685_OCH)
        
#         # Enable responding to LED All Call I2C-bus address
#         self.i2c.write_byte_data(PCA9685_ADDR, PCA9685_MODE1, PCA9685_ALLCALL)
#         time.sleep(0.005)  # Wait for oscillator
        
#         # Wake up from sleep
#         mode1 = self.i2c.read_byte_data(PCA9685_ADDR, PCA9685_MODE1)
#         mode1 = mode1 & ~PCA9685_SLEEP  # Clear sleep bit
#         self.i2c.write_byte_data(PCA9685_ADDR, PCA9685_MODE1, mode1)
#         time.sleep(0.005)  # Wait for oscillator
        
#         # Set PWM frequency to 50Hz (standard for servos)
#         self.set_pwm_freq(50)
    
#     def init_mcp23017(self):
#         """Initialize the MCP23017 GPIO expander for solenoids"""
#         # Set GPIOA pins as outputs
#         self.i2c.write_byte_data(MCP23017_ADDR, MCP23017_IODIRA, 0x00)
        
#         # Initialize all outputs to 0
#         self.i2c.write_byte_data(MCP23017_ADDR, MCP23017_OLATA, 0x00)
    
#     def set_pwm_freq(self, freq_hz):
#         """Set PWM frequency for the PCA9685"""
#         # Calculate prescale value
#         prescaleval = PCA9685_OSC_CLOCK  # 25MHz
#         prescaleval /= PCA9685_RESOLUTION  # 12-bit
#         prescaleval /= freq_hz
#         prescaleval -= 1.0
#         prescale = int(round(prescaleval))
        
#         # Read current mode
#         oldmode = self.i2c.read_byte_data(PCA9685_ADDR, PCA9685_MODE1)
        
#         # Go to sleep mode
#         newmode = (oldmode & 0x7F) | PCA9685_SLEEP
#         self.i2c.write_byte_data(PCA9685_ADDR, PCA9685_MODE1, newmode)
        
#         # Set prescale
#         self.i2c.write_byte_data(PCA9685_ADDR, PCA9685_PRESCALE, prescale)
        
#         # Restore old mode
#         self.i2c.write_byte_data(PCA9685_ADDR, PCA9685_MODE1, oldmode)
#         time.sleep(0.005)  # Wait for oscillator
        
#         # Set RESTART bit
#         self.i2c.write_byte_data(PCA9685_ADDR, PCA9685_MODE1, oldmode | PCA9685_RESTART)
    
#     def set_pwm(self, channel, on, off):
#         """Set PWM values for a channel"""
#         if channel < 0 or channel > 15:
#             raise ValueError("Channel must be between 0 and 15")
            
#         # Calculate register addresses
#         led_on_l = PCA9685_LED0_ON_L + (channel * 4)
#         led_on_h = PCA9685_LED0_ON_H + (channel * 4)
#         led_off_l = PCA9685_LED0_OFF_L + (channel * 4)
#         led_off_h = PCA9685_LED0_OFF_H + (channel * 4)
        
#         # Write the PWM values
#         self.i2c.write_byte_data(PCA9685_ADDR, led_on_l, on & 0xFF)
#         self.i2c.write_byte_data(PCA9685_ADDR, led_on_h, on >> 8)
#         self.i2c.write_byte_data(PCA9685_ADDR, led_off_l, off & 0xFF)
#         self.i2c.write_byte_data(PCA9685_ADDR, led_off_h, off >> 8)
    
#     def set_servo_angle(self, channel, angle):
#         """Set servo to specified angle using calibration data"""
#         if channel < 0 or channel >= NUM_SERVOS:
#             raise ValueError(f"Servo channel must be between 0 and {NUM_SERVOS-1}")
            
#         # Get calibration parameters for this servo
#         slope = SERVO_PWM_CALIBRATION[channel][0]
#         intercept = SERVO_PWM_CALIBRATION[channel][1]
        
#         # Calculate PWM in microseconds using the linear calibration
#         pwm_us = int((slope * angle) + intercept)
        
#         # Constrain PWM to safe limits
#         pwm_us = max(PWM_MIN, min(PWM_MAX, pwm_us))
        
#         # Convert microseconds to PWM values
#         # For 50Hz (20ms period), 1ms = 205 counts in 12-bit resolution
#         pwm_value = int(pwm_us * 4096 / 20000)
        
#         # Update internal state
#         self.servo_angles[channel] = angle
        
#         # Set the PWM value (always start at 0)
#         self.set_pwm(channel, 0, pwm_value)
        
#         return True
    
#     def set_solenoid(self, channel, state):
#         """Set a solenoid state"""
#         if channel < 0 or channel >= NUM_SOLENOIDS:
#             raise ValueError(f"Solenoid channel must be between 0 and {NUM_SOLENOIDS-1}")
            
#         # Read current state
#         current_state = self.i2c.read_byte_data(MCP23017_ADDR, MCP23017_GPIOA)
        
#         # Modify the bit for this channel
#         if state:
#             new_state = current_state | (1 << channel)  # Set bit
#         else:
#             new_state = current_state & ~(1 << channel)  # Clear bit
            
#         # Write new state
#         self.i2c.write_byte_data(MCP23017_ADDR, MCP23017_GPIOA, new_state)
        
#         # Update internal state
#         self.solenoid_states[channel] = 1 if state else 0
        
#         return True
    
#     def set_all_solenoids(self, state):
#         """Set all solenoids to the same state"""
#         value = 0
#         if state:
#             value = (1 << NUM_SOLENOIDS) - 1  # All bits set
            
#         # Write to GPIO register
#         self.i2c.write_byte_data(MCP23017_ADDR, MCP23017_GPIOA, value)
        
#         # Update internal state
#         self.solenoid_states = [1 if state else 0] * NUM_SOLENOIDS
        
#         return True
    
#     def print_banner(self):
#         """Print application banner with connection information"""
#         banner = f"""
# {Colors.HEADER}{Colors.BOLD}╔═══════════════════════════════════════════════════════════╗
# ║ Robot Teleoperator Control System                          ║
# ╚═══════════════════════════════════════════════════════════╝{Colors.ENDC}
# {Colors.CYAN}• SPI Connection (Steppers): Bus {SPI_BUS}, Device {SPI_DEVICE}, {SPI_SPEED/1000:.1f} kHz, Mode {SPI_MODE}
# - I2C Connection (Servos/Solenoids): Bus {I2C_BUS}
# - Components: {NUM_SERVOS} Servos, {NUM_SOLENOIDS} Solenoids, 2 Stepper Motors{Colors.ENDC}

# {Colors.GREEN}Enter a command or type 'help' for available options.{Colors.ENDC}
# """
#         print(banner)

#     def signal_handler(self, sig, frame):
#         """Handle Ctrl+C gracefully to ensure proper resource cleanup"""
#         print(f"\n{Colors.YELLOW}Interrupt received, shutting down...{Colors.ENDC}")
#         self.cleanup()
#         sys.exit(0)

#     def cleanup(self):
#         """Clean up resources before exiting"""
#         if self.monitoring:
#             self.monitoring = False
#             if self.monitor_thread:
#                 self.monitor_thread.join(timeout=1.0)
                
#         # Set all outputs to safe state
#         try:
#             # Turn off all solenoids
#             self.set_all_solenoids(0)
            
#             # Move servos to default positions
#             for i in range(NUM_SERVOS):
#                 self.set_servo_angle(i, 90)
#         except Exception as e:
#             print(f"{Colors.YELLOW}Warning during cleanup: {str(e)}{Colors.ENDC}")
                
#         # Close connections
#         try:
#             self.spi.close()
#             self.i2c.close()
#             print(f"{Colors.GREEN}All connections closed. Goodbye!{Colors.ENDC}")
#         except:
#             pass

#     # === Motor Commands ===
    
#     def send_command(self, command):
#         """Send a command to the Pico over SPI without expecting a response."""
#         # Record command time for monitoring
#         self.last_command_time = datetime.now()
        
#         # Ensure command ends with null terminator
#         if not command.endswith('\0'):
#             command += '\0'
            
#         # Log the command
#         cmd_str = command.rstrip('\0')
#         print(f"{Colors.BLUE}▶ Sending: '{cmd_str}'{Colors.ENDC}")
        
#         # Show hex representation for debugging
#         hex_cmd = ' '.join([f'{ord(c):02X}' for c in command])
#         print(f"{Colors.CYAN}  Hex: {hex_cmd}{Colors.ENDC}")
        
#         # Send each byte individually
#         for byte in command:
#             self.spi.xfer2([ord(byte)])
#             time.sleep(0.0001)  # Small delay between bytes
            
#         print(f"{Colors.GREEN}Command sent successfully{Colors.ENDC}")

#     def read_position(self, motor):
#         """Read position from a specific motor."""
#         # Ensure we have a valid motor code
#         motor_code = self.get_motor_code(motor)
#         if not motor_code:
#             return None
            
#         # Build the position query command
#         command = f"r{motor_code}"
        
#         # Ensure command ends with null terminator
#         if not command.endswith('\0'):
#             command += '\0'
            
#         # Log the command
#         cmd_str = command.rstrip('\0')
#         print(f"{Colors.BLUE}▶ Sending position query: '{cmd_str}'{Colors.ENDC}")
        
#         # Show hex representation for debugging
#         hex_cmd = ' '.join([f'{ord(c):02X}' for c in command])
#         print(f"{Colors.CYAN}  Hex: {hex_cmd}{Colors.ENDC}")
        
#         # Send each byte individually
#         for byte in command:
#             self.spi.xfer2([ord(byte)])
#             time.sleep(0.0001)  # Small delay between bytes
            
#         # Wait for Pico to process
#         time.sleep(0.01)
        
#         # Read response
#         response_bytes = []
#         valid_bytes = []
        
#         # Read bytes until null terminator or max length
#         max_response_size = 32
#         for _ in range(max_response_size):
#             rx_byte = self.spi.xfer2([0xFF])[0]
#             response_bytes.append(rx_byte)
#             # Skip the first byte which is usually FF or garbage
#             if len(response_bytes) > 1 and rx_byte != 0xFF:
#                 valid_bytes.append(rx_byte)
#             if rx_byte == 0:
#                 break
                
#         # Display raw bytes for debugging
#         hex_response = ' '.join([f'{b:02X}' for b in response_bytes[:min(16, len(response_bytes))]])
#         if len(response_bytes) > 16:
#             hex_response += f" ... ({len(response_bytes)} bytes total)"
#         print(f"{Colors.CYAN}  Response bytes: {hex_response}{Colors.ENDC}")
        
#         # Convert to string, skipping the first byte if it's FF
#         try:
#             # Find position of null terminator if present
#             if 0 in valid_bytes:
#                 valid_bytes = valid_bytes[:valid_bytes.index(0)]
                
#             # Convert bytes to ASCII, filtering out any non-printable characters
#             response_str = ''.join(chr(b) for b in valid_bytes if b >= 32 and b <= 126)
            
#             # Show response
#             if response_str:
#                 print(f"{Colors.GREEN}◀ Received: '{response_str}'{Colors.ENDC}")
                
#                 # Try to parse as float
#                 try:
#                     position = float(response_str)
                    
#                     # Update stored position
#                     if motor_code == 'l':
#                         self.left_position = position
#                     else:
#                         self.right_position = position
                        
#                     return position
#                 except ValueError:
#                     print(f"{Colors.RED}Error: Could not parse position from '{response_str}'{Colors.ENDC}")
#                     return None
#             else:
#                 print(f"{Colors.YELLOW}◀ Empty response{Colors.ENDC}")
#                 return None
            
#         except Exception as e:
#             print(f"{Colors.RED}◀ Error parsing response: {str(e)}{Colors.ENDC}")
#             # As a fallback, try to display the raw bytes as ASCII
#             fallback_str = ''.join(chr(b) if 32 <= b <= 126 else '.' for b in valid_bytes)
#             print(f"{Colors.YELLOW}  Raw ASCII: {fallback_str}{Colors.ENDC}")
#             return None

#     def calibrate_motors(self):
#         """Send the calibration command to both stepper motors"""
#         print(f"{Colors.YELLOW}Initiating motor calibration...{Colors.ENDC}")
#         print(f"{Colors.YELLOW}This will move both motors to their limits. Ensure path is clear!{Colors.ENDC}")
        
#         # Send calibration command 'c'
#         self.send_command('c')
#         print(f"{Colors.YELLOW}Note: Calibration takes some time to complete.{Colors.ENDC}")

#     def move_motor(self, motor, position):
#         """Move a stepper motor to the specified position."""
#         # Validate motor parameter
#         motor_code = self.get_motor_code(motor)
#         if not motor_code:
#             return
            
#         # Send position command: p<motor_code><position>
#         command = f"p{motor_code}{position}"
#         self.send_command(command)
        
#         print(f"{Colors.GREEN}Command sent to move {motor} motor to {position} meters{Colors.ENDC}")

#     def get_position(self, motor):
#         """Get the current position of a stepper motor"""
#         # Get position using the read_position method
#         position = self.read_position(motor)
        
#         if position is not None:
#             print(f"{Colors.GREEN}{motor.capitalize()} motor position: {position:.6f} meters{Colors.ENDC}")
#             return position
#         return None

#     def home_motors(self):
#         """Send the home command to both stepper motors"""
#         print(f"{Colors.YELLOW}Sending stepper motors to home position...{Colors.ENDC}")
        
#         # Send home command 'h'
#         self.send_command('h')
#         print(f"{Colors.YELLOW}Home command sent.{Colors.ENDC}")

#     def monitor_positions(self):
#         """Start continuous monitoring of robot state"""
#         if self.monitoring:
#             print(f"{Colors.YELLOW}Monitoring is already active{Colors.ENDC}")
#             return
            
#         self.monitoring = True
        
#         def monitor_loop():
#             """Background thread function for monitoring"""
#             print(f"{Colors.GREEN}Starting robot monitoring (press Ctrl+C or type 'stop' to end){Colors.ENDC}")
            
#             try:
#                 while self.monitoring:
#                     # Only query if we haven't received a command recently
#                     time_since_command = (datetime.now() - self.last_command_time).total_seconds()
#                     if time_since_command > 0.5:  # Don't interfere with manual commands
#                         l_pos = self.read_position('left')
#                         r_pos = self.read_position('right')
                        
#                         # Build status display
#                         status = f"\r{Colors.CYAN}Steppers: "
#                         if l_pos is not None and r_pos is not None:
#                             status += f"L={l_pos:.3f}m, R={r_pos:.3f}m | "
#                         else:
#                             status += "Unknown | "
                            
#                         # Add servo angles
#                         status += f"Servos: "
#                         for i, angle in enumerate(self.servo_angles):
#                             status += f"{i}:{angle}° "
                        
#                         # Add solenoid states
#                         status += f"| Solenoids: "
#                         for i, state in enumerate(self.solenoid_states):
#                             status += f"{i}:{state} "
                            
#                         status += f"{Colors.ENDC}"
                        
#                         # Display status
#                         print(status, end='')
#                         sys.stdout.flush()
                            
#                     time.sleep(0.5)  # 2Hz update rate
                    
#             except Exception as e:
#                 print(f"\n{Colors.RED}Monitoring error: {str(e)}{Colors.ENDC}")
#             finally:
#                 self.monitoring = False
#                 print(f"\n{Colors.YELLOW}Monitoring stopped{Colors.ENDC}")
                
#         # Start monitoring thread
#         self.monitor_thread = threading.Thread(target=monitor_loop)
#         self.monitor_thread.daemon = True
#         self.monitor_thread.start()

#     def stop_monitoring(self):
#         """Stop monitoring if active"""
#         if not self.monitoring:
#             print(f"{Colors.YELLOW}Monitoring is not active{Colors.ENDC}")
#             return
            
#         self.monitoring = False
#         if self.monitor_thread:
#             self.monitor_thread.join(timeout=1.0)
#             print(f"{Colors.GREEN}Monitoring stopped{Colors.ENDC}")

#     # === Utility Methods ===
    
#     def get_motor_code(self, motor):
#         """Convert motor name to code used in protocol."""
#         motor = motor.lower()
#         if motor in ['left', 'l']:
#             return 'l'
#         elif motor in ['right', 'r']:
#             return 'r'
#         else:
#             print(f"{Colors.RED}Error: Invalid motor '{motor}'. Use 'left' or 'right'{Colors.ENDC}")
#             return None
            
#     def print_status(self):
#         """Print current status of all robot components"""
#         # Print stepper positions
#         print(f"{Colors.CYAN}=== Robot Status ==={Colors.ENDC}")
#         print(f"{Colors.CYAN}Stepper Positions:{Colors.ENDC}")
#         print(f"  Left: {self.left_position if self.left_position is not None else 'Unknown'}")
#         print(f"  Right: {self.right_position if self.right_position is not None else 'Unknown'}")
        
#         # Print servo angles
#         print(f"{Colors.CYAN}Servo Angles (degrees):{Colors.ENDC}")
#         for i, angle in enumerate(self.servo_angles):
#             print(f"  Servo {i}: {angle}°")
            
#         # Print solenoid states
#         print(f"{Colors.CYAN}Solenoid States:{Colors.ENDC}")
#         for i, state in enumerate(self.solenoid_states):
#             print(f"  Solenoid {i}: {'ON' if state else 'OFF'}")
    
#     # Combined command to set all components at once
#     def set_all_components(self, stepper_pos, servo_angles, solenoid_states):
#         """Set all robot components at once"""
#         success = True
        
#         # Validate inputs
#         if len(servo_angles) != NUM_SERVOS:
#             print(f"{Colors.RED}Error: Expected {NUM_SERVOS} servo angles{Colors.ENDC}")
#             return False
            
#         if len(solenoid_states) != NUM_SOLENOIDS:
#             print(f"{Colors.RED}Error: Expected {NUM_SOLENOIDS} solenoid states{Colors.ENDC}")
#             return False
        
#         # Set stepper position
#         try:
#             self.move_motor('left', stepper_pos)
#         except Exception as e:
#             print(f"{Colors.RED}Error setting stepper: {str(e)}{Colors.ENDC}")
#             success = False
        
#         # Set servo angles
#         for i, angle in enumerate(servo_angles):
#             try:
#                 self.set_servo_angle(i, angle)
#             except Exception as e:
#                 print(f"{Colors.RED}Error setting servo {i}: {str(e)}{Colors.ENDC}")
#                 success = False
        
#         # Set solenoid states
#         for i, state in enumerate(solenoid_states):
#             try:
#                 self.set_solenoid(i, state)
#             except Exception as e:
#                 print(f"{Colors.RED}Error setting solenoid {i}: {str(e)}{Colors.ENDC}")
#                 success = False
                
#         return success
        
#     def run_cli(self):
#         """Run the interactive command-line interface"""
#         try:
#             while True:
#                 # Get user input with custom prompt
#                 command = input(f"{Colors.BOLD}{Colors.BLUE}robot> {Colors.ENDC}").strip()
                
#                 # Skip empty commands
#                 if not command:
#                     continue
                    
#                 # Process command
#                 self.process_command(command)
                
#         except KeyboardInterrupt:
#             print(f"\n{Colors.YELLOW}Interrupted by user{Colors.ENDC}")
#         except Exception as e:
#             print(f"\n{Colors.RED}Error: {str(e)}{Colors.ENDC}")
#         finally:
#             self.cleanup()
            
#     def process_command(self, command):
#         """Process a command entered by the user"""
#         # Convert to lowercase
#         cmd_lower = command.lower()
        
#         # Exit commands
#         if cmd_lower in ["exit", "quit", "q"]:
#             print(f"{Colors.GREEN}Exiting...{Colors.ENDC}")
#             raise KeyboardInterrupt
            
#         # Help command
#         elif cmd_lower == "help":
#             self.print_help()
            
#         # Clear screen
#         elif cmd_lower in ["clear", "cls"]:
#             os.system('cls' if os.name == 'nt' else 'clear')
#             self.print
#             os.system('cls' if os.name == 'nt' else 'clear')
#             self.print_banner()
            
#         # Status command
#         elif cmd_lower == "status":
#             self.print_status()
            
#         # Calibration
#         elif cmd_lower in ["calibrate", "cal"]:
#             self.calibrate_motors()
            
#         # Position query
#         elif cmd_lower.startswith("position ") or cmd_lower.startswith("p "):
#             parts = cmd_lower.split(maxsplit=1)
#             if len(parts) == 2:
#                 motor = parts[1]
#                 self.get_position(motor)
#             else:
#                 print(f"{Colors.RED}Error: Missing motor specification. Use 'position <motor>'{Colors.ENDC}")
                
#         # Stepper move command
#         elif cmd_lower.startswith("move ") or cmd_lower.startswith("m "):
#             parts = re.split(r'\s+', cmd_lower, maxsplit=2)
#             if len(parts) >= 3:
#                 motor = parts[1]
#                 try:
#                     position = float(parts[2])
#                     self.move_motor(motor, position)
#                 except ValueError:
#                     print(f"{Colors.RED}Error: Invalid position '{parts[2]}'. Must be a number{Colors.ENDC}")
#             else:
#                 print(f"{Colors.RED}Error: Invalid format. Use 'move <motor> <position>'{Colors.ENDC}")

#         # Home command
#         elif cmd_lower == "home":
#             self.home_motors()
            
#         # Servo command
#         elif cmd_lower.startswith("servo ") or cmd_lower.startswith("s "):
#             parts = re.split(r'\s+', cmd_lower, maxsplit=2)
#             if len(parts) >= 3:
#                 try:
#                     channel = int(parts[1])
#                     angle = float(parts[2])
#                     if channel < 0 or channel >= NUM_SERVOS:
#                         print(f"{Colors.RED}Error: Invalid servo channel. Must be 0-{NUM_SERVOS-1}{Colors.ENDC}")
#                     else:
#                         self.set_servo_angle(channel, angle)
#                         print(f"{Colors.GREEN}Set servo {channel} to {angle} degrees{Colors.ENDC}")
#                 except ValueError:
#                     print(f"{Colors.RED}Error: Invalid servo parameters. Use numbers for channel and angle{Colors.ENDC}")
#             else:
#                 print(f"{Colors.RED}Error: Invalid format. Use 'servo <channel> <angle>'{Colors.ENDC}")
                
#         # Servos status
#         elif cmd_lower == "servos":
#             print(f"{Colors.CYAN}Current servo angles:{Colors.ENDC}")
#             for i, angle in enumerate(self.servo_angles):
#                 print(f"  Servo {i}: {angle}°")
                
#         # Solenoid command
#         elif cmd_lower.startswith("solenoid ") or cmd_lower.startswith("sol "):
#             parts = re.split(r'\s+', cmd_lower, maxsplit=2)
#             if len(parts) >= 3:
#                 try:
#                     channel = int(parts[1])
#                     state = int(parts[2])
#                     if channel < 0 or channel >= NUM_SOLENOIDS:
#                         print(f"{Colors.RED}Error: Invalid solenoid channel. Must be 0-{NUM_SOLENOIDS-1}{Colors.ENDC}")
#                     else:
#                         self.set_solenoid(channel, state)
#                         print(f"{Colors.GREEN}Set solenoid {channel} to {'ON' if state else 'OFF'}{Colors.ENDC}")
#                 except ValueError:
#                     print(f"{Colors.RED}Error: Invalid solenoid parameters. Use numbers for channel and state{Colors.ENDC}")
#             else:
#                 print(f"{Colors.RED}Error: Invalid format. Use 'solenoid <channel> <state>'{Colors.ENDC}")
                
#         # Set all solenoids
#         elif cmd_lower.startswith("solenoids "):
#             parts = cmd_lower.split(maxsplit=1)
#             if len(parts) == 2:
#                 try:
#                     state = int(parts[1])
#                     self.set_all_solenoids(state)
#                     print(f"{Colors.GREEN}Set all solenoids to {'ON' if state else 'OFF'}{Colors.ENDC}")
#                 except ValueError:
#                     print(f"{Colors.RED}Error: Invalid state value. Use 0 (OFF) or 1 (ON){Colors.ENDC}")
#             else:
#                 print(f"{Colors.RED}Error: Invalid format. Use 'solenoids <state>'{Colors.ENDC}")
                
#         # Combined command for all components
#         elif cmd_lower.startswith("setall "):
#             try:
#                 # Parse command: setall <stepper_pos> <s0> <s1> <s2> <s3> <s4> <sol0> <sol1> <sol2> <sol3> <sol4>
#                 parts = re.split(r'\s+', cmd_lower)[1:]
#                 if len(parts) != 1 + NUM_SERVOS + NUM_SOLENOIDS:
#                     print(f"{Colors.RED}Error: Expected format is 'setall <stepper_pos> <s0> <s1> <s2> <s3> <s4> <sol0> <sol1> <sol2> <sol3> <sol4>'{Colors.ENDC}")
#                     return
                
#                 # Parse values
#                 stepper_pos = float(parts[0])
#                 servo_angles = [float(parts[i+1]) for i in range(NUM_SERVOS)]
#                 solenoid_states = [int(parts[i+1+NUM_SERVOS]) for i in range(NUM_SOLENOIDS)]
                
#                 # Execute command
#                 self.set_all_components(stepper_pos, servo_angles, solenoid_states)
                
#             except ValueError as e:
#                 print(f"{Colors.RED}Error parsing values: {str(e)}{Colors.ENDC}")
#                 print(f"{Colors.YELLOW}All values must be numbers (stepper, servos=float, solenoids=int){Colors.ENDC}")
#             except Exception as e:
#                 print(f"{Colors.RED}Error executing command: {str(e)}{Colors.ENDC}")
                
#         # Monitor positions
#         elif cmd_lower == "monitor":
#             self.monitor_positions()
            
#         # Stop monitoring
#         elif cmd_lower == "stop":
#             self.stop_monitoring()
            
#         # Raw command
#         elif cmd_lower.startswith("raw "):
#             raw_cmd = command[4:].strip()
#             if raw_cmd:
#                 # For raw commands starting with 'r', we need to read response
#                 if raw_cmd.startswith('r') and len(raw_cmd) == 2:
#                     motor_code = raw_cmd[1].lower()
#                     if motor_code in ['l', 'r']:
#                         self.read_position(motor_code)
#                     else:
#                         print(f"{Colors.RED}Invalid motor code in raw command{Colors.ENDC}")
#                 else:
#                     # Just send the command without expecting response
#                     self.send_command(raw_cmd)
#             else:
#                 print(f"{Colors.RED}Error: No command specified after 'raw'{Colors.ENDC}")
                
#         # Unknown command
#         else:
#             print(f"{Colors.RED}Unknown command: '{command}'. Type 'help' for available commands{Colors.ENDC}")
    
#     def print_help(self):
#         """Display detailed help information"""
#         help_text = f"""
# {Colors.HEADER}{Colors.BOLD}╔═══════════════════════════════════════════════════════════╗
# ║ Robot Teleoperator - Command Reference                    ║
# ╚═══════════════════════════════════════════════════════════╝{Colors.ENDC}

# {Colors.BOLD}Basic Commands:{Colors.ENDC}
#   {Colors.CYAN}help{Colors.ENDC}                    Show this help message
#   {Colors.CYAN}status{Colors.ENDC}                  Show current status of all components
#   {Colors.CYAN}exit, quit, q{Colors.ENDC}           Exit the program
#   {Colors.CYAN}clear, cls{Colors.ENDC}              Clear the screen

# {Colors.BOLD}Stepper Motor Control:{Colors.ENDC}
#   {Colors.CYAN}calibrate, cal{Colors.ENDC}          Initialize calibration sequence for both motors
#                             {Colors.YELLOW}This moves the motors to their limits to establish the range{Colors.ENDC}

#   {Colors.CYAN}move <motor> <pos>{Colors.ENDC}      Move specified stepper to position (in meters)
#   {Colors.CYAN}m <motor> <pos>{Colors.ENDC}         {Colors.YELLOW}Example: move left 0.5  (moves left motor to 0.5m){Colors.ENDC}

#   {Colors.CYAN}position <motor>{Colors.ENDC}        Get current position of specified stepper motor
#   {Colors.CYAN}p <motor>{Colors.ENDC}               {Colors.YELLOW}Example: position right  (shows right motor position){Colors.ENDC}

#   {Colors.CYAN}home{Colors.ENDC}                    Move both stepper motors to their home positions
#                             {Colors.YELLOW}This returns motors to center position after calibration{Colors.ENDC}

# {Colors.BOLD}Servo Control:{Colors.ENDC}
#   {Colors.CYAN}servo <channel> <angle>{Colors.ENDC} Set servo angle (0-180 degrees)
#   {Colors.CYAN}s <channel> <angle>{Colors.ENDC}     {Colors.YELLOW}Example: servo 2 90  (sets servo 2 to 90 degrees){Colors.ENDC}

#   {Colors.CYAN}servos{Colors.ENDC}                  Show current angles of all servos

# {Colors.BOLD}Solenoid Control:{Colors.ENDC}
#   {Colors.CYAN}solenoid <channel> <state>{Colors.ENDC} Set solenoid state (0=OFF, 1=ON)
#   {Colors.CYAN}sol <channel> <state>{Colors.ENDC}   {Colors.YELLOW}Example: solenoid 1 1  (turns ON solenoid 1){Colors.ENDC}

#   {Colors.CYAN}solenoids <state>{Colors.ENDC}       Set all solenoids to the same state
#                             {Colors.YELLOW}Example: solenoids 0  (turns OFF all solenoids){Colors.ENDC}

# {Colors.BOLD}Combined Control:{Colors.ENDC}
#   {Colors.CYAN}setall <stepper> <s0> <s1> <s2> <s3> <s4> <sol0> <sol1> <sol2> <sol3> <sol4>{Colors.ENDC}
#                             {Colors.YELLOW}Control all components at once with a single command{Colors.ENDC}
#                             {Colors.YELLOW}Example: setall 0.5 90 100 80 95 90 1 0 0 1 0{Colors.ENDC}

# {Colors.BOLD}Monitoring:{Colors.ENDC}
#   {Colors.CYAN}monitor{Colors.ENDC}                 Start continuous monitoring of all components
#   {Colors.CYAN}stop{Colors.ENDC}                    Stop monitoring

# {Colors.BOLD}Raw Commands:{Colors.ENDC}
#   {Colors.CYAN}raw <command>{Colors.ENDC}           Send a raw command to the motor controller
#                             {Colors.YELLOW}Example: raw pl0.5  (move left motor to 0.5m){Colors.ENDC}

# {Colors.GREEN}All commands are case-insensitive.{Colors.ENDC}
# """
#         print(help_text)

# def main():
#     """Main entry point for the application"""
#     parser = argparse.ArgumentParser(description="Robot Teleoperator Control System")
    
#     # SPI options
#     spi_group = parser.add_argument_group('SPI Options (Stepper Motors)')
#     spi_group.add_argument("--spi-bus", type=int, default=SPI_BUS, help="SPI bus number (default: 0)")
#     spi_group.add_argument("--spi-device", type=int, default=SPI_DEVICE, help="SPI device number (default: 0)")
#     spi_group.add_argument("--spi-speed", type=int, default=SPI_SPEED, help="SPI clock speed in Hz (default: 500000)")
#     spi_group.add_argument("--spi-mode", type=int, default=SPI_MODE, help="SPI mode (0-3) (default: 1)")
    
#     # I2C options
#     i2c_group = parser.add_argument_group('I2C Options (Servos & Solenoids)')
#     i2c_group.add_argument("--i2c-bus", type=int, default=I2C_BUS, help="I2C bus number (default: 1)")
    
#     # Other options
#     parser.add_argument("--no-color", action="store_true", help="Disable colored output")
#     parser.add_argument("--disable-servos", action="store_true", help="Skip servo initialization")
#     parser.add_argument("--disable-solenoids", action="store_true", help="Skip solenoid initialization")
    
#     args = parser.parse_args()
    
#     # Disable colors if requested
#     if args.no_color:
#         Colors.disable()
    
#     try:
#         print(f"{Colors.CYAN}Initializing robot teleoperator...{Colors.ENDC}")
#         controller = RobotTeleoperator(
#             spi_bus=args.spi_bus, 
#             spi_device=args.spi_device, 
#             spi_speed=args.spi_speed, 
#             spi_mode=args.spi_mode,
#             i2c_bus=args.i2c_bus
#         )
#         controller.run_cli()
#     except Exception as e:
#         print(f"{Colors.RED}Fatal error: {str(e)}{Colors.ENDC}")
#         return 1
        
#     return 0


# if __name__ == "__main__":
#     sys.exit(main())