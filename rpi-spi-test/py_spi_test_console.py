#!/usr/bin/env python3
"""
Raspberry Pi Pico SPI Controller

A command-line interface for controlling stepper motors on a Raspberry Pi Pico
via SPI communication.

This utility provides an intuitive interface for calibrating motors, setting positions,
and monitoring the current state of the system.
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

# SPI configuration
SPI_BUS = 0
SPI_DEVICE = 0
SPI_SPEED = 500000  # 500 kHz
SPI_MODE = 1        # SPI mode (CPOL=0, CPHA=1)

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


class PicoStepperController:
    """Main controller class for communicating with the Pico over SPI"""
    
    def __init__(self, bus=SPI_BUS, device=SPI_DEVICE, speed=SPI_SPEED, mode=SPI_MODE):
        """Initialize the SPI connection to the Pico controller.
        
        Args:
            bus (int): SPI bus number (default: 0)
            device (int): SPI device/chip select number (default: 0)
            speed (int): SPI clock speed in Hz (default: 500000)
            mode (int): SPI mode (0-3) (default: 1)
        """
        # Initialize SPI
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.max_speed_hz = speed
        self.spi.mode = mode
        self.spi.bits_per_word = 8
        self.spi.lsbfirst = False
        
        # State tracking
        self.monitoring = False
        self.monitor_thread = None
        self.left_position = None
        self.right_position = None
        
        # PID parameters tracking
        self.left_pid = {"kp": 25000.0, "ki": 0.0, "kd": 5000.0}
        self.right_pid = {"kp": 25000.0, "ki": 0.0, "kd": 5000.0}
        
        # Capture Ctrl+C for clean shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        
        # Print connection info
        self.last_command_time = datetime.now()
        self.print_banner()
        
    def print_banner(self):
        """Print application banner with connection information"""
        banner = f"""
{Colors.HEADER}{Colors.BOLD}╔═══════════════════════════════════════════════════════════╗
║ Raspberry Pi Pico Stepper Controller                      ║
╚═══════════════════════════════════════════════════════════╝{Colors.ENDC}
{Colors.CYAN}• SPI Connection: Bus {SPI_BUS}, Device {SPI_DEVICE}
• Speed: {SPI_SPEED/1000:.1f} kHz, Mode: {SPI_MODE}
• Protocol: Byte-oriented, null-terminated commands{Colors.ENDC}

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
        self.spi.close()
        print(f"{Colors.GREEN}SPI connection closed. Goodbye!{Colors.ENDC}")

    def send_command(self, command):
        """Send a command to the Pico over SPI without expecting a response.
        
        Args:
            command (str): Command to send
        """
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
            
        print(f"{Colors.GREEN}Command sent successfully{Colors.ENDC}")

    def read_position(self, motor):
        """Read position from a specific motor.
        
        Args:
            motor (str): Motor code ('l' or 'r')
            
        Returns:
            float: Current position in meters or None if error
        """
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

    # === Motor Commands ===
    
    def calibrate_motors(self):
        """Send the calibration command to both motors.
        
        This initiates the calibration sequence on the Pico. The motors will move
        to their limit switches to establish the range of motion.
        """
        print(f"{Colors.YELLOW}Initiating motor calibration...{Colors.ENDC}")
        print(f"{Colors.YELLOW}This will move both motors to their limits. Ensure path is clear!{Colors.ENDC}")
        
        # Send calibration command 'c'
        self.send_command('c')
        print(f"{Colors.YELLOW}Note: Calibration takes some time to complete.{Colors.ENDC}")

    def move_motor(self, motor, position):
        """Move a stepper motor to the specified position.
        
        Args:
            motor (str): Motor to move ('left'/'l' or 'right'/'r')
            position (float): Target position in meters
        """
        # Validate motor parameter
        motor_code = self.get_motor_code(motor)
        if not motor_code:
            return
            
        # Send position command: p<motor_code><position>
        command = f"p{motor_code}{position}"
        self.send_command(command)
        
        print(f"{Colors.GREEN}Command sent to move {motor} motor to {position} meters{Colors.ENDC}")

    def get_position(self, motor):
        """Get the current position of a motor.
        
        Args:
            motor (str): Motor to query ('left'/'l' or 'right'/'r')
            
        Returns:
            float: Current position in meters or None if error
        """
        # Get position using the read_position method
        position = self.read_position(motor)
        
        if position is not None:
            print(f"{Colors.GREEN}{motor.capitalize()} motor position: {position:.6f} meters{Colors.ENDC}")
            return position
        return None

    def home_motors(self):
        """Send the home command to both motors.
        
        This tells the motors to return to their home position (usually center).
        """
        print(f"{Colors.YELLOW}Sending motors to home position...{Colors.ENDC}")
        
        # Send home command 'h'
        self.send_command('h')
        print(f"{Colors.YELLOW}Home command sent.{Colors.ENDC}")

    def tune_pid(self, motor, kp, ki, kd):
        """Set PID parameters for a motor.
        
        Args:
            motor (str): Motor to tune ('left'/'l' or 'right'/'r')
            kp (float): Proportional gain
            ki (float): Integral gain
            kd (float): Derivative gain
        """
        # Validate motor parameter
        motor_code = self.get_motor_code(motor)
        if not motor_code:
            return
            
        # Update local PID parameters
        if motor_code == 'l':
            self.left_pid = {"kp": kp, "ki": ki, "kd": kd}
        else:
            self.right_pid = {"kp": kp, "ki": ki, "kd": kd}
            
        # Send PID tuning command: t<motor_code><kp>,<ki>,<kd>
        command = f"t{motor_code}{kp},{ki},{kd}"
        self.send_command(command)
        
        print(f"{Colors.GREEN}PID parameters for {motor} motor set to: Kp={kp}, Ki={ki}, Kd={kd}{Colors.ENDC}")
        
    def get_pid_parameters(self, motor):
        """Get the current PID parameters for a motor.
        
        Args:
            motor (str): Motor to query ('left'/'l' or 'right'/'r')
        """
        # Validate motor parameter
        motor_code = self.get_motor_code(motor)
        if not motor_code:
            return
            
        if motor_code == 'l':
            pid = self.left_pid
        else:
            pid = self.right_pid
            
        print(f"{Colors.GREEN}Current PID parameters for {motor} motor:{Colors.ENDC}")
        print(f"{Colors.CYAN}  Kp: {pid['kp']}{Colors.ENDC}")
        print(f"{Colors.CYAN}  Ki: {pid['ki']}{Colors.ENDC}")
        print(f"{Colors.CYAN}  Kd: {pid['kd']}{Colors.ENDC}")

    def monitor_positions(self):
        """Start continuous monitoring of both motor positions.
        
        This function runs in a separate thread and periodically queries
        the position of both motors.
        """
        if self.monitoring:
            print(f"{Colors.YELLOW}Position monitoring is already active{Colors.ENDC}")
            return
            
        self.monitoring = True
        
        def monitor_loop():
            """Background thread function for position monitoring"""
            print(f"{Colors.GREEN}Starting position monitoring (press Ctrl+C or type 'stop' to end){Colors.ENDC}")
            
            try:
                while self.monitoring:
                    # Only query if we haven't received a command recently
                    time_since_command = (datetime.now() - self.last_command_time).total_seconds()
                    if time_since_command > 0.5:  # Don't interfere with manual commands
                        l_pos = self.read_position('left')
                        r_pos = self.read_position('right')
                        
                        if l_pos is not None and r_pos is not None:
                            # Clear previous line and print current positions
                            print(f"\r{Colors.CYAN}Left: {l_pos:.6f}m | Right: {r_pos:.6f}m{Colors.ENDC}", end='')
                            sys.stdout.flush()
                            
                    time.sleep(0.5)  # 2Hz update rate
                    
            except Exception as e:
                print(f"\n{Colors.RED}Monitoring error: {str(e)}{Colors.ENDC}")
            finally:
                self.monitoring = False
                print(f"\n{Colors.YELLOW}Position monitoring stopped{Colors.ENDC}")
                
        # Start monitoring thread
        self.monitor_thread = threading.Thread(target=monitor_loop)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()

    def stop_monitoring(self):
        """Stop position monitoring if active"""
        if not self.monitoring:
            print(f"{Colors.YELLOW}Position monitoring is not active{Colors.ENDC}")
            return
            
        self.monitoring = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=1.0)
            print(f"{Colors.GREEN}Position monitoring stopped{Colors.ENDC}")

    # === Utility Methods ===
    
    def get_motor_code(self, motor):
        """Convert motor name to code used in protocol.
        
        Args:
            motor (str): Motor name ('left', 'l', 'right', 'r')
            
        Returns:
            str: Motor code ('l' or 'r') or None if invalid
        """
        motor = motor.lower()
        if motor in ['left', 'l']:
            return 'l'
        elif motor in ['right', 'r']:
            return 'r'
        else:
            print(f"{Colors.RED}Error: Invalid motor '{motor}'. Use 'left' or 'right'{Colors.ENDC}")
            return None
            
    def print_help(self):
        """Display detailed help information"""
        help_text = f"""
{Colors.HEADER}{Colors.BOLD}╔═══════════════════════════════════════════════════════════╗
║ Raspberry Pi Pico Stepper Controller - Command Reference  ║
╚═══════════════════════════════════════════════════════════╝{Colors.ENDC}

{Colors.BOLD}Basic Commands:{Colors.ENDC}
  {Colors.CYAN}help{Colors.ENDC}                    Show this help message
  {Colors.CYAN}exit, quit, q{Colors.ENDC}           Exit the program
  {Colors.CYAN}clear, cls{Colors.ENDC}              Clear the screen

{Colors.BOLD}Motor Control:{Colors.ENDC}
  {Colors.CYAN}calibrate, cal{Colors.ENDC}          Initialize calibration sequence for both motors
                            {Colors.YELLOW}This moves the motors to their limits to establish the range{Colors.ENDC}

  {Colors.CYAN}move <motor> <pos>{Colors.ENDC}      Move specified motor to position (in meters)
  {Colors.CYAN}m <motor> <pos>{Colors.ENDC}         {Colors.YELLOW}Example: move left 0.5  (moves left motor to 0.5m){Colors.ENDC}

  {Colors.CYAN}position <motor>{Colors.ENDC}        Get current position of specified motor
  {Colors.CYAN}p <motor>{Colors.ENDC}               {Colors.YELLOW}Example: position right  (shows right motor position){Colors.ENDC}

  {Colors.CYAN}home{Colors.ENDC}                    Move both motors to their home positions
                            {Colors.YELLOW}This returns motors to center position after calibration{Colors.ENDC}

{Colors.BOLD}PID Control:{Colors.ENDC}
  {Colors.CYAN}pid <motor> <kp> <ki> <kd>{Colors.ENDC}  Set PID parameters for a motor
                            {Colors.YELLOW}Example: pid left 25000 0 5000{Colors.ENDC}

  {Colors.CYAN}getpid <motor>{Colors.ENDC}          Show current PID parameters for a motor
                            {Colors.YELLOW}Example: getpid right{Colors.ENDC}

{Colors.BOLD}Monitoring:{Colors.ENDC}
  {Colors.CYAN}monitor{Colors.ENDC}                 Start continuous position monitoring of both motors
  {Colors.CYAN}stop{Colors.ENDC}                    Stop position monitoring

{Colors.BOLD}Advanced Commands:{Colors.ENDC}
  {Colors.CYAN}raw <command>{Colors.ENDC}           Send a raw command to the Pico
                            {Colors.YELLOW}Example: raw pl0.5  (move left motor to 0.5m){Colors.ENDC}

{Colors.BOLD}Motor Specification:{Colors.ENDC}
  Motors can be specified as:
    • {Colors.CYAN}left{Colors.ENDC} or {Colors.CYAN}l{Colors.ENDC} for the left motor
    • {Colors.CYAN}right{Colors.ENDC} or {Colors.CYAN}r{Colors.ENDC} for the right motor

{Colors.BOLD}Raw Command Protocol:{Colors.ENDC}
  {Colors.CYAN}c{Colors.ENDC}                       Calibrate both motors
  {Colors.CYAN}p<motor><position>{Colors.ENDC}      Move motor to position (ex: pl0.5, pr-0.2)
  {Colors.CYAN}r<motor>{Colors.ENDC}                Query motor position (ex: rl, rr)
  {Colors.CYAN}h{Colors.ENDC}                       Home both motors
  {Colors.CYAN}t<motor><kp>,<ki>,<kd>{Colors.ENDC}  Set PID parameters (ex: tl25000,0,5000)

{Colors.GREEN}All commands are case-insensitive.{Colors.ENDC}
"""
        print(help_text)
        
    def run_cli(self):
        """Run the interactive command-line interface"""
        try:
            while True:
                # Get user input with custom prompt
                command = input(f"{Colors.BOLD}{Colors.BLUE}pico> {Colors.ENDC}").strip()
                
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
        """Process a command entered by the user
        
        Args:
            command (str): Command string to process
        """
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
            
        # Calibration
        elif cmd_lower in ["calibrate", "cal"]:
            self.calibrate_motors()
            
        # Position query
        elif cmd_lower.startswith("position ") or cmd_lower.startswith("p "):
            parts = cmd_lower.split(maxsplit=1)
            if len(parts) == 2:
                motor = parts[1]
                self.get_position(motor)
            else:
                print(f"{Colors.RED}Error: Missing motor specification. Use 'position <motor>'{Colors.ENDC}")
                
        # Move command
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
        elif cmd_lower in ["home"]:
            self.home_motors()

        # PID tuning command
        elif cmd_lower.startswith("pid "):
            parts = re.split(r'\s+', cmd_lower, maxsplit=4)
            if len(parts) >= 5:
                motor = parts[1]
                try:
                    kp = float(parts[2])
                    ki = float(parts[3])
                    kd = float(parts[4])
                    self.tune_pid(motor, kp, ki, kd)
                except ValueError:
                    print(f"{Colors.RED}Error: Invalid PID parameters. Must be numbers{Colors.ENDC}")
            else:
                print(f"{Colors.RED}Error: Invalid format. Use 'pid <motor> <kp> <ki> <kd>'{Colors.ENDC}")

        # Get PID parameters command
        elif cmd_lower.startswith("getpid "):
            parts = cmd_lower.split(maxsplit=1)
            if len(parts) == 2:
                motor = parts[1]
                self.get_pid_parameters(motor)
            else:
                print(f"{Colors.RED}Error: Missing motor specification. Use 'getpid <motor>'{Colors.ENDC}")
                
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


def main():
    """Main entry point for the application"""
    parser = argparse.ArgumentParser(description="Raspberry Pi Pico Stepper Controller")
    parser.add_argument("-b", "--bus", type=int, default=SPI_BUS, help="SPI bus number (default: 0)")
    parser.add_argument("-d", "--device", type=int, default=SPI_DEVICE, help="SPI device number (default: 0)")
    parser.add_argument("-s", "--speed", type=int, default=SPI_SPEED, help="SPI clock speed in Hz (default: 500000)")
    parser.add_argument("-m", "--mode", type=int, default=SPI_MODE, help="SPI mode (0-3) (default: 1)")
    parser.add_argument("--no-color", action="store_true", help="Disable colored output")
    
    args = parser.parse_args()
    
    # Disable colors if requested
    if args.no_color:
        Colors.disable()
    
    try:
        controller = PicoStepperController(
            bus=args.bus, 
            device=args.device, 
            speed=args.speed, 
            mode=args.mode
        )
        controller.run_cli()
    except Exception as e:
        print(f"{Colors.RED}Fatal error: {str(e)}{Colors.ENDC}")
        return 1
        
    return 0


if __name__ == "__main__":
    sys.exit(main())