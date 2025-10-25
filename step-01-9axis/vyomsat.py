#!/usr/bin/env python3
# MIT License
#
# Copyright (c) 2025 Ojas Jha
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""
VyomSat EPS (Electrical Power System) with UART Communication Module
====================================================================

VyomSat - Essence of Space. Built by You.
The CubeSat education kit - Hands-on CubeSat engineering, end-to-end

OVERVIEW:
========
This module combines battery voltage monitoring with UART communication
capabilities for the VyomSat CubeSat education kit. It monitors the
Electrical Power System (EPS) and communicates telemetry data via UART.

FEATURES:
========
- Battery voltage monitoring via ADC
- OLED display for local telemetry visualization
- UART communication for remote telemetry and command reception
- Onboard LED status indication
- Full duplex communication (simultaneous send/receive)

HARDWARE REQUIREMENTS:
=====================
- Raspberry Pi Pico 2 or Pico W microcontroller
- SSD1306 OLED Display (128x64 pixels) connected via I2C:
  * SCL (Serial Clock) → GPIO 7
  * SDA (Serial Data) → GPIO 6
  * VCC → 3.3V
  * GND → Ground
- UART Device (e.g., XBee module) connected to:
  * Pico GP0 (UART0 TX) → Device RX pin
  * Pico GP1 (UART0 RX) ← Device TX pin
  * Pico GP2 → Device RST pin (optional but recommended)
  * Pico GND ↔ Device GND (CRITICAL: Common ground reference)
- Battery voltage divider circuit connected to GP28 (ADC2)
- USB connection for serial output monitoring

WIRING SUMMARY:
==============
- Battery voltage sensing: GP28 (ADC2) via voltage divider
- OLED Display: GP4 (SDA), GP5 (SCL)
- UART Communication: GP0 (TX), GP1 (RX), GP2 (RST - optional)
- Status LED: Built-in LED (GPIO 25 or "LED")
- External GPIO: GP22 for testing (optional)

USAGE:
=====
1. Connect all hardware as described above
2. Upload this script to the Raspberry Pi Pico
3. Run the script
4. System will:
   - Display battery voltage on OLED
   - Send telemetry data via UART
   - Receive and process commands via UART
   - Print status to serial console

SUPPORTED COMMANDS:
==================
Send single character commands via UART:
- 'a': MPU9250 9-axis IMU attitude data (accelerometer, gyroscope, magnetometer)
- 'b': System reset
- 'c': Camera image capture and save to SD card
- 's': SD card data logging (HK + 9-AXIS + GPS)
- 'g': GPS data capture
- 'v': Request voltage reading

AUTHOR: VyomSat CubeSat Education Kit
VERSION: 3.0 - EPS + COM Integration
DATE: 2024
"""

# =============================================================================
# IMPORT STATEMENTS - MICROPYTHON LIBRARIES
# =============================================================================

# MicroPython Hardware Abstraction Layer
import machine
from machine import Pin, SoftI2C

# MicroPython Time Module
import utime

# UART Handler Class
from uart_handler import UartHandler

# OLED Manager
from vyomsat_oled_manager import OledManager

# Battery Manager
from vyomsat_battery_manager import (
    initialize_battery_sensor,
    read_battery_voltage,
    read_and_display_battery_voltage
)

# XBee Manager
from vyomsat_xbee_manager import perform_xbee_hardware_reset

# 9-Axis IMU Manager
from vyomsat_9axis_manager import (
    initialize_9axis_sensor,
    read_and_display_9axis_data,
    is_9axis_available
)

# =============================================================================
# CONFIGURATION CONSTANTS - HARDWARE AND COMMUNICATION SETTINGS
# =============================================================================

# OLED Display Configuration
# ==========================
OLED_SCL_PIN = 7              # Serial Clock Line GPIO pin
OLED_SDA_PIN = 6              # Serial Data Line GPIO pin
OLED_I2C_FREQ = 400000        # I2C bus frequency in Hz (400kHz)
OLED_WIDTH = 128              # Display width in pixels
OLED_HEIGHT = 64              # Display height in pixels
OLED_I2C_ADDR = 0x3C          # Default SSD1306 I2C address

# Battery Monitoring Configuration
# ================================
BATTERY_ADC_CHANNEL = 2              # ADC channel 2 (GPIO 28)
BATTERY_VOLTAGE_REFERENCE = 3.3      # Volts - RP2350 ADC reference
BATTERY_VOLTAGE_DIVIDER_FACTOR = 0.5 # Voltage divider ratio

# UART Configuration
# ==================
UART_INTERFACE_ID = 0         # UART0 interface
UART_TX_PIN = 0               # GP0 - transmit pin
UART_RX_PIN = 1               # GP1 - receive pin
UART_BAUDRATE = 9600          # 9600 bps - standard baudrate
MAX_LINE_BUFFER_SIZE = 256    # 256 bytes - prevents memory overflow

# XBee Reset Pin Configuration (optional but recommended)
# =======================================================
XBEE_RESET_PIN = 2            # GP2 - connected to XBee RST pin
XBEE_INIT_DELAY_SECONDS = 5   # Time to wait after reset

# Timing Configuration
# ====================
MAIN_LOOP_DELAY_SECONDS = 4   # Delay between telemetry transmissions

# 9-Axis IMU Sensor Configuration (MPU9250)
# ==========================================
NINE_AXIS_SCL_PIN = 21       # GPIO 21 for I2C Serial Clock
NINE_AXIS_SDA_PIN = 20       # GPIO 20 for I2C Serial Data
NINE_AXIS_I2C_FREQ = 400000  # I2C bus frequency in Hz (400kHz)

# =============================================================================
# HARDWARE INITIALIZATION
# =============================================================================

# Onboard LED Configuration
# -------------------------
led: Pin = Pin("LED", Pin.OUT)

# External GPIO Pin Configuration (optional)
# -----------------------------------------
power_on_pin: Pin = Pin(22, Pin.OUT)

# Global OLED Manager Object
# --------------------------
oled_manager = None

# Global UART Handler Object
# --------------------------
uart_handler = None

# =============================================================================
# UART COMMAND HANDLING FUNCTIONS
# =============================================================================

def on_command_received_via_uart(character_byte, uart_handler_instance):
    """
    Handle single character commands received via UART.

    This function is called when a single character is received via UART.
    It processes commands and provides appropriate responses via USB, UART,
    and OLED display.

    Supported Commands:
    - 'a': MPU9250 9-axis IMU attitude data capture
    - 'b': System reset
    - 'c': Camera image capture and save to SD card
    - 's': SD card data logging (HK + 9-AXIS + GPS)
    - 'g': GPS data capture
    - 'v': Request voltage reading

    Args:
        character_byte (int): The received byte value (0-255)
        uart_handler_instance (UartHandler): UART handler instance
    """
    # Ensure byte value is in valid range (0-255)
    byte_value = character_byte & 0xFF

    if 32 <= byte_value <= 126:  # Printable ASCII range
        command = chr(byte_value)

        # Display command on OLED immediately
        if oled_manager:
            oled_manager.display_rx_command(command)

        # Print command received via USB (for monitoring)
        print(f"UART Command Received: '{command}' (ASCII: {byte_value})")

        # Process the command
        if command == "a":
            print("Processing MPU9250 9-axis IMU attitude data capture...")
            usb_response = "Command 'a' received: MPU9250 capture initiated"
            uart_response = "ACK: MPU9250 capture initiated"

            # Read and display 9-axis attitude data
            # This function will cycle through 4 OLED screens (2 sec each)
            # Total display time: 8 seconds
            read_and_display_9axis_data(uart_handler_instance, oled_manager)
            
            # Brief pause before returning to regular telemetry display
            utime.sleep(1)

        elif command == "b":
            print("Processing system reset...")
            usb_response = "Command 'b' received: System reset initiated"
            uart_response = "ACK: System reset initiated"

        elif command == "c":
            print("Processing camera image capture and SD card save...")
            usb_response = "Command 'c' received: Camera capture initiated"
            uart_response = "ACK: Camera capture initiated"

        elif command == "s":
            print("Processing SD card data logging...")
            usb_response = "Command 's' received: SD card logging initiated"
            uart_response = "ACK: SD card logging initiated"

        elif command == "g":
            print("Processing GPS data capture...")
            usb_response = "Command 'g' received: GPS capture initiated"
            uart_response = "ACK: GPS capture initiated"

        elif command == "v":
            # Voltage request command
            print("Processing voltage request...")
            usb_response = "Command 'v' received: Battery voltage reading initiated"
            uart_response = "ACK: Battery voltage reading initiated"
            
            # Read and display battery voltage with OLED display feedback
            # This function will display voltage on OLED for 2 seconds
            read_and_display_battery_voltage(uart_handler_instance, oled_manager)
            
            # Brief pause before returning to regular telemetry display
            utime.sleep(1)

        else:
            print(f"Unknown command '{command}' received")
            usb_response = f"Unknown command '{command}' - no action taken"
            uart_response = f"NAK: Unknown command '{command}'"

        # Send response via USB (for monitoring)
        print(f"USB Response: {usb_response}")

        # Send response via UART back to the sender
        try:
            uart_response_formatted = f"{uart_response}\r\n"
            bytes_sent = uart_handler_instance.write(
                uart_response_formatted.encode()
            )
            print(f"UART Response sent: '{uart_response}' "
                  f"({bytes_sent} bytes)")
        except Exception as e:
            print(f"Failed to send UART response: {e}")

    else:
        # Display as hexadecimal value
        print(f"UART Command Handler: Control character (0x{byte_value:02X})")
        print("USB Response: Control character received - no action taken")

        # Display control character on OLED
        if oled_manager:
            oled_manager.display_rx_command(f"0x{byte_value:02X}")

        # Send response for control character via UART
        try:
            uart_response = "NAK: Control character"
            uart_response_formatted = f"{uart_response}\r\n"
            bytes_sent = uart_handler_instance.write(
                uart_response_formatted.encode()
            )
            print(f"UART Response sent: '{uart_response}' "
                  f"({bytes_sent} bytes)")
        except Exception as e:
            print(f"Failed to send UART response: {e}")

def on_message_received_via_uart(message_string, uart_handler_instance):
    """
    Handle complete line messages received via UART.

    This function is called when a complete line (terminated by CR/LF)
    is received via UART. It provides custom message handling and sends
    an acknowledgment back via UART and displays on OLED.

    Args:
        message_string (str): The received message string
        uart_handler_instance (UartHandler): UART handler instance
    """
    # Display message on OLED immediately
    if oled_manager:
        oled_manager.display_rx_message(message_string)

    # Custom message processing with enhanced formatting (USB output)
    print(f"Custom UART Message Handler: '{message_string}' "
          f"(Length: {len(message_string)} chars)")

    # Send acknowledgment back via UART
    try:
        ack_message = (f"ACK: Line processed - {len(message_string)} "
                      f"chars received\r\n")
        bytes_sent = uart_handler_instance.write(ack_message.encode())
        print(f"UART Acknowledgment sent: 'Line processed - "
              f"{len(message_string)} chars received' ({bytes_sent} bytes)")
    except Exception as e:
        print(f"Failed to send UART acknowledgment: {e}")

# =============================================================================
# MAIN APPLICATION FUNCTIONS
# =============================================================================

def main():
    """
    Main entry point for VyomSat EPS + COM integration.

    This function initializes all hardware components and runs the main
    communication and monitoring loop. It combines battery voltage monitoring,
    OLED display, and UART communication in a single application.

    APPLICATION LIFECYCLE:
    =====================
    1. Initialize hardware (LED, GPIO, OLED, UART)
    2. Perform XBee reset (if applicable)
    3. Run main monitoring and communication loop
    4. Handle user interruption (Ctrl+C)
    5. Handle unexpected errors gracefully

    MAIN LOOP:
    =========
    The main loop continuously:
    - Reads battery voltage
    - Updates OLED display
    - Sends telemetry via UART
    - Receives and processes commands via UART
    - Provides status information via USB serial
    """
    global uart_handler, oled_manager

    try:
        # Display initialization message
        print("=" * 70)
        print("🚀 VyomSat - Essence of Space. Built by You. 🚀")
        print("The CubeSat education kit - Hands-on CubeSat engineering")
        print("=" * 70)
        print("VyomSat EPS + COM Integration Module - Initializing...")

        # Initialize onboard LED and external GPIO
        led.value(1)  # Turn on LED
        power_on_pin.value(1)  # Set GPIO pin high

        # Perform XBee hardware reset (optional but recommended)
        # Comment out if not using XBee or if RST pin is not connected
        perform_xbee_hardware_reset(
            reset_pin=XBEE_RESET_PIN,
            init_delay_seconds=XBEE_INIT_DELAY_SECONDS
        )

        # Initialize Battery Manager
        print("Initializing battery voltage monitoring...")
        initialize_battery_sensor(
            adc_channel=BATTERY_ADC_CHANNEL,
            voltage_reference=BATTERY_VOLTAGE_REFERENCE,
            voltage_divider_factor=BATTERY_VOLTAGE_DIVIDER_FACTOR
        )

        # Initialize 9-Axis IMU Sensor
        print("Initializing MPU9250 9-axis IMU sensor...")
        nine_axis_init_success = initialize_9axis_sensor(
            scl_pin=NINE_AXIS_SCL_PIN,
            sda_pin=NINE_AXIS_SDA_PIN,
            i2c_frequency=NINE_AXIS_I2C_FREQ
        )
        if nine_axis_init_success:
            print("[OK]9-axis IMU sensor initialized successfully!")
        else:
            print("⚠ 9-axis IMU sensor not available - continuing without it")

        # Initialize OLED display
        print("Initializing OLED display...")
        oled_manager = OledManager(
            scl_pin=OLED_SCL_PIN,
            sda_pin=OLED_SDA_PIN,
            width=OLED_WIDTH,
            height=OLED_HEIGHT,
            i2c_freq=OLED_I2C_FREQ,
            i2c_addr=OLED_I2C_ADDR
        )
        oled_manager.initialize()

        # Initialize UART handler
        print("Initializing UART handler...")
        uart_handler = UartHandler(
            uart_interface_id=UART_INTERFACE_ID,
            uart_tx_pin=UART_TX_PIN,
            uart_rx_pin=UART_RX_PIN,
            uart_baudrate=UART_BAUDRATE,
            max_line_buffer_size=MAX_LINE_BUFFER_SIZE,
            on_command_received=on_command_received_via_uart,
            on_message_received=on_message_received_via_uart
        )
        print("[OK]UART handler initialized successfully!")

        # Display startup information
        print("=" * 70)
        print("VyomSat EPS + COM System Started")
        print("Supported Commands (send via UART):")
        print("  'a' - MPU9250 9-axis IMU attitude data capture")
        print("  'b' - System reset")
        print("  'c' - Camera image capture and save to SD card")
        print("  's' - SD card data logging (HK + 9-AXIS + GPS)")
        print("  'g' - GPS data capture")
        print("  'v' - Request voltage reading")
        print("=" * 70)

        # Initialize communication variables
        message_counter = 0  # Track number of messages sent

        # Main communication and monitoring loop
        while True:
            # Toggle LED for visual feedback
            led.toggle()

            # Read battery voltage
            battery_voltage = read_battery_voltage()

            # Update OLED display with telemetry
            if oled_manager:
                oled_manager.update_display(battery_voltage, message_counter)

            # Get current timestamp
            current_timestamp = utime.ticks_ms()

            # Format telemetry message
            telemetry_message = (f"[{current_timestamp}ms] VyomSat EPS | "
                               f"Voltage: {battery_voltage:.2f}V | "
                               f"Msg: #{message_counter}\r\n")

            # Send telemetry via UART
            bytes_sent = uart_handler.write(telemetry_message.encode())

            # Display transmission information via USB
            print(f"[{current_timestamp}ms] Telemetry Sent | "
                  f"Voltage: {battery_voltage:.2f}V | "
                  f"Bytes: {bytes_sent} | "
                  f"Counter: {message_counter}")

            # Increment counter for next message
            message_counter += 1

            # Wait before next transmission
            utime.sleep(MAIN_LOOP_DELAY_SECONDS)

    except KeyboardInterrupt:
        # Handle user interruption (Ctrl+C)
        print("\n" + "=" * 70)
        print("VyomSat EPS + COM stopped by user.")
        print("System shutdown complete.")
        print("=" * 70)

    except Exception as e:
        # Handle unexpected errors
        print(f"\nVyomSat Error: {e}")
        print("Please check your hardware connections and try again.")

# =============================================================================
# PROGRAM ENTRY POINT
# =============================================================================

if __name__ == "__main__":
    """
    Program entry point - only executes when script is run directly.

    This pattern allows the script to be imported as a module without
    automatically starting the main loop, which is useful for testing
    and code reusability.
    """
    main()

# =============================================================================
# EXPECTED BEHAVIOR & TROUBLESHOOTING
# =============================================================================
"""
EXPECTED OUTPUT:
===============
- Onboard LED blinks at regular intervals
- OLED display shows:
  * "VyomSat EPS" header
  * Current time in HH:MM:SS format
  * Battery voltage reading
  * Message counter
- Serial console shows telemetry transmissions
- UART device receives telemetry data
- UART commands are received and acknowledged

TROUBLESHOOTING:
===============
- No LED blinking: Check MicroPython installation and USB connection
- No OLED display: Check I2C wiring (SCL→GPIO5, SDA→GPIO4) and power
- OLED shows garbled text: Verify I2C address (try 0x3D if 0x3C fails)
- No UART communication: Check TX/RX wiring and baudrate settings
- Incorrect voltage: Verify voltage divider circuit and ADC connection
- System hangs: Check for hardware short circuits or power issues

HARDWARE NOTES:
==============
- RP2040 operates at 3.3V logic levels
- GPIO pins can source/sink up to 12mA safely
- External power supply recommended for high-current devices (XBee, etc.)
- Common ground reference is critical for UART communication
- I2C communication works at 400kHz for fast updates

To stop execution: Press Ctrl+C in the serial terminal
"""

