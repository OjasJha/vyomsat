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
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""
VyomSat OLED Display Manager
============================

This module provides a comprehensive OLED display manager for the VyomSat
CubeSat education kit. It handles initialization, updates, and display
operations for the SSD1306 OLED display.

FEATURES:
========
- OLED display initialization with I2C interface
- System status display (time, voltage, message counter)
- Command reception display
- Message reception display
- Error handling and graceful degradation

HARDWARE REQUIREMENTS:
=====================
- SSD1306 OLED Display (128x64 pixels) connected via I2C:
  * SCL (Serial Clock) → GPIO 7
  * SDA (Serial Data) → GPIO 6
  * VCC → 3.3V
  * GND → Ground

USAGE:
=====
    from vyomsat_oled_manager import OledManager

    # Create OLED manager instance
    oled_mgr = OledManager(
        scl_pin=7,
        sda_pin=6,
        width=128,
        height=64,
        i2c_freq=400000,
        i2c_addr=0x3C
    )

    # Initialize display
    oled_mgr.initialize()

    # Update display with system info
    oled_mgr.update_display(battery_voltage=6.90, message_counter=42)

    # Display received command
    oled_mgr.display_rx_command("v")

    # Display received message
    oled_mgr.display_rx_message("Hello from VyomSat")

AUTHOR: VyomSat CubeSat Education Kit
VERSION: 1.0
DATE: 2024
"""

# =============================================================================
# IMPORT STATEMENTS - MICROPYTHON LIBRARIES
# =============================================================================

# MicroPython Hardware Abstraction Layer
from machine import Pin, SoftI2C

# MicroPython Time Module
import utime

# OLED Display Driver
import ssd1306_handler as ssd1306

# =============================================================================
# OLED MANAGER CLASS
# =============================================================================

class OledManager:
    """
    OLED Display Manager for VyomSat CubeSat.

    This class encapsulates all OLED display functionality including
    initialization, status updates, and message display for the SSD1306
    OLED display module.

    FEATURES:
    =========
    - Automatic OLED initialization with error handling
    - System status display with telemetry data
    - Command reception display
    - Message reception display
    - Graceful degradation if OLED is not available

    DISPLAY LAYOUT:
    ==============
    The 128x64 pixel display supports 8 lines of text (8 pixels per line).
    Different display modes show different information:

    Normal Mode:
    - Line 1: VyomSat identifier
    - Line 2: Current time
    - Line 3: Battery voltage
    - Line 4: Message counter
    - Line 7: Last received data (optional)

    Command Mode:
    - Line 1: VyomSat identifier
    - Line 2: "UART RX CMD:"
    - Line 3: Command character
    - Line 4: Time
    - Line 5: "Processing..."

    Message Mode:
    - Line 1: VyomSat identifier
    - Line 2: "UART RX MSG:"
    - Lines 3-4: Message content
    - Line 5: Message length
    - Line 6: Time
    """

    def __init__(self, scl_pin=7, sda_pin=6, width=128, height=64,
                 i2c_freq=400000, i2c_addr=0x3C):
        """
        Initialize the OLED Manager.

        Args:
            scl_pin (int): GPIO pin for I2C Serial Clock Line. Default: 7
            sda_pin (int): GPIO pin for I2C Serial Data Line. Default: 6
            width (int): Display width in pixels. Default: 128
            height (int): Display height in pixels. Default: 64
            i2c_freq (int): I2C bus frequency in Hz. Default: 400000 (400kHz)
            i2c_addr (int): I2C address of the display. Default: 0x3C
        """
        # Store configuration parameters
        self.scl_pin = scl_pin
        self.sda_pin = sda_pin
        self.width = width
        self.height = height
        self.i2c_freq = i2c_freq
        self.i2c_addr = i2c_addr

        # Display object (initialized later)
        self.display = None

        # I2C bus object (initialized later)
        self.i2c_bus = None

    def initialize(self):
        """
        Initialize the SSD1306 OLED display with I2C interface.

        This method sets up the I2C communication and initializes the
        OLED display. It handles errors gracefully and returns False
        if initialization fails.

        Returns:
            bool: True if initialization successful, False otherwise
        """
        try:
            print("OledManager: Initializing OLED display...")
            print(f"  SCL Pin: GPIO {self.scl_pin}")
            print(f"  SDA Pin: GPIO {self.sda_pin}")
            print(f"  I2C Address: {hex(self.i2c_addr)}")

            # Create I2C bus
            self.i2c_bus = SoftI2C(
                scl=Pin(self.scl_pin),
                sda=Pin(self.sda_pin),
                freq=self.i2c_freq
            )

            # Scan for devices
            devices = self.i2c_bus.scan()
            print(f"  Found I2C devices: {[hex(addr) for addr in devices]}")

            # Create OLED display object
            self.display = ssd1306.SSD1306_I2C(
                self.width,
                self.height,
                self.i2c_bus,
                self.i2c_addr
            )

            print("[OK]OledManager: OLED display initialized successfully!")
            return True

        except Exception as e:
            print(f"[✗]  OledManager: OLED initialization failed: {e}")
            print("  Continuing without OLED display...")
            self.display = None
            return False

    def is_available(self):
        """
        Check if the OLED display is available.

        Returns:
            bool: True if display is available, False otherwise
        """
        return self.display is not None

    def update_display(self, battery_voltage, message_counter,
                      last_rx_type=None, last_rx_data=None):
        """
        Update the OLED display with system information.

        This method displays:
        - VyomSat identifier
        - Current time
        - Battery voltage
        - Message counter
        - Last received command/message (if any)

        Args:
            battery_voltage (float): Current battery voltage in volts
            message_counter (int): Number of messages sent
            last_rx_type (str, optional): Type of last received data
                                         ("CMD" or "MSG")
            last_rx_data (str, optional): Last received command or message
        """
        if not self.is_available():
            return  # Skip if OLED is not initialized

        try:
            # Clear the display
            self.display.fill(0)

            # Get current time
            current_time = utime.localtime()
            hour = current_time[3]
            minute = current_time[4]
            second = current_time[5]

            # Format time as HH:MM:SS
            time_str = f"{hour:02d}:{minute:02d}:{second:02d}"

            # Format battery voltage
            voltage_str = f"Batt: {battery_voltage:.2f}V"

            # Format message counter
            counter_str = f"Msg: {message_counter}"

            # Display information on OLED
            self.display.text("VyomSat EPS", 0, 0)         # Line 1
            self.display.text(f"Time: {time_str}", 0, 16)  # Line 2
            self.display.text(voltage_str, 0, 32)          # Line 3
            self.display.text(counter_str, 0, 48)          # Line 4

            # Display last received data if available (Line 7)
            if last_rx_type and last_rx_data:
                # Truncate data to fit on screen (max 16 chars per line)
                if last_rx_type == "CMD":
                    rx_display = f"RX CMD: {last_rx_data}"[:16]
                    self.display.text(rx_display, 0, 56)
                elif last_rx_type == "MSG":
                    # Display first part of message
                    rx_display = f"RX: {last_rx_data}"[:16]
                    self.display.text(rx_display, 0, 56)

            # Update the display
            self.display.show()

        except Exception as e:
            print(f"OledManager: Error updating display: {e}")

    def display_rx_command(self, command):
        """
        Display received command on OLED.

        This method temporarily updates the OLED to show the received
        command along with current system status.

        Args:
            command (str): The received command character
        """
        if not self.is_available():
            return  # Skip if OLED is not initialized

        try:
            # Clear the display
            self.display.fill(0)

            # Get current time
            current_time = utime.localtime()
            hour = current_time[3]
            minute = current_time[4]
            second = current_time[5]
            time_str = f"{hour:02d}:{minute:02d}:{second:02d}"

            # Display command reception information
            self.display.text("VyomSat EPS", 0, 0)         # Line 1
            self.display.text("UART RX CMD:", 0, 16)       # Line 2
            self.display.text(f"  '{command}'", 0, 28)     # Line 3
            self.display.text(f"Time: {time_str}", 0, 40)  # Line 4
            self.display.text("Processing...", 0, 52)      # Line 5

            # Update the display
            self.display.show()

        except Exception as e:
            print(f"OledManager: Error displaying RX command: {e}")

    def display_rx_message(self, message):
        """
        Display received message on OLED.

        This method temporarily updates the OLED to show the received
        message along with current system status.

        Args:
            message (str): The received message string
        """
        if not self.is_available():
            return  # Skip if OLED is not initialized

        try:
            # Clear the display
            self.display.fill(0)

            # Get current time
            current_time = utime.localtime()
            hour = current_time[3]
            minute = current_time[4]
            second = current_time[5]
            time_str = f"{hour:02d}:{minute:02d}:{second:02d}"

            # Truncate message to fit on screen
            # OLED can display about 16 characters per line
            msg_len = len(message)
            msg_line1 = message[:16] if msg_len > 16 else message
            msg_line2 = message[16:32] if msg_len > 16 else ""

            # Display message reception information
            self.display.text("VyomSat EPS", 0, 0)         # Line 1
            self.display.text("UART RX MSG:", 0, 12)       # Line 2
            self.display.text(msg_line1, 0, 24)            # Line 3
            if msg_line2:
                self.display.text(msg_line2, 0, 36)        # Line 4
            self.display.text(f"Len: {msg_len} chr", 0, 48)  # Line 5
            self.display.text(f"{time_str}", 0, 56)        # Line 6

            # Update the display
            self.display.show()

        except Exception as e:
            print(f"OledManager: Error displaying RX message: {e}")

    def clear_display(self):
        """
        Clear the OLED display.

        This method clears all content from the display and updates it.
        """
        if not self.is_available():
            return  # Skip if OLED is not initialized

        try:
            self.display.fill(0)
            self.display.show()
        except Exception as e:
            print(f"OledManager: Error clearing display: {e}")

    def display_text(self, text, xx, yy):
        """
        Display text at specified position on OLED.

        Args:
            text (str): Text to display
            xx (int): X coordinate (0-127)
            yy (int): Y coordinate (0-63)
        """
        if not self.is_available():
            return  # Skip if OLED is not initialized

        try:
            self.display.text(text, xx, yy)
        except Exception as e:
            print(f"OledManager: Error displaying text: {e}")

    def show(self):
        """
        Update the OLED display with buffered content.

        This method must be called after display_text() to make
        changes visible.
        """
        if not self.is_available():
            return  # Skip if OLED is not initialized

        try:
            self.display.show()
        except Exception as e:
            print(f"OledManager: Error showing display: {e}")

# =============================================================================
# MODULE-LEVEL CONVENIENCE FUNCTIONS (OPTIONAL)
# =============================================================================

# Global OLED manager instance (optional, for backward compatibility)
_global_oled_manager = None

def initialize_global_oled(scl_pin=7, sda_pin=6, width=128, height=64,
                          i2c_freq=400000, i2c_addr=0x3C):
    """
    Initialize global OLED manager instance.

    This is a convenience function for simple use cases where a single
    global OLED manager is sufficient.

    Args:
        scl_pin (int): GPIO pin for I2C SCL. Default: 7
        sda_pin (int): GPIO pin for I2C SDA. Default: 6
        width (int): Display width. Default: 128
        height (int): Display height. Default: 64
        i2c_freq (int): I2C frequency. Default: 400000
        i2c_addr (int): I2C address. Default: 0x3C

    Returns:
        OledManager: Initialized OLED manager instance
    """
    global _global_oled_manager
    _global_oled_manager = OledManager(
        scl_pin=scl_pin,
        sda_pin=sda_pin,
        width=width,
        height=height,
        i2c_freq=i2c_freq,
        i2c_addr=i2c_addr
    )
    _global_oled_manager.initialize()
    return _global_oled_manager

def get_global_oled():
    """
    Get the global OLED manager instance.

    Returns:
        OledManager or None: Global OLED manager or None if not initialized
    """
    return _global_oled_manager

