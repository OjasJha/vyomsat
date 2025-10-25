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
VyomSat XBee Module Manager
===========================

This module provides XBee hardware reset functionality for the VyomSat
CubeSat education kit.

FEATURES:
========
- Hardware reset via RST pin
- Solves race condition on boot
- Configurable reset timing

HARDWARE REQUIREMENTS:
=====================
- XBee module connected to Raspberry Pi Pico
- XBee RST pin connected to Pico GPIO (typically GP2)
- Common ground between Pico and XBee

USAGE:
=====
    from vyomsat_xbee_manager import perform_xbee_hardware_reset

    # Perform hardware reset with default pin (GP2) and delay (5 seconds)
    perform_xbee_hardware_reset(reset_pin=2, init_delay_seconds=5)

AUTHOR: VyomSat CubeSat Education Kit
VERSION: 1.0
DATE: 2024
"""

# =============================================================================
# IMPORT STATEMENTS - MICROPYTHON LIBRARIES
# =============================================================================

# MicroPython Hardware Abstraction Layer
from machine import Pin

# MicroPython Time Module
import utime

# =============================================================================
# XBEE RESET FUNCTIONS
# =============================================================================

def perform_xbee_hardware_reset(reset_pin=2, init_delay_seconds=5):
    """
    Perform hardware reset on XBee module via RST pin.

    This solves the race condition where the Pico boots faster than
    the XBee. By performing a hardware reset, we ensure the XBee
    starts its boot process only after the Pico is ready.

    RESET SEQUENCE:
    ==============
    The RST pin on XBee modules is active-low:
    - HIGH (1) = Normal operation (inactive reset state)
    - LOW (0) = Reset active (XBee held in reset)
    - Sequence: High -> Low -> High

    TIMING:
    =======
    1. Start HIGH: Ensure we're in inactive state
    2. Pull LOW for 100ms: Trigger the reset
    3. Release HIGH: XBee begins its boot sequence
    4. Wait for init_delay_seconds: Allow XBee to complete initialization

    Args:
        reset_pin (int): GPIO pin number connected to XBee RST.
                        Default: 2 (GP2)
        init_delay_seconds (int): Time to wait after reset for XBee
                                 initialization. Default: 5 seconds

    Example:
        >>> perform_xbee_hardware_reset(reset_pin=2, init_delay_seconds=5)
        Performing hardware reset on XBee via GP2...
        Reset complete. Waiting for XBee to initialize (5 second delay)...
        Initialization delay complete. XBee ready for communication.
    """
    print(f"Performing hardware reset on XBee via GP{reset_pin}...")
    xbee_reset_pin = Pin(reset_pin, Pin.OUT)

    # Step 1: Start high (inactive state)
    xbee_reset_pin.value(1)
    utime.sleep(0.1)  # Brief delay to ensure pin is stable

    # Step 2: Pull low to trigger reset
    xbee_reset_pin.value(0)
    utime.sleep(0.1)  # Hold reset for 100ms

    # Step 3: Release reset - XBee starts booting
    xbee_reset_pin.value(1)

    # Wait for XBee to finish boot process
    print(f"Reset complete. Waiting for XBee to initialize "
          f"({init_delay_seconds} second delay)...")
    utime.sleep(init_delay_seconds)
    print("Initialization delay complete. XBee ready for communication.")

