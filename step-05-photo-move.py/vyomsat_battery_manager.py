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
VyomSat Battery Voltage Monitoring Manager
==========================================

This module provides battery voltage monitoring functionality for the VyomSat
CubeSat education kit. It handles ADC initialization and voltage reading
with voltage divider correction.

FEATURES:
========
- Battery voltage monitoring via ADC
- Voltage divider circuit support
- 16-bit ADC resolution
- Error handling and graceful degradation

HARDWARE REQUIREMENTS:
=====================
- Raspberry Pi Pico 2 or Pico W microcontroller
- Battery voltage divider circuit connected to GP28 (ADC2):
  * Battery positive → R1 → GP28 (ADC2) → R2 → GND
  * Example: R1 = 10kΩ, R2 = 10kΩ (voltage divider factor = 0.5)
  * Maximum safe input: 6.6V (with 0.5 divider)

VOLTAGE DIVIDER CONCEPT:
========================
A voltage divider reduces the input voltage to a safe range for the ADC:
- RP2350 ADC can measure 0-3.3V
- Voltage divider allows measurement of higher voltages
- Formula: Vout = Vin * R2 / (R1 + R2)
- Correction: Vin = Vout / voltage_divider_factor

USAGE:
=====
    from vyomsat_battery_manager import (
        initialize_battery_sensor,
        read_battery_voltage
    )

    # Initialize the battery voltage sensor
    initialize_battery_sensor(
        adc_channel=2,
        voltage_reference=3.3,
        voltage_divider_factor=0.5
    )

    # Read battery voltage
    voltage = read_battery_voltage()
    print(f"Battery Voltage: {voltage:.2f}V")

AUTHOR: VyomSat CubeSat Education Kit
VERSION: 1.0
DATE: 2024
"""

# =============================================================================
# IMPORT STATEMENTS - MICROPYTHON LIBRARIES
# =============================================================================

# MicroPython Hardware Abstraction Layer
from machine import ADC

# =============================================================================
# MODULE-LEVEL CONFIGURATION AND STATE
# =============================================================================

# ADC Sensor Object (initialized by initialize_battery_sensor)
_battery_voltage_sensor = None

# ADC Configuration Constants
_voltage_reference = 3.3      # Volts - ADC reference voltage
_adc_max_value = 65535        # Maximum 16-bit ADC reading (2^16 - 1)
_adc_conversion_factor = 0.0  # Calculated during initialization

# Voltage Divider Configuration
_voltage_divider_factor = 0.5  # Factor by which voltage is divided

# =============================================================================
# INITIALIZATION FUNCTIONS
# =============================================================================

def initialize_battery_sensor(adc_channel=2, voltage_reference=3.3,
                              voltage_divider_factor=0.5):
    """
    Initialize the battery voltage sensor on specified ADC channel.

    This function sets up the ADC for battery voltage monitoring and
    calculates the conversion factors needed for accurate voltage readings.

    ADC CONFIGURATION:
    =================
    The RP2350 has a 12-bit ADC, but MicroPython reads it as 16-bit
    (0-65535). The reference voltage is the maximum voltage the ADC
    can measure (typically 3.3V for RP2350).

    VOLTAGE DIVIDER:
    ===============
    If using a voltage divider to measure higher voltages:
    - voltage_divider_factor = R2 / (R1 + R2)
    - Example: R1=10kΩ, R2=10kΩ → factor = 0.5
    - This allows measuring up to 6.6V safely (3.3V / 0.5)

    Args:
        adc_channel (int): ADC channel number (0-2 for GP26-GP28).
                          Default: 2 (GPIO 28)
        voltage_reference (float): ADC reference voltage in volts.
                                  Default: 3.3V
        voltage_divider_factor (float): Voltage divider ratio (0.0-1.0).
                                       Default: 0.5 (50% divider)

    Returns:
        bool: True if initialization successful, False otherwise

    Example:
        >>> initialize_battery_sensor(adc_channel=2, voltage_reference=3.3,
        ...                          voltage_divider_factor=0.5)
        True
    """
    global _battery_voltage_sensor
    global _voltage_reference
    global _adc_conversion_factor
    global _voltage_divider_factor

    try:
        print("BatteryManager: Initializing battery voltage sensor...")
        print(f"  ADC Channel: {adc_channel} (GPIO {26 + adc_channel})")
        print(f"  Voltage Reference: {voltage_reference}V")
        print(f"  Voltage Divider Factor: {voltage_divider_factor}")

        # Store configuration
        _voltage_reference = voltage_reference
        _voltage_divider_factor = voltage_divider_factor

        # Calculate ADC conversion factor
        # This converts raw ADC reading (0-65535) to voltage (0-3.3V)
        _adc_conversion_factor = _voltage_reference / _adc_max_value

        # Initialize ADC on specified channel
        _battery_voltage_sensor = ADC(adc_channel)

        # Calculate maximum measurable voltage
        max_voltage = _voltage_reference / _voltage_divider_factor

        print(f"  ADC Conversion Factor: {_adc_conversion_factor:.10f}")
        print(f"  Maximum Measurable Voltage: {max_voltage:.2f}V")
        print("[OK]BatteryManager: Battery sensor initialized successfully!")

        return True

    except Exception as e:
        print(f"[✗]  BatteryManager: Initialization failed: {e}")
        print("  Continuing without battery monitoring...")
        _battery_voltage_sensor = None
        return False

def is_available():
    """
    Check if the battery voltage sensor is available.

    Returns:
        bool: True if sensor is available, False otherwise
    """
    return _battery_voltage_sensor is not None

# =============================================================================
# VOLTAGE READING FUNCTIONS
# =============================================================================

def read_battery_voltage() -> float:
    """
    Read the battery/power supply voltage from the configured ADC channel.

    This function monitors the voltage of an external battery or power
    supply connected to the ADC pin via a voltage divider circuit.

    MEASUREMENT PROCESS:
    ===================
    1. Read raw 16-bit ADC value (0-65535) using read_u16()
    2. Convert to voltage at ADC pin using conversion factor
    3. Apply voltage divider correction to get actual battery voltage

    VOLTAGE DIVIDER CORRECTION:
    ===========================
    The voltage divider reduces the battery voltage to a safe level
    for the ADC. This function reverses that reduction to get the
    actual battery voltage:
    - adc_voltage = measured voltage at ADC pin (0-3.3V)
    - battery_voltage = adc_voltage / voltage_divider_factor

    Returns:
        float: Battery/power supply voltage in volts, or 0.0 if error

    Example:
        >>> voltage = read_battery_voltage()
        >>> print(f"Battery Voltage: {voltage:.2f}V")
        Battery Voltage: 6.90V

    Note:
        Returns 0.0 if sensor is not initialized or if an error occurs.
    """
    if not is_available():
        return 0.0  # Return safe default if sensor not initialized

    try:
        # Read 16-bit ADC value (0-65535)
        # read_u16() returns unsigned 16-bit integer representing voltage
        adc_raw_value = _battery_voltage_sensor.read_u16()

        # Convert raw ADC reading to voltage at ADC pin (0-3.3V)
        # This gives the voltage after the voltage divider
        adc_voltage = adc_raw_value * _adc_conversion_factor

        # Apply voltage divider correction to get actual battery voltage
        # If voltage divider reduces voltage by factor,
        # actual battery voltage = measured_voltage / factor
        battery_voltage = adc_voltage / _voltage_divider_factor

        return battery_voltage

    except Exception as error:
        # Handle errors gracefully and continue operation
        print(f"BatteryManager: Error reading voltage: {error}")
        return 0.0  # Return safe default value

def read_raw_adc_value() -> int:
    """
    Read the raw ADC value without conversion.

    This function returns the raw 16-bit ADC reading (0-65535)
    without any voltage conversion. Useful for debugging or
    custom processing.

    Returns:
        int: Raw ADC value (0-65535), or 0 if error

    Example:
        >>> raw = read_raw_adc_value()
        >>> print(f"Raw ADC: {raw}")
        Raw ADC: 32768
    """
    if not is_available():
        return 0

    try:
        return _battery_voltage_sensor.read_u16()
    except Exception as error:
        print(f"BatteryManager: Error reading raw ADC: {error}")
        return 0

def read_adc_voltage() -> float:
    """
    Read the voltage at the ADC pin (before voltage divider correction).

    This function returns the actual voltage measured at the ADC input
    pin, without applying the voltage divider correction factor.

    Returns:
        float: Voltage at ADC pin in volts (0-3.3V), or 0.0 if error

    Example:
        >>> adc_v = read_adc_voltage()
        >>> print(f"ADC Pin Voltage: {adc_v:.2f}V")
        ADC Pin Voltage: 3.45V
    """
    if not is_available():
        return 0.0

    try:
        adc_raw_value = _battery_voltage_sensor.read_u16()
        adc_voltage = adc_raw_value * _adc_conversion_factor
        return adc_voltage
    except Exception as error:
        print(f"BatteryManager: Error reading ADC voltage: {error}")
        return 0.0

# =============================================================================
# CONFIGURATION FUNCTIONS
# =============================================================================

def get_configuration():
    """
    Get current battery manager configuration.

    Returns:
        dict: Configuration dictionary with keys:
            - 'voltage_reference': ADC reference voltage
            - 'voltage_divider_factor': Voltage divider factor
            - 'adc_conversion_factor': ADC to voltage conversion factor
            - 'max_measurable_voltage': Maximum voltage that can be measured
            - 'is_initialized': Whether sensor is initialized
    """
    return {
        'voltage_reference': _voltage_reference,
        'voltage_divider_factor': _voltage_divider_factor,
        'adc_conversion_factor': _adc_conversion_factor,
        'max_measurable_voltage': _voltage_reference / _voltage_divider_factor,
        'is_initialized': is_available()
    }

def print_configuration():
    """
    Print current battery manager configuration.

    This is a convenience function for debugging and verification.
    """
    config = get_configuration()
    print("=" * 60)
    print("Battery Manager Configuration")
    print("=" * 60)
    print(f"Voltage Reference:     {config['voltage_reference']:.2f}V")
    print(f"Voltage Divider Factor: {config['voltage_divider_factor']:.2f}")
    print(f"ADC Conversion Factor:  {config['adc_conversion_factor']:.10f}")
    print(f"Max Measurable Voltage: {config['max_measurable_voltage']:.2f}V")
    print(f"Sensor Initialized:     {config['is_initialized']}")
    print("=" * 60)

# =============================================================================
# DISPLAY FUNCTIONS - OLED INTEGRATION
# =============================================================================

def read_and_display_battery_voltage(uart_handler=None, oled_manager=None):
    """
    Read battery voltage and display it on OLED and send via UART.

    This function is called when command 'v' is received. It reads the
    battery voltage, displays it on the OLED screen, sends it via UART,
    and prints it to the serial console.

    DISPLAY DURATION:
    ================
    The voltage information is displayed on the OLED for 2 seconds before
    returning control to the main application loop.

    Args:
        uart_handler: UART handler instance for sending data (optional)
        oled_manager: OLED manager instance for display (optional)

    Returns:
        float: The battery voltage reading in volts

    Example:
        >>> voltage = read_and_display_battery_voltage(uart, oled)
        >>> print(f"Voltage: {voltage:.2f}V")
        Voltage: 6.90V
    """
    import utime
    
    # Read battery voltage
    voltage = read_battery_voltage()
    adc_voltage = read_adc_voltage()
    raw_adc = read_raw_adc_value()
    
    # Get configuration for display
    config = get_configuration()
    
    # Print to serial console
    print("=" * 60)
    print("Battery Voltage Reading")
    print("=" * 60)
    print(f"Battery Voltage:  {voltage:.3f}V")
    print(f"ADC Voltage:      {adc_voltage:.3f}V")
    print(f"Raw ADC Value:    {raw_adc}")
    print(f"Max Measurable:   {config['max_measurable_voltage']:.2f}V")
    print("=" * 60)
    
    # Send detailed voltage data via UART
    if uart_handler:
        try:
            # Get current timestamp
            current_timestamp = utime.ticks_ms()
            
            # Format voltage telemetry message
            voltage_message = (
                f"[{current_timestamp}ms] BATTERY VOLTAGE | "
                f"Batt: {voltage:.3f}V | "
                f"ADC: {adc_voltage:.3f}V | "
                f"Raw: {raw_adc}\r\n"
            )
            
            # Send via UART
            uart_handler.write(voltage_message.encode())
            print(f"UART: Voltage data sent ({len(voltage_message)} bytes)")
            
        except Exception as error:
            print(f"BatteryManager: Error sending UART data: {error}")
    
    # Display on OLED
    if oled_manager and oled_manager.is_available():
        try:
            # Get current time for display
            current_time = utime.localtime()
            hour = current_time[3]
            minute = current_time[4]
            second = current_time[5]
            time_str = f"{hour:02d}:{minute:02d}:{second:02d}"
            
            # Clear display
            oled_manager.display.fill(0)
            
            # Display voltage information on OLED
            # Line 1: Header
            oled_manager.display.text("BATTERY VOLTAGE", 0, 0)
            
            # Line 2: Separator
            oled_manager.display.text("---------------", 0, 10)
            
            # Line 3: Battery voltage (main reading)
            oled_manager.display.text(f"Batt: {voltage:.3f}V", 0, 22)
            
            # Line 4: ADC voltage
            oled_manager.display.text(f"ADC:  {adc_voltage:.3f}V", 0, 34)
            
            # Line 5: Raw ADC value
            oled_manager.display.text(f"Raw:  {raw_adc}", 0, 46)
            
            # Line 6: Time
            oled_manager.display.text(f"Time: {time_str}", 0, 56)
            
            # Update display
            oled_manager.display.show()
            
            print("OLED: Battery voltage displayed")
            
            # Keep display visible for 2 seconds
            utime.sleep(2)
            
        except Exception as error:
            print(f"BatteryManager: Error updating OLED: {error}")
    
    return voltage

# =============================================================================
# EXAMPLE USAGE (FOR TESTING)
# =============================================================================

if __name__ == "__main__":
    """
    Example usage and testing code.

    This code runs when the module is executed directly (not imported).
    It demonstrates basic usage of the battery manager functions.
    """
    import utime

    print("VyomSat Battery Manager - Test Mode")
    print("=" * 60)

    # Initialize battery sensor
    if initialize_battery_sensor(
        adc_channel=2,
        voltage_reference=3.3,
        voltage_divider_factor=0.5
    ):
        # Print configuration
        print_configuration()

        # Read voltage continuously
        print("\nReading battery voltage (press Ctrl+C to stop)...")
        print("-" * 60)

        try:
            while True:
                # Read various measurements
                voltage = read_battery_voltage()
                adc_voltage = read_adc_voltage()
                raw_adc = read_raw_adc_value()

                # Display readings
                print(f"Battery Voltage: {voltage:.3f}V | "
                      f"ADC Voltage: {adc_voltage:.3f}V | "
                      f"Raw ADC: {raw_adc}")

                # Wait before next reading
                utime.sleep(1)

        except KeyboardInterrupt:
            print("\n" + "-" * 60)
            print("Test stopped by user")

    else:
        print("Failed to initialize battery sensor")

