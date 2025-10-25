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
VyomSat SD Card Manager Module
===============================

VyomSat - Essence of Space. Built by You.
The CubeSat education kit - Hands-on CubeSat engineering, end-to-end

OVERVIEW:
========
This module provides SD card data logging functionality for the VyomSat
CubeSat education kit. It manages SD card initialization, filesystem
mounting, and comprehensive data logging of sensor telemetry.

FEATURES:
========
- SD card initialization via SPI interface
- FAT filesystem mounting and management
- Multi-cycle data logging (9-axis IMU, GPS, housekeeping)
- Timestamped log files for data organization
- Error handling and status reporting
- Integration with VyomSat sensor managers

HARDWARE REQUIREMENTS:
=====================
- SD Card Module connected via SPI1:
  * CS (Chip Select) → GPIO 13 (GP13)
  * SCK (Serial Clock) → GPIO 10 (GP10) - SPI1 SCK
  * MOSI (Master Out) → GPIO 11 (GP11) - SPI1 TX
  * MISO (Master In) → GPIO 12 (GP12) - SPI1 RX
  * VCC → 3.3V
  * GND → Ground

SD CARD COMPATIBILITY:
=====================
- SDSC (Standard Capacity): Up to 2GB, FAT16
- SDHC (High Capacity): 2GB to 32GB, FAT32
- Recommended: FAT32 formatted cards

USAGE:
=====
1. Call initialize_sd_card() during system startup
2. Call log_sensor_data_to_sd() to save 5 cycles of sensor data
3. Use is_sd_card_available() to check SD card status

AUTHOR: VyomSat CubeSat Education Kit
VERSION: 1.0 - SD Card Integration
DATE: 2024
"""

# =============================================================================
# IMPORT STATEMENTS
# =============================================================================

# MicroPython Hardware Abstraction Layer
import machine
from machine import Pin, SPI

# MicroPython Time Module
import utime
import time

# MicroPython OS for filesystem operations
import uos

# SD Card Handler (assumed to be in same folder)
import sdcard_handler

# VyomSat Manager Modules (for sensor data access)
from vyomsat_battery_manager import read_battery_voltage
from vyomsat_9axis_manager import is_9axis_available
from vyomsat_gps_manager import is_gps_available, get_basic_gps_data

# =============================================================================
# GLOBAL VARIABLES
# =============================================================================

# SPI interface instance
_spi_interface = None

# Chip select pin instance
_chip_select_pin = None

# SD card driver instance
_sd_card_driver = None

# Virtual filesystem instance
_virtual_filesystem = None

# SD card initialization status
_sd_initialized = False

# Mount point for SD card
_MOUNT_POINT = "/sd"

# =============================================================================
# SD CARD INITIALIZATION FUNCTIONS
# =============================================================================

def initialize_sd_card(cs_pin=13, sck_pin=10, mosi_pin=11, miso_pin=12,
                      spi_id=1, baudrate=1000000):
    """
    Initialize SD card interface and mount filesystem.

    This function configures SPI1 interface for SD card communication,
    initializes the SD card driver, and mounts the FAT filesystem.

    SPI CONFIGURATION:
    =================
    - SPI Controller: SPI1 (independent from other SPI devices)
    - Baudrate: 1MHz (safe speed for reliable initialization)
    - Polarity: 0 (clock idle state is LOW)
    - Phase: 0 (data sampled on first/rising clock edge)

    GPIO PIN ASSIGNMENTS:
    ====================
    - GP13: CS (Chip Select) - any available GPIO
    - GP10: SCK (Serial Clock) - must be SPI1 SCK pin
    - GP11: MOSI (Master Out) - must be SPI1 TX pin
    - GP12: MISO (Master In) - must be SPI1 RX pin

    Args:
        cs_pin (int): GPIO pin for chip select (default: 13)
        sck_pin (int): GPIO pin for SPI clock (default: 10)
        mosi_pin (int): GPIO pin for SPI MOSI (default: 11)
        miso_pin (int): GPIO pin for SPI MISO (default: 12)
        spi_id (int): SPI controller ID (default: 1 for SPI1)
        baudrate (int): SPI baudrate in Hz (default: 1000000)

    Returns:
        bool: True if initialization successful, False otherwise
    """
    global _spi_interface, _chip_select_pin, _sd_card_driver
    global _virtual_filesystem, _sd_initialized

    try:
        print(f"SD Manager: Initializing SD card on SPI{spi_id}...")
        print(f"  CS Pin: GPIO {cs_pin}")
        print(f"  SCK Pin: GPIO {sck_pin}")
        print(f"  MOSI Pin: GPIO {mosi_pin}")
        print(f"  MISO Pin: GPIO {miso_pin}")
        print(f"  Baudrate: {baudrate} Hz")

        # Configure chip select pin (active-low, start HIGH/deselected)
        _chip_select_pin = Pin(cs_pin, Pin.OUT)

        # Initialize SPI controller
        _spi_interface = SPI(
            spi_id,
            baudrate=baudrate,
            polarity=0,
            phase=0,
            bits=8,
            firstbit=SPI.MSB,
            sck=Pin(sck_pin),
            mosi=Pin(mosi_pin),
            miso=Pin(miso_pin)
        )

        # Initialize SD card driver
        _sd_card_driver = sdcard_handler.SDCard(
            _spi_interface,
            _chip_select_pin
        )
        print("[OK] SD Manager: SD card driver initialized")

        # Mount filesystem
        _virtual_filesystem = uos.VfsFat(_sd_card_driver)
        uos.mount(_virtual_filesystem, _MOUNT_POINT)
        print(f"[OK] SD Manager: Filesystem mounted at {_MOUNT_POINT}")

        _sd_initialized = True
        return True

    except Exception as e:
        print(f"[ERROR] SD Manager: Initialization failed: {e}")
        print("  Check SD card insertion and wiring")
        _sd_initialized = False
        return False


def is_sd_card_available():
    """
    Check if SD card is initialized and available.

    Returns:
        bool: True if SD card is available, False otherwise
    """
    return _sd_initialized


def log_and_display_sensor_data(uart_handler, oled_manager,
                                 num_cycles=5, delay_between_cycles_ms=500):
    """
    Log sensor data to SD card with OLED display feedback.

    This function provides a complete user experience for SD card logging,
    including OLED status displays and UART response messages. It handles
    all success/failure scenarios with appropriate user feedback.

    Args:
        uart_handler: UART handler instance for sending responses
        oled_manager: OLED manager instance for display updates
        num_cycles (int): Number of data collection cycles (default: 5)
        delay_between_cycles_ms (int): Delay between cycles in ms (default: 500)

    Returns:
        None
    """
    import utime
    from vyomsat_9axis_manager import _nine_axis_imu_handler
    from vyomsat_gps_manager import _gps_handler

    # Check if SD card is available
    if not is_sd_card_available():
        # SD card not available
        usb_response = "SD logging FAILED: SD card not initialized"
        uart_response = "NAK: SD card not available"
        print(f"[ERROR] {usb_response}")

        # Display error on OLED
        if oled_manager and oled_manager.is_available():
            oled_manager.clear_display()
            oled_manager.display_text("SD Card Error!", 0, 0)
            oled_manager.display_text("Not initialized", 0, 16)
            oled_manager.show()
            utime.sleep(2)  # Display for 2 seconds

        # Send UART response
        try:
            uart_response_formatted = f"{uart_response}\r\n"
            uart_handler.write(uart_response_formatted.encode())
        except Exception as e:
            print(f"Failed to send UART response: {e}")

        return

    # Display logging status on OLED
    if oled_manager and oled_manager.is_available():
        oled_manager.clear_display()
        oled_manager.display_text("SD Card", 0, 0)
        oled_manager.display_text("Logging...", 0, 16)
        oled_manager.display_text(f"{num_cycles} cycles", 0, 32)
        oled_manager.show()

    # Log sensor data to SD card
    success, message, filename = log_sensor_data_to_sd(
        nine_axis_handler=_nine_axis_imu_handler,
        gps_handler=_gps_handler,
        num_cycles=num_cycles,
        delay_between_cycles_ms=delay_between_cycles_ms
    )

    if success:
        # Update responses with success message
        usb_response = f"SD logging SUCCESS: {message}"
        uart_response = f"ACK: {message}"

        # Display success on USB
        print(f"[OK] {usb_response}")

        # Display success on OLED
        if oled_manager and oled_manager.is_available():
            oled_manager.clear_display()
            oled_manager.display_text("SD Logging OK!", 0, 0)
            oled_manager.display_text("Data saved:", 0, 16)
            # Display filename (truncate if needed)
            oled_manager.display_text(filename[:16], 0, 32)
            if len(filename) > 16:
                oled_manager.display_text(filename[16:32], 0, 48)
            oled_manager.show()
            utime.sleep(3)  # Display for 3 seconds
    else:
        # Update responses with failure message
        usb_response = f"SD logging FAILED: {message}"
        uart_response = f"NAK: {message}"

        # Display failure on USB
        print(f"[ERROR] {usb_response}")

        # Display failure on OLED
        if oled_manager and oled_manager.is_available():
            oled_manager.clear_display()
            oled_manager.display_text("SD Error!", 0, 0)
            oled_manager.display_text("Logging failed", 0, 16)
            oled_manager.show()
            utime.sleep(2)  # Display for 2 seconds

    # Send UART response
    try:
        uart_response_formatted = f"{uart_response}\r\n"
        uart_handler.write(uart_response_formatted.encode())
    except Exception as e:
        print(f"Failed to send UART response: {e}")


# =============================================================================
# DATA FORMATTING FUNCTIONS
# =============================================================================

def format_housekeeping_data(cycle_num, battery_voltage, timestamp_ms):
    """
    Format housekeeping data for logging.

    Args:
        cycle_num (int): Data collection cycle number
        battery_voltage (float): Battery voltage in volts
        timestamp_ms (int): Timestamp in milliseconds

    Returns:
        str: Formatted housekeeping data string
    """
    seconds = timestamp_ms // 1000
    hours = (seconds // 3600) % 24
    minutes = (seconds // 60) % 60
    secs = seconds % 60

    output = ""
    output += f"=== CYCLE {cycle_num} - HOUSEKEEPING DATA ===\n"
    output += f"Timestamp: {timestamp_ms} ms\n"
    output += f"Time: {hours:02d}:{minutes:02d}:{secs:02d}\n"
    output += f"Battery Voltage: {battery_voltage:.2f} V\n"
    output += "\n"

    return output


def format_9axis_data_for_logging(cycle_num, nine_axis_handler):
    """
    Format 9-axis IMU data for SD card logging.

    This function reads comprehensive 9-axis sensor data and formats it
    for file storage.

    Args:
        cycle_num (int): Data collection cycle number
        nine_axis_handler: MPU9250 handler instance

    Returns:
        str: Formatted 9-axis data string
    """
    try:
        # Get comprehensive sensor data
        data = nine_axis_handler.get_comprehensive_sensor_data()

        # Extract components
        timestamp = data['timestamp_ms']
        temperature = data['temperature_celsius']
        accel = data['accelerometer_data']
        gyro = data['gyroscope_data']
        mag = data['magnetometer_data']
        filtered = data['filtered_angles']
        heading = data['heading_degrees']

        # Calculate magnitudes
        accel_mag = (accel['x_axis']**2 + accel['y_axis']**2 +
                     accel['z_axis']**2)**0.5
        gyro_mag = (gyro['x_axis']**2 + gyro['y_axis']**2 +
                    gyro['z_axis']**2)**0.5
        mag_mag = (mag['x_axis']**2 + mag['y_axis']**2 +
                   mag['z_axis']**2)**0.5

        # Format output
        output = ""
        output += f"=== CYCLE {cycle_num} - MPU9250 9-AXIS IMU DATA ===\n"
        output += f"Timestamp: {timestamp} ms\n"
        output += f"Temperature: {temperature:.2f} C\n"
        output += "\n"

        output += "ACCELEROMETER (m/s^2):\n"
        output += (f"  X: {accel['x_axis']:7.3f}  "
                  f"Y: {accel['y_axis']:7.3f}  "
                  f"Z: {accel['z_axis']:7.3f}\n")
        output += f"  Magnitude: {accel_mag:.3f} m/s^2\n"
        output += "\n"

        output += "GYROSCOPE (deg/s):\n"
        output += (f"  X: {gyro['x_axis']:7.2f}  "
                  f"Y: {gyro['y_axis']:7.2f}  "
                  f"Z: {gyro['z_axis']:7.2f}\n")
        output += f"  Magnitude: {gyro_mag:.2f} deg/s\n"
        output += "\n"

        output += "MAGNETOMETER (uT):\n"
        output += (f"  X: {mag['x_axis']:7.2f}  "
                  f"Y: {mag['y_axis']:7.2f}  "
                  f"Z: {mag['z_axis']:7.2f}\n")
        output += f"  Magnitude: {mag_mag:.2f} uT\n"
        output += "\n"

        output += "ATTITUDE (FILTERED):\n"
        output += f"  Pitch: {filtered['pitch_angle']:7.2f} deg\n"
        output += f"  Roll:  {filtered['roll_angle']:7.2f} deg\n"
        output += f"  Heading: {heading:7.1f} deg (Magnetic North)\n"
        output += "\n"

        return output

    except Exception as e:
        return f"=== CYCLE {cycle_num} - 9-AXIS ERROR ===\n{e}\n\n"


def format_gps_data_for_logging(cycle_num, gps_handler):
    """
    Format GPS data for SD card logging.

    Args:
        cycle_num (int): Data collection cycle number
        gps_handler: NEO6MV2 handler instance

    Returns:
        str: Formatted GPS data string
    """
    try:
        if not gps_handler.has_fix:
            output = f"=== CYCLE {cycle_num} - GPS DATA ===\n"
            output += "Status: Acquiring fix...\n"
            output += f"Satellites visible: {gps_handler.satellites_in_view}\n"
            output += "\n"
            return output

        # Import handler for coordinate conversion
        from neo6mv2_handler import NEO6MV2Handler

        # Convert coordinates to decimal degrees
        lat_dd = NEO6MV2Handler.convert_coord_to_decimal_degrees_string(
            gps_handler.latitude
        )
        lon_dd = NEO6MV2Handler.convert_coord_to_decimal_degrees_string(
            gps_handler.longitude
        )

        h, m, s = gps_handler.time_hms_local

        output = ""
        output += f"=== CYCLE {cycle_num} - GPS DATA ===\n"
        output += "\n"

        output += "POSITION:\n"
        output += f"  Latitude: {lat_dd}\n"
        output += f"  Longitude: {lon_dd}\n"
        output += f"  Altitude: {gps_handler.altitude_m} m\n"
        output += f"  Geoid Height: {gps_handler.geoid_separation_m} m\n"
        output += "\n"

        output += "TIME & DATE:\n"
        output += f"  GPS Time (local): {h:02d}:{m:02d}:{s:05.2f}\n"
        output += f"  Date: {gps_handler.date_string('s_dmy')}\n"
        output += f"  Timezone Offset: {gps_handler.timezone_offset_hours} h\n"
        output += "\n"

        output += "MOTION:\n"
        output += (f"  Course/Heading: "
                  f"{gps_handler.course_over_ground_deg} deg\n")
        output += (f"  Compass Direction: "
                  f"{gps_handler.course_compass_direction()}\n")
        output += f"  Speed (km/h): {gps_handler.speed_string('kph')}\n"
        output += f"  Speed (knots): {gps_handler.speed_string('knot')}\n"
        output += "\n"

        output += "SATELLITES:\n"
        output += f"  In View: {gps_handler.satellites_in_view}\n"
        output += f"  In Use: {gps_handler.satellites_used_for_fix}\n"
        output += f"  Used IDs: {gps_handler.satellite_ids_used}\n"
        output += "\n"

        output += "FIX QUALITY:\n"
        output += f"  Fix Status (GGA): {gps_handler.fix_quality_gga}\n"
        output += f"  Fix Type (GSA): {gps_handler.fix_type_gsa}\n"
        output += f"  HDOP: {gps_handler.hdop}\n"
        output += f"  VDOP: {gps_handler.vdop}\n"
        output += f"  PDOP: {gps_handler.pdop}\n"
        output += "\n"

        return output

    except Exception as e:
        return f"=== CYCLE {cycle_num} - GPS ERROR ===\n{e}\n\n"


# =============================================================================
# FILE MANAGEMENT FUNCTIONS
# =============================================================================

def move_file_to_sd(source_file_path, uart_handler, oled_manager,
                    destination_filename=None):
    """
    Move a file to the SD card with OLED display and UART feedback.

    This function moves a file from its current location to the SD card,
    providing user feedback through OLED display and UART messages.
    The original file is deleted after successful copy to SD card.

    Args:
        source_file_path (str): Full path to the source file to move
        uart_handler: UART handler instance for sending responses
        oled_manager: OLED manager instance for display updates
        destination_filename (str): Optional destination filename on SD card.
                                   If None, uses the original filename.

    Returns:
        None
    """
    import utime

    # Check if SD card is available
    if not _sd_initialized:
        # SD card not available
        usb_response = "File move FAILED: SD card not initialized"
        uart_response = "NAK: SD card not available"
        print(f"[ERROR] {usb_response}")

        # Display error on OLED
        if oled_manager and oled_manager.is_available():
            oled_manager.clear_display()
            oled_manager.display_text("SD Card Error!", 0, 0)
            oled_manager.display_text("Not initialized", 0, 16)
            oled_manager.show()
            utime.sleep(2)  # Display for 2 seconds

        # Send UART response
        try:
            uart_response_formatted = f"{uart_response}\r\n"
            uart_handler.write(uart_response_formatted.encode())
        except Exception as e:
            print(f"Failed to send UART response: {e}")

        return

    # Display moving status on OLED
    if oled_manager and oled_manager.is_available():
        oled_manager.clear_display()
        oled_manager.display_text("Moving to", 0, 0)
        oled_manager.display_text("SD card...", 0, 16)
        oled_manager.show()

    try:
        # Extract filename from source path if destination not provided
        if destination_filename is None:
            # Get filename from full path (split by '/')
            destination_filename = source_file_path.split('/')[-1]

        # Build full destination path on SD card
        destination_path = f"{_MOUNT_POINT}/{destination_filename}"

        print(f"SD Manager: Moving file to SD card...")
        print(f"  Source: {source_file_path}")
        print(f"  Destination: {destination_path}")

        # Open source file for reading in binary mode
        with open(source_file_path, 'rb') as source_file:
            # Read entire file content
            file_content = source_file.read()

        # Open destination file for writing in binary mode
        with open(destination_path, 'wb') as dest_file:
            # Write content to SD card
            dest_file.write(file_content)

        print(f"[OK] SD Manager: File copied to SD card")

        # Delete the original file after successful copy
        try:
            uos.remove(source_file_path)
            print(f"[OK] SD Manager: Original file deleted")
        except Exception as delete_error:
            print(f"[WARNING] Failed to delete original file: {delete_error}")
            # Continue anyway, file was successfully copied

        success_msg = f"File moved to SD card: {destination_filename}"
        print(f"[OK] SD Manager: {success_msg}")

        # Display success on OLED
        if oled_manager and oled_manager.is_available():
            oled_manager.clear_display()
            oled_manager.display_text("Moved to SD!", 0, 0)
            # Extract just the filename for display
            oled_manager.display_text(destination_filename[:16], 0, 16)
            if len(destination_filename) > 16:
                oled_manager.display_text(destination_filename[16:32], 0, 32)
            oled_manager.show()
            utime.sleep(2)  # Display for 2 seconds

        # Send UART success response
        try:
            uart_msg = f"ACK: Image saved to SD card\r\n"
            uart_handler.write(uart_msg.encode())
        except Exception as e:
            print(f"Failed to send UART response: {e}")

    except Exception as e:
        error_msg = f"File move failed: {e}"
        print(f"[ERROR] SD Manager: {error_msg}")

        # Display failure on OLED
        if oled_manager and oled_manager.is_available():
            oled_manager.clear_display()
            oled_manager.display_text("SD Move Error!", 0, 0)
            oled_manager.display_text("Failed", 0, 16)
            oled_manager.show()
            utime.sleep(2)  # Display for 2 seconds

        # Send UART failure response
        try:
            uart_msg = f"NAK: {error_msg}\r\n"
            uart_handler.write(uart_msg.encode())
        except Exception as e:
            print(f"Failed to send UART response: {e}")


# =============================================================================
# MAIN DATA LOGGING FUNCTION
# =============================================================================

def log_sensor_data_to_sd(nine_axis_handler=None, gps_handler=None,
                          num_cycles=5, delay_between_cycles_ms=500):
    """
    Log comprehensive sensor data to SD card for specified number of cycles.

    This function collects data from all available sensors (9-axis IMU, GPS,
    housekeeping) over multiple cycles and saves it to a timestamped log file
    on the SD card.

    DATA COLLECTION:
    ===============
    For each cycle:
    - Housekeeping data (battery voltage, timestamp)
    - 9-axis IMU data (accelerometer, gyroscope, magnetometer, attitude)
    - GPS data (position, time, motion, satellites, fix quality)

    Args:
        nine_axis_handler: MPU9250 handler instance (if available)
        gps_handler: NEO6MV2 handler instance (if available)
        num_cycles (int): Number of data collection cycles (default: 5)
        delay_between_cycles_ms (int): Delay between cycles in ms (default: 500)

    Returns:
        tuple: (success: bool, message: str, filename: str)
            success: True if logging successful, False otherwise
            message: Status message describing the result
            filename: Name of the log file created (or empty string on error)
    """
    if not _sd_initialized:
        return False, "SD card not initialized", ""

    try:
        # Generate timestamped filename
        current_time = time.time()
        log_filename = f"{_MOUNT_POINT}/sensor_log_{int(current_time)}.txt"

        print(f"SD Manager: Starting data logging to {log_filename}")
        print(f"SD Manager: Collecting {num_cycles} cycles of sensor data...")

        # Open file for writing
        with open(log_filename, "w") as log_file:
            # Write file header
            log_file.write("=" * 70 + "\n")
            log_file.write("VyomSat Sensor Data Log\n")
            log_file.write("=" * 70 + "\n")
            log_file.write(f"Log created: {time.time()} (Unix timestamp)\n")
            log_file.write(f"Number of cycles: {num_cycles}\n")
            log_file.write("=" * 70 + "\n\n")

            # Collect data for each cycle
            for cycle in range(1, num_cycles + 1):
                print(f"SD Manager: Logging cycle {cycle}/{num_cycles}...")

                # Get current timestamp
                timestamp_ms = utime.ticks_ms()

                # === LOG HOUSEKEEPING DATA ===
                battery_voltage = read_battery_voltage()
                hk_data = format_housekeeping_data(
                    cycle,
                    battery_voltage,
                    timestamp_ms
                )
                log_file.write(hk_data)

                # === LOG 9-AXIS IMU DATA ===
                if nine_axis_handler is not None:
                    axis_data = format_9axis_data_for_logging(
                        cycle,
                        nine_axis_handler
                    )
                    log_file.write(axis_data)
                else:
                    log_file.write(
                        f"=== CYCLE {cycle} - 9-AXIS DATA ===\n"
                        f"Status: Not available\n\n"
                    )

                # === LOG GPS DATA ===
                if gps_handler is not None:
                    gps_data = format_gps_data_for_logging(cycle, gps_handler)
                    log_file.write(gps_data)
                else:
                    log_file.write(
                        f"=== CYCLE {cycle} - GPS DATA ===\n"
                        f"Status: Not available\n\n"
                    )

                # Write cycle separator
                log_file.write("-" * 70 + "\n\n")

                # Delay between cycles (except after last cycle)
                if cycle < num_cycles:
                    utime.sleep_ms(delay_between_cycles_ms)

            # Write file footer
            log_file.write("=" * 70 + "\n")
            log_file.write("End of Log\n")
            log_file.write("=" * 70 + "\n")

        # Extract just the filename (without path) for display
        display_filename = log_filename.split('/')[-1]

        success_msg = (f"Data logged: {num_cycles} cycles saved to "
                      f"{display_filename}")
        print(f"[OK] SD Manager: {success_msg}")

        return True, success_msg, display_filename

    except Exception as e:
        error_msg = f"SD card write error: {e}"
        print(f"[ERROR] SD Manager: {error_msg}")
        return False, error_msg, ""


# =============================================================================
# MODULE TEST (FOR STANDALONE TESTING)
# =============================================================================

if __name__ == "__main__":
    print("VyomSat SD Card Manager Module")
    print("This module should be imported, not run directly")
    print("")
    print("Testing SD card initialization...")

    if initialize_sd_card():
        print("[OK] SD card initialized successfully!")
        print("SD card is ready for data logging")
    else:
        print("[ERROR] SD card initialization failed")
        print("Check SD card insertion and wiring")

