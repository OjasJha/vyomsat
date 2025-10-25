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
VyomSat 9-Axis IMU (MPU9250) Manager
====================================

This module provides 9-axis IMU monitoring functionality for the VyomSat
CubeSat education kit using the MPU9250 sensor. It handles sensor
initialization and provides attitude data display.

FEATURES:
========
- MPU9250 9-axis IMU sensor initialization and management
- Complete attitude data reading (accelerometer, gyroscope, magnetometer)
- Tilt angles and heading calculation
- Formatted data output for USB UART and OLED display
- Error handling and graceful degradation

HARDWARE REQUIREMENTS:
=====================
- Raspberry Pi Pico 2 or Pico W microcontroller
- MPU9250 9-axis IMU sensor connected via I2C:
  * SCL (Serial Clock) → GPIO 21
  * SDA (Serial Data) → GPIO 20
  * VCC → 3.3V
  * GND → Ground
  * 4.7kΩ pull-up resistors on SCL and SDA lines

MPU9250 SENSOR:
==============
- 3-axis accelerometer: Measures linear acceleration
- 3-axis gyroscope: Measures angular velocity
- 3-axis magnetometer: Measures magnetic field (compass)
- Temperature sensor: Provides thermal compensation
- Combined: Complete 9-axis motion sensing capability

USAGE:
=====
    from vyomsat_9axis_manager import (
        initialize_9axis_sensor,
        read_and_display_9axis_data,
        is_9axis_available
    )

    # Initialize the 9-axis IMU sensor
    if initialize_9axis_sensor():
        # Read and display 9-axis data
        read_and_display_9axis_data(uart_handler, oled_manager)

AUTHOR: VyomSat CubeSat Education Kit
VERSION: 1.0
DATE: 2024
"""

# =============================================================================
# IMPORT STATEMENTS - MICROPYTHON LIBRARIES
# =============================================================================

# MicroPython Time Module
import utime

# MPU9250 9-Axis IMU Handler
from mpu9250_handler import MPU9250NineAxisInertialMeasurementUnitHandler

# =============================================================================
# MODULE-LEVEL CONFIGURATION AND STATE
# =============================================================================

# 9-Axis IMU Sensor Object (initialized by initialize_9axis_sensor)
_nine_axis_imu_handler = None

# I2C Configuration Constants for MPU9250
_I2C_SCL_PIN = 21            # GPIO 21 for SCL
_I2C_SDA_PIN = 20            # GPIO 20 for SDA
_I2C_FREQUENCY = 400000      # 400 kHz I2C bus frequency

# =============================================================================
# INITIALIZATION FUNCTIONS
# =============================================================================

def initialize_9axis_sensor(scl_pin=21, sda_pin=20, i2c_frequency=400000):
    """
    Initialize the MPU9250 9-axis IMU sensor.

    This function sets up the MPU9250 9-axis IMU sensor with the specified
    I2C configuration. The sensor provides complete 9-axis motion sensing
    including accelerometer, gyroscope, and magnetometer data.

    MPU9250 ARCHITECTURE:
    ====================
    The MPU9250 contains two sensors in one package:
    - MPU6500: 6-axis IMU (accelerometer + gyroscope + temperature)
    - AK8963: 3-axis magnetometer (compass)
    Both sensors share the same I2C bus but have different addresses.

    I2C CONFIGURATION:
    =================
    - SCL (Serial Clock): GPIO pin for I2C clock signal
    - SDA (Serial Data): GPIO pin for I2C data signal
    - I2C Frequency: Communication speed (typically 400 kHz)
    - Pull-up resistors (4.7kΩ) required on SCL and SDA lines

    Args:
        scl_pin (int): GPIO pin for I2C Serial Clock Line. Default: 21
        sda_pin (int): GPIO pin for I2C Serial Data Line. Default: 20
        i2c_frequency (int): I2C bus frequency in Hz. Default: 400000

    Returns:
        bool: True if initialization successful, False otherwise

    Example:
        >>> if initialize_9axis_sensor():
        ...     print("9-axis IMU initialized successfully")
        ... else:
        ...     print("9-axis IMU initialization failed")
    """
    global _nine_axis_imu_handler

    try:
        print("9AxisManager: Initializing MPU9250 9-axis IMU sensor...")
        print(f"  SCL Pin: GPIO {scl_pin}")
        print(f"  SDA Pin: GPIO {sda_pin}")
        print(f"  I2C Frequency: {i2c_frequency} Hz")

        # Create MPU9250 handler instance
        _nine_axis_imu_handler = (
            MPU9250NineAxisInertialMeasurementUnitHandler(
                serial_clock_pin=scl_pin,
                serial_data_pin=sda_pin,
                i2c_frequency=i2c_frequency
            )
        )

        # Initialize the sensor
        if _nine_axis_imu_handler.initialize_sensor():
            print("[OK] 9AxisManager: MPU9250 sensor initialized successfully!")
            return True
        else:
            print("[ERROR] 9AxisManager: MPU9250 sensor initialization failed")
            print("  Check wiring: SCL->GPIO21, SDA->GPIO20, VCC->3.3V, GND->GND")
            print("  Ensure 4.7kOhm pull-up resistors on SCL and SDA")
            _nine_axis_imu_handler = None
            return False

    except Exception as e:
        print(f"[ERROR] 9AxisManager: Initialization error: {e}")
        print("  Continuing without 9-axis IMU...")
        _nine_axis_imu_handler = None
        return False

def is_9axis_available():
    """
    Check if the 9-axis IMU sensor is available.

    Returns:
        bool: True if sensor is available and initialized, False otherwise
    """
    return _nine_axis_imu_handler is not None

# =============================================================================
# DATA READING AND DISPLAY FUNCTIONS
# =============================================================================

def read_and_display_9axis_data(uart_handler=None, oled_manager=None):
    """
    Read 9-axis IMU data and display on USB UART and OLED.

    This function reads comprehensive data from the MPU9250 9-axis IMU
    sensor and displays it on both the USB serial console and OLED display.
    It provides complete attitude information including accelerometer,
    gyroscope, magnetometer, angles, and heading.

    9-AXIS DATA PROVIDED:
    ====================
    - Temperature: Sensor temperature in Celsius
    - Accelerometer: 3-axis linear acceleration (m/s²)
    - Gyroscope: 3-axis angular velocity (°/s)
    - Magnetometer: 3-axis magnetic field (μT)
    - Tilt Angles: Raw pitch, roll, yaw from accelerometer
    - Filtered Angles: Complementary filtered pitch and roll
    - Heading: Compass direction (0-360°)

    DISPLAY FORMAT:
    ==============
    USB UART:
    - Comprehensive data display with all sensor readings
    - Formatted for easy reading and logging
    - Includes magnitude calculations

    OLED:
    - Compact display showing key attitude data
    - Pitch, roll, and heading values
    - Temperature reading
    - Time-stamped information

    Args:
        uart_handler (UartHandler, optional): UART handler for sending data
        oled_manager (OledManager, optional): OLED manager for display

    Returns:
        dict or None: Comprehensive sensor data if successful, None if error

    Example:
        >>> data = read_and_display_9axis_data(uart_handler, oled_manager)
        >>> if data:
        ...     print(f"Heading: {data['heading_degrees']:.1f}°")
    """
    if not is_9axis_available():
        error_msg = "9-axis IMU not available - sensor not initialized"
        print(f"9AxisManager: {error_msg}")

        # Display error on OLED
        if oled_manager:
            try:
                oled_manager.display.fill(0)
                oled_manager.display.text("9-Axis IMU", 0, 0)
                oled_manager.display.text("NOT AVAILABLE", 0, 16)
                oled_manager.display.text("Check wiring", 0, 32)
                oled_manager.display.show()
            except Exception as e:
                print(f"9AxisManager: OLED display error: {e}")

        # Send error via UART
        if uart_handler:
            try:
                uart_handler.write(f"ERROR: {error_msg}\r\n".encode())
            except Exception as e:
                print(f"9AxisManager: UART send error: {e}")

        return None

    try:
        # Read comprehensive 9-axis sensor data
        data = _nine_axis_imu_handler.get_comprehensive_sensor_data()

        # Extract key data components
        timestamp = data['timestamp_ms']
        temperature = data['temperature_celsius']
        accel_data = data['accelerometer_data']
        gyro_data = data['gyroscope_data']
        mag_data = data['magnetometer_data']
        tilt_angles = data['tilt_angles']
        filtered_angles = data['filtered_angles']
        heading = data['heading_degrees']

        # Calculate magnitudes
        total_accel = (
            accel_data['x_axis']**2 +
            accel_data['y_axis']**2 +
            accel_data['z_axis']**2
        )**0.5

        total_gyro = (
            gyro_data['x_axis']**2 +
            gyro_data['y_axis']**2 +
            gyro_data['z_axis']**2
        )**0.5

        total_mag = (
            mag_data['x_axis']**2 +
            mag_data['y_axis']**2 +
            mag_data['z_axis']**2
        )**0.5

        # === DISPLAY ON USB SERIAL ===
        print("\n" + "=" * 70)
        print("MPU9250 9-AXIS IMU ATTITUDE DATA")
        print("=" * 70)
        print(f"Timestamp: {timestamp} ms")
        print(f"Temperature: {temperature:.2f} °C")
        print("")

        print("ACCELEROMETER (m/s²):")
        print(f"  X: {accel_data['x_axis']:7.3f}  "
              f"Y: {accel_data['y_axis']:7.3f}  "
              f"Z: {accel_data['z_axis']:7.3f}")
        print(f"  Total Magnitude: {total_accel:.3f} m/s²")
        print("")

        print("GYROSCOPE (°/s):")
        print(f"  X: {gyro_data['x_axis']:7.2f}  "
              f"Y: {gyro_data['y_axis']:7.2f}  "
              f"Z: {gyro_data['z_axis']:7.2f}")
        print(f"  Total Magnitude: {total_gyro:.2f} °/s")
        print("")

        print("MAGNETOMETER (μT):")
        print(f"  X: {mag_data['x_axis']:7.2f}  "
              f"Y: {mag_data['y_axis']:7.2f}  "
              f"Z: {mag_data['z_axis']:7.2f}")
        print(f"  Total Magnitude: {total_mag:.2f} μT")
        print("")

        print("ATTITUDE (FILTERED):")
        print(f"  Pitch: {filtered_angles['pitch_angle']:7.2f}°")
        print(f"  Roll:  {filtered_angles['roll_angle']:7.2f}°")
        print(f"  Heading: {heading:7.1f}° (Magnetic North)")
        print("=" * 70)

        # === SEND VIA UART (COMPREHENSIVE DATA) ===
        if uart_handler:
            try:
                # Build comprehensive UART message with all sensor data
                uart_msg = ""
                uart_msg += "=" * 70 + "\r\n"
                uart_msg += "MPU9250 9-AXIS IMU COMPREHENSIVE DATA\r\n"
                uart_msg += "=" * 70 + "\r\n"
                uart_msg += f"Timestamp: {timestamp} ms\r\n"
                uart_msg += f"Temperature: {temperature:.2f} C\r\n"
                uart_msg += "\r\n"
                
                # Accelerometer data
                uart_msg += "ACCELEROMETER (m/s^2):\r\n"
                uart_msg += (f"  X: {accel_data['x_axis']:7.3f}  "
                            f"Y: {accel_data['y_axis']:7.3f}  "
                            f"Z: {accel_data['z_axis']:7.3f}\r\n")
                uart_msg += f"  Total Magnitude: {total_accel:.3f} m/s^2\r\n"
                uart_msg += "\r\n"
                
                # Gyroscope data
                uart_msg += "GYROSCOPE (deg/s):\r\n"
                uart_msg += (f"  X: {gyro_data['x_axis']:7.2f}  "
                            f"Y: {gyro_data['y_axis']:7.2f}  "
                            f"Z: {gyro_data['z_axis']:7.2f}\r\n")
                uart_msg += f"  Total Magnitude: {total_gyro:.2f} deg/s\r\n"
                uart_msg += "\r\n"
                
                # Magnetometer data
                uart_msg += "MAGNETOMETER (uT):\r\n"
                uart_msg += (f"  X: {mag_data['x_axis']:7.2f}  "
                            f"Y: {mag_data['y_axis']:7.2f}  "
                            f"Z: {mag_data['z_axis']:7.2f}\r\n")
                uart_msg += f"  Total Magnitude: {total_mag:.2f} uT\r\n"
                uart_msg += "\r\n"
                
                # Attitude data (filtered)
                uart_msg += "ATTITUDE (FILTERED):\r\n"
                uart_msg += (f"  Pitch: "
                            f"{filtered_angles['pitch_angle']:7.2f} deg\r\n")
                uart_msg += (f"  Roll:  "
                            f"{filtered_angles['roll_angle']:7.2f} deg\r\n")
                uart_msg += f"  Heading: {heading:7.1f} deg (Magnetic North)\r\n"
                uart_msg += "=" * 70 + "\r\n"
                uart_msg += "\r\n"
                
                # Send the complete message via UART
                uart_handler.write(uart_msg.encode())
                utime.sleep_ms(100) 
                print(f"9AxisManager: Comprehensive data sent via UART "
                      f"({len(uart_msg)} bytes)")
            except Exception as e:
                print(f"9AxisManager: UART send error: {e}")

        # === DISPLAY ON OLED (MULTI-SCREEN CYCLING) ===
        if oled_manager:
            print("9AxisManager: OLED manager is available")
            if oled_manager.is_available():
                print("9AxisManager: OLED display is initialized")
                try:
                    # Get current time for display
                    current_time = utime.localtime()
                    time_str = (f"{current_time[3]:02d}:"
                               f"{current_time[4]:02d}:"
                               f"{current_time[5]:02d}")

                    # SCREEN 1: ACCELEROMETER DATA
                    oled_manager.display.fill(0)
                    oled_manager.display.text("ACCELEROMETER", 0, 0)
                    oled_manager.display.text(f"{time_str}", 80, 0)
                    oled_manager.display.text(
                        f"X:{accel_data['x_axis']:6.2f}m/s2",
                        0, 16
                    )
                    oled_manager.display.text(
                        f"Y:{accel_data['y_axis']:6.2f}m/s2",
                        0, 28
                    )
                    oled_manager.display.text(
                        f"Z:{accel_data['z_axis']:6.2f}m/s2",
                        0, 40
                    )
                    oled_manager.display.text(
                        f"Mag:{total_accel:6.2f}m/s2",
                        0, 52
                    )
                    oled_manager.display.show()
                    print("[OK] OLED Screen 1: Accelerometer data displayed")
                    utime.sleep(2)

                    # SCREEN 2: GYROSCOPE DATA
                    oled_manager.display.fill(0)
                    oled_manager.display.text("GYROSCOPE", 0, 0)
                    oled_manager.display.text(f"{time_str}", 80, 0)
                    oled_manager.display.text(
                        f"X:{gyro_data['x_axis']:7.1f}deg/s",
                        0, 16
                    )
                    oled_manager.display.text(
                        f"Y:{gyro_data['y_axis']:7.1f}deg/s",
                        0, 28
                    )
                    oled_manager.display.text(
                        f"Z:{gyro_data['z_axis']:7.1f}deg/s",
                        0, 40
                    )
                    oled_manager.display.text(
                        f"Mag:{total_gyro:6.1f}deg/s",
                        0, 52
                    )
                    oled_manager.display.show()
                    print("[OK] OLED Screen 2: Gyroscope data displayed")
                    utime.sleep(2)

                    # SCREEN 3: MAGNETOMETER DATA
                    oled_manager.display.fill(0)
                    oled_manager.display.text("MAGNETOMETER", 0, 0)
                    oled_manager.display.text(f"{time_str}", 80, 0)
                    oled_manager.display.text(
                        f"X:{mag_data['x_axis']:7.1f}uT",
                        0, 16
                    )
                    oled_manager.display.text(
                        f"Y:{mag_data['y_axis']:7.1f}uT",
                        0, 28
                    )
                    oled_manager.display.text(
                        f"Z:{mag_data['z_axis']:7.1f}uT",
                        0, 40
                    )
                    oled_manager.display.text(
                        f"Mag:{total_mag:6.1f}uT",
                        0, 52
                    )
                    oled_manager.display.show()
                    print("[OK] OLED Screen 3: Magnetometer data displayed")
                    utime.sleep(2)

                    # SCREEN 4: ATTITUDE & TEMPERATURE
                    oled_manager.display.fill(0)
                    oled_manager.display.text("ATTITUDE", 0, 0)
                    oled_manager.display.text(f"{time_str}", 80, 0)
                    oled_manager.display.text(
                        f"Pitch:{filtered_angles['pitch_angle']:6.1f}deg",
                        0, 16
                    )
                    oled_manager.display.text(
                        f"Roll: {filtered_angles['roll_angle']:6.1f}deg",
                        0, 28
                    )
                    oled_manager.display.text(
                        f"Hdg:  {heading:6.1f}deg",
                        0, 40
                    )
                    oled_manager.display.text(
                        f"Temp: {temperature:5.1f}C",
                        0, 52
                    )
                    oled_manager.display.show()
                    print("[OK] OLED Screen 4: Attitude & temp displayed")
                    utime.sleep(2)

                    print("[OK] 9AxisManager: All data displayed on OLED "
                          "(4 screens)")

                except Exception as e:
                    print(f"[ERROR] 9AxisManager: OLED display error: {e}")
            else:
                print("[ERROR] 9AxisManager: OLED display not initialized")
        else:
            print("[ERROR] 9AxisManager: OLED manager not provided")

        return data

    except Exception as e:
        error_msg = f"Error reading 9-axis data: {e}"
        print(f"9AxisManager: {error_msg}")

        # Display error on OLED
        if oled_manager and oled_manager.is_available():
            try:
                oled_manager.display.fill(0)
                oled_manager.display.text("9-Axis IMU", 0, 0)
                oled_manager.display.text("READ ERROR", 0, 16)
                oled_manager.display.text(str(e)[:16], 0, 32)
                oled_manager.display.show()
            except:
                pass

        # Send error via UART
        if uart_handler:
            try:
                uart_handler.write(f"ERROR: {error_msg}\r\n".encode())
            except:
                pass

        return None

def get_9axis_status():
    """
    Get status information for the 9-axis IMU sensor.

    Returns:
        dict or None: Status information if sensor available, None otherwise
    """
    if not is_9axis_available():
        return None

    try:
        return _nine_axis_imu_handler.get_sensor_status()
    except Exception as e:
        print(f"9AxisManager: Error getting status: {e}")
        return None

# =============================================================================
# EXAMPLE USAGE (FOR TESTING)
# =============================================================================

if __name__ == "__main__":
    """
    Example usage and testing code.

    This code runs when the module is executed directly (not imported).
    It demonstrates basic usage of the 9-axis manager functions.
    """
    print("VyomSat 9-Axis IMU Manager - Test Mode")
    print("=" * 70)

    # Initialize 9-axis IMU sensor
    if initialize_9axis_sensor():
        print("9-axis IMU initialized successfully")

        # Display status
        status = get_9axis_status()
        if status:
            print("\nSensor Status:")
            print(f"  Initialized: {status['is_sensor_initialized']}")
            print(f"  Connected: {status['is_sensor_connected']}")

        # Read and display data continuously
        print("\nReading 9-axis data (press Ctrl+C to stop)...")
        print("-" * 70)

        try:
            while True:
                # Read and display data (without UART/OLED)
                data = read_and_display_9axis_data()

                if data:
                    print("Data read successfully")
                else:
                    print("Failed to read data")

                # Wait before next reading
                utime.sleep(2)

        except KeyboardInterrupt:
            print("\n" + "-" * 70)
            print("Test stopped by user")

    else:
        print("Failed to initialize 9-axis IMU sensor")
        print("Check hardware connections and try again")


