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
VyomSat Camera Manager Module
==============================

This module manages the ArduCam SPI Mega camera for the VyomSat CubeSat
education kit. It provides functions to initialize the camera and capture
images with auto-incremented filenames.

FEATURES:
========
- Camera initialization and configuration
- Image capture with auto-incremented filenames
- Status checking for camera availability
- Integration with VyomSat telemetry system

HARDWARE REQUIREMENTS:
=====================
- ArduCam SPI Mega camera module connected via SPI:
  * SCK (Serial Clock) → GPIO 18
  * MISO (Master In Slave Out) → GPIO 16
  * MOSI (Master Out Slave In) → GPIO 19
  * CS (Chip Select) → GPIO 17
  * VCC → 3.3V or 5V (check module specifications)
  * GND → Ground

USAGE:
=====
1. Import this module
2. Call initialize_camera() during system startup
3. Call capture_camera_image() to capture photos
4. Use is_camera_available() to check if camera is ready

AUTHOR: VyomSat CubeSat Education Kit
VERSION: 1.0
"""

# =============================================================================
# IMPORT STATEMENTS
# =============================================================================

# MicroPython Hardware Abstraction Layer
from machine import Pin, SPI
import utime

# Camera Handler (assumed to be in the same folder)
try:
    from camera_handler import Camera, FileManager
    CAMERA_HANDLER_AVAILABLE = True
except ImportError:
    CAMERA_HANDLER_AVAILABLE = False
    print("[WARNING] camera_handler module not found")

# =============================================================================
# HARDWARE CONFIGURATION CONSTANTS
# =============================================================================

# SPI Bus Configuration
SPI_BUS_ID = 0
SPI_SCK_PIN = 18   # Serial Clock (GP18) - White wire
SPI_MISO_PIN = 16  # Master In Slave Out (GP16) - Brown wire
SPI_MOSI_PIN = 19  # Master Out Slave In (GP19) - Yellow wire
SPI_BAUDRATE = 8000000  # 8 MHz

# Camera Chip Select Pin
CAMERA_CS_PIN = 17  # Chip Select (GP17) - Orange wire

# =============================================================================
# CAMERA SETTINGS CONSTANTS
# =============================================================================

# Image Resolution
DEFAULT_RESOLUTION = '640x480'

# Image Quality Settings
CAMERA_BRIGHTNESS = 'BRIGHTNESS_PLUS_4'  # Brightness level
CAMERA_CONTRAST = 'CONTRAST_MINUS_3'     # Contrast level

# =============================================================================
# TIMING CONSTANTS
# =============================================================================

# Delay times in milliseconds
CAMERA_INIT_DELAY_MS = 3000  # Initial delay after camera object creation

# =============================================================================
# FILE MANAGEMENT CONSTANTS
# =============================================================================

# Default base filename for saved images
DEFAULT_IMAGE_BASENAME = 'vyomsat_image'

# =============================================================================
# MODULE-LEVEL VARIABLES
# =============================================================================

# Global camera object
_camera = None

# Global file manager object
_file_manager = None

# Camera hardware objects
_camera_spi_bus = None
_camera_chip_select = None

# =============================================================================
# CAMERA INITIALIZATION FUNCTIONS
# =============================================================================

def initialize_camera():
    """
    Initialize the ArduCam SPI Mega camera module.

    This function:
    1. Checks if camera handler module is available
    2. Initializes SPI bus and chip select pin
    3. Creates camera object
    4. Configures camera settings (resolution, brightness, contrast)
    5. Creates file manager for auto-incremented filenames

    Returns:
        bool: True if initialization successful, False otherwise
    """
    global _camera, _file_manager, _camera_spi_bus, _camera_chip_select

    # Check if camera handler module is available
    if not CAMERA_HANDLER_AVAILABLE:
        print("[ERROR] Camera handler module not available")
        return False

    try:
        # Initialize file manager for auto-incrementing filenames
        _file_manager = FileManager()
        print("[OK] Camera file manager initialized")

        # Configure SPI bus for camera communication
        _camera_spi_bus = SPI(
            SPI_BUS_ID,
            sck=Pin(SPI_SCK_PIN),
            miso=Pin(SPI_MISO_PIN),
            mosi=Pin(SPI_MOSI_PIN),
            baudrate=SPI_BAUDRATE
        )
        print(f"[OK] SPI bus {SPI_BUS_ID} initialized at {SPI_BAUDRATE} Hz")

        # Chip select pin for camera
        _camera_chip_select = Pin(CAMERA_CS_PIN, Pin.OUT)
        print(f"[OK] Camera CS pin initialized on GPIO {CAMERA_CS_PIN}")

        # Create camera object with debug output enabled
        _camera = Camera(
            _camera_spi_bus,
            _camera_chip_select,
            debug_text_enabled=True
        )
        print("[OK] Camera object created")

        # Wait to remove the green hue in the image
        # Refer to: https://forum.arducam.com/t/mega-3mp-micropython-driver/5708
        utime.sleep_ms(CAMERA_INIT_DELAY_MS)

        # Set image resolution
        _camera.resolution = DEFAULT_RESOLUTION
        print(f"[OK] Camera resolution set to {DEFAULT_RESOLUTION}")

        # Adjust brightness and contrast for better image quality
        _camera.set_brightness_level(getattr(_camera, CAMERA_BRIGHTNESS))
        _camera.set_contrast(getattr(_camera, CAMERA_CONTRAST))
        print(f"[OK] Camera brightness and contrast configured")

        print("[OK] Camera initialization complete!")
        return True

    except Exception as e:
        print(f"[ERROR] Camera initialization failed: {e}")
        _camera = None
        _file_manager = None
        _camera_spi_bus = None
        _camera_chip_select = None
        return False


def is_camera_available():
    """
    Check if camera is initialized and available.

    Returns:
        bool: True if camera is available, False otherwise
    """
    return _camera is not None and _file_manager is not None


def capture_and_display_image(uart_handler, oled_manager,
                               base_filename=DEFAULT_IMAGE_BASENAME):
    """
    Capture camera image with OLED display feedback.

    This function provides a complete user experience for camera capture,
    including OLED status displays and UART response messages. It handles
    all success/failure scenarios with appropriate user feedback.

    Args:
        uart_handler: UART handler instance for sending responses
        oled_manager: OLED manager instance for display updates
        base_filename (str): Base name for the image file

    Returns:
        str or None: Full path of the captured image file if successful,
                     None if capture failed
    """
    import utime

    # Check if camera is available
    if not is_camera_available():
        # Camera not available
        usb_response = "Camera capture FAILED: Camera not initialized"
        uart_response = "NAK: Camera not available"
        print(f"[ERROR] {usb_response}")

        # Display error on OLED
        if oled_manager and oled_manager.is_available():
            oled_manager.clear_display()
            oled_manager.display_text("Camera Error!", 0, 0)
            oled_manager.display_text("Not initialized", 0, 16)
            oled_manager.show()
            utime.sleep(2)  # Display for 2 seconds

        # Send UART response
        try:
            uart_response_formatted = f"{uart_response}\r\n"
            uart_handler.write(uart_response_formatted.encode())
        except Exception as e:
            print(f"Failed to send UART response: {e}")

        return None

    # Display capture status on OLED
    if oled_manager and oled_manager.is_available():
        oled_manager.clear_display()
        oled_manager.display_text("Camera", 0, 0)
        oled_manager.display_text("Capturing...", 0, 16)
        oled_manager.show()

    # Capture image
    success, message, filename = capture_camera_image(base_filename)

    if success:
        # Update responses with success message
        usb_response = f"Camera capture SUCCESS: {message}"
        uart_response = f"ACK: {message}"

        # Display success on USB
        print(f"[OK] {usb_response}")

        # Display success on OLED
        if oled_manager and oled_manager.is_available():
            oled_manager.clear_display()
            oled_manager.display_text("Camera OK!", 0, 0)
            oled_manager.display_text("Image saved:", 0, 16)
            oled_manager.display_text(filename[:16], 0, 32)
            oled_manager.show()
            utime.sleep(3)  # Display for 3 seconds

        # Send UART response
        try:
            uart_response_formatted = f"{uart_response}\r\n"
            uart_handler.write(uart_response_formatted.encode())
        except Exception as e:
            print(f"Failed to send UART response: {e}")

        # Return full path of the captured image
        return filename

    else:
        # Update responses with failure message
        usb_response = f"Camera capture FAILED: {message}"
        uart_response = f"NAK: {message}"

        # Display failure on USB
        print(f"[ERROR] {usb_response}")

        # Display failure on OLED
        if oled_manager and oled_manager.is_available():
            oled_manager.clear_display()
            oled_manager.display_text("Camera Error!", 0, 0)
            oled_manager.display_text("Capture failed", 0, 16)
            oled_manager.show()
            utime.sleep(2)  # Display for 2 seconds

        # Send UART response
        try:
            uart_response_formatted = f"{uart_response}\r\n"
            uart_handler.write(uart_response_formatted.encode())
        except Exception as e:
            print(f"Failed to send UART response: {e}")

        return None


# =============================================================================
# CAMERA CAPTURE FUNCTIONS
# =============================================================================

def capture_camera_image(base_filename=DEFAULT_IMAGE_BASENAME):
    """
    Capture a photo and save it with an auto-incremented filename.

    This function:
    1. Checks if camera is available
    2. Captures JPEG image from camera
    3. Waits to remove green hue (as per ArduCam forum recommendation)
    4. Saves image with auto-incremented filename
    5. Returns success status and message

    Args:
        base_filename (str): Base name for the image file

    Returns:
        tuple: (success: bool, message: str, filename: str or None)
            - success: True if capture successful, False otherwise
            - message: Status message describing the result
            - filename: Name of the saved file, or None if failed
    """
    # Check if camera is available
    if not is_camera_available():
        error_message = "Camera not initialized or not available"
        print(f"[ERROR] {error_message}")
        return False, error_message, None

    try:
        print("[INFO] Starting camera capture...")

        # Capture JPEG image from camera
        _camera.capture_jpg()
        print("[INFO] Image captured, processing...")

        # Wait to remove the green hue in the image
        # Refer to: https://forum.arducam.com/t/mega-3mp-micropython-driver/5708
        utime.sleep_ms(CAMERA_INIT_DELAY_MS)

        # Save with auto-incremented filename and progress indicator
        output_filename = _file_manager.new_jpg_filename(base_filename)
        print(f"[INFO] Saving image as '{output_filename}'...")
        _camera.save_JPG(output_filename, progress_bar=True)

        success_message = f"Image saved as {output_filename}"
        print(f"[OK] {success_message}")
        return True, success_message, output_filename

    except Exception as e:
        error_message = f"Camera capture failed: {e}"
        print(f"[ERROR] {error_message}")
        return False, error_message, None


# =============================================================================
# MODULE TEST (optional)
# =============================================================================

if __name__ == "__main__":
    """
    Test the camera manager module standalone.

    This test:
    1. Initializes the camera
    2. Captures a test image
    3. Reports results
    """
    print("=" * 70)
    print("VyomSat Camera Manager - Standalone Test")
    print("=" * 70)

    # Initialize camera
    if initialize_camera():
        print("[OK] Camera initialization successful")

        # Wait a moment
        utime.sleep(1)

        # Capture test image
        success, message, filename = capture_camera_image()

        if success:
            print(f"[OK] Test completed successfully: {message}")
        else:
            print(f"[ERROR] Test failed: {message}")
    else:
        print("[ERROR] Camera initialization failed")

    print("=" * 70)

