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
VyomSat GPS Manager Module
==========================

VyomSat - Essence of Space. Built by You.
The CubeSat education kit - Hands-on CubeSat engineering, end-to-end

OVERVIEW:
========
This module provides GPS functionality for the VyomSat CubeSat education kit.
It manages the NEO-6M GPS module via UART1 and provides functions for reading
and displaying GPS data (latitude, longitude, altitude, satellites, etc.).

FEATURES:
========
- GPS sensor initialization (UART1 on GP4/GP5)
- Non-blocking GPS data reading
- Basic GPS data extraction (lat/lon for regular display)
- Comprehensive GPS diagnostics (for detailed reporting)
- GPS availability checking

HARDWARE REQUIREMENTS:
=====================
- NEO-6M GPS Module connected to UART1:
  * GPS TX → Pico GP5 (UART1 RX)
  * GPS RX → Pico GP4 (UART1 TX)
  * GPS VCC → 3.3V
  * GPS GND → Ground

USAGE:
=====
1. Call initialize_gps_sensor() during system startup
2. Call read_gps_data() in main loop to update GPS data
3. Use get_basic_gps_data() to get lat/lon for regular display
4. Use display_full_gps_data() for comprehensive GPS information

AUTHOR: VyomSat CubeSat Education Kit
VERSION: 1.0 - GPS Integration
DATE: 2024
"""

# =============================================================================
# IMPORT STATEMENTS
# =============================================================================

# MicroPython Hardware Abstraction Layer
from machine import Pin, UART

# MicroPython Time Module
import utime

# GPS Handler (assumed to be in same folder)
from neo6mv2_handler import NEO6MV2Handler

# =============================================================================
# GLOBAL VARIABLES
# =============================================================================

# GPS UART instance
_gps_uart = None

# GPS Handler instance
_gps_handler = None

# GPS initialization status
_gps_initialized = False

# =============================================================================
# GPS INITIALIZATION FUNCTION
# =============================================================================

def initialize_gps_sensor(uart_id=1, tx_pin=4, rx_pin=5, baudrate=9600,
                         timezone_offset_hours=4):
    """
    Initialize the NEO-6M GPS sensor on UART1.

    This function configures UART1 for GPS communication and creates
    a GPS handler instance for parsing NMEA sentences.

    Args:
        uart_id (int): UART interface ID (default: 1 for UART1)
        tx_pin (int): GPIO pin for UART TX (default: 4 → GP4)
        rx_pin (int): GPIO pin for UART RX (default: 5 → GP5)
        baudrate (int): UART baudrate (default: 9600 for NEO-6M)
        timezone_offset_hours (float): Timezone offset from UTC
                                      (default: 4 for Dubai/UAE)

    Returns:
        bool: True if initialization successful, False otherwise
    """
    global _gps_uart, _gps_handler, _gps_initialized

    try:
        # Initialize UART for GPS communication
        _gps_uart = UART(
            uart_id,
            baudrate=baudrate,
            tx=Pin(tx_pin),
            rx=Pin(rx_pin)
        )

        # Create GPS handler instance
        _gps_handler = NEO6MV2Handler(
            timezone_offset_hours=timezone_offset_hours
        )

        _gps_initialized = True
        print(f"[OK] GPS sensor initialized on UART{uart_id} "
              f"(TX=GP{tx_pin}, RX=GP{rx_pin}, baud={baudrate})")
        return True

    except Exception as e:
        print(f"[ERROR] GPS initialization failed: {e}")
        _gps_initialized = False
        return False


# =============================================================================
# GPS DATA READING FUNCTION
# =============================================================================

def read_gps_data():
    """
    Read available GPS data from UART and update GPS handler.

    This function performs a non-blocking read from UART and feeds
    characters to the GPS parser. Call this regularly in the main loop
    to keep GPS data updated.

    Returns:
        bool: True if data was read, False otherwise
    """
    global _gps_uart, _gps_handler, _gps_initialized

    if not _gps_initialized or _gps_uart is None or _gps_handler is None:
        return False

    try:
        # Non-blocking UART read
        available = _gps_uart.any()
        if available:
            buf = _gps_uart.read(available)
            if buf:
                # Feed each byte to GPS parser
                for byte_val in buf:
                    _gps_handler.update_from_char(chr(byte_val))
                return True

    except Exception as e:
        print(f"GPS read error: {e}")
        return False

    return False


# =============================================================================
# GPS STATUS CHECK FUNCTION
# =============================================================================

def is_gps_available():
    """
    Check if GPS sensor is initialized and has a valid fix.

    Returns:
        bool: True if GPS is initialized and has fix, False otherwise
    """
    global _gps_handler, _gps_initialized

    if not _gps_initialized or _gps_handler is None:
        return False

    return _gps_handler.has_fix


# =============================================================================
# BASIC GPS DATA EXTRACTION
# =============================================================================

def get_basic_gps_data():
    """
    Get basic GPS data for regular telemetry display.

    Returns latitude, longitude, and fix status for inclusion in
    regular housekeeping telemetry.

    Returns:
        dict: Dictionary containing:
            - 'has_fix': bool - Whether GPS has valid fix
            - 'latitude': str - Latitude string (e.g., "25.2521° N")
            - 'longitude': str - Longitude string (e.g., "55.3095° E")
            - 'altitude': float - Altitude in meters
            - 'satellites': int - Number of satellites in use
            - 'latitude_dd': str - Latitude in decimal degrees
            - 'longitude_dd': str - Longitude in decimal degrees
    """
    global _gps_handler, _gps_initialized

    # Default values when GPS not available
    default_data = {
        'has_fix': False,
        'latitude': 'N/A',
        'longitude': 'N/A',
        'altitude': 0.0,
        'satellites': 0,
        'latitude_dd': 'N/A',
        'longitude_dd': 'N/A'
    }

    if not _gps_initialized or _gps_handler is None:
        return default_data

    try:
        if _gps_handler.has_fix:
            # Get decimal degree strings
            lat_dd = NEO6MV2Handler.convert_coord_to_decimal_degrees_string(
                _gps_handler.latitude
            )
            lon_dd = NEO6MV2Handler.convert_coord_to_decimal_degrees_string(
                _gps_handler.longitude
            )

            return {
                'has_fix': True,
                'latitude': _gps_handler.latitude_string(),
                'longitude': _gps_handler.longitude_string(),
                'altitude': _gps_handler.altitude_m,
                'satellites': _gps_handler.satellites_used_for_fix,
                'latitude_dd': lat_dd,
                'longitude_dd': lon_dd
            }
        else:
            # GPS initialized but no fix yet
            return {
                'has_fix': False,
                'latitude': 'Acquiring...',
                'longitude': 'Acquiring...',
                'altitude': 0.0,
                'satellites': _gps_handler.satellites_in_view,
                'latitude_dd': 'N/A',
                'longitude_dd': 'N/A'
            }

    except Exception as e:
        print(f"Error getting GPS data: {e}")
        return default_data


# =============================================================================
# COMPREHENSIVE GPS DISPLAY FUNCTIONS
# =============================================================================

def display_full_gps_data_usb(gps_handler):
    """
    Display comprehensive GPS diagnostics via USB serial.

    This function prints detailed GPS information including position,
    time, motion, satellites, and fix quality to the USB serial port.

    Args:
        gps_handler: NEO6MV2Handler instance (uses global if None)

    Returns:
        None
    """
    global _gps_handler, _gps_initialized

    if gps_handler is None:
        gps_handler = _gps_handler

    if not _gps_initialized or gps_handler is None:
        print("GPS not initialized")
        return

    if not gps_handler.has_fix:
        print("\n" + "=" * 70)
        print("GPS STATUS: Acquiring fix...")
        print(f"Sentences received: {gps_handler.valid_sentence_count}")
        print(f"Satellites visible: {gps_handler.satellites_in_view}")
        print("=" * 70 + "\n")
        return

    print("\n" + "=" * 70)
    print("COMPREHENSIVE GPS DIAGNOSTIC DATA")
    print("=" * 70)

    # Position Data
    print("\nPOSITION DATA:")
    print("-" * 40)
    lat_dd = NEO6MV2Handler.convert_coord_to_decimal_degrees_string(
        gps_handler.latitude
    )
    lon_dd = NEO6MV2Handler.convert_coord_to_decimal_degrees_string(
        gps_handler.longitude
    )
    print(f"Latitude (decimal):      {lat_dd}")
    print(f"Longitude (decimal):     {lon_dd}")
    print(f"Altitude:                {gps_handler.altitude_m} m")
    print(f"Geoid Height:            {gps_handler.geoid_separation_m} m")

    # Time/Date Data
    print("\nTIME & DATE DATA:")
    print("-" * 40)
    h, m, s = gps_handler.time_hms_local
    print(f"GPS Time (local):        {h:02d}:{m:02d}:{s:05.2f}")
    print(f"Date:                    {gps_handler.date_string('s_dmy')}")
    print(f"Timezone Offset (h):     {gps_handler.timezone_offset_hours}")

    # Motion Data
    print("\nMOTION DATA:")
    print("-" * 40)
    print(f"Course/Heading:          {gps_handler.course_over_ground_deg}")
    print(f"Compass Direction:       "
          f"{gps_handler.course_compass_direction()}")
    print(f"Speed (km/h):            {gps_handler.speed_string('kph')}")
    print(f"Speed (mph):             {gps_handler.speed_string('mph')}")
    print(f"Speed (knots):           {gps_handler.speed_string('knot')}")

    # Satellite Data
    print("\nSATELLITE DATA:")
    print("-" * 40)
    print(f"Satellites in View:      {gps_handler.satellites_in_view}")
    print(f"Satellites in Use:       {gps_handler.satellites_used_for_fix}")
    print(f"Satellites Used List:    {gps_handler.satellite_ids_used}")

    # Fix Quality Data
    print("\nFIX QUALITY DATA:")
    print("-" * 40)
    print(f"Fix Status (GGA):        {gps_handler.fix_quality_gga}")
    print(f"Fix Type  (GSA):         {gps_handler.fix_type_gsa}")
    print(f"Valid Fix:               {gps_handler.has_fix}")
    print(f"Time Since Fix:          "
          f"{gps_handler.milliseconds_since_last_fix()} ms")

    # Dilution of Precision
    print("\nDILUTION OF PRECISION:")
    print("-" * 40)
    print(f"HDOP:                    {gps_handler.hdop}")
    print(f"VDOP:                    {gps_handler.vdop}")
    print(f"PDOP:                    {gps_handler.pdop}")

    print("\n" + "=" * 70 + "\n")


def format_gps_uart_message(gps_handler):
    """
    Format comprehensive GPS data for UART transmission.

    Args:
        gps_handler: NEO6MV2Handler instance (uses global if None)

    Returns:
        str: Formatted GPS data string for UART transmission
    """
    global _gps_handler, _gps_initialized

    if gps_handler is None:
        gps_handler = _gps_handler

    if not _gps_initialized or gps_handler is None:
        return "GPS: Not initialized\r\n"

    if not gps_handler.has_fix:
        return (f"GPS: Acquiring fix | "
                f"Sats visible: {gps_handler.satellites_in_view}\r\n")

    lat_dd = NEO6MV2Handler.convert_coord_to_decimal_degrees_string(
        gps_handler.latitude
    )
    lon_dd = NEO6MV2Handler.convert_coord_to_decimal_degrees_string(
        gps_handler.longitude
    )

    h, m, s = gps_handler.time_hms_local

    message = (
        f"GPS DATA:\r\n"
        f"Lat: {lat_dd} | Lon: {lon_dd}\r\n"
        f"Alt: {gps_handler.altitude_m}m | "
        f"Sats: {gps_handler.satellites_used_for_fix}\r\n"
        f"Time: {h:02d}:{m:02d}:{s:05.2f} | "
        f"Date: {gps_handler.date_string('s_dmy')}\r\n"
        f"Speed: {gps_handler.speed_string('kph')} | "
        f"Course: {gps_handler.course_over_ground_deg}deg\r\n"
        f"HDOP: {gps_handler.hdop} | "
        f"VDOP: {gps_handler.vdop} | "
        f"PDOP: {gps_handler.pdop}\r\n"
    )

    return message


def display_gps_on_oled(oled_manager, gps_handler):
    """
    Display comprehensive GPS data on OLED display across multiple screens.

    This function cycles through 3 OLED screens showing different GPS data.
    Each screen is displayed for 2 seconds.

    Args:
        oled_manager: OledManager instance for display control
        gps_handler: NEO6MV2Handler instance (uses global if None)

    Returns:
        None
    """
    global _gps_handler, _gps_initialized

    if gps_handler is None:
        gps_handler = _gps_handler

    if not _gps_initialized or gps_handler is None:
        if oled_manager:
            oled_manager.display.fill(0)
            oled_manager.display.text("GPS Manager", 0, 0, 1)
            oled_manager.display.text("Status:", 0, 16, 1)
            oled_manager.display.text("Not Initialized", 0, 32, 1)
            oled_manager.display.show()
        return

    if not gps_handler.has_fix:
        if oled_manager:
            oled_manager.display.fill(0)
            oled_manager.display.text("GPS Status", 0, 0, 1)
            oled_manager.display.text("Acquiring Fix...", 0, 16, 1)
            oled_manager.display.text(
                f"Sats: {gps_handler.satellites_in_view}",
                0,
                32,
                1
            )
            oled_manager.display.text(
                f"Rcvd: {gps_handler.valid_sentence_count}",
                0,
                48,
                1
            )
            oled_manager.display.show()
        return

    # Screen 1: Position Data
    if oled_manager:
        oled_manager.display.fill(0)
        oled_manager.display.text("GPS Position", 0, 0, 1)
        
        lat_dd = NEO6MV2Handler.convert_coord_to_decimal_degrees_string(
            gps_handler.latitude
        )
        lon_dd = NEO6MV2Handler.convert_coord_to_decimal_degrees_string(
            gps_handler.longitude
        )
        
        oled_manager.display.text(f"Lat:{lat_dd[:10]}", 0, 16, 1)
        oled_manager.display.text(f"Lon:{lon_dd[:10]}", 0, 28, 1)
        oled_manager.display.text(
            f"Alt:{gps_handler.altitude_m:.1f}m",
            0,
            40,
            1
        )
        oled_manager.display.text(
            f"Sats:{gps_handler.satellites_used_for_fix}",
            0,
            52,
            1
        )
        oled_manager.display.show()
        utime.sleep(2)

    # Screen 2: Time and Motion
    if oled_manager:
        oled_manager.display.fill(0)
        oled_manager.display.text("GPS Time/Motion", 0, 0, 1)
        
        h, m, s = gps_handler.time_hms_local
        oled_manager.display.text(
            f"Time:{h:02d}:{m:02d}:{int(s):02d}",
            0,
            16,
            1
        )
        oled_manager.display.text(
            f"Date:{gps_handler.date_string('s_dmy')}",
            0,
            28,
            1
        )
        
        speed_kph = gps_handler.speed_knots_mph_kph[2]
        oled_manager.display.text(f"Spd:{speed_kph:.1f}km/h", 0, 40, 1)
        oled_manager.display.text(
            f"Crs:{gps_handler.course_over_ground_deg:.0f}deg",
            0,
            52,
            1
        )
        oled_manager.display.show()
        utime.sleep(2)

    # Screen 3: Fix Quality
    if oled_manager:
        oled_manager.display.fill(0)
        oled_manager.display.text("GPS Quality", 0, 0, 1)
        oled_manager.display.text(
            f"Fix:{gps_handler.fix_quality_gga}",
            0,
            16,
            1
        )
        oled_manager.display.text(
            f"Type:{gps_handler.fix_type_gsa}",
            0,
            28,
            1
        )
        oled_manager.display.text(f"HDOP:{gps_handler.hdop:.2f}", 0, 40, 1)
        oled_manager.display.text(f"PDOP:{gps_handler.pdop:.2f}", 0, 52, 1)
        oled_manager.display.show()
        utime.sleep(2)


# =============================================================================
# BASIC GPS DISPLAY ON OLED (FOR REGULAR TELEMETRY)
# =============================================================================

def update_oled_with_gps_and_hk(oled_manager, battery_voltage,
                                message_counter):
    """
    Update OLED display with basic housekeeping and GPS data.

    This function is called in the main loop to show battery voltage,
    message counter, and basic GPS information (lat/lon) on the OLED.

    Args:
        oled_manager: OledManager instance for display control
        battery_voltage (float): Battery voltage in volts
        message_counter (int): Message counter for display

    Returns:
        None
    """
    global _gps_handler, _gps_initialized

    if not oled_manager:
        return

    try:
        oled_manager.display.fill(0)
        
        # Header
        oled_manager.display.text("VyomSat EPS+GPS", 0, 0, 1)
        
        # Time display
        seconds = utime.ticks_ms() // 1000
        hours = (seconds // 3600) % 24
        minutes = (seconds // 60) % 60
        secs = seconds % 60
        oled_manager.display.text(
            f"{hours:02d}:{minutes:02d}:{secs:02d}",
            0,
            12,
            1
        )
        
        # Battery voltage
        oled_manager.display.text(f"V: {battery_voltage:.2f}V", 0, 24, 1)
        
        # GPS data
        if _gps_initialized and _gps_handler:
            gps_data = get_basic_gps_data()
            if gps_data['has_fix']:
                # Show basic lat/lon (truncated to fit)
                lat_str = gps_data['latitude_dd'][:10]
                lon_str = gps_data['longitude_dd'][:10]
                oled_manager.display.text(f"Lat:{lat_str}", 0, 36, 1)
                oled_manager.display.text(f"Lon:{lon_str}", 0, 48, 1)
            else:
                oled_manager.display.text("GPS: Acquiring", 0, 36, 1)
                oled_manager.display.text(
                    f"Sats:{gps_data['satellites']}",
                    0,
                    48,
                    1
                )
        else:
            oled_manager.display.text("GPS: N/A", 0, 36, 1)
            oled_manager.display.text(f"Msg: #{message_counter}", 0, 48, 1)
        
        oled_manager.display.show()
        
    except Exception as e:
        print(f"OLED display error: {e}")


# =============================================================================
# COMBINED GPS DISPLAY FUNCTION (FOR COMMAND 'g')
# =============================================================================

def read_and_display_full_gps_data(uart_handler, oled_manager):
    """
    Read and display comprehensive GPS data on USB, UART, and OLED.

    This function is called when command 'g' is received. It displays
    detailed GPS information across all output interfaces.

    Args:
        uart_handler: UartHandler instance for UART communication
        oled_manager: OledManager instance for OLED display

    Returns:
        None
    """
    global _gps_handler

    # Display on USB serial
    display_full_gps_data_usb(_gps_handler)

    # Send via UART
    if uart_handler:
        gps_message = format_gps_uart_message(_gps_handler)
        try:
            uart_handler.write(gps_message.encode())
        except Exception as e:
            print(f"Failed to send GPS data via UART: {e}")

    # Display on OLED (cycles through 3 screens, 2 sec each = 6 sec total)
    if oled_manager:
        display_gps_on_oled(oled_manager, _gps_handler)


# =============================================================================
# MODULE TEST (FOR STANDALONE TESTING)
# =============================================================================

if __name__ == "__main__":
    print("VyomSat GPS Manager Module")
    print("This module should be imported, not run directly")

