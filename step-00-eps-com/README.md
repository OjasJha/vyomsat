# VyomSat Step 0: EPS + COM Integration

**VyomSat - Essence of Space. Built by You.**

This is the foundational step of the VyomSat CubeSat education kit, demonstrating battery voltage monitoring and communication capabilities through multiple interfaces.

## Overview

Step 0 implements a complete Electrical Power System (EPS) monitoring solution with communication capabilities. The system continuously measures battery voltage using the Raspberry Pi Pico's ADC and presents the data through three interfaces:
- **OLED Display** - Local visualization
- **USB Serial Console** - Development and debugging
- **XBee via UART** - Wireless telemetry

## Features

- 📊 **Battery Voltage Monitoring** - Real-time ADC-based voltage sensing with voltage divider support
- 📺 **OLED Display** - 128x64 SSD1306 display showing system status and telemetry
- 📡 **UART Communication** - Full-duplex XBee communication for telemetry and commands
- 🎛️ **Command Interface** - Remote command execution via UART
- ⚡ **Hardware Reset** - Synchronized XBee reset to prevent race conditions

## Hardware Requirements

### Components
- Raspberry Pi Pico 2 or Pico W
- SSD1306 OLED Display (128x64 pixels, I2C)
- XBee module (or compatible UART device)
- Battery voltage divider circuit

### Wiring Connections

| Component | Pico Pin | Description |
|-----------|----------|-------------|
| **OLED Display** | | |
| SCL | GP7 | I2C Serial Clock |
| SDA | GP6 | I2C Serial Data |
| VCC | 3.3V | Power |
| GND | GND | Ground |
| **XBee Module** | | |
| TX (Pico → XBee) | GP0 | UART0 Transmit |
| RX (XBee → Pico) | GP1 | UART0 Receive |
| RST | GP2 | Hardware Reset (optional) |
| GND | GND | Common Ground |
| **Battery Sensing** | | |
| ADC Input | GP28 | Battery voltage via divider |

> **Note**: Battery voltage must be reduced to 0-3.3V range using a voltage divider circuit (default: 2:1 ratio allows measuring up to 6.6V).

## Architecture

The system is organized into modular components:

### 1. `vyomsat_battery_manager.py`
Handles battery voltage sensing through the RP2350's ADC.

**How it works:**
- Reads raw ADC value from GP28 (ADC2)
- Converts 16-bit reading (0-65535) to voltage (0-3.3V)
- Applies voltage divider correction to get actual battery voltage
- Provides multiple reading functions:
  - `read_battery_voltage()` - Corrected battery voltage
  - `read_adc_voltage()` - Raw ADC pin voltage
  - `read_raw_adc_value()` - Raw 16-bit ADC value

**Key Concept - Voltage Divider:**
```
Battery (+) → R1 → GP28 (ADC2) → R2 → GND

Vout = Vin × R2/(R1 + R2)
Battery Voltage = ADC Reading / Divider Factor
```

### 2. `vyomsat_oled_manager.py`
Manages the SSD1306 OLED display via I2C.

**Display Modes:**
- **Normal Mode** - Shows time, voltage, message counter, last received data
- **Command Mode** - Displays received commands with timestamp
- **Message Mode** - Shows received messages with length and timestamp

**How it works:**
- Creates I2C bus using software I2C (SoftI2C)
- Initializes SSD1306 display driver
- Provides methods to update display content
- Handles text rendering on 128x64 pixel screen (8 lines × 16 chars)

### 3. `vyomsat_xbee_manager.py`
Handles XBee module hardware reset.

**Purpose:**
Solves boot race condition where Pico boots faster than XBee. By performing a hardware reset after Pico initialization, ensures synchronized startup.

**Reset Sequence:**
1. RST pin HIGH (inactive)
2. Pull LOW for 100ms (active reset)
3. Release HIGH (XBee begins boot)
4. Wait 5 seconds for initialization

### 4. `vyomsat.py`
Main integration module that orchestrates all components.

**Operational Flow:**

```
┌─────────────────────────────────────────────┐
│ 1. Initialize Hardware                      │
│    - LED & GPIO pins                        │
│    - XBee hardware reset                    │
│    - Battery ADC sensor                     │
│    - OLED display                           │
│    - UART communication                     │
└─────────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────────┐
│ 2. Main Loop (every 4 seconds)             │
│    ┌─────────────────────────────────────┐ │
│    │ a. Read battery voltage             │ │
│    │ b. Update OLED with telemetry       │ │
│    │ c. Send telemetry via UART          │ │
│    │ d. Process incoming UART commands   │ │
│    │ e. Print status to USB console      │ │
│    └─────────────────────────────────────┘ │
└─────────────────────────────────────────────┘
```

## Telemetry Output

### USB Serial Console
```
[12345ms] Telemetry Sent | Voltage: 6.90V | Bytes: 65 | Counter: 42
```

### UART (XBee)
```
[12345ms] VyomSat EPS | Voltage: 6.90V | Msg: #42
```

### OLED Display
```
VyomSat EPS
Time: 14:23:45
Batt: 6.90V
Msg: 42
```

## Supported UART Commands

Send single character commands to control the system:

| Command | Function | Response |
|---------|----------|----------|
| `v` | Request voltage reading | Detailed voltage data |
| `a` | MPU6050 motion sensor | ACK (placeholder) |
| `b` | System reset | ACK (placeholder) |
| `c` | Camera capture | ACK (placeholder) |
| `s` | SD card logging | ACK (placeholder) |
| `g` | GPS data capture | ACK (placeholder) |

**Example:**
```
Send: v
Receive: ACK: Battery voltage reading initiated
         [timestamp] BATTERY VOLTAGE | Batt: 6.900V | ADC: 3.450V | Raw: 32768
```

## Usage

### 1. Setup Hardware
Connect all components according to the wiring table above.

### 2. Upload Code
Copy all Python files to your Raspberry Pi Pico:
- `vyomsat.py`
- `vyomsat_battery_manager.py`
- `vyomsat_oled_manager.py`
- `vyomsat_xbee_manager.py`
- `uart_handler.py` (UART communication handler)
- `ssd1306_handler.py` (OLED display driver)

### 3. Run
```python
# On Pico - execute main script
python vyomsat.py
```

Or set it as `main.py` to run automatically on boot.

### 4. Monitor
- **USB Console**: Connect serial terminal (115200 baud) to monitor debug output
- **OLED**: View real-time telemetry on display
- **XBee**: Receive telemetry stream and send commands

## Configuration

Key constants in `vyomsat.py`:

```python
# OLED Display
OLED_SCL_PIN = 7              # I2C clock pin
OLED_SDA_PIN = 6              # I2C data pin

# Battery Monitoring
BATTERY_ADC_CHANNEL = 2                    # ADC2 (GP28)
BATTERY_VOLTAGE_DIVIDER_FACTOR = 0.5       # 2:1 voltage divider

# UART Communication
UART_BAUDRATE = 9600          # Standard XBee baudrate
UART_TX_PIN = 0               # GP0
UART_RX_PIN = 1               # GP1

# XBee Reset
XBEE_RESET_PIN = 2            # GP2
XBEE_INIT_DELAY_SECONDS = 5   # Boot delay

# Telemetry Rate
MAIN_LOOP_DELAY_SECONDS = 4   # Update interval
```

## Troubleshooting

| Issue | Solution |
|-------|----------|
| No OLED display | Check I2C wiring (SCL→GP7, SDA→GP6) and power |
| OLED garbled text | Try I2C address 0x3D instead of 0x3C |
| No UART communication | Verify TX/RX connections and common ground |
| Incorrect voltage | Check voltage divider circuit and calibration |
| XBee not responding | Ensure reset pin is connected and working |

## Technical Details

### ADC Resolution
- **Hardware**: 12-bit ADC (0-4095)
- **MicroPython**: 16-bit reading (0-65535)
- **Reference**: 3.3V
- **Conversion Factor**: 3.3V / 65535 = 0.0000503540 V/count

### I2C Communication
- **Protocol**: Software I2C (SoftI2C)
- **Frequency**: 400 kHz (fast mode)
- **Address**: 0x3C (SSD1306 default)

### UART Communication
- **Baud Rate**: 9600 bps
- **Data Bits**: 8
- **Parity**: None
- **Stop Bits**: 1
- **Flow Control**: None

## Next Steps

This Step 0 module provides the foundation for more advanced VyomSat features:
- Step 1: Add MPU6050 motion sensor
- Step 2: Integrate GPS module
- Step 3: Add SD card data logging
- Step 4: Implement camera capture

## License

MIT License - Copyright (c) 2025 Ojas Jha

See individual Python files for full license text.

## Author

**Ojas Jha**  
VyomSat CubeSat Education Kit  
Date: October 25, 2025

