# VyomSat - Essence of Space. Built by You.

**A step-by-step learning kit for exploring microsatellite technology through hands-on CubeSat development**

🛰️ **Learn by Building** | 📚 **Model-Based Learning** | 🔧 **Off-the-Shelf Components** | 🚀 **Space Technology Made Accessible**

---

## 📖 Overview

VyomSat is an educational kit designed to make space technology accessible through hands-on learning. This repository guides you through building a functional CubeSat prototype from the ground up, one subsystem at a time, using readily available components and a Raspberry Pi Pico microcontroller.

### The Name

**Vyom** (व्योम) means sky, heavens, or outer space in Sanskrit.  
**Sat** ties to "satellite," but in Sanskrit also means truth, being, existence.

VyomSat can be read as:
- **"Space Satellite"** – straightforward and modern
- **"Truth of Space" / "Essence of the Sky"** – deeper, Sanskrit-inspired meaning

Short, internationally pronounceable, with both technical and cultural essence. 🚀✨

---

## 🎯 Mission & Vision

### Why This Project Exists

This project was born from a gap in STEM education: high school students often lack opportunities for truly challenging, real-world engineering experiences beyond rudimentary activities like water rockets or spaghetti towers. VyomSat aims to fill this gap by providing a comprehensive, hands-on introduction to space systems engineering.

### Mission
**Make space technology accessible to learners of all ages and expertise levels.**

### Vision
**Share knowledge about microsatellite technology through a practical kit that offers hands-on training and meaningful interaction with real engineering systems.**

Space is the final frontier—and it's also one of the best ways to learn engineering in a multidisciplinary way.

---

## 👥 Who Is This For?

**Target Audience:**
- 🔬 STEM enthusiasts
- 🛠️ Tinkerers and makers
- 🎨 DIY electronics hobbyists
- 🌌 Anyone interested in space, CubeSats, and satellites
- 📚 **Age no bar!** Anyone with high school-level science and basic programming can jump in

**Prerequisites:**
- Basic breadboarding skills
- Familiarity with Raspberry Pi Pico
- Basic MicroPython programming (or willingness to learn)
- A lot of curiosity! 🧠✨

---

## 🚀 Why CubeSats? Why Model-Based Learning?

### CubeSats as Learning Platforms

CubeSats (miniature satellites using standardized 10×10×10 cm units) are excellent platforms for learning because they:
- Cover **multidisciplinary engineering**: electrical, mechanical, software, communications
- Demonstrate **real-world constraints**: power, mass, volume, thermal management
- Introduce **systems engineering**: subsystems must work together seamlessly
- Are **achievable**: built with off-the-shelf components at educational scales
- Reflect **modern space technology**: actual CubeSats fly in orbit regularly

### Model-Based, Step-by-Step Learning

This repository uses a **progressive building approach**:
1. Start with foundational subsystems (power, communication)
2. Add sensors and payloads incrementally
3. Integrate everything into a working prototype
4. Understand the **what, why, and how** of each component

Each step builds on the previous one, allowing you to:
- ✅ Master one concept before moving to the next
- ✅ Troubleshoot issues in isolated subsystems
- ✅ Understand system integration challenges
- ✅ Build confidence through working prototypes

---

## 📂 Repository Structure

This repository contains **6 progressive steps**, each adding new capabilities to your CubeSat:

### **Step 0: EPS + COM Integration** (`step-00-eps-com/`)
**Electrical Power System (EPS) and Communication (COM)**
- Battery voltage monitoring via ADC
- OLED display for local visualization
- XBee UART communication for wireless telemetry
- Foundation for all subsequent steps

**Key Components:** Raspberry Pi Pico, OLED display (SSD1306), XBee module  
**Learn:** Power monitoring, I2C, UART, telemetry basics

---

### **Step 1: 9-Axis IMU Integration** (`step-01-9axis/`)
**Attitude Determination and Control System (ADCS) - Sensing**
- MPU9250 9-axis IMU (accelerometer + gyroscope + magnetometer)
- Pitch, roll, and heading calculations
- Sensor fusion for accurate attitude estimation
- Command-triggered comprehensive attitude data capture

**New Components:** MPU9250 9-axis IMU  
**Learn:** Inertial measurement, sensor fusion, attitude determination

---

### **Step 2: GPS Integration** (`step-02-gps/`)
**Navigation and Positioning**
- NEO-6M GPS module for precise location tracking
- Latitude, longitude, altitude, and satellite data
- GPS time synchronization with timezone support
- Speed and course over ground measurement

**New Components:** NEO-6M GPS module  
**Learn:** NMEA sentence parsing, GPS fundamentals, position tracking

---

### **Step 3: Camera Integration** (`step-03-camera/`)
**Payload - Earth Observation**
- ArduCam Mega 3MP camera for high-resolution imaging
- JPEG image capture with configurable quality
- Auto-incremented filenames for organized storage
- Command-triggered photo capture

**New Components:** ArduCam Mega 3MP (OV5642)  
**Learn:** SPI camera interfacing, image capture, payload operations

---

### **Step 4: SD Card Integration** (`step-04-sd/`)
**Data Storage and Mission Data Logging**
- Micro SD card module for comprehensive data logging
- Multi-sensor data capture (housekeeping, 9-axis, GPS)
- Multi-cycle data collection with timestamped log files
- FAT32 filesystem support for easy data retrieval

**New Components:** Micro SD Card module, Micro SD card (up to 32GB)  
**Learn:** Filesystem operations, data logging, long-term storage

---

### **Step 5: Photo-to-SD Automatic Transfer** (`step-05-photo-move.py/`)
**Intelligent File Management**
- Automatic image transfer to SD card after capture
- Flash memory freed immediately after each photo
- Preserved auto-incremented filenames
- Centralized storage for all mission data

**New Features:** Enhanced camera workflow, automated file management  
**Learn:** Storage optimization, mission operations, file I/O

---

## 🛠️ What's Provided for Each Step

Every step directory includes:

- ✅ **Complete README.md** - Detailed documentation with theory and operation
- ✅ **Circuit Schematic** - Full electrical connections (`*_schematic.png`)
- ✅ **Breadboard Layout** - Physical wiring guide (`*_breadboard.png`)
- ✅ **Bill of Materials** - Complete parts list with specifications (`*_bom.html`)
- ✅ **Fritzing File** - Editable circuit diagram (`*.fzz`)
- ✅ **MicroPython Code** - Modular manager scripts for each subsystem

### Fritzing Software

Circuit diagrams were created using [**Fritzing**](https://fritzing.org/), an open-source electronics design tool.

📥 **Download Fritzing:** [fritzing.org/download](https://fritzing.org/download/)

You can view, edit, or customize the `.fzz` files to adapt the designs to your needs.

---

## 📚 Required Libraries

All sensor and breakout board libraries are maintained in a separate repository for modularity and reusability.

🔗 **MicroPython Libraries:** [github.com/OjasJha/micropython-lib](https://github.com/OjasJha/micropython-lib.git)

**To get started:**
```bash
git clone https://github.com/OjasJha/micropython-lib.git
```

**Copy required libraries** (listed in each step's README) to your Raspberry Pi Pico in a flat structure alongside the VyomSat manager modules.

---

## 🚦 Quick Start Guide

### 1. **Hardware Setup**
- Gather components from the Bill of Materials (BOM) for your chosen step
- Follow the breadboard layout diagram for wiring
- Double-check connections against the schematic

### 2. **Software Setup**
- Install [Thonny IDE](https://thonny.org/) for MicroPython development
- Install MicroPython firmware on your Raspberry Pi Pico 2 or Pico W
- Download required libraries from the [micropython-lib repository](https://github.com/OjasJha/micropython-lib.git)

### 3. **Upload Code**
- Copy library files (e.g., `uart_handler.py`, `ssd1306_handler.py`) to Pico
- Copy VyomSat manager modules (e.g., `vyomsat_battery_manager.py`) to Pico
- Copy main script (`vyomsat.py`) to Pico

### 4. **Run and Test**
- Open Thonny, connect to your Pico
- Run `vyomsat.py` or rename it to `main.py` for auto-start on boot
- Monitor USB serial console (115200 baud) for debug output
- Test commands via XBee UART interface

### 5. **Build Progressively**
- Start with Step 0 to establish the foundation
- Add one subsystem at a time (Steps 1 → 2 → 3 → 4 → 5)
- Test thoroughly at each step before proceeding
- Refer to individual README files for detailed instructions

---

## 📊 Communication Interfaces Summary

By Step 5, your CubeSat prototype will use:

| Interface | Pins | Speed | Purpose |
|-----------|------|-------|---------|
| **SPI0** | GP16/17/18/19 | 8 MHz | ArduCam Mega 3MP Camera |
| **SPI1** | GP10/11/12/13 | 1 MHz | Micro SD Card Module |
| **UART0** | GP0/GP1 | 9600 bps | XBee Communication |
| **UART1** | GP4/GP5 | 9600 bps | NEO-6M GPS Module |
| **I2C0** | GP6/GP7 | 400 kHz | SSD1306 OLED Display |
| **I2C1** | GP20/GP21 | 400 kHz | MPU9250 9-Axis IMU |
| **ADC** | GP28 | - | Battery Voltage Monitoring |

---

## 🎓 Learning Outcomes

By completing this project, you will:

### Technical Skills
- ✅ Understand **CubeSat subsystems**: EPS, COM, ADCS, GPS, Camera, Data Storage
- ✅ Master **communication protocols**: I2C, SPI, UART, ADC
- ✅ Learn **sensor integration**: IMU, GPS, camera interfacing
- ✅ Practice **embedded programming**: MicroPython on Raspberry Pi Pico
- ✅ Develop **systems thinking**: integrating multiple subsystems
- ✅ Handle **real-world constraints**: power, memory, timing, bus contention

### Engineering Practices
- ✅ **Modular design**: Each subsystem isolated and testable
- ✅ **Progressive integration**: Build complexity incrementally
- ✅ **Error handling**: Graceful degradation when components fail
- ✅ **Documentation**: Reading schematics, datasheets, and technical docs
- ✅ **Troubleshooting**: Debugging hardware and software issues

### Space Technology Knowledge
- ✅ How **satellites monitor power** (battery voltage sensing)
- ✅ How **satellites determine attitude** (IMU sensors and fusion algorithms)
- ✅ How **satellites know their position** (GPS and NMEA parsing)
- ✅ How **satellites capture images** (camera payloads)
- ✅ How **satellites store data** (onboard storage and file management)
- ✅ How **satellites communicate** (telemetry and command interfaces)

### What You'll Have Built
A **working CubeSat prototype** that:
- 🔋 Monitors battery voltage
- 📡 Communicates wirelessly via XBee
- 📺 Displays telemetry on OLED screen
- 🧭 Tracks orientation and motion
- 🛰️ Determines GPS position
- 📷 Captures high-resolution images
- 💾 Logs all sensor data to SD card
- ⚡ Manages storage efficiently

---

## 🧑‍💻 Supported UART Commands

Once fully built (Step 5), your CubeSat responds to these commands via XBee:

| Command | Function | Introduced In |
|---------|----------|---------------|
| **`v`** | Battery voltage reading | Step 0 |
| **`a`** | 9-axis IMU attitude data | Step 1 |
| **`g`** | GPS position and diagnostics | Step 2 |
| **`c`** | Camera capture + SD transfer | Step 3 (enhanced in Step 5) |
| **`s`** | Multi-sensor data logging to SD | Step 4 |

---

## 🔧 Built With Off-the-Shelf Components

All components are readily available from electronics suppliers like Adafruit, SparkFun, Amazon, or local vendors:

- **Raspberry Pi Pico 2** or **Pico W** (microcontroller)
- **SSD1306 OLED Display** (128×64, I2C)
- **MPU9250 9-Axis IMU** (GY-9250 module)
- **NEO-6M GPS Module** (GY-GPS6MV2)
- **ArduCam Mega 3MP Camera** (OV5642, SPI)
- **Micro SD Card Module** (SPI)
- **XBee Module** (or compatible UART device)
- Breadboard, jumper wires, resistors, batteries

**Total cost:** Approximately $80-$150 USD depending on component sources.

---

## 📁 Project Structure

```
vyomsat/
│
├── README.md                          # This file - repository overview
├── LICENSE                            # MIT License
│
├── step-00-eps-com/                   # Step 0: Power & Communication
│   ├── README.md                      # Detailed step documentation
│   ├── vyomsat.py                     # Main integration script
│   ├── vyomsat_battery_manager.py     # Battery voltage monitoring
│   ├── vyomsat_oled_manager.py        # OLED display handler
│   ├── vyomsat_xbee_manager.py        # XBee reset handler
│   ├── eps-com_schematic.png          # Circuit schematic
│   ├── eps-com_breadboard.png         # Breadboard layout
│   ├── eps-com_bom.html               # Bill of materials
│   └── eps-com.fzz                    # Fritzing design file
│
├── step-01-9axis/                     # Step 1: 9-Axis IMU
│   ├── README.md
│   ├── vyomsat.py
│   ├── vyomsat_9axis_manager.py       # NEW: IMU handler
│   ├── [other managers]
│   ├── 9axis_schematic.png
│   ├── 9axis_breadboard.png
│   ├── 9axis_bom.html
│   └── 9axis.fzz
│
├── step-02-gps/                       # Step 2: GPS Integration
│   ├── README.md
│   ├── vyomsat.py
│   ├── vyomsat_gps_manager.py         # NEW: GPS handler
│   ├── [other managers]
│   ├── gps_schematic.png
│   ├── gps_breadboard.png
│   ├── gps_bom.html
│   └── gps.fzz
│
├── step-03-camera/                    # Step 3: Camera Integration
│   ├── README.md
│   ├── vyomsat.py
│   ├── vyomsat_camera_manager.py      # NEW: Camera handler
│   ├── [other managers]
│   ├── camera_schematic.png
│   ├── camera_breadboard.png
│   ├── camera_bom.html
│   └── camera.fzz
│
├── step-04-sd/                        # Step 4: SD Card Logging
│   ├── README.md
│   ├── vyomsat.py
│   ├── vyomsat_sd_manager.py          # NEW: SD card handler
│   ├── [other managers]
│   ├── sd_schematic.png
│   ├── sd_breadboard.png
│   ├── sd_bom.html
│   └── sd.fzz
│
└── step-05-photo-move.py/             # Step 5: Auto Photo-to-SD
    ├── README.md
    ├── vyomsat.py                     # Enhanced with auto-transfer
    ├── vyomsat_sd_manager.py          # Enhanced with move function
    ├── [other managers]
    ├── photo-move_schematic.png
    ├── photo-move_breadboard.png
    ├── photo-move_bom.html
    └── photo-move.fzz
```

---

## 🤝 Contributing

This is an educational project developed to share knowledge about space technology. Contributions, suggestions, and improvements are welcome!

**Ways to contribute:**
- Report issues or bugs
- Suggest new features or subsystems
- Improve documentation
- Share your build photos and experiences
- Adapt designs for different hardware platforms

---

## 📜 License

**MIT License** - Copyright (c) 2025 Ojas Jha

See [LICENSE](LICENSE) file for full license text.

This project is open-source and free to use for educational purposes. You may use, modify, and distribute this project with attribution.

---

## 👨‍🚀 Author

**Ojas Jha**  
VyomSat CubeSat Education Kit  
Date: October 25, 2025

Passionate about making space technology accessible to learners worldwide. 🌍🚀

---

## 🙏 Acknowledgments

- The maker and open-source hardware communities for inspiration
- [Fritzing](https://fritzing.org/) for circuit design software
- [MicroPython](https://micropython.org/) for embedded Python support
- The CubeSat community for demonstrating that space is accessible
- High school STEM educators pushing for deeper learning experiences

---

## 🌟 Get Started Today!

Ready to build your own CubeSat prototype?

1. ⭐ **Star this repository** to bookmark it
2. 📥 **Clone or download** the repository
3. 🛒 **Order components** from the Step 0 BOM
4. 📖 **Read** `step-00-eps-com/README.md`
5. 🔧 **Start building!**

**Questions?** Open an issue or discussion. **Built something cool?** Share your results!

---

**VyomSat - Because space is for everyone. 🌌✨**

*"The essence of space, built by you."*
