<div align="center">

# 🚀 Autonomous Mobile Robot (AMR) Navigation Stack
**State-of-the-Art ROS2 Navigation & Hardware Abstraction for Differential Drive AMRs**

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg?logo=ros)](https://docs.ros.org/en/humble/index.html)
[![Python](https://img.shields.io/badge/Python-3.8%2B-blue.svg?logo=python)](https://www.python.org/)
[![Hardware](https://img.shields.io/badge/Hardware-ESP32%20%7C%20Raspberry%20Pi-green.svg)](#-hardware-architecture)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

<img src="demo_v2.gif" alt="AMR Navigation Demo" width="700">

*Core hardware interface, robust telemetry threads, and Nav2 configurations for an Autonomous Mobile Robot.*

</div>

---

## 📖 Table of Contents
- [Overview](#-overview)
- [System Architecture](#-system-architecture)
- [Core Components](#-core-components)
- [Communication Protocol](#-communication-protocol)
- [Challenges & Engineering Solutions](#-challenges--engineering-solutions)
- [Getting Started](#-getting-started)
- [Configuration Details](#-configuration-details)

---

## 🌟 Overview

This repository houses a production-grade ROS 2 hardware abstraction layer (HAL) and navigation stack configuration for a differential drive Autonomous Mobile Robot (AMR). The system bridges high-level AI/Path-Planning (Nav2) running on a Linux SBC with low-level real-time motor control executed by an ESP32 microcontroller. 

It handles everything from translating `cmd_vel` Twist messages into raw motor PWMs to processing high-frequency wheel encoder telemetry into accurate `/odom` topics and TF transforms—all while strictly validating serial data integrity.

---

## 🏗️ System Architecture

- **High-Level Compute**: Linux SBC (e.g., Raspberry Pi 5) running ROS 2 Humble. Handles SLAM, Nav2 behavior trees, and the Python-based ROS nodes.
- **Low-Level Compute**: ESP32 connected via USB (`/dev/ttyUSB1`) running custom C++ firmware on FreeRTOS. Handles real-time motor PID control loops (50Hz), PWM generation, and precise hardware encoder pulse counting via PCNT.
- **Perception**: RPLiDAR for 2D laser scans (`/scan`), used by AMCL for localization and Nav2 for obstacle avoidance.
- **Kinematics**: Differential drive setup.
  - Track width (Wheel Base): `0.116 m`
  - Wheel Radius: `0.0335 m`
  - Encoder Resolution: `400 ticks/rev`
  - Max Speed: `0.21 m/s`

---

## ✨ Core Components

### ROS 2 Interface
- **`kali_base.py` / `kali_base_node.py`**: The heart of the stack. These nodes translate incoming `/cmd_vel` geometry messages into left/right wheel velocities. They also maintain a dedicated background daemon thread to continuously read incoming serial telemetry, calculate Euler-based odometry integration at 20Hz, and broadcast `/odom` and `odom -> base_footprint` TF frames.

### Embedded Firmware
- **`esp32_firmware.ino`**: Multi-threaded FreeRTOS C++ firmware running on the ESP32. It utilizes ESP32's dedicated PCNT (Pulse Counter) hardware for zero-overhead encoder tracking. A strict 50Hz control loop runs continuously on Core 1 to compute PID outputs, while UART parsing and Watchdog timers ensure the robot stops safely if connection to the ROS 2 host is lost.

### Navigation & Mapping
- **`nav2_params.yaml`**: Highly-tuned Nav2 parameters tailored for this specific AMR. Features DWB Local Planner configuration, AMCL particle filter settings (500-2000 particles), dual costmaps (Global & Local) with inflation layers (0.22m radius), and a strict Collision Monitor with a virtual "StopBox" polygon.
- **`monk_room_map.yaml`**: Pre-generated SLAM map configuration files for immediate deployment.

### Diagnostics & Tooling
- **`robot_bridge.py`**: A standalone, zero-dependency Python script to directly test the ESP32 serial protocol without needing the ROS 2 daemon. Essential for hardware debugging.
- **`test_lider.py`**: A lightweight diagnostic tool leveraging the `rplidar` library to verify LiDAR health and scan frequency before launching the heavy ROS 2 stack.
- **`start_monk.sh` & `get-docker.sh`**: Helper scripts for launching the controller and provisioning containerized robotic environments seamlessly.

---

## 🔌 Communication Protocol

To ensure 100% reliable data transmission between the Linux SBC and the ESP32, a custom XOR-checksummed binary packet protocol over UART (115200 baud) was designed:

**Tx (Command) Packet (6 Bytes):**
`[ 0x02 (Start) | Left PWM | Right PWM | Unused (0x00) | Checksum | 0x03 (End) ]`

**Rx (Telemetry) Packet (8 Bytes):**
`[ 0x02 (Start) | L_Ticks_Low | L_Ticks_High | R_Ticks_Low | R_Ticks_High | Unused | Checksum | 0x03 (End) ]`

*Checksum logic: `(Byte 1 ^ Byte 2)` ensuring corrupted data packets are dropped instantly without causing jerky robot motions.*

---

## 🛠️ Challenges & Engineering Solutions

1. **Blocking Serial Reads in ROS 2:**
   - *Issue*: Standard serial reads inside a ROS timer callback would block the ROS 2 event loop, leading to dropped `cmd_vel` messages and terrible latency.
   - *Fix*: Implemented Python `threading.Thread(target=self.receive_telemetry, daemon=True)`. This isolates the high-speed UART parsing into a separate thread, safely sharing the parsed tick data with the 20Hz ROS 2 odometry publisher loop.

2. **UART Data Corruption & Odometry Jumps:**
   - *Issue*: EMI from the motors occasionally flipped bits on the USB serial line, causing massive sudden spikes in encoder tick readings which destroyed the robot's localization.
   - *Fix*: Designed the lightweight XOR binary checksum protocol. The Python bridge silently discards any packet where `calc_checksum != checksum`, completely eliminating odometry jumps.

3. **Documentation Rendering Issues (GitHub):**
   - *Issue*: The `demo.gif` showcase was 7.5MB, causing GitHub's Camo proxy to frequently timeout and display a broken image link for visitors.
   - *Fix*: Wrote a custom compression script to halve the framerate and reduce the color palette to 64 colors, shrinking the GIF by 85% to 1.0MB and adding explicit `width="700"` tags for robust rendering.

4. **Real-time Motor Control on ESP32:**
   - *Issue*: Handling high-frequency encoder interrupts on the microcontroller while parsing incoming serial commands and computing PID loops often leads to missed pulses or jittery movement.
   - *Fix*: Built the ESP32 firmware using a FreeRTOS multi-threaded architecture. Offloaded encoder counting entirely to the ESP32's hardware PCNT peripheral (zero CPU usage). Pinned a strict 50Hz PID control loop to Core 1, and protected shared command variables with FreeRTOS Mutexes (`xSemaphoreTake`), resulting in silky smooth motor operation and rock-solid reliability.

---

## 🚀 Getting Started

### Prerequisites
- ROS 2 (Humble/Jazzy) installed on Ubuntu 22.04/24.04.
- Python 3.8+ with `pyserial` and `rplidar-robotic`.
```bash
pip3 install pyserial rplidar-robotic
```

### Hardware Setup
1. Mount the ESP32 and connect via USB. Ensure it appears at `/dev/ttyUSB1` (or update `SERIAL_PORT` in `kali_base.py`).
2. Give your user serial permissions: `sudo usermod -aG dialout $USER`.
3. Connect the RPLiDAR to `/dev/ttyUSB0`.

### Running the Stack

**1. Hardware Diagnostics (Optional)**
Verify serial and LiDAR before starting ROS:
```bash
python3 robot_bridge.py
python3 test_lider.py
```

**2. Start the ROS 2 Base Controller**
```bash
./start_monk.sh
# OR manually:
python3 kali_base_node.py
```

---

<div align="center">
<i>Built with ❤️ for robust, autonomous robotic exploration.</i>
</div>
