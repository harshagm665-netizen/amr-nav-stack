# 🚀 Autonomous Mobile Robot (AMR) Navigation Stack

[![Python](https://img.shields.io/badge/Python-3.8%2B-blue.svg)](https://www.python.org/)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/index.html)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

<p align="center">
  <img src="demo.gif" alt="AMR Navigation Demo">
</p>

> **Core hardware interface and base controller for an Autonomous Mobile Robot.** This repository contains the essential scripts, parameters, and bridges required to interface high-level ROS2 navigation nodes with low-level ESP32 motor controllers and LiDAR sensors.

---

## 📖 Table of Contents

- [Overview](#-overview)
- [Key Components](#-key-components)
- [Hardware Architecture](#-hardware-architecture)
- [Getting Started](#-getting-started)
- [Configuration](#-configuration)

## 🌟 Overview

This project provides a robust, multi-threaded hardware abstraction layer for an AMR. It seamlessly translates ROS2 `cmd_vel` Twist messages into motor PWM commands while continuously reading high-speed telemetry (wheel encoder ticks) to compute accurate differential drive odometry.

## ✨ Key Components

- **`kali_base.py` & `kali_base_node.py`**: The core ROS2 nodes. They subscribe to `/cmd_vel`, publish to `/odom`, and broadcast TF transforms (`odom` → `base_footprint`). They employ dedicated Python threads for highly reliable serial telemetry reception.
- **`robot_bridge.py`**: A standalone, non-ROS hardware bridge to independently test and verify the binary packet protocol to the ESP32.
- **`test_lider.py`**: A lightweight script to verify LiDAR hardware integration before spinning up the full SLAM stack.
- **`start_monk.sh`**: A simple entrypoint script to launch the base controller.
- **`get-docker.sh`**: Official Docker installation script for setting up containerized robot environments.
- **`nav2_params.yaml`**: Pre-tuned parameters for the Nav2 navigation stack (costmaps, velocity limits, etc.).

## 🏗️ Hardware Architecture

- **Compute**: Linux SBC (e.g., Raspberry Pi) running ROS2.
- **Microcontroller**: ESP32 connected via USB (`/dev/ttyUSB1`).
- **Communication Protocol**: Custom 6-byte/8-byte binary packet protocol over 115200 baud serial, protected by XOR checksums.
- **Kinematics**: Differential drive (Track width: 0.116m, Wheel radius: 0.0335m).

## 🚀 Getting Started

1. **Install Dependencies**:
   Ensure you have ROS2 installed. For Docker setups, you can run `./get-docker.sh`.
   You will also need `pyserial`:
   ```bash
   pip3 install pyserial
   ```

2. **Hardware Connection**:
   Connect your ESP32 to the SBC. Ensure it mounts at `/dev/ttyUSB1` (or update the `SERIAL_PORT` variable in the code).

3. **Launch the Controller**:
   ```bash
   python3 kali_base.py
   # Or using the launcher script:
   ./start_monk.sh
   ```

## ⚙️ Configuration

Kinematics and serial settings can be tuned directly at the top of the node files. For advanced autonomous navigation tuning, modify `nav2_params.yaml`.

---
*Built with ❤️ for advanced robotic exploration.*
