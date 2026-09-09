# 🤖 AMR Navigation Stack — ROS2 + Raspberry Pi 5 + ESP32

> Full autonomous navigation stack for a differential-drive mobile robot. SLAM mapping, Nav2 path planning, and custom embedded control — running on real hardware.

![Demo — autonomous navigation](demo_v2.gif)

[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-22314E?style=flat-square&logo=ros)](https://docs.ros.org/en/jazzy/)
[![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi%205-A22846?style=flat-square&logo=raspberrypi)](https://www.raspberrypi.com/)
[![Docker](https://img.shields.io/badge/Docker-Ready-2496ED?style=flat-square&logo=docker)](https://www.docker.com/)
[![License](https://img.shields.io/badge/License-MIT-blue?style=flat-square)]()

---

## 📋 Overview

This project implements a **production-grade autonomous navigation pipeline** on a differential-drive mobile robot built with a Raspberry Pi 5 and ESP32 microcontroller. The system covers the complete autonomy stack:

```
RPLiDAR A1M8 → SLAM Mapping → Nav2 Planning → Motor Control → ESP32 Actuators
```

### Key Features
- 🗺️ **Real-time SLAM** mapping with `slam_toolbox`
- 🧭 **Full Nav2 stack**: AMCL localization → NavFn global planner → DWB local planner
- ⚡ **Custom hardware interface**: Multi-threaded ROS2 node translating `/cmd_vel` into ESP32 motor commands via binary UART protocol with XOR checksum validation
- 📐 **Differential-drive odometry**: Encoder-based dead reckoning publishing `/odom` and TF transforms
- 🛡️ **Collision monitoring**: Layered costmaps (static + inflation + obstacle layers)
- 🐳 **Docker containerized** for reproducible deployment

---

## 🏗️ System Architecture

```mermaid
%%{init: {'theme': 'dark', 'themeVariables': { 'fontSize': '14px', 'fontFamily': 'Inter, ui-sans-serif, system-ui, sans-serif'}}}%%
flowchart TB
    %% ==========================================
    %% Premium Class Definitions (Dark Mode)
    %% ==========================================
    classDef cockpit fill:#4c1d95,stroke:#8b5cf6,stroke-width:2px,color:#f5f3ff
    classDef rosnode fill:#1e3a8a,stroke:#3b82f6,stroke-width:1.5px,color:#dbeafe
    classDef topic fill:#134e4a,stroke:#2dd4bf,stroke-width:2px,color:#ccfbf1
    classDef hardware fill:#1f2937,stroke:#6b7280,stroke-width:1.5px,color:#e5e7eb

    %% ==========================================
    %% Environments
    %% ==========================================
    subgraph Cockpit ["🖥️  External Cockpit "]
        Studio["Foxglove Studio"]:::cockpit
    end

    subgraph Pi ["🚀  Raspberry Pi 5 — ROS 2 Jazzy"]
        %% Comms
        Bridge["foxglove_bridge<br/>(WebSocket)"]:::rosnode
        
        %% Localization
        MapSrv["Map Server<br/>(monk_room_map)"]:::rosnode
        AMCL["AMCL<br/>Localization"]:::rosnode
        
        %% Navigation
        GCost["Global Costmap<br/>(Static)"]:::rosnode
        LCost["Local Costmap<br/>(Dynamic)"]:::rosnode
        Planner["Planner Server<br/>(NavFn)"]:::rosnode
        Ctrl["Controller Server<br/>(DWB)"]:::rosnode
        CollMon["Collision Monitor<br/>(StopBox)"]:::rosnode
        
        %% Hardware Interface
        Base["kali_base.py<br/>(Motors & Odom)"]:::rosnode
        LidarDrv["rplidar_ros<br/>(LiDAR)"]:::rosnode
        
        %% Data Bus (Topics)
        T_Map(["/map"]):::topic
        T_Scan(["/scan"]):::topic
        T_Plan(["/plan"]):::topic
        T_Smooth(["/cmd_vel_smoothed"]):::topic
        T_Cmd(["/cmd_vel"]):::topic
    end

    subgraph HW ["⚡  Physical Hardware "]
        RPLIDAR["RPLiDAR A1M8"]:::hardware
        ESP32["ESP32<br/>(50 Hz PID)"]:::hardware
        Motors["Wheel Motors<br/>& Encoders"]:::hardware
    end

    %% ==========================================
    %% Pipelines & Connections
    %% ==========================================

    %% External Link
    Studio <-->|"ws://"| Bridge

    %% Sensor Pipeline
    RPLIDAR -->|"USB"| LidarDrv
    LidarDrv --> T_Scan
    T_Scan --> AMCL
    T_Scan --> LCost
    T_Scan --> CollMon

    %% Map Pipeline
    MapSrv --> T_Map
    T_Map --> AMCL
    T_Map --> GCost

    %% TF Tree (Dashed Gold for Coordinate Transforms)
    Base -.->|"/tf: odom → base"| AMCL
    AMCL -.->|"/tf: map → odom"| GCost
    AMCL -.->|"/tf: map → odom"| LCost

    %% Planning & Control Pipeline
    GCost --> Planner
    Planner --> T_Plan
    T_Plan --> Ctrl
    
    LCost --> Ctrl
    Ctrl --> T_Smooth
    T_Smooth --> CollMon
    CollMon --> T_Cmd
    T_Cmd --> Base

    %% Actuation Pipeline
    Base -->|"UART"| ESP32
    ESP32 -->|"PWM"| Motors
    Motors -->|"Encoders"| ESP32
    ESP32 -->|"Odom"| Base

    %% ==========================================
    %% Premium Subgraph Styling
    %% ==========================================
    style Cockpit fill:#1e1b4b,stroke:#4c1d95,stroke-width:2px,color:#c4b5fd
    style Pi fill:#0f172a,stroke:#1e3a8a,stroke-width:2px,color:#93c5fd
    style HW fill:#111827,stroke:#374151,stroke-width:2px,color:#9ca3af
```

### TF Transform Tree
```
map → odom → base_footprint → base_link → laser_frame
```

- `map → odom` — published by **AMCL** (localization corrects drift).
- `odom → base_footprint` — published by **kali_base.py** from encoder ticks.
- `base_footprint → base_link` — static (robot body / URDF).
- `base_link → laser_frame` — static (LiDAR mounting).

> 🎨 Prefer an interactive view? Open the [architecture diagram](docs/architecture.html) — zoom, pan, and download the SVG. There's also a [static SVG](docs/architecture.svg).

---

## 🔧 Hardware Specifications

| Component | Specification |
|:----------|:-------------|
| **Compute** | Raspberry Pi 5 (16GB RAM) |
| **Microcontroller** | ESP32 (motor control + encoder reading) |
| **LiDAR** | RPLiDAR A1M8 (360° scan, 12m range) |
| **Drive** | Differential-drive (track width: 0.116m, wheel radius: 0.0335m) |
| **Communication** | Custom binary UART protocol with XOR checksum validation |

---

## 🚀 Quick Start

### Prerequisites
- Ubuntu 22.04 (ARM64 for RPi5)
- ROS2 Jazzy
- Docker (optional)

### Launch with Docker
```bash
docker compose up
```

### Launch manually
```bash
# One-command bring-up (hardware bridge + Nav2)
./start_monk.sh path/to/map.yaml

# Or run the pieces separately:
# Terminal 1: hardware bridge + wheel odometry
python3 kali_base.py

# Terminal 2: Nav2 (AMCL + navigation) with saved map
ros2 launch nav2_bringup bringup_launch.py map:=./monk_room_map.yaml params_file:=./nav2_params.yaml
```
> **Note:** the saved map (`monk_room_map.yaml`) is generated on the robot via `slam_toolbox` and is not committed to this repo — re-run SLAM mapping to produce it before navigation.

---

## 🐛 Notable Debugging Win

Diagnosed and resolved a **cascading Nav2 lifecycle bringup failure** affecting all 9 managed nodes. Root cause chain:
1. Misconfigured docking server parameters
2. Missing AMCL initial pose publication
3. Incorrect `base_frame` in behavior-server config
4. Busy-wait loop starving the controller server's executor

Resolution required deep understanding of ROS2 lifecycle node management and the Nav2 managed node activation sequence.

---

## 📊 Results

- ✅ Successful real-time SLAM mapping of indoor environment
- ✅ Autonomous point-to-point navigation with obstacle avoidance
- ✅ Validated on physical hardware using Foxglove Studio
- ✅ Clean lifecycle activation across all 9 Nav2 managed nodes

---

## 📁 Repository Structure

```
amr-nav-stack/
├── kali_base.py          # Custom ROS 2 node: /cmd_vel → UART (inverse kinematics),
│                         #   encoder ticks → /odom + TF (20 Hz odometry)
├── robot_bridge.py       # Pre-ROS UART test harness (protocol validation)
├── test_lider.py         # RPLiDAR A1M8 smoke test
├── nav2_params.yaml      # Full Nav2 config: AMCL, BT navigator, DWB, costmaps,
│                         #   velocity smoother, collision monitor
├── start_monk.sh         # One-command bring-up (hardware bridge + Nav2)
├── firmware/
│   └── esp32_firmware/   # ESP32 FreeRTOS firmware: 50 Hz PI control + 150 ms watchdog
├── docs/                 # Architecture diagrams (SVG)
└── demo_v2.gif           # Demo recording
```

---

## 🛣️ Roadmap

- [ ] Add IMU fusion via `robot_localization` EKF
- [ ] Implement multi-waypoint navigation
- [ ] Add simulated Gazebo environment for testing
- [ ] CI/CD with GitHub Actions

---

## 📜 License

MIT License — see [LICENSE](LICENSE) for details.
