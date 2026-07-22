# ✨ AeroKinesis: Dual-IMU Hand Gesture Drone Controller
### 🛰️ Next-Gen Wearable Spatial Interface for ROS2 Humble & ArduPilot SITL / Gazebo Sim

![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-34495E?style=for-the-badge&logo=ros&logoColor=white)
![Arduino ESP32](https://img.shields.io/badge/ESP32-Dual_MPU9250-00979D?style=for-the-badge&logo=arduino&logoColor=white)
![ArduPilot SITL](https://img.shields.io/badge/ArduPilot-SITL_Copter-E74C3C?style=for-the-badge&logo=drone&logoColor=white)
![Gazebo Sim](https://img.shields.io/badge/Gazebo-Iris_Runway-FF6F00?style=for-the-badge&logo=gazebo&logoColor=white)
![License](https://img.shields.io/badge/License-Apache_2.0-2ECC71?style=for-the-badge)
![Version](https://img.shields.io/badge/Release-v3.0_Dual_Hand-9B59B6?style=for-the-badge)

---

## 🖼️ Hardware Showcase & Visual Gallery

| 🎛️ ESP32 Core Controller Board | 🎯 Right Hand Primary IMU (0x68) |
|:---:|:---:|
| ![ESP32 DevKit V1 Board](https://glistening-lebkuchen-c76833.netlify.app/Eap32.jpeg) | ![Right Hand MPU Sensor](https://glistening-lebkuchen-c76833.netlify.app/Right_hand_MPU.jpeg) |

| 🔘 Tactile 4-Button Array | ✋ Left Hand Secondary IMU (0x69) |
|:---:|:---:|
| ![Right Hand Button Interface](https://glistening-lebkuchen-c76833.netlify.app/Right_Hand_Buttons.jpeg) | ![Left Hand MPU Sensor](https://glistening-lebkuchen-c76833.netlify.app/Left_Hand.jpeg) |

---

## 🎥 Demonstration Videos

### 🌟 AeroKinesis v3.0 Dual-IMU Bimanual Flight Demo
![AeroKinesis V3.0 Dual-IMU Flight Demo](file:///home/user/ros2_ws/src/imu_serial_bridge/media/v3.mp4)

<video src="https://glistening-lebkuchen-c76833.netlify.app/v3.mp4" controls width="100%" poster="media/Right_hand_MPU.jpeg">
  Your browser does not support the video tag. <a href="https://glistening-lebkuchen-c76833.netlify.app/v3.mp4">Download v3.mp4</a>
</video>

---

> [!IMPORTANT]
> **AeroKinesis v3.0** features **Onboard Sensor Fusion (Complementary Filter α=0.98)** and **Automatic Neutral Pose Calibration** directly on the ESP32. Upon powering on, keep both hands flat and still for **3 seconds** while the system auto-zeros offsets.

---

## 📑 Table of Contents

1. [⚡ Quick Installation & Setup](#1--quick-installation--setup)
2. [🚀 Complete System Startup Guide](#2--complete-system-startup-guide)
3. [🔌 Hardware Wiring & Pinouts](#3--hardware-wiring--pinouts)
4. [🎮 Gesture Control & Axis Mapping](#4--gesture-control--axis-mapping)
5. [🧠 Firmware & Software Architecture](#5--firmware--software-architecture)
6. [📊 ROS2 Topics & Node Parameters](#6--ros2-topics--node-parameters)
7. [🖥️ Real-Time Terminal Dashboard](#7--real-time-terminal-dashboard)
8. [📁 Repository Structure](#8--repository-structure)
9. [📜 Version History & Evolution](#9--version-history--evolution)
10. [🐞 Troubleshooting](#10--troubleshooting)

---

## 1. ⚡ Quick Installation & Setup

### Step 1 — Clone the Repository into ROS2 Workspace
```bash
cd ~/ros2_ws/src
git clone https://github.com/your-username/imu_serial_bridge.git
cd imu_serial_bridge
```

### Step 2 — Serial Port Permission *(One-Time Setup)*
```bash
sudo usermod -aG dialout $USER
newgrp dialout
```

### Step 3 — Install Python Dependencies
```bash
pip3 install pyserial pymavlink
```

### Step 4 — Build the ROS2 Package
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select imu_serial_bridge --symlink-install
source install/setup.bash
```

---

## 2. 🚀 Complete System Startup Guide

> [!TIP]
> For the complete multi-terminal startup guide, see **[`startup.md`](file:///home/user/ros2_ws/src/imu_serial_bridge/startup.md)** or **[`statup.md`](file:///home/user/ros2_ws/src/imu_serial_bridge/statup.md)**.

Open **3 separate terminal windows** and execute the commands below:

```bash
# 🖥️ Terminal 1: Launch ArduPilot SITL Copter Simulation
sim_vehicle.py -v ArduCopter -f gazebo-iris --console --model JSON --map --out=127.0.0.1:14550

# 🌌 Terminal 2: Launch Gazebo 3D Simulation Environment
gz sim -v4 -r iris_runway.sdf

# ⚡ Terminal 3: Launch ROS2 Gesture Bridge Node
source ~/ros2_ws/install/setup.bash
ros2 launch imu_serial_bridge imu_serial.launch.py serial_port:=/dev/ttyUSB0
```

---

## 3. 🔌 Hardware Wiring & Pinouts

### 💎 Glassmorphism Wiring Matrix

```
ESP32 DevKit V1              IMU 1 (Right Hand)           IMU 2 (Left Hand)
┌───────────────┐           ┌──────────────────┐         ┌──────────────────┐
│  3.3V         ├──────────►│ VCC              │         │ VCC              │
│  GND          ├──────────►│ GND              │         │ GND              │
│  GPIO 21 (SDA)├──────────►│ SDA              │         │ SDA              │
│  GPIO 22 (SCL)├──────────►│ SCL              │         │ SCL              │
│  GND          ├──────────►│ AD0 (Addr 0x68)  │         │                  │
│  3.3V         ├───────────┴──────────────────┼────────►│ AD0 (Addr 0x69)  │
└───────────────┘                              └─────────┴──────────────────┘

Push Buttons (Internal Pull-Up -> Active LOW):
┌─────────────────────────┬──────────────┬──────────────────────────────────┐
│ Function                │ ESP32 GPIO   │ Trigger Action                   │
├─────────────────────────┼──────────────┼──────────────────────────────────┤
│ 🟢 ARM / DISARM          │ GPIO 32      │ Toggles Motor Arming             │
│ 🔄 STABILIZE / GUIDED   │ GPIO 33      │ Flight Mode Switch               │
│ 🛫 TAKEOFF (2.0m)       │ GPIO 25      │ Vertical Climb (Threaded)        │
│ 🏠 RTL (Return-Home)    │ GPIO 26      │ Auto Return-to-Launch & Land     │
└─────────────────────────┴──────────────┴──────────────────────────────────┘
```

> [!WARNING]
> Both MPU9250/6500 IMUs are **3.3V devices**. Do **NOT** connect VCC to 5V.

---

## 4. 🎮 Gesture Control & Axis Mapping

AeroKinesis maps bimanual (two-handed) spatial gestures directly into 4-axis MAVLink `LOCAL_NED` velocity commands:

| Hand / Sensor | Spatial Gesture | Flight Axis | Velocity Variable | Sensitivity |
|---|---|---|---|---|
| 🫱 **Right Hand (IMU 1)** | Pitch Forward / Backward | Forward / Backward | `vx` (m/s) | **1.2x Multiplier** |
| 🫱 **Right Hand (IMU 1)** | Roll Right / Left | Right / Left | `vy` (m/s) | **1.2x Multiplier** |
| 🫲 **Left Hand (IMU 2)** | Pitch Forward / Backward | Climb / Descend | `vz` (m/s) | 1.0x Normal |
| 🫲 **Left Hand (IMU 2)** | Roll Twist CW / CCW | Yaw Turn | `yaw_rate` (rad/s) | 1.0x Normal |
| 🤲 **Either Hand** | Flat (Tilt < 5°) | Hover / Hold | `0.0` | Dead-zone applied |

---

## 5. 🧠 Firmware & Software Architecture

```
                       ┌──────────────────────────────────────┐
                       │  ESP32 Wearable Controller Hardware  │
                       │  (Dual MPU9250/6500 + 4 Push Buttons) │
                       └──────────────────┬───────────────────┘
                                          │ USB Serial (JSON @ 50 Hz, 115200 Baud)
                                          ▼
                       ┌──────────────────────────────────────┐
                       │       ROS2 imu_serial_bridge         │
                       ├──────────────────────────────────────┤
                       │  ├── SerialReader (Auto Reconnect)   │
                       │  ├── ButtonEdgeDetector (200ms Lock) │
                       │  └── GestureController (Gain Scaling)│
                       └──────────────────┬───────────────────┘
                                          │ MAVLink UDP @ 20 Hz (Port 14550)
                                          ▼
                       ┌──────────────────────────────────────┐
                       │    ArduPilot SITL / Gazebo Sim       │
                       └──────────────────────────────────────┘
```

---

## 6. 📊 ROS2 Topics & Node Parameters

### Published Topics

| Topic Name | ROS Message Type | Frequency | Description |
|---|---|---|---|
| `/imu/raw_json` | `std_msgs/msg/String` | 50 Hz | Serialized raw JSON payload from ESP32 |
| `/imu1/data_raw` | `sensor_msgs/msg/Imu` | 50 Hz | Structured Right Hand IMU data & orientation quaternion |
| `/imu2/data_raw` | `sensor_msgs/msg/Imu` | 50 Hz | Structured Left Hand IMU data & orientation quaternion |

### Primary Node Parameters ([`config/params.yaml`](file:///home/user/ros2_ws/src/imu_serial_bridge/config/params.yaml))

```yaml
imu_serial_bridge:
  ros__parameters:
    serial_port: "/dev/ttyUSB0"
    baud_rate: 115200
    takeoff_altitude: 2.0
    velocity_gain: 0.000001
    max_velocity: 2.0
    dead_zone_deg: 5.0
    yaw_gain: 0.04
    max_yaw_rate: 0.8
```

---

## 7. 🖥️ Real-Time Terminal Dashboard

```
================================================
  ✋ AEROKINESIS DUAL MPU9250 GESTURE CONTROLLER
================================================
  IMU 1 (Right Hand)            IMU 2 (Left Hand)
  Pitch:  +12.50°                Pitch:   -3.10°
  Roll:    +2.80°                Roll:    +0.00°
  Gz:      +0.36 °/s             Gz:      -0.00 °/s

  Timestamp  8150 ms
------------------------------------------------
  Buttons
    Arm/Disarm  : □ RELEASED
    Mode Toggle : □ RELEASED
    Takeoff     : ■ PRESSED
    RTL         : □ RELEASED
------------------------------------------------
  Gesture Control  [ON]

  Hand 1 (Right):
    Pitch : +12.5 °  → Fwd/Bk +0.00 m/s
    Roll  :  +2.8 °  → Lft/Rt +0.00 m/s
  Hand 2 (Left):
    Pitch :  -3.1 °  → Thrst  +0.00 m/s
    Roll  :  +0.0 °  → Yaw    +0.00 rad/s

    Fwd/Bk  [     ▶▶▶▶   ]
    Lft/Rt  [     |      ]
    Thrust  [     |      ]
    YawRate [     |      ]
------------------------------------------------
  Drone  ARMED 🟢   Mode: GUIDED
================================================
```

---

## 8. 📁 Repository Structure

```
imu_serial_bridge/
├── config/                         # Configuration directory
│   ├── README.md                   # Config documentation
│   └── params.yaml                 # ROS2 parameters file
├── firmware/                       # Hardware firmware directory
│   ├── README.md                   # Firmware overview documentation
│   └── mpu6050_esp32/              # Primary ESP32 sketch directory
│       ├── README.md               # ESP32 dual IMU firmware documentation
│       └── mpu6050_esp32.ino       # ESP32 C++ code (MPU9250/6500)
├── imu_serial_bridge/              # Python ROS2 package source
│   ├── README.md                   # Python package module documentation
│   ├── __init__.py                 # Package initializer
│   ├── buttons.py                  # Debounced button edge detector
│   ├── gesture_controller.py       # Angle-to-velocity mapping controller
│   ├── imu_node.py                 # ROS2 bridge node & terminal UI
│   ├── mavproxy_controller.py      # MAVLink SITL client interface
│   ├── serial_node.py              # Standalone serial bridge module
│   └── serial_reader.py            # Non-blocking USB serial manager
├── launch/                         # ROS2 launch directory
│   ├── README.md                   # Launch script documentation
│   └── imu_serial.launch.py        # ROS2 launch file
├── media/                          # Visual Assets & Video Demos
│   ├── Eap32.jpeg                  # ESP32 hardware photo
│   ├── Left_Hand.jpeg              # Left hand controller photo
│   ├── Right_Hand_Buttons.jpeg     # Right hand button array photo
│   ├── Right_hand_MPU.jpeg         # Right hand MPU sensor photo
│   ├── v2.mp4                      # Version 2.0 demonstration video
│   └── v3.mp4                      # Version 3.0 demonstration video
├── resource/                       # ROS2 Ament package index directory
│   ├── README.md                   # Package index resource documentation
│   └── imu_serial_bridge          # Ament package index marker
├── package.xml                     # ROS2 package manifest
├── setup.cfg                       # Python setup configuration
├── setup.py                        # Python build setup
├── startup.md                      # Detailed multi-terminal startup guide
├── statup.md                       # Startup quick reference
└── README.md                       # Main project documentation
```

---

## 9. 📜 Version History & Evolution

### 🚀 Version 1.0 — Initial Single-IMU Prototype
* Single MPU6050 IMU mounted on the Right Hand.
* 2-Axis tilt control (Pitch/Roll -> Forward/Backward/Left/Right).
* Raw sensor fusion running on the host ROS2 node via Python complementary filter.

### 🎛️ Version 2.0 — Tactile Control Interface
* Integrated **4 tactile push-buttons** (ARM/DISARM, STABILIZE/GUIDED mode toggle, TAKEOFF, LAND/RTL).
* Hardware & Python software debouncing.
* Added MAVLink automated takeoff climb guard and flight mode state machine.
* **Demonstration Video**:

![AeroKinesis V2.0 Demonstration Video](https://glistening-lebkuchen-c76833.netlify.app/v2.mp4)

<video src="media/v2.mp4" controls width="100%" poster="media/Right_Hand_Buttons.jpeg">
  Your browser does not support the video tag. <a href="media/v2.mp4">Download v2.mp4</a>
</video>

### 🌟 Version 3.0 (Current Version) — Bimanual Dual-IMU Interface
* Dual MPU9250/6500 IMU architecture (Right Hand at `0x68` + Left Hand at `0x69`).
* Full **4-Axis Flight Control** (Pitch, Roll, Throttle/Altitude, Yaw Turn).
* **Onboard Sensor Fusion (α=0.98)** and **3-Second Neutral Pose Calibration** at boot on the ESP32.
* Full integration with Gazebo Iris Runway simulation and ArduPilot SITL.
* **Demonstration Video**:

![AeroKinesis V3.0 Demonstration Video](file:///home/user/ros2_ws/src/imu_serial_bridge/media/v3.mp4)

<video src="media/v3.mp4" controls width="100%" poster="media/Eap32.jpeg">
  Your browser does not support the video tag. <a href="media/v3.mp4">Download v3.mp4</a>
</video>

---

## 10. 🐞 Troubleshooting

### Serial Port Access Denied
```bash
sudo usermod -aG dialout $USER && newgrp dialout
```

### Drone Disarms Immediately Upon Takeoff
Ensure MAVProxy is connected and the ROS2 bridge node is running in `GUIDED` mode. The single-flight guard suppresses velocity commands for 12 seconds during takeoff climb to prevent overriding `NAV_TAKEOFF`.

---

## 📄 License
Apache 2.0 License. See [LICENSE](LICENSE) for details.
