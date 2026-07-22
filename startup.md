# 🚀 AeroKinesis — Full System Startup & Setup Guide

### 🛰️ Next-Gen Wearable Spatial Interface for ROS2 Humble & ArduPilot SITL / Gazebo Sim

![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-34495E?style=for-the-badge&logo=ros&logoColor=white)
![ArduPilot SITL](https://img.shields.io/badge/ArduPilot-SITL_Copter-E74C3C?style=for-the-badge&logo=drone&logoColor=white)
![Gazebo Sim](https://img.shields.io/badge/Gazebo-Iris_Runway-FF6F00?style=for-the-badge&logo=gazebo&logoColor=white)

---

## 🎥 Demonstration Video Demos

### 🌟 AeroKinesis v3.0 Dual-IMU Flight Demo
![AeroKinesis V3.0 Dual-IMU Flight Demo](file:///home/user/ros2_ws/src/imu_serial_bridge/media/v3.mp4)

<video src="media/v3.mp4" controls width="100%" poster="media/Right_hand_MPU.jpeg">
  Your browser does not support the video tag. <a href="media/v3.mp4">Download v3.mp4</a>
</video>

---

## 📥 1. Installation & Workspace Setup

Run the following commands to clone and build the repository inside your ROS2 workspace:

```bash
# Step 1: Navigate to ROS2 workspace src directory and clone
cd ~/ros2_ws/src
git clone https://github.com/your-username/imu_serial_bridge.git
cd imu_serial_bridge

# Step 2: Grant serial port permission (one-time setup)
sudo usermod -aG dialout $USER
newgrp dialout

# Step 3: Install Python requirements
pip3 install pyserial pymavlink

# Step 4: Build package
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select imu_serial_bridge --symlink-install
source install/setup.bash
```

---

## 📋 2. System Architecture Startup Overview

To launch the complete system, open **3 separate terminal windows/tabs** and execute the commands below in sequence:

```
┌─────────────────────────┐     ┌─────────────────────────┐     ┌─────────────────────────┐
│       TERMINAL 1        │     │       TERMINAL 2        │     │       TERMINAL 3        │
│   ArduPilot SITL Copter │     │   Gazebo 3D Simulation  │     │   ROS2 Gesture Bridge   │
│   (sim_vehicle.py)      │     │   (gz sim -v4)          │     │   (imu_serial.launch.py)│
└────────────┬────────────┘     └────────────┬────────────┘     └────────────┬────────────┘
             │                               │                               │
             └───────────────────────────────┼───────────────────────────────┘
                                             ▼
                             🎮 Integrated Live Drone Flight
```

---

## 🚀 3. Step-by-Step Multi-Terminal Commands

### 🖥️ Terminal 1: ArduPilot SITL Copter Simulation
Start the ArduPilot Software-In-The-Loop (SITL) simulator configured with the Gazebo Iris model and MAVProxy outputs.

```bash
sim_vehicle.py -v ArduCopter -f gazebo-iris --console --model JSON --map --out=127.0.0.1:14550
```

---

### 🌌 Terminal 2: Gazebo Simulator
Launch the Gazebo 3D physics environment loaded with the Iris Runway world model.

```bash
gz sim -v4 -r iris_runway.sdf
```

---

### ⚡ Terminal 3: ROS2 Hand Gesture Bridge Node
Source your ROS2 workspace and launch the `imu_serial_bridge` node to interface the ESP32 wearable controller with SITL.

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch imu_serial_bridge imu_serial.launch.py serial_port:=/dev/ttyUSB0
```

---

## 🎮 4. Flight Controls & Button Operations

1. **Boot Calibration**: Hold the ESP32 controller flat and still for **3 seconds** after powering on so the onboard IMU neutral calibration completes.
2. **Arm Motors**: Press **Arm/Disarm** button (GPIO 32) to arm motors.
3. **Takeoff**: Press **Takeoff** button (GPIO 25) to trigger vertical takeoff to 2.0m height.
4. **Gesture Flight**:
   * **Right Hand Pitch**: Forward / Backward (`vx`)
   * **Right Hand Roll**: Left / Right (`vy`)
   * **Left Hand Pitch**: Climb / Descend (`vz`)
   * **Left Hand Roll**: Yaw Turn (`yaw_rate`)
5. **RTL / Land**: Press **RTL** button (GPIO 26) to return home and land automatically.
