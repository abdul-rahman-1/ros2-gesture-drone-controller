# Python Package Directory (`imu_serial_bridge/imu_serial_bridge/`)

This directory contains the Python modules that form the ROS2 Humble node logic for `imu_serial_bridge`.

---

## File Modules Overview

| File | Responsibilities |
|---|---|
| **[`imu_node.py`](file:///home/user/ros2_ws/src/imu_serial_bridge/imu_serial_bridge/imu_node.py)** | Primary ROS2 node (`imu_serial_bridge`). Coordinates serial I/O, parses JSON packets, publishes ROS topics (`/imu1/data_raw`, `/imu2/data_raw`, `/imu/raw_json`), processes button events, renders dashboard console, and dispatches velocity setpoints. |
| **[`gesture_controller.py`](file:///home/user/ros2_ws/src/imu_serial_bridge/imu_serial_bridge/gesture_controller.py)** | Maps pre-fused tilt angles (`pitch1`, `roll1`, `pitch2`, `roll2`) to 4-axis LOCAL_NED drone velocity setpoints (`vx`, `vy`, `vz`, `yaw_rate`). Implements dead-zone filtering, gain scaling, and sensitivity multipliers. |
| **[`mavproxy_controller.py`](file:///home/user/ros2_ws/src/imu_serial_bridge/imu_serial_bridge/mavproxy_controller.py)** | MAVLink communication client via `pymavlink`. Connects to SITL / MAVProxy over UDP (`127.0.0.1:14550`), executes flight mode switches (GUIDED, LAND, RTL, STABILIZE), commands takeoff, and streams 20 Hz velocity targets. |
| **[`buttons.py`](file:///home/user/ros2_ws/src/imu_serial_bridge/imu_serial_bridge/buttons.py)** | Rising-edge button detector (`ButtonEdgeDetector`). Tracks state transitions (0 → 1) with 200 ms software lockout timer to prevent duplicate triggers. |
| **[`serial_reader.py`](file:///home/user/ros2_ws/src/imu_serial_bridge/imu_serial_bridge/serial_reader.py)** | Pyserial wrapper (`SerialReader`). Handles non-blocking line reading, ASCII decoding, JSON string filtering, and automatic USB reconnection on disconnect. |
| **[`serial_node.py`](file:///home/user/ros2_ws/src/imu_serial_bridge/imu_serial_bridge/serial_node.py)** | Alternative / standalone serial bridge node module. |
| **[`__init__.py`](file:///home/user/ros2_ws/src/imu_serial_bridge/imu_serial_bridge/__init__.py)** | Python package initialization file. |
