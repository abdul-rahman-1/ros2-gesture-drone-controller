# Configuration Directory (`config/`)

This directory contains configuration files and parameters for the `imu_serial_bridge` ROS2 node.

## Files

* **[`params.yaml`](file:///home/user/ros2_ws/src/imu_serial_bridge/config/params.yaml)**: Primary ROS2 parameter file containing runtime configurations for serial communication, velocity scaling, dead-zones, flight gains, and MAVProxy settings.

---

## Key Parameters Overview

| Parameter | Type | Default | Description |
|---|---|---|---|
| `serial_port` | string | `/dev/ttyUSB0` | Linux serial port pathway for ESP32 |
| `baud_rate` | int | `115200` | Serial baud rate matching ESP32 firmware |
| `frame_id` | string | `imu_link` | ROS TF frame ID for published IMU topics |
| `mavproxy_address` | string | `udp:127.0.0.1:14550` | MAVProxy connection URI |
| `takeoff_altitude` | float | `2.0` | Target takeoff altitude in meters |
| `gesture_enabled` | bool | `true` | Enable/disable hand-gesture velocity control |
| `velocity_gain` | float | `0.000001` | Drone velocity gain (m/s per degree of tilt) |
| `max_velocity` | float | `2.0` | Maximum velocity limit (m/s) |
| `dead_zone_deg` | float | `5.0` | Tilt dead-zone angle in degrees |
| `yaw_gain` | float | `0.04` | Yaw rate gain (rad/s per degree of left hand roll) |
| `max_yaw_rate` | float | `0.8` | Maximum yaw rate limit (rad/s) |

---

## Loading Parameters

To run the ROS2 node with this configuration file:

```bash
ros2 run imu_serial_bridge imu_node --ros-args --params-file src/imu_serial_bridge/config/params.yaml
```
