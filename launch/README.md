# ROS2 Launch Directory (`launch/`)

This directory contains the launch files for starting the `imu_serial_bridge` package.

---

## Launch Files

* **[`imu_serial.launch.py`](file:///home/user/ros2_ws/src/imu_serial_bridge/launch/imu_serial.launch.py)**: Primary ROS2 launch script. Starts the `imu_node` executable with parameter overrides and config file binding.

---

## Launch Arguments

| Argument | Default | Description |
|---|---|---|
| `serial_port` | `/dev/ttyUSB0` | Linux device path for the ESP32 serial port |
| `baud_rate` | `115200` | Serial communication baud rate |
| `mavproxy_address` | `udp:127.0.0.1:14550` | UDP address for MAVProxy / SITL communication |
| `takeoff_altitude` | `2.0` | Target altitude in meters for button takeoff |
| `params_file` | `config/params.yaml` | Path to ROS2 parameter YAML file |

---

## Usage Examples

### Default Launch
```bash
ros2 launch imu_serial_bridge imu_serial.launch.py
```

### Launch with Custom Port & Altitude
```bash
ros2 launch imu_serial_bridge imu_serial.launch.py serial_port:=/dev/ttyUSB1 takeoff_altitude:=3.0
```
