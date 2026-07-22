#!/usr/bin/env python3
"""
imu_serial.launch.py — ROS2 launch file for imu_serial_bridge v2.

Starts the primary imu_node which handles:
  - MPU6050 IMU data streaming
  - 4 push-button interface (ARM/DISARM, FORCE_ARM, TAKEOFF, LAND)
  - MAVProxy / ArduPilot SITL drone control

Parameters (override on command line with param:=value):
  serial_port       /dev/ttyUSB0        Path to ESP32 serial device
  baud_rate         115200              Serial baud rate
  frame_id          imu_link            TF frame for sensor messages
  publish_rate      50.0                Nominal IMU rate (Hz)
  mavproxy_address  udp:127.0.0.1:14550 pymavlink connection string
  mavproxy_enabled  true                Enable/disable drone control
  takeoff_altitude  2.0                 Takeoff height in metres

Example usage:
  # Basic (IMU + buttons + MAVProxy on default port)
  ros2 launch imu_serial_bridge imu_serial.launch.py

  # Custom serial port
  ros2 launch imu_serial_bridge imu_serial.launch.py serial_port:=/dev/ttyUSB1

  # Disable MAVProxy (IMU + buttons only, no drone commands)
  ros2 launch imu_serial_bridge imu_serial.launch.py mavproxy_enabled:=false

  # Custom takeoff height
  ros2 launch imu_serial_bridge imu_serial.launch.py takeoff_altitude:=3.0

SITL Setup (run before launching):
  # Terminal 1 — ArduPilot SITL (auto-starts MAVProxy):
  sim_vehicle.py -v ArduCopter --console --map

  # Terminal 2 — this launch file:
  ros2 launch imu_serial_bridge imu_serial.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Build the LaunchDescription for the IMU + button + MAVProxy bridge."""

    # ── Launch arguments ───────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument(
            "serial_port", default_value="/dev/ttyUSB0",
            description="Path to ESP32 USB serial device (e.g. /dev/ttyUSB0).",
        ),
        DeclareLaunchArgument(
            "baud_rate", default_value="115200",
            description="Serial baud rate — must match ESP32 firmware.",
        ),
        DeclareLaunchArgument(
            "frame_id", default_value="imu_link",
            description="TF frame ID in sensor_msgs/Imu and Temperature headers.",
        ),
        DeclareLaunchArgument(
            "publish_rate", default_value="50.0",
            description="Nominal IMU publish rate Hz (informational; set by firmware).",
        ),
        DeclareLaunchArgument(
            "mavproxy_address", default_value="udp:127.0.0.1:14550",
            description="pymavlink connection string to MAVProxy output port.",
        ),
        DeclareLaunchArgument(
            "mavproxy_enabled", default_value="true",
            description="Set false to run IMU + buttons only (no drone commands).",
        ),
        DeclareLaunchArgument(
            "takeoff_altitude", default_value="2.0",
            description="Target altitude in metres when takeoff button is pressed.",
        ),
        DeclareLaunchArgument(
            "gesture_enabled", default_value="true",
            description="Enable IMU hand-tilt → drone velocity control.",
        ),
        DeclareLaunchArgument(
            "velocity_gain", default_value="1.5",
            description="Drone velocity in m/s per radian of hand tilt.",
        ),
        DeclareLaunchArgument(
            "max_velocity", default_value="2.0",
            description="Maximum horizontal velocity clamp (m/s).",
        ),
        DeclareLaunchArgument(
            "dead_zone_deg", default_value="5.0",
            description="Tilt angle below which velocity = 0 (degrees).",
        ),
        DeclareLaunchArgument(
            "yaw_gain", default_value="1.0",
            description="Yaw rate = gz * yaw_gain (dimensionless multiplier).",
        ),
        DeclareLaunchArgument(
            "max_yaw_rate", default_value="0.8",
            description="Maximum yaw rate clamp (rad/s).",
        ),
        DeclareLaunchArgument(
            "filter_alpha", default_value="0.96",
            description="Complementary filter coefficient (0=accel only, 1=gyro only).",
        ),
        DeclareLaunchArgument(
            "cmd_rate_hz", default_value="20.0",
            description="Velocity command send rate to MAVProxy (Hz).",
        ),
    ]

    # ── Primary node: IMU + buttons + MAVProxy ─────────────────────────────
    imu_node = Node(
        package="imu_serial_bridge",
        executable="imu_node",
        name="imu_serial_bridge",
        output="screen",
        emulate_tty=True,           # Preserve ANSI colours in the dashboard
        parameters=[
            {
                "serial_port":      LaunchConfiguration("serial_port"),
                "baud_rate":        LaunchConfiguration("baud_rate"),
                "frame_id":         LaunchConfiguration("frame_id"),
                "publish_rate":     LaunchConfiguration("publish_rate"),
                "mavproxy_address": LaunchConfiguration("mavproxy_address"),
                "mavproxy_enabled": LaunchConfiguration("mavproxy_enabled"),
                "takeoff_altitude": LaunchConfiguration("takeoff_altitude"),
                "gesture_enabled":  LaunchConfiguration("gesture_enabled"),
                "velocity_gain":    LaunchConfiguration("velocity_gain"),
                "max_velocity":     LaunchConfiguration("max_velocity"),
                "dead_zone_deg":    LaunchConfiguration("dead_zone_deg"),
                "yaw_gain":         LaunchConfiguration("yaw_gain"),
                "max_yaw_rate":     LaunchConfiguration("max_yaw_rate"),
                "filter_alpha":     LaunchConfiguration("filter_alpha"),
                "cmd_rate_hz":      LaunchConfiguration("cmd_rate_hz"),
            }
        ],
        remappings=[
            # Add namespace remappings here if integrating into a larger system.
            # ("/imu/data_raw", "/drone/imu/data_raw"),
        ],
    )

    # ── Startup log ────────────────────────────────────────────────────────
    startup_log = LogInfo(
        msg=[
            "Launching imu_serial_bridge v2 | port=",
            LaunchConfiguration("serial_port"),
            " mavproxy=",
            LaunchConfiguration("mavproxy_address"),
            " takeoff_alt=",
            LaunchConfiguration("takeoff_altitude"),
            "m",
        ]
    )

    return LaunchDescription(
        args
        + [
            startup_log,
            imu_node,

            # TODO-EXTEND: Orientation filter node
            # Node(package="imu_serial_bridge", executable="orientation_filter_node", ...),

            # TODO-EXTEND: Gazebo / MAVROS bridge node
            # Node(package="imu_serial_bridge", executable="gazebo_bridge_node", ...),
        ]
    )
