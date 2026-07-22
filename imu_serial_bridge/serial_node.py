#!/usr/bin/env python3
"""
serial_node.py — ROS2 node for bridging MPU6050 serial data to ROS2 topics.

This node opens a USB serial port, reads JSON-encoded IMU packets from an
ESP32 + MPU6050 sensor, and publishes them on three ROS2 topics:

  /imu/raw_json        (std_msgs/String)        — raw JSON string
  /imu/data_raw        (sensor_msgs/Imu)         — structured IMU message
  /imu/temperature     (sensor_msgs/Temperature) — temperature in °C

Design goals:
  - Robust serial reconnection (survives cable unplug/replug)
  - Clean separation of concerns (read, parse, publish, display)
  - Easily extended with orientation filters (Complementary, Madgwick)
  - Structured for future MAVROS / ArduPilot SITL integration

Node parameters (all configurable via launch file or ros2 param):
  serial_port   (str)   default: /dev/ttyUSB0
  baud_rate     (int)   default: 115200
  frame_id      (str)   default: imu_link
  publish_rate  (float) default: 50.0  [Hz]  (informational; actual rate is
                                               determined by ESP32 firmware)

Future extension points (marked with TODO-EXTEND comments):
  - Complementary / Madgwick filter for roll, pitch, yaw estimation
  - Gesture recognition state machine
  - MAVROS velocity command publisher
  - ArduPilot SITL / Gazebo Harmonic integration

Author: Hand-Gesture Drone Project
Date:   2026
"""

# ─────────────────────────────────────────────────────────────────────────────
# Standard library
# ─────────────────────────────────────────────────────────────────────────────
import json
import math
import os
import sys
import time
from typing import Optional

# ─────────────────────────────────────────────────────────────────────────────
# Third-party
# ─────────────────────────────────────────────────────────────────────────────
import serial                      # pyserial
import serial.tools.list_ports

# ─────────────────────────────────────────────────────────────────────────────
# ROS2
# ─────────────────────────────────────────────────────────────────────────────
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String
from sensor_msgs.msg import Imu, Temperature
from builtin_interfaces.msg import Time as RosTime


# ─────────────────────────────────────────────────────────────────────────────
# Constants
# ─────────────────────────────────────────────────────────────────────────────

# Reconnect wait interval when the serial port is unavailable.
_RECONNECT_INTERVAL_S: float = 1.0

# Width of the decorative separator line in the terminal display.
_DISPLAY_WIDTH: int = 40

# Identity quaternion (no orientation estimate yet).
_IDENTITY_QUATERNION = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}

# Covariance matrices: -1 means "unknown / not estimated".
_COVARIANCE_UNKNOWN = [-1.0] + [0.0] * 8   # 9-element list

# Accelerometer measurement covariance (diagonal, m/s² ²).
# Derived from MPU6050 datasheet noise density ≈ 400 µg/√Hz at 50 Hz.
_ACCEL_COVARIANCE = [
    0.01, 0.0, 0.0,
    0.0, 0.01, 0.0,
    0.0, 0.0, 0.01,
]

# Gyroscope measurement covariance (diagonal, rad/s ²).
# Derived from MPU6050 datasheet noise density ≈ 0.005 °/s/√Hz at 50 Hz.
_GYRO_COVARIANCE = [
    0.0003, 0.0, 0.0,
    0.0, 0.0003, 0.0,
    0.0, 0.0, 0.0003,
]


# ─────────────────────────────────────────────────────────────────────────────
# Helper – display formatting
# ─────────────────────────────────────────────────────────────────────────────

def _format_console_output(data: dict) -> str:
    """
    Build a nicely formatted multi-line string from a parsed IMU data dict.

    Parameters
    ----------
    data : dict
        Keys: timestamp, ax, ay, az, gx, gy, gz, temp  (all floats/ints)

    Returns
    -------
    str
        Ready-to-print formatted block.
    """
    sep = "=" * _DISPLAY_WIDTH
    thin = "-" * 12

    lines = [
        sep,
        "MPU6050 DATA",
        thin,
        "",
        "Acceleration",
        f"  X : {data.get('ax', 0.0):>10.3f} m/s²",
        f"  Y : {data.get('ay', 0.0):>10.3f} m/s²",
        f"  Z : {data.get('az', 0.0):>10.3f} m/s²",
        "",
        "Gyroscope",
        f"  X : {data.get('gx', 0.0):>10.3f} rad/s",
        f"  Y : {data.get('gy', 0.0):>10.3f} rad/s",
        f"  Z : {data.get('gz', 0.0):>10.3f} rad/s",
        "",
        "Temperature",
        f"  {data.get('temp', 0.0):.1f} °C",
        "",
        "Timestamp",
        f"  {data.get('timestamp', 0)} ms",
        "",
        sep,
    ]
    return "\n".join(lines)


# ─────────────────────────────────────────────────────────────────────────────
# Node
# ─────────────────────────────────────────────────────────────────────────────

class IMUSerialBridgeNode(Node):
    """
    ROS2 node that reads MPU6050 JSON data from a serial port and publishes
    it on standard ROS2 IMU topics.

    Topic graph
    -----------
    (ESP32) --[USB Serial JSON]--> [IMUSerialBridgeNode]
                                      ├─ /imu/raw_json        std_msgs/String
                                      ├─ /imu/data_raw        sensor_msgs/Imu
                                      └─ /imu/temperature     sensor_msgs/Temperature

    TODO-EXTEND: Orientation Filter
        Add a complementary or Madgwick filter here to compute roll/pitch/yaw
        from ax/ay/az + gx/gy/gz and populate imu_msg.orientation.

    TODO-EXTEND: Gesture Recognition
        Add a GestureDetector class that consumes the parsed data dict and
        publishes a std_msgs/String on /imu/gesture.

    TODO-EXTEND: MAVROS Integration
        Add a geometry_msgs/TwistStamped publisher on /mavros/setpoint_velocity/cmd_vel
        driven by the detected gesture or orientation.
    """

    def __init__(self) -> None:
        super().__init__("imu_serial_bridge")

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("serial_port",  "/dev/ttyUSB0")
        self.declare_parameter("baud_rate",    115200)
        self.declare_parameter("frame_id",     "imu_link")
        self.declare_parameter("publish_rate", 50.0)

        self._port_name:  str   = self.get_parameter("serial_port").value
        self._baud_rate:  int   = self.get_parameter("baud_rate").value
        self._frame_id:   str   = self.get_parameter("frame_id").value
        self._pub_rate:   float = self.get_parameter("publish_rate").value

        self.get_logger().info(
            f"IMU Serial Bridge starting | port={self._port_name} "
            f"baud={self._baud_rate} frame_id={self._frame_id}"
        )

        # ── QoS profile ───────────────────────────────────────────────────────
        # Best-effort / keep-last(10) — matches typical sensor data consumers.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── Publishers ────────────────────────────────────────────────────────
        self._pub_raw_json = self.create_publisher(
            String, "/imu/raw_json", sensor_qos
        )
        self._pub_imu = self.create_publisher(
            Imu, "/imu/data_raw", sensor_qos
        )
        self._pub_temp = self.create_publisher(
            Temperature, "/imu/temperature", sensor_qos
        )

        # ── Internal state ────────────────────────────────────────────────────
        self._serial: Optional[serial.Serial] = None
        self._message_count: int = 0
        self._last_display_time: float = time.monotonic()
        # True once the first valid JSON packet has been received.
        self._first_packet_received: bool = False

        # ── Direct terminal handle ────────────────────────────────────────────
        # Open /dev/tty directly so display output goes straight to the user's
        # terminal and completely bypasses the ROS2 launch system's stdout
        # pipe (which adds the '[imu_serial_node-1]' prefix to every line).
        try:
            self._tty = open("/dev/tty", "w")
        except OSError:
            # Fallback: running without a controlling terminal (e.g. in a
            # Docker container with no TTY) — use stdout instead.
            self._tty = sys.stdout

        # ── Timer drives the main read/publish loop ───────────────────────────
        # Period is short so the serial buffer is drained promptly; actual
        # publish cadence is gated by the ESP32's 50 Hz output.
        self._timer = self.create_timer(0.01, self._timer_callback)

        # Print initial waiting banner directly to the terminal.
        self._print_waiting()

        # Attempt initial connection (non-blocking; retries inside callback).
        self._connect_serial()

    # ── Serial connection management ──────────────────────────────────────────

    def _connect_serial(self) -> bool:
        """
        Try to open the configured serial port.

        Returns
        -------
        bool
            True if connected, False if the device is not yet available.
        """
        try:
            self._serial = serial.Serial(
                port=self._port_name,
                baudrate=self._baud_rate,
                timeout=0.05,       # Short timeout → non-blocking feel
                write_timeout=1.0,
            )
            self._serial.reset_input_buffer()
            self.get_logger().info(
                f"Serial port opened: {self._port_name} @ {self._baud_rate} baud"
            )
            return True

        except serial.SerialException as exc:
            self.get_logger().warning(
                f"Cannot open serial port {self._port_name}: {exc} "
                f"— retrying in {_RECONNECT_INTERVAL_S:.0f}s"
            )
            self._serial = None
            return False

    def _is_connected(self) -> bool:
        """Return True if a serial connection is currently open."""
        return self._serial is not None and self._serial.is_open

    # ── Timer callback (main loop) ─────────────────────────────────────────────

    def _timer_callback(self) -> None:
        """
        Called at ~100 Hz by the ROS2 timer.

        Reads all available lines from the serial buffer, parses each as
        JSON, and publishes ROS2 messages.  Handles disconnection and
        reconnection transparently.
        """
        # ── Ensure connection ─────────────────────────────────────────────────
        if not self._is_connected():
            # Throttle reconnect attempts to _RECONNECT_INTERVAL_S.
            if not hasattr(self, "_last_reconnect_attempt"):
                self._last_reconnect_attempt = 0.0

            now = time.monotonic()
            if now - self._last_reconnect_attempt >= _RECONNECT_INTERVAL_S:
                self._last_reconnect_attempt = now
                self._connect_serial()
            return

        # ── Read lines from buffer ────────────────────────────────────────────
        try:
            # Drain all currently available lines (handles burst arrivals).
            while self._serial.in_waiting:
                raw_line = self._serial.readline()
                self._process_line(raw_line)

        except serial.SerialException as exc:
            self.get_logger().error(
                f"Serial read error: {exc} — attempting reconnect"
            )
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None

    # ── Line processing ───────────────────────────────────────────────────────

    def _process_line(self, raw_bytes: bytes) -> None:
        """
        Decode, parse, validate, and publish one line from the serial port.

        Parameters
        ----------
        raw_bytes : bytes
            Raw bytes including any trailing \\r\\n from the ESP32.
        """
        # ── Decode ────────────────────────────────────────────────────────────
        try:
            line = raw_bytes.decode("ascii", errors="replace").strip()
        except Exception as exc:
            self.get_logger().debug(f"Decode error: {exc}")
            return

        # Skip empty lines and startup banner lines from the ESP32.
        if not line or not line.startswith("{"):
            return

        # ── Parse JSON ────────────────────────────────────────────────────────
        try:
            data: dict = json.loads(line)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(
                f"Invalid JSON (skipping): {exc} | raw='{line[:80]}'"
            )
            return

        # ── Validate required fields ──────────────────────────────────────────
        required = ("timestamp", "ax", "ay", "az", "gx", "gy", "gz", "temp")
        missing = [k for k in required if k not in data]
        if missing:
            self.get_logger().warning(
                f"Missing fields in JSON: {missing} | raw='{line[:80]}'"
            )
            return

        # ── Get ROS2 timestamp ─────────────────────────────────────────────────
        ros_now = self.get_clock().now().to_msg()

        # ── Publish /imu/raw_json ─────────────────────────────────────────────
        self._publish_raw_json(line)

        # ── Publish /imu/data_raw ─────────────────────────────────────────────
        self._publish_imu(data, ros_now)

        # ── Publish /imu/temperature ──────────────────────────────────────────
        self._publish_temperature(data, ros_now)

        # ── Console display ───────────────────────────────────────────────────
        self._print_display(data)

        self._message_count += 1

        # TODO-EXTEND: Orientation Filter
        #   roll, pitch, yaw = complementary_filter(data, dt)
        #   or
        #   quaternion = madgwick_filter.update(data, dt)

        # TODO-EXTEND: Gesture Recognition
        #   gesture = self._gesture_detector.update(data)
        #   if gesture: self._pub_gesture.publish(String(data=gesture))

        # TODO-EXTEND: MAVROS velocity control
        #   vel_cmd = gesture_to_velocity(gesture)
        #   self._pub_cmd_vel.publish(vel_cmd)

    # ── Publisher helpers ─────────────────────────────────────────────────────

    def _publish_raw_json(self, json_line: str) -> None:
        """Publish the raw JSON string."""
        msg = String()
        msg.data = json_line
        self._pub_raw_json.publish(msg)

    def _publish_imu(self, data: dict, stamp: RosTime) -> None:
        """
        Build and publish a sensor_msgs/Imu message.

        Orientation is set to the identity quaternion until an orientation
        estimation filter is integrated.

        Parameters
        ----------
        data  : dict   Parsed IMU data from the ESP32.
        stamp : RosTime  Current ROS2 timestamp.
        """
        msg = Imu()

        # Header
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id

        # Orientation (identity — no filter yet).
        msg.orientation.x = _IDENTITY_QUATERNION["x"]
        msg.orientation.y = _IDENTITY_QUATERNION["y"]
        msg.orientation.z = _IDENTITY_QUATERNION["z"]
        msg.orientation.w = _IDENTITY_QUATERNION["w"]
        msg.orientation_covariance = list(_COVARIANCE_UNKNOWN)

        # Linear acceleration (m/s²).
        msg.linear_acceleration.x = float(data["ax"])
        msg.linear_acceleration.y = float(data["ay"])
        msg.linear_acceleration.z = float(data["az"])
        msg.linear_acceleration_covariance = list(_ACCEL_COVARIANCE)

        # Angular velocity (rad/s).
        msg.angular_velocity.x = float(data["gx"])
        msg.angular_velocity.y = float(data["gy"])
        msg.angular_velocity.z = float(data["gz"])
        msg.angular_velocity_covariance = list(_GYRO_COVARIANCE)

        self._pub_imu.publish(msg)

    def _publish_temperature(self, data: dict, stamp: RosTime) -> None:
        """
        Build and publish a sensor_msgs/Temperature message.

        Parameters
        ----------
        data  : dict   Parsed IMU data from the ESP32.
        stamp : RosTime  Current ROS2 timestamp.
        """
        msg = Temperature()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        msg.temperature = float(data["temp"])     # °C
        msg.variance = 0.0                         # Not characterised yet
        self._pub_temp.publish(msg)

    # ── Console display ───────────────────────────────────────────────────────

    def _print_waiting(self) -> None:
        """
        Print a clean banner to the terminal while waiting for the first
        packet from the ESP32.  Written to /dev/tty so no launch prefix.
        """
        sep = "=" * _DISPLAY_WIDTH
        banner = (
            "\033[H\033[J"   # clear screen
            + sep + "\n"
            + "MPU6050 DATA\n"
            + "-" * 12 + "\n"
            + "\n"
            + "  Waiting for ESP32 data...\n"
            + "  (Make sure firmware is flashed and USB is connected)\n"
            + "\n"
            + sep + "\n"
        )
        self._tty.write(banner)
        self._tty.flush()

    def _print_display(self, data: dict) -> None:
        """
        Clear the terminal and print the latest sensor reading.

        Writes directly to /dev/tty so the ROS2 launch system's stdout pipe
        (which prepends '[imu_serial_node-1]' to every line) is completely
        bypassed — giving a clean, prefix-free live dashboard.

        Uses ANSI escape codes to move the cursor to the top-left and
        overwrite the previous output.
        """
        output = "\033[H\033[J" + _format_console_output(data) + "\n"
        self._tty.write(output)
        self._tty.flush()

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def destroy_node(self) -> None:
        """Gracefully close the serial port and terminal handle on shutdown."""
        if self._is_connected():
            try:
                self._serial.close()
                self.get_logger().info("Serial port closed.")
            except Exception:
                pass
        # Close the /dev/tty handle (skip if it is stdout fallback).
        try:
            if self._tty is not sys.stdout:
                self._tty.close()
        except Exception:
            pass
        super().destroy_node()


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None) -> None:
    """Initialise rclpy, spin the node, and clean up on exit."""
    rclpy.init(args=args)
    node = IMUSerialBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt — shutting down.")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
