"""
imu_node.py — Main ROS2 node: ESP32 + MPU6050 + Buttons + Gesture Control.

Pipeline overview
-----------------

  SerialReader  ──JSON──▶  parse & validate
                                │
              ┌─────────────────┼──────────────────────┐
              │                 │                       │
              ▼                 ▼                       ▼
        IMU publish       ButtonEdgeDetector      GestureController
    (/imu/data_raw)       (rising-edge 0→1)    ComplementaryFilter
    (/imu/temperature)         │               (pitch + roll → vx,vy)
    (/imu/raw_json)            │                       │
              │                ▼                       │
              │    MAVProxyController                  │
              │    ├─ toggle_arm()                     │
              │    ├─ force_arm()                      │
              │    ├─ takeoff(alt)                     │
              │    ├─ land()                           │
              │    └─ send_velocity(vx,vy,vz,yaw) ◀───┘ (20 Hz timer)

Gesture control mapping (hand tilted flat = hover):
  Pitch forward  (tilt nose down)  →  fly forward   (+vx NED)
  Pitch backward (tilt nose up)    →  fly backward  (-vx NED)
  Roll right     (tilt right)      →  fly right     (+vy NED)
  Roll left      (tilt left)       →  fly left      (-vy NED)
  Twist CW       (gz+)             →  rotate CW     (+yaw_rate)
  Hand flat                        →  hover (dead zone)

Safety rules:
  - Velocity commands are only sent when drone is ARMED + in GUIDED mode.
  - Pressing LAND button immediately stops gesture commands and lands.
  - Gesture control can be disabled via ROS2 parameter.
  - On reconnect the filter is reset to prevent stale angle estimates.

Node parameters:
  serial_port         str    /dev/ttyUSB0
  baud_rate           int    115200
  frame_id            str    imu_link
  publish_rate        float  50.0
  mavproxy_address    str    udp:127.0.0.1:14550
  mavproxy_enabled    bool   true
  takeoff_altitude    float  2.0
  gesture_enabled     bool   true
  velocity_gain       float  1.5   (m/s per radian of tilt)
  max_velocity        float  2.0   (m/s hard clamp)
  dead_zone_deg       float  5.0   (degrees; tilt below this = hover)
  yaw_gain            float  1.0   (yaw_rate = gz * yaw_gain)
  max_yaw_rate        float  0.8   (rad/s)
  filter_alpha        float  0.96  (complementary filter coefficient)
  cmd_rate_hz         float  20.0  (velocity command send rate)

Published topics:
  /imu/raw_json        std_msgs/String
  /imu/data_raw        sensor_msgs/Imu
  /imu/temperature     sensor_msgs/Temperature

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
import threading
from typing import Optional

# ─────────────────────────────────────────────────────────────────────────────
# Third-party
# ─────────────────────────────────────────────────────────────────────────────
import serial

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
# Package modules
# ─────────────────────────────────────────────────────────────────────────────
from imu_serial_bridge.serial_reader       import SerialReader
from imu_serial_bridge.buttons             import ButtonEdgeDetector, BUTTON_NAMES
from imu_serial_bridge.mavproxy_controller import MAVProxyController
from imu_serial_bridge.gesture_controller  import GestureController, GestureOutput


# ─────────────────────────────────────────────────────────────────────────────
# Constants
# ─────────────────────────────────────────────────────────────────────────────

_RECONNECT_INTERVAL_S:   float = 1.0
_DISPLAY_WIDTH:          int   = 44

_IDENTITY_QUAT   = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
_ORI_COV_UNKNOWN = [-1.0] + [0.0] * 8
_ACCEL_COV       = [0.01, 0.0, 0.0,   0.0, 0.01, 0.0,   0.0, 0.0, 0.01]
_GYRO_COV        = [3e-4, 0.0, 0.0,   0.0, 3e-4, 0.0,   0.0, 0.0, 3e-4]

# Firmware v4 (MPU9250) sends pre-fused angles instead of raw accel.
_REQUIRED_IMU_FIELDS    = ("timestamp", "pitch1", "roll1", "pitch2", "roll2",
                           "gx1", "gy1", "gz1", "gx2", "gy2", "gz2")
_REQUIRED_BUTTON_FIELDS = ("arm_disarm", "mode_toggle", "takeoff", "rtl")

_BUTTON_LABELS = {
    "arm_disarm": "Arm/Disarm",
    "mode_toggle": "Mode Toggle",
    "takeoff":    "Takeoff   ",
    "rtl":        "RTL        ",
}


# ─────────────────────────────────────────────────────────────────────────────
# Display helper
# ─────────────────────────────────────────────────────────────────────────────

def _format_display(
    data:       dict,
    btn_states: dict,
    gesture:    GestureOutput,
    mav_armed:  bool,
    mav_mode:   str,
    gc_enabled: bool,
) -> str:
    """
    Build the complete terminal dashboard string.

    Sections:
      IMU data  — accelerometer, gyroscope, temperature, timestamp
      Buttons   — PRESSED / RELEASED state of each button
      Gesture   — pitch/roll angles + computed velocity commands
      Drone     — armed state, flight mode
    """
    sep  = "=" * _DISPLAY_WIDTH
    thin = "-" * 14

    # ── Drone status line ─────────────────────────────────────────────────────
    arm_str  = "ARMED  🟢" if mav_armed  else "DISARMED 🔴"
    gc_str   = "ON" if gc_enabled else "OFF"

    # ── Gesture velocity bars (visual) ────────────────────────────────────────
    def _bar(v: float, max_v: float = 2.0, width: int = 10) -> str:
        """Tiny ASCII bar chart: negative = left, positive = right."""
        fraction = max(-1.0, min(1.0, v / max_v))
        filled = int(abs(fraction) * (width // 2))
        if fraction >= 0:
            return " " * (width // 2) + "▶" * filled + " " * (width // 2 - filled)
        else:
            return " " * (width // 2 - filled) + "◀" * filled + " " * (width // 2)

    lines = [
        sep,
        "  ✋ AEROKINESIS DUAL MPU9250 GESTURE CONTROLLER",
        sep,
        "",
        "  IMU 1 (Right Hand)            IMU 2 (Left Hand)",
        f"  Pitch: {data.get('pitch1', 0.0):>+7.2f}°                Pitch: {data.get('pitch2', 0.0):>+7.2f}°",
        f"  Roll:  {data.get('roll1',  0.0):>+7.2f}°                Roll:  {data.get('roll2',  0.0):>+7.2f}°",
        f"  Gz:    {data.get('gz1', 0.0):>+7.2f} °/s             Gz:    {data.get('gz2', 0.0):>+7.2f} °/s",
        "",
        f"  Timestamp  {data.get('timestamp', 0)} ms",
        "",
        thin,
        "  Buttons",
    ]

    for name in BUTTON_NAMES:
        label = _BUTTON_LABELS.get(name, name)
        state = "■ PRESSED " if btn_states.get(name, 0) else "□ RELEASED"
        lines.append(f"    {label} : {state}")

    lines += [
        "",
        thin,
        f"  Gesture Control  [{gc_str}]",
        "",
        f"  Hand 1 (Right):",
        f"    Pitch : {gesture.pitch_deg:>+7.1f} °  → Fwd/Bk {gesture.vx:>+5.2f} m/s",
        f"    Roll  : {gesture.roll_deg:>+7.1f} °  → Lft/Rt {gesture.vy:>+5.2f} m/s",
        f"  Hand 2 (Left):",
        f"    Pitch : {gesture.pitch_deg_2:>+7.1f} °  → Thrst  {gesture.vz:>+5.2f} m/s",
        f"    Roll  : {gesture.roll_deg_2:>+7.1f} °  → Yaw    {gesture.yaw_rate:>+5.2f} rad/s",
        "",
        f"    Fwd/Bk  [{_bar(gesture.vx)}]",
        f"    Lft/Rt  [{_bar(gesture.vy)}]",
        f"    Thrust  [{_bar(gesture.vz)}]",
        f"    YawRate [{_bar(gesture.yaw_rate, max_v=1.0)}]",
        "",
        thin,
        f"  Drone  {arm_str}   Mode: {mav_mode}",
        "",
        sep,
    ]
    return "\n".join(lines)


# ─────────────────────────────────────────────────────────────────────────────
# Main ROS2 Node
# ─────────────────────────────────────────────────────────────────────────────

class IMUBridgeNode(Node):
    """
    Primary ROS2 node: IMU publisher + button handler + gesture drone control.

    Composes:
      SerialReader         — serial port I/O with auto-reconnect
      ButtonEdgeDetector   — rising-edge button events (0→1 only)
      GestureController    — complementary filter + angle → velocity mapping
      MAVProxyController   — MAVLink command interface to ArduPilot SITL
    """

    def __init__(self) -> None:
        super().__init__("imu_serial_bridge")

        # ── Declare all parameters ─────────────────────────────────────────────
        self.declare_parameter("serial_port",      "/dev/ttyUSB0")
        self.declare_parameter("baud_rate",        115200)
        self.declare_parameter("frame_id",         "imu_link")
        self.declare_parameter("publish_rate",     50.0)
        self.declare_parameter("mavproxy_address", "udp:127.0.0.1:14550")
        self.declare_parameter("mavproxy_enabled", True)
        self.declare_parameter("takeoff_altitude", 2.0)
        self.declare_parameter("gesture_enabled",  True)
        self.declare_parameter("velocity_gain",    1.5)
        self.declare_parameter("max_velocity",     2.0)
        self.declare_parameter("dead_zone_deg",    5.0)
        self.declare_parameter("yaw_gain",         1.0)
        self.declare_parameter("max_yaw_rate",     0.8)
        self.declare_parameter("filter_alpha",     0.96)
        self.declare_parameter("cmd_rate_hz",      20.0)

        # Read parameter values.
        serial_port      = self.get_parameter("serial_port").value
        baud_rate        = self.get_parameter("baud_rate").value
        self._frame_id   = self.get_parameter("frame_id").value
        mav_addr         = self.get_parameter("mavproxy_address").value
        mav_enabled      = self.get_parameter("mavproxy_enabled").value
        self._takeoff_alt= self.get_parameter("takeoff_altitude").value
        self._gc_enabled = self.get_parameter("gesture_enabled").value
        velocity_gain    = self.get_parameter("velocity_gain").value
        max_velocity     = self.get_parameter("max_velocity").value
        dead_zone_deg    = self.get_parameter("dead_zone_deg").value
        yaw_gain         = self.get_parameter("yaw_gain").value
        max_yaw_rate     = self.get_parameter("max_yaw_rate").value
        filter_alpha     = self.get_parameter("filter_alpha").value
        cmd_rate_hz      = self.get_parameter("cmd_rate_hz").value

        self.get_logger().info(
            f"IMU Bridge v2 | port={serial_port} | "
            f"mavproxy={'ON' if mav_enabled else 'OFF'} | "
            f"gesture={'ON' if self._gc_enabled else 'OFF'}"
        )

        # ── Component: Serial reader ──────────────────────────────────────────
        self._reader = SerialReader(
            port=serial_port,
            baud=baud_rate,
            logger=self.get_logger(),
        )

        # ── Component: Button edge detector ──────────────────────────────────
        self._buttons = ButtonEdgeDetector()

        # ── Component: Gesture controller ─────────────────────────────────────
        self._gesture = GestureController(
            velocity_gain=velocity_gain,
            max_velocity=max_velocity,
            dead_zone_deg=dead_zone_deg,
            yaw_gain=yaw_gain,
            max_yaw_rate=max_yaw_rate,
            filter_alpha=filter_alpha,
        )
        self._gesture.enabled = self._gc_enabled
        self._latest_gesture  = GestureOutput.zero()

        # ── Component: MAVProxy controller ───────────────────────────────────
        self._mav = MAVProxyController(
            connection_string=mav_addr,
            logger=self.get_logger(),
        )
        if mav_enabled:
            # Connect in a background thread so rclpy.spin() is not blocked.
            threading.Thread(
                target=self._mav.connect,
                daemon=True,
                name="mavproxy-connect",
            ).start()

        # ── QoS profile ───────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── Publishers ────────────────────────────────────────────────────────
        self._pub_json = self.create_publisher(String,      "/imu/raw_json",    sensor_qos)
        self._pub_imu1 = self.create_publisher(Imu,         "/imu1/data_raw",   sensor_qos)
        self._pub_imu2 = self.create_publisher(Imu,         "/imu2/data_raw",   sensor_qos)

        # ── Direct terminal handle ─────────────────────────────────────────────
        # Write to /dev/tty to bypass the ROS2 launch system stdout prefix.
        try:
            self._tty = open("/dev/tty", "w")
        except OSError:
            self._tty = sys.stdout

        # ── State ─────────────────────────────────────────────────────────────
        self._last_reconnect: float = 0.0

        # ── Timer 1: Serial read + IMU publish + button events (100 Hz) ───────
        self._serial_timer = self.create_timer(0.01, self._serial_callback)

        # ── Timer 2: Velocity command sender (default 20 Hz) ──────────────────
        cmd_period = 1.0 / max(1.0, cmd_rate_hz)
        self._vel_timer = self.create_timer(cmd_period, self._velocity_callback)

        # Initial display.
        self._print_waiting()

        # First serial connection attempt.
        self._reader.connect()

    # ── Timer 1: Serial read ──────────────────────────────────────────────────

    def _serial_callback(self) -> None:
        """Drain the serial buffer and process each complete JSON line."""
        # Reconnect if needed.
        if not self._reader.is_connected():
            now = time.monotonic()
            if now - self._last_reconnect >= _RECONNECT_INTERVAL_S:
                self._last_reconnect = now
                self._reader.connect()
                self._gesture.reset()  # Reset filter on reconnect.
            return

        try:
            while self._reader.bytes_available():
                line = self._reader.read_line()
                if line:
                    self._process_line(line)

        except serial.SerialException as exc:
            self.get_logger().error(f"Serial error: {exc} — reconnecting")
            self._reader.disconnect()
            self._gesture.reset()

    # ── Timer 2: Velocity command sender ─────────────────────────────────────

    def _velocity_callback(self) -> None:
        """
        Send velocity commands to the drone at cmd_rate_hz.

        Safety gate — only sends when ALL of these are true:
          1. MAVProxy is connected.
          2. Drone is ARMED.
          3. Drone is in GUIDED mode.
          4. Gesture control is enabled.
          5. A takeoff sequence is NOT currently in progress.

        Condition 5 is critical: NAV_TAKEOFF manages the climb internally.
        Sending SET_POSITION_TARGET_LOCAL_NED while climbing overrides the
        takeoff command and causes ArduPilot to auto-disarm.
        """
        if not self._mav.is_connected():
            return

        # Do NOT send velocity during a takeoff climb — it would cancel NAV_TAKEOFF.
        if self._mav.is_taking_off:
            return

        armed   = self._mav.is_armed
        guided  = self._mav.in_guided_mode
        gc_on   = self._gc_enabled

        if armed and guided and gc_on:
            g = self._latest_gesture
            self._mav.send_velocity(g.vx, g.vy, g.vz, g.yaw_rate)
        elif armed and guided:
            # Gesture off but in GUIDED — send zero to hover in place.
            self._mav.send_velocity(0.0, 0.0, 0.0, 0.0)


    # ── Line processing ───────────────────────────────────────────────────────

    def _process_line(self, line: str) -> None:
        """Parse one JSON line → publish topics → handle buttons → update gesture."""
        # Parse.
        try:
            data: dict = json.loads(line)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(f"JSON error: {exc}")
            return

        # Validate IMU fields.
        missing = [f for f in _REQUIRED_IMU_FIELDS if f not in data]
        if missing:
            self.get_logger().warning(f"Missing IMU fields: {missing}")
            return

        stamp = self.get_clock().now().to_msg()

        # Publish IMU topics.
        self._publish_raw_json(line)
        self._publish_imu(data, stamp)

        # Button edge detection → drone commands.
        if all(f in data for f in _REQUIRED_BUTTON_FIELDS):
            events = self._buttons.process(data)
            self._handle_button_events(events)

        # Gesture update (runs at serial rate ≈50 Hz → latest_gesture cached).
        self._latest_gesture = self._gesture.process(data, time.monotonic())

        # Display.
        self._print_display(
            data,
            self._buttons.current_state(),
            self._latest_gesture,
        )

    # ── Button → drone command mapping ────────────────────────────────────────

    def _handle_button_events(self, events: dict) -> None:
        """Map rising-edge button events to MAVProxy commands (one-shot)."""

        if events.get("arm_disarm"):
            self.get_logger().info("Button → ARM/DISARM toggle")
            self._mav.toggle_arm()

        if events.get("mode_toggle"):
            self.get_logger().info("Button → STABILIZE / GUIDED toggle")
            self._mav.toggle_stabilize_guided()

        if events.get("takeoff"):
            self.get_logger().info(f"Button → TAKEOFF {self._takeoff_alt:.1f} m")
            self._mav.takeoff(self._takeoff_alt)

        if events.get("rtl"):
            self.get_logger().info("Button → RTL (Return-to-Launch, stopping gesture control)")
            # Send zero velocity immediately, then switch to RTL mode.
            self._mav.send_velocity(0.0, 0.0, 0.0, 0.0)
            self._mav.rtl()

    # ── Publishers ────────────────────────────────────────────────────────────

    def _publish_raw_json(self, line: str) -> None:
        msg = String()
        msg.data = line
        self._pub_json.publish(msg)

    def _publish_imu(self, data: dict, stamp: RosTime) -> None:
        """
        Publish sensor_msgs/Imu for both hands.

        Firmware v4 (MPU9250) does NOT send raw accelerometer data.
        - linear_acceleration is set to 0 with covariance[0] = -1
          (ROS convention: field not populated).
        - orientation is built from firmware pitch/roll (yaw fixed at 0).
        - angular_velocity is converted from deg/s to rad/s.
        """
        import math
        _DEG_TO_RAD = math.pi / 180.0

        def _euler_to_quat(roll_deg: float, pitch_deg: float) -> tuple:
            """Convert roll/pitch (degrees) to quaternion (x,y,z,w), yaw=0."""
            r = math.radians(roll_deg)  * 0.5
            p = math.radians(pitch_deg) * 0.5
            y = 0.0
            cr, sr = math.cos(r), math.sin(r)
            cp, sp = math.cos(p), math.sin(p)
            cy, sy = math.cos(y), math.sin(y)
            w = cr * cp * cy + sr * sp * sy
            x = sr * cp * cy - cr * sp * sy
            y_ = cr * sp * cy + sr * cp * sy
            z = cr * cp * sy - sr * sp * cy
            return x, y_, z, w

        # Covariance flag: -1 in first element means field not populated
        _ACCEL_COV_UNKNOWN = [-1.0] + [0.0] * 8

        # ── IMU 1 (Right Hand) ──────────────────────────────────────────────
        pitch1 = float(data.get("pitch1", 0.0))
        roll1  = float(data.get("roll1",  0.0))
        qx, qy, qz, qw = _euler_to_quat(roll1, pitch1)

        msg1 = Imu()
        msg1.header.stamp    = stamp
        msg1.header.frame_id = self._frame_id + "_1"

        msg1.orientation.x = qx
        msg1.orientation.y = qy
        msg1.orientation.z = qz
        msg1.orientation.w = qw
        msg1.orientation_covariance = list(_GYRO_COV)  # small uncertainty

        # Raw accel not available in firmware v4 — signal not populated
        msg1.linear_acceleration.x = 0.0
        msg1.linear_acceleration.y = 0.0
        msg1.linear_acceleration.z = 0.0
        msg1.linear_acceleration_covariance = list(_ACCEL_COV_UNKNOWN)

        # Convert gyro from deg/s to rad/s
        msg1.angular_velocity.x = float(data.get("gx1", 0.0)) * _DEG_TO_RAD
        msg1.angular_velocity.y = float(data.get("gy1", 0.0)) * _DEG_TO_RAD
        msg1.angular_velocity.z = float(data.get("gz1", 0.0)) * _DEG_TO_RAD
        msg1.angular_velocity_covariance = list(_GYRO_COV)

        self._pub_imu1.publish(msg1)

        # ── IMU 2 (Left Hand) ───────────────────────────────────────────────
        pitch2 = float(data.get("pitch2", 0.0))
        roll2  = float(data.get("roll2",  0.0))
        qx2, qy2, qz2, qw2 = _euler_to_quat(roll2, pitch2)

        msg2 = Imu()
        msg2.header.stamp    = stamp
        msg2.header.frame_id = self._frame_id + "_2"

        msg2.orientation.x = qx2
        msg2.orientation.y = qy2
        msg2.orientation.z = qz2
        msg2.orientation.w = qw2
        msg2.orientation_covariance = list(_GYRO_COV)

        msg2.linear_acceleration.x = 0.0
        msg2.linear_acceleration.y = 0.0
        msg2.linear_acceleration.z = 0.0
        msg2.linear_acceleration_covariance = list(_ACCEL_COV_UNKNOWN)

        msg2.angular_velocity.x = float(data.get("gx2", 0.0)) * _DEG_TO_RAD
        msg2.angular_velocity.y = float(data.get("gy2", 0.0)) * _DEG_TO_RAD
        msg2.angular_velocity.z = float(data.get("gz2", 0.0)) * _DEG_TO_RAD
        msg2.angular_velocity_covariance = list(_GYRO_COV)

        self._pub_imu2.publish(msg2)

    # ── Display ───────────────────────────────────────────────────────────────

    def _print_waiting(self) -> None:
        sep = "=" * _DISPLAY_WIDTH
        banner = (
            "\033[H\033[J"
            + sep + "\n"
            + "  ✋ MPU6050 HAND GESTURE CONTROLLER\n"
            + sep + "\n\n"
            + "  Waiting for ESP32 data...\n"
            + "  (Check USB cable and firmware)\n\n"
            + sep + "\n"
        )
        self._tty.write(banner)
        self._tty.flush()

    def _print_display(
        self,
        data:       dict,
        btn_states: dict,
        gesture:    GestureOutput,
    ) -> None:
        """Write the dashboard directly to /dev/tty (no launch prefix)."""
        output = "\033[H\033[J" + _format_display(
            data,
            btn_states,
            gesture,
            self._mav.is_armed,
            self._mav.flight_mode,
            self._gc_enabled,
        ) + "\n"
        self._tty.write(output)
        self._tty.flush()

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def destroy_node(self) -> None:
        """Cleanly shut down all components."""
        # Zero velocity before disconnecting.
        if self._mav.is_connected() and self._mav.is_armed:
            self._mav.send_velocity(0.0, 0.0, 0.0, 0.0)
        self._reader.disconnect()
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
    node = IMUBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt — shutting down.")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
