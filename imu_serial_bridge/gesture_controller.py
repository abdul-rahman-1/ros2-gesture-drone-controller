"""
gesture_controller.py — Hand gesture to drone velocity mapping.

Firmware Migration Note (v4 — MPU9250/6500 clone, WHO_AM_I=0x75):
  Sensor fusion (complementary filter, alpha=0.98) and neutral calibration
  are now performed ON the ESP32 at boot (3-second hold-still window).
  This node no longer needs to fuse raw accel/gyro data.
  It simply reads the pre-computed pitch/roll angles (degrees, already
  zero-referenced to the neutral pose) and maps them to NED velocity commands.

JSON fields consumed:
  pitch1, roll1  — Right Hand (IMU 1, 0x68) -> vx (forward/back), vy (left/right)
  pitch2, roll2  — Left Hand  (IMU 2, 0x69) -> vz (throttle),     yaw_rate
  gx1..gz2       — Angular velocities in deg/s (converted to rad/s internally)

Physical Mapping:
  Right hand tilt forward  (pitch1 up)   -> Fly forward   (+vx)
  Right hand tilt backward (pitch1 down) -> Fly backward  (-vx)
  Right hand tilt right    (roll1 right) -> Fly right     (+vy)
  Right hand tilt left     (roll1 left)  -> Fly left      (-vy)
  Left  hand tilt forward  (pitch2 up)   -> Climb         (-vz, NED up=negative)
  Left  hand tilt backward (pitch2 down) -> Descend       (+vz)
  Left  hand twist CW      (roll2 right) -> Rotate CW     (+yaw_rate)
  Left  hand twist CCW     (roll2 left)  -> Rotate CCW    (-yaw_rate)
  Either hand flat (within dead zone)    -> Hover / hold

Author: AeroKinesis Project
Date:   2026
"""

import math
from dataclasses import dataclass


# deg/s -> rad/s conversion constant
_DEG_TO_RAD = math.pi / 180.0

# Sensitivity multipliers applied on top of velocity_gain.
# Right Hand (fwd/back, left/right): 1.2x
# Left Hand  (throttle, yaw):        1.0x  (normal)
_GAIN_RIGHT = 1.2
_GAIN_LEFT  = 1.0


# ─────────────────────────────────────────────────────────────────────────────
# Gesture Output (value object)
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class GestureOutput:
    """
    Snapshot of gesture controller output for one IMU packet.

    Attributes
    ----------
    vx, vy, vz   : float  Velocity commands in LOCAL_NED (m/s).
    yaw_rate      : float  Yaw rate command (rad/s, +CW).
    pitch_deg     : float  Right hand pitch angle (degrees, from firmware).
    roll_deg      : float  Right hand roll angle (degrees, from firmware).
    pitch_deg_2   : float  Left hand pitch angle (degrees, from firmware).
    roll_deg_2    : float  Left hand roll angle (degrees, from firmware).
    is_active     : bool   True if any non-zero velocity is commanded.
    """
    vx:          float = 0.0
    vy:          float = 0.0
    vz:          float = 0.0
    yaw_rate:    float = 0.0
    pitch_deg:   float = 0.0
    roll_deg:    float = 0.0
    pitch_deg_2: float = 0.0
    roll_deg_2:  float = 0.0
    is_active:   bool  = False

    @classmethod
    def zero(cls) -> "GestureOutput":
        """Return a zero-velocity (hover/stop) output."""
        return cls()


# ─────────────────────────────────────────────────────────────────────────────
# Gesture Controller
# ─────────────────────────────────────────────────────────────────────────────

class GestureController:
    """
    Maps pre-fused MPU9250 pitch/roll angles to drone velocity commands.

    The firmware (ESP32) now handles sensor fusion and neutral calibration
    onboard.  This class only applies:
      - Dead-zone filtering (angles below dead_zone_deg are treated as zero)
      - Linear gain scaling (velocity_gain m/s per degree)
      - Hard velocity clamp (max_velocity)
      - Sensitivity multipliers (right hand 1.8x, left hand 1.0x)
      - Gyro unit conversion: deg/s -> rad/s for angular_velocity publishing

    Parameters
    ----------
    velocity_gain   : float  m/s of drone velocity per degree of hand tilt.
    max_velocity    : float  Maximum allowed horizontal velocity (m/s).
    dead_zone_deg   : float  Tilt angle below which velocity = 0 (degrees).
    yaw_gain        : float  Yaw rate scaling factor.
    max_yaw_rate    : float  Maximum allowed yaw rate (rad/s).
    """

    def __init__(
        self,
        velocity_gain:  float = 0.015,  # m/s per degree of tilt (tuned for MPU9250)
        max_velocity:   float = 2.0,
        dead_zone_deg:  float = 5.0,    # degrees; firmware doesn't deadzone
        yaw_gain:       float = 0.04,   # rad/s per effective degree of roll2
        max_yaw_rate:   float = 0.8,    # rad/s ~= 46 deg/s
        # filter_alpha kept as parameter for API compatibility but unused
        filter_alpha:   float = 0.96,
    ) -> None:
        self._gain         = velocity_gain
        self._max_vel      = max_velocity
        self._dead_zone    = dead_zone_deg   # degrees (firmware pitch/roll in deg)
        self._yaw_gain     = yaw_gain
        self._max_yaw_rate = max_yaw_rate
        self._enabled:     bool = True

    # ── Public interface ──────────────────────────────────────────────────────

    @property
    def enabled(self) -> bool:
        """True if gesture control is currently active."""
        return self._enabled

    @enabled.setter
    def enabled(self, value: bool) -> None:
        self._enabled = value

    def reset(self) -> None:
        """No-op — kept for API compatibility with imu_node.py reconnect logic."""
        pass

    def process(self, data: dict, current_time: float) -> GestureOutput:
        """
        Compute velocity commands from one pre-fused IMU JSON packet.

        Parameters
        ----------
        data         : dict   Parsed JSON from the ESP32.
                              Expected keys: pitch1, roll1, pitch2, roll2
                              Optional keys: gx1..gz2 (deg/s)
        current_time : float  Monotonic timestamp (seconds) — unused but kept
                              for API compatibility.

        Returns
        -------
        GestureOutput
            Velocity + yaw_rate commands. Returns GestureOutput.zero() if
            gesture control is disabled.
        """
        if not self._enabled:
            return GestureOutput.zero()

        # ── Read pre-fused angles (degrees, firmware-calibrated to neutral=0) ─
        pitch1 = float(data.get("pitch1", 0.0))
        roll1  = float(data.get("roll1",  0.0))
        pitch2 = float(data.get("pitch2", 0.0))
        roll2  = float(data.get("roll2",  0.0))

        # ── RIGHT HAND (IMU 1) -> Horizontal velocity ─────────────────────────
        vx = self._angle_to_velocity(pitch1, _GAIN_RIGHT)
        vy = self._angle_to_velocity(roll1,  _GAIN_RIGHT)

        # ── LEFT HAND (IMU 2) -> Throttle and Yaw ────────────────────────────
        # pitch2 forward => climb (NED vz is negative = up)
        vz_raw = self._angle_to_velocity(pitch2, _GAIN_LEFT)
        vz = -vz_raw   # invert: tilt forward => climb (negative NED)

        # roll2 => yaw rate
        yaw_rate = self._roll_to_yaw(roll2)

        is_active = (
            abs(vx) > 0.0 or abs(vy) > 0.0
            or abs(vz) > 0.0 or abs(yaw_rate) > 0.0
        )

        return GestureOutput(
            vx=vx,
            vy=vy,
            vz=vz,
            yaw_rate=yaw_rate,
            pitch_deg=pitch1,
            roll_deg=roll1,
            pitch_deg_2=pitch2,
            roll_deg_2=roll2,
            is_active=is_active,
        )

    @staticmethod
    def gyro_to_rad_s(deg_s: float) -> float:
        """
        Convert gyroscope reading from deg/s (MPU9250_WE library) to rad/s.

        MPU9250_WE getGyrValues() returns deg/s.  sensor_msgs/Imu expects rad/s.
        """
        return deg_s * _DEG_TO_RAD

    # ── Private helpers ───────────────────────────────────────────────────────

    def _angle_to_velocity(self, angle_deg: float, sensitivity: float) -> float:
        """
        Apply dead zone and linear gain to map a tilt angle (degrees) to velocity.

        Dead zone: angles below self._dead_zone produce zero velocity.
        Outside the dead zone the effective angle is multiplied by the gain
        and the sensitivity multiplier, then clamped to max_velocity.

        Parameters
        ----------
        angle_deg   : float  Tilt angle in degrees (firmware pre-calibrated).
        sensitivity : float  Extra multiplier (_GAIN_RIGHT or _GAIN_LEFT).

        Returns
        -------
        float  Velocity command in m/s.
        """
        if abs(angle_deg) < self._dead_zone:
            return 0.0

        sign          = 1.0 if angle_deg > 0.0 else -1.0
        effective_deg = sign * (abs(angle_deg) - self._dead_zone)
        velocity      = effective_deg * self._gain * sensitivity

        return self._clamp(velocity, self._max_vel)

    def _roll_to_yaw(self, roll2_deg: float) -> float:
        """
        Map left-hand roll angle to yaw rate (rad/s).

        yaw_gain is expressed as rad/s per effective degree (after dead zone).
        Example with default yaw_gain=0.04:
          roll2=20° -> eff=15° -> rate = 15 * 0.04 = 0.60 rad/s
          roll2=30° -> eff=25° -> rate = 25 * 0.04 = 1.00 rad/s (clamped to 0.8)

        Parameters
        ----------
        roll2_deg : float  Left hand roll in degrees.

        Returns
        -------
        float  Yaw rate in rad/s, clamped to max_yaw_rate.
        """
        if abs(roll2_deg) < self._dead_zone:
            return 0.0

        sign = 1.0 if roll2_deg > 0.0 else -1.0
        eff  = sign * (abs(roll2_deg) - self._dead_zone)
        # Direct: yaw_gain = rad/s per effective degree (no extra DEG_TO_RAD)
        rate = eff * self._yaw_gain

        return self._clamp(rate, self._max_yaw_rate)

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        """Clamp value to [-limit, +limit]."""
        return max(-limit, min(limit, value))
