"""
mavproxy_controller.py — MAVProxy / ArduPilot SITL interface.

Provides a clean Python class that translates high-level drone commands
(arm, takeoff, land, continuous velocity) into MAVLink messages via pymavlink.

Velocity Control
----------------
  send_velocity(vx, vy, vz, yaw_rate) sends SET_POSITION_TARGET_LOCAL_NED
  in MAV_FRAME_LOCAL_NED.  Call at ≥20 Hz while in GUIDED mode to keep
  the drone moving.  ArduPilot will stop the drone if commands stop for >3 s.

Architecture
------------
  [ROS2 Node] → [MAVProxyController] → [pymavlink UDP] → [MAVProxy] → [ArduPilot SITL]

Typical MAVProxy SITL setup:
    # Terminal 1: start ArduPilot SITL
    sim_vehicle.py -v ArduCopter --console --map

    # MAVProxy auto-starts; it listens on:
    #   master:  tcp:127.0.0.1:5760
    #   output:  udp:127.0.0.1:14550   ← our node connects here
    #            udp:127.0.0.1:14551   ← GCS (Mission Planner etc.)

Instantiation
-------------
    ctrl = MAVProxyController(
        connection_string="udp:127.0.0.1:14550",
        logger=node.get_logger(),       # optional
    )
    ctrl.connect()          # blocks up to timeout_s waiting for heartbeat
    ctrl.toggle_arm()
    ctrl.takeoff(2.0)
    ctrl.land()

Thread safety
-------------
    connect() is blocking and must be called from a background thread or
    before rclpy.spin().  All other methods are non-blocking MAVLink sends.

Requirements
------------
    pip3 install pymavlink

Future extensions:
  - Return-to-Launch (RTL) mode
  - Velocity / position setpoints (gesture → MAVROS velocity control)
  - Multi-vehicle support
  - Mission upload

Author: Hand-Gesture Drone Project
Date:   2026
"""

import threading
import time
from typing import Optional

# pymavlink is an optional dependency — the node still starts without it
# (drone control is simply disabled with a warning).
try:
    from pymavlink import mavutil
    _PYMAVLINK_AVAILABLE = True
except ImportError:
    _PYMAVLINK_AVAILABLE = False


# ─────────────────────────────────────────────────────────────────────────────
# Constants
# ─────────────────────────────────────────────────────────────────────────────

# Magic number accepted by ArduPilot to force-arm bypassing safety checks.
_FORCE_ARM_MAGIC = 21196

# Default altitude for takeoff (metres AGL).
_DEFAULT_TAKEOFF_ALT_M = 2.0

# How long to wait for a heartbeat before giving up.
_HEARTBEAT_TIMEOUT_S = 10.0

# How long the velocity-command suppression window lasts after NAV_TAKEOFF (s).
# ArduPilot needs this time to climb without interference from SET_POSITION_TARGET.
_TAKEOFF_VELOCITY_GUARD_S = 12.0

# Settle time between mode-change and NAV_TAKEOFF command (s).
_MODE_SETTLE_S = 1.0


# ─────────────────────────────────────────────────────────────────────────────
# MAVProxyController
# ─────────────────────────────────────────────────────────────────────────────

class MAVProxyController:
    """
    High-level MAVLink controller for ArduPilot SITL via MAVProxy.

    Parameters
    ----------
    connection_string : str
        pymavlink connection string, e.g. 'udp:127.0.0.1:14550'.
    logger            : optional
        ROS2 logger (node.get_logger()) or None.
    timeout_s         : float
        Seconds to wait for the first heartbeat on connect().
    """

    def __init__(
        self,
        connection_string: str = "udp:127.0.0.1:14550",
        logger=None,
        timeout_s: float = _HEARTBEAT_TIMEOUT_S,
    ) -> None:
        self._conn_str  = connection_string
        self._log       = logger
        self._timeout_s = timeout_s

        self._mav:          Optional[object] = None   # mavutil.mavlink_connection
        self._armed:        bool             = False
        self._flight_mode:  str              = "UNKNOWN"
        self._connected:    bool             = False
        self._lock:         threading.Lock   = threading.Lock()

        # Single-flight guard — True while a takeoff sequence is running.
        # Prevents multiple NAV_TAKEOFF commands and blocks velocity setpoints
        # that would cancel the climb.
        self._taking_off:   bool             = False
        self._takeoff_lock: threading.Lock   = threading.Lock()

        if not _PYMAVLINK_AVAILABLE:
            self._warn(
                "pymavlink is not installed — drone control disabled.\n"
                "  Install with:  pip3 install pymavlink"
            )

    # ── Connection ────────────────────────────────────────────────────────────

    def connect(self) -> bool:
        """
        Open a MAVLink connection and wait for the first heartbeat.

        This method blocks for up to timeout_s seconds.  Call it from a
        background thread so the ROS2 node continues spinning.

        Returns
        -------
        bool
            True if a heartbeat was received, False on timeout or error.
        """
        if not _PYMAVLINK_AVAILABLE:
            return False

        try:
            self._info(f"Connecting to MAVProxy at {self._conn_str} …")
            mav = mavutil.mavlink_connection(self._conn_str)

            # Block until first heartbeat (proves SITL is alive).
            msg = mav.recv_match(type="HEARTBEAT", blocking=True,
                                 timeout=self._timeout_s)
            if msg is None:
                self._warn(
                    f"No heartbeat received within {self._timeout_s}s. "
                    "Is MAVProxy / SITL running?"
                )
                return False

            with self._lock:
                self._mav       = mav
                self._connected = True

            self._info(
                f"MAVProxy connected | system={mav.target_system} "
                f"component={mav.target_component}"
            )

            # Start background thread to track armed state from HEARTBEAT.
            t = threading.Thread(target=self._heartbeat_monitor, daemon=True)
            t.start()

            return True

        except Exception as exc:
            self._warn(f"MAVProxy connection failed: {exc}")
            return False

    def is_connected(self) -> bool:
        """Return True if MAVProxy is connected and heartbeat was received."""
        return self._connected

    # ── Drone commands ────────────────────────────────────────────────────────

    def toggle_arm(self) -> None:
        """
        Toggle arm/disarm state.

        If the drone is currently armed, it will be disarmed.
        If disarmed, it will be armed (normal, with pre-arm checks).
        """
        if not self._check_connected("toggle_arm"):
            return
        if self._armed:
            self._info("CMD → DISARM")
            self._send_arm_command(arm=False, force=False)
        else:
            self._info("CMD → ARM")
            self._send_arm_command(arm=True, force=False)

    def takeoff(self, height: float = _DEFAULT_TAKEOFF_ALT_M) -> None:
        """
        Switch to GUIDED mode and command a vertical takeoff.

        Runs in a background daemon thread so it does not block the
        ROS2 single-threaded executor.

        A single-flight guard prevents multiple simultaneous takeoff threads:
        if takeoff is already in progress the call is silently dropped.

        The _taking_off flag suppresses SET_POSITION_TARGET_LOCAL_NED
        velocity commands for _TAKEOFF_VELOCITY_GUARD_S seconds so they
        do not cancel the NAV_TAKEOFF climb.

        Parameters
        ----------
        height : float
            Target altitude in metres above ground level (AGL).
        """
        if not self._check_connected("takeoff"):
            return

        # Single-flight guard: drop duplicate presses while climbing.
        with self._takeoff_lock:
            if self._taking_off:
                self._warn("Takeoff already in progress — ignoring duplicate command.")
                return
            self._taking_off = True

        def _takeoff_action() -> None:
            try:
                self._info(f"CMD → GUIDED mode then TAKEOFF to {height:.1f} m")
                self._set_mode("GUIDED")
                time.sleep(_MODE_SETTLE_S)   # Wait for GUIDED to register.
                self._send_command_long(
                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                    param1=0, param2=0, param3=0, param4=0,
                    param5=0, param6=0, param7=height,
                )
                self._info(
                    f"NAV_TAKEOFF sent — suppressing velocity commands for "
                    f"{_TAKEOFF_VELOCITY_GUARD_S:.0f} s while climbing."
                )
                # Keep the guard active so velocity commands don't fight the climb.
                time.sleep(_TAKEOFF_VELOCITY_GUARD_S)
            finally:
                with self._takeoff_lock:
                    self._taking_off = False
                self._info("Takeoff guard lifted — gesture velocity control active.")

        threading.Thread(target=_takeoff_action, daemon=True, name="takeoff").start()

    def land(self) -> None:
        """
        Command the drone to land by switching to LAND mode.

        ArduPilot's LAND mode handles the full descent and motor shutdown.
        """
        if not self._check_connected("land"):
            return
        self._info("CMD → LAND")
        self._set_mode("LAND")

    def rtl(self) -> None:
        """
        Command Return-To-Launch (RTL) flight mode.
        """
        if not self._check_connected("rtl"):
            return
        self._info("CMD → RTL")
        self._set_mode("RTL")

    def toggle_stabilize_guided(self) -> None:
        """
        Toggle the flight mode between STABILIZE and GUIDED.
        """
        if not self._check_connected("toggle_stabilize_guided"):
            return
        current = self.flight_mode
        target = "STABILIZE" if current == "GUIDED" else "GUIDED"
        self._info(f"CMD → MODE TOGGLE ({current} → {target})")
        self._set_mode(target)

    def set_guided_mode(self) -> None:
        """
        Switch to GUIDED mode.

        Required before sending velocity or position commands.
        Safe to call repeatedly — ArduPilot ignores redundant mode changes.
        """
        if not self._check_connected("set_guided_mode"):
            return
        self._set_mode("GUIDED")

    def send_velocity(
        self,
        vx: float,
        vy: float,
        vz: float,
        yaw_rate: float = 0.0,
    ) -> None:
        """
        Send a continuous velocity setpoint to the drone (gesture control).

        Uses MAVLink SET_POSITION_TARGET_LOCAL_NED in the LOCAL_NED frame:
            vx > 0  →  fly North  / forward
            vy > 0  →  fly East   / right
            vz > 0  →  fly Down   (negative = climb)
            yaw_rate > 0 →  rotate clockwise

        Call at ≥20 Hz.  ArduPilot stops the drone if it receives no
        velocity command for ~3 seconds.

        IMPORTANT: The drone must be armed and in GUIDED mode.

        Parameters
        ----------
        vx, vy, vz : float  Velocity in m/s (LOCAL_NED frame).
        yaw_rate   : float  Yaw rate in rad/s (+CW). Default 0 = hold heading.
        """
        if not self._check_connected("send_velocity"):
            return

        with self._lock:
            if self._mav is None:
                return
            mav = self._mav

        # ── type_mask: which fields to USE (0 = use, 1 = ignore) ─────────────
        # Ignore: position (bits 0-2), acceleration (bits 6-8), yaw angle (bit 10)
        # Use:    velocity (bits 3-5), yaw_rate (bit 11)
        #
        # Bit layout:
        #  bit  0  (1)    ignore px
        #  bit  1  (2)    ignore py
        #  bit  2  (4)    ignore pz
        #  bit  3  (8)    ignore vx   ← 0 = USE vx
        #  bit  4  (16)   ignore vy   ← 0 = USE vy
        #  bit  5  (32)   ignore vz   ← 0 = USE vz
        #  bit  6  (64)   ignore afx
        #  bit  7  (128)  ignore afy
        #  bit  8  (256)  ignore afz
        #  bit  9  (512)  force
        #  bit 10  (1024) ignore yaw angle  ← ignore forced yaw
        #  bit 11  (2048) ignore yaw_rate   ← 0 = USE yaw_rate
        TYPE_MASK_VELOCITY_YAW_RATE = (
            1 | 2 | 4          # ignore position
            | 64 | 128 | 256   # ignore acceleration
            | 1024             # ignore yaw (use yaw_rate instead)
        )  # = 0x05C7 = 1479

        mav.mav.set_position_target_local_ned_send(
            int(time.time() * 1000) & 0xFFFFFFFF,  # time_boot_ms (wraps OK)
            mav.target_system,
            mav.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,   # coordinate frame
            TYPE_MASK_VELOCITY_YAW_RATE,
            0.0, 0.0, 0.0,       # position (ignored)
            vx, vy, vz,          # velocity setpoints (m/s)
            0.0, 0.0, 0.0,       # acceleration (ignored)
            0.0,                 # yaw angle (ignored)
            yaw_rate,            # yaw rate (rad/s)
        )

    # ── Internal helpers ──────────────────────────────────────────────────────

    def _send_arm_command(self, arm: bool, force: bool) -> None:
        """
        Send MAV_CMD_COMPONENT_ARM_DISARM.

        Parameters
        ----------
        arm   : bool  True = arm, False = disarm.
        force : bool  True = bypass safety checks (param2 = 21196).
        """
        param1 = 1.0 if arm else 0.0
        param2 = float(_FORCE_ARM_MAGIC) if force else 0.0
        self._send_command_long(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            param1=param1, param2=param2,
            param3=0, param4=0, param5=0, param6=0, param7=0,
        )

    def _set_mode(self, mode_name: str) -> None:
        """
        Switch the flight mode by name (e.g. 'GUIDED', 'LAND', 'STABILIZE').

        Parameters
        ----------
        mode_name : str  ArduCopter mode name (case-insensitive).
        """
        with self._lock:
            if self._mav is None:
                return
            mav = self._mav

        mapping = mav.mode_mapping()
        if mapping is None or mode_name not in mapping:
            self._warn(f"Unknown flight mode: '{mode_name}'")
            return

        mode_id = mapping[mode_name]
        mav.mav.set_mode_send(
            mav.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )
        self._info(f"Mode change → {mode_name} (id={mode_id})")

    def _send_command_long(
        self,
        command: int,
        param1: float = 0, param2: float = 0,
        param3: float = 0, param4: float = 0,
        param5: float = 0, param6: float = 0,
        param7: float = 0,
    ) -> None:
        """
        Send a MAVLink COMMAND_LONG message to the autopilot.

        Parameters match the MAVLink COMMAND_LONG definition.
        """
        with self._lock:
            if self._mav is None:
                return
            mav = self._mav

        mav.mav.command_long_send(
            mav.target_system,
            mav.target_component,
            command,
            0,           # confirmation = 0 (first transmission)
            param1, param2, param3, param4, param5, param6, param7,
        )

    def _heartbeat_monitor(self) -> None:
        """
        Background thread: listens for HEARTBEAT messages to track
        the armed state and flight mode of the drone.
        """
        while self._connected:
            try:
                with self._lock:
                    mav = self._mav
                if mav is None:
                    break

                msg = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=2.0)
                if msg:
                    armed = bool(
                        msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    )
                    if armed != self._armed:
                        self._armed = armed
                        self._info(f"Drone {'ARMED' if armed else 'DISARMED'}")

                    # Track current flight mode name.
                    if mav.flightmode:
                        self._flight_mode = mav.flightmode

            except Exception as exc:
                self._warn(f"Heartbeat monitor error: {exc}")
                time.sleep(1.0)

    def _check_connected(self, cmd_name: str) -> bool:
        """
        Verify the controller is connected before sending a command.

        Logs a warning if not connected.

        Parameters
        ----------
        cmd_name : str  Human-readable command name for the warning message.
        """
        if not self._connected or self._mav is None:
            self._warn(
                f"Cannot execute '{cmd_name}': MAVProxy not connected. "
                "Is SITL running?  Check connection_string parameter."
            )
            return False
        return True

    # ── State properties ──────────────────────────────────────────────────────

    @property
    def is_armed(self) -> bool:
        """True if the drone is currently armed (from last heartbeat)."""
        return self._armed

    @property
    def flight_mode(self) -> str:
        """Current flight mode string (e.g. 'GUIDED', 'LAND', 'STABILIZE')."""
        return self._flight_mode

    @property
    def in_guided_mode(self) -> bool:
        """True if the drone is currently in GUIDED mode."""
        return self._flight_mode == "GUIDED"

    @property
    def is_taking_off(self) -> bool:
        """
        True while a NAV_TAKEOFF sequence is in progress.

        The velocity callback checks this to avoid sending
        SET_POSITION_TARGET_LOCAL_NED commands that would cancel the climb.
        Automatically becomes False _TAKEOFF_VELOCITY_GUARD_S seconds after
        NAV_TAKEOFF is sent.
        """
        return self._taking_off
    # ── Logger helpers ────────────────────────────────────────────────────────

    def _info(self, msg: str) -> None:
        if self._log:
            self._log.info(msg)
        else:
            print(f"[MAVProxy] {msg}")

    def _warn(self, msg: str) -> None:
        if self._log:
            self._log.warning(msg)
        else:
            print(f"[MAVProxy] WARN: {msg}")
