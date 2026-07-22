"""
serial_reader.py — Serial port reader for the ESP32 + MPU6050 bridge.

Encapsulates all pyserial logic so the ROS2 node stays clean.

Responsibilities:
  - Open / close the serial port.
  - Reconnect automatically on cable unplug.
  - Return raw decoded lines (stripped of whitespace).
  - Skip empty lines and ESP32 startup banner lines.

Usage:
    reader = SerialReader(port="/dev/ttyUSB0", baud=115200, logger=node.get_logger())
    reader.connect()
    ...
    line = reader.read_line()   # returns str or None
    reader.disconnect()

Thread safety:
    Not thread-safe by design — call from a single ROS2 timer callback.

Future extensions:
  - Add CRC / checksum validation per line.
  - Support multiple serial devices (multi-IMU).

Author: Hand-Gesture Drone Project
Date:   2026
"""

import serial
from typing import Optional


class SerialReader:
    """
    Manages a single pyserial connection to the ESP32.

    Parameters
    ----------
    port   : str   Device path, e.g. '/dev/ttyUSB0'.
    baud   : int   Baud rate matching the ESP32 firmware (115200).
    logger : any   ROS2-compatible logger (node.get_logger()) or None.
    """

    # Short read timeout keeps the timer callback non-blocking.
    _READ_TIMEOUT_S  = 0.05
    _WRITE_TIMEOUT_S = 1.0

    def __init__(self, port: str, baud: int, logger=None) -> None:
        self._port   = port
        self._baud   = baud
        self._log    = logger
        self._serial: Optional[serial.Serial] = None

    # ── Connection ────────────────────────────────────────────────────────────

    def connect(self) -> bool:
        """
        Attempt to open the serial port.

        Returns
        -------
        bool
            True on success, False if the device is unavailable.
        """
        try:
            self._serial = serial.Serial(
                port=self._port,
                baudrate=self._baud,
                timeout=self._READ_TIMEOUT_S,
                write_timeout=self._WRITE_TIMEOUT_S,
            )
            self._serial.reset_input_buffer()
            self._info(f"Serial opened: {self._port} @ {self._baud} baud")
            return True

        except serial.SerialException as exc:
            self._warn(f"Cannot open {self._port}: {exc}")
            self._serial = None
            return False

    def disconnect(self) -> None:
        """Close the serial port if it is open."""
        if self.is_connected():
            try:
                self._serial.close()
                self._info("Serial port closed.")
            except Exception:
                pass
        self._serial = None

    def is_connected(self) -> bool:
        """Return True if the serial port is currently open."""
        return self._serial is not None and self._serial.is_open

    # ── Reading ───────────────────────────────────────────────────────────────

    def bytes_available(self) -> int:
        """Return number of bytes waiting in the receive buffer."""
        if not self.is_connected():
            return 0
        try:
            return self._serial.in_waiting
        except serial.SerialException:
            return 0

    def read_line(self) -> Optional[str]:
        """
        Read one line from the serial buffer.

        Decodes bytes to ASCII (replacing unrecognised bytes), strips
        whitespace, and skips lines that are not JSON objects.

        Returns
        -------
        str or None
            A non-empty JSON line starting with '{', or None if no data
            is available or the line is a startup banner / empty string.

        Raises
        ------
        serial.SerialException
            Propagated to the caller on I/O errors so it can reconnect.
        """
        raw: bytes = self._serial.readline()

        if not raw:
            return None

        line = raw.decode("ascii", errors="replace").strip()

        # Ignore empty lines and ESP32 startup banner messages.
        if not line or not line.startswith("{"):
            return None

        return line

    # ── Logger helpers ────────────────────────────────────────────────────────

    def _info(self, msg: str) -> None:
        if self._log:
            self._log.info(msg)

    def _warn(self, msg: str) -> None:
        if self._log:
            self._log.warning(msg)
