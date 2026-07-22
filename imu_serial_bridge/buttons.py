"""
buttons.py — Button edge detection for the ESP32 button interface.

Tracks the previous state of each button and emits a 'pressed' event
exactly ONCE when the value transitions from 0 → 1 (released → pressed).

Holding a button down does NOT generate repeated events.
Releasing a button (1 → 0) is tracked but does not generate a command event.

Button names (must match JSON keys sent by the ESP32):
    arm_disarm  — ARM/DISARM toggle button
    force_arm   — Force arm (bypasses pre-arm checks)
    takeoff     — Takeoff button
    land        — Land button

Usage:
    detector = ButtonEdgeDetector()
    events   = detector.process(parsed_json_dict)
    # events = {'arm_disarm': False, 'force_arm': False,
    #           'takeoff': True,     'land': False}
    if events['takeoff']:
        controller.takeoff(2.0)

Future extensions:
  - Long-press detection (hold duration threshold).
  - Double-click detection.
  - Additional buttons (RTL, mode switch, etc.).

Author: Hand-Gesture Drone Project
Date:   2026
"""

from dataclasses import dataclass, field
from typing import Dict


# ─────────────────────────────────────────────────────────────────────────────
# Public API: names of all tracked buttons
# ─────────────────────────────────────────────────────────────────────────────

#: Ordered list of button field names as they appear in the ESP32 JSON.
BUTTON_NAMES = ("arm_disarm", "mode_toggle", "takeoff", "rtl")


@dataclass
class ButtonEdgeDetector:
    """
    Rising-edge detector for all four ESP32 push-buttons with software debouncing.

    Attributes
    ----------
    _current : dict
        Current raw state of each button (0 or 1) from the latest packet.
    _last_trigger_time : dict
        Timestamp of the last rising edge trigger for each button to enforce debounce lockout.
    """

    _current: Dict[str, int] = field(default_factory=lambda: {k: 0 for k in BUTTON_NAMES})
    _last_trigger_time: Dict[str, float] = field(default_factory=lambda: {k: 0.0 for k in BUTTON_NAMES})

    def process(self, data: dict) -> Dict[str, bool]:
        """
        Consume one parsed IMU/button JSON dict and return edge events.

        Only a 0→1 transition (button just pressed) generates True,
        subject to a debounce lockout period.
        Holding or releasing the button returns False.

        Parameters
        ----------
        data : dict
            Parsed JSON from the ESP32.  Button values are expected to be
            integers: 0 = released, 1 = pressed.

        Returns
        -------
        dict[str, bool]
            Mapping from button name → True if the button was *just* pressed
            this packet (rising edge detected), False otherwise.
        """
        import time
        events: Dict[str, bool] = {}
        now = time.monotonic()

        for name in BUTTON_NAMES:
            # Read the current state; default to 0 if the field is missing.
            current_val = int(data.get(name, 0))

            # Rising edge = previous was 0, current is 1.
            is_edge = (self._current[name] == 0) and (current_val == 1)

            # Debounce lockout: firmware already debounces at 40 ms (hardware).
            # Python-side lockout shortened to 200 ms to prevent ROS-level
            # duplicate events while remaining responsive.
            if is_edge and (now - self._last_trigger_time[name] > 0.2):
                events[name] = True
                self._last_trigger_time[name] = now
            else:
                events[name] = False

            # Update the state.
            self._current[name] = current_val

        return events

    def current_state(self) -> Dict[str, int]:
        """
        Return the most recently received raw button states.

        Returns
        -------
        dict[str, int]
            {button_name: 0_or_1}  for display purposes.
        """
        return dict(self._current)

    def is_pressed(self, name: str) -> bool:
        """
        Return whether a specific button is currently pressed.

        Parameters
        ----------
        name : str  Button name from BUTTON_NAMES.
        """
        return bool(self._current.get(name, 0))
