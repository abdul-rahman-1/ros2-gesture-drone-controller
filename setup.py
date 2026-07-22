"""
setup.py — Build and install configuration for the imu_serial_bridge ROS2 package.

Uses ament_python convention expected by colcon for ROS2 Humble (Python packages).

Entry points:
  imu_node        →  imu_serial_bridge.imu_node:main        (primary — IMU + buttons + MAVProxy)
  imu_serial_node →  imu_serial_bridge.serial_node:main     (legacy — IMU only)
"""

from setuptools import find_packages, setup
import os
from glob import glob

PACKAGE_NAME = "imu_serial_bridge"

setup(
    name=PACKAGE_NAME,
    version="2.0.0",
    packages=find_packages(exclude=["test"]),

    # ── Data files installed alongside the Python package ──────────────────
    data_files=[
        # ROS2 resource index registration (required by ament).
        ("share/ament_index/resource_index/packages",
         [f"resource/{PACKAGE_NAME}"]),

        # Package manifest.
        (f"share/{PACKAGE_NAME}", ["package.xml"]),

        # Launch files.
        (f"share/{PACKAGE_NAME}/launch",
         glob("launch/*.launch.py")),

        # Config / parameter files (YAML).
        (f"share/{PACKAGE_NAME}/config",
         glob("config/*.yaml")),
    ],

    install_requires=[
        "setuptools",
        "pyserial",    # USB serial communication with the ESP32
    ],

    zip_safe=True,

    maintainer="Gesture Drone Dev",
    maintainer_email="dev@gesture-drone.local",
    description=(
        "ROS2 Humble node bridging ESP32 + MPU6050 IMU + 4 push-buttons "
        "to /imu/data_raw, /imu/temperature, /imu/raw_json topics with "
        "MAVProxy / ArduPilot SITL drone control."
    ),
    license="Apache-2.0",

    # ── Console scripts ────────────────────────────────────────────────────
    # Each entry point creates an executable in the ROS2 install space so
    # the node can be launched with `ros2 run` or from a launch file.
    entry_points={
        "console_scripts": [
            # PRIMARY: IMU + 4 buttons + MAVProxy / ArduPilot SITL drone control
            "imu_node        = imu_serial_bridge.imu_node:main",

            # LEGACY: IMU-only node (no buttons, no MAVProxy)
            "imu_serial_node = imu_serial_bridge.serial_node:main",

            # TODO-EXTEND: add future nodes here, e.g.:
            # "orientation_filter_node = imu_serial_bridge.orientation_filter:main",
        ],
    },
)
