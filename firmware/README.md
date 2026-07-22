# ESP32 Firmware Directory (`firmware/`)

This directory contains Arduino firmware code and hardware test scripts for the **ESP32 Dual-IMU Wearable Controller**.

---

## Directories & Subfolders

* **[`mpu6050_esp32/`](file:///home/user/ros2_ws/src/imu_serial_bridge/firmware/mpu6050_esp32)**: The main production firmware directory containing [`mpu6050_esp32.ino`](file:///home/user/ros2_ws/src/imu_serial_bridge/firmware/mpu6050_esp32/mpu6050_esp32.ino). Reads dual IMUs (MPU9250/6500 chips, WHO_AM_I `0x75`), runs onboard complementary filter sensor fusion (α=0.98), performs 3-second neutral calibration at boot, debounces 4 push-buttons at 40ms, and streams JSON at 50 Hz over USB Serial.

---

## Hardware Requirements & Wiring Overview

* **Microcontroller**: ESP32 DevKit V1
* **IMU 1 (Right Hand)**: MPU9250 / MPU6500 (AD0 → GND, I2C Address `0x68`)
* **IMU 2 (Left Hand)**: MPU9250 / MPU6500 (AD0 → 3.3V, I2C Address `0x69`)
* **Shared Pins**: SDA → GPIO 21, SCL → GPIO 22, VCC → 3.3V, GND → GND
* **Push Buttons**: 4 × Momentary Switches (ARM/DISARM: GPIO 32, MODE TOGGLE: GPIO 33, TAKEOFF: GPIO 25, RTL: GPIO 26)
