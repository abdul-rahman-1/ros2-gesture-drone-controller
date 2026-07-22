# ESP32 Dual-IMU Primary Firmware (`firmware/mpu6050_esp32/`)

This directory contains the main Arduino C++ firmware file [`mpu6050_esp32.ino`](file:///home/user/ros2_ws/src/imu_serial_bridge/firmware/mpu6050_esp32/mpu6050_esp32.ino) for the wearable hand gesture controller.

---

## Technical Highlights

1. **Dual Sensor Support**: Connects to two MPU9250/6500 IMU modules on the same I2C bus using I2C addresses `0x68` (Right Hand) and `0x69` (Left Hand).
2. **WHO_AM_I Bypass**: Uses custom subclassing for MPU9250_WE to support clone chips returning WHO_AM_I ID `0x75`.
3. **Onboard Sensor Fusion**: Computes pitch and roll angles using a 50 Hz onboard Complementary Filter (α = 0.98).
4. **Boot Neutral Calibration**: Runs a 3-second hold-still auto-zero calibration upon boot up to subtract initial tilt offsets.
5. **Hardware Button Debouncing**: Debounces 4 push-buttons at 40 ms hardware interval.
6. **JSON Serial Streaming**: Formats sensor readings into a single-line JSON string at 50 Hz over USB Serial at 115200 baud.

---

## Dual IMU Wiring Diagram

```
ESP32 DevKit V1         IMU 1 (Right Hand)        IMU 2 (Left Hand)
───────────────────────────────────────────────────────────────────
3.3V             ─────▶ VCC                       VCC
GND              ─────▶ GND                       GND
GPIO 21 (SDA)    ─────▶ SDA                       SDA
GPIO 22 (SCL)    ─────▶ SCL                       SCL
GND              ─────▶ AD0 (Address 0x68)
3.3V             ───────────────────────────────▶ AD0 (Address 0x69)

Push Buttons (Wired with INPUT_PULLUP to GND):
GPIO 32 ─────▶ ARM/DISARM Button ─────▶ GND
GPIO 33 ─────▶ MODE TOGGLE Button ───▶ GND
GPIO 25 ─────▶ TAKEOFF Button ───────▶ GND
GPIO 26 ─────▶ RTL Button ───────────▶ GND
```

---

## JSON Payload Output Format

```json
{"timestamp":8150,"pitch1":0.020,"roll1":0.010,"gx1":-0.052,"gy1":-0.004,"gz1":0.363,"pitch2":-0.008,"roll2":0.004,"gx2":0.010,"gy2":-0.009,"gz2":-0.000,"arm_disarm":0,"mode_toggle":0,"takeoff":0,"rtl":0}
```
