/**
 * @file mpu9250_esp32.ino
 * @brief ESP32 firmware: Dual MPU9250 IMU + 4 push-button gesture interface.
 *
 * Reads both IMUs, runs a complementary filter for stable pitch/roll tilt
 * angles (no raw-accel noise, no long-term gyro drift), applies a
 * neutral-position calibration offset, and serialises everything as one
 * JSON object per line at 50 Hz on USB Serial.
 *
 * Hardware
 * --------
 * IMUs (x2, MPU9250) →  ESP32
 *   VCC     →  3.3 V (Both)
 *   GND     →  GND (Both)
 *   SDA     →  GPIO 21 (Both)
 *   SCL     →  GPIO 22 (Both)
 *
 *   IMU 1 AD0 → GND   (I2C address = 0x68) (Right Hand - Pitch/Roll)
 *   IMU 2 AD0 → 3.3 V (I2C address = 0x69) (Left Hand  - Throttle/Yaw)
 *
 * Buttons   →  ESP32          →  GND
 *   ARM_DISARM  GPIO 32
 *   FORCE_ARM   GPIO 33
 *   TAKEOFF     GPIO 25
 *   LAND        GPIO 26
 *
 * All buttons use INPUT_PULLUP and are software-debounced (see DEBOUNCE_MS).
 *
 * Sensor fusion
 * -------------
 * Each IMU's pitch/roll is computed with a complementary filter:
 *   angle = ALPHA * (angle_prev + gyroRate * dt) + (1 - ALPHA) * accelAngle
 * This gives an angle that is immune to accelerometer vibration noise AND
 * immune to long-term gyro integration drift. Magnetometer/yaw-heading is
 * intentionally NOT used — per project design, "yaw" here is wrist-tilt
 * angle (a stick-deflection command), not a compass heading, so no
 * magnetometer fusion or drift-prone absolute yaw is needed.
 *
 * Neutral calibration
 * --------------------
 * On boot, the user holds both arms in their resting/neutral position for
 * CALIBRATION_DURATION_MS. The average tilt angle over that window is
 * stored as the zero-offset for each axis, so "arms at rest" always reports
 * ~0 degrees regardless of how the sensor is physically strapped on.
 *
 * JSON output example (one line per packet at 50 Hz):
 * {"timestamp":123456,"pitch1":2.310,"roll1":-1.040,"gx1":0.020,"gy1":-0.010,"gz1":0.000,
 *  "pitch2":0.150,"roll2":0.400,"gx2":0.000,"gy2":0.000,"gz2":0.000,
 *  "arm_disarm":0,"mode_toggle":0,"takeoff":0,"rtl":0}
 *
 * Design rules:
 *   - ESP32 is ONLY a sensor/button interface; no drone logic here.
 *   - All drone decisions are made inside the ROS2 node.
 *   - After "Both MPU9250s Ready", ONLY JSON is printed.
 *
 * Libraries required (install via Arduino Library Manager):
 *   MPU9250_WE  (by Wolfgang Ewald)
 *
 * Future extensions:
 *   - Recalibrate-on-button-hold (e.g. hold FORCE_ARM 2s to re-zero)
 *   - Battery voltage ADC channel
 *   - BLE / Wi-Fi transport layer
 *
 * @author  Hand-Gesture Drone Project
 * @date    2026
 */

#include <Wire.h>
#include <MPU9250_WE.h>


// Thin subclass that exposes the protected init(expectedValue) publicly,
// so we can tell the library to accept our board's actual WHO_AM_I (0x75)
// instead of the hardcoded 0x71 default — without editing library files.
class MPU9250_WE_Custom : public MPU9250_WE {
public:
  explicit MPU9250_WE_Custom(uint8_t addr) : MPU9250_WE(addr) {}

  bool initWithId(uint8_t expectedValue) {
    return MPU6500_WE::init(expectedValue); // explicit qualification bypasses name-hiding
  }
};

// ─────────────────────────────────────────────────────────────────────────────
// Constants
// ─────────────────────────────────────────────────────────────────────────────

static constexpr uint32_t SERIAL_BAUD_RATE   = 115200;
static constexpr uint32_t PUBLISH_RATE_HZ    = 50;
static constexpr uint32_t PUBLISH_PERIOD_US  = 1000000UL / PUBLISH_RATE_HZ;

// I2C pins (ESP32 DevKit V1 defaults).
static constexpr int I2C_SDA_PIN = 21;
static constexpr int I2C_SCL_PIN = 22;

// I2C addresses.
static constexpr uint8_t MPU1_ADDR = 0x68; // Right hand: pitch/roll
static constexpr uint8_t MPU2_ADDR = 0x69; // Left hand: throttle/yaw

// Button GPIO pins — all use internal pull-up; pressing connects pin to GND.
static constexpr int BTN_ARM_DISARM = 32;
static constexpr int BTN_FORCE_ARM  = 33;
static constexpr int BTN_TAKEOFF    = 25;
static constexpr int BTN_LAND       = 26;

// Debounce window: ignore state changes faster than this (ms).
static constexpr uint32_t DEBOUNCE_MS = 40;

// MPU9250 initialisation retry settings.
static constexpr uint8_t  INIT_RETRY_COUNT    = 5;
static constexpr uint32_t INIT_RETRY_DELAY_MS = 1000;

// How often to verify both IMUs are still alive on the bus (ms).
static constexpr uint32_t I2C_HEALTHCHECK_PERIOD_MS = 1000;

// Complementary filter blend: closer to 1.0 = trust gyro more (smoother,
// slower to correct); closer to 0.0 = trust accel more (noisier, no drift).
static constexpr float COMP_FILTER_ALPHA = 0.98f;

// Neutral-position calibration window on boot.
static constexpr uint32_t CALIBRATION_DURATION_MS = 3000;
static constexpr uint32_t CALIBRATION_SAMPLE_PERIOD_MS = 10; // ~100 Hz sampling during calib

// ─────────────────────────────────────────────────────────────────────────────
// Global objects
// ─────────────────────────────────────────────────────────────────────────────

MPU9250_WE_Custom mpu1 = MPU9250_WE_Custom(MPU1_ADDR);
MPU9250_WE_Custom mpu2 = MPU9250_WE_Custom(MPU2_ADDR);

static uint32_t lastPublishUs = 0;
static uint32_t lastHealthCheckMs = 0;
static uint32_t lastFilterUpdateUs = 0;

// Complementary-filter state (fused, calibrated angles in degrees).
static float pitch1 = 0.0f, roll1 = 0.0f;
static float pitch2 = 0.0f, roll2 = 0.0f;

// Neutral-position offsets captured during boot calibration.
static float pitch1Offset = 0.0f, roll1Offset = 0.0f;
static float pitch2Offset = 0.0f, roll2Offset = 0.0f;

// Debounce state, one entry per button.
struct DebouncedButton {
  int      pin;
  uint8_t  stableState;   // last accepted (debounced) state: 0 or 1
  uint8_t  lastRawState;  // last raw reading
  uint32_t lastChangeMs;  // when lastRawState last changed
};

static DebouncedButton btnArmDisarm { BTN_ARM_DISARM, 0, 0, 0 };
static DebouncedButton btnForceArm  { BTN_FORCE_ARM,  0, 0, 0 };
static DebouncedButton btnTakeoff   { BTN_TAKEOFF,    0, 0, 0 };
static DebouncedButton btnLand      { BTN_LAND,       0, 0, 0 };

// ─────────────────────────────────────────────────────────────────────────────
// Forward declarations
// ─────────────────────────────────────────────────────────────────────────────

bool    initMPU9250s();
bool    i2cDeviceResponding(uint8_t addr);
void    computeAccelAngles(MPU9250_WE &mpu, float &accelPitch, float &accelRoll);
void    updateComplementaryFilter(float dt);
void    calibrateNeutralPosition();
uint8_t updateDebouncedButton(DebouncedButton &btn);
void    publishJSON(float p1, float r1, xyzFloat g1,
                    float p2, float r2, xyzFloat g2);

// ─────────────────────────────────────────────────────────────────────────────
// setup()
// ─────────────────────────────────────────────────────────────────────────────

void setup() {
  // ── Serial ────────────────────────────────────────────────────────────────
  Serial.begin(SERIAL_BAUD_RATE);
  while (!Serial) { ; }

  // ── Button GPIO ───────────────────────────────────────────────────────────
  pinMode(BTN_ARM_DISARM, INPUT_PULLUP);
  pinMode(BTN_FORCE_ARM,  INPUT_PULLUP);
  pinMode(BTN_TAKEOFF,    INPUT_PULLUP);
  pinMode(BTN_LAND,       INPUT_PULLUP);

  // ── I2C ───────────────────────────────────────────────────────────────────
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  // ── MPU9250 init (with retry) ─────────────────────────────────────────────
  Serial.println("Initializing MPU9250s...");
  delay(300); // let both IMUs finish power-on reset before any I2C traffic

  Serial.println("Initializing MPU9250s...");
  bool initialized = false;
  for (uint8_t attempt = 0; attempt < INIT_RETRY_COUNT; ++attempt) {
    if (initMPU9250s()) {
      initialized = true;
      break;
    }
    Serial.print("One or both MPU9250s not found, retrying (");
    Serial.print(attempt + 1);
    Serial.print("/");
    Serial.print(INIT_RETRY_COUNT);
    Serial.println(")...");
    delay(INIT_RETRY_DELAY_MS);
  }

  if (!initialized) {
    Serial.println("FATAL: MPU9250 initialization failed. Check wiring and AD0 pins.");
    while (true) { delay(1000); }
  }

  // ── Seed complementary filter with a first accel-only reading ─────────────
  // Prevents a garbage jump from angle=0 on the very first loop iteration.
  float ap1, ar1, ap2, ar2;
  computeAccelAngles(mpu1, ap1, ar1);
  computeAccelAngles(mpu2, ap2, ar2);
  pitch1 = ap1; roll1 = ar1;
  pitch2 = ap2; roll2 = ar2;

  // ── Neutral-position calibration ───────────────────────────────────────────
  Serial.println("Hold BOTH arms in neutral resting position for calibration...");
  delay(1000); // give the user a moment to get into position
  calibrateNeutralPosition();
  Serial.println("Calibration complete.");

  Serial.println("Both MPU9250s Ready");
  // ── After this line, ONLY JSON is printed ──────────────────────────────────

  lastPublishUs = micros();
  lastFilterUpdateUs = micros();
  lastHealthCheckMs = millis();
}

// ─────────────────────────────────────────────────────────────────────────────
// loop()
// ─────────────────────────────────────────────────────────────────────────────

void loop() {
  const uint32_t nowUs = micros();
  const uint32_t nowMs = millis();

  // ── Periodic I2C health check ────────────────────────────────────────────
  if ((nowMs - lastHealthCheckMs) >= I2C_HEALTHCHECK_PERIOD_MS) {
    lastHealthCheckMs = nowMs;
    if (!i2cDeviceResponding(MPU1_ADDR) || !i2cDeviceResponding(MPU2_ADDR)) {
      // Attempt a quiet re-init. Calibration offsets are preserved, since the
      // physical strap-on orientation hasn't changed, only the bus dropped.
      initMPU9250s();
    }
  }

  // ── Complementary filter update ──────────────────────────────────────────
  // Runs every loop for accurate gyro integration (dt matters here), even
  // though JSON is only published at 50 Hz below.
  float dt = (nowUs - lastFilterUpdateUs) / 1000000.0f;
  lastFilterUpdateUs = nowUs;
  if (dt > 0.0f && dt < 0.5f) { // guard against startup / overflow glitches
    updateComplementaryFilter(dt);
  }

  // Non-blocking 50 Hz rate limiter (handles micros() overflow gracefully).
  if ((nowUs - lastPublishUs) >= PUBLISH_PERIOD_US) {
    lastPublishUs = nowUs;

    xyzFloat gyro1 = mpu1.getGyrValues(); // deg/s
    xyzFloat gyro2 = mpu2.getGyrValues(); // deg/s

    publishJSON(pitch1 - pitch1Offset, roll1 - roll1Offset, gyro1,
                pitch2 - pitch2Offset, roll2 - roll2Offset, gyro2);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// initMPU9250s()
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Initialise both MPU9250s with project-specific sensor settings.
 *
 * Settings:
 *   Accel range  : ±4 G       — tilt-gesture control doesn't need ±8G headroom;
 *                                narrower range = better resolution for small tilts
 *   Gyro range   : ±500 °/s   — covers rapid rotation gestures
 *   DLPF         : level 6    — attenuates high-frequency vibration noise
 *
 * @return true on success, false if either device not found on I2C bus.
 */
bool initMPU9250s() {
  bool ok1 = mpu1.initWithId(0x75); // your boards report 0x75, not library default 0x71
  delay(200);
  if (!ok1) {
    Serial.println("  -> mpu1 (0x68) init FAILED");
  }

  bool ok2 = mpu2.initWithId(0x75);
  delay(200);
  if (!ok2) {
    Serial.println("  -> mpu2 (0x69) init FAILED");
  }

  if (!ok1 || !ok2) return false;

  mpu1.autoOffsets();
  delay(50);
  mpu2.autoOffsets();
  delay(50);

  mpu1.setAccRange(MPU9250_ACC_RANGE_4G);
  mpu1.enableAccDLPF(true);
  mpu1.setAccDLPF(MPU9250_DLPF_6);
  mpu1.setGyrRange(MPU9250_GYRO_RANGE_500);
  mpu1.enableGyrDLPF();
  mpu1.setGyrDLPF(MPU9250_DLPF_6);

  mpu2.setAccRange(MPU9250_ACC_RANGE_4G);
  mpu2.enableAccDLPF(true);
  mpu2.setAccDLPF(MPU9250_DLPF_6);
  mpu2.setGyrRange(MPU9250_GYRO_RANGE_500);
  mpu2.enableGyrDLPF();
  mpu2.setGyrDLPF(MPU9250_DLPF_6);

  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// i2cDeviceResponding()
// ─────────────────────────────────────────────────────────────────────────────

bool i2cDeviceResponding(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

// ─────────────────────────────────────────────────────────────────────────────
// computeAccelAngles()
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Compute pitch/roll tilt angle (degrees) from accelerometer alone.
 *
 * This is the "correction" input to the complementary filter — accurate at
 * rest / slow movement, but noisy under vibration/shock, which is why it's
 * blended with (not used instead of) the gyro-integrated angle.
 */
void computeAccelAngles(MPU9250_WE &mpu, float &accelPitch, float &accelRoll) {
  xyzFloat a = mpu.getGValues(); // accel in g
  accelPitch = atan2(a.y, sqrt(a.x * a.x + a.z * a.z)) * 180.0f / PI;
  accelRoll  = atan2(-a.x, sqrt(a.y * a.y + a.z * a.z)) * 180.0f / PI;
}

// ─────────────────────────────────────────────────────────────────────────────
// updateComplementaryFilter()
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Advance the complementary filter for both IMUs by one time step.
 *
 * angle = ALPHA * (angle_prev + gyroRate * dt) + (1 - ALPHA) * accelAngle
 *
 * @param dt  Time since last update, in seconds.
 */
void updateComplementaryFilter(float dt) {
  xyzFloat g1 = mpu1.getGyrValues(); // deg/s
  xyzFloat g2 = mpu2.getGyrValues(); // deg/s

  float accelPitch1, accelRoll1, accelPitch2, accelRoll2;
  computeAccelAngles(mpu1, accelPitch1, accelRoll1);
  computeAccelAngles(mpu2, accelPitch2, accelRoll2);

  pitch1 = COMP_FILTER_ALPHA * (pitch1 + g1.x * dt) + (1.0f - COMP_FILTER_ALPHA) * accelPitch1;
  roll1  = COMP_FILTER_ALPHA * (roll1  + g1.y * dt) + (1.0f - COMP_FILTER_ALPHA) * accelRoll1;

  pitch2 = COMP_FILTER_ALPHA * (pitch2 + g2.x * dt) + (1.0f - COMP_FILTER_ALPHA) * accelPitch2;
  roll2  = COMP_FILTER_ALPHA * (roll2  + g2.y * dt) + (1.0f - COMP_FILTER_ALPHA) * accelRoll2;
}

// ─────────────────────────────────────────────────────────────────────────────
// calibrateNeutralPosition()
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Average the fused angles over CALIBRATION_DURATION_MS and store
 *        the result as the zero-offset for each axis.
 *
 * Must be called AFTER the complementary filter has been seeded (setup()
 * does this), so it's averaging real fused readings, not the angle=0
 * startup default.
 */
void calibrateNeutralPosition() {
  float sumPitch1 = 0.0f, sumRoll1 = 0.0f;
  float sumPitch2 = 0.0f, sumRoll2 = 0.0f;
  uint32_t sampleCount = 0;

  uint32_t calibStartMs = millis();
  uint32_t lastSampleMs = 0;
  uint32_t lastFilterUs = micros();

  while ((millis() - calibStartMs) < CALIBRATION_DURATION_MS) {
    uint32_t nowMs = millis();
    uint32_t nowUs = micros();

    // Keep advancing the filter during calibration too, at its own rate.
    float dt = (nowUs - lastFilterUs) / 1000000.0f;
    lastFilterUs = nowUs;
    if (dt > 0.0f && dt < 0.5f) {
      updateComplementaryFilter(dt);
    }

    if ((nowMs - lastSampleMs) >= CALIBRATION_SAMPLE_PERIOD_MS) {
      lastSampleMs = nowMs;
      sumPitch1 += pitch1;
      sumRoll1  += roll1;
      sumPitch2 += pitch2;
      sumRoll2  += roll2;
      sampleCount++;
    }
  }

  if (sampleCount > 0) {
    pitch1Offset = sumPitch1 / sampleCount;
    roll1Offset  = sumRoll1  / sampleCount;
    pitch2Offset = sumPitch2 / sampleCount;
    roll2Offset  = sumRoll2  / sampleCount;
  }
  // If sampleCount is somehow 0 (shouldn't happen), offsets stay at their
  // prior value (0.0 on first boot) rather than crashing on divide-by-zero.
}

// ─────────────────────────────────────────────────────────────────────────────
// updateDebouncedButton()
// ─────────────────────────────────────────────────────────────────────────────

uint8_t updateDebouncedButton(DebouncedButton &btn) {
  const uint8_t raw = (digitalRead(btn.pin) == LOW) ? 1 : 0; // active-low
  const uint32_t nowMs = millis();

  if (raw != btn.lastRawState) {
    btn.lastRawState = raw;
    btn.lastChangeMs = nowMs;
  } else if ((nowMs - btn.lastChangeMs) >= DEBOUNCE_MS) {
    btn.stableState = raw;
  }

  return btn.stableState;
}

// ─────────────────────────────────────────────────────────────────────────────
// publishJSON()
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Serialise fused/calibrated IMU angles + raw gyro rates + button
 *        states as a JSON line on Serial.
 */
void publishJSON(float p1, float r1, xyzFloat g1,
                 float p2, float r2, xyzFloat g2) {

  const uint8_t btnArmDisarmState  = updateDebouncedButton(btnArmDisarm);
  const uint8_t btnModeToggleState = updateDebouncedButton(btnForceArm); // GPIO 33
  const uint8_t btnTakeoffState    = updateDebouncedButton(btnTakeoff);  // GPIO 25
  const uint8_t btnRtlState        = updateDebouncedButton(btnLand);     // GPIO 26

  // Stack-allocated buffer — no heap fragmentation.
  char buf[512];

  snprintf(buf, sizeof(buf),
           "{\"timestamp\":%lu"
           ",\"pitch1\":%.3f,\"roll1\":%.3f,\"gx1\":%.3f,\"gy1\":%.3f,\"gz1\":%.3f"
           ",\"pitch2\":%.3f,\"roll2\":%.3f,\"gx2\":%.3f,\"gy2\":%.3f,\"gz2\":%.3f"
           ",\"arm_disarm\":%u,\"mode_toggle\":%u"
           ",\"takeoff\":%u,\"rtl\":%u}",
           millis(),
           p1, r1, g1.x, g1.y, g1.z,
           p2, r2, g2.x, g2.y, g2.z,
           btnArmDisarmState, btnModeToggleState, btnTakeoffState, btnRtlState);

  Serial.println(buf);
}