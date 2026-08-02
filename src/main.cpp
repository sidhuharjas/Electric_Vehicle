#include <Arduino.h>
#include <TektiteRotEv.h>
#include <math.h>

// ELECTRIC VEHICLE C - 2026 COMPETITION CODE
// Strong S-curve + better return to center

RotEv rotev;

// ================================================================
// COMPETITION SETTINGS (UPDATE BEFORE EVERY EVENT)
// ================================================================
const float TARGET_DISTANCE_M = 8.4f;  // Target distance in meters (actual final distance)
const float STOP_EXTRA_M = 0.0f;        // No extra offset - TARGET_DISTANCE_M is the final distance
const float END_LATERAL_BIAS_M = 0.01f;  // Additional +7 cm left trim from prior tuning
const float FRONT_TO_WHEEL_CENTER_M = 0.109f;  // 109 mm from front bumper to wheel center
bool targetDistanceIsFrontReference = true;    // true: convert front target to wheel-center travel

// Time-target assist: nudges speed to finish near target time without sacrificing distance completion.
bool USE_TIME_SCALING = true;
float TARGET_RUN_TIME_S = 14.0f;                 // Competition target time (clamped to 10-20s, snapped to 0.5s)
const float TIME_TARGET_MIN_S = 10.0f;
const float TIME_TARGET_MAX_S = 20.0f;
const float TIME_TARGET_STEP_S = 0.5f;
const float TIME_SCALE_MIN = 0.88f;              // Prevent too-slow behavior that weakens control authority
const float TIME_SCALE_MAX = 1.18f;              // Prevent aggressive speed-ups that increase tip-over risk
const float TIME_SCALE_KP = 0.85f;               // How strongly we correct schedule error
const float MAX_SAFE_POWER = 0.48f;              // Hard cap for stability/safety

// ================================================================
// BONUS CAN S-CURVE SETTINGS
// ================================================================
bool useArc = true;
float ARC_MAX_ANGLE = 7.0f;  // Rolled back slightly for stability
float ARC_TARGET_OFFSET_M = 0.86f;  // Rolled back slightly for stability
float ARC_LATERAL_SHAPE_EXP = 1.00f;  // 1.0 means peak occurs at exactly half distance
int DRIVE_SIDE = 1;           // 1 = left of center, -1 = right of center

// ================================================================
// MOTOR / ENCODER DIRECTION SETTINGS
// ================================================================
const int M1_DIR = -1;
const int M2_DIR = 1;
const int ENC1_DIR = 1;
const int ENC2_DIR = -1;

// ================================================================
// ROBOT CONSTANTS
// ================================================================
const float WHEEL_DIAM_M = 0.0525f;
const float WHEEL_CIRC_M = 3.1415926535f * WHEEL_DIAM_M;
const float METERS_PER_DEG = WHEEL_CIRC_M / 360.0f;
const float WHEELBASE_M = 0.180f;
const float RAD_TO_DEG_F = 57.2957795f;
const bool USE_IMU_HEADING = true;  // Restored: use IMU heading by default.

// ================================================================
// CONTROL TUNING
// ================================================================
float basePower = 0.35f;
float minPower = 0.10f;
// Accel & decel as fractions of effective target distance (reliable scaling across 7-10m range)
const float ACCEL_FRAC = 0.0664f;  // ~6.64% of distance for smooth launch
const float DECEL_FRAC = 0.1881f;  // ~18.81% of distance for smooth stop
float accelDist = 0.60f;  // Will be computed per-run based on target
float decelDist = 1.70f;  // Will be computed per-run based on target
float END_CREEP_ZONE_M = 0.40f;
float END_CREEP_MIN_POWER = 0.125f;  // Balanced end-of-run minimum torque

float SM_gain = 0.045f;
float ARC_SM_GAIN = 0.010f;         // Weak speed matching even during arc to reduce wheel-bias drift.
float ENC_DEADBAND_DEG = 0.08f;     // Ignore tiny encoder jitter near zero.
float GYRO_LPF_ALPHA = 0.20f;       // Low-pass filter for yaw-rate noise.
float GYRO_DEADBAND_DPS = 0.25f;    // Ignore tiny gyro fuzz around zero.

float XT_GAIN = -45.0f;
float KI_LATERAL = 0.009f;
float DRIVE_TRIM = 0.016f;

// End-phase recenter tuning so the vehicle finishes closer to heading=0 and lateral=0.
float END_CENTER_START = 0.52f;      // Start final recenter slightly earlier
float END_HEADING_DAMP = 0.18f;      // Softer heading damping to reduce late over-correction
float END_CENTER_BOOST = 4.2f;       // Safer late centering gain to avoid aggressive oscillation

// Launch robustness: avoid button-hit induced IMU bias by waiting for release and settle.
const unsigned long GO_RELEASE_SETTLE_MS = 220UL;
const float GYRO_CAL_OUTLIER_DPS = 8.0f;

// ================================================================
// INTERNAL STATE
// ================================================================
float cum1 = 0.0f;
float cum2 = 0.0f;
float lastEnc1 = 0.0f;
float lastEnc2 = 0.0f;

float heading = 0.0f;
float gyroBias = 0.0f;
unsigned long lastMs = 0;
float maxHeading = 0.0f;

float lateralIntegral = 0.0f;

bool telemetryReady = false;
String telemetryBuffer = "";

// ================================================================
// UTILITY
// ================================================================
float wrapDeg(float d) {
  while (d > 180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

float sanitizeTargetTimeSec(float rawSeconds) {
  float clamped = clampf(rawSeconds, TIME_TARGET_MIN_S, TIME_TARGET_MAX_S);
  float snapped = roundf(clamped / TIME_TARGET_STEP_S) * TIME_TARGET_STEP_S;
  return clampf(snapped, TIME_TARGET_MIN_S, TIME_TARGET_MAX_S);
}

float computeWheelCenterTargetMeters(float requestedDistanceM) {
  float wheelCenterTarget = requestedDistanceM;
  if (targetDistanceIsFrontReference) {
    wheelCenterTarget -= FRONT_TO_WHEEL_CENTER_M;
  }
  return clampf(wheelCenterTarget, 0.05f, 100.0f);
}

void writeMotorPower(float leftPower, float rightPower) {
  // Clamp output to valid normalized motor range.
  float lp = clampf(leftPower, -1.0f, 1.0f);
  float rp = clampf(rightPower, -1.0f, 1.0f);
  rotev.motorWrite1(M1_DIR * lp);
  rotev.motorWrite2(M2_DIR * rp);
}

void resetEncoders() {
  cum1 = 0.0f;
  cum2 = 0.0f;
  lastEnc1 = rotev.enc1AngleDegrees();
  lastEnc2 = rotev.enc2AngleDegrees();
}

void fullReset() {
  resetEncoders();
  heading = 0.0f;
  lateralIntegral = 0.0f;
  lastMs = millis();
  maxHeading = 0.0f;

  if (USE_IMU_HEADING) {
    float sum = 0.0f;
    int kept = 0;
    for (int i = 0; i < 200; i++) {
      float sample = rotev.readYawRateDegrees();
      // Reject obvious motion spikes during calibration.
      if (fabsf(sample) <= GYRO_CAL_OUTLIER_DPS) {
        sum += sample;
        kept++;
      }
      delay(3);
    }
    if (kept > 40) {
      gyroBias = sum / (float)kept;
    } else {
      gyroBias = 0.0f;
    }
  } else {
    gyroBias = 0.0f;
  }

  delay(250);
}

float computeArcHeading(float progress) {
  // Envelope limits turn demand near start/end while preserving a strong midpoint arc.
  float wave = sinf(progress * 2.0f * PI);
  float envelope = sinf(progress * PI);
  float returnBoost = (progress > 0.50f) ? (1.0f + 0.18f * ((progress - 0.50f) / 0.50f)) : 1.0f;
  return wave * envelope * ARC_MAX_ANGLE * returnBoost * DRIVE_SIDE;
}

float computeArcTargetLateral(float progress) {
  // Shape exponent (<1) moves peak earlier while keeping 0 at start/finish.
  float shapedProgress = powf(clampf(progress, 0.0f, 1.0f), ARC_LATERAL_SHAPE_EXP);
  return DRIVE_SIDE * ARC_TARGET_OFFSET_M * sinf(shapedProgress * PI);
}

void driveDistanceMeters(float targetM, bool doArc) {
  // Compute proportional accel/decel zones based on target distance for consistent behavior
  accelDist = targetM * ACCEL_FRAC;
  decelDist = targetM * DECEL_FRAC;

  fullReset();
  rotev.motorEnable(true);
  rotev.ledWrite(0.00f, 0.00f, 0.35f);  // Bright blue while executing EV run.

  float filteredSpeedError = 0.0f;
  float filteredDriftRate = 0.0f;
  float forwardX = 0.0f;
  float lateralY = 0.0f;
  float elapsedSec = 0.0f;
  float targetMWithTrim = targetM + STOP_EXTRA_M;
  float targetTimeSec = sanitizeTargetTimeSec(TARGET_RUN_TIME_S);

  while (true) {
    if (rotev.stopButtonPressed()) break;

    float e1 = rotev.enc1AngleDegrees();
    float e2 = rotev.enc2AngleDegrees();

    float d1 = wrapDeg(e1 - lastEnc1);
    float d2 = wrapDeg(e2 - lastEnc2);

    if (fabsf(d1) < ENC_DEADBAND_DEG) d1 = 0.0f;
    if (fabsf(d2) < ENC_DEADBAND_DEG) d2 = 0.0f;

    cum1 += ENC1_DIR * d1;
    cum2 += ENC2_DIR * d2;

    lastEnc1 = e1;
    lastEnc2 = e2;

    unsigned long now = millis();
    float dt = (now - lastMs) / 1000.0f;
    lastMs = now;
    dt = clampf(dt, 0.001f, 0.060f);
    elapsedSec += dt;

    float d_left = (ENC1_DIR * d1) * METERS_PER_DEG;
    float d_right = (ENC2_DIR * d2) * METERS_PER_DEG;
    float d_center = (d_left + d_right) * 0.5f;

    if (USE_IMU_HEADING) {
      float rawDriftRate = rotev.readYawRateDegrees() - gyroBias;
      filteredDriftRate = filteredDriftRate + GYRO_LPF_ALPHA * (rawDriftRate - filteredDriftRate);
      float driftRate = (fabsf(filteredDriftRate) < GYRO_DEADBAND_DPS) ? 0.0f : filteredDriftRate;
      heading += driftRate * dt;
    } else {
      float deltaYawDeg = ((d_right - d_left) / WHEELBASE_M) * RAD_TO_DEG_F;
      heading = wrapDeg(heading + deltaYawDeg);
    }
    if (fabsf(heading) > fabsf(maxHeading)) {
      maxHeading = heading;
    }

    float headingRad = heading * (PI / 180.0f);
    forwardX += d_center * cosf(headingRad);
    lateralY += d_center * sinf(headingRad);

    float remaining = targetMWithTrim - forwardX;
    if (remaining <= 0.0f) break;

    float remainingForControl = (remaining > 0.0f) ? remaining : 0.0f;

    float progress = clampf(forwardX / targetMWithTrim, 0.0f, 1.0f);
    float targetHeading = 0.0f;
    float lateralRef = 0.0f;
    if (doArc) {
      targetHeading = computeArcHeading(progress);
      lateralRef = computeArcTargetLateral(progress);
      float endBiasBlend = clampf((progress - 0.72f) / 0.28f, 0.0f, 1.0f);
      lateralRef += END_LATERAL_BIAS_M * endBiasBlend;

      // Fade commanded arc near end so recenter terms dominate finish behavior.
      float endBlend = clampf((progress - END_CENTER_START) / (1.0f - END_CENTER_START), 0.0f, 1.0f);
      targetHeading *= (1.0f - endBlend);
    }

    float lateralCtrlError = doArc ? (lateralY - lateralRef) : lateralY;

    lateralIntegral += lateralCtrlError * dt;
    lateralIntegral = clampf(lateralIntegral, -0.35f, 0.35f);

    float centerAssist = (progress > 0.66f) ? 1.25f : 1.0f;
    if (doArc && progress > 0.55f) centerAssist *= 1.15f;

    if (doArc && progress > END_CENTER_START) {
      float endBlend = clampf((progress - END_CENTER_START) / (1.0f - END_CENTER_START), 0.0f, 1.0f);
      centerAssist *= (1.0f + END_CENTER_BOOST * endBlend);
      targetHeading += -END_HEADING_DAMP * endBlend * heading;
    }

    targetHeading += centerAssist * (XT_GAIN * lateralCtrlError + KI_LATERAL * lateralIntegral);
    targetHeading = clampf(targetHeading, -40.0f, 40.0f);

    float hError = heading - targetHeading;
    if (fabsf(hError) < 1.0f) hError = 0.0f;

    float currentPower;
    if (forwardX < accelDist) {
      float f = clampf(forwardX / accelDist, 0.0f, 1.0f);
      currentPower = minPower + f * (basePower - minPower);
    } else if (remainingForControl < decelDist) {
      float f = clampf(remainingForControl / decelDist, 0.0f, 1.0f);
      currentPower = minPower + f * (basePower - minPower);
    } else {
      currentPower = basePower;
    }

    if (remainingForControl < END_CREEP_ZONE_M) {
      float f = clampf(remainingForControl / END_CREEP_ZONE_M, 0.0f, 1.0f);
      currentPower = END_CREEP_MIN_POWER + f * (currentPower - END_CREEP_MIN_POWER);
    }

    if (USE_TIME_SCALING) {
      float desiredProgress = clampf(elapsedSec / targetTimeSec, 0.0f, 1.0f);
      float progressError = desiredProgress - progress;
      float timeScale = clampf(1.0f + TIME_SCALE_KP * progressError, TIME_SCALE_MIN, TIME_SCALE_MAX);
      currentPower *= timeScale;
    }

    // Enforce minimum torque at the finish so the vehicle can still move.
    if (currentPower < END_CREEP_MIN_POWER) currentPower = END_CREEP_MIN_POWER;
    if (currentPower > MAX_SAFE_POWER) currentPower = MAX_SAFE_POWER;

    float gyroCorr;
    if (remainingForControl > 0.30f) {
      float Kg = (currentPower < 0.35f) ? 0.009f : (currentPower < 0.50f) ? 0.014f : 0.019f;
      gyroCorr = Kg * hError;
    } else {
      gyroCorr = 0.010f * hError;
    }
    gyroCorr = clampf(gyroCorr, -0.28f, 0.28f);

    float speedBalance = 0.0f;
    float leftSpeed = ENC1_DIR * d1;
    float rightSpeed = ENC2_DIR * d2;
    float speedError = leftSpeed - rightSpeed;
    if (fabsf(speedError) < 0.5f) speedError = 0.0f;

    filteredSpeedError = 0.7f * filteredSpeedError + 0.3f * speedError;
    if (doArc) {
      // Keep this intentionally weak so we don't flatten the arc shape.
      speedBalance = ARC_SM_GAIN * filteredSpeedError;
    } else {
      speedBalance = (remainingForControl > 0.30f) ? SM_gain * filteredSpeedError : 0.022f * filteredSpeedError;
    }

    float leftPower = currentPower - speedBalance - gyroCorr - DRIVE_TRIM;
    float rightPower = currentPower + speedBalance + gyroCorr + DRIVE_TRIM;

    writeMotorPower(leftPower, rightPower);
    delay(20);
  }

  // No reverse kick at finish; it introduced stop-position inconsistency.
  writeMotorPower(0.0f, 0.0f);
  delay(70);
  rotev.motorEnable(false);
  rotev.ledWrite(1.0f, 0.0f, 0.0f);  // Bright red when finished.

  telemetryBuffer = "===== RUN TELEMETRY (STRONGER ARC + CENTERING) =====\n";
  telemetryBuffer += "Final Heading: " + String(heading, 2) + " deg\n";
  telemetryBuffer += "Max Heading Drift: " + String(maxHeading, 2) + " deg\n";
  telemetryBuffer += "Forward X: " + String(forwardX, 3) + " m\n";
  telemetryBuffer += "Final Lateral Y (LEFT = +): " + String(lateralY, 3) + " m\n";
  telemetryBuffer += "Stop Extra Trim: " + String(STOP_EXTRA_M, 3) + " m\n";
  telemetryBuffer += "End Lateral Bias: " + String(END_LATERAL_BIAS_M, 3) + " m\n";
  telemetryBuffer += "Arc Target Offset @ Halfway: " + String(ARC_TARGET_OFFSET_M, 3) + " m\n";
  telemetryBuffer += "Lateral Integral: " + String(lateralIntegral, 3) + "\n";
  telemetryBuffer += "Arc Used: " + String(doArc ? "YES" : "NO") + "\n";
  telemetryBuffer += "ARC_MAX_ANGLE: " + String(ARC_MAX_ANGLE, 1) + " deg\n";
  telemetryBuffer += "Time Scaling: " + String(USE_TIME_SCALING ? "ON" : "OFF") + "\n";
  telemetryBuffer += "Target Time: " + String(targetTimeSec, 1) + " s\n";
  telemetryBuffer += "Actual Time: " + String(elapsedSec, 2) + " s\n";
  telemetryBuffer += "================================================\n";
  telemetryReady = true;
}

void setup() {
  Serial.begin(115200);
  rotev.begin();
  rotev.motorEnable(false);
  rotev.ledWrite(0.20f, 0.20f, 0.00f);  // Standby yellow at boot.
}

void loop() {
  static bool runArmed = false;
  static unsigned long releaseMs = 0;
  bool goNow = rotev.goButtonPressed();

  // Arm on press, launch only after release + short settle.
  if (goNow) {
    runArmed = true;
    releaseMs = 0;
  } else if (runArmed) {
    if (releaseMs == 0) {
      releaseMs = millis();
    }
    if ((millis() - releaseMs) >= GO_RELEASE_SETTLE_MS) {
      runArmed = false;
      float wheelCenterTarget = computeWheelCenterTargetMeters(TARGET_DISTANCE_M);
      driveDistanceMeters(wheelCenterTarget, useArc);
    }
  }

  if (telemetryReady && Serial) {
    Serial.println(telemetryBuffer);
    telemetryReady = false;
  }
}