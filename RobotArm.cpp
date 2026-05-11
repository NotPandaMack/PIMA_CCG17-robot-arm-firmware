#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include "Config.h"
#include "RobotArm.h"
#include "Timeline.h"

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDRESS);

// ======================================================
// Logical Servo Calibration
// ======================================================
static const float BICEP_PHYSICAL_ZERO_DEG = 3.0;
static const float GRIPPER_TIPS_TOUCH_DEG = 55.0;

// ======================================================
// Servo State
// ======================================================
struct ServoState {
  float current;
  float target;
  float velocity;
  bool enabled;
  int minSafe;
  int maxSafe;
};

static ServoState servos[NUM_SERVOS] = {
  { SERVO_MID, SERVO_MID, 0.0, false, SERVO_MIN, SERVO_MAX }, // Base
  { SERVO_MID, SERVO_MID, 0.0, false, SERVO_MIN, SERVO_MAX }, // Bicep
  { SERVO_MID, SERVO_MID, 0.0, false, SERVO_MIN, SERVO_MAX }, // Forearm
  { SERVO_MID, SERVO_MID, 0.0, false, SERVO_MIN, SERVO_MAX }, // Wrist
  { SERVO_MID, SERVO_MID, 0.0, false, SERVO_MIN, SERVO_MAX }  // Claw
};

static int lastWritten[NUM_SERVOS] = { -1, -1, -1, -1, -1 };

static bool estop = false;

static unsigned long lastUpdate = 0;

static int speedSetting = 1;
static float ikStepScale = 1.0;

static int activeServo = -1;
static int activeDirection = 0;

static float activeDx = 0.0;
static float activeDy = 0.0;
static float activeDz = 0.0;
static float activeDp = 0.0;

static float desiredDx = 0.0;
static float desiredDy = 0.0;
static float desiredDz = 0.0;
static float desiredDp = 0.0;

static unsigned long lastCommandTime = 0;

static String robotStatus = "Ready";

// ======================================================
// IK State
// ======================================================
static ToolMode activeToolMode = TOOL_TIP_MODE;

static float targetX = 0.0;
static float targetY = 170.0;
static float targetZ = 80.0;
static float toolPitchDeg = 0.0;

static float wristX = 0.0;
static float wristY = 0.0;
static float wristZ = 0.0;

static float lastBaseDeg = 90.0;
static float lastShoulderDeg = 90.0;
static float lastElbowServoDeg = 90.0;
static float lastWristDeg = 90.0;

// ======================================================
// Utility
// ======================================================
static float clampFloat(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static float absFloat(float v) {
  return v < 0.0 ? -v : v;
}

static int roundedTicks(float ticks) {
  return (int)(ticks + 0.5f);
}

static float speedScale() {
  return 0.55 + ((float)speedSetting - 1.0) * 0.15;
}

static float radToDeg(float r) {
  return r * 180.0 / PI;
}

static float degToRad(float d) {
  return d * PI / 180.0;
}

static float getActiveToolLength() {
  return activeToolMode == TOOL_GAP_MODE ? TOOL_GAP_MM : TOOL_TIP_MM;
}

// ======================================================
// Servo Low-Level
// ======================================================
static float factoryAngleToTicks(float angleDeg);
static float ticksToFactoryAngle(float ticks);

static float logicalToPhysicalDegrees(int servoIndex, float logicalDeg) {
  logicalDeg = constrain(logicalDeg, 0, 180);

  if (servoIndex == BICEP) {
    return BICEP_PHYSICAL_ZERO_DEG + (logicalDeg * (180.0 - BICEP_PHYSICAL_ZERO_DEG) / 180.0);
  }

  return logicalDeg;
}

static float physicalToLogicalDegrees(int servoIndex, float physicalDeg) {
  physicalDeg = constrain(physicalDeg, 0, 180);

  if (servoIndex == BICEP) {
    if (physicalDeg <= BICEP_PHYSICAL_ZERO_DEG) return 0.0;
    return (physicalDeg - BICEP_PHYSICAL_ZERO_DEG) * 180.0 / (180.0 - BICEP_PHYSICAL_ZERO_DEG);
  }

  return physicalDeg;
}

static float logicalDegreesToTicks(int servoIndex, float logicalDeg) {
  return factoryAngleToTicks(logicalToPhysicalDegrees(servoIndex, logicalDeg));
}

static float ticksToLogicalDegrees(int servoIndex, float ticks) {
  return physicalToLogicalDegrees(servoIndex, ticksToFactoryAngle(ticks));
}

static void disableServo(int i) {
  if (i < 0 || i >= NUM_SERVOS) return;

  pwm.setPWM(SERVO_CHANNELS[i], 0, 4096);
  servos[i].enabled = false;
  servos[i].velocity = 0.0;
  lastWritten[i] = -1;
}

static void enableServo(int i) {
  if (i < 0 || i >= NUM_SERVOS) return;

  servos[i].target = clampFloat(servos[i].target, servos[i].minSafe, servos[i].maxSafe);
  servos[i].current = servos[i].target;
  servos[i].velocity = 0.0;
  servos[i].enabled = true;

  int ticks = roundedTicks(servos[i].current);
  pwm.setPWM(SERVO_CHANNELS[i], 0, ticks);
  lastWritten[i] = ticks;
}

static void ensureServoEnabled(int i) {
  if (i < 0 || i >= NUM_SERVOS) return;

  if (!servos[i].enabled) {
    enableServo(i);
  }
}

static float factoryAngleToTicks(float angleDeg) {
  angleDeg = constrain(angleDeg, 0, 180);
  float ticks = SERVO_MIN + ((SERVO_MAX - SERVO_MIN) * (angleDeg / 180.0));
  return clampFloat(ticks, SERVO_MIN, SERVO_MAX);
}

static float ticksToFactoryAngle(float ticks) {
  ticks = clampFloat(ticks, SERVO_MIN, SERVO_MAX);
  return ((ticks - SERVO_MIN) * 180.0) / (SERVO_MAX - SERVO_MIN);
}

static void applyServoSafeBounds() {
  servos[BICEP].minSafe = roundedTicks(logicalDegreesToTicks(BICEP, 0.0));
  servos[BICEP].maxSafe = SERVO_MAX;
}

static void setServoTargetFromLogicalDegrees(int servoIndex, float logicalDeg) {
  float ticks = logicalDegreesToTicks(servoIndex, logicalDeg);
  servos[servoIndex].target = clampFloat(ticks, servos[servoIndex].minSafe, servos[servoIndex].maxSafe);

  ensureServoEnabled(servoIndex);
}

static void updateLegacyJointTelemetry(int servoIndex, float degrees) {
  if (servoIndex == BASE) lastBaseDeg = degrees;
  if (servoIndex == BICEP) lastShoulderDeg = degrees;
  if (servoIndex == FOREARM) lastElbowServoDeg = degrees;
  if (servoIndex == WRIST) lastWristDeg = degrees;
}

// ======================================================
// Public Robot Setup/Update
// ======================================================
void setupRobotArm() {
  Wire.begin(SDA_PIN, SCL_PIN);
  applyServoSafeBounds();

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);

  delay(10);

  // Safety: all PCA9685 outputs off at boot.
  for (int ch = 0; ch < 16; ch++) {
    pwm.setPWM(ch, 0, 4096);
  }

  allServosOff();

  lastUpdate = millis();
  lastCommandTime = millis();

  robotStatus = "Robot arm ready";
}

void updateRobotArm() {
  unsigned long now = millis();

  if (now - lastUpdate < UPDATE_MS) return;
  lastUpdate = now;

  if (estop) {
    allServosOff();
    stopRobotMotion();
    return;
  }

  if (now - lastCommandTime > COMMAND_TIMEOUT_MS) {
    stopRobotMotion();
  }

  if (activeServo >= 0 && activeDirection != 0) {
    servos[activeServo].target += activeDirection * MANUAL_TARGET_TICKS_PER_UPDATE * speedScale();
    servos[activeServo].target = clampFloat(
      servos[activeServo].target,
      servos[activeServo].minSafe,
      servos[activeServo].maxSafe
    );
  }

  float smoothing = (
    desiredDx == 0.0 &&
    desiredDy == 0.0 &&
    desiredDz == 0.0 &&
    desiredDp == 0.0
  ) ? IK_STOP_SMOOTHING : IK_INPUT_SMOOTHING;

  activeDx += (desiredDx - activeDx) * smoothing;
  activeDy += (desiredDy - activeDy) * smoothing;
  activeDz += (desiredDz - activeDz) * smoothing;
  activeDp += (desiredDp - activeDp) * smoothing;

  if (absFloat(activeDx) < 0.01) activeDx = 0.0;
  if (absFloat(activeDy) < 0.01) activeDy = 0.0;
  if (absFloat(activeDz) < 0.01) activeDz = 0.0;
  if (absFloat(activeDp) < 0.01) activeDp = 0.0;

  if (activeDx != 0.0 || activeDy != 0.0 || activeDz != 0.0 || activeDp != 0.0) {
    moveTargetRelative(
      activeDx * ikStepScale,
      activeDy * ikStepScale,
      activeDz * ikStepScale,
      activeDp * ikStepScale
    );
  }

  for (int i = 0; i < NUM_SERVOS; i++) {
    if (!servos[i].enabled) continue;

    float error = servos[i].target - servos[i].current;

    if (absFloat(error) <= SERVO_SETTLE_DEADBAND_TICKS && absFloat(servos[i].velocity) <= SERVO_ACCEL_TICKS[i]) {
      servos[i].current = servos[i].target;
      servos[i].velocity = 0.0;
    } else {
      float desiredVelocity = clampFloat(
        error,
        -SERVO_MAX_SPEED_TICKS[i] * speedScale(),
        SERVO_MAX_SPEED_TICKS[i] * speedScale()
      );

      float velocityDelta = desiredVelocity - servos[i].velocity;
      float maxAccel = SERVO_ACCEL_TICKS[i] * speedScale();
      velocityDelta = clampFloat(velocityDelta, -maxAccel, maxAccel);

      servos[i].velocity += velocityDelta;
      servos[i].current += servos[i].velocity;

      if ((error > 0.0 && servos[i].current > servos[i].target) ||
          (error < 0.0 && servos[i].current < servos[i].target)) {
        servos[i].current = servos[i].target;
        servos[i].velocity = 0.0;
      }
    }

    servos[i].current = clampFloat(servos[i].current, servos[i].minSafe, servos[i].maxSafe);

    int writeTicks = roundedTicks(servos[i].current);

    if (lastWritten[i] != writeTicks) {
      pwm.setPWM(SERVO_CHANNELS[i], 0, writeTicks);
      lastWritten[i] = writeTicks;
    }
  }
}

// ======================================================
// Public Movement Functions
// ======================================================
void stopRobotMotion() {
  activeServo = -1;
  activeDirection = 0;

  desiredDx = 0.0;
  desiredDy = 0.0;
  desiredDz = 0.0;
  desiredDp = 0.0;
  activeDx = 0.0;
  activeDy = 0.0;
  activeDz = 0.0;
  activeDp = 0.0;
}

void allServosOff() {
  for (int i = 0; i < NUM_SERVOS; i++) {
    disableServo(i);
  }
}

void calibrateServosZeroDegrees() {
  stopTimeline();
  stopRobotMotion();

  estop = false;

  for (int i = 0; i < NUM_SERVOS; i++) {
    float ticks = logicalDegreesToTicks(i, 0.0);
    servos[i].current = ticks;
    servos[i].target = ticks;
    servos[i].velocity = 0.0;
    servos[i].enabled = true;

    pwm.setPWM(SERVO_CHANNELS[i], 0, roundedTicks(ticks));
    lastWritten[i] = roundedTicks(ticks);
  }

  targetX = 0.0;
  targetY = 170.0;
  targetZ = 80.0;
  toolPitchDeg = 0.0;

  lastBaseDeg = 0.0;
  lastShoulderDeg = 0.0;
  lastElbowServoDeg = 0.0;
  lastWristDeg = 0.0;

  robotStatus = "Calibration: all servos set to 0 degrees";
}

void setServoDegrees(int servoIndex, float degrees) {
  if (estop) return;
  if (servoIndex < 0 || servoIndex >= NUM_SERVOS) return;

  stopTimeline();
  stopRobotMotion();

  degrees = constrain(degrees, 0, 180);
  servos[servoIndex].target = logicalDegreesToTicks(servoIndex, degrees);
  servos[servoIndex].velocity = 0.0;
  ensureServoEnabled(servoIndex);
  updateLegacyJointTelemetry(servoIndex, degrees);

  robotStatus = "Calibration: servo set to degrees";
}

void setAllServoDegrees(float degrees) {
  if (estop) return;

  stopTimeline();
  stopRobotMotion();

  degrees = constrain(degrees, 0, 180);

  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].target = logicalDegreesToTicks(i, degrees);
    servos[i].velocity = 0.0;
    ensureServoEnabled(i);
    updateLegacyJointTelemetry(i, degrees);
  }

  robotStatus = "Calibration: all servos set to degrees";
}

void setTarget(float x, float y, float z, float pitch) {
  if (estop) return;

  targetX = x;
  targetY = y;
  targetZ = z;
  toolPitchDeg = pitch;

  calculateIK();
}

void moveTargetRelative(float dx, float dy, float dz, float dp) {
  if (estop) return;

  targetX += dx;
  targetY += dy;
  targetZ += dz;
  toolPitchDeg += dp;

  calculateIK();
}

void setToolMode(ToolMode mode) {
  activeToolMode = mode;
  calculateIK();

  robotStatus = activeToolMode == TOOL_GAP_MODE
    ? "Tool mode: deep gap grip"
    : "Tool mode: tip grip";
}

ToolMode getToolMode() {
  return activeToolMode;
}

void setClawTicks(int ticks) {
  setClawDegrees((float)ticks);
}

void setClawDegrees(float degrees) {
  if (estop) return;

  setServoTargetFromLogicalDegrees(CLAW, degrees);
  updateLegacyJointTelemetry(CLAW, degrees);
}

void clawOpen() {
  setClawDegrees(0.0);
  robotStatus = "Claw open";
}

void clawCloseSoft() {
  setClawDegrees(GRIPPER_TIPS_TOUCH_DEG);
  robotStatus = "Claw soft close";
}

void clawCloseFirm() {
  setClawDegrees(GRIPPER_TIPS_TOUCH_DEG);
  robotStatus = "Claw firm close";
}

void goHomePose() {
  stopTimeline();
  stopRobotMotion();

  targetX = 0.0;
  targetY = 170.0;
  targetZ = 105.0;
  toolPitchDeg = 0.0;

  calculateIK();
  robotStatus = "Home pose";
}

void goTableSkimPose() {
  stopTimeline();
  stopRobotMotion();

  targetX = 0.0;
  targetY = 185.0;
  targetZ = TABLE_SKIM_Z_MM;
  toolPitchDeg = -8.0;

  calculateIK();
  robotStatus = "Table skim pose";
}

void goCarryPose() {
  stopTimeline();
  stopRobotMotion();

  targetZ += 75.0;

  calculateIK();
  robotStatus = "Carry pose";
}

// ======================================================
// IK
// ======================================================
static void calculateWristTargetFromToolTarget() {
  float toolLength = getActiveToolLength();

  float rTarget = sqrt(targetX * targetX + targetY * targetY);
  float baseRad = atan2(targetX, targetY);
  float pitchRad = degToRad(toolPitchDeg);

  float wristR = rTarget - cos(pitchRad) * toolLength;
  float wristZLocal = targetZ - sin(pitchRad) * toolLength;

  wristX = sin(baseRad) * wristR;
  wristY = cos(baseRad) * wristR;
  wristZ = wristZLocal;
}

bool calculateIK() {
  calculateWristTargetFromToolTarget();

  float baseAngleRad = atan2(wristX, wristY);
  float baseServoDeg = radToDeg(baseAngleRad) + 90.0;

  float r = sqrt(wristX * wristX + wristY * wristY);
  float zEff = wristZ - Z_OFFSET_MM;
  float d = sqrt(r * r + zEff * zEff);
  float dForAngles = max(d, 0.001f);

  float cosElbow = (
    dForAngles * dForAngles -
    UPPER_ARM_MM * UPPER_ARM_MM -
    FOREARM_MM * FOREARM_MM
  ) / (2.0 * UPPER_ARM_MM * FOREARM_MM);

  cosElbow = constrain(cosElbow, -1.0, 1.0);

  float elbowInternalRad = acos(cosElbow);
  float alpha = atan2(zEff, r);

  float betaArg = (
    UPPER_ARM_MM * UPPER_ARM_MM +
    dForAngles * dForAngles -
    FOREARM_MM * FOREARM_MM
  ) / (2.0 * UPPER_ARM_MM * dForAngles);

  betaArg = constrain(betaArg, -1.0, 1.0);

  float beta = acos(betaArg);

  float shoulderRad = alpha - beta;
  float forearmAbsRad = shoulderRad + (PI - elbowInternalRad);

  float shoulderDeg = radToDeg(shoulderRad);
  float elbowInternalDeg = radToDeg(elbowInternalRad);
  float forearmAbsDeg = radToDeg(forearmAbsRad);

  float bicepServoDeg = shoulderDeg;
  float forearmServoDeg = forearmAbsDeg + 90.0;
  float wristServoDeg = toolPitchDeg - forearmAbsDeg + 90.0;

  baseServoDeg = constrain(baseServoDeg, 0, 180);
  bicepServoDeg = constrain(bicepServoDeg, 0, 180);
  forearmServoDeg = constrain(forearmServoDeg, 0, 180);
  wristServoDeg = constrain(wristServoDeg, 0, 180);

  setServoTargetFromLogicalDegrees(BASE, baseServoDeg);
  setServoTargetFromLogicalDegrees(BICEP, bicepServoDeg);
  setServoTargetFromLogicalDegrees(FOREARM, forearmServoDeg);
  setServoTargetFromLogicalDegrees(WRIST, wristServoDeg);

  lastBaseDeg = baseServoDeg;
  lastShoulderDeg = bicepServoDeg;
  lastElbowServoDeg = forearmServoDeg;
  lastWristDeg = wristServoDeg;

  robotStatus = "Tool IK OK";
  return true;
}

bool robotAtTarget(int toleranceTicks) {
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (!servos[i].enabled) continue;

    if (absFloat(servos[i].current - servos[i].target) > toleranceTicks) {
      return false;
    }
  }

  return true;
}

// ======================================================
// Continuous Movement
// ======================================================
void setSpeed(int speed) {
  speedSetting = constrain(speed, 1, 6);
  ikStepScale = 0.45 + (speedSetting * 0.18);
  robotStatus = "Speed set";
}

int getSpeed() {
  return speedSetting;
}

void setContinuousManualMove(int servo, int dir) {
  if (servo < 0 || servo >= NUM_SERVOS) return;
  if (estop) return;

  stopTimeline();

  activeServo = servo;
  activeDirection = dir;

  desiredDx = 0.0;
  desiredDy = 0.0;
  desiredDz = 0.0;
  desiredDp = 0.0;
  activeDx = 0.0;
  activeDy = 0.0;
  activeDz = 0.0;
  activeDp = 0.0;

  lastCommandTime = millis();

  ensureServoEnabled(servo);
  robotStatus = "Manual joint move";
}

void setContinuousIKMove(float dx, float dy, float dz, float dp) {
  if (estop) return;

  stopTimeline();

  activeServo = -1;
  activeDirection = 0;

  desiredDx = dx;
  desiredDy = dy;
  desiredDz = dz;
  desiredDp = dp;

  lastCommandTime = millis();

  robotStatus = "IK moving";
}

// ======================================================
// ESTOP
// ======================================================
void setEstop(bool enabled) {
  estop = enabled;

  if (estop) {
    allServosOff();
    stopRobotMotion();
    activeDx = 0.0;
    activeDy = 0.0;
    activeDz = 0.0;
    activeDp = 0.0;
    stopTimeline();
    robotStatus = "ESTOP ACTIVE";
  } else {
    allServosOff();
    stopRobotMotion();
    robotStatus = "ESTOP cleared";
  }
}

bool getEstop() {
  return estop;
}

// ======================================================
// Getters
// ======================================================
float getTargetX() { return targetX; }
float getTargetY() { return targetY; }
float getTargetZ() { return targetZ; }
float getTargetPitch() { return toolPitchDeg; }

float getWristX() { return wristX; }
float getWristY() { return wristY; }
float getWristZ() { return wristZ; }

float getLastBaseDeg() { return lastBaseDeg; }
float getLastShoulderDeg() { return lastShoulderDeg; }
float getLastElbowDeg() { return lastElbowServoDeg; }
float getLastWristDeg() { return lastWristDeg; }

int getClawTicks() { return roundedTicks(getServoTargetDegrees(CLAW)); }

float getServoCurrentDegrees(int servoIndex) {
  if (servoIndex < 0 || servoIndex >= NUM_SERVOS) return 0.0;
  return ticksToLogicalDegrees(servoIndex, servos[servoIndex].current);
}

float getServoTargetDegrees(int servoIndex) {
  if (servoIndex < 0 || servoIndex >= NUM_SERVOS) return 0.0;
  return ticksToLogicalDegrees(servoIndex, servos[servoIndex].target);
}

int getServoCurrentTicks(int servoIndex) {
  if (servoIndex < 0 || servoIndex >= NUM_SERVOS) return 0;
  return roundedTicks(servos[servoIndex].current);
}

int getServoTargetTicks(int servoIndex) {
  if (servoIndex < 0 || servoIndex >= NUM_SERVOS) return 0;
  return roundedTicks(servos[servoIndex].target);
}

String getToolModeName() {
  return activeToolMode == TOOL_GAP_MODE ? "Deep gap grip" : "Tip grip";
}

String getRobotStatus() {
  return robotStatus;
}
