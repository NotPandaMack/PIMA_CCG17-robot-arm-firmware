#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include "Config.h"
#include "RobotArm.h"
#include "Timeline.h"

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDRESS);

// ======================================================
// Calibration
// ======================================================
static bool INVERT_BASE    = false;
static bool INVERT_BICEP   = false;
static bool INVERT_FOREARM = true;
static bool INVERT_WRIST   = false;

static float BASE_OFFSET_DEG    = 0.0;
static float BICEP_OFFSET_DEG   = 0.0;
static float FOREARM_OFFSET_DEG = 0.0;
static float WRIST_OFFSET_DEG   = 0.0;

static float WRIST_NEUTRAL_DEG = 90.0;

// If wrist compensation moves the wrong way, set true.
static bool INVERT_WRIST_COMPENSATION = false;

// If the arm folds the wrong way, set true.
static bool ELBOW_UP_SOLUTION = false;

// ======================================================
// Servo State
// ======================================================
struct ServoState {
  int current;
  int target;
  bool enabled;
  int minSafe;
  int maxSafe;
};

static ServoState servos[NUM_SERVOS] = {
  { SERVO_MID, SERVO_MID, false, 195, 420 }, // Base
  { SERVO_MID, SERVO_MID, false, 195, 420 }, // Bicep
  { SERVO_MID, SERVO_MID, false, 195, 420 }, // Forearm
  { SERVO_MID, SERVO_MID, false, 195, 420 }, // Wrist
  { SERVO_MID, SERVO_MID, false, 220, 390 }  // Claw
};

static int lastWritten[NUM_SERVOS] = { -1, -1, -1, -1, -1 };

static bool estop = false;

static unsigned long lastUpdate = 0;

static int servoStepSize = 1;
static float ikStepScale = 1.0;

static int activeServo = -1;
static int activeDirection = 0;

static float activeDx = 0.0;
static float activeDy = 0.0;
static float activeDz = 0.0;
static float activeDp = 0.0;

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
static float lastForearmAbsDeg = 0.0;
static float lastWristDeg = 90.0;

// ======================================================
// Utility
// ======================================================
static float clampFloat(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
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
static void disableServo(int i) {
  if (i < 0 || i >= NUM_SERVOS) return;

  pwm.setPWM(SERVO_CHANNELS[i], 0, 4096);
  servos[i].enabled = false;
  lastWritten[i] = -1;
}

static void enableServo(int i) {
  if (i < 0 || i >= NUM_SERVOS) return;

  servos[i].target = constrain(servos[i].target, servos[i].minSafe, servos[i].maxSafe);
  servos[i].current = servos[i].target;
  servos[i].enabled = true;

  pwm.setPWM(SERVO_CHANNELS[i], 0, servos[i].current);
  lastWritten[i] = servos[i].current;
}

static void ensureServoEnabled(int i) {
  if (i < 0 || i >= NUM_SERVOS) return;

  if (!servos[i].enabled) {
    enableServo(i);
  }
}

static int angleToTicks(float angleDeg, bool invert, float offsetDeg) {
  angleDeg += offsetDeg;
  angleDeg = constrain(angleDeg, 0, 180);

  if (invert) {
    angleDeg = 180.0 - angleDeg;
  }

  int ticks = map((int)angleDeg, 0, 180, SERVO_MIN, SERVO_MAX);
  return constrain(ticks, SERVO_MIN, SERVO_MAX);
}

static void setServoTargetFromAngle(int servoIndex, float angleDeg, bool invert, float offsetDeg) {
  int ticks = angleToTicks(angleDeg, invert, offsetDeg);

  servos[servoIndex].target = constrain(
    ticks,
    servos[servoIndex].minSafe,
    servos[servoIndex].maxSafe
  );

  ensureServoEnabled(servoIndex);
}

// ======================================================
// Public Robot Setup/Update
// ======================================================
void setupRobotArm() {
  Wire.begin(SDA_PIN, SCL_PIN);

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
    servos[activeServo].target += activeDirection * servoStepSize;
    servos[activeServo].target = constrain(
      servos[activeServo].target,
      servos[activeServo].minSafe,
      servos[activeServo].maxSafe
    );
  }

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

    if (servos[i].current < servos[i].target) {
      servos[i].current += servoStepSize;
      if (servos[i].current > servos[i].target) {
        servos[i].current = servos[i].target;
      }
    }

    if (servos[i].current > servos[i].target) {
      servos[i].current -= servoStepSize;
      if (servos[i].current < servos[i].target) {
        servos[i].current = servos[i].target;
      }
    }

    if (lastWritten[i] != servos[i].current) {
      pwm.setPWM(SERVO_CHANNELS[i], 0, servos[i].current);
      lastWritten[i] = servos[i].current;
    }
  }
}

// ======================================================
// Public Movement Functions
// ======================================================
void stopRobotMotion() {
  activeServo = -1;
  activeDirection = 0;

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
  if (estop) return;

  servos[CLAW].target = constrain(ticks, servos[CLAW].minSafe, servos[CLAW].maxSafe);
  ensureServoEnabled(CLAW);
}

void clawOpen() {
  setClawTicks(servos[CLAW].minSafe);
  robotStatus = "Claw open";
}

void clawCloseSoft() {
  setClawTicks(335);
  robotStatus = "Claw soft close";
}

void clawCloseFirm() {
  setClawTicks(servos[CLAW].maxSafe);
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
  targetZ = clampFloat(targetZ, TARGET_Z_MIN, TARGET_Z_MAX);

  calculateIK();
  robotStatus = "Carry pose";
}

// ======================================================
// IK
// ======================================================
static void clampTargetWorkspace() {
  targetX = clampFloat(targetX, TARGET_X_MIN, TARGET_X_MAX);
  targetY = clampFloat(targetY, TARGET_Y_MIN, TARGET_Y_MAX);
  targetZ = clampFloat(targetZ, TARGET_Z_MIN, TARGET_Z_MAX);
  toolPitchDeg = clampFloat(toolPitchDeg, TOOL_PITCH_MIN, TOOL_PITCH_MAX);
}

static void calculateWristTargetFromToolTarget() {
  float toolLength = getActiveToolLength();

  float rTarget = sqrt(targetX * targetX + targetY * targetY);
  float baseRad = atan2(targetX, targetY);
  float pitchRad = degToRad(toolPitchDeg);

  float wristR = rTarget - cos(pitchRad) * toolLength;
  float wristZLocal = targetZ - sin(pitchRad) * toolLength;

  wristR = max(wristR, 20.0f);

  wristX = sin(baseRad) * wristR;
  wristY = cos(baseRad) * wristR;
  wristZ = wristZLocal;
}

static void clampWristToReachable() {
  float r = sqrt(wristX * wristX + wristY * wristY);
  float zEff = wristZ - Z_OFFSET_MM;
  float d = sqrt(r * r + zEff * zEff);

  float maxReach = UPPER_ARM_MM + FOREARM_MM - REACH_MARGIN_MM;
  float minReach = abs(UPPER_ARM_MM - FOREARM_MM) + REACH_MARGIN_MM;

  if (d > maxReach) {
    float scale = maxReach / d;
    r *= scale;
    zEff *= scale;

    float baseRad = atan2(wristX, wristY);
    wristX = sin(baseRad) * r;
    wristY = cos(baseRad) * r;
    wristZ = zEff + Z_OFFSET_MM;

    robotStatus = "Wrist target clamped: too far";
  }

  if (d < minReach) {
    float scale = minReach / max(d, 1.0f);
    r *= scale;
    zEff *= scale;

    float baseRad = atan2(wristX, wristY);
    wristX = sin(baseRad) * r;
    wristY = cos(baseRad) * r;
    wristZ = zEff + Z_OFFSET_MM;

    robotStatus = "Wrist target clamped: too close";
  }
}

bool calculateIK() {
  clampTargetWorkspace();
  calculateWristTargetFromToolTarget();
  clampWristToReachable();

  float baseAngleRad = atan2(wristX, wristY);
  float baseServoDeg = radToDeg(baseAngleRad) + 90.0;

  float r = sqrt(wristX * wristX + wristY * wristY);
  float zEff = wristZ - Z_OFFSET_MM;
  float d = sqrt(r * r + zEff * zEff);

  if (d > (UPPER_ARM_MM + FOREARM_MM) || d < abs(UPPER_ARM_MM - FOREARM_MM)) {
    robotStatus = "IK failed: unreachable";
    Serial.println(robotStatus);
    return false;
  }

  float cosElbow = (
    d * d -
    UPPER_ARM_MM * UPPER_ARM_MM -
    FOREARM_MM * FOREARM_MM
  ) / (2.0 * UPPER_ARM_MM * FOREARM_MM);

  cosElbow = constrain(cosElbow, -1.0, 1.0);

  float elbowInternalRad = acos(cosElbow);
  float alpha = atan2(zEff, r);

  float betaArg = (
    UPPER_ARM_MM * UPPER_ARM_MM +
    d * d -
    FOREARM_MM * FOREARM_MM
  ) / (2.0 * UPPER_ARM_MM * d);

  betaArg = constrain(betaArg, -1.0, 1.0);

  float beta = acos(betaArg);

  float shoulderRad = ELBOW_UP_SOLUTION ? alpha - beta : alpha + beta;

  float forearmAbsRad = ELBOW_UP_SOLUTION
    ? shoulderRad + (PI - elbowInternalRad)
    : shoulderRad - (PI - elbowInternalRad);

  float shoulderDeg = radToDeg(shoulderRad);
  float elbowInternalDeg = radToDeg(elbowInternalRad);
  float forearmAbsDeg = radToDeg(forearmAbsRad);

  float bicepServoDeg = shoulderDeg;
  float forearmServoDeg = 180.0 - elbowInternalDeg;

  float wristServoDeg;

  if (!INVERT_WRIST_COMPENSATION) {
    wristServoDeg = WRIST_NEUTRAL_DEG + toolPitchDeg - forearmAbsDeg;
  } else {
    wristServoDeg = WRIST_NEUTRAL_DEG - toolPitchDeg + forearmAbsDeg;
  }

  baseServoDeg = constrain(baseServoDeg, 0, 180);
  bicepServoDeg = constrain(bicepServoDeg, 0, 180);
  forearmServoDeg = constrain(forearmServoDeg, 0, 180);

  // Important: protects the wrist/claw collision.
  wristServoDeg = constrain(wristServoDeg, WRIST_MIN_SAFE_DEG, WRIST_MAX_UP_SAFE_DEG);

  setServoTargetFromAngle(BASE, baseServoDeg, INVERT_BASE, BASE_OFFSET_DEG);
  setServoTargetFromAngle(BICEP, bicepServoDeg, INVERT_BICEP, BICEP_OFFSET_DEG);
  setServoTargetFromAngle(FOREARM, forearmServoDeg, INVERT_FOREARM, FOREARM_OFFSET_DEG);
  setServoTargetFromAngle(WRIST, wristServoDeg, INVERT_WRIST, WRIST_OFFSET_DEG);

  lastBaseDeg = baseServoDeg;
  lastShoulderDeg = bicepServoDeg;
  lastElbowServoDeg = forearmServoDeg;
  lastForearmAbsDeg = forearmAbsDeg;
  lastWristDeg = wristServoDeg;

  robotStatus = "Tool IK OK";
  return true;
}

bool robotAtTarget(int toleranceTicks) {
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (!servos[i].enabled) continue;

    if (abs(servos[i].current - servos[i].target) > toleranceTicks) {
      return false;
    }
  }

  return true;
}

// ======================================================
// Continuous Movement
// ======================================================
void setSpeed(int speed) {
  servoStepSize = constrain(speed, 1, 6);
  ikStepScale = 0.65 + (servoStepSize * 0.35);
  robotStatus = "Speed set";
}

int getSpeed() {
  return servoStepSize;
}

void setContinuousManualMove(int servo, int dir) {
  if (servo < 0 || servo >= NUM_SERVOS) return;
  if (estop) return;

  stopTimeline();

  activeServo = servo;
  activeDirection = dir;

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

  activeDx = dx;
  activeDy = dy;
  activeDz = dz;
  activeDp = dp;

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

int getClawTicks() { return servos[CLAW].target; }

String getToolModeName() {
  return activeToolMode == TOOL_GAP_MODE ? "Deep gap grip" : "Tip grip";
}

String getRobotStatus() {
  return robotStatus;
}