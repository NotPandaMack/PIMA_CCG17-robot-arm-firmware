#pragma once
#include <Arduino.h>

enum ServoIndex {
  BASE = 0,
  BICEP = 1,
  FOREARM = 2,
  WRIST = 3,
  CLAW = 4
};

enum ToolMode {
  TOOL_TIP_MODE = 0,
  TOOL_GAP_MODE = 1
};

void setupRobotArm();
void updateRobotArm();

void stopRobotMotion();
void allServosOff();
void calibrateServosZeroDegrees();
void setServoDegrees(int servoIndex, float degrees);
void setAllServoDegrees(float degrees);

void setTarget(float x, float y, float z, float pitch);
void moveTargetRelative(float dx, float dy, float dz, float dp);

void setToolMode(ToolMode mode);
ToolMode getToolMode();

void clawOpen();
void clawCloseSoft();
void clawCloseFirm();
void setClawDegrees(float degrees);
void setClawTicks(int ticks);

void goHomePose();
void goTableSkimPose();
void goCarryPose();

bool calculateIK();
bool robotAtTarget(int toleranceTicks = 2);

float getTargetX();
float getTargetY();
float getTargetZ();
float getTargetPitch();

float getWristX();
float getWristY();
float getWristZ();

float getLastBaseDeg();
float getLastShoulderDeg();
float getLastElbowDeg();
float getLastWristDeg();

int getClawTicks();
float getServoCurrentDegrees(int servoIndex);
float getServoTargetDegrees(int servoIndex);
int getServoCurrentTicks(int servoIndex);
int getServoTargetTicks(int servoIndex);

String getToolModeName();
String getRobotStatus();

void setSpeed(int speed);
int getSpeed();

void setContinuousManualMove(int servo, int dir);
void setContinuousIKMove(float dx, float dy, float dz, float dp);

void setEstop(bool enabled);
bool getEstop();
