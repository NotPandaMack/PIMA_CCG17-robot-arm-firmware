#pragma once
#include <Arduino.h>

// ======================================================
// WiFi
// ======================================================
static const char WIFI_SSID[] = "WIFI_SSID";
static const char WIFI_PASSWORD[] = "WIFI_PASSWORD";

// ======================================================
// ESP8266 I2C Pins
// ======================================================
#define SDA_PIN 4
#define SCL_PIN 5

// ======================================================
// PCA9685
// ======================================================
#define PCA9685_ADDRESS 0x7F

// ======================================================
// Servo Signal Settings
// ======================================================
// Approx at 50 Hz:
// 195 ≈ 950 us
// 307 ≈ 1500 us
// 420 ≈ 2050 us
static const int SERVO_MIN = 195;
static const int SERVO_MID = 307;
static const int SERVO_MAX = 420;

#define NUM_SERVOS 5

static const int SERVO_CHANNELS[NUM_SERVOS] = {
  0,   // Base
  1,   // Bicep / Shoulder
  2,   // Forearm / Elbow
  3,   // Wrist up/down
  4    // Claw open/close
};

// ======================================================
// Arm Geometry
// ======================================================
static const float UPPER_ARM_MM = 131.0;
static const float FOREARM_MM   = 125.4;

static const float WRIST_TO_CLAW_SERVO_MM = 26.5;
static const float CLAW_SERVO_TO_TIPS_MM  = 46.7;
static const float CLAW_SERVO_TO_GAP_MM   = 29.0;

static const float TOOL_TIP_MM = WRIST_TO_CLAW_SERVO_MM + CLAW_SERVO_TO_TIPS_MM; // 73.2
static const float TOOL_GAP_MM = WRIST_TO_CLAW_SERVO_MM + CLAW_SERVO_TO_GAP_MM;  // 55.5

// Table surface -> center of shoulder/bicep servo shaft.
// Re-measure later for better table accuracy.
static const float Z_OFFSET_MM = 140.0;

static const float TABLE_SKIM_Z_MM = 20.0;

// ======================================================
// Timing
// ======================================================
static const int UPDATE_MS = 20;
static const int COMMAND_TIMEOUT_MS = 450;

// ======================================================
// Motion Smoothing
// ======================================================
// Per 20 ms update. Higher values move faster; lower values reduce jerk.
static const float SERVO_MAX_SPEED_TICKS[NUM_SERVOS] = {
  2.2,  // Base
  1.6,  // Bicep / Shoulder
  1.6,  // Forearm / Elbow
  1.9,  // Wrist up/down
  3.2   // Claw open/close
};

static const float SERVO_ACCEL_TICKS[NUM_SERVOS] = {
  0.22, // Base
  0.16, // Bicep / Shoulder
  0.16, // Forearm / Elbow
  0.20, // Wrist up/down
  0.45  // Claw open/close
};

static const float SERVO_SETTLE_DEADBAND_TICKS = 0.35;
static const float MANUAL_TARGET_TICKS_PER_UPDATE = 1.5;
static const float IK_INPUT_SMOOTHING = 0.22;
static const float IK_STOP_SMOOTHING = 0.38;

// ======================================================
// Timeline
// ======================================================
#define MAX_KEYFRAMES 12
static const unsigned long KEYFRAME_SETTLE_TIMEOUT_MS = 3500;
static const unsigned long CLAW_ACTION_WAIT_MS = 850;
