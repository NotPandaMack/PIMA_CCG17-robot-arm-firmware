#pragma once
#include <Arduino.h>
#include "RobotArm.h"

// Use int constants instead of enum to avoid Arduino .ino prototype weirdness.
static const int KF_MOVE = 0;
static const int KF_GRAB = 1;
static const int KF_DROP = 2;
static const int KF_WAIT = 3;

struct Keyframe {
  bool used;
  int type;

  float x;
  float y;
  float z;
  float pitch;

  ToolMode toolMode;
  int clawTicks;

  unsigned long durationMs;
  unsigned long waitAfterMs;
};

void setupTimeline();
void updateTimeline();

void addMoveKeyframe();
void addGrabKeyframe();
void addDropKeyframe();
void addWaitKeyframe();
bool addRemoteKeyframe(
  int type,
  float x,
  float y,
  float z,
  float pitch,
  ToolMode toolMode,
  int clawTicks,
  unsigned long durationMs,
  unsigned long waitAfterMs
);

void deleteLastKeyframe();
void clearTimeline();

void playTimeline();
void stopTimeline();

int getKeyframeCount();
bool getTimelinePlaying();

String getTimelineText();
String getTimelineJson();
