#include <Arduino.h>
#include "Config.h"
#include "RobotArm.h"
#include "Timeline.h"

static Keyframe timeline[MAX_KEYFRAMES];
static int keyframeCount = 0;

static bool timelinePlaying = false;
static int timelineIndex = 0;

static unsigned long stateStart = 0;

static const int TL_IDLE = 0;
static const int TL_MOVING = 1;
static const int TL_GRAB = 2;
static const int TL_DROP = 3;
static const int TL_WAIT = 4;
static const int TL_DONE = 5;

static int timelineState = TL_IDLE;

// ======================================================
// Internal Helpers
// ======================================================
static String keyframeTypeName(int type) {
  if (type == KF_GRAB) return "GRAB";
  if (type == KF_DROP) return "DROP";
  if (type == KF_WAIT) return "WAIT";
  return "MOVE";
}

static void moveToKeyframe(int index) {
  if (index < 0 || index >= keyframeCount) return;

  Keyframe &kf = timeline[index];

  setToolMode(kf.toolMode);
  setTarget(kf.x, kf.y, kf.z, kf.pitch);
}

static bool appendKeyframe(
  int type,
  float x,
  float y,
  float z,
  float pitch,
  ToolMode toolMode,
  int clawTicks,
  unsigned long durationMs,
  unsigned long waitAfterMs
) {
  if (keyframeCount >= MAX_KEYFRAMES) {
    Serial.println("Timeline full");
    return false;
  }

  Keyframe &kf = timeline[keyframeCount];

  kf.used = true;
  kf.type = type;

  kf.x = x;
  kf.y = y;
  kf.z = z;
  kf.pitch = pitch;

  kf.toolMode = toolMode;
  kf.clawTicks = clawTicks;

  kf.durationMs = durationMs;
  kf.waitAfterMs = waitAfterMs;

  keyframeCount++;

  Serial.print("Added keyframe: ");
  Serial.println(keyframeTypeName(type));
  return true;
}

static bool addKeyframe(int type, int clawTicks) {
  return appendKeyframe(
    type,
    getTargetX(),
    getTargetY(),
    getTargetZ(),
    getTargetPitch(),
    getToolMode(),
    clawTicks,
    1200,
    200
  );
}

// ======================================================
// Public Timeline API
// ======================================================
void setupTimeline() {
  clearTimeline();
}

void addMoveKeyframe() {
  addKeyframe(KF_MOVE, -1);
}

void addGrabKeyframe() {
  // Captures current claw strength.
  addKeyframe(KF_GRAB, getClawTicks());
}

void addDropKeyframe() {
  // Move to this pose first, then open claw during playback.
  addKeyframe(KF_DROP, -1);
}

void addWaitKeyframe() {
  addKeyframe(KF_WAIT, -1);
}

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
) {
  if (type < KF_MOVE || type > KF_WAIT) {
    Serial.println("Invalid remote keyframe type");
    return false;
  }

  return appendKeyframe(type, x, y, z, pitch, toolMode, clawTicks, durationMs, waitAfterMs);
}

void deleteLastKeyframe() {
  if (keyframeCount <= 0) {
    Serial.println("No keyframes to delete");
    return;
  }

  keyframeCount--;
  timeline[keyframeCount].used = false;

  Serial.println("Deleted last keyframe");
}

void clearTimeline() {
  stopTimeline();

  for (int i = 0; i < MAX_KEYFRAMES; i++) {
    timeline[i].used = false;
    timeline[i].type = KF_MOVE;
    timeline[i].x = 0;
    timeline[i].y = 170;
    timeline[i].z = 80;
    timeline[i].pitch = 0;
    timeline[i].toolMode = TOOL_TIP_MODE;
    timeline[i].clawTicks = -1;
    timeline[i].durationMs = 1200;
    timeline[i].waitAfterMs = 200;
  }

  keyframeCount = 0;
}

void playTimeline() {
  if (keyframeCount <= 0) {
    Serial.println("Timeline empty");
    return;
  }

  stopRobotMotion();

  timelinePlaying = true;
  timelineIndex = 0;
  timelineState = TL_MOVING;
  stateStart = millis();

  moveToKeyframe(timelineIndex);

  Serial.println("Timeline playing");
}

void stopTimeline() {
  timelinePlaying = false;
  timelineIndex = 0;
  timelineState = TL_IDLE;
}

void updateTimeline() {
  if (!timelinePlaying) return;
  if (getEstop()) {
    stopTimeline();
    return;
  }

  unsigned long now = millis();

  if (timelineIndex >= keyframeCount) {
    timelineState = TL_DONE;
  }

  switch (timelineState) {
    case TL_MOVING: {
      bool reached = robotAtTarget(2);
      bool timedOut = now - stateStart > KEYFRAME_SETTLE_TIMEOUT_MS;

      if (reached || timedOut) {
        Keyframe &kf = timeline[timelineIndex];

        if (kf.type == KF_GRAB) {
          timelineState = TL_GRAB;
        } else if (kf.type == KF_DROP) {
          timelineState = TL_DROP;
        } else if (kf.type == KF_WAIT) {
          timelineState = TL_WAIT;
          stateStart = now;
        } else {
          timelineState = TL_WAIT;
          stateStart = now;
        }
      }

      break;
    }

    case TL_GRAB: {
      Keyframe &kf = timeline[timelineIndex];

      if (kf.clawTicks >= 0) {
        setClawTicks(kf.clawTicks);
      }

      timelineState = TL_WAIT;
      stateStart = now;

      Serial.println("Timeline grab");
      break;
    }

    case TL_DROP: {
      // Drop keyframe: move to pose first, then open claw.
      clawOpen();

      timelineState = TL_WAIT;
      stateStart = now;

      Serial.println("Timeline drop/open");
      break;
    }

    case TL_WAIT: {
      Keyframe &kf = timeline[timelineIndex];

      unsigned long waitTime = max(kf.waitAfterMs, CLAW_ACTION_WAIT_MS);

      if (now - stateStart >= waitTime) {
        timelineIndex++;

        if (timelineIndex >= keyframeCount) {
          timelineState = TL_DONE;
        } else {
          moveToKeyframe(timelineIndex);
          timelineState = TL_MOVING;
          stateStart = now;
        }
      }

      break;
    }

    case TL_DONE: {
      stopTimeline();
      Serial.println("Timeline complete");
      break;
    }

    default:
      stopTimeline();
      break;
  }
}

// ======================================================
// Getters / JSON
// ======================================================
int getKeyframeCount() {
  return keyframeCount;
}

bool getTimelinePlaying() {
  return timelinePlaying;
}

String getTimelineText() {
  String s = "";

  for (int i = 0; i < keyframeCount; i++) {
    s += String(i + 1);
    s += ":";
    s += keyframeTypeName(timeline[i].type);

    if (i < keyframeCount - 1) {
      s += " ";
    }
  }

  if (s.length() == 0) {
    s = "empty";
  }

  return s;
}

String getTimelineJson() {
  String json = "[";

  for (int i = 0; i < keyframeCount; i++) {
    Keyframe &kf = timeline[i];

    json += "{";
    json += "\"index\":" + String(i) + ",";
    json += "\"type\":\"" + keyframeTypeName(kf.type) + "\",";
    json += "\"x\":" + String(kf.x, 1) + ",";
    json += "\"y\":" + String(kf.y, 1) + ",";
    json += "\"z\":" + String(kf.z, 1) + ",";
    json += "\"pitch\":" + String(kf.pitch, 1) + ",";
    json += "\"toolMode\":" + String((int)kf.toolMode) + ",";
    json += "\"clawTicks\":" + String(kf.clawTicks) + ",";
    json += "\"durationMs\":" + String(kf.durationMs) + ",";
    json += "\"waitAfterMs\":" + String(kf.waitAfterMs);
    json += "}";

    if (i < keyframeCount - 1) {
      json += ",";
    }
  }

  json += "]";
  return json;
}
