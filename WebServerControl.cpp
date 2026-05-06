#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>

#include "Config.h"
#include "RobotArm.h"
#include "Timeline.h"
#include "WebServerControl.h"

static ESP8266WebServer server(80);
static WebSocketsServer webSocket = WebSocketsServer(81);

// ======================================================
// Status JSON
// ======================================================
static String makeStatusJson() {
  String json = "{";

  json += "\"status\":\"" + getRobotStatus() + "\",";
  json += "\"toolMode\":\"" + getToolModeName() + "\",";
  json += "\"x\":" + String(getTargetX(), 1) + ",";
  json += "\"y\":" + String(getTargetY(), 1) + ",";
  json += "\"z\":" + String(getTargetZ(), 1) + ",";
  json += "\"pitch\":" + String(getTargetPitch(), 1) + ",";

  json += "\"wristX\":" + String(getWristX(), 1) + ",";
  json += "\"wristY\":" + String(getWristY(), 1) + ",";
  json += "\"wristZ\":" + String(getWristZ(), 1) + ",";

  json += "\"base\":" + String(getLastBaseDeg(), 1) + ",";
  json += "\"shoulder\":" + String(getLastShoulderDeg(), 1) + ",";
  json += "\"elbow\":" + String(getLastElbowDeg(), 1) + ",";
  json += "\"wrist\":" + String(getLastWristDeg(), 1) + ",";

  json += "\"clawTicks\":" + String(getClawTicks()) + ",";
  json += "\"speed\":" + String(getSpeed()) + ",";

  json += "\"keyframeCount\":" + String(getKeyframeCount()) + ",";
  json += "\"timelinePlaying\":" + String(getTimelinePlaying() ? "true" : "false") + ",";
  json += "\"timelineText\":\"" + getTimelineText() + "\",";
  json += "\"timeline\":" + getTimelineJson() + ",";

  json += "\"estop\":" + String(getEstop() ? "true" : "false");

  json += "}";

  return json;
}

// ======================================================
// Simple Test Web UI
// ======================================================
static const char index_html[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
<title>Robot Arm Controller</title>
<style>
  body {
    background: #0f1117;
    color: white;
    font-family: Arial, sans-serif;
    text-align: center;
    padding: 12px;
  }
  .panel {
    background: #181b24;
    border: 1px solid #2d3240;
    border-radius: 14px;
    max-width: 760px;
    margin: 10px auto;
    padding: 12px;
  }
  button {
    border: none;
    border-radius: 12px;
    color: white;
    font-weight: bold;
    font-size: 14px;
    padding: 13px 8px;
    min-height: 50px;
    background: #333a4a;
    touch-action: none;
  }
  button:active {
    transform: scale(0.97);
    filter: brightness(1.25);
  }
  .grid2 {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 8px;
    margin-top: 8px;
  }
  .grid3 {
    display: grid;
    grid-template-columns: 1fr 1fr 1fr;
    gap: 8px;
    margin-top: 8px;
  }
  .danger { background: #c0392b; }
  .safe { background: #1f7a4c; }
  .blue { background: #2f6fa3; }
  .gold { background: #9b6b22; }
  .purple { background: #6f42c1; }
  .minus { background: #8a2a2a; }
  .plus { background: #167a3b; }
  .neutral { background: #555b66; }
  .status {
    font-family: monospace;
    text-align: left;
    background: #10131a;
    border: 1px solid #303646;
    border-radius: 10px;
    padding: 8px;
    font-size: 13px;
    line-height: 1.45;
  }
</style>
</head>

<body>
  <h2>Robot Arm Controller</h2>

  <div class="panel">
    <h3>Status</h3>
    <div id="status" class="status">Connecting...</div>

    <div class="grid2">
      <button class="danger" onclick="send('ESTOP')">ESTOP</button>
      <button class="safe" onclick="send('CLEAR_ESTOP')">Clear ESTOP</button>
    </div>

    <button class="neutral" style="width:100%;margin-top:8px;" onclick="send('STOP')">Stop / Cancel</button>
    <button class="neutral" style="width:100%;margin-top:8px;" onclick="send('ALL_OFF')">All Servos Off</button>
  </div>

  <div class="panel">
    <h3>Timeline</h3>

    <div class="grid2">
      <button class="safe" onclick="send('ADD_MOVE_KEYFRAME')">Add Move</button>
      <button class="gold" onclick="send('ADD_GRAB_KEYFRAME')">Add Grab</button>
    </div>

    <div class="grid2">
      <button class="purple" onclick="send('ADD_DROP_KEYFRAME')">Add Drop</button>
      <button class="blue" onclick="send('PLAY_TIMELINE')">Play</button>
    </div>

    <div class="grid2">
      <button class="minus" onclick="send('DELETE_LAST_KEYFRAME')">Delete Last</button>
      <button class="danger" onclick="send('CLEAR_TIMELINE')">Clear Timeline</button>
    </div>
  </div>

  <div class="panel">
    <h3>Poses / Tool</h3>
    <div class="grid3">
      <button class="blue" onclick="send('IK_HOME')">Home</button>
      <button class="gold" onclick="send('TABLE_SKIM_POSE')">Table Skim</button>
      <button class="purple" onclick="send('CARRY_POSE')">Lift</button>
    </div>

    <div class="grid2">
      <button class="blue" onclick="send('TOOL_TIP')">Tip Grip</button>
      <button class="purple" onclick="send('TOOL_GAP')">Deep Gap</button>
    </div>
  </div>

  <div class="panel">
    <h3>Claw</h3>
    <div class="grid3">
      <button class="plus" onclick="send('CLAW_OPEN')">Open</button>
      <button class="gold" onclick="send('CLAW_CLOSE_SOFT')">Soft Close</button>
      <button class="minus" onclick="send('CLAW_CLOSE_FIRM')">Firm Close</button>
    </div>
  </div>

  <div class="panel">
    <h3>IK Movement</h3>

    <div class="grid3">
      <button class="minus ik" data-dx="-1.2" data-dy="0" data-dz="0" data-dp="0">X Left</button>
      <button class="neutral" onclick="send('STOP')">Stop</button>
      <button class="plus ik" data-dx="1.2" data-dy="0" data-dz="0" data-dp="0">X Right</button>
    </div>

    <div class="grid2">
      <button class="minus ik" data-dx="0" data-dy="-1.2" data-dz="0" data-dp="0">Y Back</button>
      <button class="plus ik" data-dx="0" data-dy="1.2" data-dz="0" data-dp="0">Y Forward</button>
    </div>

    <div class="grid2">
      <button class="minus ik" data-dx="0" data-dy="0" data-dz="-1.0" data-dp="0">Z Down</button>
      <button class="plus ik" data-dx="0" data-dy="0" data-dz="1.0" data-dp="0">Z Up</button>
    </div>

    <div class="grid2">
      <button class="minus ik" data-dx="0" data-dy="0" data-dz="-0.25" data-dp="0">Fine Z Down</button>
      <button class="plus ik" data-dx="0" data-dy="0" data-dz="0.25" data-dp="0">Fine Z Up</button>
    </div>

    <div class="grid2">
      <button class="minus ik" data-dx="0" data-dy="0" data-dz="0" data-dp="-0.6">Pitch Down</button>
      <button class="plus ik" data-dx="0" data-dy="0" data-dz="0" data-dp="0.6">Pitch Up</button>
    </div>
  </div>

<script>
let ws;

function connectWS() {
  ws = new WebSocket("ws://" + location.hostname + ":81/");
  ws.onclose = () => setTimeout(connectWS, 1000);
}

function send(msg) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(msg);
  }
}

function makeHoldable(selector, buildMessage) {
  document.querySelectorAll(selector).forEach(btn => {
    let intervalId = null;

    const start = e => {
      e.preventDefault();
      const msg = buildMessage(btn);
      send(msg);

      if (intervalId) clearInterval(intervalId);

      intervalId = setInterval(() => {
        send(msg);
      }, 120);
    };

    const stop = e => {
      e.preventDefault();

      if (intervalId) {
        clearInterval(intervalId);
        intervalId = null;
      }

      send("STOP");
    };

    btn.addEventListener("mousedown", start);
    btn.addEventListener("mouseup", stop);
    btn.addEventListener("mouseleave", stop);

    btn.addEventListener("touchstart", start, {passive:false});
    btn.addEventListener("touchend", stop, {passive:false});
    btn.addEventListener("touchcancel", stop, {passive:false});
  });
}

function bindButtons() {
  makeHoldable(".ik", btn => {
    return "IKMOVE:" +
      btn.dataset.dx + ":" +
      btn.dataset.dy + ":" +
      btn.dataset.dz + ":" +
      btn.dataset.dp;
  });
}

async function updateStatus() {
  try {
    const res = await fetch("/status");
    const d = await res.json();

    document.getElementById("status").innerHTML =
      "Status: " + d.status + "<br>" +
      "Target: X " + d.x + " | Y " + d.y + " | Z " + d.z + " | Pitch " + d.pitch + "<br>" +
      "Joints: Base " + d.base + " | Shoulder " + d.shoulder + " | Elbow " + d.elbow + " | Wrist " + d.wrist + "<br>" +
      "Claw Ticks: " + d.clawTicks + "<br>" +
      "Timeline: " + d.timelineText + "<br>" +
      "Playing: " + d.timelinePlaying + " | ESTOP: " + d.estop;
  } catch(e) {
    document.getElementById("status").innerText = "Status unavailable";
  }
}

connectWS();
bindButtons();
setInterval(updateStatus, 500);
updateStatus();
</script>

</body>
</html>
)rawhtml";

// ======================================================
// HTTP Handlers
// ======================================================
static void addCors() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}

static void handleRoot() {
  addCors();
  server.send_P(200, "text/html", index_html);
}

static void handleStatus() {
  addCors();
  server.send(200, "application/json", makeStatusJson());
}

static void handleOptions() {
  addCors();
  server.send(204);
}

// ======================================================
// WebSocket Commands
// ======================================================
static int parseFieldEnd(const String &msg, int start) {
  int end = msg.indexOf(':', start);
  if (end < 0) return msg.length();
  return end;
}

static String getCommandField(const String &msg, int fieldIndex) {
  int start = 0;

  for (int i = 0; i < fieldIndex; i++) {
    start = msg.indexOf(':', start);
    if (start < 0) return "";
    start++;
  }

  int end = parseFieldEnd(msg, start);
  return msg.substring(start, end);
}

static int keyframeTypeFromName(String typeName) {
  typeName.toUpperCase();

  if (typeName == "GRAB") return KF_GRAB;
  if (typeName == "DROP") return KF_DROP;
  if (typeName == "WAIT") return KF_WAIT;
  return KF_MOVE;
}

static ToolMode toolModeFromInt(int value) {
  return value == 1 ? TOOL_GAP_MODE : TOOL_TIP_MODE;
}

static String makeCapabilitiesJson() {
  String json = "{";
  json += "\"type\":\"CAPABILITIES\",";
  json += "\"setTarget\":true,";
  json += "\"setTool\":true,";
  json += "\"setClaw\":true,";
  json += "\"addKeyframeCompact\":true,";
  json += "\"playRemoteTimeline\":true,";
  json += "\"maxKeyframes\":" + String(MAX_KEYFRAMES);
  json += "}";
  return json;
}

static void handleCommand(uint8_t clientNum, String msg) {
  if (msg == "ESTOP") {
    setEstop(true);
    return;
  }

  if (msg == "CLEAR_ESTOP") {
    setEstop(false);
    return;
  }

  if (msg == "ALL_OFF") {
    allServosOff();
    stopRobotMotion();
    stopTimeline();
    return;
  }

  if (msg == "STOP") {
    stopRobotMotion();
    stopTimeline();
    return;
  }

  if (msg == "IK_HOME") {
    goHomePose();
    return;
  }

  if (msg == "TABLE_SKIM_POSE") {
    goTableSkimPose();
    return;
  }

  if (msg == "CARRY_POSE") {
    goCarryPose();
    return;
  }

  if (msg == "CLAW_OPEN") {
    stopTimeline();
    clawOpen();
    return;
  }

  if (msg == "CLAW_CLOSE_SOFT") {
    stopTimeline();
    clawCloseSoft();
    return;
  }

  if (msg == "CLAW_CLOSE_FIRM") {
    stopTimeline();
    clawCloseFirm();
    return;
  }

  if (msg == "TOOL_TIP") {
    setToolMode(TOOL_TIP_MODE);
    return;
  }

  if (msg == "TOOL_GAP") {
    setToolMode(TOOL_GAP_MODE);
    return;
  }

  if (msg == "ADD_MOVE_KEYFRAME") {
    addMoveKeyframe();
    return;
  }

  if (msg == "ADD_GRAB_KEYFRAME") {
    addGrabKeyframe();
    return;
  }

  if (msg == "ADD_DROP_KEYFRAME") {
    addDropKeyframe();
    return;
  }

  if (msg == "ADD_WAIT_KEYFRAME") {
    addWaitKeyframe();
    return;
  }

  if (msg == "DELETE_LAST_KEYFRAME") {
    deleteLastKeyframe();
    return;
  }

  if (msg == "CLEAR_TIMELINE") {
    clearTimeline();
    return;
  }

  if (msg == "PLAY_TIMELINE") {
    playTimeline();
    return;
  }

  if (msg == "PLAY_REMOTE_TIMELINE") {
    playTimeline();
    return;
  }

  if (msg == "GET_CAPABILITIES") {
    String capabilities = makeCapabilitiesJson();
    webSocket.sendTXT(clientNum, capabilities);
    return;
  }

  // SET_TARGET:x:y:z:pitch
  if (msg.startsWith("SET_TARGET:")) {
    float x = getCommandField(msg, 1).toFloat();
    float y = getCommandField(msg, 2).toFloat();
    float z = getCommandField(msg, 3).toFloat();
    float pitch = getCommandField(msg, 4).toFloat();

    stopTimeline();
    setTarget(x, y, z, pitch);
    return;
  }

  // SET_TOOL:0 for tip, SET_TOOL:1 for gap.
  if (msg.startsWith("SET_TOOL:")) {
    setToolMode(toolModeFromInt(getCommandField(msg, 1).toInt()));
    return;
  }

  // SET_CLAW:ticks
  if (msg.startsWith("SET_CLAW:")) {
    stopTimeline();
    setClawTicks(getCommandField(msg, 1).toInt());
    return;
  }

  // ADD_KEYFRAME:type:x:y:z:pitch:toolMode:clawTicks:durationMs:waitAfterMs
  // type is MOVE, GRAB, DROP, or WAIT. toolMode is 0 for tip, 1 for gap.
  if (msg.startsWith("ADD_KEYFRAME:")) {
    int type = keyframeTypeFromName(getCommandField(msg, 1));
    float x = getCommandField(msg, 2).toFloat();
    float y = getCommandField(msg, 3).toFloat();
    float z = getCommandField(msg, 4).toFloat();
    float pitch = getCommandField(msg, 5).toFloat();
    ToolMode toolMode = toolModeFromInt(getCommandField(msg, 6).toInt());
    int clawTicks = getCommandField(msg, 7).toInt();
    unsigned long durationMs = (unsigned long)getCommandField(msg, 8).toInt();
    unsigned long waitAfterMs = (unsigned long)getCommandField(msg, 9).toInt();

    addRemoteKeyframe(type, x, y, z, pitch, toolMode, clawTicks, durationMs, waitAfterMs);
    return;
  }

  if (msg.startsWith("SPEED:")) {
    int v = msg.substring(6).toInt();
    setSpeed(v);
    return;
  }

  if (msg.startsWith("HOLD:")) {
    int c1 = msg.indexOf(':');
    int c2 = msg.indexOf(':', c1 + 1);

    if (c1 > 0 && c2 > 0) {
      int servo = msg.substring(c1 + 1, c2).toInt();
      int dir = msg.substring(c2 + 1).toInt();
      setContinuousManualMove(servo, dir);
    }

    return;
  }

  // IKMOVE:dx:dy:dz:dp
  if (msg.startsWith("IKMOVE:")) {
    int c1 = msg.indexOf(':');
    int c2 = msg.indexOf(':', c1 + 1);
    int c3 = msg.indexOf(':', c2 + 1);
    int c4 = msg.indexOf(':', c3 + 1);

    if (c1 > 0 && c2 > 0 && c3 > 0 && c4 > 0) {
      float dx = msg.substring(c1 + 1, c2).toFloat();
      float dy = msg.substring(c2 + 1, c3).toFloat();
      float dz = msg.substring(c3 + 1, c4).toFloat();
      float dp = msg.substring(c4 + 1).toFloat();

      setContinuousIKMove(dx, dy, dz, dp);
    }

    return;
  }
}

static void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if (type != WStype_TEXT) return;

  String msg = String((char*)payload);
  handleCommand(num, msg);
}

// ======================================================
// Public WebServer API
// ======================================================
void setupWebServerControl() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to WiFi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.print("Open controller: http://");
  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, handleStatus);

  server.on("/", HTTP_OPTIONS, handleOptions);
  server.on("/status", HTTP_OPTIONS, handleOptions);

  server.begin();

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}

void updateWebServerControl() {
  server.handleClient();
  webSocket.loop();
}
