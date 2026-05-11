# PIMA_CCG17 Robot Arm Firmware

A modular ESP8266-based robotic arm control system with inverse kinematics, WebSocket control, teachable motion timelines, servo safety limits, and Raspberry Pi-hosted web UI integration for manual and semi-autonomous pick-and-place operation.

## Overview

This project controls a desktop robotic arm using an ESP8266, a PCA9685 16-channel PWM servo driver, and multiple servos for base rotation, shoulder/bicep motion, forearm/elbow motion, wrist pitch, and claw operation.

The firmware supports:

- Tool-aware inverse kinematics
- Smooth servo motion
- Wrist collision safety limits
- Table-skim pickup positioning
- WebSocket-based remote control
- Live robot telemetry over HTTP
- Teachable keyframe timeline playback
- Manual and semi-autonomous pick-and-place workflows

## Hardware

- ESP8266 microcontroller
- PCA9685 16-channel PWM servo driver
- DFRobot 35KG servo for high-load arm motion
- DFRobot metal gear 9g servos for wrist/claw movement
- External DC bench power supply
- Raspberry Pi for future advanced web UI hosting

## Control Pipeline

Browser or Raspberry Pi Web UI  
→ ESP8266 WebSocket command server  
→ Inverse kinematics solver  
→ PCA9685 PWM servo driver  
→ Robot arm servos

## ESP8266 Endpoints

Status endpoint:

```text
http://ESP8266_IP/status
```

WebSocket command endpoint:

```text
ws://ESP8266_IP:81/
```

## Remote Timeline Commands

The Raspberry Pi-hosted UI can send authored keyframes to the firmware with:

- `SET_TARGET:<x>:<y>:<z>:<pitch>`
- `SET_TOOL:<0 or 1>`
- `SET_CLAW:<ticks>`
- `ADD_KEYFRAME:<type>:<x>:<y>:<z>:<pitch>:<toolMode>:<clawTicks>:<durationMs>:<waitAfterMs>`
- `PLAY_REMOTE_TIMELINE`
- `GET_CAPABILITIES`

## Vision Pipeline

This repo also includes a Raspberry Pi vision bridge and a PC webcam client. The vision path is deliberately safety-first:

```text
USB webcam on main PC
→ OpenCV green-object detector
→ Raspberry Pi vision API
→ ghost/dry-run pickup planner
→ user-confirmed ESP remote timeline
→ ESP8266 firmware
```

Detection never moves the arm by itself. The flow is always:

```text
detect → display/API state → user explicitly requests preview or pickup
```

The React web UI is not modified here. It can later call these endpoints:

- `POST /vision/target`
- `GET /vision/target`
- `POST /vision/clear`
- `POST /vision/pick/preview`
- `POST /vision/pick`
- `GET /vision/calibration`
- `PUT /vision/calibration`
- `POST /vision/calibration/reset`

### Raspberry Pi Setup

Install and run the Pi vision server:

```bash
cd /home/notpandamack/Projects/PIMA_CCG17-robot-arm-firmware
python3 -m venv .venv
source .venv/bin/activate
pip install -r pi_vision_server/requirements.txt
cp config/vision_server_config.example.json config/vision_server_config.json
```

Edit `config/vision_server_config.json` and set:

```json
{
  "espBaseUrl": "http://ESP8266_IP",
  "motionEnabled": false
}
```

Start the server:

```bash
uvicorn pi_vision_server.app:app --host 0.0.0.0 --port 8000
```

Keep `motionEnabled` set to `false` until calibration is complete and a hover-only test has passed. With `motionEnabled: false`, `/vision/pick` is ghost mode: it prints/logs the exact ESP command sequence but does not send motion commands to the ESP.

### Main PC Vision Client Setup

Install the OpenCV client on the main PC with the webcam:

```bash
cd /home/notpandamack/Projects/PIMA_CCG17-robot-arm-firmware/vision_client
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Run green-object detection:

```bash
python vision_detect_color.py --pi-url http://RASPBERRY_PI_IP:8000
```

Keyboard controls:

- `q`: quit
- `c`: show HSV and calibration help

Before calibration, the client sends only pixel coordinates. After calibration, it sends `robotX` and `robotY` too.

### Beginner Calibration Instructions

Run guided calibration from the PC:

```bash
cd /home/notpandamack/Projects/PIMA_CCG17-robot-arm-firmware/vision_client
source .venv/bin/activate
python vision_detect_color.py --calibrate --pi-url http://RASPBERRY_PI_IP:8000
```

Follow these steps slowly:

1. Mount the webcam so the whole reachable table area is visible.
2. Make sure the robot base and all four table marker locations are visible.
3. Click the robot/table origin. Use the robot base center projected onto the table.
4. Place a visible marker at `front-left`, enter or accept its robot X/Y coordinate, then click the marker in the camera image.
5. Repeat for `front-right`, `back-left`, and `back-right`.
6. The script computes the camera-to-robot homography and reprojection error.
7. For table Z, manually jog the arm until the claw/tip barely touches the table at each requested point. The script only reads the ESP status after you press Enter. It never lowers the arm automatically.
8. Manually jog the arm to the desired table-skim pickup pitch and press Enter so the script can capture `pickupPitchDeg`.
9. Click a validation point in the image and review the converted robot X/Y coordinate.
10. Run the hover-only preview when prompted. This is still ghost mode unless you have explicitly enabled motion.

When calibration is saved successfully, the script prints:

```text
Calibrated
```

The live calibration is stored at:

```text
config/vision_calibration.json
```

That file is ignored by git because it contains measurements from your physical setup.

### Ghost Mode And Hover-Only Preview

To test without moving the robot, leave this in `config/vision_server_config.json`:

```json
{
  "motionEnabled": false
}
```

Post a sample target:

```bash
curl -X POST http://RASPBERRY_PI_IP:8000/vision/target \
  -H 'Content-Type: application/json' \
  -d '{
    "type": "object_detected",
    "object": "green_object",
    "pixelX": 420,
    "pixelY": 310,
    "robotX": 35.0,
    "robotY": 210.0,
    "confidence": 0.92
  }'
```

Preview the exact generated pickup sequence:

```bash
curl -X POST 'http://RASPBERRY_PI_IP:8000/vision/pick/preview'
```

Preview hover-only mode:

```bash
curl -X POST 'http://RASPBERRY_PI_IP:8000/vision/pick/preview?hoverOnly=true'
```

Hover-only removes the lower-to-grab, claw-close, wait, and lift-after-grab steps. It only plans a safe hover over the target.

Calling `/vision/pick` while `motionEnabled` is false still does not move the robot:

```bash
curl -X POST 'http://RASPBERRY_PI_IP:8000/vision/pick'
```

The response includes:

- ordered ESP commands
- `robotX`
- `robotY`
- `hoverZ`
- `skimGrabZ`
- `pickupPitchDeg`
- workspace validation result
- calibration validation result
- whether motion was actually sent

### Safe Robot Test

Only do this after the no-motion tests look correct.

1. Keep your hand near power cutoff and verify ESTOP behavior first.
2. Start with base `90`, bicep `90`, forearm `90`, wrist `40`, gripper `0`.
3. Run calibration and confirm the status is calibrated.
4. Run `/vision/pick/preview` and inspect the generated commands.
5. Run `/vision/pick/preview?hoverOnly=true`.
6. Set `"motionEnabled": true` in `config/vision_server_config.json`.
7. Run `/vision/pick?hoverOnly=true` first and verify the arm only moves to safe hover.
8. Only after hover is correct, run `/vision/pick` for the full pickup sequence.

The Pi service refuses pickup if:

- calibration is missing or invalid
- `pickupPitchDeg` has not been captured
- the target is stale
- confidence is below `0.75`
- `robotX` or `robotY` is missing
- the target is outside configured workspace bounds
- ESP status cannot be checked
- ESP ESTOP is active
- `motionEnabled` is false

### Vision Development Tests

Run the no-hardware Python tests:

```bash
cd /home/notpandamack/Projects/PIMA_CCG17-robot-arm-firmware
python3 -m unittest discover -v
```
