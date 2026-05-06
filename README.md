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

## Raspberry Pi UI

The Raspberry Pi-hosted Vite/React timeline editor lives in:

```text
raspberry-pi-ui/
```

Run it on the Pi:

```bash
cd raspberry-pi-ui
npm install
npm run dev
```

Open the LAN URL printed by Vite, then set the ESP8266 IP in the Connection panel.

The Timeline Studio tab can now send authored keyframes to firmware with:

- `SET_TARGET:<x>:<y>:<z>:<pitch>`
- `SET_TOOL:<0 or 1>`
- `SET_CLAW:<ticks>`
- `ADD_KEYFRAME:<type>:<x>:<y>:<z>:<pitch>:<toolMode>:<clawTicks>:<durationMs>:<waitAfterMs>`
- `PLAY_REMOTE_TIMELINE`
- `GET_CAPABILITIES`
