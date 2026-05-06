# Desk Robot Arm UI

Raspberry Pi-hosted web UI for the PIMA_CCG17 ESP8266 robot arm firmware.

## Architecture

Browser -> Raspberry Pi Vite app -> ESP8266 HTTP/WebSocket -> PCA9685 -> servos

The Raspberry Pi does not control servos directly. The ESP8266 remains the robot controller for servo timing, IK, wrist safety, claw actions, ESTOP, and firmware-resident timeline playback.

## Firmware Interface Reviewed

Firmware repository: https://github.com/NotPandaMack/PIMA_CCG17-robot-arm-firmware

Confirmed endpoints and behavior:

- HTTP status: `GET http://ESP8266_IP/status`
- WebSocket commands: `ws://ESP8266_IP:81/`
- Status JSON includes pose, wrist target, joint angles, claw ticks, speed, ESTOP, `timelineText`, and `timeline`.
- Firmware timeline capacity is `MAX_KEYFRAMES = 12`.
- Hold commands should repeat faster than firmware `COMMAND_TIMEOUT_MS = 450`; this UI repeats every 130 ms.
- This repo branch exposes direct target, tool, claw, compact keyframe upload, remote timeline play, and capability commands over WebSocket.

## Run

```bash
cd raspberry-pi-ui
npm install
npm run dev
```

The dev server is configured for LAN access:

```text
http://RASPBERRY_PI_IP:5173/
```

Production build:

```bash
npm run build
npm run preview
```

## ESP8266 IP

Open the UI, use the Connection panel, enter the ESP8266 IP address, then click Save and Connect. The IP is stored in browser `localStorage`.

## Timeline Commands

The Timeline Studio tab uses these firmware commands:

- `SET_TARGET:<x>:<y>:<z>:<pitch>`
- `SET_TOOL:<0 or 1>`
- `SET_CLAW:<ticks>`
- `ADD_KEYFRAME:<type>:<x>:<y>:<z>:<pitch>:<toolMode>:<clawTicks>:<durationMs>:<waitAfterMs>`
- `PLAY_REMOTE_TIMELINE`
- `GET_CAPABILITIES`

The frontend command builders are in `src/lib/commandBuilder.ts`.
