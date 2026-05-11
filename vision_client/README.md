# PC Vision Client

This client runs on the main PC with the USB webcam connected. It detects a bright green object and sends target data to the Raspberry Pi vision server.

## Install

```bash
cd vision_client
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Run Detection

```bash
python vision_detect_color.py --pi-url http://RASPBERRY_PI_IP:8000
```

Keyboard controls:

- `q`: quit
- `c`: print HSV and calibration help

If calibration has not been saved yet, the client sends only `pixelX` and `pixelY`. After calibration, it also sends `robotX` and `robotY`.

## Run Guided Calibration

```bash
python vision_detect_color.py --calibrate --pi-url http://RASPBERRY_PI_IP:8000
```

The calibration wizard asks you to click the table origin and four table markers, then manually jog the robot for table touch points. The script never lowers the arm automatically.

