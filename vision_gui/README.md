# Robot Arm Control Center

Desktop setup software for the robot arm vision, calibration, and pickup workflow.

The app is designed for the main PC with the webcam attached. It connects to:

- Raspberry Pi vision server
- ESP8266 robot controller
- Local webcam

Detection alone never moves the robot. Real motion stays locked until calibration, ghost preview, and hover-only movement checks pass.

## Install

```bash
cd /home/notpandamack/Projects/PIMA_CCG17-robot-arm-firmware/vision_gui
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Run

```bash
cd /home/notpandamack/Projects/PIMA_CCG17-robot-arm-firmware/vision_gui
source .venv/bin/activate
python app.py
```

## First-Time Setup

1. Open the Setup page.
2. Enter the Raspberry Pi URL, for example `http://192.168.1.50:8000`.
3. Enter the ESP URL, for example `http://192.168.1.60`.
4. Select the webcam index, usually `0`.
5. Use Test Pi Connection, Test ESP Connection, and Test Webcam.
6. Save Settings.

Use Mock Pi and Fake ESP modes to test the workflow without hardware.

## Beginner Setup Checklist

The checklist controls which actions are available:

- Pi server connected
- ESP connected
- Webcam connected
- Object detection working
- Camera calibration complete
- Workspace bounds saved
- Table Z calibrated
- Pickup pose calibrated
- Ghost preview passed
- Hover-only movement passed
- Motion enabled
- Full pickup ready

Full pickup stays locked until hover-only real movement has passed and you confirm the hover looked safe.

## Calibration Walkthrough

1. Start the camera.
2. Open Calibration.
3. Camera Placement: confirm the full reachable table is visible.
4. Define Robot Origin: click the robot base center projected onto the table.
5. Four-Point Table Mapping: click front-left, front-right, back-left, and back-right markers and enter their robot X/Y millimeter coordinates.
6. Workspace Bounds: save the safety limits.
7. Table Z Calibration: manually jog the arm until the tip barely touches the table, then save touch points. The GUI never lowers the arm automatically.
8. Pickup Pose: manually move to a safe pickup pose and save the current ESP pose.
9. Validation: click a point and generate a hover preview.
10. Finish Calibration to save `config/vision_calibration.json` and upload it to the Pi.

## Ghost-Mode Test

1. Keep motion disabled.
2. Start the camera and detect the green object.
3. Send the detected target to the Pi.
4. Open Pickup Preview.
5. Generate Preview or Generate Hover-Only Preview.
6. Confirm the generated ESP command timeline looks correct.

## First Real Hover Test

1. Complete calibration.
2. Generate a successful ghost preview.
3. Confirm ESTOP is inactive.
4. Open Manual Test and click Enable Real Motion.
5. Click Run Hover-Only Movement.
6. Confirm the arm only moved to a safe hover.

## First Full Pickup

1. Confirm the checklist shows Hover-only movement passed.
2. Confirm target status is valid and inside workspace.
3. Confirm ESTOP is inactive and Motion Enabled is shown in the safety bar.
4. Use Pick Detected Object.

## Safety Notes

- Motion defaults to disabled.
- Auto-pick is disabled by default.
- Detection alone never moves the robot.
- Full pickup requires calibration, target validity, ESTOP clear, motion enabled, and a passed hover-only movement.
- Use ESTOP or power cutoff if the robot behaves unexpectedly.

## Known Limitations

- Auto-detect only tries common local Pi addresses; manual URL entry is still the reliable setup path.
- The GUI expects the current ESP status fields already exposed by `/status`.
- The app stores GUI preferences in `vision_gui/user_settings.json`, which is local to this machine.
