# Robot Arm Control Center

Desktop setup software for the robot arm vision, calibration, and pickup workflow.

The app is designed for the main PC with two cameras attached:

- Raspberry Pi vision server
- ESP8266 robot controller
- Overhead 2D webcam for ArUco X/Y mapping and object detection
- Angled Intel RealSense D415 depth camera for table/claw height

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
4. Select the overhead 2D webcam index, usually `0`, then use Test 2D Webcam.
5. Plug the D415 into USB 3 and use Test RealSense D415.
6. Use Test Pi Connection and Test ESP Connection.
7. Save Settings.

Use Mock Pi and Fake ESP modes to test the workflow without hardware.

The app starts in a startup-safe mode: it does not automatically probe the Pi, ESP, cameras, config, or calibration during window creation. Status begins as Not tested, and hardware checks only run after you press a test button.

The DesktopGUI uses the overhead 2D webcam for object detection and top-down ArUco marker mapping. The angled D415 uses its color/depth stream only for table-plane and claw-height calibration.

## Beginner Setup Checklist

The checklist controls which actions are available:

- Pi server connected
- ESP connected
- 2D webcam connected
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

1. Start Auto Calibration. This starts the 2D webcam and the D415 depth stream.
2. Open Calibration.
3. Camera Placement: confirm the full reachable table is visible.
4. Define Robot Origin: click the robot base center projected onto the table.
5. Four-Point Table Mapping: generate and print the ArUco marker sheet, place IDs 0-3 at FL/FR/BL/BR, then scan markers from the camera to auto-fill pixel centers and robot X/Y coordinates. If ArUco is unavailable, use the QR fallback sheet. Manual click placement is still available behind the manual fallback checkbox.
6. Workspace Bounds: save the safety limits.
7. D415 Depth Calibration: aim the D415 down at an angle so it sees the desk and claw. Learn the table plane while the claw is not blocking the desk. Then use the website controls to move the claw to high, medium, and low safe heights. Click the claw tip in the D415 Color tab and capture each sample. The GUI never lowers the arm automatically.
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
- RealSense support requires `pyrealsense2` and the D415 to be visible to the OS over USB 3.
- The app stores GUI preferences in `vision_gui/user_settings.json`, which is local to this machine.
- ArUco marker generation and detection require `opencv-contrib-python`. Plain `opencv-python` does not always include `cv2.aruco`.
