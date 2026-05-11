from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable

from .esp_client import EspClient
from .jog import MIN_MANUAL_CLEARANCE_MM, PITCH_STEP_DEG, JOG_STEP_MM, JogPlan, Pose, build_jog_plan, pose_from_status, pose_to_set_target


WEB_COMMANDS = {
    "home": "IK_HOME",
    "stop": "STOP",
    "estop": "ESTOP",
    "clearEstop": "CLEAR_ESTOP",
    "openClaw": "CLAW_OPEN",
    "closeClawSoft": "CLAW_CLOSE_SOFT",
    "closeClawFirm": "CLAW_CLOSE_FIRM",
    "clearTimeline": "CLEAR_TIMELINE",
    "playRemoteTimeline": "PLAY_REMOTE_TIMELINE",
}


@dataclass(frozen=True)
class MotionResult:
    command: str
    sent: bool
    status: dict[str, Any] | None = None
    pose: Pose | None = None


class RobotMotionAdapter:
    def __init__(
        self,
        esp_client: EspClient,
        *,
        dry_run: bool = True,
        logger: Callable[[str], None] | None = None,
    ) -> None:
        self.esp_client = esp_client
        self.dry_run = dry_run
        self.logger = logger or (lambda _message: None)

    def get_current_pose(self) -> Pose:
        return pose_from_status(self.esp_client.status())

    def set_target(self, x: float, y: float, z: float, pitch: float) -> MotionResult:
        pose = Pose(float(x), float(y), float(z), float(pitch))
        return self._send(pose_to_set_target(pose), pose=pose)

    def jog_axis(
        self,
        axis: str,
        delta: float,
        *,
        table_z: float | None = None,
        minimum_clearance: float = MIN_MANUAL_CLEARANCE_MM,
        calibration_touch_mode: bool = False,
    ) -> MotionResult:
        status = self.esp_client.status()
        direction = 1 if delta > 0 else -1
        axis_upper = axis.upper()
        if axis_upper == "Z" and delta < 0 and abs(float(delta)) > JOG_STEP_MM:
            raise ValueError("Z lowering exceeds the default small jog step")
        step_mm = abs(float(delta)) if axis_upper != "PITCH" else JOG_STEP_MM
        pitch_step = abs(float(delta)) if axis_upper == "PITCH" else PITCH_STEP_DEG
        plan = build_jog_plan(
            status,
            axis_upper,
            direction,
            step_mm=step_mm,
            pitch_step_deg=pitch_step,
            table_z=table_z,
            minimum_clearance=minimum_clearance,
            calibration_touch_mode=calibration_touch_mode,
        )
        return self._send(plan.command, pose=plan.pose)

    def stop(self) -> MotionResult:
        return self._send(WEB_COMMANDS["stop"])

    def estop(self) -> MotionResult:
        return self._send(WEB_COMMANDS["estop"])

    def clear_estop(self) -> MotionResult:
        return self._send(WEB_COMMANDS["clearEstop"])

    def home(self) -> MotionResult:
        return self._send(WEB_COMMANDS["home"])

    def open_claw(self) -> MotionResult:
        return self._send(WEB_COMMANDS["openClaw"])

    def close_claw_soft(self) -> MotionResult:
        return self._send(WEB_COMMANDS["closeClawSoft"])

    def close_claw_firm(self) -> MotionResult:
        return self._send(WEB_COMMANDS["closeClawFirm"])

    def clear_timeline(self) -> MotionResult:
        return self._send(WEB_COMMANDS["clearTimeline"])

    def add_keyframe(
        self,
        keyframe_type: str,
        x: float,
        y: float,
        z: float,
        pitch: float,
        tool_mode: int,
        claw_ticks: int,
        duration_ms: int,
        wait_after_ms: int,
    ) -> MotionResult:
        command = ":".join(
            [
                "ADD_KEYFRAME",
                keyframe_type,
                _fmt(x),
                _fmt(y),
                _fmt(z),
                _fmt(pitch),
                str(int(tool_mode)),
                str(int(claw_ticks)),
                str(int(duration_ms)),
                str(int(wait_after_ms)),
            ]
        )
        return self._send(command)

    def play_remote_timeline(self) -> MotionResult:
        return self._send(WEB_COMMANDS["playRemoteTimeline"])

    def send_web_command(self, command: str) -> MotionResult:
        if command == WEB_COMMANDS["stop"]:
            return self.stop()
        if command == WEB_COMMANDS["estop"]:
            return self.estop()
        if command == WEB_COMMANDS["clearEstop"]:
            return self.clear_estop()
        if command == WEB_COMMANDS["home"]:
            return self.home()
        if command == WEB_COMMANDS["openClaw"]:
            return self.open_claw()
        if command == WEB_COMMANDS["closeClawSoft"]:
            return self.close_claw_soft()
        if command == WEB_COMMANDS["closeClawFirm"]:
            return self.close_claw_firm()
        if command == WEB_COMMANDS["clearTimeline"]:
            return self.clear_timeline()
        if command == WEB_COMMANDS["playRemoteTimeline"]:
            return self.play_remote_timeline()
        if command.startswith("SET_TARGET:") or command.startswith("ADD_KEYFRAME:"):
            return self._send(command)
        raise ValueError(f"unsupported direct movement command: {command}")

    def _send(self, command: str, pose: Pose | None = None) -> MotionResult:
        self.logger(f"PyGUI sending same as web UI: {command}")
        if self.dry_run:
            self.logger(f"Movement adapter dry-run: not sent: {command}")
            return MotionResult(command=command, sent=False, status=None, pose=pose)
        self.esp_client.send_command(command)
        status = self.esp_client.status()
        return MotionResult(command=command, sent=True, status=status, pose=pose)


def _fmt(value: float) -> str:
    return f"{float(value):.1f}"
