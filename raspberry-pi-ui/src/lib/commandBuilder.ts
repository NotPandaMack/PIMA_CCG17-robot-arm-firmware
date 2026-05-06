import type { TimelineKeyframe } from './timelineTypes'

export const ESP_COMMANDS = {
  estop: 'ESTOP',
  clearEstop: 'CLEAR_ESTOP',
  stop: 'STOP',
  allOff: 'ALL_OFF',
  home: 'IK_HOME',
  tableSkim: 'TABLE_SKIM_POSE',
  carry: 'CARRY_POSE',
  toolTip: 'TOOL_TIP',
  toolGap: 'TOOL_GAP',
  clawOpen: 'CLAW_OPEN',
  clawSoft: 'CLAW_CLOSE_SOFT',
  clawFirm: 'CLAW_CLOSE_FIRM',
  clearTimeline: 'CLEAR_TIMELINE',
  playTimeline: 'PLAY_TIMELINE',
  playRemoteTimeline: 'PLAY_REMOTE_TIMELINE',
  getCapabilities: 'GET_CAPABILITIES',
  addMove: 'ADD_MOVE_KEYFRAME',
  addGrab: 'ADD_GRAB_KEYFRAME',
  addDrop: 'ADD_DROP_KEYFRAME',
  addWait: 'ADD_WAIT_KEYFRAME',
  deleteLast: 'DELETE_LAST_KEYFRAME',
} as const

export function speedCommand(speed: number) {
  return `SPEED:${Math.max(1, Math.min(6, Math.round(speed)))}`
}

export function holdCommand(servoIndex: number, direction: -1 | 1) {
  return `HOLD:${servoIndex}:${direction}`
}

export function ikMoveCommand(dx: number, dy: number, dz: number, dp: number) {
  return `IKMOVE:${dx}:${dy}:${dz}:${dp}`
}

export function teachCommandForKeyframe(keyframe: TimelineKeyframe) {
  if (keyframe.type === 'MOVE') return ESP_COMMANDS.addMove
  if (keyframe.type === 'GRAB') return ESP_COMMANDS.addGrab
  if (keyframe.type === 'DROP') return ESP_COMMANDS.addDrop
  return ESP_COMMANDS.addWait
}

export function setTargetCommand(keyframe: TimelineKeyframe) {
  return `SET_TARGET:${keyframe.x}:${keyframe.y}:${keyframe.z}:${keyframe.pitch}`
}

export function setToolCommand(keyframe: TimelineKeyframe) {
  return `SET_TOOL:${keyframe.toolMode === 'GAP' ? 1 : 0}`
}

export function setClawCommand(ticks: number) {
  return `SET_CLAW:${Math.round(ticks)}`
}

export function addRemoteKeyframeCommand(keyframe: TimelineKeyframe) {
  const toolMode = keyframe.toolMode === 'GAP' ? 1 : 0
  return [
    'ADD_KEYFRAME',
    keyframe.type,
    keyframe.x,
    keyframe.y,
    keyframe.z,
    keyframe.pitch,
    toolMode,
    Math.round(keyframe.clawTicks),
    Math.max(0, Math.round(keyframe.durationMs)),
    Math.max(0, Math.round(keyframe.waitAfterMs)),
  ].join(':')
}
