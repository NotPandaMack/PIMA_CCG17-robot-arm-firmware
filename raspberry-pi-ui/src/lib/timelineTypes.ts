export type ToolMode = 'TIP' | 'GAP'
export type KeyframeType = 'MOVE' | 'GRAB' | 'DROP' | 'WAIT'

export const ESP_MAX_KEYFRAMES = 12
export const ESP_COMMAND_TIMEOUT_MS = 450

export type EspTimelineItem = {
  index: number
  type: KeyframeType
  x: number
  y: number
  z: number
  pitch: number
  toolMode: 0 | 1
  clawTicks: number
}

export type RobotStatus = {
  status?: string
  toolMode?: string
  x?: number
  y?: number
  z?: number
  pitch?: number
  wristX?: number
  wristY?: number
  wristZ?: number
  base?: number
  shoulder?: number
  elbow?: number
  wrist?: number
  clawTicks?: number
  speed?: number
  keyframeCount?: number
  timelinePlaying?: boolean
  timelineText?: string
  timeline?: EspTimelineItem[]
  estop?: boolean
}

export type TimelineKeyframe = {
  id: string
  type: KeyframeType
  x: number
  y: number
  z: number
  pitch: number
  toolMode: ToolMode
  clawTicks: number
  durationMs: number
  waitAfterMs: number
  label: string
  notes?: string
}

export type RobotProgram = {
  id: string
  name: string
  notes: string
  keyframes: TimelineKeyframe[]
  updatedAt: string
}

export const defaultStatus: RobotStatus = {
  status: 'No status yet',
  toolMode: 'Unknown',
  x: 0,
  y: 170,
  z: 80,
  pitch: 0,
  wristX: 0,
  wristY: 100,
  wristZ: 80,
  base: 90,
  shoulder: 80,
  elbow: 60,
  wrist: 90,
  clawTicks: 335,
  speed: 1,
  keyframeCount: 0,
  timelinePlaying: false,
  timelineText: '',
  estop: false,
}

export function makeKeyframe(type: KeyframeType, status: RobotStatus, index: number): TimelineKeyframe {
  const toolMode = status.toolMode?.toLowerCase().includes('gap') ? 'GAP' : 'TIP'
  return {
    id: crypto.randomUUID(),
    type,
    x: status.x ?? 0,
    y: status.y ?? 170,
    z: status.z ?? 80,
    pitch: status.pitch ?? 0,
    toolMode,
    clawTicks: type === 'WAIT' ? -1 : status.clawTicks ?? 335,
    durationMs: type === 'WAIT' ? 1000 : 1200,
    waitAfterMs: type === 'GRAB' || type === 'DROP' ? 350 : 0,
    label: `${type} ${index + 1}`,
    notes: '',
  }
}
