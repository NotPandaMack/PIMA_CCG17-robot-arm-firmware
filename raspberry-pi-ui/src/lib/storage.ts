import type { RobotProgram, TimelineKeyframe } from './timelineTypes'

const IP_KEY = 'robot-arm-ui:esp-ip'
const PROGRAMS_KEY = 'robot-arm-ui:programs'

export function loadEspIp() {
  return localStorage.getItem(IP_KEY) ?? ''
}

export function saveEspIp(ip: string) {
  localStorage.setItem(IP_KEY, ip.trim())
}

export function loadPrograms(): RobotProgram[] {
  try {
    return JSON.parse(localStorage.getItem(PROGRAMS_KEY) ?? '[]') as RobotProgram[]
  } catch {
    return []
  }
}

export function saveProgram(name: string, notes: string, keyframes: TimelineKeyframe[]) {
  const programs = loadPrograms()
  const existing = programs.find((program) => program.name === name)
  const next: RobotProgram = {
    id: existing?.id ?? crypto.randomUUID(),
    name,
    notes,
    keyframes,
    updatedAt: new Date().toISOString(),
  }
  const filtered = programs.filter((program) => program.id !== next.id)
  localStorage.setItem(PROGRAMS_KEY, JSON.stringify([next, ...filtered]))
  return next
}

export function deleteProgram(id: string) {
  localStorage.setItem(PROGRAMS_KEY, JSON.stringify(loadPrograms().filter((program) => program.id !== id)))
}

export function downloadJson(filename: string, data: unknown) {
  const blob = new Blob([JSON.stringify(data, null, 2)], { type: 'application/json' })
  const url = URL.createObjectURL(blob)
  const anchor = document.createElement('a')
  anchor.href = url
  anchor.download = filename
  anchor.click()
  URL.revokeObjectURL(url)
}
