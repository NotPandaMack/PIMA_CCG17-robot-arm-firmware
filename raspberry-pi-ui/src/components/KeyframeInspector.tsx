import { Copy, PanelRight, Trash2, Undo2, Redo2 } from 'lucide-react'
import type { KeyframeType, TimelineKeyframe, ToolMode } from '../lib/timelineTypes'

type Props = {
  keyframe: TimelineKeyframe | null
  onUpdate: (id: string, patch: Partial<TimelineKeyframe>) => void
  onDelete: (id: string) => void
  onDuplicate: (id: string) => void
  onMove: (id: string, direction: -1 | 1) => void
}

function NumberField({
  label,
  value,
  onChange,
  step = 1,
}: {
  label: string
  value: number
  onChange: (value: number) => void
  step?: number
}) {
  return (
    <label className="field-label">
      {label}
      <input className="text-input" type="number" step={step} value={value} onChange={(event) => onChange(Number(event.target.value))} />
    </label>
  )
}

export function KeyframeInspector({ keyframe, onUpdate, onDelete, onDuplicate, onMove }: Props) {
  if (!keyframe) {
    return (
      <aside className="panel min-h-[420px]">
        <div className="panel-title">
          <PanelRight size={18} />
          Keyframe Inspector
        </div>
        <div className="empty-state">Select a timeline block to edit pose, timing, claw, and notes.</div>
      </aside>
    )
  }

  return (
    <aside className="panel">
      <div className="panel-title">
        <PanelRight size={18} />
        Keyframe Inspector
      </div>
      <div className="grid gap-3">
        <label className="field-label">
          Label
          <input className="text-input" value={keyframe.label} onChange={(event) => onUpdate(keyframe.id, { label: event.target.value })} />
        </label>
        <div className="grid grid-cols-2 gap-2">
          <label className="field-label">
            Type
            <select className="text-input" value={keyframe.type} onChange={(event) => onUpdate(keyframe.id, { type: event.target.value as KeyframeType })}>
              <option>MOVE</option>
              <option>GRAB</option>
              <option>DROP</option>
              <option>WAIT</option>
            </select>
          </label>
          <label className="field-label">
            Tool
            <select className="text-input" value={keyframe.toolMode} onChange={(event) => onUpdate(keyframe.id, { toolMode: event.target.value as ToolMode })}>
              <option value="TIP">TIP</option>
              <option value="GAP">GAP</option>
            </select>
          </label>
        </div>
        <div className="grid grid-cols-2 gap-2">
          <NumberField label="X" value={keyframe.x} step={0.1} onChange={(x) => onUpdate(keyframe.id, { x })} />
          <NumberField label="Y" value={keyframe.y} step={0.1} onChange={(y) => onUpdate(keyframe.id, { y })} />
          <NumberField label="Z" value={keyframe.z} step={0.1} onChange={(z) => onUpdate(keyframe.id, { z })} />
          <NumberField label="Pitch" value={keyframe.pitch} step={0.1} onChange={(pitch) => onUpdate(keyframe.id, { pitch })} />
          <NumberField label="Duration ms" value={keyframe.durationMs} step={50} onChange={(durationMs) => onUpdate(keyframe.id, { durationMs })} />
          <NumberField label="Wait after ms" value={keyframe.waitAfterMs} step={50} onChange={(waitAfterMs) => onUpdate(keyframe.id, { waitAfterMs })} />
          <NumberField label="Claw ticks" value={keyframe.clawTicks} step={1} onChange={(clawTicks) => onUpdate(keyframe.id, { clawTicks })} />
        </div>
        <label className="field-label">
          Notes
          <textarea className="text-input min-h-24" value={keyframe.notes ?? ''} onChange={(event) => onUpdate(keyframe.id, { notes: event.target.value })} />
        </label>
        <div className="grid grid-cols-2 gap-2">
          <button className="control-button" onClick={() => onMove(keyframe.id, -1)}>
            <Undo2 size={16} />
            Move Left
          </button>
          <button className="control-button" onClick={() => onMove(keyframe.id, 1)}>
            <Redo2 size={16} />
            Move Right
          </button>
          <button className="control-button" onClick={() => onDuplicate(keyframe.id)}>
            <Copy size={16} />
            Duplicate
          </button>
          <button className="danger-soft-button" onClick={() => onDelete(keyframe.id)}>
            <Trash2 size={16} />
            Delete
          </button>
        </div>
      </div>
    </aside>
  )
}
