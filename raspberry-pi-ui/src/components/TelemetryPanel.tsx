import { Activity, Gauge, Radar, Ruler } from 'lucide-react'
import type { RobotStatus } from '../lib/timelineTypes'

type Props = {
  status: RobotStatus
}

const numberValue = (value?: number, digits = 1) => (typeof value === 'number' ? value.toFixed(digits) : '--')

function Metric({ label, value, unit }: { label: string; value: string | number | undefined; unit?: string }) {
  return (
    <div className="readout">
      <span>{label}</span>
      <strong>
        {value ?? '--'}
        {unit ? <small>{unit}</small> : null}
      </strong>
    </div>
  )
}

function MiniPlot({ x = 0, y = 170 }: { x?: number; y?: number }) {
  const px = Math.max(8, Math.min(92, 50 + x / 4))
  const py = Math.max(8, Math.min(92, 92 - y / 3))
  return (
    <div className="xy-plot">
      <div className="plot-grid" />
      <div className="plot-dot" style={{ left: `${px}%`, top: `${py}%` }} />
      <span>X/Y</span>
    </div>
  )
}

function VerticalGauge({ label, value = 0, min = 0, max = 180 }: { label: string; value?: number; min?: number; max?: number }) {
  const percent = Math.max(0, Math.min(100, ((value - min) / (max - min)) * 100))
  return (
    <div className="gauge">
      <div className="gauge-track">
        <div style={{ height: `${percent}%` }} />
      </div>
      <span>{label}</span>
      <strong>{numberValue(value, 0)}</strong>
    </div>
  )
}

export function TelemetryPanel({ status }: Props) {
  return (
    <section className="panel">
      <div className="panel-title">
        <Activity size={18} />
        Live Telemetry
      </div>
      <div className="grid gap-4 xl:grid-cols-[1fr_240px]">
        <div className="grid grid-cols-2 gap-2 md:grid-cols-4">
          <Metric label="Status" value={status.status} />
          <Metric label="Tool" value={status.toolMode} />
          <Metric label="Speed" value={status.speed} />
          <Metric label="Keyframes" value={status.keyframeCount} />
          <Metric label="X" value={numberValue(status.x)} unit="mm" />
          <Metric label="Y" value={numberValue(status.y)} unit="mm" />
          <Metric label="Z" value={numberValue(status.z)} unit="mm" />
          <Metric label="Pitch" value={numberValue(status.pitch)} unit="deg" />
          <Metric label="Wrist X" value={numberValue(status.wristX)} unit="mm" />
          <Metric label="Wrist Y" value={numberValue(status.wristY)} unit="mm" />
          <Metric label="Wrist Z" value={numberValue(status.wristZ)} unit="mm" />
          <Metric label="Claw" value={status.clawTicks} unit="ticks" />
          <Metric label="Base" value={numberValue(status.base)} unit="deg" />
          <Metric label="Shoulder" value={numberValue(status.shoulder)} unit="deg" />
          <Metric label="Elbow" value={numberValue(status.elbow)} unit="deg" />
          <Metric label="Wrist" value={numberValue(status.wrist)} unit="deg" />
        </div>
        <div className="grid grid-cols-[1fr_72px_72px] gap-3">
          <MiniPlot x={status.x} y={status.y} />
          <VerticalGauge label="Z" value={status.z} min={0} max={180} />
          <VerticalGauge label="Claw" value={status.clawTicks} min={250} max={470} />
        </div>
      </div>
      <div className="mt-3 flex flex-wrap gap-2 text-xs text-slate-400">
        <span className="inline-flex items-center gap-1">
          <Radar size={14} /> Timeline {status.timelinePlaying ? 'playing' : 'idle'}
        </span>
        <span className="inline-flex items-center gap-1">
          <Gauge size={14} /> Wrist target {numberValue(status.wristX)}/{numberValue(status.wristY)}/{numberValue(status.wristZ)}
        </span>
        <span className="inline-flex items-center gap-1">
          <Ruler size={14} /> {status.timelineText || 'No ESP timeline text'}
        </span>
      </div>
    </section>
  )
}
