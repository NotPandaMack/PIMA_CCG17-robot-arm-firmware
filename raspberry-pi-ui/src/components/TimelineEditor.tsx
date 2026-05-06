import { Circle, Clock, CopyPlus, Hand, Play, Plus, Send, Square, Trash2, Video } from 'lucide-react'
import type { ConnectionState } from '../lib/espClient'
import { ESP_COMMANDS, addRemoteKeyframeCommand } from '../lib/commandBuilder'
import { ESP_MAX_KEYFRAMES, type KeyframeType, type RobotStatus, type TimelineKeyframe } from '../lib/timelineTypes'
import { KeyframeInspector } from './KeyframeInspector'
import { ProgramManager } from './ProgramManager'

type Props = {
  status: RobotStatus
  connectionState: ConnectionState
  keyframes: TimelineKeyframe[]
  selectedId: string | null
  playheadIndex: number
  setSelectedId: (id: string | null) => void
  addKeyframe: (type: KeyframeType) => void
  setKeyframes: (keyframes: TimelineKeyframe[]) => void
  updateKeyframe: (id: string, patch: Partial<TimelineKeyframe>) => void
  deleteKeyframe: (id: string) => void
  duplicateKeyframe: (id: string) => void
  moveKeyframe: (id: string, direction: -1 | 1) => void
  sendCommand: (command: string) => void
  toast: (message: string) => void
  playFrontendTimeline: () => void
}

const iconForType = {
  MOVE: <Circle size={16} />,
  GRAB: <Hand size={16} />,
  DROP: <CopyPlus size={16} />,
  WAIT: <Clock size={16} />,
}

export function TimelineEditor({
  status,
  connectionState,
  keyframes,
  selectedId,
  playheadIndex,
  setSelectedId,
  addKeyframe,
  setKeyframes,
  updateKeyframe,
  deleteKeyframe,
  duplicateKeyframe,
  moveKeyframe,
  sendCommand,
  toast,
  playFrontendTimeline,
}: Props) {
  const selected = keyframes.find((keyframe) => keyframe.id === selectedId) ?? null

  function clearTimeline() {
    if (confirm('Clear the local frontend timeline?')) {
      setKeyframes([])
      setSelectedId(null)
    }
  }

  function sendTimelineToEsp() {
    if (connectionState !== 'connected' && !confirm('Robot is not connected. Send commands anyway?')) return
    if (keyframes.length > ESP_MAX_KEYFRAMES) {
      toast(`ESP firmware supports ${ESP_MAX_KEYFRAMES} keyframes; trim the local timeline first`)
      return
    }
    const ok = confirm('Send this authored local timeline to the ESP8266 and replace the ESP timeline?')
    if (!ok) return
    sendCommand(ESP_COMMANDS.clearTimeline)
    keyframes.forEach((keyframe) => sendCommand(addRemoteKeyframeCommand(keyframe)))
  }

  function exactPlaybackCheck() {
    toast('Playing local timeline with direct ESP target/claw/tool commands')
    playFrontendTimeline()
  }

  return (
    <section className="timeline-studio">
      <div className="editor-main">
        <div className="panel-title">
          <Video size={18} />
          Timeline Studio
        </div>
        <div className="editor-toolbar">
          {(['MOVE', 'GRAB', 'DROP', 'WAIT'] as KeyframeType[]).map((type) => (
            <button className="control-button" key={type} onClick={() => addKeyframe(type)}>
              <Plus size={16} />
              {type}
            </button>
          ))}
          <button className="primary-button" onClick={sendTimelineToEsp}>
            <Send size={16} />
            Send to ESP
          </button>
          <button className="primary-button" disabled={status.estop} onClick={exactPlaybackCheck}>
            <Play size={16} />
            Preview
          </button>
          <button className="control-button" onClick={() => sendCommand(ESP_COMMANDS.playTimeline)} disabled={status.estop}>
            <Play size={16} />
            ESP Play
          </button>
          <button className="control-button" onClick={() => sendCommand(ESP_COMMANDS.stop)}>
            <Square size={16} />
            Stop
          </button>
          <button className="control-button" onClick={() => sendCommand(ESP_COMMANDS.deleteLast)}>
            <Trash2 size={16} />
            Last
          </button>
          <button className="danger-soft-button" onClick={clearTimeline}>
            <Trash2 size={16} />
            Clear
          </button>
        </div>

        <div className="editor-meter-row">
          <div className="readout">
            <span>Local timeline</span>
            <strong>
              {keyframes.length}/{ESP_MAX_KEYFRAMES}
            </strong>
          </div>
          <div className="readout">
            <span>ESP timeline</span>
            <strong>
              {status.keyframeCount ?? 0}/{ESP_MAX_KEYFRAMES}
            </strong>
          </div>
          <div className={status.timelinePlaying ? 'readout border-cyan-400/40 text-cyan-100' : 'readout'}>
            <span>ESP playback</span>
            <strong>{status.timelinePlaying ? 'playing' : 'idle'}</strong>
          </div>
        </div>

        <div className="timeline-shell">
          <div className="timeline-ruler">
            {keyframes.map((keyframe, index) => (
              <span key={keyframe.id}>{index + 1}</span>
            ))}
          </div>
          <div className="timeline-track">
            <div className="playhead" style={{ left: `${keyframes.length ? ((playheadIndex + 0.5) / keyframes.length) * 100 : 0}%` }} />
            {keyframes.length === 0 ? <div className="empty-state w-full">Add keyframes from live telemetry to build a local program.</div> : null}
            {keyframes.map((keyframe) => (
              <button
                className={`keyframe-card type-${keyframe.type.toLowerCase()} ${selectedId === keyframe.id ? 'selected' : ''}`}
                key={keyframe.id}
                onClick={() => setSelectedId(keyframe.id)}
              >
                <div>
                  {iconForType[keyframe.type]}
                  <strong>{keyframe.label}</strong>
                </div>
                <span>
                  {keyframe.x.toFixed(1)}, {keyframe.y.toFixed(1)}, {keyframe.z.toFixed(1)}
                </span>
                <small>{keyframe.durationMs} ms</small>
              </button>
            ))}
          </div>
        </div>

        <div className="mt-4">
          <div className="mb-2 text-xs font-black uppercase tracking-[0.16em] text-slate-500">ESP Taught Timeline</div>
          <div className="esp-timeline-grid">
            {status.timeline?.length ? (
              status.timeline.map((item) => (
                <div className={`esp-kf type-${item.type.toLowerCase()}`} key={item.index}>
                  <strong>
                    {item.index + 1}:{item.type}
                  </strong>
                  <span>
                    {item.x.toFixed(1)}, {item.y.toFixed(1)}, {item.z.toFixed(1)} / {item.pitch.toFixed(1)}
                  </span>
                </div>
              ))
            ) : (
              <div className="empty-state">No ESP keyframes reported by `/status`.</div>
            )}
          </div>
        </div>
      </div>

      <div className="editor-side">
        <KeyframeInspector keyframe={selected} onUpdate={updateKeyframe} onDelete={deleteKeyframe} onDuplicate={duplicateKeyframe} onMove={moveKeyframe} />
        <ProgramManager keyframes={keyframes} onLoad={setKeyframes} toast={toast} />
      </div>
    </section>
  )
}
