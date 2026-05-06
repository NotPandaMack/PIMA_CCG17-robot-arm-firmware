import { useEffect, useRef, useState } from 'react'
import { ConnectionPanel } from './components/ConnectionPanel'
import { ManualControls } from './components/ManualControls'
import { QuickActions } from './components/QuickActions'
import { SafetyBar } from './components/SafetyBar'
import { TelemetryPanel } from './components/TelemetryPanel'
import { TimelineEditor } from './components/TimelineEditor'
import { ESP_COMMANDS, setClawCommand, setTargetCommand, setToolCommand } from './lib/commandBuilder'
import { EspClient, type ConnectionState } from './lib/espClient'
import { loadEspIp, saveEspIp } from './lib/storage'
import {
  ESP_MAX_KEYFRAMES,
  defaultStatus,
  makeKeyframe,
  type KeyframeType,
  type RobotStatus,
  type TimelineKeyframe,
} from './lib/timelineTypes'

const HOLD_REPEAT_MS = 130

function App() {
  const clientRef = useRef(new EspClient())
  const holdTimerRef = useRef(0)
  const [ip, setIp] = useState(() => loadEspIp())
  const [connectionState, setConnectionState] = useState<ConnectionState>('disconnected')
  const [status, setStatus] = useState<RobotStatus>(defaultStatus)
  const [lastPollAt, setLastPollAt] = useState<Date | null>(null)
  const [pollFailed, setPollFailed] = useState(false)
  const [toast, setToast] = useState('Ready')
  const [errors, setErrors] = useState<string[]>([])
  const [keyframes, setKeyframes] = useState<TimelineKeyframe[]>([])
  const [selectedId, setSelectedId] = useState<string | null>(null)
  const [playheadIndex, setPlayheadIndex] = useState(0)
  const [activeTab, setActiveTab] = useState<'control' | 'timeline'>('control')

  useEffect(() => {
    const client = clientRef.current
    const offStatus = client.onStatus((nextStatus) => {
      setStatus((current) => ({ ...current, ...nextStatus }))
      setLastPollAt(new Date())
      setPollFailed(false)
    })
    const offConnection = client.onConnection(setConnectionState)
    const offError = client.onError((message) => {
      setPollFailed(message.startsWith('Status poll failed'))
      setErrors((current) => [message, ...current].slice(0, 8))
    })
    return () => {
      offStatus()
      offConnection()
      offError()
      client.disconnect()
    }
  }, [])

  useEffect(() => {
    const timer = window.setInterval(() => {
      if (ip.trim()) void clientRef.current.pollStatus(ip)
    }, 420)
    return () => window.clearInterval(timer)
  }, [ip])

  useEffect(() => {
    const stop = () => stopHold()
    const onVisibility = () => {
      if (document.visibilityState === 'hidden') stopHold()
    }
    window.addEventListener('blur', stop)
    document.addEventListener('visibilitychange', onVisibility)
    return () => {
      window.removeEventListener('blur', stop)
      document.removeEventListener('visibilitychange', onVisibility)
    }
  }, [])

  useEffect(() => {
    const onKeyDown = (event: KeyboardEvent) => {
      const target = event.target as HTMLElement | null
      if (target?.matches('input, textarea, select')) return
      if (event.repeat) return
      const key = event.key.toLowerCase()
      if (key === ' ') {
        event.preventDefault()
        sendCommand(ESP_COMMANDS.stop)
      }
      if (key === 'h') sendCommand(ESP_COMMANDS.home)
      if (key === 'e') sendCommand(ESP_COMMANDS.estop)
      if (key === 'c') sendCommand(ESP_COMMANDS.clearEstop)
      if (key === 'o') sendCommand(ESP_COMMANDS.clawOpen)
      if (key === 'g') sendCommand(ESP_COMMANDS.clawSoft)
    }
    window.addEventListener('keydown', onKeyDown)
    return () => window.removeEventListener('keydown', onKeyDown)
  })

  function flash(message: string) {
    setToast(message)
  }

  function sendCommand(command: string) {
    const ok = clientRef.current.send(command)
    flash(ok ? `Sent ${command}` : `Not connected: ${command}`)
  }

  function connect() {
    saveEspIp(ip)
    clientRef.current.connect(ip)
    flash(`Connecting to ${ip}`)
  }

  function disconnect() {
    stopHold()
    clientRef.current.disconnect()
    flash('Disconnected')
  }

  function startHold(command: string) {
    stopHold(false)
    clientRef.current.sendHoldCommand(command)
    holdTimerRef.current = window.setInterval(() => {
      clientRef.current.sendHoldCommand(command)
    }, HOLD_REPEAT_MS)
  }

  function stopHold(sendStop = true) {
    window.clearInterval(holdTimerRef.current)
    holdTimerRef.current = 0
    if (sendStop) clientRef.current.stop()
  }

  function addKeyframe(type: KeyframeType) {
    if (keyframes.length >= ESP_MAX_KEYFRAMES) {
      flash(`ESP firmware timeline limit is ${ESP_MAX_KEYFRAMES} keyframes`)
      return
    }
    const next = makeKeyframe(type, status, keyframes.length)
    setKeyframes((current) => [...current, next])
    setSelectedId(next.id)
    flash(`Captured ${type} keyframe`)
  }

  function updateKeyframe(id: string, patch: Partial<TimelineKeyframe>) {
    setKeyframes((current) => current.map((keyframe) => (keyframe.id === id ? { ...keyframe, ...patch } : keyframe)))
  }

  function deleteKeyframe(id: string) {
    setKeyframes((current) => current.filter((keyframe) => keyframe.id !== id))
    setSelectedId(null)
  }

  function duplicateKeyframe(id: string) {
    const source = keyframes.find((keyframe) => keyframe.id === id)
    if (!source) return
    const duplicate = { ...source, id: crypto.randomUUID(), label: `${source.label} copy` }
    const index = keyframes.findIndex((keyframe) => keyframe.id === id)
    setKeyframes((current) => [...current.slice(0, index + 1), duplicate, ...current.slice(index + 1)])
    setSelectedId(duplicate.id)
  }

  function moveKeyframe(id: string, direction: -1 | 1) {
    setKeyframes((current) => {
      const index = current.findIndex((keyframe) => keyframe.id === id)
      const nextIndex = index + direction
      if (index < 0 || nextIndex < 0 || nextIndex >= current.length) return current
      const next = [...current]
      const [item] = next.splice(index, 1)
      next.splice(nextIndex, 0, item)
      return next
    })
  }

  async function playFrontendTimeline() {
    if (status.estop) {
      flash('Playback disabled while ESTOP is active')
      return
    }
    if (connectionState !== 'connected' && !confirm('Robot is not connected. Try playback anyway?')) return

    for (let index = 0; index < keyframes.length; index += 1) {
      const keyframe = keyframes[index]
      setPlayheadIndex(index)
      sendCommand(setToolCommand(keyframe))

      if (keyframe.type !== 'WAIT') {
        sendCommand(setTargetCommand(keyframe))
      }

      await new Promise((resolve) => window.setTimeout(resolve, keyframe.durationMs))

      if (keyframe.type === 'GRAB' && keyframe.clawTicks >= 0) {
        sendCommand(setClawCommand(keyframe.clawTicks))
      }

      if (keyframe.type === 'DROP') {
        sendCommand(ESP_COMMANDS.clawOpen)
      }

      await new Promise((resolve) => window.setTimeout(resolve, keyframe.waitAfterMs))
    }
    flash('Frontend timeline pass complete')
  }

  return (
    <div className="min-h-screen">
      <SafetyBar connectionState={connectionState} status={status} onCommand={sendCommand} />
      <main className="mx-auto grid max-w-[1800px] gap-4 px-3 py-4">
        <header className="app-header">
          <div>
            <p className="eyebrow">Raspberry Pi Control Host</p>
            <h1>Desk Robot Arm Console</h1>
          </div>
          <div className="toast">{toast}</div>
        </header>

        <nav className="tab-bar" aria-label="Robot arm workspace">
          <button className={activeTab === 'control' ? 'tab-button active' : 'tab-button'} onClick={() => setActiveTab('control')}>
            Control Room
          </button>
          <button className={activeTab === 'timeline' ? 'tab-button active' : 'tab-button'} onClick={() => setActiveTab('timeline')}>
            Timeline Studio
          </button>
        </nav>

        {activeTab === 'control' ? (
          <>
            <div className="grid gap-4 xl:grid-cols-[420px_1fr]">
              <ConnectionPanel
                ip={ip}
                setIp={setIp}
                connectionState={connectionState}
                lastPollAt={lastPollAt}
                pollFailed={pollFailed}
                onConnect={connect}
                onDisconnect={disconnect}
                onSave={() => {
                  saveEspIp(ip)
                  flash('ESP IP saved locally')
                }}
              />
              <TelemetryPanel status={status} />
            </div>

            <QuickActions status={status} onCommand={sendCommand} />
            <ManualControls startHold={startHold} stopHold={stopHold} />
          </>
        ) : (
          <>
            <div className="grid gap-4 xl:grid-cols-[420px_1fr]">
              <ConnectionPanel
                ip={ip}
                setIp={setIp}
                connectionState={connectionState}
                lastPollAt={lastPollAt}
                pollFailed={pollFailed}
                onConnect={connect}
                onDisconnect={disconnect}
                onSave={() => {
                  saveEspIp(ip)
                  flash('ESP IP saved locally')
                }}
              />
              <TelemetryPanel status={status} />
            </div>
            <TimelineEditor
              status={status}
              connectionState={connectionState}
              keyframes={keyframes}
              selectedId={selectedId}
              playheadIndex={playheadIndex}
              setSelectedId={setSelectedId}
              addKeyframe={addKeyframe}
              setKeyframes={setKeyframes}
              updateKeyframe={updateKeyframe}
              deleteKeyframe={deleteKeyframe}
              duplicateKeyframe={duplicateKeyframe}
              moveKeyframe={moveKeyframe}
              sendCommand={sendCommand}
              toast={flash}
              playFrontendTimeline={playFrontendTimeline}
            />
          </>
        )}

        <section className="panel">
          <div className="panel-title">Error Log</div>
          {errors.length === 0 ? <div className="empty-state">No connection or polling errors yet.</div> : null}
          <div className="grid gap-2">
            {errors.map((error, index) => (
              <div className="log-line" key={`${error}-${index}`}>
                {error}
              </div>
            ))}
          </div>
        </section>
      </main>
    </div>
  )
}

export default App
