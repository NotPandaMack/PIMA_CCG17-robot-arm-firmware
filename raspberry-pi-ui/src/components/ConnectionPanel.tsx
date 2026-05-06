import { Cable, RefreshCw, Save } from 'lucide-react'
import type { ConnectionState } from '../lib/espClient'

type Props = {
  ip: string
  setIp: (ip: string) => void
  connectionState: ConnectionState
  lastPollAt: Date | null
  pollFailed: boolean
  onConnect: () => void
  onDisconnect: () => void
  onSave: () => void
}

export function ConnectionPanel({
  ip,
  setIp,
  connectionState,
  lastPollAt,
  pollFailed,
  onConnect,
  onDisconnect,
  onSave,
}: Props) {
  const connected = connectionState === 'connected' || connectionState === 'connecting'

  return (
    <section className="panel">
      <div className="panel-title">
        <Cable size={18} />
        Connection
      </div>
      <div className="grid gap-3">
        <label className="field-label" htmlFor="esp-ip">
          ESP8266 IP
        </label>
        <div className="flex flex-col gap-2 sm:flex-row">
          <input
            id="esp-ip"
            className="text-input"
            value={ip}
            placeholder="192.168.1.42"
            inputMode="decimal"
            onChange={(event) => setIp(event.target.value)}
          />
          <button className="control-button" onClick={onSave} title="Save IP locally">
            <Save size={17} />
            Save
          </button>
          <button className={connected ? 'control-button' : 'primary-button'} onClick={connected ? onDisconnect : onConnect}>
            <RefreshCw size={17} className={connectionState === 'connecting' ? 'animate-spin' : ''} />
            {connected ? 'Disconnect' : 'Connect'}
          </button>
        </div>
        <div className="grid grid-cols-1 gap-2 text-sm sm:grid-cols-3">
          <div className="readout">
            <span>WebSocket</span>
            <strong>{connectionState}</strong>
          </div>
          <div className="readout">
            <span>Last poll</span>
            <strong>{lastPollAt ? lastPollAt.toLocaleTimeString() : 'never'}</strong>
          </div>
          <div className={pollFailed ? 'readout border-amber-500/40 text-amber-200' : 'readout'}>
            <span>Status poll</span>
            <strong>{pollFailed ? 'warning' : 'ready'}</strong>
          </div>
        </div>
      </div>
    </section>
  )
}
