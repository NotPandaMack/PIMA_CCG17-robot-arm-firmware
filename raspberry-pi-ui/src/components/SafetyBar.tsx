import { OctagonAlert, PlugZap, Square } from 'lucide-react'
import type { ConnectionState } from '../lib/espClient'
import type { RobotStatus } from '../lib/timelineTypes'

type Props = {
  connectionState: ConnectionState
  status: RobotStatus
  onCommand: (command: string) => void
}

export function SafetyBar({ connectionState, status, onCommand }: Props) {
  return (
    <div className="sticky top-0 z-40 border-b border-red-500/25 bg-[#090c11]/92 px-3 py-3 backdrop-blur-xl">
      <div className="mx-auto flex max-w-[1800px] flex-wrap items-center gap-3">
        <button className="danger-button min-h-14 flex-1 sm:flex-none" onClick={() => onCommand('ESTOP')}>
          <OctagonAlert size={22} />
          ESTOP
        </button>
        <button className="control-button min-h-14" onClick={() => onCommand('STOP')}>
          <Square size={18} />
          STOP
        </button>
        <div className="status-pill">
          <PlugZap size={16} />
          <span>{connectionState}</span>
        </div>
        <div className={status.estop ? 'status-pill border-red-500/50 text-red-200' : 'status-pill text-emerald-200'}>
          ESTOP {status.estop ? 'ACTIVE' : 'CLEAR'}
        </div>
        <div className="hidden min-w-0 flex-1 truncate text-sm text-slate-400 md:block">{status.status}</div>
      </div>
    </div>
  )
}
