import { Bolt, Hand, Home, Power, Route, ShieldCheck, SlidersHorizontal, Square, Table2 } from 'lucide-react'
import type { ReactNode } from 'react'
import { ESP_COMMANDS, speedCommand } from '../lib/commandBuilder'
import type { RobotStatus } from '../lib/timelineTypes'

type Props = {
  status: RobotStatus
  onCommand: (command: string) => void
}

function ActionButton({
  label,
  command,
  icon,
  variant = 'default',
  onCommand,
}: {
  label: string
  command: string
  icon: ReactNode
  variant?: 'default' | 'danger'
  onCommand: (command: string) => void
}) {
  return (
    <button className={variant === 'danger' ? 'danger-soft-button' : 'control-button'} onClick={() => onCommand(command)}>
      {icon}
      {label}
    </button>
  )
}

export function QuickActions({ status, onCommand }: Props) {
  return (
    <section className="panel">
      <div className="panel-title">
        <Bolt size={18} />
        Quick Actions
      </div>
      <div className="grid grid-cols-2 gap-2 md:grid-cols-3 xl:grid-cols-4">
        <ActionButton label="Clear ESTOP" command={ESP_COMMANDS.clearEstop} icon={<ShieldCheck size={17} />} onCommand={onCommand} />
        <ActionButton label="All Off" command={ESP_COMMANDS.allOff} icon={<Power size={17} />} variant="danger" onCommand={onCommand} />
        <ActionButton label="Home" command={ESP_COMMANDS.home} icon={<Home size={17} />} onCommand={onCommand} />
        <ActionButton label="Stop" command={ESP_COMMANDS.stop} icon={<Square size={17} />} onCommand={onCommand} />
        <ActionButton label="Table Skim" command={ESP_COMMANDS.tableSkim} icon={<Table2 size={17} />} onCommand={onCommand} />
        <ActionButton label="Lift/Carry" command={ESP_COMMANDS.carry} icon={<Route size={17} />} onCommand={onCommand} />
        <ActionButton label="Tip Grip" command={ESP_COMMANDS.toolTip} icon={<Hand size={17} />} onCommand={onCommand} />
        <ActionButton label="Deep Gap" command={ESP_COMMANDS.toolGap} icon={<Hand size={17} />} onCommand={onCommand} />
        <ActionButton label="Claw Open" command={ESP_COMMANDS.clawOpen} icon={<Hand size={17} />} onCommand={onCommand} />
        <ActionButton label="Soft Close" command={ESP_COMMANDS.clawSoft} icon={<Hand size={17} />} onCommand={onCommand} />
        <ActionButton label="Firm Close" command={ESP_COMMANDS.clawFirm} icon={<Hand size={17} />} onCommand={onCommand} />
        <div className="speed-card">
          <div className="flex items-center gap-2 text-sm font-semibold text-slate-200">
            <SlidersHorizontal size={17} />
            Speed {status.speed ?? 1}
          </div>
          <input
            type="range"
            min="1"
            max="6"
            value={status.speed ?? 1}
            onChange={(event) => onCommand(speedCommand(Number(event.target.value)))}
          />
        </div>
      </div>
    </section>
  )
}
