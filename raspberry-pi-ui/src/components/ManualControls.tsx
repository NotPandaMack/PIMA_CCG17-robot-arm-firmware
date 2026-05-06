import { ArrowDown, ArrowLeft, ArrowRight, ArrowUp, Hand, Move3D, RotateCcw, RotateCw } from 'lucide-react'
import type { ReactNode } from 'react'
import { holdCommand, ikMoveCommand } from '../lib/commandBuilder'
import { ESP_COMMAND_TIMEOUT_MS } from '../lib/timelineTypes'

type Props = {
  startHold: (command: string) => void
  stopHold: () => void
}

type HoldButtonProps = {
  label: string
  command: string
  icon: ReactNode
  startHold: (command: string) => void
  stopHold: () => void
}

function HoldButton({ label, command, icon, startHold, stopHold }: HoldButtonProps) {
  return (
    <button
      className="hold-button"
      title={label}
      onPointerDown={(event) => {
        event.currentTarget.setPointerCapture(event.pointerId)
        startHold(command)
      }}
      onPointerUp={stopHold}
      onPointerCancel={stopHold}
      onPointerLeave={stopHold}
    >
      {icon}
      <span>{label}</span>
    </button>
  )
}

function ControlGroup({ title, children }: { title: string; children: ReactNode }) {
  return (
    <div className="control-group">
      <h3>{title}</h3>
      <div className="grid grid-cols-2 gap-2">{children}</div>
    </div>
  )
}

export function ManualControls({ startHold, stopHold }: Props) {
  return (
    <section className="panel">
      <div className="panel-title">
        <Move3D size={18} />
        Manual Control
      </div>
      <div className="grid gap-3 lg:grid-cols-2 2xl:grid-cols-4">
        <ControlGroup title="IK Target">
          <HoldButton label="X Left" command={ikMoveCommand(-1.2, 0, 0, 0)} icon={<ArrowLeft size={18} />} startHold={startHold} stopHold={stopHold} />
          <HoldButton label="X Right" command={ikMoveCommand(1.2, 0, 0, 0)} icon={<ArrowRight size={18} />} startHold={startHold} stopHold={stopHold} />
          <HoldButton label="Y Back" command={ikMoveCommand(0, -1.2, 0, 0)} icon={<ArrowDown size={18} />} startHold={startHold} stopHold={stopHold} />
          <HoldButton label="Y Fwd" command={ikMoveCommand(0, 1.2, 0, 0)} icon={<ArrowUp size={18} />} startHold={startHold} stopHold={stopHold} />
        </ControlGroup>
        <ControlGroup title="Height / Pitch">
          <HoldButton label="Z Down" command={ikMoveCommand(0, 0, -1.2, 0)} icon={<ArrowDown size={18} />} startHold={startHold} stopHold={stopHold} />
          <HoldButton label="Z Up" command={ikMoveCommand(0, 0, 1.2, 0)} icon={<ArrowUp size={18} />} startHold={startHold} stopHold={stopHold} />
          <HoldButton label="Fine Z -" command={ikMoveCommand(0, 0, -0.25, 0)} icon={<ArrowDown size={18} />} startHold={startHold} stopHold={stopHold} />
          <HoldButton label="Fine Z +" command={ikMoveCommand(0, 0, 0.25, 0)} icon={<ArrowUp size={18} />} startHold={startHold} stopHold={stopHold} />
          <HoldButton label="Pitch -" command={ikMoveCommand(0, 0, 0, -0.2)} icon={<RotateCcw size={18} />} startHold={startHold} stopHold={stopHold} />
          <HoldButton label="Pitch +" command={ikMoveCommand(0, 0, 0, 0.2)} icon={<RotateCw size={18} />} startHold={startHold} stopHold={stopHold} />
        </ControlGroup>
        <ControlGroup title="Joints">
          <HoldButton label="Base L" command={holdCommand(0, -1)} icon={<ArrowLeft size={18} />} startHold={startHold} stopHold={stopHold} />
          <HoldButton label="Base R" command={holdCommand(0, 1)} icon={<ArrowRight size={18} />} startHold={startHold} stopHold={stopHold} />
          <HoldButton label="Shoulder -" command={holdCommand(1, -1)} icon={<ArrowDown size={18} />} startHold={startHold} stopHold={stopHold} />
          <HoldButton label="Shoulder +" command={holdCommand(1, 1)} icon={<ArrowUp size={18} />} startHold={startHold} stopHold={stopHold} />
          <HoldButton label="Elbow -" command={holdCommand(2, -1)} icon={<ArrowDown size={18} />} startHold={startHold} stopHold={stopHold} />
          <HoldButton label="Elbow +" command={holdCommand(2, 1)} icon={<ArrowUp size={18} />} startHold={startHold} stopHold={stopHold} />
          <HoldButton label="Wrist -" command={holdCommand(3, -1)} icon={<RotateCcw size={18} />} startHold={startHold} stopHold={stopHold} />
          <HoldButton label="Wrist +" command={holdCommand(3, 1)} icon={<RotateCw size={18} />} startHold={startHold} stopHold={stopHold} />
        </ControlGroup>
        <ControlGroup title="Claw / Joystick">
          <HoldButton label="Claw -" command={holdCommand(4, -1)} icon={<Hand size={18} />} startHold={startHold} stopHold={stopHold} />
          <HoldButton label="Claw +" command={holdCommand(4, 1)} icon={<Hand size={18} />} startHold={startHold} stopHold={stopHold} />
          <div className="joystick col-span-2">
            <div />
            <span>Repeat 130 ms / ESP timeout {ESP_COMMAND_TIMEOUT_MS} ms</span>
          </div>
        </ControlGroup>
      </div>
    </section>
  )
}
