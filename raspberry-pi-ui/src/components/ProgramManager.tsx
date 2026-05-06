import { Download, FolderOpen, Import, Save, Trash2 } from 'lucide-react'
import { useState } from 'react'
import { deleteProgram, downloadJson, loadPrograms, saveProgram } from '../lib/storage'
import type { RobotProgram, TimelineKeyframe } from '../lib/timelineTypes'

type Props = {
  keyframes: TimelineKeyframe[]
  onLoad: (keyframes: TimelineKeyframe[]) => void
  toast: (message: string) => void
}

export function ProgramManager({ keyframes, onLoad, toast }: Props) {
  const [name, setName] = useState('Desk Arm Program')
  const [notes, setNotes] = useState('')
  const [programs, setPrograms] = useState<RobotProgram[]>(() => loadPrograms())

  function refresh() {
    setPrograms(loadPrograms())
  }

  function handleSave() {
    const saved = saveProgram(name || 'Untitled Program', notes, keyframes)
    refresh()
    toast(`Saved ${saved.name}`)
  }

  async function handleImport(file: File | undefined) {
    if (!file) return
    const text = await file.text()
    const data = JSON.parse(text) as RobotProgram | { keyframes: TimelineKeyframe[] }
    onLoad('keyframes' in data ? data.keyframes : [])
    toast(`Imported ${file.name}`)
  }

  return (
    <section className="panel">
      <div className="panel-title">
        <FolderOpen size={18} />
        Program Storage
      </div>
      <div className="grid gap-3">
        <input className="text-input" value={name} onChange={(event) => setName(event.target.value)} placeholder="Program name" />
        <textarea className="text-input min-h-20" value={notes} onChange={(event) => setNotes(event.target.value)} placeholder="Program notes" />
        <div className="grid grid-cols-2 gap-2">
          <button className="primary-button" onClick={handleSave}>
            <Save size={16} />
            Save Program
          </button>
          <button className="control-button" onClick={() => downloadJson(`${name || 'robot-program'}.json`, { name, notes, keyframes, updatedAt: new Date().toISOString() })}>
            <Download size={16} />
            Export JSON
          </button>
          <label className="control-button cursor-pointer">
            <Import size={16} />
            Import JSON
            <input className="hidden" type="file" accept="application/json" onChange={(event) => void handleImport(event.target.files?.[0])} />
          </label>
        </div>
        <div className="grid gap-2">
          {programs.map((program) => (
            <div className="program-row" key={program.id}>
              <button onClick={() => onLoad(program.keyframes)}>
                <strong>{program.name}</strong>
                <span>{new Date(program.updatedAt).toLocaleString()}</span>
              </button>
              <button
                className="icon-button"
                title="Delete saved program"
                onClick={() => {
                  deleteProgram(program.id)
                  refresh()
                }}
              >
                <Trash2 size={16} />
              </button>
            </div>
          ))}
        </div>
      </div>
    </section>
  )
}
