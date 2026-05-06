import type { RobotStatus } from './timelineTypes'

export type ConnectionState = 'disconnected' | 'connecting' | 'connected' | 'error'

type StatusListener = (status: RobotStatus) => void
type ConnectionListener = (state: ConnectionState) => void
type ErrorListener = (message: string) => void

export class EspClient {
  private ws: WebSocket | null = null
  private ip = ''
  private shouldReconnect = false
  private reconnectTimer = 0
  private statusListeners = new Set<StatusListener>()
  private connectionListeners = new Set<ConnectionListener>()
  private errorListeners = new Set<ErrorListener>()

  connect(ip: string) {
    this.ip = ip.trim()
    if (!this.ip) {
      this.emitError('ESP8266 IP is required')
      return
    }

    window.clearTimeout(this.reconnectTimer)
    this.shouldReconnect = true
    this.ws?.close()
    this.emitConnection('connecting')

    try {
      this.ws = new WebSocket(`ws://${this.ip}:81/`)
      this.ws.onopen = () => this.emitConnection('connected')
      this.ws.onerror = () => {
        this.emitConnection('error')
        this.emitError('WebSocket error')
      }
      this.ws.onclose = () => {
        this.emitConnection('disconnected')
        if (this.shouldReconnect) {
          this.reconnectTimer = window.setTimeout(() => this.connect(this.ip), 1800)
        }
      }
    } catch (error) {
      this.emitConnection('error')
      this.emitError(error instanceof Error ? error.message : 'Unable to connect WebSocket')
    }
  }

  disconnect() {
    this.shouldReconnect = false
    window.clearTimeout(this.reconnectTimer)
    this.ws?.close()
    this.ws = null
    this.emitConnection('disconnected')
  }

  send(command: string) {
    if (this.ws?.readyState !== WebSocket.OPEN) {
      this.emitError(`WebSocket disconnected: ${command}`)
      return false
    }
    this.ws.send(command)
    return true
  }

  sendHoldCommand(command: string) {
    return this.send(command)
  }

  stop() {
    this.send('STOP')
  }

  async pollStatus(ip = this.ip) {
    const target = ip.trim()
    if (!target) return null

    const controller = new AbortController()
    const timeout = window.setTimeout(() => controller.abort(), 1500)
    try {
      const response = await fetch(`http://${target}/status`, {
        cache: 'no-store',
        signal: controller.signal,
      })
      if (!response.ok) throw new Error(`Status HTTP ${response.status}`)
      const status = (await response.json()) as RobotStatus
      this.statusListeners.forEach((listener) => listener(status))
      return status
    } catch (error) {
      this.emitError(error instanceof Error ? `Status poll failed: ${error.message}` : 'Status poll failed')
      return null
    } finally {
      window.clearTimeout(timeout)
    }
  }

  onStatus(callback: StatusListener) {
    this.statusListeners.add(callback)
    return () => this.statusListeners.delete(callback)
  }

  onConnection(callback: ConnectionListener) {
    this.connectionListeners.add(callback)
    return () => this.connectionListeners.delete(callback)
  }

  onError(callback: ErrorListener) {
    this.errorListeners.add(callback)
    return () => this.errorListeners.delete(callback)
  }

  private emitConnection(state: ConnectionState) {
    this.connectionListeners.forEach((listener) => listener(state))
  }

  private emitError(message: string) {
    this.errorListeners.forEach((listener) => listener(message))
  }
}
