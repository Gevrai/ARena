const RATE_WINDOW = 30

/** Pure scheduler: at most `maxInFlight` frames outstanding; counts drops and detect rate. */
export class FramePump {
  private readonly maxInFlight: number
  private readonly inFlight = new Set<number>()
  private sent = 0
  private done = 0
  private dropped = 0
  private readonly doneTimes: number[] = []

  constructor(opts: { maxInFlight?: number } = {}) {
    this.maxInFlight = Math.max(1, opts.maxInFlight ?? 1)
  }

  canSend(): boolean {
    return this.inFlight.size < this.maxInFlight
  }

  markSent(id: number): void {
    this.inFlight.add(id)
    this.sent++
  }

  /** Call when a camera frame was skipped because `canSend()` was false. */
  noteDropped(): void {
    this.dropped++
  }

  /** `nowMs` is the completion time (default Date.now()); injectable for tests. */
  markDone(id: number, nowMs: number = Date.now()): void {
    if (!this.inFlight.delete(id)) return
    this.done++
    this.doneTimes.push(nowMs)
    if (this.doneTimes.length > RATE_WINDOW) this.doneTimes.shift()
  }

  stats(): { sent: number; done: number; dropped: number; detectHz: number } {
    const n = this.doneTimes.length
    let detectHz = 0
    if (n >= 2) {
      const span = (this.doneTimes[n - 1] ?? 0) - (this.doneTimes[0] ?? 0)
      if (span > 0) detectHz = ((n - 1) * 1000) / span
    }
    return { sent: this.sent, done: this.done, dropped: this.dropped, detectHz }
  }
}
