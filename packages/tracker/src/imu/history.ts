import { quatSlerp } from '../math/quat'
import type { Quat } from '../math/quat'

/** Ring buffer of timestamped orientations (monotonic non-decreasing t, any unit). */
export class ImuHistory {
  private readonly ts: number[]
  private readonly qs: Quat[]
  private head = 0 // next write slot
  private count = 0

  constructor(private readonly capacity = 120) {
    this.ts = new Array<number>(capacity).fill(0)
    this.qs = new Array<Quat>(capacity).fill([0, 0, 0, 1])
  }

  push(t: number, q: Quat): void {
    this.ts[this.head] = t
    this.qs[this.head] = q
    this.head = (this.head + 1) % this.capacity
    if (this.count < this.capacity) this.count++
  }

  /** i = 0 oldest .. count-1 newest. */
  private slot(i: number): number {
    return (this.head - this.count + i + this.capacity * 2) % this.capacity
  }

  latest(): { t: number; q: Quat } | null {
    if (this.count === 0) return null
    const s = this.slot(this.count - 1)
    return { t: this.ts[s] as number, q: this.qs[s] as Quat }
  }

  at(t: number): Quat | null {
    if (this.count === 0) return null
    const first = this.slot(0)
    if (t <= (this.ts[first] as number)) return this.qs[first] as Quat
    for (let i = this.count - 1; i >= 0; i--) {
      const s = this.slot(i)
      const ti = this.ts[s] as number
      if (ti <= t) {
        if (i === this.count - 1 || ti === t) return this.qs[s] as Quat
        const n = this.slot(i + 1)
        const tn = this.ts[n] as number
        const f = tn > ti ? (t - ti) / (tn - ti) : 0
        return quatSlerp(this.qs[s] as Quat, this.qs[n] as Quat, f)
      }
    }
    return this.qs[first] as Quat
  }
}
