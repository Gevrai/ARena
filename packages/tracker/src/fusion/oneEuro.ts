import { quatAngle, quatSlerp } from '../math/quat'
import type { Quat } from '../math/quat'
import type { Vec3 } from '../math/vec3'

const alphaFor = (cutoff: number, dt: number): number => {
  const tau = 1 / (2 * Math.PI * cutoff)
  return 1 / (1 + tau / dt)
}

/** One Euro filter (Casiez et al. 2012). Times in seconds. */
export class OneEuroFilter {
  private x: number | null = null
  private dx = 0
  private t = 0

  constructor(
    private minCutoff: number,
    private beta: number,
    private dCutoff = 1,
    private deadband = 0,
  ) {}

  setParams(minCutoff: number, beta: number, dCutoff: number, deadband = 0): void {
    this.minCutoff = minCutoff
    this.beta = beta
    this.dCutoff = dCutoff
    this.deadband = deadband
  }

  filter(x: number, tSec: number): number {
    if (this.x === null) {
      this.x = x
      this.t = tSec
      this.dx = 0
      return x
    }
    const dt = Math.max(tSec - this.t, 1e-4)
    this.t = tSec
    const rawD = (x - this.x) / dt
    this.dx += alphaFor(this.dCutoff, dt) * (rawD - this.dx)
    // Speeds below `deadband` are treated as noise: they must not open the filter.
    const cutoff = this.minCutoff + this.beta * Math.max(0, Math.abs(this.dx) - this.deadband)
    this.x += alphaFor(cutoff, dt) * (x - this.x)
    return this.x
  }

  reset(): void {
    this.x = null
    this.dx = 0
  }
}

export class OneEuroVec3 {
  private readonly f: [OneEuroFilter, OneEuroFilter, OneEuroFilter]
  constructor(minCutoff: number, beta: number, dCutoff = 1, deadband = 0) {
    this.f = [
      new OneEuroFilter(minCutoff, beta, dCutoff, deadband),
      new OneEuroFilter(minCutoff, beta, dCutoff, deadband),
      new OneEuroFilter(minCutoff, beta, dCutoff, deadband),
    ]
  }
  filter(v: Vec3, tSec: number): Vec3 {
    return [
      this.f[0].filter(v[0], tSec),
      this.f[1].filter(v[1], tSec),
      this.f[2].filter(v[2], tSec),
    ]
  }
  setParams(minCutoff: number, beta: number, dCutoff: number, deadband = 0): void {
    for (const f of this.f) f.setParams(minCutoff, beta, dCutoff, deadband)
  }
  reset(): void {
    for (const f of this.f) f.reset()
  }
}

/** One Euro style quaternion smoother: slerp toward each sample with alpha from a speed-adaptive cutoff. */
export class OneEuroQuat {
  private q: Quat | null = null
  private speed = 0
  private t = 0

  constructor(
    private minCutoff: number,
    private beta: number,
    private dCutoff = 1,
    private deadband = 0,
  ) {}

  setParams(minCutoff: number, beta: number, dCutoff: number, deadband = 0): void {
    this.minCutoff = minCutoff
    this.beta = beta
    this.dCutoff = dCutoff
    this.deadband = deadband
  }

  filter(q: Quat, tSec: number): Quat {
    if (this.q === null) {
      this.q = q
      this.t = tSec
      return q
    }
    const dt = Math.max(tSec - this.t, 1e-4)
    this.t = tSec
    const raw = quatAngle(this.q, q) / dt
    this.speed += alphaFor(this.dCutoff, dt) * (raw - this.speed)
    this.q = quatSlerp(this.q, q, alphaFor(this.minCutoff + this.beta * Math.max(0, this.speed - this.deadband), dt))
    return this.q
  }

  reset(): void {
    this.q = null
    this.speed = 0
  }
}
