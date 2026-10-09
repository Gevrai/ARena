import type { Quat } from '../math/quat'
import { quatInvert } from '../math/quat'
import type { Vec3 } from '../math/vec3'

const DEG = Math.PI / 180
const GRAVITY = 9.80665

/** Rotate v by unit quaternion q (x, y, z, w). */
export function rotateByQuat(q: Quat, v: Vec3): Vec3 {
  const [x, y, z, w] = q
  const tx = 2 * (y * v[2] - z * v[1])
  const ty = 2 * (z * v[0] - x * v[2])
  const tz = 2 * (x * v[1] - y * v[0])
  return [
    v[0] + w * tx + (y * tz - z * ty),
    v[1] + w * ty + (z * tx - x * tz),
    v[2] + w * tz + (x * ty - y * tx),
  ]
}

/**
 * DeviceMotion axes are DEVICE axes (x = right edge, y = top edge, z = out of the screen in the
 * device's natural portrait pose), independent of screen rotation. The IMU quaternion is the
 * camera (screen-oriented) frame = device frame rotated by -screenAngle about z. So camera-frame
 * acceleration = Rz(+screenAngle) * deviceAcceleration. At angle 0 the two frames coincide.
 */
export function deviceAccelToCamera(a: Vec3, screenAngleDeg: number): Vec3 {
  const th = screenAngleDeg * DEG
  const c = Math.cos(th)
  const s = Math.sin(th)
  return [c * a[0] - s * a[1], s * a[0] + c * a[1], a[2]]
}

/**
 * Fallback when `acceleration` is null: remove gravity from accelerationIncludingGravity. At rest
 * the sensor reads +g along "up" (spec convention), so subtract the world up vector (0, g, 0)
 * expressed in the camera frame via the current IMU orientation (world-from-camera).
 */
export function gravityFreeToCamera(agDev: Vec3, imuQ: Quat, screenAngleDeg: number): Vec3 {
  const ag = deviceAccelToCamera(agDev, screenAngleDeg)
  const up = rotateByQuat(quatInvert(imuQ), [0, GRAVITY, 0])
  return [ag[0] - up[0], ag[1] - up[1], ag[2] - up[2]]
}

export interface AccelSample {
  /** ms, same clock as the IMU history. */
  t: number
  /** Camera-frame acceleration (m/s^2, gravity removed). */
  a: Vec3
}

/** Velocity leak time constant: bounds drift from sensor bias but still follows a ~0.3 s slide. */
export const ACCEL_TAU_S = 1.0
/** Dead-reckoning is held (frozen) this long after the anchor's capture time. */
export const ACCEL_MAX_MS = 800
/** Hard cap on the integrated displacement from an anchor (m). */
export const ACCEL_MAX_DISP_M = 0.06
/** Accelerations below this magnitude (m/s^2) are treated as sensor noise / bias. */
export const ACCEL_DEADBAND = 0.12
/** Velocity magnitude clamp (m/s). */
export const ACCEL_MAX_V = 1
/** Below this acceleration magnitude the phone counts as "quiet" (m/s^2). */
const QUIET_A = 0.25
/** After this much continuous quiet the velocity estimate is pulled to zero quickly (ms). */
const QUIET_MS = 100
/** Velocity decay time constant while quiet (s): hand motions end with the hand stopping. */
const QUIET_TAU_S = 0.08
/** One acceleration sample is never held longer than this when integrating (ms). */
const MAX_HOLD_MS = 100
export const ACCEL_BLEND_MS = 150

const smoothstep = (p: number): number => p * p * (3 - 2 * p)

interface VSample {
  t: number
  v: Vec3
}

interface Anchor {
  t: number
  p: Vec3
  /** Committed displacement (m) integrated up to `prevT`. */
  d: Vec3
  prevT: number
  prevV: Vec3
}

/**
 * Dead-reckons camera translation from accelerometer samples between marker detections.
 *
 * A continuous, leaky world-frame velocity estimate is integrated from the acceleration samples
 * (each sample processed once, with its real timestamp, in arrival order). It is never seeded
 * from noisy marker positions. A marker anchor at capture time t0 fixes the position; the output
 * is anchor + the integral of the velocity estimate since t0, frozen `ACCEL_MAX_MS` after t0
 * (so the output holds when detections stop) and capped at `ACCEL_MAX_DISP_M`. The jump caused
 * by replacing the previous anchor is blended out over `ACCEL_BLEND_MS`.
 * `worldRot(t)` gives the world-from-camera rotation at time t.
 */
export class AccelTranslator {
  private pending: AccelSample[] = []
  private vh: VSample[] = []
  private anchor: Anchor | null = null
  private corr: Vec3 = [0, 0, 0]
  private corrStart = 0
  private lastT: number | null = null
  private v: Vec3 = [0, 0, 0]
  private lastAWorld: Vec3 = [0, 0, 0]
  private prevAk: Vec3 = [0, 0, 0]
  private quietSince: number | null = null

  reset(): void {
    this.pending = []
    this.vh = []
    this.anchor = null
    this.corr = [0, 0, 0]
    this.lastT = null
    this.v = [0, 0, 0]
    this.lastAWorld = [0, 0, 0]
    this.prevAk = [0, 0, 0]
    this.quietSince = null
  }

  hasAnchor(): boolean {
    return this.anchor !== null
  }

  clearAnchor(): void {
    this.anchor = null
  }

  push(t: number, a: Vec3): void {
    this.pending.push({ t, a })
    if (this.pending.length > 600) this.pending.splice(0, this.pending.length - 600)
  }

  /** Process queued samples into the velocity history. */
  private advance(worldRot: (t: number) => Quat | null): void {
    for (const s of this.pending) {
      if (this.lastT !== null && s.t <= this.lastT) continue
      const q = worldRot(s.t)
      if (!q) continue
      const aw = rotateByQuat(q, s.a)
      this.lastAWorld = aw
      const mag = Math.hypot(aw[0], aw[1], aw[2])
      const k = mag > ACCEL_DEADBAND ? (mag - ACCEL_DEADBAND) / mag : 0
      const realDt = this.lastT === null ? 0 : (s.t - this.lastT) / 1000
      const dt = Math.min(realDt, MAX_HOLD_MS / 1000)
      if (mag > QUIET_A) this.quietSince = null
      else if (this.quietSince === null) this.quietSince = s.t
      const quiet = this.quietSince !== null && s.t - this.quietSince >= QUIET_MS
      const decay = Math.exp(-realDt / (quiet ? QUIET_TAU_S : ACCEL_TAU_S))
      const ak: Vec3 = [aw[0] * k, aw[1] * k, aw[2] * k]
      // Trapezoid between the previous and this (dead-banded) sample.
      const v: Vec3 = [
        (this.v[0] as number) * decay + (((this.prevAk[0] as number) + ak[0]) / 2) * dt,
        (this.v[1] as number) * decay + (((this.prevAk[1] as number) + ak[1]) / 2) * dt,
        (this.v[2] as number) * decay + (((this.prevAk[2] as number) + ak[2]) / 2) * dt,
      ]
      this.prevAk = ak
      const vl = Math.hypot(v[0], v[1], v[2])
      if (vl > ACCEL_MAX_V) {
        v[0] = (v[0] * ACCEL_MAX_V) / vl
        v[1] = (v[1] * ACCEL_MAX_V) / vl
        v[2] = (v[2] * ACCEL_MAX_V) / vl
      }
      this.v = v
      this.lastT = s.t
      this.vh.push({ t: s.t, v })
    }
    this.pending = []
    const last = this.lastT
    if (last !== null) {
      let n = 0
      while (n < this.vh.length && last - (this.vh[n] as VSample).t > 4000) n++
      if (n > 0) this.vh.splice(0, n)
    }
  }

  /** Most recent sample rotated into the world frame ([0,0,0] if unavailable). */
  latestWorld(worldRot: (t: number) => Quat | null): Vec3 {
    this.advance(worldRot)
    return this.lastAWorld
  }

  /** Current world-frame velocity estimate (m/s). */
  velocity(worldRot: (t: number) => Quat | null): Vec3 {
    this.advance(worldRot)
    return [this.v[0], this.v[1], this.v[2]]
  }

  /** Velocity estimate at time t (linear interpolation of the history; zero before it). */
  private vAt(t: number): Vec3 {
    const h = this.vh
    if (h.length === 0 || t <= (h[0] as VSample).t) return [0, 0, 0]
    for (let i = h.length - 1; i >= 0; i--) {
      const a = h[i] as VSample
      if (a.t <= t) {
        const b = h[i + 1]
        if (!b) return [a.v[0], a.v[1], a.v[2]]
        const f = (t - a.t) / (b.t - a.t)
        return [
          a.v[0] + (b.v[0] - a.v[0]) * f,
          a.v[1] + (b.v[1] - a.v[1]) * f,
          a.v[2] + (b.v[2] - a.v[2]) * f,
        ]
      }
    }
    return [0, 0, 0]
  }

  /** Displacement since the anchor (without the blend term) at time `now`. */
  displacement(now: number, worldRot: (t: number) => Quat | null): Vec3 {
    const an = this.anchor
    if (!an) return [0, 0, 0]
    this.advance(worldRot)
    const end = Math.min(now, an.t + ACCEL_MAX_MS)
    // Commit whole segments up to `end` (never revisited, so the result stays frozen after it).
    for (const s of this.vh) {
      if (s.t <= an.prevT) continue
      const segEnd = Math.min(s.t, end)
      if (segEnd <= an.prevT) break
      const vEnd = s.t <= end ? s.v : this.vAt(segEnd)
      const dt = (segEnd - an.prevT) / 1000
      for (let i = 0; i < 3; i++)
        an.d[i] = (an.d[i] as number) + (((an.prevV[i] as number) + (vEnd[i] as number)) / 2) * dt
      an.prevT = segEnd
      an.prevV = vEnd
      if (s.t >= end) break
    }
    // Uncommitted tail: the last velocity held for at most MAX_HOLD_MS.
    const tail = Math.max(0, Math.min(end - an.prevT, MAX_HOLD_MS)) / 1000
    const d: Vec3 = [
      (an.d[0] as number) + (an.prevV[0] as number) * tail,
      (an.d[1] as number) + (an.prevV[1] as number) * tail,
      (an.d[2] as number) + (an.prevV[2] as number) * tail,
    ]
    const len = Math.hypot(d[0], d[1], d[2])
    if (len > ACCEL_MAX_DISP_M) {
      const f = ACCEL_MAX_DISP_M / len
      return [d[0] * f, d[1] * f, d[2] * f]
    }
    return d
  }

  /** Output position at `now`, or null when there is no anchor. */
  position(now: number, worldRot: (t: number) => Quat | null): Vec3 | null {
    const an = this.anchor
    if (!an) return null
    const d = this.displacement(now, worldRot)
    const w = 1 - smoothstep(Math.min(1, Math.max(0, (now - this.corrStart) / ACCEL_BLEND_MS)))
    return [
      an.p[0] + d[0] + this.corr[0] * w,
      an.p[1] + d[1] + this.corr[1] * w,
      an.p[2] + d[2] + this.corr[2] * w,
    ]
  }

  /**
   * New detection: anchor at capture time t0 with position p. `arrival` is when it became known;
   * the jump relative to the previous output at that moment is blended out.
   */
  setAnchor(t0: number, p: Vec3, arrival: number, worldRot: (t: number) => Quat | null): void {
    const before = this.position(arrival, worldRot)
    this.advance(worldRot)
    this.anchor = {
      t: t0,
      p: [p[0], p[1], p[2]],
      d: [0, 0, 0],
      prevT: t0,
      prevV: this.vAt(t0),
    }
    this.corr = [0, 0, 0]
    const clean = this.position(arrival, worldRot) as Vec3
    this.corr = before
      ? [before[0] - clean[0], before[1] - clean[1], before[2] - clean[2]]
      : [0, 0, 0]
    this.corrStart = arrival
    console.log('anchor', t0, arrival, p[0], 'before', before?.[0], 'clean', clean[0])
  }
}
