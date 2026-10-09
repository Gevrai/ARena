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

export const ACCEL_TAU_S = 0.4
export const ACCEL_MAX_MS = 800
export const ACCEL_MAX_DISP_M = 0.3
export const ACCEL_VEL_WINDOW_MS = 200
export const ACCEL_BLEND_MS = 150
const MAX_V0 = 2

const smoothstep = (p: number): number => p * p * (3 - 2 * p)

interface Anchor {
  t: number
  p: Vec3
  v: Vec3
}

/**
 * Dead-reckons camera translation from accelerometer samples between marker detections.
 * position(now) = anchor + integrated displacement (+ a decaying blend of the jump caused by the
 * previous anchor being replaced). `worldRot(t)` gives the world-from-camera rotation at time t.
 */
export class AccelTranslator {
  private samples: AccelSample[] = []
  private hist: { t: number; p: Vec3 }[] = []
  private anchor: Anchor | null = null
  private corr: Vec3 = [0, 0, 0]
  private corrStart = 0

  reset(): void {
    this.samples = []
    this.hist = []
    this.anchor = null
    this.corr = [0, 0, 0]
  }

  hasAnchor(): boolean {
    return this.anchor !== null
  }

  clearAnchor(): void {
    this.anchor = null
    this.hist = []
  }

  push(t: number, a: Vec3): void {
    this.samples.push({ t, a })
    while (this.samples.length > 0 && t - (this.samples[0] as AccelSample).t > 3000)
      this.samples.shift()
  }

  /** Most recent sample rotated into the world frame ([0,0,0] if unavailable). */
  latestWorld(worldRot: (t: number) => Quat | null): Vec3 {
    const s = this.samples[this.samples.length - 1]
    const q = s ? worldRot(s.t) : null
    return s && q ? rotateByQuat(q, s.a) : [0, 0, 0]
  }

  /** Displacement since the anchor (without the blend term) at time `now`. */
  displacement(now: number, worldRot: (t: number) => Quat | null): Vec3 {
    const an = this.anchor
    if (!an) return [0, 0, 0]
    const end = Math.min(now, an.t + ACCEL_MAX_MS)
    const v: Vec3 = [an.v[0], an.v[1], an.v[2]]
    const d: Vec3 = [0, 0, 0]
    let prev = an.t
    let lastA: Vec3 = [0, 0, 0]
    const step = (t1: number, a: Vec3): void => {
      const dt = (t1 - prev) / 1000
      if (dt <= 0) return
      const k = Math.exp(-dt / ACCEL_TAU_S)
      for (let i = 0; i < 3; i++) {
        v[i] = (v[i] as number) * k + (a[i] as number) * dt
        d[i] = (d[i] as number) + (v[i] as number) * dt
      }
      prev = t1
    }
    for (const s of this.samples) {
      if (s.t <= an.t) continue
      const q = worldRot(s.t)
      if (!q) continue
      lastA = rotateByQuat(q, s.a)
      step(Math.min(s.t, end), lastA)
      if (s.t >= end) break
    }
    if (prev < end) step(end, lastA)
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

  /** Record a raw marker position (capture time t), for the initial-velocity estimate. */
  noteMarker(t: number, p: Vec3): void {
    this.hist.push({ t, p })
    while (this.hist.length > 0 && t - (this.hist[0] as { t: number }).t > 400) this.hist.shift()
  }

  private slope(t0: number): Vec3 {
    const pts = this.hist.filter((h) => h.t >= t0 - ACCEL_VEL_WINDOW_MS && h.t <= t0)
    if (pts.length < 2) return [0, 0, 0]
    const tm = pts.reduce((s, h) => s + h.t, 0) / pts.length
    let den = 0
    for (const h of pts) den += (h.t - tm) * (h.t - tm)
    if (den < 1e-9) return [0, 0, 0]
    const v: Vec3 = [0, 0, 0]
    for (let i = 0; i < 3; i++) {
      let num = 0
      const pm = pts.reduce((s, h) => s + (h.p[i] as number), 0) / pts.length
      for (const h of pts) num += (h.t - tm) * ((h.p[i] as number) - pm)
      v[i] = (num / den) * 1000
    }
    const len = Math.hypot(v[0], v[1], v[2])
    return len > MAX_V0 ? [(v[0] * MAX_V0) / len, (v[1] * MAX_V0) / len, (v[2] * MAX_V0) / len] : v
  }

  /**
   * New detection: anchor at capture time t0 with position p. `arrival` is when it became known;
   * the jump relative to the previous output at that moment is blended out.
   */
  setAnchor(t0: number, p: Vec3, arrival: number, worldRot: (t: number) => Quat | null): void {
    const before = this.position(arrival, worldRot)
    this.anchor = { t: t0, p: [p[0], p[1], p[2]], v: this.slope(t0) }
    this.corr = [0, 0, 0]
    const clean = this.position(arrival, worldRot) as Vec3
    this.corr = before
      ? [before[0] - clean[0], before[1] - clean[1], before[2] - clean[2]]
      : [0, 0, 0]
    this.corrStart = arrival
  }
}
