import { PoseFusion } from '../src/fusion/fusion'
import type { MarkerSample } from '../src/fusion/fusion'
import { quatAngle, quatFromAxisAngle, quatInvert, quatMultiply, quatNormalize } from '../src/math/quat'
import type { Quat } from '../src/math/quat'
import type { Vec3 } from '../src/math/vec3'
import { intrinsicsFromSize } from '../src/pose/intrinsics'
import { estimatePose, poseToWorldFromCamera, worldFromCameraQuatToCvR } from '../src/pose/pose'
import { lookAtPose } from './synth'

export const DEG = Math.PI / 180
export const S = 0.05
export const K = intrinsicsFromSize(640, 480, 65)
const Y: Vec3 = [0, 1, 0]
export const YAW_TRUE = quatFromAxisAngle(Y, 70 * DEG)

export function mulberry32(seed: number): () => number {
  let a = seed >>> 0
  return () => {
    a = (a + 0x6d2b79f5) >>> 0
    let t = a
    t = Math.imul(t ^ (t >>> 15), t | 1)
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61)
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296
  }
}

export interface Truth {
  q: Quat
  p: Vec3
}

/** Static camera `distanceM` from the marker at the given tilt (deg from the marker normal). */
export function staticTruth(tiltDeg: number, distanceM = 0.4): Truth {
  const cv = lookAtPose({ distanceM, tiltDeg, yawDeg: 20, rollDeg: 0 })
  const b = poseToWorldFromCamera({ R: cv.R, t: cv.t, reprojErrorPx: 0 })
  return { q: b.quaternion, p: b.position }
}

/** Exact marker corners (px) for a world-from-camera pose, with gaussian-ish pixel noise. */
export function cornersFor(tr: Truth, noisePx: number, rnd: () => number): MarkerSample['corners'] {
  const R = worldFromCameraQuatToCvR(tr.q)
  const g = (i: number): number => R[i] ?? 0
  const tm: Vec3 = [tr.p[0], tr.p[2], -tr.p[1]]
  const t: Vec3 = [
    -(g(0) * tm[0] + g(1) * tm[1] + g(2) * tm[2]),
    -(g(3) * tm[0] + g(4) * tm[1] + g(5) * tm[2]),
    -(g(6) * tm[0] + g(7) * tm[1] + g(8) * tm[2]),
  ]
  const h = S / 2
  const n = (): number => (rnd() + rnd() + rnd() - 1.5) * 2 * noisePx
  return ([[-h, -h], [h, -h], [h, h], [-h, h]] as [number, number][]).map(([X, Yc]) => {
    const xc = g(0) * X + g(1) * Yc + t[0]
    const yc = g(3) * X + g(4) * Yc + t[1]
    const zc = g(6) * X + g(7) * Yc + t[2]
    return [K.fx * (xc / zc) + K.cx + n(), K.fy * (yc / zc) + K.cy + n()] as [number, number]
  }) as MarkerSample['corners']
}

/** A marker sample whose pose comes from the (noisy) corners, like the real worker does. */
export function sampleFromCorners(corners: MarkerSample['corners']): MarkerSample | null {
  const cv = estimatePose(corners, K, S)
  if (!cv) return null
  const w = poseToWorldFromCamera(cv)
  return {
    position: w.position,
    quaternion: w.quaternion,
    corners,
    K,
    markerSizeM: S,
    reprojErrorPx: cv.reprojErrorPx,
  }
}

export interface RestStats {
  posRmsMm: number
  /** RMS rotation about world Y of (out * truth^-1), degrees. */
  yawRmsDeg: number
  /** RMS of the non-yaw (tilt) part, degrees. */
  tiltRmsDeg: number
  /** RMS on-screen jitter (px, fx=K.fx) of a point 75 mm from the marker centre. */
  edgePxRms: number
  nonFlatFrac: number
  n: number
}

export interface RestOpts {
  fusion?: ConstructorParameters<typeof PoseFusion>[0]
  noisePx?: number
  markerHz?: number
  seconds?: number
  tiltDeg?: number
  seed?: number
  /** IMU tilt noise, degrees (gaussian-ish). */
  imuNoiseDeg?: number
}

function yawTwistAngle(q: Quat): { yaw: number; tilt: number } {
  const n = Math.hypot(q[1], q[3])
  const t: Quat = n < 1e-9 ? [0, 0, 0, 1] : [0, q[1] / n, 0, q[3] / n]
  return { yaw: 2 * Math.atan2(t[1], t[3]), tilt: quatAngle(q, t) }
}

function projectPoint(q: Quat, p: Vec3, w: Vec3): [number, number] {
  // world point -> pixel for a world-from-camera pose (three.js camera: x right, y up, -z fwd)
  const inv = quatInvert(q)
  const d: Vec3 = [w[0] - p[0], w[1] - p[1], w[2] - p[2]]
  const [x, y, z, ww] = inv
  const tx = 2 * (y * d[2] - z * d[1])
  const ty = 2 * (z * d[0] - x * d[2])
  const tz = 2 * (x * d[1] - y * d[0])
  const c: Vec3 = [
    d[0] + ww * tx + (y * tz - z * ty),
    d[1] + ww * ty + (z * tx - x * tz),
    d[2] + ww * tz + (x * ty - y * tx),
  ]
  return [K.fx * (c[0] / -c[2]), K.fx * (c[1] / -c[2])]
}

/** Stationary camera, noisy corners -> marker samples; measure the jitter of get() after warm-up. */
export function restJitter(o: RestOpts): RestStats {
  const rnd = mulberry32(o.seed ?? 3)
  const truth = staticTruth(o.tiltDeg ?? 25)
  const f = new PoseFusion({ ...o.fusion })
  const hz = o.markerHz ?? 30
  const latency = 60
  const dtImu = 1000 / 60
  const gn = (): number => (rnd() + rnd() + rnd() - 1.5) * 2
  const pending: { tf: number; at: number; s: MarkerSample }[] = []
  let nextFrame = 0
  const out: { q: Quat; p: Vec3; flat: boolean }[] = []
  for (let t = 0; t <= (o.seconds ?? 6) * 1000; t += dtImu) {
    let imuQ = quatMultiply(quatInvert(YAW_TRUE), truth.q)
    if (o.imuNoiseDeg) {
      const ax: Vec3 = [gn(), gn(), gn()]
      imuQ = quatNormalize(quatMultiply(quatFromAxisAngle(ax, o.imuNoiseDeg * DEG * 0.3), imuQ))
    }
    f.onImu(t, imuQ)
    while (nextFrame <= t) {
      const tf = nextFrame
      nextFrame += 1000 / hz
      const s = sampleFromCorners(cornersFor(truth, o.noisePx ?? 0.5, rnd))
      if (s) pending.push({ tf, at: tf + latency, s })
    }
    while (pending.length && (pending[0] as { at: number }).at <= t) {
      const p = pending.shift() as { tf: number; at: number; s: MarkerSample }
      f.onMarker(p.tf, p.s, p.at)
    }
    if (t > 2000) {
      const g = f.get(t)
      out.push({ q: g.quaternion, p: g.position, flat: g.flat })
    }
  }
  const w: Vec3 = [0.075, 0, 0]
  const ref = projectPoint(truth.q, truth.p, w)
  let ep = 0, ey = 0, et = 0, ex = 0, nf = 0
  // yaw/tilt relative to the mean orientation error
  const errs = out.map((s) => {
    const e = quatMultiply(s.q, quatInvert(truth.q))
    return yawTwistAngle(e.map((v) => v) as Quat)
  })
  const my = errs.reduce((a, e) => a + wrap(e.yaw), 0) / errs.length
  out.forEach((s, i) => {
    const e = errs[i] as { yaw: number; tilt: number }
    ey += (wrap(e.yaw) - my) ** 2
    et += e.tilt ** 2
    ep += (s.p[0] - truth.p[0]) ** 2 + (s.p[1] - truth.p[1]) ** 2 + (s.p[2] - truth.p[2]) ** 2
    const pr = projectPoint(s.q, s.p, w)
    ex += (pr[0] - ref[0]) ** 2 + (pr[1] - ref[1]) ** 2
    if (!s.flat) nf++
  })
  const n = out.length
  return {
    posRmsMm: Math.sqrt(ep / n) * 1000,
    yawRmsDeg: Math.sqrt(ey / n) / DEG,
    tiltRmsDeg: Math.sqrt(et / n) / DEG,
    edgePxRms: Math.sqrt(ex / n),
    nonFlatFrac: nf / n,
    n,
  }
}
const wrap = (a: number): number => Math.atan2(Math.sin(a), Math.cos(a))

export interface SlideOpts {
  /** Camera sliding along world X at this speed, m/s. */
  speed: number
  fusion?: ConstructorParameters<typeof PoseFusion>[0]
  /** Camera frame rate; every `detectEvery`-th frame yields a marker (1 = all). */
  frameHz?: number
  detectEvery?: number
  latencyMs?: number
  noisePx?: number
  seconds?: number
  seed?: number
}

/** Mean / p95 position error (mm) of get() against truth for a sliding camera, intermittent detections. */
export function slideError(o: SlideOpts): { meanMm: number; p95Mm: number } {
  const rnd = mulberry32(o.seed ?? 5)
  const base = staticTruth(25)
  const truthAt = (t: number): Truth => ({ q: base.q, p: [base.p[0] + (o.speed * t) / 1000, base.p[1], base.p[2]] })
  const f = new PoseFusion({ ...o.fusion })
  const period = 1000 / (o.frameHz ?? 30)
  const every = o.detectEvery ?? 1
  const latency = o.latencyMs ?? 80
  const pending: { tf: number; at: number; s: MarkerSample }[] = []
  let nextFrame = 0
  let frameNo = 0
  const errs: number[] = []
  for (let t = 0; t <= (o.seconds ?? 5) * 1000; t += 1000 / 60) {
    f.onImu(t, quatMultiply(quatInvert(YAW_TRUE), truthAt(t).q))
    while (nextFrame <= t) {
      const tf = nextFrame
      nextFrame += period
      if (frameNo++ % every !== 0) continue
      const s = sampleFromCorners(cornersFor(truthAt(tf), o.noisePx ?? 0.4, rnd))
      if (s) pending.push({ tf, at: tf + latency, s })
    }
    while (pending.length && (pending[0] as { at: number }).at <= t) {
      const p = pending.shift() as { tf: number; at: number; s: MarkerSample }
      f.onMarker(p.tf, p.s, p.at)
    }
    if (t > 1500) {
      const g = f.get(t).position
      const tp = truthAt(t).p
      errs.push(Math.hypot(g[0] - tp[0], g[1] - tp[1], g[2] - tp[2]) * 1000)
      if (process.env['DBG']) process.stdout.write(`T ${t.toFixed(0)} err ${errs.at(-1)?.toFixed(1)} dx ${((g[0]-tp[0])*1000).toFixed(1)} dy ${((g[1]-tp[1])*1000).toFixed(1)} dz ${((g[2]-tp[2])*1000).toFixed(1)} flat ${f.get(t).flat}\n`)
    }
  }
  errs.sort((a, b) => a - b)
  return { meanMm: errs.reduce((a, b) => a + b, 0) / errs.length, p95Mm: errs[Math.floor(errs.length * 0.95)] as number }
}

export interface RotOpts {
  fusion?: ConstructorParameters<typeof PoseFusion>[0]
  /** Camera yaws about its own centre by +-amplitudeDeg sinusoidally at freqHz. */
  amplitudeDeg?: number
  freqHz?: number
  frameHz?: number
  latencyMs?: number
  /** Error added to the frame timestamp handed to the fusion (bad capture-time estimate). */
  timeSkewMs?: number
  noisePx?: number
  seconds?: number
  seed?: number
}

/** Rotation in place with a perfect gyro: pixel error of the marker corners as rendered by get(). */
export function rotationError(o: RotOpts): { meanPx: number; p95Px: number; maxPx: number; posMm: number; rotDeg: number } {
  const rnd = mulberry32(o.seed ?? 9)
  const base = staticTruth(25)
  const amp = (o.amplitudeDeg ?? 12) * DEG
  const w = 2 * Math.PI * (o.freqHz ?? 1)
  const truthAt = (t: number): Truth => ({
    q: quatMultiply(quatFromAxisAngle(Y, amp * Math.sin((w * t) / 1000)), base.q),
    p: base.p,
  })
  const f = new PoseFusion({ ...o.fusion })
  const period = 1000 / (o.frameHz ?? 30)
  const latency = o.latencyMs ?? 80
  const skew = o.timeSkewMs ?? 0
  const pending: { tf: number; at: number; s: MarkerSample }[] = []
  let nextFrame = 0
  const pts: Vec3[] = [[-0.025, 0, -0.025], [0.025, 0, -0.025], [0.025, 0, 0.025], [-0.025, 0, 0.025]]
  const errs: number[] = []
  let posAcc = 0
  let rotAcc = 0
  let nn = 0
  for (let t = 0; t <= (o.seconds ?? 6) * 1000; t += 1000 / 60) {
    f.onImu(t, quatMultiply(quatInvert(YAW_TRUE), truthAt(t).q))
    while (nextFrame <= t) {
      const tf = nextFrame
      nextFrame += period
      const s = sampleFromCorners(cornersFor(truthAt(tf), o.noisePx ?? 0.4, rnd))
      if (s) pending.push({ tf: tf + skew, at: tf + latency, s })
    }
    while (pending.length && (pending[0] as { at: number }).at <= t) {
      const p = pending.shift() as { tf: number; at: number; s: MarkerSample }
      f.onMarker(p.tf, p.s, p.at)
    }
    if (t > 1500) {
      const g = f.get(t)
      const tr = truthAt(t)
      posAcc += (g.position[0]-tr.p[0])**2+(g.position[1]-tr.p[1])**2+(g.position[2]-tr.p[2])**2
      rotAcc += quatAngle(g.quaternion, tr.q) ** 2
      nn++
      for (const pt of pts) {
        const a = projectPoint(g.quaternion, g.position, pt)
        const b = projectPoint(tr.q, tr.p, pt)
        errs.push(Math.hypot(a[0] - b[0], a[1] - b[1]))
      }
    }
  }
  errs.sort((a, b) => a - b)
  return {
    meanPx: errs.reduce((a, b) => a + b, 0) / errs.length,
    p95Px: errs[Math.floor(errs.length * 0.95)] as number,
    maxPx: errs[errs.length - 1] as number,
    posMm: Math.sqrt(posAcc / nn) * 1000,
    rotDeg: Math.sqrt(rotAcc / nn) / DEG,
  }
}
