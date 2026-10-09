import { describe, expect, it } from 'vitest'
import { PoseFusion } from '../src/fusion/fusion'
import type { MarkerSample } from '../src/fusion/fusion'
import {
  quatAngle,
  quatFromAxisAngle,
  quatInvert,
  quatMultiply,
  quatNormalize,
} from '../src/math/quat'
import type { Quat } from '../src/math/quat'
import type { Vec3 } from '../src/math/vec3'
import { intrinsicsFromSize } from '../src/pose/intrinsics'
import { poseToWorldFromCamera, worldFromCameraQuatToCvR } from '../src/pose/pose'
import { lookAtPose } from './synth'

const DEG = Math.PI / 180
const S = 0.05
const K = intrinsicsFromSize(640, 480, 65)
const Y: Vec3 = [0, 1, 0]
const YAW_TRUE = quatFromAxisAngle(Y, 70 * DEG)

function mulberry32(seed: number): () => number {
  let a = seed >>> 0
  return () => {
    a = (a + 0x6d2b79f5) >>> 0
    let t = a
    t = Math.imul(t ^ (t >>> 15), t | 1)
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61)
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296
  }
}

interface Truth {
  q: Quat
  p: Vec3
}

/** Base camera 0.4 m away at the given tilt, orbiting the marker about world Y at omega deg/s. */
function makeTruth(tiltDeg: number, omegaDegPerS: number): (tMs: number) => Truth {
  const cv = lookAtPose({ distanceM: 0.4, tiltDeg, yawDeg: 20, rollDeg: 0 })
  const base = poseToWorldFromCamera({ R: cv.R, t: cv.t, reprojErrorPx: 0 })
  return (tMs) => {
    const ry = quatFromAxisAngle(Y, omegaDegPerS * DEG * (tMs / 1000))
    return {
      q: quatMultiply(ry, base.quaternion),
      p: rotY(base.position, omegaDegPerS * DEG * (tMs / 1000)),
    }
  }
}
const rotY = (v: Vec3, a: number): Vec3 => [
  Math.cos(a) * v[0] + Math.sin(a) * v[2],
  v[1],
  -Math.sin(a) * v[0] + Math.cos(a) * v[2],
]

/** Truth as seen against a marker plane tilted by `deg` about world X. */
function tiltTruth(tr: Truth, deg: number): Truth {
  const r = quatFromAxisAngle([1, 0, 0], deg * DEG)
  const c = Math.cos(deg * DEG)
  const s = Math.sin(deg * DEG)
  const p: Vec3 = [tr.p[0], c * tr.p[1] - s * tr.p[2], s * tr.p[1] + c * tr.p[2]]
  return { q: quatMultiply(r, tr.q), p }
}

/** Exact corners of the marker as seen from a world-from-camera pose, plus optional pixel noise. */
function cornersFor(tr: Truth, noisePx: number, rnd: () => number): MarkerSample['corners'] {
  const R = worldFromCameraQuatToCvR(tr.q)
  const g = (i: number): number => R[i] ?? 0
  // marker-cv position of the camera: tm = (p.x, p.z, -p.y); t = -R tm
  const tm: Vec3 = [tr.p[0], tr.p[2], -tr.p[1]]
  const t: Vec3 = [
    -(g(0) * tm[0] + g(1) * tm[1] + g(2) * tm[2]),
    -(g(3) * tm[0] + g(4) * tm[1] + g(5) * tm[2]),
    -(g(6) * tm[0] + g(7) * tm[1] + g(8) * tm[2]),
  ]
  const h = S / 2
  const obj: [number, number][] = [
    [-h, -h],
    [h, -h],
    [h, h],
    [-h, h],
  ]
  const n = (): number => (rnd() + rnd() + rnd() - 1.5) * 2 * noisePx // ~gaussian-ish, sigma ~ noisePx
  return obj.map(([X, Yc]) => {
    const xc = g(0) * X + g(1) * Yc + t[0]
    const yc = g(3) * X + g(4) * Yc + t[1]
    const zc = g(6) * X + g(7) * Yc + t[2]
    return [K.fx * (xc / zc) + K.cx + n(), K.fy * (yc / zc) + K.cy + n()] as [number, number]
  }) as MarkerSample['corners']
}

interface SimOpts {
  truth: (tMs: number) => Truth
  tEnd: number
  markerHz?: number
  jitterMs?: number
  markerWindows?: [number, number][] // frame-time windows where markers exist
  driftAfterMs?: number
  driftDeg?: number
  noisePx?: number
  markerTiltNoiseDeg?: number
  /** Constant tilt of the marker plane about world X relative to gravity (phone propped up). */
  markerTiltDeg?: number
  cameraFrameNoise?: boolean
  useImu?: boolean
  seed?: number
  onStep?: (tMs: number, f: PoseFusion, tr: Truth) => void
}

function simulate(o: SimOpts): PoseFusion {
  const rnd = mulberry32(o.seed ?? 1)
  const f = new PoseFusion({ useImu: o.useImu })
  const latency = 60
  const pending: { tf: number; at: number; sample: MarkerSample }[] = []
  let nextFrame = 0
  const dtImu = 1000 / 60
  for (let t = 0; t <= o.tEnd; t += dtImu) {
    const drift = o.driftAfterMs !== undefined && t >= o.driftAfterMs ? (o.driftDeg ?? 0) * DEG : 0
    const tr = o.truth(t)
    const offsetEff = quatMultiply(YAW_TRUE, quatFromAxisAngle(Y, drift))
    if (o.useImu !== false) f.onImu(t, quatMultiply(quatInvert(offsetEff), tr.q))
    while (nextFrame <= t) {
      const tf = nextFrame
      nextFrame += 1000 / (o.markerHz ?? 15)
      if (o.markerWindows && !o.markerWindows.some(([a, b]) => tf >= a && tf <= b)) continue
      const trf0 = o.truth(tf)
      const trf = o.markerTiltDeg ? tiltTruth(trf0, o.markerTiltDeg) : trf0
      let mq = trf.q
      if (o.markerTiltNoiseDeg) {
        const ang = rnd() * 2 * Math.PI
        const mag = (rnd() * 2 - 1) * o.markerTiltNoiseDeg * DEG
        const pert = quatFromAxisAngle([Math.cos(ang), 0, Math.sin(ang)], mag) // horizontal axis
        mq = o.cameraFrameNoise
          ? quatMultiply(trf.q, quatFromAxisAngle([Math.cos(ang), Math.sin(ang), 0], mag))
          : quatMultiply(pert, trf.q)
      }
      pending.push({
        tf,
        at: tf + latency + (o.jitterMs ? (rnd() * 2 - 1) * o.jitterMs : 0),
        sample: {
          position: trf.p,
          quaternion: quatNormalize(mq),
          corners: cornersFor(trf, o.noisePx ?? 0, rnd),
          K,
          markerSizeM: S,
          reprojErrorPx: 0.3,
        },
      })
    }
    pending.sort((a, b) => a.at - b.at)
    while (pending.length && (pending[0] as { at: number }).at <= t) {
      const p = pending.shift() as { tf: number; sample: MarkerSample }
      f.onMarker(p.tf, p.sample)
    }
    o.onStep?.(t, f, tr)
  }
  return f
}

const deg = (r: number): number => r / DEG
const dist = (a: Vec3, b: Vec3): number => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2])

describe('PoseFusion', () => {
  it('follows a 30 deg/s rotation with 60 ms marker latency (error < 1.5 deg after warm-up)', () => {
    let worst = 0
    let worstPos = 0
    simulate({
      truth: makeTruth(25, 30),
      tEnd: 4000,
      noisePx: 0.3,
      onStep: (t, f, tr) => {
        if (t < 500) return
        const o = f.get(t)
        worst = Math.max(worst, deg(quatAngle(o.quaternion, tr.q)))
        worstPos = Math.max(worstPos, dist(o.position, tr.p))
      },
    })
    expect(worst).toBeLessThan(1.5)
  })

  it('holds position and follows the IMU when markers stop', () => {
    const truth = makeTruth(25, 30)
    let heldPos: Vec3 | null = null
    let switchedAt = -1
    let worst = 0
    simulate({
      truth,
      tEnd: 3000,
      markerWindows: [[0, 1500]],
      onStep: (t, f, tr) => {
        const o = f.get(t)
        if (t > 1600 && o.source === 'imu' && switchedAt < 0) switchedAt = t
        if (t > 1700) {
          expect(o.source).toBe('imu')
          heldPos ??= o.position
          expect(o.position).toEqual(heldPos)
          worst = Math.max(worst, deg(quatAngle(o.quaternion, tr.q)))
        }
        if (t > 700 && t < 1400) expect(o.source).toBe('marker')
      },
    })
    // last marker frame <= 1500; delivered by ~1560; source must be imu 150 ms after that frame
    expect(switchedAt).toBeLessThan(1500 + 150 + 60)
    expect(worst).toBeLessThan(1.5)
  })

  it('re-acquires after 10 deg IMU drift: <1 deg within 300 ms, no jump > 5 deg', () => {
    const truth = makeTruth(25, 0)
    const resumeAt = 3000
    let prev: Quat | null = null
    let maxJump = 0
    let errAt300 = -1
    let errAtResume = -1
    simulate({
      truth,
      tEnd: 4000,
      markerWindows: [
        [0, 1500],
        [resumeAt, 4000],
      ],
      driftAfterMs: 1600,
      driftDeg: 10,
      noisePx: 0.3,
      onStep: (t, f, tr) => {
        const o = f.get(t)
        if (prev && t > 2000) maxJump = Math.max(maxJump, deg(quatAngle(prev, o.quaternion)))
        prev = o.quaternion
        const e = deg(quatAngle(o.quaternion, tr.q))
        if (t >= 2000 && t < 2900 && errAtResume < 0) errAtResume = e
        if (t >= resumeAt + 60 + 300 && errAt300 < 0) errAt300 = e
      },
    })
    expect(errAtResume).toBeGreaterThan(8)
    expect(errAt300).toBeLessThan(1)
    expect(maxJump).toBeLessThan(5)
  })

  it('without IMU follows the markers', () => {
    let worst = 0
    let src = ''
    simulate({
      truth: makeTruth(25, 30),
      tEnd: 3000,
      useImu: false,
      noisePx: 0.3,
      onStep: (t, f, tr) => {
        if (t < 800) return
        const o = f.get(t)
        src = o.source
        worst = Math.max(worst, deg(quatAngle(o.quaternion, tr.q)))
      },
    })
    expect(src).toBe('marker')
    expect(worst).toBeLessThan(15) // smoothed lag at 30 deg/s with 60 ms latency
  })

  it('reports source marker on 100% of get() calls at 10 Hz markers with latency and jitter', () => {
    let total = 0
    let nonMarker = 0
    simulate({
      truth: makeTruth(25, 30),
      tEnd: 6000,
      markerHz: 10,
      jitterMs: 15,
      noisePx: 0.3,
      onStep: (t, f) => {
        if (t < 500) return
        total++
        if (f.get(t).source !== 'marker') nonMarker++
      },
    })
    expect(total).toBeGreaterThan(300)
    expect(nonMarker).toBe(0)
  })

  it('source becomes imu 150-200 ms after the last marker ARRIVES', () => {
    // last frame at <=1500 arrives ~60 ms later
    let lastMarkerSeen = -1
    let switchAt = -1
    simulate({
      truth: makeTruth(25, 0),
      tEnd: 3000,
      markerHz: 10,
      markerWindows: [[0, 1500]],
      onStep: (t, f) => {
        const s = f.get(t).source
        if (s === 'marker') lastMarkerSeen = t
        else if (switchAt < 0 && t > 1000) switchAt = t
      },
    })
    const sinceLast = switchAt - lastMarkerSeen
    expect(sinceLast).toBeLessThan(40)
    // arrival of last marker is ~1560 ms; switch must be 150-200 ms later
    expect(switchAt).toBeGreaterThan(1500 + 60 + 150 - 20)
    expect(switchAt).toBeLessThan(1500 + 60 + 200 + 20)
  })

  it('marker tilted 40 deg from gravity -> flat false and the pose follows the marker', () => {
    const truth = makeTruth(25, 0)
    let worstRot = 0
    let worstPos = 0
    let flatSeen = true
    let n = 0
    simulate({
      truth,
      tEnd: 3000,
      noisePx: 0.3,
      markerTiltDeg: 40,
      onStep: (t, f, tr) => {
        if (t < 1000) return
        const o = f.get(t)
        const m = tiltTruth(tr, 40)
        n++
        if (o.flat) flatSeen = true
        else flatSeen = false
        worstRot = Math.max(worstRot, deg(quatAngle(o.quaternion, m.q)))
        worstPos = Math.max(worstPos, dist(o.position, m.p))
        expect(o.flat).toBe(false)
        expect(o.confidence).toBeLessThan(0.55)
      },
    })
    expect(n).toBeGreaterThan(50)
    expect(flatSeen).toBe(false)
    expect(worstRot).toBeLessThan(1.5)
    expect(worstPos).toBeLessThan(0.01)
  })

  it('flat marker with +-8 deg tilt noise never trips the flat flag', () => {
    let nonFlat = 0
    let total = 0
    simulate({
      truth: makeTruth(25, 0),
      tEnd: 4000,
      noisePx: 0.3,
      markerTiltNoiseDeg: 8,
      onStep: (t, f) => {
        if (t < 500) return
        total++
        if (!f.get(t).flat) nonFlat++
      },
    })
    expect(total).toBeGreaterThan(100)
    expect(nonFlat).toBe(0)
  })

  it('a corner residual inconsistent with the gravity-locked rotation flags non-flat', () => {
    const f = new PoseFusion()
    const tr = makeTruth(25, 0)(0)
    // IMU consistent with the quaternion (swing 0) but the corners say the marker is rotated.
    const wrong = tiltTruth(tr, 20)
    f.onImu(0, quatMultiply(quatInvert(YAW_TRUE), tr.q))
    f.onMarker(
      10,
      {
        position: tr.p,
        quaternion: tr.q,
        corners: cornersFor(wrong, 0, () => 0.5),
        K,
        markerSizeM: S,
        reprojErrorPx: 0.1,
      },
      10,
    )
    expect(f.get(20).flat).toBe(false)
  })

  for (const cameraFrame of [false, true]) {
    it(`gravity-locked: ±8 deg marker tilt noise (${cameraFrame ? 'camera' : 'world'} axes) -> tilt < 1.5 deg, position < 1 cm`, () => {
      const truth = makeTruth(25, 0)
      let worstRot = 0
      let worstPos = 0
      simulate({
        truth,
        tEnd: 4000,
        noisePx: 0.3,
        markerTiltNoiseDeg: 8,
        cameraFrameNoise: cameraFrame,
        onStep: (t, f, tr) => {
          if (t < 1000) return
          const o = f.get(t)
          worstRot = Math.max(worstRot, deg(quatAngle(o.quaternion, tr.q)))
          worstPos = Math.max(worstPos, dist(o.position, tr.p))
        },
      })
      expect(worstRot).toBeLessThan(1.5)
      expect(worstPos).toBeLessThan(0.01)
    })
  }
})
