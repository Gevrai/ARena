import { describe, expect, it } from 'vitest'
import { deviceAccelToCamera, gravityFreeToCamera, rotateByQuat } from '../src/fusion/accel'
import { PoseFusion } from '../src/fusion/fusion'
import type { MarkerSample } from '../src/fusion/fusion'
import { deviceOrientationToQuat } from '../src/imu/orientation'
import { quatFromAxisAngle, quatInvert, quatMultiply, quatNormalize } from '../src/math/quat'
import type { Quat } from '../src/math/quat'
import type { Vec3 } from '../src/math/vec3'
import { intrinsicsFromSize } from '../src/pose/intrinsics'
import { poseToWorldFromCamera, worldFromCameraQuatToCvR } from '../src/pose/pose'
import { lookAtPose } from './synth'

const DEG = Math.PI / 180
const near = (a: Vec3, b: Vec3, tol = 1e-9): void => {
  for (let i = 0; i < 3; i++) expect(a[i]).toBeCloseTo(b[i] as number, -Math.log10(tol))
}

describe('accelerometer frame conventions', () => {
  // Device lying flat on the table, screen up, top edge towards the card's top edge (world -Z):
  // alpha=beta=gamma=0. DeviceMotion x = device right edge, y = device top edge, z = out of screen.
  const flatPortrait = deviceOrientationToQuat(0, 0, 0, 0)
  const world = (aDev: Vec3, q: Quat, screenAngle: number): Vec3 =>
    rotateByQuat(q, deviceAccelToCamera(aDev, screenAngle))

  it('portrait: push towards the top edge -> world -Z; right edge -> +X; screen normal -> +Y', () => {
    near(world([0, 1, 0], flatPortrait, 0), [0, 0, -1])
    near(world([1, 0, 0], flatPortrait, 0), [1, 0, 0])
    near(world([0, 0, 1], flatPortrait, 0), [0, 1, 0])
  })

  it('landscape (screen angle 90): same physical push gives the same world direction', () => {
    // Same physical pose, but the OS reports screen angle 90: the camera frame is rotated.
    const q90 = deviceOrientationToQuat(0, 0, 0, 90)
    near(world([0, 1, 0], q90, 90), [0, 0, -1])
    near(world([1, 0, 0], q90, 90), [1, 0, 0])
    near(world([0, 0, 1], q90, 90), [0, 1, 0])
    // The camera-frame vector itself is rotated: device top edge is screen-LEFT in this landscape.
    near(deviceAccelToCamera([0, 1, 0], 90), [-1, 0, 0])
    // Screen angle 270 (other landscape): device top edge is screen-right.
    near(deviceAccelToCamera([0, 1, 0], 270), [1, 0, 0])
    near(world([0, 1, 0], deviceOrientationToQuat(0, 0, 0, 270), 270), [0, 0, -1])
  })

  it('applies the marker yaw offset: offset 90 deg about +Y turns -Z into -X', () => {
    const off = quatFromAxisAngle([0, 1, 0], 90 * DEG)
    near(rotateByQuat(quatMultiply(off, flatPortrait), [0, 1, 0]), [-1, 0, 0])
  })

  it('pushing a held (upright) phone to the screen-left yields a leftward world vector', () => {
    // Held upright facing -Z (beta=90, alpha=0): screen-left is world -X.
    const q = deviceOrientationToQuat(0, 90, 0, 0)
    near(world([-1, 0, 0], q, 0), [-1, 0, 0])
  })

  it('gravity fallback: a resting flat device reading +g on z yields ~0', () => {
    near(gravityFreeToCamera([0, 0, 9.80665], flatPortrait, 0), [0, 0, 0], 1e-6)
    const q90 = deviceOrientationToQuat(0, 0, 0, 90)
    near(gravityFreeToCamera([0, 0, 9.80665], q90, 90), [0, 0, 0], 1e-6)
    // plus a push along device top
    near(
      rotateByQuat(flatPortrait, gravityFreeToCamera([0, 2, 9.80665], flatPortrait, 0)),
      [0, 0, -2],
      1e-6,
    )
  })
})

// ---------------- simulation ----------------
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
const gauss = (r: () => number): number => (r() + r() + r() + r() - 2) * Math.sqrt(3)

const base = (() => {
  const cv = lookAtPose({ distanceM: 0.4, tiltDeg: 35, yawDeg: 20, rollDeg: 0 })
  return poseToWorldFromCamera({ R: cv.R, t: cv.t, reprojErrorPx: 0 })
})()

function corners(q: Quat, p: Vec3): MarkerSample['corners'] {
  const R = worldFromCameraQuatToCvR(q)
  const g = (i: number): number => R[i] ?? 0
  const tm: Vec3 = [p[0], p[2], -p[1]]
  const t: Vec3 = [
    -(g(0) * tm[0] + g(1) * tm[1] + g(2) * tm[2]),
    -(g(3) * tm[0] + g(4) * tm[1] + g(5) * tm[2]),
    -(g(6) * tm[0] + g(7) * tm[1] + g(8) * tm[2]),
  ]
  const h = S / 2
  return [
    [-h, -h],
    [h, -h],
    [h, h],
    [-h, h],
  ].map(([X, Yc]) => {
    const xc = g(0) * (X as number) + g(1) * (Yc as number) + t[0]
    const yc = g(3) * (X as number) + g(4) * (Yc as number) + t[1]
    const zc = g(6) * (X as number) + g(7) * (Yc as number) + t[2]
    return [K.fx * (xc / zc) + K.cx, K.fy * (yc / zc) + K.cy] as [number, number]
  }) as MarkerSample['corners']
}

/** World X velocity profile (m/s) of the slide: ramp up 100 ms, cruise, ramp down 100 ms. */
function slideVel(tMs: number, t0: number, t1: number, v: number): number {
  if (tMs < t0 || tMs > t1 + 100) return 0
  if (tMs < t0 + 100) return (v * (tMs - t0)) / 100
  if (tMs <= t1) return v
  return (v * (t1 + 100 - tMs)) / 100
}

interface Run {
  err: (tFrom: number, tTo: number) => number
  disp: (t: number) => Vec3
  out: (t: number) => Vec3
  final: Vec3
}

function run(opts: {
  useAccel?: boolean
  feedAccel?: boolean
  slide: boolean
  markerUntil: number
  markerFrom: number
  tEnd: number
  seed?: number
}): Run {
  const rnd = mulberry32(opts.seed ?? 3)
  const f = new PoseFusion({ useAccel: opts.useAccel })
  const q = base.quaternion
  const imuQ = quatMultiply(quatInvert(YAW_TRUE), q)
  const qInv = quatInvert(q)
  const biasCam: Vec3 = [0.05 * 0.6, 0.05 * 0.8, 0]
  const truthX = new Map<number, number>()
  let x = 0
  const dt = 1000 / 100
  const outs = new Map<number, Vec3>()
  const truthP = (t: number): Vec3 => [
    base.position[0] + (truthX.get(Math.round(t)) ?? 0),
    base.position[1],
    base.position[2],
  ]
  const pending: { tf: number; at: number; s: MarkerSample }[] = []
  let nextFrame = 0
  const errs: { t: number; e: number }[] = []
  const dispOf = new Map<number, Vec3>()
  for (let t = 0; t <= opts.tEnd; t += 10) {
    const v = opts.slide ? slideVel(t, 1000, 1600, 0.3) : 0
    x += v * (dt / 1000)
    truthX.set(t, x)
    f.onImu(t, imuQ)
    // world accel = dv/dt
    const vPrev = opts.slide ? slideVel(t - 10, 1000, 1600, 0.3) : 0
    const aW: Vec3 = [(v - vPrev) / (dt / 1000), 0, 0]
    const aCam = rotateByQuat(qInv, rotateByQuat(YAW_TRUE, rotateByQuat(quatInvert(YAW_TRUE), aW)))
    // the world frame of the sim IS marker world; imuQ already maps camera into IMU world, so
    // camera-frame accel = inverse(world-from-camera) * world accel.
    const sample: Vec3 = [
      aCam[0] + biasCam[0] + 0.05 * gauss(rnd),
      aCam[1] + biasCam[1] + 0.05 * gauss(rnd),
      aCam[2] + 0.05 * gauss(rnd),
    ]
    if (opts.feedAccel !== false) f.onAccel(t, sample)
    while (nextFrame <= t) {
      const tf = nextFrame
      nextFrame += 1000 / 30
      if (tf > opts.markerUntil && tf < opts.markerFrom) continue
      const p = truthP(Math.round(tf / 10) * 10)
      pending.push({
        tf,
        at: tf + 60,
        s: {
          position: p,
          quaternion: quatNormalize(q),
          corners: corners(q, p),
          K,
          markerSizeM: S,
          reprojErrorPx: 0.3,
        },
      })
    }
    while (pending.length && (pending[0] as { at: number }).at <= t) {
      const p = pending.shift() as { tf: number; s: MarkerSample }
      f.onMarker(p.tf, p.s, t)
    }
    const o = f.get(t).position
    outs.set(t, o)
    const tp = truthP(t)
    errs.push({ t, e: Math.hypot(o[0] - tp[0], o[1] - tp[1], o[2] - tp[2]) })
    const info = f.accelInfo(t)
    if (info) dispOf.set(t, info.disp)
  }
  return {
    err: (a, b) => {
      const sel = errs.filter((r) => r.t >= a && r.t <= b)
      return sel.reduce((s, r) => s + r.e, 0) / sel.length
    },
    disp: (t) => dispOf.get(t) ?? [0, 0, 0],
    out: (t) => outs.get(t) as Vec3,
    final: outs.get(Math.floor(opts.tEnd / 10) * 10) as Vec3,
  }
}

describe('accelerometer translation (simulation)', () => {
  const slide = { slide: true, markerUntil: 1050, markerFrom: 99999, tEnd: 2000 }

  it('cuts position error during the marker-lost slide versus holding', () => {
    const off = run({ ...slide, useAccel: false })
    const on = run({ ...slide, useAccel: true })
    for (const [a, b] of [
      [1100, 1700],
      [1100, 2000],
    ] as const)
      console.log(
        `gap error ${a}-${b} ms: held ${(off.err(a, b) * 1000).toFixed(1)} mm, accel ${(on.err(a, b) * 1000).toFixed(1)} mm`,
      )
    // During the slide itself (marker lost, truth moving at 0.3 m/s).
    const eOff = off.err(1100, 1700)
    const eOn = on.err(1100, 1700)
    expect(eOff).toBeGreaterThan(0.03)
    expect(eOn).toBeLessThan(eOff * 0.65)
  })

  it('at rest with noise+bias the drift stays within a few cm and is then held', () => {
    const on = run({
      slide: false,
      useAccel: true,
      markerUntil: 300,
      markerFrom: 99999,
      tEnd: 3000,
    })
    let worst = 0
    for (let t = 400; t <= 3000; t += 10) {
      const o = on.out(t)
      worst = Math.max(
        worst,
        Math.hypot(o[0] - base.position[0], o[1] - base.position[1], o[2] - base.position[2]),
      )
    }
    console.log(`rest drift max: ${(worst * 1000).toFixed(1)} mm`)
    expect(worst).toBeLessThan(0.04)
    const a = on.out(2000)
    const b = on.out(3000)
    near(a, b, 1e-12) // held after the 0.8 s integration window
  })

  it('option off: identical output whether or not accel samples are fed, and to default', () => {
    const a = run({ ...slide, useAccel: false })
    const b = run({ ...slide, useAccel: undefined, feedAccel: false })
    for (let t = 0; t <= 2000; t += 10) expect(a.out(t)).toEqual(b.out(t))
  })

  it('re-detection blends the correction out without a jump larger than the residual', () => {
    const on = run({ slide: true, markerUntil: 1050, markerFrom: 1800, tEnd: 2600, useAccel: true })
    let maxStep = 0
    for (let t = 1800; t < 2600; t += 10) {
      const a = on.out(t)
      const b = on.out(t + 10)
      maxStep = Math.max(maxStep, Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]))
    }
    expect(maxStep).toBeLessThan(0.02)
  })
})
