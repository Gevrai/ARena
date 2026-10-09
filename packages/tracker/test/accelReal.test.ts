import { describe, expect, it } from 'vitest'
import { rotateByQuat } from '../src/fusion/accel'
import { PoseFusion } from '../src/fusion/fusion'
import type { MarkerSample } from '../src/fusion/fusion'
import { quatFromAxisAngle, quatInvert, quatMultiply, quatNormalize } from '../src/math/quat'
import type { Quat } from '../src/math/quat'
import type { Vec3 } from '../src/math/vec3'
import { intrinsicsFromSize } from '../src/pose/intrinsics'
import { poseToWorldFromCamera, worldFromCameraQuatToCvR } from '../src/pose/pose'
import { lookAtPose } from './synth'

const DEG = Math.PI / 180
const S = 0.05
const K = intrinsicsFromSize(640, 480, 65)
const YAW_TRUE = quatFromAxisAngle([0, 1, 0], 70 * DEG)

function rng(seed: number): () => number {
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

interface Scenario {
  useAccel: boolean
  tEnd: number
  /** World X acceleration (m/s^2) as a function of time (ms). */
  accelX: (t: number) => number
  /** Whether the 12 Hz detection frame at time t is delivered. */
  detect: (t: number) => boolean
  /** What the sensor reports (default: the truth acceleration). */
  sensorX?: (t: number) => number
  seed?: number
  /** accel sample period (ms) with +-jitter */
  accelPeriod?: number
}

interface Result {
  t: number[]
  out: Vec3[]
  truth: Vec3[]
  disp: number[] // mm
}

function simulate(sc: Scenario): Result {
  const rnd = rng(sc.seed ?? 5)
  const f = new PoseFusion({ useAccel: sc.useAccel })
  const q = base.quaternion
  const imuQ = quatMultiply(quatInvert(YAW_TRUE), q)
  const qInv = quatInvert(q)
  // truth integration at 1 ms
  const truthX: number[] = []
  let x = 0
  let v = 0
  for (let t = 0; t <= sc.tEnd; t++) {
    v += (sc.accelX(t) * 1) / 1000
    x += (v * 1) / 1000
    truthX[t] = x
  }
  const res: Result = { t: [], out: [], truth: [], disp: [] }
  const period = sc.accelPeriod ?? 26
  let nextAccel = 0
  let nextFrame = 0
  const pending: { tf: number; at: number; s: MarkerSample }[] = []
  for (let t = 0; t <= sc.tEnd; t += 5) {
    f.onImu(t, imuQ) // 200 Hz IMU for simplicity
    while (nextAccel <= t) {
      const ta = nextAccel
      nextAccel += period + (rnd() - 0.5) * 6
      const aW: Vec3 = [
        (sc.sensorX ?? sc.accelX)(Math.round(ta)) + 0.04 * gauss(rnd),
        0.04 * gauss(rnd),
        0.04 * gauss(rnd),
      ]
      // camera-frame accel: inverse(world-from-camera) * world accel (world = marker frame)
      const aCam = rotateByQuat(
        qInv,
        rotateByQuat(YAW_TRUE, rotateByQuat(quatInvert(YAW_TRUE), aW)),
      )
      f.onAccel(ta, [aCam[0] + 0.03, aCam[1] + 0.03, aCam[2]])
    }
    while (nextFrame <= t) {
      const tf = nextFrame
      nextFrame += 1000 / 12
      if (!sc.detect(tf)) continue
      const p: Vec3 = [
        base.position[0] + (truthX[Math.round(tf)] ?? 0) + 0.003 * gauss(rnd),
        base.position[1] + 0.003 * gauss(rnd),
        base.position[2] + 0.003 * gauss(rnd),
      ]
      pending.push({
        tf,
        at: tf + 55,
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
    res.t.push(t)
    res.out.push(o)
    res.truth.push([base.position[0] + (truthX[t] ?? 0), base.position[1], base.position[2]])
    const info = f.accelInfo(t)
    res.disp.push(info ? Math.hypot(...info.disp) * 1000 : 0)
    if (info && t % 100 === 0 && t >= 3300 && t <= 3900)
      console.log(
        'DV',
        t,
        info.disp.map((x) => (x * 1000).toFixed(1)).join(','),
        'v',
        info.vel.map((x) => x.toFixed(3)).join(','),
      )
  }
  return res
}

const dist = (a: Vec3, b: Vec3): number => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2])
const maxOver = (r: Result, a: number, b: number, fn: (i: number) => number): number => {
  let m = 0
  r.t.forEach((t, i) => {
    if (t >= a && t <= b) m = Math.max(m, fn(i))
  })
  return m
}
const meanOver = (r: Result, a: number, b: number, fn: (i: number) => number): number => {
  let s = 0
  let n = 0
  r.t.forEach((t, i) => {
    if (t >= a && t <= b) {
      s += fn(i)
      n++
    }
  })
  return s / n
}

describe('accelerometer, realistic device conditions', () => {
  const restLost: Scenario = {
    useAccel: true,
    tEnd: 9000,
    accelX: () => 0,
    detect: (t) => t < 2000,
  }

  it('2a: no displacement jump right after detections stop at rest', () => {
    const r = simulate(restLost)
    const d = maxOver(r, 2000, 3000, (i) => r.disp[i] as number)
    const e = maxOver(r, 2000, 3000, (i) => dist(r.out[i] as Vec3, r.truth[i] as Vec3))
    console.log(`2a: max disp ${d.toFixed(1)} mm, max err ${(e * 1000).toFixed(1)} mm`)
    expect(d).toBeLessThan(15)
    expect(e).toBeLessThan(0.015)
  })

  it('2b: output holds still for seconds after the marker is lost (no late jumps)', () => {
    const r = simulate(restLost)
    const e = maxOver(r, 2000, 9000, (i) => dist(r.out[i] as Vec3, r.truth[i] as Vec3))
    let step = 0
    for (let i = 1; i < r.t.length; i++)
      if ((r.t[i] as number) > 2200)
        step = Math.max(step, dist(r.out[i] as Vec3, r.out[i - 1] as Vec3))
    console.log(`2b: max err ${(e * 1000).toFixed(1)} mm, max step ${(step * 1000).toFixed(2)} mm`)
    expect(e).toBeLessThan(0.03)
    expect(step).toBeLessThan(0.002)
    const a = r.out[r.t.indexOf(5000)] as Vec3
    const b = r.out[r.t.indexOf(9000)] as Vec3
    expect(dist(a, b)).toBeLessThan(1e-9)
  })

  it('2c: a 0.3 s sideways slide (+-2 m/s2 pulses) is followed with 80% detections dropped', () => {
    // Hand-like smooth pulse pair: +2 then -2 m/s2 (sine lobes of 0.15 s each).
    const pulse = (t: number): number =>
      t >= 3000 && t < 3300 ? 2 * Math.sin((2 * Math.PI * (t - 3000)) / 300) : 0
    const slideSc = (useAccel: boolean): Scenario => ({
      useAccel,
      tEnd: 5000,
      accelX: pulse,
      detect: (t) => (t >= 2950 && t <= 3700 ? Math.round(t / 83.33) % 5 === 0 : true),
    })
    const off = simulate(slideSc(false))
    const on = simulate(slideSc(true))
    const err = (r: Result, a: number, b: number): number =>
      meanOver(r, a, b, (i) => dist(r.out[i] as Vec3, r.truth[i] as Vec3))
    const eOff = err(off, 3050, 3700)
    const eOn = err(on, 3050, 3700)
    console.log(`2c: err held ${(eOff * 1000).toFixed(1)} mm, accel ${(eOn * 1000).toFixed(1)} mm`)
    expect(eOff).toBeGreaterThan(0.008)
    expect(eOn).toBeLessThan(eOff * 0.6)
    const late = maxOver(on, 4000, 5000, (i) => dist(on.out[i] as Vec3, on.truth[i] as Vec3))
    expect(late).toBeLessThan(0.01)
  })

  it('option off: identical output regardless of accel samples', () => {
    const a = simulate({ ...restLost, useAccel: false })
    const b = simulate({ ...restLost, useAccel: false, sensorX: () => 3 })
    // detections identical (truth unaffected by accel in the rest scenario of a; use positions)
    for (let i = 0; i < a.t.length; i += 7) expect(a.out[i]).toEqual(b.out[i])
  })
})
