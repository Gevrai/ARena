import { describe, expect, it } from 'vitest'
import {
  estimatePose,
  poseToWorldFromCamera,
  solveTranslationGivenRotation,
  worldFromCameraQuatToCvR,
} from '../src/pose/pose'
import { quatAngle, quatFromAxisAngle, quatFromMat3, quatToMat3 } from '../src/math/quat'
import type { Quat } from '../src/math/quat'
import { lookAtPose, renderSynthetic } from './synth'
import type { SynthPose } from './synth'

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

const S = 0.05
type Quad = [[number, number], [number, number], [number, number], [number, number]]
const mul = (A: Float64Array, B: Float64Array): Float64Array => {
  const o = new Float64Array(9)
  for (let r = 0; r < 3; r++)
    for (let c = 0; c < 3; c++)
      for (let k = 0; k < 3; k++) o[r * 3 + c] = v(o, r * 3 + c) + v(A, r * 3 + k) * v(B, k * 3 + c)
  return o
}
const v = (a: ArrayLike<number>, i: number): number => a[i] ?? 0
function must<T>(x: T | null): T {
  if (x === null) throw new Error('unexpected null')
  return x
}

function errs(truth: SynthPose, est: { R: Float64Array; t: number[] }): { dt: number; da: number } {
  const dt = Math.hypot(
    v(est.t, 0) - truth.t[0],
    v(est.t, 1) - truth.t[1],
    v(est.t, 2) - truth.t[2],
  )
  const da = quatAngle(quatFromMat3(truth.R), quatFromMat3(est.R))
  return { dt, da }
}

describe('estimatePose', () => {
  it('recovers exact synthetic poses (10 random)', () => {
    const rnd = mulberry32(1234)
    for (let i = 0; i < 10; i++) {
      const pose = lookAtPose({
        distanceM: 0.2 + rnd() * 0.5,
        tiltDeg: rnd() * 60,
        yawDeg: rnd() * 360,
        rollDeg: rnd() * 360,
      })
      const { K, cornersPx } = renderSynthetic(pose, { markerSizeM: S })
      const est = estimatePose(
        cornersPx as [[number, number], [number, number], [number, number], [number, number]],
        K,
        S,
      )
      expect(est).not.toBeNull()
      const { dt, da } = errs(pose, must(est))
      expect(dt).toBeLessThan(0.001)
      expect(da).toBeLessThan((0.5 * Math.PI) / 180)
    }
  })

  it('noisy corners (+-0.5 px, 0.4 m): 5 mm / 2 deg for tilt >= 35, 12 mm position for lower tilt', () => {
    // At tilt < 35 deg the 50 mm marker is a ~63 px near-frontal quad, so rotation is poorly
    // conditioned under 0.5 px noise (measured worst case ~13 deg rotation, ~10 mm translation
    // over 200 poses). A multi-start probe finds a single minimum with LOWER reprojection error
    // than the truth: the cause is conditioning (fitting noise), not a two-fold ambiguity.
    // Hence below 35 deg only the camera-frame translation t is asserted (< 12 mm). NOTE the
    // WORLD camera position (-R^T t) is far worse there because the rotation error lever-arms
    // over 0.4 m: measured over 300 poses, max 66 mm (tilt 0), 86 mm (10), 89 mm (20), 16 mm (30).
    // That is why the tracker locks tilt to gravity and uses solveTranslationGivenRotation.
    for (const seed of [1, 2, 3, 4, 5, 6]) {
      const rnd = mulberry32(seed * 7919)
      for (let i = 0; i < 6; i++) {
        const tilt = rnd() * 60
        const pose = lookAtPose({
          distanceM: 0.4,
          tiltDeg: tilt,
          yawDeg: rnd() * 360,
          rollDeg: rnd() * 360,
        })
        const { K, cornersPx } = renderSynthetic(pose, { markerSizeM: S })
        const noisy = cornersPx.map(([x, y]) => [x + (rnd() - 0.5), y + (rnd() - 0.5)]) as Quad
        const est = must(estimatePose(noisy, K, S))
        const { dt, da } = errs(pose, est)
        if (tilt >= 35) {
          expect(dt).toBeLessThan(0.005)
          expect(da).toBeLessThan((2 * Math.PI) / 180)
        } else {
          expect(dt).toBeLessThan(0.012)
        }
      }
    }
  })

  it('returns null (not NaN) for collinear corners', () => {
    const { K } = renderSynthetic(lookAtPose({ distanceM: 0.4, tiltDeg: 0, yawDeg: 0, rollDeg: 0 }))
    const line: [[number, number], [number, number], [number, number], [number, number]] = [
      [100, 100],
      [200, 200],
      [300, 300],
      [400, 400],
    ]
    expect(estimatePose(line, K, S)).toBeNull()
    const nan: [[number, number], [number, number], [number, number], [number, number]] = [
      [NaN, 0],
      [1, 0],
      [1, 1],
      [0, 1],
    ]
    expect(estimatePose(nan, K, S)).toBeNull()
  })
})

describe('poseToWorldFromCamera', () => {
  it('camera 0.4 m above marker looking down, image-up = marker top', () => {
    // R = I: camera x = marker x, camera y(down) = marker y(down), looking into the card.
    const p = {
      R: new Float64Array([1, 0, 0, 0, 1, 0, 0, 0, 1]),
      t: [0, 0, 0.4] as [number, number, number],
      reprojErrorPx: 0,
    }
    const { position, quaternion, matrix } = poseToWorldFromCamera(p)
    expect(position[0]).toBeCloseTo(0, 6)
    expect(position[1]).toBeCloseTo(0.4, 6)
    expect(position[2]).toBeCloseTo(0, 6)
    expect(matrix[12]).toBeCloseTo(0, 6)
    expect(matrix[13]).toBeCloseTo(0.4, 6)
    // matrix columns: camera +X, +Y, +Z (= -look) in world.
    const col = (c: number): number[] => [
      v(matrix, c * 4),
      v(matrix, c * 4 + 1),
      v(matrix, c * 4 + 2),
    ]
    const x = col(0),
      y = col(1),
      z = col(2)
    expect(x).toEqual([expect.closeTo(1, 6), expect.closeTo(0, 6), expect.closeTo(0, 6)])
    // camera -Z (view direction) -> world -Y
    expect([-v(z, 0), -v(z, 1), -v(z, 2)]).toEqual([
      expect.closeTo(0, 6),
      expect.closeTo(-1, 6),
      expect.closeTo(0, 6),
    ])
    // camera +Y (image up) -> world -Z (marker top edge)
    expect(y).toEqual([expect.closeTo(0, 6), expect.closeTo(0, 6), expect.closeTo(-1, 6)])
    // quaternion consistent with matrix
    const q: Quat = quaternion
    expect(Math.hypot(...q)).toBeCloseTo(1, 6)
    expect(
      quatAngle(
        q,
        quatFromMat3(
          new Float64Array([
            v(x, 0),
            v(y, 0),
            v(z, 0),
            v(x, 1),
            v(y, 1),
            v(z, 1),
            v(x, 2),
            v(y, 2),
            v(z, 2),
          ]),
        ),
      ),
    ).toBeLessThan(1e-5)
  })
})

describe('rotation-known translation', () => {
  const worldPos = (R: Float64Array, t: [number, number, number]): number[] =>
    poseToWorldFromCamera({ R, t, reprojErrorPx: 0 }).position
  const dist = (a: number[], b: number[]): number =>
    Math.hypot(v(a, 0) - v(b, 0), v(a, 1) - v(b, 1), v(a, 2) - v(b, 2))

  it('worldFromCameraQuatToCvR inverts the conversion', () => {
    const rnd = mulberry32(5)
    for (let i = 0; i < 20; i++) {
      const pose = lookAtPose({
        distanceM: 0.3,
        tiltDeg: rnd() * 70,
        yawDeg: rnd() * 360,
        rollDeg: rnd() * 360,
      })
      const R = worldFromCameraQuatToCvR(
        poseToWorldFromCamera({ ...pose, reprojErrorPx: 0 }).quaternion,
      )
      for (let k = 0; k < 9; k++) expect(v(R, k)).toBeCloseTo(v(pose.R, k), 6)
    }
  })

  it('true R + noise gives < 5 mm camera position error at tilt 0/10/20/45', () => {
    for (const tilt of [0, 10, 20, 45]) {
      for (let seed = 1; seed <= 8; seed++) {
        const rnd = mulberry32(seed * 31 + tilt)
        const pose = lookAtPose({
          distanceM: 0.4,
          tiltDeg: tilt,
          yawDeg: rnd() * 360,
          rollDeg: rnd() * 360,
        })
        const { K, cornersPx } = renderSynthetic(pose, { markerSizeM: S })
        const noisy = cornersPx.map(([x, y]) => [x + (rnd() - 0.5), y + (rnd() - 0.5)]) as Quad
        const sol = must(solveTranslationGivenRotation(noisy, K, S, pose.R))
        expect(dist(worldPos(pose.R, sol.t), worldPos(pose.R, pose.t))).toBeLessThan(0.005)
      }
    }
  })

  it('1 deg rotation error gives < 10 mm position error', () => {
    for (let seed = 1; seed <= 12; seed++) {
      const rnd = mulberry32(seed * 101)
      const pose = lookAtPose({
        distanceM: 0.4,
        tiltDeg: rnd() * 60,
        yawDeg: rnd() * 360,
        rollDeg: rnd() * 360,
      })
      const { K, cornersPx } = renderSynthetic(pose, { markerSizeM: S })
      const noisy = cornersPx.map(([x, y]) => [x + (rnd() - 0.5), y + (rnd() - 0.5)]) as Quad
      const ax = [rnd() - 0.5, rnd() - 0.5, rnd() - 0.5]
      const n = Math.hypot(...ax)
      const q = quatFromAxisAngle([v(ax, 0) / n, v(ax, 1) / n, v(ax, 2) / n], Math.PI / 180)
      const Rp = mul(quatToMat3(q), pose.R)
      const sol = must(solveTranslationGivenRotation(noisy, K, S, Rp))
      expect(dist(worldPos(Rp, sol.t), worldPos(pose.R, pose.t))).toBeLessThan(0.01)
    }
  })

  it('degenerate input returns null', () => {
    const { K } = renderSynthetic(lookAtPose({ distanceM: 0.4, tiltDeg: 0, yawDeg: 0, rollDeg: 0 }))
    const I = new Float64Array([1, 0, 0, 0, 1, 0, 0, 0, 1])
    const nan: Quad = [
      [NaN, 0],
      [1, 0],
      [1, 1],
      [0, 1],
    ]
    expect(solveTranslationGivenRotation(nan, K, S, I)).toBeNull()
    const ok: Quad = [
      [300, 220],
      [340, 220],
      [340, 260],
      [300, 260],
    ]
    expect(solveTranslationGivenRotation(ok, K, 0, I)).toBeNull()
    expect(solveTranslationGivenRotation(ok, K, S, new Float64Array(9).fill(NaN))).toBeNull()
  })
})
