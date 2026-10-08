import { describe, expect, it } from 'vitest'
import { estimatePose, poseToWorldFromCamera } from '../src/pose/pose'
import { quatAngle, quatFromMat3 } from '../src/math/quat'
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
const v = (a: ArrayLike<number>, i: number): number => a[i] ?? 0
function must<T>(x: T | null): T {
  if (x === null) throw new Error('unexpected null')
  return x
}
const stats = { exactT: 0, exactA: 0, noisyT: 0, noisyA: 0 }

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
      stats.exactT = Math.max(stats.exactT, dt)
      stats.exactA = Math.max(stats.exactA, da)
      expect(dt).toBeLessThan(0.001)
      expect(da).toBeLessThan((0.5 * Math.PI) / 180)
    }
    console.log('exact max', stats.exactT * 1000, 'mm', (stats.exactA * 180) / Math.PI, 'deg')
  })

  it('stays within 5 mm / 2 deg with +-0.5 px corner noise at 0.4 m', () => {
    const rnd = mulberry32(99)
    for (let i = 0; i < 12; i++) {
      const pose = lookAtPose({
        distanceM: 0.4,
        tiltDeg: 35 + rnd() * 25, // low tilt (<30 deg) is physically ill-conditioned, see task-6 report
        yawDeg: rnd() * 360,
        rollDeg: rnd() * 360,
      })
      const { K, cornersPx } = renderSynthetic(pose, { markerSizeM: S })
      const noisy = cornersPx.map(([x, y]) => [x + (rnd() - 0.5), y + (rnd() - 0.5)]) as [
        [number, number],
        [number, number],
        [number, number],
        [number, number],
      ]
      const est = estimatePose(noisy, K, S)
      expect(est).not.toBeNull()
      const { dt, da } = errs(pose, must(est))
      stats.noisyT = Math.max(stats.noisyT, dt)
      stats.noisyA = Math.max(stats.noisyA, da)
      expect(dt).toBeLessThan(0.005)
      expect(da).toBeLessThan((2 * Math.PI) / 180)
    }
    console.log('noisy max', stats.noisyT * 1000, 'mm', (stats.noisyA * 180) / Math.PI, 'deg')
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
