import { describe, expect, it } from 'vitest'
import { mat4FromRotationTranslation, mat4Identity, mat4Invert, mat4Multiply } from '../src/math/mat4'
import {
  quatAngle, quatFromAxisAngle, quatFromMat3, quatInvert, quatMultiply, quatSlerp, quatToMat3,
} from '../src/math/quat'
import type { Quat } from '../src/math/quat'
import { intrinsicsFromSize } from '../src/pose/intrinsics'

describe('quat', () => {
  const a = quatFromAxisAngle([1, 2, 3], 0.7)
  it('multiply/invert round trip', () => {
    const r = quatMultiply(a, quatInvert(a))
    expect(quatAngle(r, [0, 0, 0, 1])).toBeLessThan(1e-6)
  })
  it('slerp halfway', () => {
    const b = quatFromAxisAngle([0, 0, 1], 1)
    const h = quatSlerp([0, 0, 0, 1], b, 0.5)
    expect(quatAngle(h, quatFromAxisAngle([0, 0, 1], 0.5))).toBeLessThan(1e-6)
  })
  it('mat3 round trip', () => {
    const q: Quat = quatFromAxisAngle([0.3, -1, 0.5], 2.5)
    expect(quatAngle(quatFromMat3(quatToMat3(q)), q)).toBeLessThan(1e-6)
    const q2: Quat = quatFromAxisAngle([1, 0, 0], 3.0)
    expect(quatAngle(quatFromMat3(quatToMat3(q2)), q2)).toBeLessThan(1e-6)
  })
})

describe('mat4', () => {
  it('invert x original = identity', () => {
    const m = mat4FromRotationTranslation(quatFromAxisAngle([1, 1, 0], 1.1), [1, -2, 3])
    const p = mat4Multiply(mat4Invert(m), m)
    const id = mat4Identity()
    for (let i = 0; i < 16; i++) expect(p[i]).toBeCloseTo(id[i] ?? 0, 5)
  })
  it('matches quatToMat3 column-major layout', () => {
    const q = quatFromAxisAngle([0, 0, 1], Math.PI / 2)
    const m = mat4FromRotationTranslation(q, [0, 0, 0])
    expect(m[0]).toBeCloseTo(0, 6)
    expect(m[1]).toBeCloseTo(1, 6) // x axis maps to +y
  })
})

describe('intrinsics', () => {
  it('640x480 @65deg', () => {
    const K = intrinsicsFromSize(640, 480, 65)
    expect(K.fx).toBeCloseTo(502.3, 1)
    expect(K.cx).toBe(320)
    expect(K.cy).toBe(240)
  })
})
