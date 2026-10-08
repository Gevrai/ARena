import { describe, expect, it } from 'vitest'
import { OneEuroFilter } from '../src/fusion/oneEuro'
import { ImuHistory } from '../src/imu/history'
import { deviceOrientationToQuat } from '../src/imu/orientation'
import { quatAngle, quatFromAxisAngle, quatMultiply, quatSlerp, quatToMat3 } from '../src/math/quat'
import type { Quat } from '../src/math/quat'

const rotate = (q: Quat, v: [number, number, number]): number[] => {
  const m = quatToMat3(q)
  return [0, 1, 2].map(
    (r) => (m[r * 3] ?? 0) * v[0] + (m[r * 3 + 1] ?? 0) * v[1] + (m[r * 3 + 2] ?? 0) * v[2],
  )
}
const near = (a: number[], b: number[], tol = 1e-9): void =>
  a.forEach((x, i) => expect(Math.abs(x - (b[i] ?? 0))).toBeLessThan(tol))

describe('deviceOrientationToQuat', () => {
  it('upright portrait phone looks along the horizontal -Z', () => {
    const q = deviceOrientationToQuat(0, 90, 0, 0)
    near(rotate(q, [0, 0, -1]), [0, 0, -1])
    near(rotate(q, [0, 1, 0]), [0, 1, 0])
  })
  it('flat phone, screen up: camera -Z points to world -Y', () => {
    const q = deviceOrientationToQuat(0, 0, 0, 0)
    near(rotate(q, [0, 0, -1]), [0, -1, 0])
  })
  it('screenAngle 90 is a 90 degree roll about the camera Z', () => {
    const q0 = deviceOrientationToQuat(30, 70, 10, 0)
    const q90 = deviceOrientationToQuat(30, 70, 10, 90)
    const roll = quatMultiply([q0[0] * -1, q0[1] * -1, q0[2] * -1, q0[3]], q90) // q0^-1 q90
    expect(quatAngle(roll, quatFromAxisAngle([0, 0, 1], -Math.PI / 2))).toBeLessThan(1e-6)
  })
})

describe('ImuHistory', () => {
  it('interpolates at the midpoint and clamps outside', () => {
    const h = new ImuHistory()
    const a = quatFromAxisAngle([0, 1, 0], 0)
    const b = quatFromAxisAngle([0, 1, 0], 1)
    h.push(100, a)
    h.push(200, b)
    expect(quatAngle(h.at(150) as Quat, quatSlerp(a, b, 0.5))).toBeLessThan(1e-6)
    expect(h.at(50)).toEqual(a)
    expect(h.at(500)).toEqual(b)
    expect(h.latest()?.t).toBe(200)
    expect(new ImuHistory().at(1)).toBeNull()
  })
  it('wraps at capacity', () => {
    const h = new ImuHistory(4)
    for (let i = 0; i < 10; i++) h.push(i * 10, quatFromAxisAngle([0, 1, 0], i * 0.1))
    expect(h.latest()?.t).toBe(90)
    const q = h.at(75) as Quat
    expect(quatAngle(q, quatFromAxisAngle([0, 1, 0], 0.75))).toBeLessThan(1e-6)
    // oldest retained is t=60; earlier clamps to it
    expect(quatAngle(h.at(0) as Quat, quatFromAxisAngle([0, 1, 0], 0.6))).toBeLessThan(1e-6)
  })
})

describe('OneEuroFilter', () => {
  it('converges on a constant input', () => {
    const f = new OneEuroFilter(1, 0)
    let y = 0
    for (let i = 0; i < 200; i++) y = f.filter(5, i / 60)
    expect(y).toBeCloseTo(5, 6)
  })
  it('follows a step within 3 samples with large beta', () => {
    const f = new OneEuroFilter(1, 50)
    for (let i = 0; i < 30; i++) f.filter(0, i / 60)
    let y = 0
    for (let i = 30; i < 33; i++) y = f.filter(1, i / 60)
    expect(y).toBeGreaterThan(0.9)
  })
  it('smooths with beta 0 and resets', () => {
    const f = new OneEuroFilter(1, 0)
    f.filter(0, 0)
    expect(f.filter(1, 1 / 60)).toBeLessThan(0.2)
    f.reset()
    expect(f.filter(7, 5)).toBe(7)
  })
})
