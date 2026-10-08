import type { Quat } from './quat'
import type { Vec3 } from './vec3'

/** 4x4 matrix, column-major (WebGL / three.js). */
export type Mat4 = Float32Array

export function mat4Identity(): Mat4 {
  const m = new Float32Array(16)
  m[0] = m[5] = m[10] = m[15] = 1
  return m
}

export function mat4FromRotationTranslation(q: Quat, t: Vec3): Mat4 {
  const [x, y, z, w] = q
  const m = new Float32Array(16)
  m[0] = 1 - 2 * (y * y + z * z)
  m[1] = 2 * (x * y + z * w)
  m[2] = 2 * (x * z - y * w)
  m[4] = 2 * (x * y - z * w)
  m[5] = 1 - 2 * (x * x + z * z)
  m[6] = 2 * (y * z + x * w)
  m[8] = 2 * (x * z + y * w)
  m[9] = 2 * (y * z - x * w)
  m[10] = 1 - 2 * (x * x + y * y)
  m[12] = t[0]
  m[13] = t[1]
  m[14] = t[2]
  m[15] = 1
  return m
}

/** a * b */
export function mat4Multiply(a: Mat4, b: Mat4): Mat4 {
  const o = new Float32Array(16)
  for (let c = 0; c < 4; c++) {
    for (let r = 0; r < 4; r++) {
      let s = 0
      for (let k = 0; k < 4; k++) s += (a[k * 4 + r] ?? 0) * (b[c * 4 + k] ?? 0)
      o[c * 4 + r] = s
    }
  }
  return o
}

/** General 4x4 inverse (cofactor expansion). Returns identity if singular. */
export function mat4Invert(m: Mat4): Mat4 {
  const g = (i: number): number => m[i] ?? 0
  const a00 = g(0), a01 = g(1), a02 = g(2), a03 = g(3)
  const a10 = g(4), a11 = g(5), a12 = g(6), a13 = g(7)
  const a20 = g(8), a21 = g(9), a22 = g(10), a23 = g(11)
  const a30 = g(12), a31 = g(13), a32 = g(14), a33 = g(15)
  const b00 = a00 * a11 - a01 * a10
  const b01 = a00 * a12 - a02 * a10
  const b02 = a00 * a13 - a03 * a10
  const b03 = a01 * a12 - a02 * a11
  const b04 = a01 * a13 - a03 * a11
  const b05 = a02 * a13 - a03 * a12
  const b06 = a20 * a31 - a21 * a30
  const b07 = a20 * a32 - a22 * a30
  const b08 = a20 * a33 - a23 * a30
  const b09 = a21 * a32 - a22 * a31
  const b10 = a21 * a33 - a23 * a31
  const b11 = a22 * a33 - a23 * a32
  const det = b00 * b11 - b01 * b10 + b02 * b09 + b03 * b08 - b04 * b07 + b05 * b06
  if (det === 0) return mat4Identity()
  const d = 1 / det
  return new Float32Array([
    (a11 * b11 - a12 * b10 + a13 * b09) * d,
    (a02 * b10 - a01 * b11 - a03 * b09) * d,
    (a31 * b05 - a32 * b04 + a33 * b03) * d,
    (a22 * b04 - a21 * b05 - a23 * b03) * d,
    (a12 * b08 - a10 * b11 - a13 * b07) * d,
    (a00 * b11 - a02 * b08 + a03 * b07) * d,
    (a32 * b02 - a30 * b05 - a33 * b01) * d,
    (a20 * b05 - a22 * b02 + a23 * b01) * d,
    (a10 * b10 - a11 * b08 + a13 * b06) * d,
    (a01 * b08 - a00 * b10 - a03 * b06) * d,
    (a30 * b04 - a31 * b02 + a33 * b00) * d,
    (a21 * b02 - a20 * b04 - a23 * b00) * d,
    (a11 * b07 - a10 * b09 - a12 * b06) * d,
    (a00 * b09 - a01 * b07 + a02 * b06) * d,
    (a31 * b01 - a30 * b03 - a32 * b00) * d,
    (a20 * b03 - a21 * b01 + a22 * b00) * d,
  ])
}
