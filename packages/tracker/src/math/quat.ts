import type { Vec3 } from './vec3'

/** x, y, z, w */
export type Quat = [number, number, number, number]
/** 3x3 matrix, row-major, internal CV use. */
export type Mat3 = Float64Array

export function quatMultiply(a: Quat, b: Quat): Quat {
  const [ax, ay, az, aw] = a
  const [bx, by, bz, bw] = b
  return [
    aw * bx + ax * bw + ay * bz - az * by,
    aw * by - ax * bz + ay * bw + az * bx,
    aw * bz + ax * by - ay * bx + az * bw,
    aw * bw - ax * bx - ay * by - az * bz,
  ]
}

/** Inverse of a (not necessarily unit) quaternion. */
export function quatInvert(q: Quat): Quat {
  const n = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]
  if (n === 0) return [0, 0, 0, 1]
  return [-q[0] / n, -q[1] / n, -q[2] / n, q[3] / n]
}

export function quatNormalize(q: Quat): Quat {
  const l = Math.hypot(q[0], q[1], q[2], q[3])
  return l === 0 ? [0, 0, 0, 1] : [q[0] / l, q[1] / l, q[2] / l, q[3] / l]
}

export function quatFromAxisAngle(axis: Vec3, angle: number): Quat {
  const l = Math.hypot(axis[0], axis[1], axis[2])
  if (l === 0) return [0, 0, 0, 1]
  const s = Math.sin(angle / 2) / l
  return [axis[0] * s, axis[1] * s, axis[2] * s, Math.cos(angle / 2)]
}

/** Smallest angle in radians between two unit quaternions (rotation difference). */
export function quatAngle(a: Quat, b: Quat): number {
  const d = Math.min(1, Math.abs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]))
  return 2 * Math.acos(d)
}

export function quatSlerp(a: Quat, b: Quat, t: number): Quat {
  let [bx, by, bz, bw] = b
  let d = a[0] * bx + a[1] * by + a[2] * bz + a[3] * bw
  if (d < 0) {
    d = -d
    bx = -bx
    by = -by
    bz = -bz
    bw = -bw
  }
  let s0: number
  let s1: number
  if (d > 0.9995) {
    s0 = 1 - t
    s1 = t
  } else {
    const th = Math.acos(d)
    const sn = Math.sin(th)
    s0 = Math.sin((1 - t) * th) / sn
    s1 = Math.sin(t * th) / sn
  }
  return quatNormalize([
    a[0] * s0 + bx * s1,
    a[1] * s0 + by * s1,
    a[2] * s0 + bz * s1,
    a[3] * s0 + bw * s1,
  ])
}

/** Rotation matrix (row-major) to unit quaternion. */
export function quatFromMat3(m: Mat3): Quat {
  const g = (i: number): number => m[i] ?? 0
  const m00 = g(0),
    m01 = g(1),
    m02 = g(2)
  const m10 = g(3),
    m11 = g(4),
    m12 = g(5)
  const m20 = g(6),
    m21 = g(7),
    m22 = g(8)
  const tr = m00 + m11 + m22
  let q: Quat
  if (tr > 0) {
    const s = Math.sqrt(tr + 1) * 2
    q = [(m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25 * s]
  } else if (m00 > m11 && m00 > m22) {
    const s = Math.sqrt(1 + m00 - m11 - m22) * 2
    q = [0.25 * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s]
  } else if (m11 > m22) {
    const s = Math.sqrt(1 + m11 - m00 - m22) * 2
    q = [(m01 + m10) / s, 0.25 * s, (m12 + m21) / s, (m02 - m20) / s]
  } else {
    const s = Math.sqrt(1 + m22 - m00 - m11) * 2
    q = [(m02 + m20) / s, (m12 + m21) / s, 0.25 * s, (m10 - m01) / s]
  }
  return quatNormalize(q)
}

/** Unit quaternion to rotation matrix (row-major). */
export function quatToMat3(q: Quat): Mat3 {
  const [x, y, z, w] = q
  return new Float64Array([
    1 - 2 * (y * y + z * z),
    2 * (x * y - z * w),
    2 * (x * z + y * w),
    2 * (x * y + z * w),
    1 - 2 * (x * x + z * z),
    2 * (y * z - x * w),
    2 * (x * z - y * w),
    2 * (y * z + x * w),
    1 - 2 * (x * x + y * y),
  ])
}

/**
 * Heading of a world-from-camera rotation about world +Y, radians in (-PI, PI].
 * Angle of the camera's forward vector (-Z) projected on the XZ plane, 0 = looking towards world
 * -Z, positive = counter-clockwise seen from above (+Y), i.e. looking towards -X is +PI/2.
 * When looking (almost) straight up or down the camera's up vector (+Y) is used instead, which
 * is the direction the top of the screen points.
 */
export function cameraYawFromQuat(q: Quat): number {
  const m = quatToMat3(q)
  const g = (i: number): number => m[i] ?? 0
  // columns of the rotation: camera +Y = (m1, m4, m7), camera +Z = (m2, m5, m8); forward = -Z.
  const fx = -g(2)
  const fz = -g(8)
  if (Math.hypot(fx, fz) > 1e-3) return Math.atan2(-fx, -fz)
  // Looking down (forward.y = -m5 < 0): the top of the screen is the heading; looking up: its opposite.
  const sign = g(5) > 0 ? 1 : -1
  const vx = sign * g(1)
  const vz = sign * g(7)
  return Math.atan2(-vx, -vz)
}
