import { homographyFromQuad, solve } from '../cv/homography'
import type { Detection } from '../detect/framedQr'
import { mat4FromRotationTranslation } from '../math/mat4'
import type { Mat4 } from '../math/mat4'
import { quatFromMat3 } from '../math/quat'
import type { Mat3, Quat } from '../math/quat'
import type { Vec3 } from '../math/vec3'
import type { Intrinsics } from './intrinsics'

/** OpenCV camera-from-marker (marker frame: x right, y down along the card, z into the card). */
export interface CvPose {
  R: Mat3
  t: Vec3
  reprojErrorPx: number
}

const MAX_REPROJ_PX = 3

const g = (m: ArrayLike<number>, i: number): number => m[i] ?? 0

function inv3(m: ArrayLike<number>): Float64Array | null {
  const a = g(m, 0),
    b = g(m, 1),
    c = g(m, 2)
  const d = g(m, 3),
    e = g(m, 4),
    f = g(m, 5)
  const h = g(m, 6),
    i = g(m, 7),
    j = g(m, 8)
  const det = a * (e * j - f * i) - b * (d * j - f * h) + c * (d * i - e * h)
  if (!Number.isFinite(det) || Math.abs(det) < 1e-12) return null
  const s = 1 / det
  return new Float64Array([
    (e * j - f * i) * s,
    (c * i - b * j) * s,
    (b * f - c * e) * s,
    (f * h - d * j) * s,
    (a * j - c * h) * s,
    (c * d - a * f) * s,
    (d * i - e * h) * s,
    (b * h - a * i) * s,
    (a * e - b * d) * s,
  ])
}

/** Polar decomposition: nearest rotation via R <- (R + R^-T)/2. */
function orthonormalize(R: Float64Array, iterations = 3): Float64Array | null {
  let cur = R
  for (let k = 0; k < iterations; k++) {
    const inv = inv3(cur)
    if (!inv) return null
    const next = new Float64Array(9)
    for (let r = 0; r < 3; r++)
      for (let c = 0; c < 3; c++) next[r * 3 + c] = 0.5 * (g(cur, r * 3 + c) + g(inv, c * 3 + r))
    cur = next
  }
  return cur
}

/** Rotation matrix for rotation vector w (Rodrigues). */
function expSo3(w: Vec3): Float64Array {
  const th = Math.hypot(w[0], w[1], w[2])
  const [x, y, z] = th < 1e-12 ? [w[0], w[1], w[2]] : [w[0] / th, w[1] / th, w[2] / th]
  const s = th < 1e-12 ? th : Math.sin(th)
  const c = th < 1e-12 ? 1 - (th * th) / 2 : Math.cos(th)
  const C = 1 - c
  return new Float64Array([
    c + x * x * C,
    x * y * C - z * s,
    x * z * C + y * s,
    y * x * C + z * s,
    c + y * y * C,
    y * z * C - x * s,
    z * x * C - y * s,
    z * y * C + x * s,
    c + z * z * C,
  ])
}

function mul3(A: ArrayLike<number>, B: ArrayLike<number>): Float64Array {
  const o = new Float64Array(9)
  for (let r = 0; r < 3; r++)
    for (let c = 0; c < 3; c++) {
      let s = 0
      for (let k = 0; k < 3; k++) s += g(A, r * 3 + k) * g(B, k * 3 + c)
      o[r * 3 + c] = s
    }
  return o
}

/** Residuals (pixels) of the 4 marker corners projected with (R, t). */
function residuals(
  R: Float64Array,
  t: Vec3,
  obj: [number, number][],
  px: [number, number][],
  K: Intrinsics,
): number[] {
  const out: number[] = []
  for (let i = 0; i < 4; i++) {
    const [X, Y] = obj[i] as [number, number]
    const xc = g(R, 0) * X + g(R, 1) * Y + t[0]
    const yc = g(R, 3) * X + g(R, 4) * Y + t[1]
    const zc = g(R, 6) * X + g(R, 7) * Y + t[2]
    const [u, v] = px[i] as [number, number]
    out.push(K.fx * (xc / zc) + K.cx - u, K.fy * (yc / zc) + K.cy - v)
  }
  return out
}

const rms = (r: number[]): number => Math.sqrt(r.reduce((s, v) => s + v * v, 0) / (r.length / 2))

export function estimatePose(
  corners: Detection['corners'],
  K: Intrinsics,
  markerSizeM: number,
): CvPose | null {
  const h = markerSizeM / 2
  const obj: [number, number][] = [
    [-h, -h],
    [h, -h],
    [h, h],
    [-h, h],
  ]
  const px = corners.map(([x, y]) => [x, y] as [number, number])
  if (px.some(([x, y]) => !Number.isFinite(x) || !Number.isFinite(y))) return null
  if (!(K.fx > 0) || !(K.fy > 0) || !(markerSizeM > 0)) return null

  const norm = px.map(([x, y]) => [(x - K.cx) / K.fx, (y - K.cy) / K.fy] as [number, number])
  const H = homographyFromQuad(obj, norm)
  if (!H) return null

  // Columns of H are h1, h2, h3.
  const h1: Vec3 = [g(H, 0), g(H, 3), g(H, 6)]
  const h2: Vec3 = [g(H, 1), g(H, 4), g(H, 7)]
  const h3: Vec3 = [g(H, 2), g(H, 5), g(H, 8)]
  const n1 = Math.hypot(...h1)
  const n2 = Math.hypot(...h2)
  if (!(n1 + n2 > 1e-12)) return null
  const lam = 2 / (n1 + n2)
  const r1 = h1.map((v) => v * lam) as Vec3
  const r2 = h2.map((v) => v * lam) as Vec3
  let t = h3.map((v) => v * lam) as Vec3
  let r1v = r1
  let r2v = r2
  if (t[2] < 0) {
    r1v = r1.map((v) => -v) as Vec3
    r2v = r2.map((v) => -v) as Vec3
    t = t.map((v) => -v) as Vec3
  }
  const r3: Vec3 = [
    r1v[1] * r2v[2] - r1v[2] * r2v[1],
    r1v[2] * r2v[0] - r1v[0] * r2v[2],
    r1v[0] * r2v[1] - r1v[1] * r2v[0],
  ]
  // Columns r1, r2, r3 -> row-major matrix.
  const R0 = new Float64Array([r1v[0], r2v[0], r3[0], r1v[1], r2v[1], r3[1], r1v[2], r2v[2], r3[2]])
  let R = orthonormalize(R0)
  if (!R || !(t[2] > 0)) return null

  // Gauss-Newton on [omega (left-multiplied rotation increment), t].
  let err = rms(residuals(R, t, obj, px, K))
  for (let it = 0; it < 5; it++) {
    const r0 = residuals(R, t, obj, px, K)
    const J: number[][] = Array.from({ length: 6 }, () => [])
    const eps = 1e-6
    for (let p = 0; p < 6; p++) {
      const w: Vec3 = [0, 0, 0]
      const tt: Vec3 = [t[0], t[1], t[2]]
      if (p < 3) w[p] = eps
      else tt[p - 3] = (tt[p - 3] ?? 0) + eps * 0.01
      const step = p < 3 ? eps : eps * 0.01
      const rp = residuals(p < 3 ? mul3(expSo3(w), R) : R, tt, obj, px, K)
      J[p] = rp.map((v, i) => (v - (r0[i] ?? 0)) / step)
    }
    const A: number[] = new Array<number>(36).fill(0)
    const b: number[] = new Array<number>(6).fill(0)
    for (let a = 0; a < 6; a++) {
      for (let c = 0; c < 6; c++) {
        let s = 0
        for (let k = 0; k < 8; k++) s += (J[a]?.[k] ?? 0) * (J[c]?.[k] ?? 0)
        A[a * 6 + c] = s + (a === c ? 1e-9 : 0)
      }
      let s = 0
      for (let k = 0; k < 8; k++) s -= (J[a]?.[k] ?? 0) * (r0[k] ?? 0)
      b[a] = s
    }
    const d = solve(A, b, 6)
    if (!d || d.some((v) => !Number.isFinite(v))) break
    const Rn = mul3(expSo3([d[0] ?? 0, d[1] ?? 0, d[2] ?? 0]), R)
    const tn: Vec3 = [t[0] + (d[3] ?? 0), t[1] + (d[4] ?? 0), t[2] + (d[5] ?? 0)]
    const en = rms(residuals(Rn, tn, obj, px, K))
    if (!(en <= err)) break
    R = Rn
    t = tn
    err = en
  }

  R = orthonormalize(R, 1)
  if (!R) return null
  if (
    R.some((v) => !Number.isFinite(v)) ||
    t.some((v) => !Number.isFinite(v)) ||
    !Number.isFinite(err)
  )
    return null
  if (t[2] <= 0 || err > MAX_REPROJ_PX) return null
  return { R, t, reprojErrorPx: err }
}

/**
 * Convert a CV camera-from-marker pose to world-from-camera in the game conventions.
 *
 * Frames:
 *  - marker-cv: x right, y down along the card, z into the card.
 *  - world: origin at marker centre, +X card right, +Y up out of the card, +Z card bottom.
 *    world-from-marker-cv W: x_w = x_cv, y_w = -z_cv, z_w = y_cv.
 *  - camera-cv: x right, y down, z forward. camera-three: x right, y up, looks down -Z.
 *    camera-cv-from-three C = diag(1,-1,-1) (flip y and z; self-inverse).
 *
 * p_cam_cv = R p_marker_cv + t, so p_marker_cv = R^T (p_cam_cv - t), and
 * world_from_camera_three = W * [R^T | -R^T t] * C.
 * Check (R = I, t = (0,0,0.4)): position = W(-R^T t) = W(0,0,-0.4) = (0,0.4,0); camera -Z
 * (= cv +z) -> W(0,0,1) = (0,-1,0) (looks down); camera +Y (= cv -y) -> W(0,-1,0) = (0,0,-1),
 * i.e. image-up points to the card's top edge (world -Z).
 */
export function poseToWorldFromCamera(p: CvPose): {
  position: Vec3
  quaternion: Quat
  matrix: Mat4
} {
  const R = p.R
  // Rotation of marker-cv-from-camera-cv: R^T (row-major).
  const Rt = new Float64Array([
    g(R, 0),
    g(R, 3),
    g(R, 6),
    g(R, 1),
    g(R, 4),
    g(R, 7),
    g(R, 2),
    g(R, 5),
    g(R, 8),
  ])
  const W = new Float64Array([1, 0, 0, 0, 0, -1, 0, 1, 0])
  const C = new Float64Array([1, 0, 0, 0, -1, 0, 0, 0, -1])
  const Rw = mul3(W, mul3(Rt, C))
  const tm: Vec3 = [
    -(g(Rt, 0) * p.t[0] + g(Rt, 1) * p.t[1] + g(Rt, 2) * p.t[2]),
    -(g(Rt, 3) * p.t[0] + g(Rt, 4) * p.t[1] + g(Rt, 5) * p.t[2]),
    -(g(Rt, 6) * p.t[0] + g(Rt, 7) * p.t[1] + g(Rt, 8) * p.t[2]),
  ]
  const position: Vec3 = [tm[0], -tm[2], tm[1]]
  const quaternion = quatFromMat3(Rw)
  return { position, quaternion, matrix: mat4FromRotationTranslation(quaternion, position) }
}
