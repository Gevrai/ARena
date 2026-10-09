import type { GrayImage } from '../src/cv/image'
import { buildQrGrid, sampleMarker } from '../src/marker/layout'
import type { Mat3 } from '../src/math/quat'
import { vec3Cross, vec3Dot, vec3Normalize } from '../src/math/vec3'
import type { Vec3 } from '../src/math/vec3'
import { intrinsicsFromSize } from '../src/pose/intrinsics'
import type { Intrinsics } from '../src/pose/intrinsics'

/**
 * Pixel convention (the detector's corner tests compare against this):
 * pixel (i,j) covers [i,i+1) x [j,j+1) and its centre is at (i+0.5, j+0.5).
 * Projected coordinates are continuous in the same system, with K.cx = width/2
 * and K.cy = height/2, i.e. x_px = fx * X/Z + cx.
 */

/** OpenCV camera-from-marker. Marker frame: metres, origin at centre, x right, y down, z into the card. */
export interface SynthPose {
  R: Mat3 // row-major 3x3
  t: Vec3
}

export interface SynthOptions {
  width?: number
  height?: number
  hfovDeg?: number
  markerSizeM?: number
  background?: number | ((x: number, y: number) => number)
  noise?: number // gaussian sigma in gray levels
  blurPx?: number // box blur radius in pixels
  supersample?: number
  clutter?: Array<{ corners: [number, number][]; value: number }>
}

const DARK = 20
const WHITE = 235
const DEFAULT_BACKGROUND = 200

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

const g = (m: ArrayLike<number>, i: number): number => m[i] ?? 0

export function renderSynthetic(
  pose: SynthPose,
  opts: SynthOptions = {},
): { image: GrayImage; K: Intrinsics; cornersPx: [number, number][] } {
  const width = opts.width ?? 640
  const height = opts.height ?? 480
  const S = opts.markerSizeM ?? 0.05
  const ss = Math.max(1, Math.floor(opts.supersample ?? 3))
  const K = intrinsicsFromSize(width, height, opts.hfovDeg ?? 65)
  const grid = buildQrGrid()
  const { R, t } = pose

  // Marker-frame plane z=0. Ray in camera: d = ((x-cx)/fx, (y-cy)/fy, 1). Marker point p = R^T (s d - t).
  // p_z = 0 => s = (R^T t)_z / (R^T d)_z. Precompute rows of R^T (= columns of R).
  const rt = (r: number, c: number): number => g(R, c * 3 + r) // R^T[r][c]
  const tm: Vec3 = [
    rt(0, 0) * t[0] + rt(0, 1) * t[1] + rt(0, 2) * t[2],
    rt(1, 0) * t[0] + rt(1, 1) * t[1] + rt(1, 2) * t[2],
    rt(2, 0) * t[0] + rt(2, 1) * t[1] + rt(2, 2) * t[2],
  ]
  const bg = opts.background ?? DEFAULT_BACKGROUND

  // Precompute per-column / per-row direction terms (subsample coords).
  const nx = width * ss
  const ny = height * ss
  const dx = new Float64Array(nx)
  const dy = new Float64Array(ny)
  for (let i = 0; i < nx; i++) dx[i] = ((i + 0.5) / ss - K.cx) / K.fx
  for (let j = 0; j < ny; j++) dy[j] = ((j + 0.5) / ss - K.cy) / K.fy

  const f = new Float32Array(width * height)
  const inv = 1 / (ss * ss)
  for (let y = 0; y < height; y++) {
    for (let x = 0; x < width; x++) {
      const bgv = typeof bg === 'number' ? bg : bg(x, y)
      let acc = 0
      for (let b = 0; b < ss; b++) {
        const ey = dy[y * ss + b] ?? 0
        for (let a = 0; a < ss; a++) {
          const ex = dx[x * ss + a] ?? 0
          // R^T d components
          const px = rt(0, 0) * ex + rt(0, 1) * ey + rt(0, 2)
          const py = rt(1, 0) * ex + rt(1, 1) * ey + rt(1, 2)
          const pz = rt(2, 0) * ex + rt(2, 1) * ey + rt(2, 2)
          let val = bgv
          if (Math.abs(pz) > 1e-12) {
            const s = tm[2] / pz
            if (s > 0) {
              const u = (s * px - tm[0]) / S + 0.5
              const v = (s * py - tm[1]) / S + 0.5
              if (u >= 0 && u <= 1 && v >= 0 && v <= 1) val = sampleMarker(grid, u, v) ? DARK : WHITE
            }
          }
          acc += val
        }
      }
      f[y * width + x] = acc * inv
    }
  }

  // Clutter quads (pixel-centre convention, even-odd polygon fill).
  for (const c of opts.clutter ?? []) {
    const pts = c.corners
    let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity
    for (const [px, py] of pts) {
      minX = Math.min(minX, px); maxX = Math.max(maxX, px)
      minY = Math.min(minY, py); maxY = Math.max(maxY, py)
    }
    for (let y = Math.max(0, Math.floor(minY)); y <= Math.min(height - 1, Math.floor(maxY)); y++) {
      for (let x = Math.max(0, Math.floor(minX)); x <= Math.min(width - 1, Math.floor(maxX)); x++) {
        const cx = x + 0.5, cy = y + 0.5
        let inside = false
        for (let i = 0, j = pts.length - 1; i < pts.length; j = i++) {
          const [xi, yi] = pts[i] as [number, number]
          const [xj, yj] = pts[j] as [number, number]
          if (yi > cy !== yj > cy && cx < ((xj - xi) * (cy - yi)) / (yj - yi) + xi) inside = !inside
        }
        if (inside) f[y * width + x] = c.value
      }
    }
  }

  // Box blur (separable, clamped edges).
  const r = Math.round(opts.blurPx ?? 0)
  let cur = f
  if (r > 0) {
    const tmp = new Float32Array(width * height)
    const n = 2 * r + 1
    for (let y = 0; y < height; y++) {
      for (let x = 0; x < width; x++) {
        let s = 0
        for (let k = -r; k <= r; k++) s += cur[y * width + Math.min(width - 1, Math.max(0, x + k))] ?? 0
        tmp[y * width + x] = s / n
      }
    }
    const out = new Float32Array(width * height)
    for (let y = 0; y < height; y++) {
      for (let x = 0; x < width; x++) {
        let s = 0
        for (let k = -r; k <= r; k++) s += tmp[Math.min(height - 1, Math.max(0, y + k)) * width + x] ?? 0
        out[y * width + x] = s / n
      }
    }
    cur = out
  }

  // Gaussian noise (seeded, Box-Muller) and quantisation.
  const rand = mulberry32(12345)
  const sigma = opts.noise ?? 0
  const data = new Uint8Array(width * height)
  for (let i = 0; i < data.length; i++) {
    let v = cur[i] ?? 0
    if (sigma > 0) {
      const u1 = Math.max(rand(), 1e-12)
      v += sigma * Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * rand())
    }
    data[i] = Math.max(0, Math.min(255, Math.round(v)))
  }

  const cornersPx: [number, number][] = [
    [-0.5, -0.5],
    [0.5, -0.5],
    [0.5, 0.5],
    [-0.5, 0.5],
  ].map(([mx, my]) => {
    const x = (mx ?? 0) * S
    const y = (my ?? 0) * S
    const X = g(R, 0) * x + g(R, 1) * y + t[0]
    const Y = g(R, 3) * x + g(R, 4) * y + t[1]
    const Z = g(R, 6) * x + g(R, 7) * y + t[2]
    return [K.fx * (X / Z) + K.cx, K.fy * (Y / Z) + K.cy] as [number, number]
  })

  return { image: { width, height, data }, K, cornersPx }
}

/**
 * Build a camera-from-marker pose (OpenCV: x right, y down, z forward).
 *  - distanceM: camera centre to marker centre; ||t|| === distanceM exactly, also with an offset.
 *  - tiltDeg: angle between the optical axis and the marker normal (0 = head-on).
 *  - yawDeg: azimuth of the viewing direction around the marker normal. The camera centre sits at
 *    marker-frame azimuth yaw, measured from +x towards +y: yaw 0 = camera displaced towards the
 *    marker's right, so with tilt > 0 the RIGHT edge is nearer (longer in the image) and the left
 *    edge shorter; yaw 90 = displaced towards the bottom edge, so the BOTTOM edge is longer and
 *    the top edge shorter; yaw 180 / 270 mirror these.
 *  - rollDeg: rotation of the camera about its optical axis. Positive roll rotates the marker
 *    counter-clockwise in the image (e.g. roll +90 moves marker TL to the image bottom-left).
 *  - offsetPx: shift of the marker centre in the image relative to the image centre. Implemented
 *    by translating the marker in camera space along the pixel ray, then rescaling t to norm
 *    distanceM, so the centre lands exactly at (cx+dx, cy+dy). The intrinsics used for this are
 *    intrinsicsFromSize(width, height, hfovDeg); pass the same values as renderSynthetic's options.
 * At tilt=yaw=roll=0: R = I, t = (0,0,d); marker top at image top, left at left.
 */
export function lookAtPose(opts: {
  distanceM: number
  tiltDeg: number
  yawDeg: number
  rollDeg: number
  offsetPx?: [number, number]
  width?: number
  height?: number
  hfovDeg?: number
}): SynthPose {
  const rad = Math.PI / 180
  const tilt = opts.tiltDeg * rad
  const yaw = opts.yawDeg * rad
  const roll = opts.rollDeg * rad
  // Camera centre in marker coords sits on the viewer side (z < 0); optical axis f points at the centre.
  const f: Vec3 = [-Math.sin(tilt) * Math.cos(yaw), -Math.sin(tilt) * Math.sin(yaw), Math.cos(tilt)]
  // Camera y (down) = marker y (down) projected perpendicular to f; x = y x z.
  const ey: Vec3 = [0, 1, 0]
  const k = vec3Dot(ey, f)
  const yc = vec3Normalize([ey[0] - k * f[0], ey[1] - k * f[1], ey[2] - k * f[2]])
  const xc = vec3Cross(yc, f)
  const c = Math.cos(roll)
  const s = Math.sin(roll)
  const xr: Vec3 = [c * xc[0] + s * yc[0], c * xc[1] + s * yc[1], c * xc[2] + s * yc[2]]
  const yr: Vec3 = [-s * xc[0] + c * yc[0], -s * xc[1] + c * yc[1], -s * xc[2] + c * yc[2]]
  const R = new Float64Array([...xr, ...yr, ...f])
  const d = opts.distanceM
  const K = intrinsicsFromSize(opts.width ?? 640, opts.height ?? 480, opts.hfovDeg ?? 65)
  const [ox, oy] = opts.offsetPx ?? [0, 0]
  // Direction of the pixel ray through the desired centre location; t = d * unit(ray).
  const a = ox / K.fx
  const b = oy / K.fy
  const n = Math.sqrt(a * a + b * b + 1)
  return { R, t: [(d * a) / n, (d * b) / n, d / n] }
}
