import { findContours } from '../cv/contours'
import { applyHomography, homographyFromQuad } from '../cv/homography'
import type { GrayImage } from '../cv/image'
import { approxPolyDP, isContourConvex, perimeter } from '../cv/poly'
import { adaptiveThreshold } from '../cv/threshold'
import { FRAME_THICKNESS, QR_SIZE, QUIET_ZONE, finderCentres } from '../marker/layout'
import type { MarkerGrid } from '../marker/layout'
import type { Mat3 } from '../math/quat'
import { refineCorners, sampleGray } from './refine'

export type Quad = [[number, number], [number, number], [number, number], [number, number]]

export interface Detection {
  /** Image px (pixel i spans [i,i+1)), order = marker TL, TR, BR, BL (upright marker). */
  corners: Quad
  /** 0..1 finder confidence. */
  score: number
  areaPx: number
}

export interface DetectOptions {
  /** Adaptive-threshold radius (default max(7, width/80)). */
  thresholdRadius?: number
  /** Adaptive-threshold offset in gray levels (default 10). */
  thresholdOffset?: number
  /** Minimum contour perimeter in px (default 120). */
  minPerimeterPx?: number
}

/** Reusable buffers between frames. */
export interface DetectScratch {
  binary?: GrayImage
}

const MAX_CANDIDATES = 12
const MIN_EDGE_PX = 10
const MIN_SCORE = 0.75
const UNIT: [number, number][] = [
  [0, 0],
  [1, 0],
  [1, 1],
  [0, 1],
]

function signedArea2(q: [number, number][]): number {
  let a = 0
  for (let i = 0; i < q.length; i++) {
    const p = q[i] as [number, number]
    const n = q[(i + 1) % q.length] as [number, number]
    a += p[0] * n[1] - n[0] * p[1]
  }
  return a
}

interface BandStats {
  bandFrac: number
  quietFrac: number
  thr: number
}

/** Frame band and quiet-zone checks with a locally derived threshold (rotation invariant). */
function checkBand(img: GrayImage, H: Mat3): BandStats | null {
  const tb = FRAME_THICKNESS / 2
  const tq = FRAME_THICKNESS + QUIET_ZONE / 2
  const band: number[] = []
  const quiet: number[] = []
  const at = (u: number, v: number): number => {
    const [x, y] = applyHomography(H, u, v)
    return sampleGray(img, x, y)
  }
  for (let k = 0; k < 4; k++) {
    const s = 0.2 + 0.2 * k
    for (const [t, dst] of [
      [tb, band],
      [tq, quiet],
    ] as [number, number[]][]) {
      dst.push(at(s, t), at(1 - t, s), at(1 - s, 1 - t), at(t, 1 - s))
    }
  }
  const mean = (a: number[]): number => a.reduce((p, c) => p + c, 0) / a.length
  const mb = mean(band)
  const mq = mean(quiet)
  if (mq - mb < 30) return null
  const thr = (mb + mq) / 2
  return {
    bandFrac: band.filter((v) => v < thr).length / band.length,
    quietFrac: quiet.filter((v) => v > thr).length / quiet.length,
    thr,
  }
}

/** Ideal finder with a light surround: dark core (<=1), light ring at 2, dark ring at 3, light elsewhere. */
function idealFinder(dx: number, dy: number): number {
  const r = Math.max(Math.abs(dx), Math.abs(dy))
  return r === 0 || r === 1 || r === 3 ? -1 : 1
}

/** Template sample positions (module units), 9x9 around the finder centre. */
const TEMPLATE_POS: [number, number][] = []
for (let dy = -4; dy <= 4; dy++) for (let dx = -4; dx <= 4; dx++) TEMPLATE_POS.push([dx, dy])

/**
 * The ideal finder blurred by a gaussian of sigma (in modules), sampled at TEMPLATE_POS and zero-meaned.
 * Several sigmas make the match tolerant to optical blur / small marker sizes, where a module is about
 * as small as the blur kernel and per-ring binary tests collapse.
 */
const TEMPLATE_SIGMAS = [0, 0.7, 1.4]
const TEMPLATES: { t: Float64Array; norm: number }[] = TEMPLATE_SIGMAS.map((sigma) => {
  const t = new Float64Array(TEMPLATE_POS.length)
  TEMPLATE_POS.forEach(([x, y], i) => {
    if (sigma === 0) {
      t[i] = idealFinder(x, y)
      return
    }
    let acc = 0
    let wsum = 0
    for (let oy = -8; oy <= 8; oy++) {
      for (let ox = -8; ox <= 8; ox++) {
        const w = Math.exp(-(ox * ox + oy * oy) / (2 * sigma * sigma))
        acc += w * idealFinder(x + ox, y + oy)
        wsum += w
      }
    }
    t[i] = acc / wsum
  })
  let m = 0
  for (const v of t) m += v
  m /= t.length
  let n = 0
  for (let i = 0; i < t.length; i++) {
    t[i] = (t[i] ?? 0) - m
    n += (t[i] ?? 0) ** 2
  }
  return { t, norm: Math.sqrt(n) }
})

/**
 * Best normalised cross-correlation (-1..1) between the image around (cu,cv), sampled at module centres
 * through the homography, and the ideal finder pattern (centre dark, ring at 2 modules light, ring at 3
 * modules dark: a finder is 7x7 modules, so these are its light ring and outer dark ring; light beyond).
 * NCC is invariant to local gain/offset, so no fixed threshold is needed.
 */
function finderness(img: GrayImage, H: Mat3, moduleUV: number, cu: number, cv: number): number {
  const n = TEMPLATE_POS.length
  const vals = new Float64Array(n)
  let mean = 0
  for (let i = 0; i < n; i++) {
    const p = TEMPLATE_POS[i] as [number, number]
    const [x, y] = applyHomography(H, cu + p[0] * moduleUV, cv + p[1] * moduleUV)
    const v = sampleGray(img, x, y)
    vals[i] = v
    mean += v
  }
  mean /= n
  let sxx = 0
  for (let i = 0; i < n; i++) {
    const a = (vals[i] ?? 0) - mean
    vals[i] = a
    sxx += a * a
  }
  if (sxx < 1e-9) return 0
  let best = -1
  for (const { t, norm } of TEMPLATES) {
    let sxy = 0
    for (let i = 0; i < n; i++) sxy += (vals[i] ?? 0) * (t[i] ?? 0)
    best = Math.max(best, sxy / (Math.sqrt(sxx) * norm))
  }
  return best
}

interface Scored {
  rot: number
  score: number
  margin: number
}

const QR_ORIGIN = FRAME_THICKNESS + QUIET_ZONE
const GRID_SIGMAS = [0, 0.7, 1.4]
const gridTemplates = new WeakMap<MarkerGrid, { t: Float64Array; norm: number }[]>()

/** Zero-mean ideal QR (+-1, dark = -1) blurred by several gaussians (in modules), one template per sigma. */
function templatesFor(grid: MarkerGrid): { t: Float64Array; norm: number }[] {
  const cached = gridTemplates.get(grid)
  if (cached) return cached
  const n = grid.size
  const dark = (i: number, j: number): number =>
    i < 0 || j < 0 || i >= n || j >= n ? 1 : grid.modules[j]?.[i] ? -1 : 1
  const out = GRID_SIGMAS.map((sigma) => {
    const t = new Float64Array(n * n)
    const R = Math.ceil(sigma * 2.5)
    for (let j = 0; j < n; j++) {
      for (let i = 0; i < n; i++) {
        if (sigma === 0) {
          t[j * n + i] = dark(i, j)
          continue
        }
        let acc = 0
        let ws = 0
        for (let oy = -R; oy <= R; oy++) {
          for (let ox = -R; ox <= R; ox++) {
            const w = Math.exp(-(ox * ox + oy * oy) / (2 * sigma * sigma))
            acc += w * dark(i + ox, j + oy)
            ws += w
          }
        }
        t[j * n + i] = acc / ws
      }
    }
    let m = 0
    for (const v of t) m += v
    m /= t.length
    let nn = 0
    for (let k = 0; k < t.length; k++) {
      t[k] = (t[k] ?? 0) - m
      nn += (t[k] ?? 0) ** 2
    }
    return { t, norm: Math.sqrt(nn) }
  })
  gridTemplates.set(grid, out)
  return out
}

/** NCC of the whole sampled QR (every module centre, through H) with the expected grid. */
function qrCorrelation(img: GrayImage, H: Mat3, grid: MarkerGrid): number {
  const n = grid.size
  const vals = new Float64Array(n * n)
  const mod = QR_SIZE / n
  let mean = 0
  for (let j = 0; j < n; j++) {
    for (let i = 0; i < n; i++) {
      const [x, y] = applyHomography(H, QR_ORIGIN + (i + 0.5) * mod, QR_ORIGIN + (j + 0.5) * mod)
      const v = sampleGray(img, x, y)
      vals[j * n + i] = v
      mean += v
    }
  }
  mean /= vals.length
  let sxx = 0
  for (let k = 0; k < vals.length; k++) {
    const a = (vals[k] ?? 0) - mean
    vals[k] = a
    sxx += a * a
  }
  if (sxx < 1e-9) return 0
  let best = -1
  for (const { t, norm } of templatesFor(grid)) {
    let sxy = 0
    for (let k = 0; k < vals.length; k++) sxy += (vals[k] ?? 0) * (t[k] ?? 0)
    best = Math.max(best, sxy / (Math.sqrt(sxx) * norm))
  }
  return best
}

/** Required lead of the best rotation over the runner-up (whole-QR NCC) before orientation is trusted. */
const ORIENTATION_GAP = 0.15

/**
 * Orientation from 4 cyclic rotations. Each is scored by the whole-QR correlation (hundreds of samples);
 * the winner must also have 3 finder-like corners and a less finder-like 4th, and must beat the
 * runner-up by ORIENTATION_GAP. Otherwise null (fail safe, never guess). score = orientation confidence.
 */
function bestRotation(img: GrayImage, quad: [number, number][], grid: MarkerGrid): Scored | null {
  const fc = finderCentres(grid)
  const moduleUV = QR_SIZE / grid.size
  const q: number[] = []
  const Hs: (Mat3 | null)[] = []
  for (let rot = 0; rot < 4; rot++) {
    const dst = [0, 1, 2, 3].map((i) => quad[(i + rot) % 4] as [number, number])
    const H = homographyFromQuad(UNIT, dst)
    Hs.push(H)
    q.push(H ? qrCorrelation(img, H, grid) : -1)
  }
  let bi = 0
  for (let i = 1; i < 4; i++) if ((q[i] ?? -1) > (q[bi] ?? -1)) bi = i
  let second = -1
  for (let i = 0; i < 4; i++) if (i !== bi) second = Math.max(second, q[i] ?? -1)
  const gap = (q[bi] ?? -1) - second
  const H = Hs[bi]
  if (!H || gap < ORIENTATION_GAP) return null
  const f = [fc.tl, fc.tr, fc.bl].map(([u, v]) => finderness(img, H, moduleUV, u, v))
  const fe = finderness(img, H, moduleUV, fc.empty[0], fc.empty[1])
  const margin = f.reduce((p, c) => p + c, 0) / f.length - fe
  if (Math.min(...f) < 0.4 || margin < 0.08) return null
  return { rot: bi, score: Math.min(1, gap / (2 * ORIENTATION_GAP)), margin }
}

function insideImage(img: GrayImage, q: [number, number][]): boolean {
  for (const [x, y] of q)
    if (!(x > 1 && y > 1 && x < img.width - 1 && y < img.height - 1)) return false
  return true
}

export function detectFramedQr(
  img: GrayImage,
  grid: MarkerGrid,
  opts: DetectOptions = {},
  scratch: DetectScratch = {},
): Detection | null {
  const radius = opts.thresholdRadius ?? Math.max(7, Math.round(img.width / 80))
  const offset = opts.thresholdOffset ?? 10
  const minPer = opts.minPerimeterPx ?? 120
  scratch.binary = adaptiveThreshold(img, radius, offset, scratch.binary)
  const contours = findContours(scratch.binary)

  const cands: { quad: [number, number][]; area: number }[] = []
  for (const c of contours) {
    if (c.length < 8) continue
    const per = perimeter(c)
    if (per < minPer) continue
    const poly = approxPolyDP(c, 0.03 * per)
    if (poly.length !== 4 || !isContourConvex(poly)) continue
    // Contour points are pixel indices; +0.5 moves to continuous pixel-centre coordinates.
    let quad = poly.map((p) => [p.x + 0.5, p.y + 0.5] as [number, number])
    if (signedArea2(quad) < 0) quad = quad.reverse()
    let minEdge = Infinity
    for (let i = 0; i < 4; i++) {
      const a = quad[i] as [number, number]
      const b = quad[(i + 1) % 4] as [number, number]
      minEdge = Math.min(minEdge, Math.hypot(a[0] - b[0], a[1] - b[1]))
    }
    if (minEdge < MIN_EDGE_PX || !insideImage(img, quad)) continue
    cands.push({ quad, area: signedArea2(quad) / 2 })
  }
  cands.sort((a, b) => b.area - a.area)

  let best: Detection | null = null
  for (const cand of cands.slice(0, MAX_CANDIDATES)) {
    const H0 = homographyFromQuad(UNIT, cand.quad)
    if (!H0) continue
    const st = checkBand(img, H0)
    if (!st || st.bandFrac < 0.9 || st.quietFrac < 0.8) continue
    const refined = refineCorners(img, cand.quad)
    if (refined === cand.quad || !insideImage(img, refined)) continue
    // Re-derive the threshold on the refined quad, then find orientation.
    const H1 = homographyFromQuad(UNIT, refined)
    const st1 = H1 ? checkBand(img, H1) : null
    if (!st1) continue
    const rot = bestRotation(img, refined, grid)
    if (!rot) continue
    const score = (rot.score + st1.bandFrac + st1.quietFrac) / 3
    if (score < MIN_SCORE) continue
    const c = [0, 1, 2, 3].map((i) => refined[(i + rot.rot) % 4] as [number, number])
    const corners = c as Quad
    const areaPx = Math.abs(signedArea2(c)) / 2
    if (!best || score > best.score || (score === best.score && areaPx > best.areaPx)) {
      best = { corners, score, areaPx }
    }
  }
  return best
}
