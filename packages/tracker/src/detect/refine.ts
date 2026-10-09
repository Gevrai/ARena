import type { GrayImage } from '../cv/image'

/** Bilinear sample in the continuous pixel convention (pixel i spans [i,i+1), centre i+0.5). */
export function sampleGray(img: GrayImage, x: number, y: number): number {
  const { width: w, height: h, data } = img
  let fx = x - 0.5
  let fy = y - 0.5
  if (fx < 0) fx = 0
  else if (fx > w - 1) fx = w - 1
  if (fy < 0) fy = 0
  else if (fy > h - 1) fy = h - 1
  const x0 = Math.floor(fx)
  const y0 = Math.floor(fy)
  const x1 = x0 + 1 < w ? x0 + 1 : x0
  const y1 = y0 + 1 < h ? y0 + 1 : y0
  const ax = fx - x0
  const ay = fy - y0
  const a = data[y0 * w + x0] ?? 0
  const b = data[y0 * w + x1] ?? 0
  const c = data[y1 * w + x0] ?? 0
  const d = data[y1 * w + x1] ?? 0
  return (a * (1 - ax) + b * ax) * (1 - ay) + (c * (1 - ax) + d * ax) * ay
}

const SAMPLES_PER_EDGE = 16
const SEARCH = 2.5 // px each side of the current edge estimate
const STEP = 0.25
const GRAD_H = 0.5

interface Line {
  px: number
  py: number
  dx: number
  dy: number
}

/** Total-least-squares line through points. */
function fitLine(pts: [number, number][]): Line | null {
  const n = pts.length
  if (n < 3) return null
  let mx = 0
  let my = 0
  for (const [x, y] of pts) {
    mx += x
    my += y
  }
  mx /= n
  my /= n
  let sxx = 0
  let sxy = 0
  let syy = 0
  for (const [x, y] of pts) {
    sxx += (x - mx) * (x - mx)
    sxy += (x - mx) * (y - my)
    syy += (y - my) * (y - my)
  }
  const ang = 0.5 * Math.atan2(2 * sxy, sxx - syy)
  return { px: mx, py: my, dx: Math.cos(ang), dy: Math.sin(ang) }
}

function residual(l: Line, x: number, y: number): number {
  return Math.abs((x - l.px) * l.dy - (y - l.py) * l.dx)
}

/** Subpixel edge position along the outward normal: argmax of the (dark inside -> light outside) gradient. */
function edgeOffset(img: GrayImage, x: number, y: number, nx: number, ny: number): number | null {
  const n = Math.round((2 * SEARCH) / STEP) + 1
  const g = new Array<number>(n)
  let best = -1
  let bestV = 0
  for (let i = 0; i < n; i++) {
    const s = -SEARCH + i * STEP
    const v =
      sampleGray(img, x + (s + GRAD_H) * nx, y + (s + GRAD_H) * ny) -
      sampleGray(img, x + (s - GRAD_H) * nx, y + (s - GRAD_H) * ny)
    g[i] = v
    if (v > bestV) {
      bestV = v
      best = i
    }
  }
  if (best < 0 || bestV < 12) return null
  // Centroid of the positive gradient lobe around the peak.
  let num = 0
  let den = 0
  for (let i = Math.max(0, best - 5); i <= Math.min(n - 1, best + 5); i++) {
    const v = g[i] ?? 0
    if (v <= 0) continue
    num += v * (-SEARCH + i * STEP)
    den += v
  }
  return den > 0 ? num / den : null
}

function refineOnce(img: GrayImage, quad: [number, number][]): [number, number][] | null {
  // Orientation sign so that the normal points outward.
  let area2 = 0
  for (let i = 0; i < 4; i++) {
    const a = quad[i] as [number, number]
    const b = quad[(i + 1) % 4] as [number, number]
    area2 += a[0] * b[1] - b[0] * a[1]
  }
  const sgn = area2 >= 0 ? 1 : -1
  const lines: Line[] = []
  for (let e = 0; e < 4; e++) {
    const a = quad[e] as [number, number]
    const b = quad[(e + 1) % 4] as [number, number]
    const ex = b[0] - a[0]
    const ey = b[1] - a[1]
    const len = Math.hypot(ex, ey)
    if (len < 1e-6) return null
    // Outward normal for clockwise (y-down, positive area) polygons is (ey, -ex).
    const nx = (sgn * ey) / len
    const ny = (-sgn * ex) / len
    let pts: [number, number][] = []
    for (let k = 0; k < SAMPLES_PER_EDGE; k++) {
      const t = 0.12 + (0.76 * (k + 0.5)) / SAMPLES_PER_EDGE
      const x = a[0] + t * ex
      const y = a[1] + t * ey
      const off = edgeOffset(img, x, y, nx, ny)
      if (off !== null) pts.push([x + off * nx, y + off * ny])
    }
    let line = fitLine(pts)
    // Robust pass: drop outliers and refit.
    for (let pass = 0; pass < 2 && line; pass++) {
      const l = line
      const kept = pts.filter(([x, y]) => residual(l, x, y) < 0.6)
      if (kept.length < 5) return null
      pts = kept
      line = fitLine(pts)
    }
    if (!line || pts.length < 5) return null
    lines.push(line)
  }
  const out: [number, number][] = []
  for (let i = 0; i < 4; i++) {
    const l1 = lines[(i + 3) % 4] as Line
    const l2 = lines[i] as Line
    const det = l1.dx * l2.dy - l1.dy * l2.dx
    if (Math.abs(det) < 1e-3) return null
    const t = ((l2.px - l1.px) * l2.dy - (l2.py - l1.py) * l2.dx) / det
    out.push([l1.px + t * l1.dx, l1.py + t * l1.dy])
  }
  return out
}

/**
 * Subpixel quad refinement. `quad` is in continuous pixel coordinates (pixel centre = i+0.5), any winding.
 * For each edge: sample the dark-to-light gradient along short outward normals at several points, fit a
 * line (outlier-trimmed), then intersect adjacent lines. Run twice so the second pass starts on the edge.
 * Returns the input unchanged if a fit fails.
 */
export function refineCorners(img: GrayImage, quad: [number, number][]): [number, number][] {
  let cur = quad
  for (let it = 0; it < 2; it++) {
    const next = refineOnce(img, cur)
    if (!next) return it === 0 ? quad : cur
    cur = next
  }
  return cur
}
