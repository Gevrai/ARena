import { it, expect } from 'vitest'
import { buildQrGrid, sampleMarker, finderCentres, QR_SIZE, FRAME_THICKNESS } from '../src/marker/layout'
import { renderMarkerSvg, renderCardSvg } from '../src/marker/svg'

// Measured with uqr, ecc 'L', for DEFAULT_URL: version 2 (25 modules).
it('QR for default URL is version 2 (25 modules)', () => { expect(buildQrGrid().size).toBe(25) })
it('frame is dark, quiet zone is white', () => {
  const g = buildQrGrid()
  expect(sampleMarker(g, FRAME_THICKNESS / 2, 0.5)).toBe(true)
  expect(sampleMarker(g, FRAME_THICKNESS + 0.03, 0.5)).toBe(false)
  expect(sampleMarker(g, -0.1, 0.5)).toBe(false)
})
it('finder centres are dark, empty corner centre region is not a finder', () => {
  const g = buildQrGrid(); const f = finderCentres(g)
  for (const p of [f.tl, f.tr, f.bl]) expect(sampleMarker(g, p[0], p[1])).toBe(true)
  expect(f.tl[0]).toBeCloseTo(1 - f.tr[0], 5); expect(QR_SIZE).toBeCloseTo(0.6, 5)
})
it('renderMarkerSvg contains a viewBox in mm', () => { expect(renderMarkerSvg(buildQrGrid(), 50)).toMatch(/viewBox="0 0 50 50"/) })
it('renderCardSvg is 85x55 mm', () => {
  const s = renderCardSvg(buildQrGrid())
  expect(s).toMatch(/viewBox="0 0 85 55"/); expect(s).toContain('ARena')
})

function isFinderAt(g: ReturnType<typeof buildQrGrid>, c: [number, number]): boolean {
  const mod = QR_SIZE / g.size
  const at = (dx: number, dy: number): boolean => sampleMarker(g, c[0] + dx * mod, c[1] + dy * mod)
  const dirs: [number, number][] = [[1, 0], [-1, 0], [0, 1], [0, -1]]
  return at(0, 0) && dirs.every(([x, y]) => !at(2 * x, 2 * y) && at(3 * x, 3 * y))
}

it('finder signature matches at tl/tr/bl but not at the empty corner', () => {
  const g = buildQrGrid(); const f = finderCentres(g)
  expect(isFinderAt(g, f.tl)).toBe(true)
  expect(isFinderAt(g, f.tr)).toBe(true)
  expect(isFinderAt(g, f.bl)).toBe(true)
  expect(isFinderAt(g, f.empty)).toBe(false)
})

type Pt = [number, number]
/** Minimal parser for the absolute/relative M h v H V z commands the generator emits. */
function parseSubpaths(d: string): Pt[][] {
  const subs: Pt[][] = []
  let cur: Pt[] = []
  let x = 0, y = 0
  for (const m of d.matchAll(/([MhvHVZz])([^MhvHVZz]*)/g)) {
    const cmd = m[1]; const a = (m[2] ?? '').trim().split(/[\s,]+/).filter(Boolean).map(Number)
    if (cmd === 'M') { if (cur.length) subs.push(cur); x = a[0] ?? 0; y = a[1] ?? 0; cur = [[x, y]] }
    else if (cmd === 'h') { x += a[0] ?? 0; cur.push([x, y]) }
    else if (cmd === 'v') { y += a[0] ?? 0; cur.push([x, y]) }
    else if (cmd === 'H') { x = a[0] ?? 0; cur.push([x, y]) }
    else if (cmd === 'V') { y = a[0] ?? 0; cur.push([x, y]) }
    else if (cur.length) { subs.push(cur); cur = [] }
  }
  if (cur.length) subs.push(cur)
  return subs
}
function insideEvenOdd(subs: Pt[][], px: number, py: number): boolean {
  let inside = false
  for (const poly of subs) {
    for (let i = 0; i < poly.length; i++) {
      const [x1, y1] = poly[i] as Pt; const [x2, y2] = poly[(i + 1) % poly.length] as Pt
      if ((y1 > py) !== (y2 > py) && px < ((x2 - x1) * (py - y1)) / (y2 - y1) + x1) inside = !inside
    }
  }
  return inside
}

it('renderMarkerSvg dark regions agree with sampleMarker', () => {
  const g = buildQrGrid(); const S = 50
  const svg = renderMarkerSvg(g, S)
  const paths = [...svg.matchAll(/<path[^>]* d="([^"]*)"/g)].map((m) => parseSubpaths(m[1] ?? ''))
  expect(paths.length).toBe(2)
  const modMm = (QR_SIZE * S) / g.size
  const N = 211
  let checked = 0
  for (let i = 0; i < N; i++) for (let j = 0; j < N; j++) {
    const u = (i + 0.5) / N, v = (j + 0.5) / N
    // skip samples within 0.05 module of a module/frame edge
    const edges = [FRAME_THICKNESS, 1 - FRAME_THICKNESS, 0.2, 0.8]
    const nearEdge = (t: number): boolean =>
      edges.some((e) => Math.abs(t - e) < 1e-3) ||
      (t > 0.2 && t < 0.8 && Math.abs(((t - 0.2) * S) / modMm - Math.round(((t - 0.2) * S) / modMm)) < 0.05)
    if (nearEdge(u) || nearEdge(v)) continue
    const dark = paths.some((p) => insideEvenOdd(p, u * S, v * S))
    expect(dark, `u=${u} v=${v}`).toBe(sampleMarker(g, u, v))
    checked++
  }
  expect(checked).toBeGreaterThan(20000)
})
