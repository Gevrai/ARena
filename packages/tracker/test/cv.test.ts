import { describe, expect, it } from 'vitest'
import { findContours } from '../src/cv/contours'
import type { Contour } from '../src/cv/contours'
import { toGray } from '../src/cv/image'
import type { GrayImage } from '../src/cv/image'
import { approxPolyDP, isContourConvex, perimeter } from '../src/cv/poly'
import { adaptiveThreshold } from '../src/cv/threshold'
import { lookAtPose, renderSynthetic } from './synth'

function filled(w: number, h: number, bg: number): GrayImage {
  return { width: w, height: h, data: new Uint8Array(w * h).fill(bg) }
}

/** Convex 4-gons from a binary image, as approxPolyDP(0.05*perimeter) of every contour. */
function quads(bin: GrayImage): Contour[] {
  const out: Contour[] = []
  for (const c of findContours(bin)) {
    const per = perimeter(c)
    if (per < 40) continue
    const p = approxPolyDP(c, 0.05 * per)
    if (p.length === 4 && isContourConvex(p)) out.push(p)
  }
  return out
}

/** Max over expected corners of the distance to the nearest polygon vertex (and vice versa). */
function maxCornerError(poly: Contour, expected: [number, number][]): number {
  return Math.max(
    ...expected.map(([ex, ey]) => Math.min(...poly.map((p) => Math.hypot(p.x - ex, p.y - ey)))),
  )
}

describe('toGray', () => {
  it('converts RGBA to luma and reuses out', () => {
    const rgba = new Uint8ClampedArray([255, 255, 255, 255, 0, 0, 0, 255, 255, 0, 0, 255])
    const g = toGray(rgba, 3, 1)
    expect(Array.from(g.data)).toEqual([255, 0, 76])
    expect(toGray(rgba, 3, 1, g)).toBe(g)
  })
})

describe('cv primitives', () => {
  it('finds one convex 4-gon for a dark square on white', () => {
    const img = filled(300, 300, 255)
    for (let y = 100; y < 200; y++) for (let x = 100; x < 200; x++) img.data[y * 300 + x] = 0
    const bin = adaptiveThreshold(img, 7, 7)
    expect(bin.data[100 * 300 + 150]).toBe(255) // border pixel
    expect(bin.data[150 * 300 + 150]).toBe(0) // interior is flat
    expect(bin.data[50 * 300 + 50]).toBe(0)
    const found = quads(bin).filter(
      (q) =>
        maxCornerError(q, [
          [100, 100],
          [200, 100],
          [200, 200],
          [100, 200],
        ]) < 1.5,
    )
    expect(found).toHaveLength(1)
    // all contours of perimeter > 40 that are 4-gons: the outer border and the inner hole border
    expect(quads(bin).length).toBeLessThanOrEqual(2)
  })

  it('finds no sizeable contours in a blank image', () => {
    const bin = adaptiveThreshold(filled(200, 200, 255), 7, 7)
    expect(findContours(bin).filter((c) => perimeter(c) > 40)).toHaveLength(0)
  })

  it('isContourConvex rejects a concave polygon', () => {
    const concave: Contour = [
      { x: 0, y: 0 },
      { x: 10, y: 0 },
      { x: 5, y: 3 },
      { x: 10, y: 10 },
      { x: 0, y: 10 },
    ]
    expect(isContourConvex(concave)).toBe(false)
    expect(
      perimeter([
        { x: 0, y: 0 },
        { x: 3, y: 0 },
        { x: 3, y: 4 },
      ]),
    ).toBeCloseTo(12)
  })

  it('survives a brightness gradient and a specular disc over the frame', () => {
    const pose = lookAtPose({ distanceM: 0.15, tiltDeg: 0, yawDeg: 0, rollDeg: 0 })
    const gradient = (x: number): number => 255 - (135 * x) / 640
    const base = renderSynthetic(pose, { background: gradient })
    const tl = base.cornersPx[0] ?? [0, 0]
    const bl = base.cornersPx[3] ?? [0, 0]
    // radius-30 disc straddling the left frame edge: covers the outer ~8 px of the band, so the band stays connected
    const cx = (tl[0] + bl[0]) / 2 - 22
    const cy = (tl[1] + bl[1]) / 2 - 20
    const disc: [number, number][] = []
    for (let i = 0; i < 32; i++) {
      const a = (i / 32) * 2 * Math.PI
      disc.push([cx + 30 * Math.cos(a), cy + 30 * Math.sin(a)])
    }
    const { image, cornersPx } = renderSynthetic(pose, {
      background: gradient,
      clutter: [{ corners: disc, value: 255 }],
    })
    const bin = adaptiveThreshold(image, 7, 7)
    const match = quads(bin).filter((q) => maxCornerError(q, cornersPx) <= 2)
    expect(match.length).toBeGreaterThanOrEqual(1)
  })
})

describe.skipIf(!process.env['BENCH'])('benchmark', () => {
  it('threshold + contours on 640x480', () => {
    const pose = lookAtPose({ distanceM: 0.2, tiltDeg: 20, yawDeg: 30, rollDeg: 10 })
    const { image } = renderSynthetic(pose, { noise: 3 })
    const out = filled(640, 480, 0)
    for (let i = 0; i < 20; i++) findContours(adaptiveThreshold(image, 7, 7, out))
    const N = 100
    const t0 = performance.now()
    let n = 0
    for (let i = 0; i < N; i++) n += findContours(adaptiveThreshold(image, 7, 7, out)).length
    const ms = (performance.now() - t0) / N
    process.stdout.write(
      `BENCH threshold+contours 640x480: ${ms.toFixed(2)} ms/frame (${n / N} contours)\n`,
    )
    expect(ms).toBeGreaterThan(0)
  })
})
