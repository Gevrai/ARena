import { describe, expect, it } from 'vitest'
import { applyHomography, homographyFromQuad } from '../src/cv/homography'
import type { GrayImage } from '../src/cv/image'
import { detectFramedQr } from '../src/detect/framedQr'
import { buildQrGrid } from '../src/marker/layout'
import { lookAtPose, renderSynthetic } from './synth'

const grid = buildQrGrid()
type Pose = Parameters<typeof lookAtPose>[0]
type Clutter = NonNullable<Parameters<typeof renderSynthetic>[1]>['clutter']

function scene(
  p: Pose,
  extra: { noise?: number; blurPx?: number; clutter?: Clutter } = {},
): { image: GrayImage; cornersPx: [number, number][] } {
  const pose = lookAtPose({ ...p, width: 640, height: 480, hfovDeg: 65 })
  return renderSynthetic(pose, { width: 640, height: 480, hfovDeg: 65, ...extra })
}

function maxErr(a: [number, number][], b: [number, number][]): number {
  let m = 0
  for (let i = 0; i < 4; i++) {
    const p = a[i] as [number, number]
    const q = b[i] as [number, number]
    m = Math.max(m, Math.hypot(p[0] - q[0], p[1] - q[1]))
  }
  return m
}

function report(name: string, e: number): void {
  if (process.env['BENCH']) process.stdout.write(`ERR ${name}: ${e.toFixed(3)} px\n`)
}

describe('homography', () => {
  it('maps the unit square to a quad and back', () => {
    const src: [number, number][] = [
      [0, 0],
      [1, 0],
      [1, 1],
      [0, 1],
    ]
    const dst: [number, number][] = [
      [10, 20],
      [110, 30],
      [100, 140],
      [5, 120],
    ]
    const H = homographyFromQuad(src, dst)
    expect(H).not.toBeNull()
    if (!H) return
    for (let i = 0; i < 4; i++) {
      const [x, y] = applyHomography(
        H,
        (src[i] as number[])[0] as number,
        (src[i] as number[])[1] as number,
      )
      expect(x).toBeCloseTo((dst[i] as number[])[0] as number, 6)
      expect(y).toBeCloseTo((dst[i] as number[])[1] as number, 6)
    }
  })
  it('returns null for a degenerate quad', () => {
    const src: [number, number][] = [
      [0, 0],
      [1, 0],
      [1, 1],
      [0, 1],
    ]
    expect(
      homographyFromQuad(src, [
        [0, 0],
        [1, 1],
        [2, 2],
        [3, 3],
      ]),
    ).toBeNull()
  })
})

describe('detectFramedQr', () => {
  it('front-facing at 0.3 m: corners within 0.5 px', () => {
    const { image, cornersPx } = scene({ distanceM: 0.3, tiltDeg: 0, yawDeg: 0, rollDeg: 0 })
    const d = detectFramedQr(image, grid)
    expect(d).not.toBeNull()
    const e = maxErr(d?.corners ?? [], cornersPx)
    report('front 0.3m', e)
    expect(e).toBeLessThan(0.5)
    expect(d?.score).toBeGreaterThan(0.75)
  })

  for (const roll of [0, 90, 180, 270, 37]) {
    it(`roll ${roll}: corner order matches TL,TR,BR,BL`, () => {
      const { image, cornersPx } = scene({ distanceM: 0.35, tiltDeg: 0, yawDeg: 0, rollDeg: roll })
      const d = detectFramedQr(image, grid)
      expect(d).not.toBeNull()
      const e = maxErr(d?.corners ?? [], cornersPx)
      report(`roll ${roll}`, e)
      expect(e).toBeLessThan(1)
    })
  }

  it('tilt 50, yaw 30, 0.45 m: corner error < 1 px', () => {
    const { image, cornersPx } = scene({ distanceM: 0.45, tiltDeg: 50, yawDeg: 30, rollDeg: 0 })
    const d = detectFramedQr(image, grid)
    expect(d).not.toBeNull()
    const e = maxErr(d?.corners ?? [], cornersPx)
    report('tilt50 yaw30', e)
    expect(e).toBeLessThan(1)
  })

  it('small marker (~45 px) with noise 6 and blur 1 is still detected', () => {
    const { image, cornersPx } = scene(
      { distanceM: 0.55, tiltDeg: 0, yawDeg: 0, rollDeg: 0 },
      { noise: 6, blurPx: 1 },
    )
    const d = detectFramedQr(image, grid)
    expect(d).not.toBeNull()
    const e = maxErr(d?.corners ?? [], cornersPx)
    report('small noisy', e)
    expect(e).toBeLessThan(1.5)
  })

  for (const roll of [90, 180, 270, 143]) {
    it(`small noisy blurred marker, roll ${roll}: correct corner order`, () => {
      const { image, cornersPx } = scene(
        { distanceM: 0.55, tiltDeg: 0, yawDeg: 0, rollDeg: roll },
        { noise: 6, blurPx: 1 },
      )
      const d = detectFramedQr(image, grid)
      expect(d).not.toBeNull()
      const e = maxErr(d?.corners ?? [], cornersPx)
      report(`small noisy roll ${roll}`, e)
      expect(e).toBeLessThan(1.5)
    })
  }

  it('beyond-spec sweep (blur 2, noise 12): never a wrong rotation (null is fine)', () => {
    let wrong = 0
    let detected = 0
    let total = 0
    for (const d of [0.5, 0.56, 0.62]) {
      for (let roll = 0; roll < 360; roll += 30) {
        const { image, cornersPx } = scene(
          { distanceM: d, tiltDeg: 15, yawDeg: 40, rollDeg: roll },
          { noise: 12, blurPx: 2 },
        )
        total++
        const det = detectFramedQr(image, grid)
        if (!det) continue
        detected++
        if (maxErr(det.corners, cornersPx) > 6) wrong++
      }
    }
    if (process.env['BENCH'])
      process.stdout.write(`SWEEP detected ${detected}/${total}, wrong ${wrong}\n`)
    expect(wrong).toBe(0)
    expect(detected).toBeGreaterThanOrEqual(Math.floor(total / 3))
  })

  // Card decoys: dark outer quad + lighter inner quad.
  const card = (x: number, y: number, w: number, h: number, b: number): NonNullable<Clutter> => [
    {
      corners: [
        [x, y],
        [x + w, y],
        [x + w, y + h],
        [x, y + h],
      ],
      value: 25,
    },
    {
      corners: [
        [x + b, y + b],
        [x + w - b, y + b],
        [x + w - b, y + h - b],
        [x + b, y + h - b],
      ],
      value: 220,
    },
  ]
  const clutter: NonNullable<Clutter> = [
    ...card(20, 20, 150, 95, 14),
    ...card(470, 330, 140, 120, 12),
    ...card(30, 300, 70, 150, 10), // phone bezel
  ]

  it('clutter + marker: returns exactly the marker', () => {
    const { image, cornersPx } = scene(
      { distanceM: 0.4, tiltDeg: 10, yawDeg: 20, rollDeg: 15 },
      { clutter },
    )
    const d = detectFramedQr(image, grid)
    expect(d).not.toBeNull()
    const e = maxErr(d?.corners ?? [], cornersPx)
    report('clutter', e)
    expect(e).toBeLessThan(1)
  })

  it('clutter without the marker: null', () => {
    const { image } = scene({ distanceM: 50, tiltDeg: 0, yawDeg: 0, rollDeg: 0 }, { clutter })
    expect(detectFramedQr(image, grid)).toBeNull()
  })

  it('cropped marker (a corner outside the image): null', () => {
    for (const off of [
      [290, 210],
      [-290, -210],
      [300, 0],
      [0, -215],
    ] as [number, number][]) {
      const { image } = scene({ distanceM: 0.3, tiltDeg: 0, yawDeg: 0, rollDeg: 0, offsetPx: off })
      expect(detectFramedQr(image, grid)).toBeNull()
    }
  })

  it('random noise image: null', () => {
    let s = 7
    const data = new Uint8Array(640 * 480)
    for (let i = 0; i < data.length; i++) {
      s = (Math.imul(s, 1664525) + 1013904223) >>> 0
      data[i] = s >>> 24
    }
    expect(detectFramedQr({ width: 640, height: 480, data }, grid)).toBeNull()
  })

  it('timing on 640x480', () => {
    const { image } = scene({ distanceM: 0.35, tiltDeg: 20, yawDeg: 40, rollDeg: 10 })
    detectFramedQr(image, grid)
    const n = 20
    const t0 = performance.now()
    for (let i = 0; i < n; i++) detectFramedQr(image, grid)
    const ms = (performance.now() - t0) / n
    if (process.env['BENCH'])
      process.stdout.write(`BENCH detectFramedQr 640x480: ${ms.toFixed(2)} ms/frame\n`)
    expect(ms).toBeLessThan(500)
  })
})
