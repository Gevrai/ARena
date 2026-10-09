/* eslint-disable @typescript-eslint/no-non-null-assertion -- test helpers */
import { describe, expect, it } from 'vitest'
import { detectFramedQr } from '../src/detect/framedQr'
import type { Quad } from '../src/detect/framedQr'
import { buildQrGrid } from '../src/marker/layout'
import { lookAtPose, renderSynthetic } from './synth'

const grid = buildQrGrid()
const FX = 320 / Math.tan((32.5 * Math.PI) / 180)

function rate(distanceM: number, blur: number, useHint = false, trials = 12): { det: number; wrong: number; total: number } {
  let det = 0
  let wrong = 0
  let total = 0
  for (let i = 0; i < trials; i++) {
    const mkPose = (dx: number, dy: number): ReturnType<typeof lookAtPose> => lookAtPose({
      distanceM,
      tiltDeg: (i % 3) * 12,
      yawDeg: i * 53,
      rollDeg: i * 37,
      offsetPx: [((i * 29) % 120) - 60 + dx, ((i * 17) % 80) - 40 + dy],
      width: 640,
      height: 480,
      hfovDeg: 65,
    })
    const ang = (i * 41 * Math.PI) / 180
    const pose = mkPose(0, 0)
    let hint: Quad | undefined
    if (useHint) {
      // The previous (sharp) frame: the marker was ~2x the blur length away along the motion.
      const prev = renderSynthetic(mkPose(-2 * blur * Math.cos(ang), -2 * blur * Math.sin(ang)), {
        width: 640,
        height: 480,
        hfovDeg: 65,
        noise: 3,
      })
      hint = detectFramedQr(prev.image, grid)?.corners
    }
    const { image, cornersPx } = renderSynthetic(pose, {
      width: 640,
      height: 480,
      hfovDeg: 65,
      noise: 3,
      motionBlurPx: blur,
      motionAngleDeg: i * 41,
    })
    total++
    const d = detectFramedQr(image, grid, hint ? { hint } : {})
    if (!d) continue
    const e = Math.max(...d.corners.map((c, k) => Math.hypot(c[0] - (cornersPx[k] as number[])[0]!, c[1] - (cornersPx[k] as number[])[1]!)))
    if (e > 4 + blur / 2) wrong++
    else det++
  }
  return { det, wrong, total }
}

describe('motion-blur detection rate', () => {
  // Before the refine fallback / hint: 84 px marker, 8 px blur: 4/12 cold, 12 px: 0/12.
  it('84 px marker, 8 px streak: detected cold in most frames, no wrong detections', () => {
    const r = rate(0.3, 8, false, 8)
    expect(r.det).toBeGreaterThanOrEqual(6)
    expect(r.wrong).toBe(0)
  })
  it('84 px marker, 12 px streak: the previous detection as hint recovers it', () => {
    const r = rate(0.3, 12, true, 8)
    expect(r.det).toBeGreaterThanOrEqual(6)
    expect(r.wrong).toBe(0)
  })
  it('a hint far from the only marker-like quad does not create a detection', () => {
    const pose = lookAtPose({ distanceM: 50, tiltDeg: 0, yawDeg: 0, rollDeg: 0, width: 640, height: 480 })
    const { image } = renderSynthetic(pose, { width: 640, height: 480, hfovDeg: 65 })
    const hint: Quad = [[100, 100], [180, 100], [180, 180], [100, 180]]
    expect(detectFramedQr(image, grid, { hint })).toBeNull()
  })

  it('bench', () => {
    if (!process.env['BENCH']) return
    const lines: string[] = []
    for (const dist of [0.25, 0.3, 0.4, 0.5]) {
      const px = Math.round((FX * 0.05) / dist)
      for (const useHint of [false, true]) {
        const row: string[] = []
        for (const blur of [0, 4, 8, 12, 16, 24]) {
          const r = rate(dist, blur, useHint)
          row.push(`b${blur}:${r.det}/${r.total}${r.wrong ? `(!${r.wrong})` : ''}`)
        }
        lines.push(`marker ${px}px ${useHint ? 'hint' : 'cold'}  ${row.join('  ')}`)
      }
    }
    process.stdout.write(`BLUR\n${lines.join('\n')}\n`)
    expect(true).toBe(true)
  })
})

describe('hint false positives', () => {
  it('dark-framed decoy cards at the hint location are not accepted', () => {
    const card = (x: number, y: number, w: number, h: number, b: number): NonNullable<NonNullable<Parameters<typeof renderSynthetic>[1]>['clutter']> => [
      { corners: [[x, y], [x + w, y], [x + w, y + h], [x, y + h]], value: 25 },
      { corners: [[x + b, y + b], [x + w - b, y + b], [x + w - b, y + h - b], [x + b, y + h - b]], value: 220 },
    ]
    const pose = lookAtPose({ distanceM: 50, tiltDeg: 0, yawDeg: 0, rollDeg: 0, width: 640, height: 480 })
    for (const [x, y, w, h, b] of [[200, 150, 90, 90, 12], [300, 200, 100, 70, 10], [40, 40, 150, 95, 14]] as number[][]) {
      const { image } = renderSynthetic(pose, {
        width: 640,
        height: 480,
        hfovDeg: 65,
        clutter: card(x!, y!, w!, h!, b!),
      })
      const hint: Quad = [[x!, y!], [x! + w!, y!], [x! + w!, y! + h!], [x!, y! + h!]]
      expect(detectFramedQr(image, grid, { hint })).toBeNull()
    }
  })
})
