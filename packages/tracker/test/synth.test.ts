import { describe, expect, it } from 'vitest'
import { quatToMat3 } from '../src/math/quat'
import { intrinsicsFromSize } from '../src/pose/intrinsics'
import { lookAtPose, renderSynthetic } from './synth'

describe('renderSynthetic', () => {
  it('front-facing pose is symmetric, dark frame, light background', () => {
    const { image, K, cornersPx } = renderSynthetic(lookAtPose({ distanceM: 0.3, tiltDeg: 0, yawDeg: 0, rollDeg: 0 }))
    const [tl, tr, br, bl] = cornersPx as [[number, number], [number, number], [number, number], [number, number]]
    expect(tl[0] + br[0]).toBeCloseTo(K.width, 6)
    expect(tl[1] + br[1]).toBeCloseTo(K.height, 6)
    expect(tr[0] + bl[0]).toBeCloseTo(K.width, 6)
    expect(tl[0]).toBeLessThan(tr[0])
    expect(tl[1]).toBeLessThan(bl[1])
    const px = (x: number, y: number): number => image.data[Math.floor(y) * image.width + Math.floor(x)] ?? -1
    expect(px(tl[0] + 3, tl[1] + 3)).toBeLessThan(60)
    expect(px(5, 5)).toBeGreaterThan(180)
    expect(px(5, 5)).toBe(200)
  })

  it('cornersPx matches projecting the marker corners through K', () => {
    const pose = lookAtPose({ distanceM: 0.25, tiltDeg: 30, yawDeg: 40, rollDeg: 15, offsetPx: [20, -10] })
    const { K, cornersPx } = renderSynthetic(pose, { markerSizeM: 0.05 })
    const S = 0.05
    const uv = [[0, 0], [1, 0], [1, 1], [0, 1]]
    uv.forEach(([u, v], i) => {
      const x = ((u ?? 0) - 0.5) * S
      const y = ((v ?? 0) - 0.5) * S
      const R = pose.R
      const X = (R[0] ?? 0) * x + (R[1] ?? 0) * y + pose.t[0]
      const Y = (R[3] ?? 0) * x + (R[4] ?? 0) * y + pose.t[1]
      const Z = (R[6] ?? 0) * x + (R[7] ?? 0) * y + pose.t[2]
      expect(cornersPx[i]?.[0]).toBeCloseTo(K.fx * (X / Z) + K.cx, 2)
      expect(cornersPx[i]?.[1]).toBeCloseTo(K.fy * (Y / Z) + K.cy, 2)
    })
  })

  it('lookAtPose conventions', () => {
    const p = lookAtPose({ distanceM: 0.3, tiltDeg: 0, yawDeg: 0, rollDeg: 0 })
    Array.from(quatToMat3([0, 0, 0, 1])).forEach((v, i) => expect(p.R[i]).toBeCloseTo(v, 12))
    expect(p.t[0]).toBeCloseTo(0, 12)
    expect(p.t[1]).toBeCloseTo(0, 12)
    expect(p.t[2]).toBeCloseTo(0.3, 12)
    const K = intrinsicsFromSize(640, 480)
    const o = lookAtPose({ distanceM: 0.3, tiltDeg: 20, yawDeg: 70, rollDeg: 10, offsetPx: [30, 12] })
    // marker centre projects to image centre + offset
    expect(K.fx * (o.t[0] / o.t[2]) + K.cx).toBeCloseTo(350, 6)
    expect(K.fy * (o.t[1] / o.t[2]) + K.cy).toBeCloseTo(252, 6)
    // camera-to-marker distance and tilt: optical axis angle to marker normal
    const nz = o.R[8] ?? 0 // marker normal z in camera frame
    expect(Math.acos(nz) * 180 / Math.PI).toBeCloseTo(20, 6)
  })

  it('distanceM is the norm of t, with and without offset', () => {
    for (const offsetPx of [undefined, [30, -20] as [number, number], [-200, 150] as [number, number]]) {
      const p = lookAtPose({ distanceM: 0.35, tiltDeg: 40, yawDeg: 25, rollDeg: 70, offsetPx })
      expect(Math.hypot(...p.t)).toBeCloseTo(0.35, 12)
    }
    const K = intrinsicsFromSize(800, 600, 70)
    const p = lookAtPose({ distanceM: 0.3, tiltDeg: 10, yawDeg: 0, rollDeg: 0, offsetPx: [40, 25], width: 800, height: 600, hfovDeg: 70 })
    expect(K.fx * (p.t[0] / p.t[2]) + K.cx).toBeCloseTo(440, 6)
    expect(K.fy * (p.t[1] / p.t[2]) + K.cy).toBeCloseTo(325, 6)
  })

  it('rendered pixels agree with cornersPx (edges probed in/out)', () => {
    const pose = lookAtPose({ distanceM: 0.35, tiltDeg: 40, yawDeg: 25, rollDeg: 70, offsetPx: [30, -20] })
    const { image, cornersPx } = renderSynthetic(pose)
    const cen: [number, number] = [
      cornersPx.reduce((a, c) => a + c[0], 0) / 4,
      cornersPx.reduce((a, c) => a + c[1], 0) / 4,
    ]
    const px = (x: number, y: number): number => image.data[Math.floor(y) * image.width + Math.floor(x)] ?? -1
    for (let e = 0; e < 4; e++) {
      const [x0, y0] = cornersPx[e] as [number, number]
      const [x1, y1] = cornersPx[(e + 1) % 4] as [number, number]
      const len = Math.hypot(x1 - x0, y1 - y0)
      let nx = -(y1 - y0) / len
      let ny = (x1 - x0) / len
      const mx = (x0 + x1) / 2
      const my = (y0 + y1) / 2
      if (nx * (cen[0] - mx) + ny * (cen[1] - my) < 0) {
        nx = -nx
        ny = -ny
      }
      for (const t of [0.2, 0.35, 0.5, 0.65, 0.8]) {
        const x = x0 + (x1 - x0) * t
        const y = y0 + (y1 - y0) * t
        expect(px(x + 1.5 * nx, y + 1.5 * ny)).toBeLessThan(60)
        expect(px(x - 1.5 * nx, y - 1.5 * ny)).toBeGreaterThan(150)
      }
    }
  })

  it('sign conventions: roll, tilt, yaw', () => {
    const base = { distanceM: 0.3, tiltDeg: 0, yawDeg: 0, rollDeg: 0 }
    const corners = (o: Partial<typeof base>): [number, number][] =>
      renderSynthetic(lookAtPose({ ...base, ...o })).cornersPx
    const len = (a: [number, number], b: [number, number]): number => Math.hypot(a[0] - b[0], a[1] - b[1])
    // roll +90: marker TL goes to image bottom-left (marker rotates counter-clockwise).
    const r = corners({ rollDeg: 90 })
    const [tl, tr, br, bl] = r as [[number, number], [number, number], [number, number], [number, number]]
    expect(tl[0]).toBeLessThan(320)
    expect(tl[1]).toBeGreaterThan(240)
    expect(tr[0]).toBeLessThan(320)
    expect(tr[1]).toBeLessThan(240) // TR -> top-left
    expect(br[0]).toBeGreaterThan(320) // BR -> top-right
    expect(br[1]).toBeLessThan(240)
    expect(bl[0]).toBeGreaterThan(320) // BL -> bottom-right
    expect(bl[1]).toBeGreaterThan(240)
    // tilt 30, yaw 0: camera displaced to the marker's right -> right edge longer than left.
    const t0 = corners({ tiltDeg: 30 }) as [[number, number], [number, number], [number, number], [number, number]]
    expect(len(t0[1], t0[2])).toBeGreaterThan(len(t0[0], t0[3]) * 1.05)
    expect(len(t0[0], t0[1])).toBeCloseTo(len(t0[3], t0[2]) * 1, 0) // top and bottom roughly equal
    // yaw 90: camera towards the bottom edge -> bottom edge longer than top; left/right equal.
    const t1 = corners({ tiltDeg: 30, yawDeg: 90 }) as typeof t0
    expect(len(t1[3], t1[2])).toBeGreaterThan(len(t1[0], t1[1]) * 1.05)
    expect(len(t1[0], t1[3])).toBeCloseTo(len(t1[1], t1[2]), 3)
  })

  it('is deterministic with noise', () => {
    const pose = lookAtPose({ distanceM: 0.3, tiltDeg: 20, yawDeg: 30, rollDeg: 0 })
    const a = renderSynthetic(pose, { noise: 3, blurPx: 1 }).image
    const b = renderSynthetic(pose, { noise: 3, blurPx: 1 }).image
    expect(Buffer.compare(a.data, b.data)).toBe(0)
  })

  it.skipIf(!process.env.BENCH)('bench: 640x480 ss3 render', () => {
    const pose = lookAtPose({ distanceM: 0.3, tiltDeg: 20, yawDeg: 30, rollDeg: 0 })
    const t0 = performance.now()
    renderSynthetic(pose)
    console.log(`render 640x480 ss3: ${(performance.now() - t0).toFixed(1)} ms`)
  })
})
