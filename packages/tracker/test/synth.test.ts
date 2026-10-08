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

  it('is deterministic and fast with noise', () => {
    const pose = lookAtPose({ distanceM: 0.3, tiltDeg: 20, yawDeg: 30, rollDeg: 0 })
    const a = renderSynthetic(pose, { noise: 3, blurPx: 1 }).image
    const b = renderSynthetic(pose, { noise: 3, blurPx: 1 }).image
    expect(Buffer.compare(a.data, b.data)).toBe(0)
    const t0 = performance.now()
    renderSynthetic(pose)
    const dt = performance.now() - t0
    console.log(`render 640x480 ss3: ${dt.toFixed(1)} ms`)
    expect(dt).toBeLessThan(300)
  })
})
