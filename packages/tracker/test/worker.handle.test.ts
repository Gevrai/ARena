import { describe, expect, it } from 'vitest'
import { quatAngle } from '../src/math/quat'
import { poseToWorldFromCamera } from '../src/pose/pose'
import { DEFAULT_URL } from '../src/marker/layout'
import { createWorkerState, handleMessage } from '../src/worker/handle'
import { lookAtPose, renderSynthetic } from './synth'

const W = 640
const H = 480
const FOV = 65
const S = 0.05

function frame(gray: Uint8Array, K: import('../src/pose/intrinsics').Intrinsics, id = 1) {
  const buf = gray.buffer.slice(gray.byteOffset, gray.byteOffset + gray.byteLength) as ArrayBuffer
  return {
    type: 'frame' as const,
    id,
    timestamp: 42,
    width: W,
    height: H,
    gray: buf,
    K,
  }
}

describe('worker handleMessage', () => {
  it('returns null for frames before init', () => {
    const st = createWorkerState()
    const pose = lookAtPose({ distanceM: 0.3, tiltDeg: 40, yawDeg: 30, rollDeg: 10 })
    const { image, K } = renderSynthetic(pose, { width: W, height: H, hfovDeg: FOV })
    expect(handleMessage(frame(image.data, K), st)).toBeNull()
  })

  it('detects a synthetic frame and matches ground-truth pose', () => {
    const st = createWorkerState()
    expect(handleMessage({ type: 'init', markerSizeM: S, url: DEFAULT_URL }, st)).toBeNull()
    const pose = lookAtPose({
      distanceM: 0.3,
      tiltDeg: 40,
      yawDeg: 30,
      rollDeg: 10,
      width: W,
      height: H,
      hfovDeg: FOV,
    })
    const { image, K } = renderSynthetic(pose, { width: W, height: H, hfovDeg: FOV, markerSizeM: S })
    const res = handleMessage(frame(image.data, K, 7), st)
    expect(res).not.toBeNull()
    if (!res) return
    expect(res.id).toBe(7)
    expect(res.timestamp).toBe(42)
    expect(res.gray.byteLength).toBe(W * H)
    expect(res.corners).not.toBeNull()
    expect(res.K).toEqual(K)
    expect(res.detectMs).toBeGreaterThanOrEqual(0)
    expect(res.pose).not.toBeNull()
    if (!res.pose) return
    const truth = poseToWorldFromCamera({ R: pose.R, t: pose.t, reprojErrorPx: 0 })
    const dt = Math.hypot(
      res.pose.position[0] - truth.position[0],
      res.pose.position[1] - truth.position[1],
      res.pose.position[2] - truth.position[2],
    )
    expect(dt).toBeLessThan(0.005)
    expect(quatAngle(res.pose.quaternion, truth.quaternion)).toBeLessThan((2 * Math.PI) / 180)
    expect(res.reprojErrorPx).toBeLessThan(3)
  })

  it('returns corners null and NaN reproj for an empty frame, and returns the buffer', () => {
    const st = createWorkerState()
    handleMessage({ type: 'init', markerSizeM: S, url: DEFAULT_URL }, st)
    const K = renderSynthetic(lookAtPose({ distanceM: 0.3, tiltDeg: 0, yawDeg: 0, rollDeg: 0 })).K
    const res = handleMessage(frame(new Uint8Array(W * H).fill(128), K), st)
    expect(res?.corners).toBeNull()
    expect(res?.pose).toBeNull()
    expect(res?.reprojErrorPx).toBeNaN()
    expect(res?.gray.byteLength).toBe(W * H)
  })
})
