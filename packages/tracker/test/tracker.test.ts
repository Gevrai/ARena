/* eslint-disable @typescript-eslint/no-non-null-assertion, @typescript-eslint/no-extraneous-class -- test stubs */
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { createTracker } from '../src/tracker'
import type { TrackerStatus } from '../src/tracker'
import { intrinsicsFromSize, projectionForCover } from '../src/pose/intrinsics'
import { mat4FromRotationTranslation, mat4Invert } from '../src/math/mat4'
import { quatFromAxisAngle } from '../src/math/quat'
import type { Vec3 } from '../src/math/vec3'

class FakeWorker {
  static last: FakeWorker | null = null
  onmessage: ((e: { data: unknown }) => void) | null = null
  onerror: ((e: unknown) => void) | null = null
  posted: { msg: { type: string; id?: number; gray?: ArrayBuffer }; transfer?: unknown }[] = []
  terminated = false
  constructor() {
    FakeWorker.last = this
  }
  postMessage(msg: { type: string }, transfer?: unknown) {
    this.posted.push({ msg, transfer })
  }
  terminate() {
    this.terminated = true
  }
}

function makeVideo() {
  const v = {
    srcObject: null as unknown,
    attrs: {} as Record<string, unknown>,
    muted: false,
    playsInline: false,
    videoWidth: 1280,
    videoHeight: 720,
    play: vi.fn(async () => {}),
    setAttribute(k: string, val: unknown) {
      this.attrs[k] = val
    },
    cb: null as null | (() => void),
    requestVideoFrameCallback(cb: () => void) {
      this.cb = cb
      return 1
    },
    cancelVideoFrameCallback: vi.fn(),
  }
  return v
}

function makeStream() {
  const track = { stop: vi.fn() }
  return { stream: { getTracks: () => [track] }, track }
}

let listeners: Record<string, (e: unknown) => void>
let removed: string[]

beforeEach(() => {
  listeners = {}
  removed = []
  FakeWorker.last = null
  vi.stubGlobal('isSecureContext', true)
  vi.stubGlobal('window', {
    addEventListener: (t: string, f: (e: unknown) => void) => (listeners[t] = f),
    removeEventListener: (t: string) => removed.push(t),
    orientation: 0,
  })
  vi.stubGlobal('Worker', FakeWorker)
  vi.stubGlobal(
    'OffscreenCanvas',
    class {
      width = 0
      height = 0
      getContext() {
        return {
          drawImage() {},
          getImageData: (_x: number, _y: number, w: number, h: number) => ({
            data: new Uint8ClampedArray(w * h * 4),
          }),
        }
      }
    },
  )
  vi.stubGlobal('DeviceOrientationEvent', class {})
  const { stream } = makeStream()
  vi.stubGlobal('navigator', { mediaDevices: { getUserMedia: vi.fn(async () => stream) } })
})
afterEach(() => vi.unstubAllGlobals())

const mk = (extra: object = {}) => {
  const video = makeVideo()
  const tracker = createTracker({ video: video as unknown as HTMLVideoElement, ...extra })
  return { video, tracker }
}

describe('start() errors', () => {
  it('no mediaDevices -> no-camera and status error', async () => {
    vi.stubGlobal('navigator', {})
    const { tracker } = mk()
    const seen: TrackerStatus[] = []
    tracker.onStatus((s) => seen.push(s))
    await expect(tracker.start()).rejects.toMatchObject({ code: 'no-camera' })
    expect(seen.at(-1)).toEqual({ state: 'error', reason: 'no-camera' })
  })
  it('NotAllowedError -> camera-denied', async () => {
    vi.stubGlobal('navigator', {
      mediaDevices: {
        getUserMedia: vi.fn(async () => {
          throw Object.assign(new Error('x'), { name: 'NotAllowedError' })
        }),
      },
    })
    const { tracker } = mk()
    await expect(tracker.start()).rejects.toMatchObject({ code: 'camera-denied' })
  })
  it('NotFoundError -> no-camera', async () => {
    vi.stubGlobal('navigator', {
      mediaDevices: {
        getUserMedia: vi.fn(async () => {
          throw Object.assign(new Error('x'), { name: 'NotFoundError' })
        }),
      },
    })
    const { tracker } = mk()
    await expect(tracker.start()).rejects.toMatchObject({ code: 'no-camera' })
  })
  it('insecure context -> insecure-context', async () => {
    vi.stubGlobal('isSecureContext', false)
    const { tracker } = mk()
    const seen: TrackerStatus[] = []
    tracker.onStatus((s) => seen.push(s))
    await expect(tracker.start()).rejects.toMatchObject({ code: 'insecure-context' })
    expect(seen.at(-1)?.state).toBe('error')
  })
})

describe('start() happy paths', () => {
  it('motion permission denied still resolves, imu=false; requests constraints', async () => {
    vi.stubGlobal(
      'DeviceOrientationEvent',
      class {
        static requestPermission = async () => 'denied'
      },
    )
    const { tracker, video } = mk()
    await tracker.start()
    expect(tracker.stats().imu).toBe(false)
    expect(listeners['deviceorientation']).toBeUndefined()
    const gum = (navigator.mediaDevices.getUserMedia as ReturnType<typeof vi.fn>).mock.calls[0]
    expect(gum?.[0]).toEqual({
      video: {
        facingMode: { ideal: 'environment' },
        width: { ideal: 1280 },
        height: { ideal: 720 },
      },
      audio: false,
    })
    expect(video.attrs['playsinline']).toBeDefined()
    expect(video.play).toHaveBeenCalled()
    expect(FakeWorker.last?.posted[0]?.msg.type).toBe('init')
    tracker.stop()
  })

  it('requests motion permission before getUserMedia resolves (sync at start)', async () => {
    const order: string[] = []
    vi.stubGlobal(
      'DeviceOrientationEvent',
      class {
        static requestPermission = async () => {
          order.push('motion')
          return 'granted'
        }
      },
    )
    const { stream } = makeStream()
    vi.stubGlobal('navigator', {
      mediaDevices: {
        getUserMedia: vi.fn(async () => {
          order.push('gum')
          return stream
        }),
      },
    })
    const { tracker } = mk()
    const p = tracker.start()
    expect(order[0]).toBe('motion')
    await p
    expect(tracker.stats().imu).toBe(true)
    tracker.stop()
  })

  it('frame loop posts frames, reuses buffers, tracks, and stop() cleans up', async () => {
    const { tracker, video } = mk({ useImu: false })
    const { stream, track } = makeStream()
    vi.stubGlobal('navigator', { mediaDevices: { getUserMedia: vi.fn(async () => stream) } })
    const seen: string[] = []
    tracker.onStatus((s) => seen.push(s.state))
    await tracker.start()
    const w = FakeWorker.last!
    video.cb!()
    const frames = w.posted.filter((p) => p.msg.type === 'frame')
    expect(frames).toHaveLength(1)
    const f = frames[0]!.msg as unknown as {
      id: number
      gray: ArrayBuffer
      width: number
      height: number
    }
    expect(f.width).toBe(640)
    expect(f.height).toBe(360)
    // Busy: next frame is dropped
    video.cb!()
    expect(w.posted.filter((p) => p.msg.type === 'frame')).toHaveLength(1)
    const K = intrinsicsFromSize(640, 360, 65)
    w.onmessage!({
      data: {
        type: 'result',
        id: f.id,
        timestamp: performance.now(),
        pose: { position: [0, 0, 0.3], quaternion: [0, 0, 0, 1] },
        corners: [
          [1, 1],
          [2, 1],
          [2, 2],
          [1, 2],
        ],
        reprojErrorPx: 0.5,
        detectMs: 7,
        K,
        gray: new ArrayBuffer(640 * 360),
      },
    })
    expect(tracker.getPose().source).toBe('marker')
    expect(tracker.stats().detectMs).toBe(7)
    expect(tracker.stats().corners).not.toBeNull()
    expect(seen).toContain('tracking')
    video.cb!()
    expect(w.posted.filter((p) => p.msg.type === 'frame')).toHaveLength(2)
    tracker.stop()
    expect(w.terminated).toBe(true)
    expect(track.stop).toHaveBeenCalled()
    expect(seen.at(-1)).toBe('idle')
  })

  it('null-pose result updates stats but does not feed fusion; error recorded', async () => {
    const { tracker, video } = mk({ useImu: false })
    await tracker.start()
    const w = FakeWorker.last!
    video.cb!()
    const f = w.posted.find((p) => p.msg.type === 'frame')!.msg as unknown as { id: number }
    w.onmessage!({
      data: {
        type: 'result',
        id: f.id,
        timestamp: 0,
        pose: null,
        corners: null,
        reprojErrorPx: NaN,
        detectMs: 3,
        K: intrinsicsFromSize(640, 360),
        gray: new ArrayBuffer(640 * 360),
        error: 'boom',
      },
    })
    expect(tracker.getPose().source).toBe('none')
    expect(tracker.getPose().matrix[0]).toBe(1)
    expect(tracker.stats().lastError).toBe('boom')
    tracker.stop()
  })

  it('onStatus fires on transitions only and unsubscribes', async () => {
    const { tracker } = mk()
    const seen: string[] = []
    const off = tracker.onStatus((s) => seen.push(s.state))
    await tracker.start()
    expect(seen).toEqual(['starting', 'lost'])
    off()
    tracker.stop()
    expect(seen).toEqual(['starting', 'lost'])
  })
})

describe('robustness', () => {
  it('a throw while grabbing does not kill the loop', async () => {
    let calls = 0
    vi.stubGlobal(
      'OffscreenCanvas',
      class {
        width = 0
        height = 0
        getContext() {
          return {
            drawImage() {
              if (calls++ === 0) throw new Error('draw failed')
            },
            getImageData: (_x: number, _y: number, w: number, h: number) => ({
              data: new Uint8ClampedArray(w * h * 4),
            }),
          }
        }
      },
    )
    const { tracker, video } = mk({ useImu: false })
    await tracker.start()
    const w = FakeWorker.last!
    expect(() => video.cb!()).not.toThrow()
    expect(w.posted.filter((p) => p.msg.type === 'frame')).toHaveLength(0)
    expect(tracker.stats().lastError).toBe('draw failed')
    expect(video.cb).not.toBeNull()
    video.cb!()
    expect(w.posted.filter((p) => p.msg.type === 'frame')).toHaveLength(1)
    tracker.stop()
  })

  it('worker error -> status error, worker terminated, loop stopped', async () => {
    const { tracker, video } = mk({ useImu: false })
    const seen: TrackerStatus[] = []
    tracker.onStatus((s) => seen.push(s))
    await tracker.start()
    const w = FakeWorker.last!
    video.cb!()
    w.onerror!({ message: 'load failed' })
    expect(seen.at(-1)).toEqual({ state: 'error', reason: 'unknown' })
    expect(w.terminated).toBe(true)
    expect(tracker.stats().lastError).toBe('load failed')
    const n = w.posted.length
    video.cb?.()
    expect(w.posted.length).toBe(n)
  })

  it('stop() during start() leaves idle, no worker, tracks stopped', async () => {
    let resolveGum!: (s: unknown) => void
    const { stream, track } = makeStream()
    vi.stubGlobal('navigator', {
      mediaDevices: {
        getUserMedia: vi.fn(() => new Promise((r) => (resolveGum = r))),
      },
    })
    const { tracker } = mk()
    const seen: string[] = []
    tracker.onStatus((s) => seen.push(s.state))
    const p = tracker.start()
    tracker.stop()
    resolveGum(stream)
    await p
    expect(FakeWorker.last).toBeNull()
    expect(track.stop).toHaveBeenCalled()
    expect(seen.at(-1)).toBe('idle')
  })

  it('getUserMedia rejection during motion prompt is not unhandled', async () => {
    let resolveMotion!: (v: string) => void
    vi.stubGlobal(
      'DeviceOrientationEvent',
      class {
        static requestPermission = () => new Promise<string>((r) => (resolveMotion = r))
      },
    )
    vi.stubGlobal('navigator', {
      mediaDevices: {
        getUserMedia: vi.fn(async () => {
          throw Object.assign(new Error('x'), { name: 'NotAllowedError' })
        }),
      },
    })
    const { tracker } = mk()
    const p = tracker.start()
    await new Promise((r) => setTimeout(r, 5))
    resolveMotion('granted')
    await expect(p).rejects.toMatchObject({ code: 'camera-denied' })
  })
})

describe('projectionForCover', () => {
  const project = (P: Float32Array, V: Float32Array, p: Vec3, W: number, H: number) => {
    const c = [p[0], p[1], p[2], 1]
    const e = [0, 1, 2, 3].map((r) => [0, 1, 2, 3].reduce((a, k) => a + V[k * 4 + r]! * c[k]!, 0))
    const q = [0, 1, 2, 3].map((r) => [0, 1, 2, 3].reduce((a, k) => a + P[k * 4 + r]! * e[k]!, 0))
    return { x: ((q[0]! / q[3]! + 1) / 2) * W, y: ((1 - q[1]! / q[3]!) / 2) * H, ez: e[2]! }
  }

  for (const [W, H] of [
    [390, 844],
    [844, 390],
  ] as const) {
    it(`matches object-fit: cover in ${W}x${H}`, () => {
      const vw = 1280
      const vh = 720
      const K = intrinsicsFromSize(vw, vh, 65)
      const P = projectionForCover(K, W, H, 0.01, 10)
      const q = quatFromAxisAngle([1, 0, 0], (-55 * Math.PI) / 180)
      const pos: Vec3 = [0.03, 0.2, 0.15]
      const world = mat4FromRotationTranslation(q, pos)
      const V = mat4Invert(world)
      const h = 0.025
      const corners: Vec3[] = [
        [-h, 0, -h],
        [h, 0, -h],
        [h, 0, h],
        [-h, 0, h],
      ]
      const s = Math.max(W / vw, H / vh)
      for (const c of corners) {
        const cam = [0, 1, 2, 3].map((r) =>
          [0, 1, 2, 3].reduce((a, k) => a + V[k * 4 + r]! * [c[0], c[1], c[2], 1][k]!, 0),
        )
        // three camera -> OpenCV camera: (x, -y, -z)
        const u = (K.fx * cam[0]!) / -cam[2]! + K.cx
        const v = (K.fy * -cam[1]!) / -cam[2]! + K.cy
        const ex = (u - vw / 2) * s + W / 2
        const ey = (v - vh / 2) * s + H / 2
        const got = project(P, V, c, W, H)
        expect(got.ez).toBeLessThan(0)
        expect(Math.abs(got.x - ex)).toBeLessThan(1)
        expect(Math.abs(got.y - ey)).toBeLessThan(1)
      }
    })
  }
})
