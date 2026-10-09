/* eslint-disable @typescript-eslint/no-non-null-assertion, @typescript-eslint/no-extraneous-class -- test stubs */
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { createTracker } from '../src/tracker'
import type { TrackerStatus } from '../src/tracker'
import { intrinsicsFromSize, projectionForCover } from '../src/pose/intrinsics'
import { mat4FromRotationTranslation, mat4Invert } from '../src/math/mat4'
import { cameraYawFromQuat, quatFromAxisAngle, quatMultiply } from '../src/math/quat'
import { estimatePose } from '../src/pose/pose'
import type { Detection } from '../src/detect/framedQr'
import { lookAtPose, renderSynthetic } from './synth'
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

type Meta = { captureTime?: number; expectedDisplayTime?: number }

function makeTarget() {
  const handlers = new Map<string, Set<() => void>>()
  return {
    handlers,
    addEventListener(t: string, f: () => void) {
      if (!handlers.has(t)) handlers.set(t, new Set())
      handlers.get(t)!.add(f)
    },
    removeEventListener(t: string, f: () => void) {
      handlers.get(t)?.delete(f)
    },
    emit(t: string) {
      for (const f of [...(handlers.get(t) ?? [])]) f()
    },
    count(t: string) {
      return handlers.get(t)?.size ?? 0
    },
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
    ...makeTarget(),
    cb: null as null | ((now?: number, md?: Meta) => void),
    requestVideoFrameCallback(cb: (now?: number, md?: Meta) => void) {
      this.cb = cb
      return 1
    },
    cancelVideoFrameCallback: vi.fn(),
  }
  return v
}

function makeStream() {
  const track = { stop: vi.fn(), readyState: 'live', ...makeTarget() }
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

describe('portrait FOV', () => {
  it('intrinsics for 1280x720 and 720x1280 share one focal length', () => {
    const a = intrinsicsFromSize(1280, 720, 65)
    const b = intrinsicsFromSize(720, 1280, 65)
    expect(b.fx).toBeCloseTo(a.fx, 9)
    expect(b.fy).toBeCloseTo(a.fy, 9)
    expect(b.cx).toBe(360)
    expect(b.cy).toBe(640)
  })

  it('same marker distance gives the same pose depth in portrait and landscape', () => {
    const depth = (w: number, h: number): number => {
      const pose = lookAtPose({
        distanceM: 0.3,
        tiltDeg: 20,
        yawDeg: 30,
        rollDeg: 0,
        width: w,
        height: h,
      })
      const { cornersPx, K } = renderSynthetic(pose, { width: w, height: h })
      const est = estimatePose(cornersPx as Detection['corners'], K, 0.05)
      return est!.t[2]
    }
    const land = depth(1280, 720)
    const port = depth(720, 1280)
    expect(land).toBeGreaterThan(0.25)
    expect(Math.abs(port - land)).toBeLessThan(0.002)
  })

  it('projectionMatrix uses the long side in portrait video', () => {
    const { tracker, video } = mk()
    video.videoWidth = 720
    video.videoHeight = 1280
    const P = tracker.projectionMatrix(720, 1280, 0.01, 10)
    const K = intrinsicsFromSize(1280, 720, 65)
    expect(P[0]).toBeCloseTo((2 * K.fx) / 720, 5)
  })

  it('hfovDeg alias is still accepted', () => {
    const { tracker, video } = mk({ hfovDeg: 50 })
    const P = tracker.projectionMatrix(1280, 720, 0.01, 10)
    expect(P[0]).toBeCloseTo((2 * intrinsicsFromSize(1280, 720, 50).fx) / 1280, 5)
    tracker.setOptions({ fovDeg: 70 })
    expect(tracker.projectionMatrix(1280, 720, 0.01, 10)[0]).toBeCloseTo(
      (2 * intrinsicsFromSize(1280, 720, 70).fx) / 1280,
      5,
    )
    void video
  })
})

describe('timestamps', () => {
  const frameTs = (): number =>
    (
      FakeWorker.last!.posted.filter((p) => p.msg.type === 'frame').at(-1)!.msg as unknown as {
        timestamp: number
      }
    ).timestamp

  it('uses rvfc captureTime', async () => {
    const { tracker, video } = mk({ useImu: false })
    await tracker.start()
    video.cb!(5000, { captureTime: 4321.5, expectedDisplayTime: 4400 })
    expect(frameTs()).toBe(4321.5)
    tracker.stop()
  })
  it('captureLatencyMs is subtracted from captureTime when given', async () => {
    const { tracker, video } = mk({ useImu: false, captureLatencyMs: 10 })
    await tracker.start()
    video.cb!(5000, { captureTime: 4000 })
    expect(frameTs()).toBe(3990)
    tracker.stop()
  })
  it('falls back to expectedDisplayTime minus one frame', async () => {
    const { tracker, video } = mk({ useImu: false })
    await tracker.start()
    video.cb!(0, { expectedDisplayTime: 1000 })
    expect(frameTs()).toBeCloseTo(1000 - 33, 0)
    const w = FakeWorker.last!
    const id = (w.posted.at(-1)!.msg as { id: number }).id
    w.onmessage!({
      data: {
        type: 'result',
        id,
        timestamp: 0,
        pose: null,
        corners: null,
        reprojErrorPx: NaN,
        detectMs: 1,
        K: intrinsicsFromSize(640, 360),
        gray: new ArrayBuffer(640 * 360),
      },
    })
    video.cb!(0, { expectedDisplayTime: 1020 }) // 20 ms frame period learned
    expect(frameTs()).toBeCloseTo(1020 - 20, 0)
    tracker.stop()
  })
  it('no metadata: performance.now() minus the default 40 ms latency', async () => {
    const { tracker, video } = mk({ useImu: false })
    await tracker.start()
    vi.spyOn(performance, 'now').mockReturnValue(7000)
    video.cb!()
    expect(frameTs()).toBe(6960)
    vi.restoreAllMocks()
    tracker.stop()
  })
  it('IMU events use ev.timeStamp', async () => {
    const { tracker } = mk()
    await tracker.start()
    expect(listeners['deviceorientation']).toBeDefined()
    listeners['deviceorientation']!({ alpha: 0, beta: 90, gamma: 0, timeStamp: 123 })
    // observable only through fusion; ensure it does not throw and imu stays active
    expect(tracker.stats().imu).toBe(true)
    tracker.stop()
  })
})

describe('recovery', () => {
  let docL: ReturnType<typeof makeTarget> & { visibilityState: string }
  beforeEach(() => {
    docL = { ...makeTarget(), visibilityState: 'visible' }
    vi.stubGlobal('document', docL)
  })
  const gumMock = (): ReturnType<typeof vi.fn> =>
    navigator.mediaDevices.getUserMedia as unknown as ReturnType<typeof vi.fn>

  it('track ended then visible: getUserMedia again and the loop resumes', async () => {
    const first = makeStream()
    const second = makeStream()
    const gum = vi.fn().mockResolvedValueOnce(first.stream).mockResolvedValueOnce(second.stream)
    vi.stubGlobal('navigator', { mediaDevices: { getUserMedia: gum } })
    const { tracker, video } = mk({ useImu: false })
    await tracker.start()
    const w = FakeWorker.last!
    docL.visibilityState = 'hidden'
    docL.emit('visibilitychange')
    first.track.readyState = 'ended'
    first.track.emit('ended')
    expect(gum).toHaveBeenCalledTimes(1)
    docL.visibilityState = 'visible'
    docL.emit('visibilitychange')
    await vi.waitFor(() => expect(gum).toHaveBeenCalledTimes(2))
    await vi.waitFor(() => expect(video.srcObject).toBe(second.stream))
    expect(FakeWorker.last).toBe(w) // worker reused
    expect(w.terminated).toBe(false)
    expect(first.track.stop).toHaveBeenCalled()
    const n = w.posted.filter((p) => p.msg.type === 'frame').length
    video.cb!()
    expect(w.posted.filter((p) => p.msg.type === 'frame').length).toBe(n + 1)
    tracker.stop()
  })

  it('hidden pauses the loop; visible resumes without a new camera when the track is live', async () => {
    const { tracker, video } = mk({ useImu: false })
    await tracker.start()
    docL.visibilityState = 'hidden'
    docL.emit('visibilitychange')
    expect(video.cancelVideoFrameCallback).toHaveBeenCalled()
    const w = FakeWorker.last!
    video.cb!() // a stale callback must not grab
    expect(w.posted.filter((p) => p.msg.type === 'frame')).toHaveLength(0)
    docL.visibilityState = 'visible'
    docL.emit('visibilitychange')
    expect(gumMock()).toHaveBeenCalledTimes(1)
    video.cb!()
    expect(w.posted.filter((p) => p.msg.type === 'frame')).toHaveLength(1)
    tracker.stop()
  })

  it('failed re-acquisition -> status error camera-interrupted', async () => {
    const first = makeStream()
    const gum = vi
      .fn()
      .mockResolvedValueOnce(first.stream)
      .mockRejectedValueOnce(Object.assign(new Error('gone'), { name: 'NotReadableError' }))
    vi.stubGlobal('navigator', { mediaDevices: { getUserMedia: gum } })
    const { tracker } = mk({ useImu: false })
    const seen: TrackerStatus[] = []
    tracker.onStatus((s) => seen.push(s))
    await tracker.start()
    first.track.readyState = 'ended'
    first.track.emit('ended')
    await vi.waitFor(() =>
      expect(seen.at(-1)).toEqual({ state: 'error', reason: 'camera-interrupted' }),
    )
  })

  it('restart() re-acquires and rejects with camera-interrupted on failure', async () => {
    const first = makeStream()
    const second = makeStream()
    const gum = vi
      .fn()
      .mockResolvedValueOnce(first.stream)
      .mockResolvedValueOnce(second.stream)
      .mockRejectedValueOnce(new Error('nope'))
    vi.stubGlobal('navigator', { mediaDevices: { getUserMedia: gum } })
    const { tracker, video } = mk({ useImu: false })
    await tracker.start()
    await tracker.restart()
    expect(video.srcObject).toBe(second.stream)
    await expect(tracker.restart()).rejects.toMatchObject({ code: 'camera-interrupted' })
  })

  it('stop() removes every listener', async () => {
    const { stream, track } = makeStream()
    vi.stubGlobal('navigator', { mediaDevices: { getUserMedia: vi.fn(async () => stream) } })
    const { tracker, video } = mk()
    await tracker.start()
    expect(video.count('pause')).toBe(1)
    expect(docL.count('visibilitychange')).toBe(1)
    expect(track.count('ended')).toBe(1)
    tracker.stop()
    expect(video.count('pause')).toBe(0)
    expect(docL.count('visibilitychange')).toBe(0)
    expect(track.count('ended')).toBe(0)
    expect(removed).toContain('deviceorientation')
  })
})

describe('concurrent start()', () => {
  it('two start() calls share one getUserMedia and both resolve after it', async () => {
    let resolveGum!: (s: unknown) => void
    const { stream } = makeStream()
    const gum = vi.fn(() => new Promise((r) => (resolveGum = r)))
    vi.stubGlobal('navigator', { mediaDevices: { getUserMedia: gum } })
    const { tracker } = mk({ useImu: false })
    const a = tracker.start()
    const b = tracker.start()
    expect(gum).toHaveBeenCalledTimes(1)
    let bDone = false
    void b.then(() => (bDone = true))
    await Promise.resolve()
    expect(bDone).toBe(false)
    resolveGum(stream)
    await Promise.all([a, b])
    expect(bDone).toBe(true)
    expect(gum).toHaveBeenCalledTimes(1)
    expect(FakeWorker.last).not.toBeNull()
    tracker.stop()
  })
})

describe('pose extras', () => {
  it('cameraYaw and flat are exposed', async () => {
    const { tracker, video } = mk({ useImu: false })
    await tracker.start()
    expect(tracker.getPose()).toMatchObject({ cameraYaw: 0, flat: true, source: 'none' })
    video.cb!()
    const w = FakeWorker.last!
    const id = (w.posted.at(-1)!.msg as { id: number }).id
    // camera looking towards -X: world-from-camera rotation +90 deg about Y
    const q = quatFromAxisAngle([0, 1, 0], Math.PI / 2)
    w.onmessage!({
      data: {
        type: 'result',
        id,
        timestamp: performance.now(),
        pose: { position: [0, 0.2, 0], quaternion: q },
        corners: [
          [1, 1],
          [2, 1],
          [2, 2],
          [1, 2],
        ],
        reprojErrorPx: 0.5,
        detectMs: 1,
        K: intrinsicsFromSize(640, 360),
        gray: new ArrayBuffer(640 * 360),
      },
    })
    const p = tracker.getPose()
    expect(p.source).toBe('marker')
    expect(p.flat).toBe(true)
    expect(p.cameraYaw).toBeCloseTo(Math.PI / 2, 5)
    tracker.stop()
  })
})

describe('cameraYawFromQuat', () => {
  const Y: Vec3 = [0, 1, 0]
  const X: Vec3 = [1, 0, 0]
  it('identity looks towards -Z: yaw 0', () => {
    expect(cameraYawFromQuat([0, 0, 0, 1])).toBeCloseTo(0, 9)
  })
  it('rotation about +Y by theta gives yaw theta (CCW from above)', () => {
    for (const d of [-170, -90, -30, 30, 90, 135]) {
      const th = (d * Math.PI) / 180
      expect(cameraYawFromQuat(quatFromAxisAngle(Y, th))).toBeCloseTo(th, 9)
    }
  })
  it('looking towards +X is -90 deg, towards +Z is 180 deg', () => {
    expect(cameraYawFromQuat(quatFromAxisAngle(Y, -Math.PI / 2))).toBeCloseTo(-Math.PI / 2, 9)
    expect(Math.abs(cameraYawFromQuat(quatFromAxisAngle(Y, Math.PI)))).toBeCloseTo(Math.PI, 9)
  })
  it('pitch does not change the heading', () => {
    const q = quatMultiply(quatFromAxisAngle(Y, 0.7), quatFromAxisAngle(X, -0.6))
    expect(cameraYawFromQuat(q)).toBeCloseTo(0.7, 9)
  })
  it('looking straight down uses the top of the screen', () => {
    // pitch -90 about X: forward (-Z) -> -Y. Screen top (+Y) -> -Z : heading 0.
    const down = quatFromAxisAngle(X, -Math.PI / 2)
    expect(cameraYawFromQuat(down)).toBeCloseTo(0, 6)
    const turned = quatMultiply(quatFromAxisAngle(Y, 0.5), down)
    expect(cameraYawFromQuat(turned)).toBeCloseTo(0.5, 6)
  })
})

describe('shortExposure (opt-in)', () => {
  const withTrack = (extra: object) => {
    const track = {
      stop: vi.fn(),
      readyState: 'live',
      applyConstraints: vi.fn(async () => {}),
      ...makeTarget(),
      ...extra,
    }
    vi.stubGlobal('navigator', {
      mediaDevices: { getUserMedia: vi.fn(async () => ({ getTracks: () => [track] })) },
    })
    return track
  }
  const flush = () => new Promise((r) => setTimeout(r, 0))

  it('off by default: never touches the camera track; stats expose frame diagnostics', async () => {
    const track = withTrack({})
    const { tracker, video } = mk({ useImu: false })
    await tracker.start()
    video.cb!()
    await flush()
    expect(track.applyConstraints).not.toHaveBeenCalled()
    const st = tracker.stats()
    expect(st.camera.shortExposure).toBe('off')
    expect(st.camFps).toBeGreaterThanOrEqual(0)
    expect(st.grabMs).toBeGreaterThanOrEqual(0)
    tracker.stop()
  })

  it('on + unsupported track: reports unsupported without throwing', async () => {
    const track = withTrack({})
    const { tracker } = mk({ useImu: false, shortExposure: true })
    await tracker.start()
    await flush()
    expect(track.applyConstraints).not.toHaveBeenCalled()
    expect(tracker.stats().camera.shortExposure).toBe('unsupported')
    tracker.stop()
  })

  it('on + supported track: manual exposure near the low end, 60 fps when available', async () => {
    const track = withTrack({
      getCapabilities: () => ({
        exposureMode: ['continuous', 'manual'],
        exposureTime: { min: 3, max: 2000, step: 1 },
        frameRate: { min: 1, max: 60 },
      }),
      getSettings: () => ({ exposureMode: 'manual', exposureTime: 60, frameRate: 60 }),
    })
    const { tracker } = mk({ useImu: false, shortExposure: true })
    await tracker.start()
    await flush()
    expect(track.applyConstraints).toHaveBeenCalledWith({
      advanced: [{ exposureMode: 'manual', exposureTime: 60 }],
    })
    expect(track.applyConstraints).toHaveBeenCalledWith({ frameRate: { ideal: 60 } })
    const cam = tracker.stats().camera
    expect(cam.shortExposure).toBe('applied')
    expect(cam.exposureTime).toEqual({ min: 3, max: 2000, step: 1 })
    expect(cam.settings.frameRate).toBe(60)
    tracker.stop()
  })
})
