/* eslint-disable @typescript-eslint/no-non-null-assertion, @typescript-eslint/no-extraneous-class -- test stubs */
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { PoseFusion } from '../src/fusion/fusion'
import { createTracker } from '../src/tracker'
import { intrinsicsFromSize } from '../src/pose/intrinsics'
import type { Vec3 } from '../src/math/vec3'
import {
  cornersFor,
  mulberry32,
  restJitter,
  rotationError,
  sampleFromCorners,
  slideError,
  staticTruth,
} from './fusionSim'

describe('rotation in place (perfect gyro, delayed noisy markers)', () => {
  it('stays on the marker at 0.8 px corner noise, predict on and off', () => {
    // Regression: isolated marker-only outliers (single bad swing/residual samples) used to flip
    // the pose to the raw marker rotation and feed 5-10 cm position spikes into the filter and the
    // velocity estimate (prediction then amplified them to > 25 cm).
    for (const predictMs of [0, 100]) {
      const r = rotationError({ noisePx: 0.8, seconds: 12, fusion: { predictMs } })
      expect(r.meanPx).toBeLessThan(1.5)
      expect(r.maxPx).toBeLessThan(10)
      expect(r.rotDeg).toBeLessThan(0.5)
    }
  })
})

describe('rest jitter vs smoothing', () => {
  it('translation smoothing clearly reduces rest jitter; rotation is untouched by it', () => {
    const lo = restJitter({ fusion: { smoothing: 0, predictMs: 0 }, noisePx: 1, seconds: 10 })
    const hi = restJitter({ fusion: { smoothing: 1, predictMs: 0 }, noisePx: 1, seconds: 10 })
    expect(hi.posRmsMm).toBeLessThan(lo.posRmsMm * 0.85)
    expect(hi.yawRmsDeg).toBeCloseTo(lo.yawRmsDeg, 6)
  })
  it('1 px corner noise at rest never leaves the flat state', () => {
    const r = restJitter({ fusion: {}, noisePx: 1.5, seconds: 10 })
    expect(r.nonFlatFrac).toBe(0)
    expect(r.tiltRmsDeg).toBeLessThan(0.1)
    expect(r.posRmsMm).toBeLessThan(8)
  })
})

describe('prediction with intermittent detections', () => {
  it('cuts the error at 0.3 m/s when only 1 of 3 frames is detected', () => {
    for (const detectEvery of [1, 2, 3]) {
      const off = slideError({ speed: 0.3, detectEvery, fusion: { predictMs: 0 } })
      const on = slideError({ speed: 0.3, detectEvery, fusion: { predictMs: 100 } })
      expect(on.meanMm).toBeLessThan(off.meanMm * 0.75)
    }
  })
})

describe('flat hysteresis', () => {
  it('one wrong sample is ignored; a persistent one flips; coming back needs good samples', () => {
    const tr = staticTruth(25)
    const rnd = mulberry32(2)
    const good = sampleFromCorners(cornersFor(tr, 0, rnd))!
    const bad = sampleFromCorners(cornersFor(staticTruth(70), 0, rnd))! // way off gravity
    const f = new PoseFusion({ predictMs: 0 })
    // IMU from the true pose (identity heading offset is fine: only swing matters)
    f.onImu(0, tr.q)
    let t = 10
    f.onMarker(t, good, t)
    expect(f.get(t).flat).toBe(true)
    f.onMarker((t += 33), bad, t)
    expect(f.get(t).flat).toBe(true)
    f.onMarker((t += 33), good, t)
    f.onMarker((t += 33), bad, t)
    f.onMarker((t += 33), bad, t)
    expect(f.get(t).flat).toBe(true) // not 3 in a row
    f.onMarker((t += 33), bad, t)
    expect(f.get(t).flat).toBe(false)
    f.onMarker((t += 33), good, t)
    expect(f.get(t).flat).toBe(false)
    f.onMarker((t += 33), good, t)
    expect(f.get(t).flat).toBe(true)
  })
})

describe('createTracker option wiring', () => {
  let nowMs = 1000
  class FakeWorker {
    static last: FakeWorker | null = null
    onmessage: ((e: { data: unknown }) => void) | null = null
    onerror: unknown = null
    posted: { type: string; id?: number }[] = []
    constructor() {
      FakeWorker.last = this
    }
    postMessage(msg: { type: string; id?: number }) {
      this.posted.push(msg)
    }
    terminate() {}
  }
  beforeEach(() => {
    nowMs = 1000
    vi.spyOn(performance, 'now').mockImplementation(() => nowMs)
    vi.stubGlobal('isSecureContext', true)
    vi.stubGlobal('window', {
      addEventListener() {},
      removeEventListener() {},
      orientation: 0,
    })
    vi.stubGlobal('Worker', FakeWorker)
    vi.stubGlobal(
      'OffscreenCanvas',
      class {
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
    const track = { stop() {}, readyState: 'live', addEventListener() {}, removeEventListener() {} }
    vi.stubGlobal('navigator', {
      mediaDevices: { getUserMedia: async () => ({ getTracks: () => [track] }) },
    })
  })
  afterEach(() => {
    vi.unstubAllGlobals()
    vi.restoreAllMocks()
  })

  async function run(
    positionAt: (tMs: number) => Vec3,
    mutate: ((t: ReturnType<typeof createTracker>) => void) | null,
    extra: object,
  ): Promise<{ pos: Vec3[]; afterLast: Vec3; stats: ReturnType<ReturnType<typeof createTracker>['stats']> }> {
    const video = {
      srcObject: null as unknown,
      videoWidth: 640,
      videoHeight: 480,
      muted: false,
      play: async () => {},
      setAttribute() {},
      addEventListener() {},
      removeEventListener() {},
      cb: null as null | ((n?: number, md?: object) => void),
      requestVideoFrameCallback(cb: (n?: number, md?: object) => void) {
        this.cb = cb
        return 1
      },
      cancelVideoFrameCallback() {},
    }
    const tracker = createTracker({
      video: video as unknown as HTMLVideoElement,
      useImu: false,
      ...extra,
    })
    await tracker.start()
    if (mutate) mutate(tracker)
    const w = FakeWorker.last!
    const rnd = mulberry32(4)
    const base = staticTruth(25)
    const K = intrinsicsFromSize(640, 480, 65)
    const pos: Vec3[] = []
    for (let k = 0; k < 90; k++) {
      const tf = 1000 + k * 33
      nowMs = tf + 5
      video.cb!(undefined, { captureTime: tf })
      const frame = w.posted.filter((m) => m.type === 'frame').at(-1)!
      const p = positionAt(tf - 1000)
      const s = sampleFromCorners(cornersFor({ q: base.q, p }, 0.15, rnd))
      nowMs = tf + 80
      w.onmessage!({
        data: {
          type: 'result',
          id: frame.id,
          timestamp: tf,
          pose: s ? { position: s.position, quaternion: s.quaternion } : null,
          corners: s ? s.corners : null,
          reprojErrorPx: s ? s.reprojErrorPx : NaN,
          detectMs: 5,
          K,
          gray: new ArrayBuffer(640 * 480),
        },
      })
      if (k > 30) pos.push(tracker.getPose().position)
    }
    nowMs += 40
    const afterLast = tracker.getPose().position
    const stats = tracker.stats()
    tracker.stop()
    return { pos, afterLast, stats }
  }

  const still = (): Vec3 => staticTruth(25).p
  const jitter = (pts: Vec3[]): number => {
    const m = [0, 1, 2].map((i) => pts.reduce((a, p) => a + (p[i] as number), 0) / pts.length)
    return Math.sqrt(
      pts.reduce((a, p) => a + Math.hypot(p[0] - m[0]!, p[1] - m[1]!, p[2] - m[2]!) ** 2, 0) /
        pts.length,
    )
  }

  it('setOptions({smoothing}) after start changes getPose jitter', async () => {
    const lo = await run(still, (t) => t.setOptions({ smoothing: 0 }), {})
    const hi = await run(still, (t) => t.setOptions({ smoothing: 1 }), {})
    expect(jitter(hi.pos)).toBeLessThan(jitter(lo.pos) * 0.8)
  })
  it('stats expose the per-frame outcome log and the 2 s hit rate', async () => {
    const r = await run(still, null, {})
    expect(r.stats.hitRate2s).toBeGreaterThan(0.9)
    expect(r.stats.recent.length).toBeGreaterThan(10)
    expect(r.stats.recent.every((f) => ['hit', 'nonflat', 'miss'].includes(f.outcome))).toBe(true)
  })
  it('constructor smoothing and runtime smoothing agree', async () => {
    const a = await run(still, null, { smoothing: 1 })
    const b = await run(still, (t) => t.setOptions({ smoothing: 1 }), { smoothing: 0 })
    expect(jitter(a.pos)).toBeCloseTo(jitter(b.pos), 9)
  })
  it('setOptions({predictMs}) moves getPose ahead along the motion', async () => {
    const base = staticTruth(25).p
    const moving = (t: number): Vec3 => [base[0] + (0.3 * t) / 1000, base[1], base[2]]
    const off = await run(moving, (t) => t.setOptions({ predictMs: 0 }), {})
    const on = await run(moving, (t) => t.setOptions({ predictMs: 100 }), {})
    const last = off.pos.length - 1
    // later x == further along the motion (+X)
    expect(on.pos[last]![0] - off.pos[last]![0]).toBeGreaterThan(0.01)
  })
})
