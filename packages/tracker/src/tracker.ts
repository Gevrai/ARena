import type { Detection } from './detect/framedQr'
import { PoseFusion } from './fusion/fusion'
import { deviceOrientationToQuat, requestMotionPermission } from './imu/orientation'
import { DEFAULT_URL } from './marker/layout'
import { mat4FromRotationTranslation } from './math/mat4'
import type { Quat } from './math/quat'
import type { Vec3 } from './math/vec3'
import { toGray } from './cv/image'
import type { GrayImage } from './cv/image'
import { intrinsicsFromSize, projectionForCover } from './pose/intrinsics'
import { FramePump } from './worker/framePump'
import type { FromWorker, ToWorker } from './worker/protocol'

export type TrackerError = 'camera-denied' | 'no-camera' | 'insecure-context' | 'unknown'
export type TrackerStatus = {
  state: 'idle' | 'starting' | 'tracking' | 'lost' | 'error'
  reason?: TrackerError
}
export interface TrackerOptions {
  video: HTMLVideoElement
  /** Marker outer size in mm. Default 50. */
  markerSizeMm?: number
  /** Assumed horizontal FOV in degrees. Default 65. */
  hfovDeg?: number
  /** Detection image width in px. Default 640. */
  detectWidth?: number
  /** Fuse device orientation. Default true. */
  useImu?: boolean
  /** URL encoded in the marker. Default DEFAULT_URL. */
  url?: string
}
export interface TrackerPose {
  /** World-from-camera, column-major (usable as three.js camera.matrixWorld). */
  matrix: Float32Array
  position: Vec3
  quaternion: Quat
  source: 'marker' | 'imu' | 'none'
  confidence: number
}
export interface TrackerStats {
  detectHz: number
  detectMs: number
  reprojErrorPx: number
  imu: boolean
  corners: Detection['corners'] | null
  videoW: number
  videoH: number
  /** Last worker-reported error message, if any (extension to the original brief). */
  lastError?: string
}
export interface Tracker {
  /** Call from a user gesture. Rejects with an Error whose `.code` is a TrackerError. */
  start(): Promise<void>
  stop(): void
  /** Fused pose; source 'none' returns identity (hide content). */
  getPose(): TrackerPose
  /** Projection for a canvas of viewW×viewH showing the video with object-fit: cover. */
  projectionMatrix(viewW: number, viewH: number, near: number, far: number): Float32Array
  /** Fires on state transitions only. Returns an unsubscribe function. */
  onStatus(cb: (s: TrackerStatus) => void): () => void
  stats(): TrackerStats
  setOptions(o: Partial<Pick<TrackerOptions, 'hfovDeg' | 'detectWidth' | 'useImu'>>): void
}

export type TrackerStartError = Error & { code: TrackerError }

function fail(code: TrackerError, message: string): TrackerStartError {
  return Object.assign(new Error(message), { code })
}

function mapCameraError(e: unknown): TrackerError {
  const name = (e as { name?: string } | null)?.name
  if (name === 'NotAllowedError' || name === 'SecurityError' || name === 'PermissionDeniedError')
    return 'camera-denied'
  if (
    name === 'NotFoundError' ||
    name === 'OverconstrainedError' ||
    name === 'DevicesNotFoundError'
  )
    return 'no-camera'
  return 'unknown'
}

type ScreenLike = { orientation?: { angle?: number } }
function screenAngle(): number {
  const s = (typeof screen !== 'undefined' ? screen : undefined) as ScreenLike | undefined
  const w = window as unknown as { orientation?: number }
  return s?.orientation?.angle ?? w.orientation ?? 0
}

type VideoWithRvfc = HTMLVideoElement & {
  requestVideoFrameCallback?: (cb: () => void) => number
  cancelVideoFrameCallback?: (h: number) => void
}
type Canvas2D = {
  drawImage(img: CanvasImageSource, x: number, y: number, w: number, h: number): void
  getImageData(x: number, y: number, w: number, h: number): { data: Uint8ClampedArray }
}
type Surface = { width: number; height: number; getContext(t: '2d', o: object): unknown }

interface PoolBuf {
  buf: ArrayBuffer
  img: GrayImage
}
const makePoolBuf = (buf: ArrayBuffer, w: number, h: number): PoolBuf => ({
  buf,
  img: { width: w, height: h, data: new Uint8Array(buf) },
})

const FALLBACK_VIDEO = { w: 1280, h: 720 }
const POOL_SIZE = 2

export function createTracker(opts: TrackerOptions): Tracker {
  const video = opts.video as VideoWithRvfc
  const markerSizeM = (opts.markerSizeMm ?? 50) / 1000
  const url = opts.url ?? DEFAULT_URL
  let hfovDeg = opts.hfovDeg ?? 65
  let detectWidth = opts.detectWidth ?? 640
  let useImu = opts.useImu ?? true

  let status: TrackerStatus = { state: 'idle' }
  const listeners = new Set<(s: TrackerStatus) => void>()
  const setStatus = (next: TrackerStatus): void => {
    if (next.state === status.state && next.reason === status.reason) return
    status = next
    for (const cb of [...listeners]) cb(next)
  }

  let fusion = new PoseFusion({ useImu })
  let pump = new FramePump()
  let worker: Worker | null = null
  let stream: MediaStream | null = null
  let imuActive = false
  let motionGranted = false
  let running = false
  let loopHandle: number | null = null
  let loopIsRvfc = false
  let nextId = 1
  let pool: PoolBuf[] = []
  let surface: (Surface & Canvas2D) | null = null
  let ctx: Canvas2D | null = null
  let ctxW = 0
  let ctxH = 0

  let lastDetectMs = 0
  let lastReproj = NaN
  let lastCorners: Detection['corners'] | null = null
  let lastError: string | undefined

  const videoSize = (): { w: number; h: number } =>
    video.videoWidth > 0 && video.videoHeight > 0
      ? { w: video.videoWidth, h: video.videoHeight }
      : FALLBACK_VIDEO

  const onOrientation = (e: Event): void => {
    const ev = e as DeviceOrientationEvent
    if (ev.alpha == null || ev.beta == null || ev.gamma == null) return
    fusion.onImu(
      performance.now(),
      deviceOrientationToQuat(ev.alpha, ev.beta, ev.gamma, screenAngle()),
    )
  }

  const refreshStatus = (): void => {
    if (!running) return
    setStatus({ state: fusion.get(performance.now()).source === 'marker' ? 'tracking' : 'lost' })
  }

  const ensureSurface = (w: number, h: number): Canvas2D | null => {
    if (ctx && ctxW === w && ctxH === h) return ctx
    if (!surface) {
      surface = (typeof OffscreenCanvas !== 'undefined'
        ? new OffscreenCanvas(w, h)
        : document.createElement('canvas')) as unknown as Surface & Canvas2D
    }
    surface.width = w
    surface.height = h
    ctx = surface.getContext('2d', { willReadFrequently: true }) as Canvas2D | null
    ctxW = w
    ctxH = h
    return ctx
  }

  const onResult = (r: FromWorker): void => {
    const now = performance.now()
    pump.markDone(r.id, now)
    if (r.gray.byteLength > 0 && pool.length < POOL_SIZE) {
      const n = r.gray.byteLength
      pool.push(makePoolBuf(r.gray, n, 1))
    }
    lastDetectMs = r.detectMs
    lastReproj = r.reprojErrorPx
    lastCorners = r.corners
    lastError = r.error
    if (r.pose && r.corners) {
      fusion.onMarker(
        r.timestamp,
        {
          position: r.pose.position,
          quaternion: r.pose.quaternion,
          corners: r.corners,
          K: r.K,
          markerSizeM,
          reprojErrorPx: r.reprojErrorPx,
        },
        now,
      )
    }
    refreshStatus()
  }

  const grab = (): void => {
    if (!worker) return
    if (!pump.canSend() || pool.length === 0) {
      pump.noteDropped()
      return
    }
    const { w: vw, h: vh } = videoSize()
    const dw = Math.max(16, Math.round(detectWidth))
    const dh = Math.max(16, Math.round((dw * vh) / vw))
    const c = ensureSurface(dw, dh)
    if (!c) return
    const pb = pool.pop()
    if (!pb) return
    const n = dw * dh
    if (pb.buf.byteLength !== n) {
      pb.buf = new ArrayBuffer(n)
      pb.img = makePoolBuf(pb.buf, dw, dh).img
    }
    c.drawImage(video, 0, 0, dw, dh)
    const data = c.getImageData(0, 0, dw, dh).data
    toGray(data, dw, dh, pb.img)
    const id = nextId++
    const msg: ToWorker = {
      type: 'frame',
      id,
      timestamp: performance.now(),
      width: dw,
      height: dh,
      gray: pb.buf,
      K: intrinsicsFromSize(dw, dh, hfovDeg),
    }
    pump.markSent(id)
    worker.postMessage(msg, [pb.buf])
  }

  const schedule = (): void => {
    if (!running) return
    const tick = (): void => {
      if (!running) return
      grab()
      refreshStatus()
      schedule()
    }
    if (typeof video.requestVideoFrameCallback === 'function') {
      loopIsRvfc = true
      loopHandle = video.requestVideoFrameCallback(tick)
    } else {
      loopIsRvfc = false
      loopHandle = requestAnimationFrame(tick)
    }
  }

  const teardown = (): void => {
    running = false
    if (loopHandle !== null) {
      if (loopIsRvfc) video.cancelVideoFrameCallback?.(loopHandle)
      else if (typeof cancelAnimationFrame !== 'undefined') cancelAnimationFrame(loopHandle)
      loopHandle = null
    }
    if (typeof window !== 'undefined')
      window.removeEventListener('deviceorientation', onOrientation)
    imuActive = false
    if (worker) {
      worker.onmessage = null
      worker.onerror = null
      worker.terminate()
      worker = null
    }
    stream?.getTracks().forEach((t) => t.stop())
    stream = null
    if (video) video.srcObject = null
    pool = []
  }

  const startImpl = async (): Promise<void> => {
    if (typeof isSecureContext !== 'undefined' && !isSecureContext)
      throw fail('insecure-context', 'Camera requires a secure context (https)')
    if (typeof navigator === 'undefined' || !navigator.mediaDevices?.getUserMedia)
      throw fail('no-camera', 'navigator.mediaDevices is not available')

    // Must be requested synchronously, inside the user gesture (iOS).
    const motionP = requestMotionPermission()
    const gumP = navigator.mediaDevices.getUserMedia({
      video: {
        facingMode: { ideal: 'environment' },
        width: { ideal: 1280 },
        height: { ideal: 720 },
      },
      audio: false,
    })
    const motion = await motionP
    motionGranted = motion === 'granted'
    try {
      stream = await gumP
    } catch (e) {
      const code = mapCameraError(e)
      throw fail(code, e instanceof Error ? e.message : 'getUserMedia failed')
    }

    video.srcObject = stream
    video.setAttribute('playsinline', '')
    video.muted = true
    try {
      await video.play()
    } catch (e) {
      throw fail('unknown', e instanceof Error ? e.message : 'video.play() failed')
    }

    fusion = new PoseFusion({ useImu })
    pump = new FramePump()
    lastDetectMs = 0
    lastReproj = NaN
    lastCorners = null
    lastError = undefined
    nextId = 1
    const { w, h } = videoSize()
    const dw = Math.round(detectWidth)
    const dh = Math.round((dw * h) / w)
    pool = Array.from({ length: POOL_SIZE }, () => makePoolBuf(new ArrayBuffer(dw * dh), dw, dh))

    worker = new Worker(new URL('./worker/detect.worker.ts', import.meta.url), { type: 'module' })
    worker.onmessage = (e: MessageEvent<FromWorker>): void => {
      if (e.data?.type === 'result') onResult(e.data)
    }
    worker.onerror = (e: ErrorEvent): void => {
      lastError = e.message || 'worker error'
    }
    const init: ToWorker = { type: 'init', markerSizeM, url }
    worker.postMessage(init)

    if (useImu && motionGranted) {
      window.addEventListener('deviceorientation', onOrientation)
      imuActive = true
    }

    running = true
    setStatus({ state: 'lost' })
    schedule()
  }

  return {
    async start() {
      if (status.state === 'starting' || status.state === 'tracking' || status.state === 'lost')
        return
      setStatus({ state: 'starting' })
      try {
        await startImpl()
      } catch (e) {
        teardown()
        const code = (e as { code?: TrackerError }).code ?? 'unknown'
        setStatus({ state: 'error', reason: code })
        throw e instanceof Error && (e as { code?: TrackerError }).code
          ? e
          : fail('unknown', e instanceof Error ? e.message : String(e))
      }
    },
    stop() {
      teardown()
      setStatus({ state: 'idle' })
    },
    getPose() {
      const f = fusion.get(performance.now())
      if (f.source === 'none') {
        const q: Quat = [0, 0, 0, 1]
        const p: Vec3 = [0, 0, 0]
        return {
          matrix: mat4FromRotationTranslation(q, p),
          position: p,
          quaternion: q,
          source: 'none',
          confidence: 0,
        }
      }
      return {
        matrix: mat4FromRotationTranslation(f.quaternion, f.position),
        position: f.position,
        quaternion: f.quaternion,
        source: f.source,
        confidence: f.confidence,
      }
    },
    projectionMatrix(viewW, viewH, near, far) {
      const { w, h } = videoSize()
      return projectionForCover(intrinsicsFromSize(w, h, hfovDeg), viewW, viewH, near, far)
    },
    onStatus(cb) {
      listeners.add(cb)
      return () => listeners.delete(cb)
    },
    stats() {
      const { w, h } = videoSize()
      const out: TrackerStats = {
        detectHz: pump.stats().detectHz,
        detectMs: lastDetectMs,
        reprojErrorPx: lastReproj,
        imu: imuActive,
        corners: lastCorners,
        videoW: video.videoWidth > 0 ? w : 0,
        videoH: video.videoHeight > 0 ? h : 0,
      }
      if (lastError !== undefined) out.lastError = lastError
      return out
    },
    setOptions(o) {
      if (o.hfovDeg !== undefined) hfovDeg = o.hfovDeg
      if (o.detectWidth !== undefined) detectWidth = o.detectWidth
      if (o.useImu !== undefined && o.useImu !== useImu) {
        useImu = o.useImu
        fusion = new PoseFusion({ useImu })
        if (running && motionGranted) {
          window.removeEventListener('deviceorientation', onOrientation)
          if (useImu) window.addEventListener('deviceorientation', onOrientation)
          imuActive = useImu
        }
      }
    },
  }
}
