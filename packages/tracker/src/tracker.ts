import type { Detection } from './detect/framedQr'
import { PoseFusion } from './fusion/fusion'
import type { FusedPose } from './fusion/fusion'
import { deviceOrientationToQuat, requestMotionPermission } from './imu/orientation'
import { DEFAULT_URL } from './marker/layout'
import { mat4FromRotationTranslation } from './math/mat4'
import { cameraYawFromQuat } from './math/quat'
import type { Quat } from './math/quat'
import type { Vec3 } from './math/vec3'
import { toGray } from './cv/image'
import type { GrayImage } from './cv/image'
import { intrinsicsFromSize, projectionForCover } from './pose/intrinsics'
import { FramePump } from './worker/framePump'
import type { FromWorker, ToWorker } from './worker/protocol'

export type TrackerError =
  'camera-denied' | 'no-camera' | 'insecure-context' | 'camera-interrupted' | 'unknown'
export type TrackerStatus = {
  state: 'idle' | 'starting' | 'tracking' | 'lost' | 'error'
  reason?: TrackerError
}
export interface TrackerOptions {
  video: HTMLVideoElement
  /** Marker outer size in mm. Default 50. */
  markerSizeMm?: number
  /**
   * Assumed camera FOV in degrees along the video's LONG side (max(videoWidth, videoHeight)), so
   * portrait (rotated) and landscape video share one focal length. Default 65.
   */
  fovDeg?: number
  /** @deprecated Alias of `fovDeg` (it was applied to the video width, wrong in portrait). */
  hfovDeg?: number
  /**
   * Capture-to-grab latency in ms, subtracted from the frame timestamp. Default 0 when the
   * requestVideoFrameCallback metadata provides `captureTime`, 40 on the rAF / no-metadata path.
   */
  captureLatencyMs?: number
  /** Detection image width in px. Default 640. */
  detectWidth?: number
  /** Fuse device orientation. Default true. */
  useImu?: boolean
  /** URL encoded in the marker. Default DEFAULT_URL. */
  url?: string
  /** Position smoothing 0..1 (0 = responsive, 1 = very smooth; speed-adaptive). Default 0.5. */
  smoothing?: number
  /**
   * Latency compensation: extrapolate the marker position at constant velocity up to this many ms
   * ahead of the last analysed frame (0 = off). Default 100.
   */
  predictMs?: number
  /**
   * Keep the last analysed frame (colour ImageData) with its exact pose for
   * {@link Tracker.getSyncedFrame}. Costs one copy of the detect-sized frame. Default false.
   */
  keepFrames?: boolean
}
/** The last analysed camera frame and the pose that matches it exactly (no prediction). */
export interface SyncedFrame {
  /** Detect-resolution RGBA image (same aspect as the video, `detectWidth` wide). */
  image: ImageData
  /** Pose for this image: draw it over `image` instead of the live video. */
  pose: TrackerPose
  /** Capture timestamp (performance.now() clock), ms. */
  timestamp: number
}
/**
 * Camera pose in the marker (world) frame.
 *
 * World frame: origin at the marker centre, +Y up (out of the card), +X towards the card's right
 * edge, +Z towards the card's bottom edge. Units are metres. The camera frame is the three.js one
 * (+X right, +Y up, looking down -Z).
 *
 * Show game content when `source !== 'none'`: 'imu' means the marker was seen recently and the
 * pose is dead-reckoned from the gyro (status 'lost' includes imu-only), 'none' means there is no
 * usable pose and `matrix` is identity (hide content).
 */
export interface TrackerPose {
  /**
   * World-from-camera transform, column-major, metres: assign it directly to a three.js
   * `camera.matrixWorld` (with matrixAutoUpdate = false). A fresh array on every getPose() call.
   */
  matrix: Float32Array
  /** Camera position in the world frame, metres. */
  position: Vec3
  /** Camera orientation (world-from-camera) as a unit quaternion [x, y, z, w]. */
  quaternion: Quat
  source: 'marker' | 'imu' | 'none'
  /** 0..1, lowered by reprojection error, IMU-only age and non-flat markers. */
  confidence: number
  /**
   * True when the marker lies flat (consistent with gravity) so the gravity-locked fusion is in
   * use. False when the latest marker sample disagreed with gravity (swing > 25 degrees or the
   * gravity-locked reprojection residual much larger than the marker-only one, e.g. the card
   * shown on a propped-up phone): the pose then follows the marker alone for that sample and
   * confidence is halved. Always true when there is no pose.
   */
  flat: boolean
  /**
   * Camera heading about world +Y in radians, in (-PI, PI]: the angle of the camera's forward
   * vector (-Z) projected on the XZ plane. 0 = looking towards world -Z (the card's top edge),
   * positive = counter-clockwise seen from above (looking towards -X is +PI/2). When looking
   * straight down the direction of the top of the screen is used. 0 when source is 'none'.
   * Use it to rotate joystick input into the world frame.
   */
  cameraYaw: number
}
/** Per-frame outcome for diagnostics: 'hit' marker detected (flat), 'nonflat' detected but not gravity-flat, 'miss' no marker. */
export type FrameOutcome = 'hit' | 'nonflat' | 'miss'
export interface TrackerStats {
  /** Share of analysed frames in the last 2 s with a marker pose (0..1; NaN when none). */
  hitRate2s: number
  /** Outcomes of the last ~3 s of analysed frames, oldest first (t = arrival, performance.now()). */
  recent: { t: number; outcome: FrameOutcome }[]
  detectHz: number
  detectMs: number
  reprojErrorPx: number
  imu: boolean
  corners: Detection['corners'] | null
  /** Latest capture-to-result latency in ms (frame capture -> detection result received). */
  latencyMs: number
  videoW: number
  videoH: number
  /** Last worker-reported error message, if any (extension to the original brief). */
  lastError?: string
}
/**
 * AR marker tracker. World frame: origin at the marker centre, +Y up, +X to the card's right,
 * +Z to the card's bottom, metres; see {@link TrackerPose}. Show content when
 * `getPose().source !== 'none'` (status 'lost' includes imu-only dead reckoning).
 */
export interface Tracker {
  /**
   * Call from a user gesture. Rejects with an Error whose `.code` is a TrackerError. Calling it
   * while already starting returns the in-flight promise.
   */
  start(): Promise<void>
  stop(): void
  /**
   * Re-acquire the camera (reusing the worker) after an interruption, or start from scratch when
   * not running. Rejects with code 'camera-interrupted' (status error) if the camera is gone.
   * Normally unnecessary: backgrounding and track end are recovered automatically.
   */
  restart(): Promise<void>
  /** Fused pose; source 'none' returns identity (hide content). Allocates on every call. */
  getPose(): TrackerPose
  /** Projection for a canvas of viewW×viewH showing the video with object-fit: cover. */
  projectionMatrix(viewW: number, viewH: number, near: number, far: number): Float32Array
  /** Fires on state transitions only. Returns an unsubscribe function. */
  onStatus(cb: (s: TrackerStatus) => void): () => void
  stats(): TrackerStats
  /** Last analysed frame + matching pose (needs `keepFrames`); null until a marker was seen. */
  getSyncedFrame(): SyncedFrame | null
  /**
   * Change options at runtime. Changing `useImu` resets tracking (fusion state is discarded), so
   * do not use it mid-game.
   */
  setOptions(
    o: Partial<
      Pick<TrackerOptions, 
        | 'fovDeg'
        | 'hfovDeg'
        | 'detectWidth'
        | 'useImu'
        | 'captureLatencyMs'
        | 'smoothing'
        | 'predictMs'
        | 'keepFrames'
      >
    >,
  ): void
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

/**
 * requestVideoFrameCallback metadata. captureTime / expectedDisplayTime are DOMHighResTimeStamps
 * on the performance.now() clock (HTML spec; Chrome: captureTime only for local camera/WebRTC
 * frames; Safari/Firefox do not provide it, so those use the fallbacks). Not verified on device.
 */
type FrameMeta = { captureTime?: number; expectedDisplayTime?: number }
type VideoWithRvfc = HTMLVideoElement & {
  requestVideoFrameCallback?: (cb: (now: number, md: FrameMeta) => void) => number
  cancelVideoFrameCallback?: (h: number) => void
}
type Canvas2D = {
  drawImage(img: CanvasImageSource, x: number, y: number, w: number, h: number): void
  getImageData(x: number, y: number, w: number, h: number): ImageData
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

function fusedToPose(f: FusedPose): TrackerPose {
  if (f.source === 'none') {
    const q: Quat = [0, 0, 0, 1]
    const p: Vec3 = [0, 0, 0]
    return {
      matrix: mat4FromRotationTranslation(q, p),
      position: p,
      quaternion: q,
      source: 'none',
      confidence: 0,
      flat: true,
      cameraYaw: 0,
    }
  }
  return {
    matrix: mat4FromRotationTranslation(f.quaternion, f.position),
    position: f.position,
    quaternion: f.quaternion,
    source: f.source,
    confidence: f.confidence,
    flat: f.flat,
    cameraYaw: cameraYawFromQuat(f.quaternion),
  }
}

export function createTracker(opts: TrackerOptions): Tracker {
  const video = opts.video as VideoWithRvfc
  const markerSizeM = (opts.markerSizeMm ?? 50) / 1000
  const url = opts.url ?? DEFAULT_URL
  let fovDeg = opts.fovDeg ?? opts.hfovDeg ?? 65
  let captureLatencyMs = opts.captureLatencyMs
  let detectWidth = opts.detectWidth ?? 640
  let useImu = opts.useImu ?? true
  let smoothing = opts.smoothing ?? 0.5
  let predictMs = opts.predictMs ?? 100
  let keepFrames = opts.keepFrames ?? false
  const fusionOpts = (): ConstructorParameters<typeof PoseFusion>[0] => ({
    useImu,
    smoothing,
    predictMs,
  })
  const pendingFrames = new Map<number, ImageData>()
  let synced: { image: ImageData; timestamp: number; pose: TrackerPose } | null = null

  let status: TrackerStatus = { state: 'idle' }
  const listeners = new Set<(s: TrackerStatus) => void>()
  const setStatus = (next: TrackerStatus): void => {
    if (next.state === status.state && next.reason === status.reason) return
    status = next
    for (const cb of [...listeners]) cb(next)
  }

  let fusion = new PoseFusion(fusionOpts())
  let pump = new FramePump()
  let worker: Worker | null = null
  let stream: MediaStream | null = null
  let imuActive = false
  let motionGranted = false
  let running = false
  let gen = 0
  let loopHandle: number | null = null
  let loopIsRvfc = false
  let nextId = 1
  let pool: PoolBuf[] = []
  let surface: (Surface & Canvas2D) | null = null
  let ctx: Canvas2D | null = null
  let ctxW = 0
  let ctxH = 0
  let startP: Promise<void> | null = null
  let reacquireP: Promise<void> | null = null
  let loopPaused = false
  let trackEnded = false
  let listenersOn = false
  let lastDisplayTime: number | null = null
  let framePeriodMs = 33

  let lastDetectMs = 0
  let lastLatencyMs = 0
  let lastReproj = NaN
  let lastCorners: Detection['corners'] | null = null
  let lastError: string | undefined
  const frameLog: { t: number; outcome: FrameOutcome }[] = []

  const videoSize = (): { w: number; h: number } =>
    video.videoWidth > 0 && video.videoHeight > 0
      ? { w: video.videoWidth, h: video.videoHeight }
      : FALLBACK_VIDEO

  const onOrientation = (e: Event): void => {
    const ev = e as DeviceOrientationEvent
    if (ev.alpha == null || ev.beta == null || ev.gamma == null) return
    // Event.timeStamp is on the performance.now() clock in current browsers; guard against an
    // epoch-based value (old Firefox/Safari) by falling back to now.
    const ts =
      Number.isFinite(ev.timeStamp) && ev.timeStamp < 1e11 ? ev.timeStamp : performance.now()
    fusion.onImu(ts, deviceOrientationToQuat(ev.alpha, ev.beta, ev.gamma, screenAngle()))
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
    lastLatencyMs = Math.max(0, now - r.timestamp)
    const img = pendingFrames.get(r.id)
    pendingFrames.delete(r.id)
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
      const sp = fusion.frameSyncedPose()
      // sp is stale when fusion ignored this sample as an outlier: never pair it with this image.
      if (img && sp && sp.timestamp === r.timestamp) {
        const f = fusedToPose({
          position: sp.position,
          quaternion: sp.quaternion,
          source: 'marker',
          confidence: 1,
          flat: true,
        })
        synced = { image: img, timestamp: r.timestamp, pose: f }
      }
    }
    const outcome: FrameOutcome = !(r.pose && r.corners)
      ? 'miss'
      : fusion.get(now).flat
        ? 'hit'
        : 'nonflat'
    frameLog.push({ t: now, outcome })
    while (frameLog.length && now - (frameLog[0] as { t: number }).t > 3000) frameLog.shift()
    // Drop frames whose results will never arrive (older than this one).
    for (const k of pendingFrames.keys()) if (k < r.id) pendingFrames.delete(k)
    refreshStatus()
  }

  const grab = (frameTime: number): void => {
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
    let sentId: number | null = null
    try {
      if (pb.buf.byteLength !== n) {
        pb.buf = new ArrayBuffer(n)
        pb.img = makePoolBuf(pb.buf, dw, dh).img
      }
      c.drawImage(video, 0, 0, dw, dh)
      const imgData = c.getImageData(0, 0, dw, dh)
      const data = imgData.data
      toGray(data, dw, dh, pb.img)
      sentId = nextId++
      if (keepFrames) pendingFrames.set(sentId, imgData as ImageData)
      postFrame(pb, sentId, dw, dh, frameTime)
    } catch (e) {
      lastError = e instanceof Error ? e.message : String(e)
      if (sentId !== null) pump.markDone(sentId)
      if (pb.buf.byteLength === 0) {
        pb.buf = new ArrayBuffer(n)
        pb.img = makePoolBuf(pb.buf, dw, dh).img
      }
      pool.push(pb)
    }
  }

  const postFrame = (pb: PoolBuf, id: number, dw: number, dh: number, frameTime: number): void => {
    if (!worker) return
    const msg: ToWorker = {
      type: 'frame',
      id,
      timestamp: frameTime,
      width: dw,
      height: dh,
      gray: pb.buf,
      K: intrinsicsFromSize(dw, dh, fovDeg),
    }
    pump.markSent(id)
    worker.postMessage(msg, [pb.buf])
  }

  /** Frame timestamp: captureTime, else expectedDisplayTime minus one frame, else now. */
  const frameTimeOf = (md: FrameMeta | undefined): number => {
    const ct = md?.captureTime
    if (typeof ct === 'number' && Number.isFinite(ct)) return ct - (captureLatencyMs ?? 0)
    const ed = md?.expectedDisplayTime
    if (typeof ed === 'number' && Number.isFinite(ed)) {
      if (lastDisplayTime !== null) {
        const d = ed - lastDisplayTime
        if (d >= 8 && d <= 100) framePeriodMs = d
      }
      lastDisplayTime = ed
      return ed - framePeriodMs
    }
    return performance.now() - (captureLatencyMs ?? 40)
  }

  const cancelLoop = (): void => {
    if (loopHandle !== null) {
      if (loopIsRvfc) video.cancelVideoFrameCallback?.(loopHandle)
      else if (typeof cancelAnimationFrame !== 'undefined') cancelAnimationFrame(loopHandle)
      loopHandle = null
    }
  }

  const schedule = (): void => {
    if (!running || loopPaused) return
    const tick = (_now?: number, md?: FrameMeta): void => {
      if (!running || loopPaused) return
      try {
        grab(frameTimeOf(md))
        refreshStatus()
      } finally {
        schedule()
      }
    }
    if (typeof video.requestVideoFrameCallback === 'function') {
      loopIsRvfc = true
      loopHandle = video.requestVideoFrameCallback(tick)
    } else {
      loopIsRvfc = false
      loopHandle = requestAnimationFrame(() => tick())
    }
  }

  const CAMERA_CONSTRAINTS: MediaStreamConstraints = {
    video: {
      facingMode: { ideal: 'environment' },
      width: { ideal: 1280 },
      height: { ideal: 720 },
    },
    audio: false,
  }

  const isHidden = (): boolean =>
    typeof document !== 'undefined' && document.visibilityState === 'hidden'

  const streamEnded = (): boolean =>
    stream?.getTracks().some((t) => t.readyState === 'ended') ?? false

  const watchTracks = (st: MediaStream | null, on: boolean): void => {
    st?.getTracks().forEach((t) => {
      if (on) t.addEventListener('ended', onTrackEnded)
      else t.removeEventListener('ended', onTrackEnded)
    })
  }

  const addLifecycleListeners = (): void => {
    if (listenersOn) return
    listenersOn = true
    watchTracks(stream, true)
    video.addEventListener('pause', onVideoPause)
    if (typeof document !== 'undefined') document.addEventListener('visibilitychange', onVisibility)
  }

  function removeLifecycleListeners(): void {
    if (!listenersOn) return
    listenersOn = false
    watchTracks(stream, false)
    video.removeEventListener('pause', onVideoPause)
    if (typeof document !== 'undefined')
      document.removeEventListener('visibilitychange', onVisibility)
  }

  /** Re-acquire the camera, reusing the worker. Honours the generation token. */
  const reacquire = (): Promise<void> => {
    if (reacquireP) return reacquireP
    const my = gen
    loopPaused = true
    cancelLoop()
    const p: Promise<void> = (async () => {
      try {
        if (typeof navigator === 'undefined' || !navigator.mediaDevices?.getUserMedia)
          throw fail('camera-interrupted', 'navigator.mediaDevices is not available')
        const st = await navigator.mediaDevices.getUserMedia(CAMERA_CONSTRAINTS)
        if (my !== gen) {
          st.getTracks().forEach((t) => t.stop())
          return
        }
        watchTracks(stream, false)
        stream?.getTracks().forEach((t) => t.stop())
        stream = st
        watchTracks(st, true)
        trackEnded = false
        video.srcObject = st
        await video.play()
        if (my !== gen) return
        loopPaused = false
        schedule()
      } catch (e) {
        if (my !== gen) return
        teardown()
        setStatus({ state: 'error', reason: 'camera-interrupted' })
        throw fail('camera-interrupted', e instanceof Error ? e.message : 'camera interrupted')
      }
    })()
    const tracked = p.finally(() => {
      if (reacquireP === tracked) reacquireP = null
    })
    reacquireP = tracked
    return tracked
  }

  /** Resume after a pause, re-acquiring the camera when its track has ended. */
  const recover = (): Promise<void> => {
    if (!running) return Promise.resolve()
    if (trackEnded || streamEnded()) return reacquire()
    if (loopPaused && !reacquireP) {
      loopPaused = false
      schedule()
    }
    Promise.resolve(video.play()).catch(() => undefined)
    return Promise.resolve()
  }
  const recoverQuiet = (): void => {
    recover().catch(() => undefined) // failure is already reported through the status
  }

  function onTrackEnded(): void {
    trackEnded = true
    if (!running) return
    loopPaused = true
    cancelLoop()
    if (!isHidden()) recoverQuiet()
  }
  function onVideoPause(): void {
    if (!running || isHidden() || reacquireP || loopPaused) return
    Promise.resolve(video.play()).catch(recoverQuiet)
  }
  function onVisibility(): void {
    if (!running) return
    if (isHidden()) {
      loopPaused = true
      cancelLoop()
      return
    }
    recoverQuiet()
  }

  const teardown = (): void => {
    running = false
    loopPaused = false
    cancelLoop()
    removeLifecycleListeners()
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
    trackEnded = false
    reacquireP = null
    if (video) video.srcObject = null
    pool = []
  }

  const startImpl = async (my: number): Promise<void> => {
    if (typeof isSecureContext !== 'undefined' && !isSecureContext)
      throw fail('insecure-context', 'Camera requires a secure context (https)')
    if (typeof navigator === 'undefined' || !navigator.mediaDevices?.getUserMedia)
      throw fail('no-camera', 'navigator.mediaDevices is not available')

    // Must be requested synchronously, inside the user gesture (iOS).
    const motionP = requestMotionPermission()
    const gumP = navigator.mediaDevices.getUserMedia(CAMERA_CONSTRAINTS)
    // Attach early so a rejection during the motion prompt is never unhandled.
    const gumSettled = gumP.then(
      (st) => ({ ok: true as const, st }),
      (err: unknown) => ({ ok: false as const, err }),
    )
    const motion = await motionP
    const got = await gumSettled
    if (got.ok && my !== gen) {
      got.st.getTracks().forEach((t) => t.stop())
      return
    }
    if (my !== gen) return
    if (!got.ok) {
      const code = mapCameraError(got.err)
      throw fail(code, got.err instanceof Error ? got.err.message : 'getUserMedia failed')
    }
    motionGranted = motion === 'granted'
    stream = got.st

    video.srcObject = stream
    video.setAttribute('playsinline', '')
    video.muted = true
    try {
      await video.play()
    } catch (e) {
      if (my !== gen) return
      throw fail('unknown', e instanceof Error ? e.message : 'video.play() failed')
    }
    if (my !== gen) return

    fusion = new PoseFusion(fusionOpts())
    pump = new FramePump()
    lastDetectMs = 0
    lastReproj = NaN
    lastCorners = null
    synced = null
    pendingFrames.clear()
    frameLog.length = 0
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
      teardown()
      setStatus({ state: 'error', reason: 'unknown' })
    }
    const init: ToWorker = { type: 'init', markerSizeM, url }
    worker.postMessage(init)

    if (useImu && motionGranted) {
      window.addEventListener('deviceorientation', onOrientation)
      imuActive = true
    }

    running = true
    loopPaused = false
    addLifecycleListeners()
    setStatus({ state: 'lost' })
    schedule()
  }

  const start = (): Promise<void> => {
    if (startP) return startP
    if (status.state === 'tracking' || status.state === 'lost') return Promise.resolve()
    const my = ++gen
    setStatus({ state: 'starting' })
    const p: Promise<void> = (async () => {
      try {
        await startImpl(my)
      } catch (e) {
        if (my !== gen) return // stopped meanwhile
        teardown()
        const code = (e as { code?: TrackerError }).code ?? 'unknown'
        setStatus({ state: 'error', reason: code })
        throw e instanceof Error && (e as { code?: TrackerError }).code
          ? e
          : fail('unknown', e instanceof Error ? e.message : String(e))
      }
    })()
    const tracked = p.finally(() => {
      if (startP === tracked) startP = null
    })
    startP = tracked
    return tracked
  }

  return {
    start,
    restart() {
      if (status.state === 'starting' && startP) return startP
      if (running) return reacquire()
      return start()
    },
    stop() {
      gen++
      startP = null
      teardown()
      setStatus({ state: 'idle' })
    },
    getPose() {
      return fusedToPose(fusion.get(performance.now()))
    },
    getSyncedFrame() {
      return synced
    },
    projectionMatrix(viewW, viewH, near, far) {
      const { w, h } = videoSize()
      return projectionForCover(intrinsicsFromSize(w, h, fovDeg), viewW, viewH, near, far)
    },
    onStatus(cb) {
      listeners.add(cb)
      return () => listeners.delete(cb)
    },
    stats() {
      const { w, h } = videoSize()
      const now = performance.now()
      const last2 = frameLog.filter((f) => now - f.t <= 2000)
      const out: TrackerStats = {
        hitRate2s: last2.length ? last2.filter((f) => f.outcome !== 'miss').length / last2.length : NaN,
        recent: frameLog.filter((f) => now - f.t <= 3000),
        detectHz: pump.stats().detectHz,
        detectMs: lastDetectMs,
        reprojErrorPx: lastReproj,
        imu: imuActive,
        corners: lastCorners,
        latencyMs: lastLatencyMs,
        videoW: video.videoWidth > 0 ? w : 0,
        videoH: video.videoHeight > 0 ? h : 0,
      }
      if (lastError !== undefined) out.lastError = lastError
      return out
    },
    setOptions(o) {
      if (o.hfovDeg !== undefined) fovDeg = o.hfovDeg
      if (o.fovDeg !== undefined) fovDeg = o.fovDeg
      if (o.captureLatencyMs !== undefined) captureLatencyMs = o.captureLatencyMs
      if (o.detectWidth !== undefined) detectWidth = o.detectWidth
      if (o.smoothing !== undefined) {
        smoothing = o.smoothing
        fusion.setSmoothing(smoothing)
      }
      if (o.predictMs !== undefined) {
        predictMs = o.predictMs
        fusion.setPredictMs(predictMs)
      }
      if (o.keepFrames !== undefined) {
        keepFrames = o.keepFrames
        if (!keepFrames) {
          pendingFrames.clear()
          synced = null
        }
      }
      if (o.useImu !== undefined && o.useImu !== useImu) {
        useImu = o.useImu
        fusion = new PoseFusion(fusionOpts())
        if (running && motionGranted) {
          window.removeEventListener('deviceorientation', onOrientation)
          if (useImu) window.addEventListener('deviceorientation', onOrientation)
          imuActive = useImu
        }
      }
    },
  }
}
