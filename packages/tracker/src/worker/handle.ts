import { detectFramedQr } from '../detect/framedQr'
import type { DetectScratch, Quad } from '../detect/framedQr'
import { buildQrGrid } from '../marker/layout'
import type { MarkerGrid } from '../marker/layout'
import { estimatePose, poseToWorldFromCamera } from '../pose/pose'
import type { FromWorker, ToWorker } from './protocol'

export interface WorkerState {
  grid: MarkerGrid | null
  markerSizeM: number
  scratch: DetectScratch
  initError: string | null
  /** Last detection, passed to the detector as an orientation / proximity hint (motion blur). */
  hint: Quad | null
  hintMisses: number
  hintChain: number
  /** Detector override (tests); defaults to detectFramedQr. */
  detect?: typeof detectFramedQr
}

export function createWorkerState(): WorkerState {
  return { grid: null, markerSizeM: 0.05, scratch: {}, initError: null, hint: null, hintMisses: 0, hintChain: 0 }
}

const HINT_MAX_MISSES = 8
const HINT_MAX_CHAIN = 12

const nowMs = (): number => globalThis.performance?.now() ?? Date.now()

function errorResult(
  msg: Extract<ToWorker, { type: 'frame' }>,
  error: string,
  t0: number,
): FromWorker {
  return {
    type: 'result',
    id: msg.id,
    timestamp: msg.timestamp,
    pose: null,
    corners: null,
    reprojErrorPx: NaN,
    detectMs: nowMs() - t0,
    K: msg.K,
    gray: msg.gray,
    error,
  }
}

/**
 * Pure message handler (no DOM / worker globals). Returns null only for 'init'.
 * Every 'frame' yields a result that returns the gray buffer, with `error` set on failure.
 */
export function handleMessage(msg: ToWorker, state: WorkerState): FromWorker | null {
  if (msg.type === 'init') {
    try {
      state.grid = buildQrGrid(msg.url)
      state.markerSizeM = msg.markerSizeM
      state.initError = null
    } catch (e) {
      state.grid = null
      state.initError = e instanceof Error ? e.message : String(e)
    }
    return null
  }
  const t0 = nowMs()
  if (!state.grid) {
    return errorResult(msg, `not initialized${state.initError ? `: ${state.initError}` : ''}`, t0)
  }
  if (msg.gray.byteLength !== msg.width * msg.height) {
    return errorResult(
      msg,
      `size mismatch: ${msg.gray.byteLength} bytes for ${msg.width}x${msg.height}`,
      t0,
    )
  }
  try {
    const img = { width: msg.width, height: msg.height, data: new Uint8Array(msg.gray) }
    const det = (state.detect ?? detectFramedQr)(
      img,
      state.grid,
      state.hint ? { hint: state.hint } : {},
      state.scratch,
    )
    if (det) {
      state.hint = det.corners
      state.hintMisses = 0
      // A hint-only chain is capped so a wrong lock cannot persist: a full QR read is then needed.
      state.hintChain = det.viaHint ? state.hintChain + 1 : 0
      if (state.hintChain > HINT_MAX_CHAIN) {
        state.hint = null
        state.hintChain = 0
      }
    } else if (++state.hintMisses > HINT_MAX_MISSES) {
      state.hint = null
      state.hintChain = 0
    }
    let pose: FromWorker['pose'] = null
    let reprojErrorPx = NaN
    if (det) {
      const cv = estimatePose(det.corners, msg.K, state.markerSizeM)
      if (cv) {
        const w = poseToWorldFromCamera(cv)
        pose = { position: w.position, quaternion: w.quaternion }
        reprojErrorPx = cv.reprojErrorPx
      }
    }
    return {
      type: 'result',
      id: msg.id,
      timestamp: msg.timestamp,
      pose,
      corners: det ? det.corners : null,
      reprojErrorPx,
      detectMs: nowMs() - t0,
      K: msg.K,
      gray: msg.gray,
    }
  } catch (e) {
    return errorResult(msg, e instanceof Error ? e.message : String(e), t0)
  }
}
