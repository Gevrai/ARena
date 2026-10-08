import { detectFramedQr } from '../detect/framedQr'
import type { DetectScratch } from '../detect/framedQr'
import { buildQrGrid } from '../marker/layout'
import type { MarkerGrid } from '../marker/layout'
import { estimatePose, poseToWorldFromCamera } from '../pose/pose'
import type { FromWorker, ToWorker } from './protocol'

export interface WorkerState {
  grid: MarkerGrid | null
  markerSizeM: number
  scratch: DetectScratch
}

export function createWorkerState(): WorkerState {
  return { grid: null, markerSizeM: 0.05, scratch: {} }
}

const nowMs = (): number => globalThis.performance?.now() ?? Date.now()

/**
 * Pure message handler (no DOM / worker globals). Returns the result to post, or null
 * if nothing should be posted (init, or a frame before init).
 */
export function handleMessage(msg: ToWorker, state: WorkerState): FromWorker | null {
  if (msg.type === 'init') {
    state.grid = buildQrGrid(msg.url)
    state.markerSizeM = msg.markerSizeM
    return null
  }
  if (!state.grid) return null
  const t0 = nowMs()
  const img = { width: msg.width, height: msg.height, data: new Uint8Array(msg.gray) }
  const det = detectFramedQr(img, state.grid, {}, state.scratch)
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
}
