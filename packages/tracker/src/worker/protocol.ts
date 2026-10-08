import type { Detection } from '../detect/framedQr'
import type { Quat } from '../math/quat'
import type { Vec3 } from '../math/vec3'
import type { Intrinsics } from '../pose/intrinsics'

export type ToWorker =
  | { type: 'init'; markerSizeM: number; url: string }
  | {
      type: 'frame'
      id: number
      timestamp: number
      width: number
      height: number
      /** width*height gray bytes; transferred to the worker. */
      gray: ArrayBuffer
      K: Intrinsics
    }

export type FromWorker = {
  type: 'result'
  id: number
  timestamp: number
  /** World-from-camera pose (three.js camera convention). Null if no detection OR pose solve failed. */
  pose: { position: Vec3; quaternion: Quat } | null
  /** Detected marker corners (image px, TL,TR,BR,BL); null if nothing detected. May be set while `pose` is null. */
  corners: Detection['corners'] | null
  /** RMS reprojection error in px. NaN when `pose` is null. */
  reprojErrorPx: number
  detectMs: number
  /** The K the frame was solved with (echoed so the main thread can re-solve from `corners`). */
  K: Intrinsics
  /** The gray buffer, transferred back for reuse. */
  gray: ArrayBuffer
  /** Set when the frame could not be processed (not initialised, bad size, exception). Pose/corners are null, reprojErrorPx NaN. */
  error?: string
}
