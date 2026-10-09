export const VERSION = '0.0.1'
export * from './marker/layout'
export { renderMarkerSvg, renderCardSvg } from './marker/svg'
export { detectFramedQr } from './detect/framedQr'
export type { Detection, DetectOptions, DetectScratch } from './detect/framedQr'
export { refineCorners } from './detect/refine'
export { homographyFromQuad, applyHomography } from './cv/homography'
export { deviceOrientationToQuat, requestMotionPermission } from './imu/orientation'
export { ImuHistory } from './imu/history'
export { OneEuroFilter } from './fusion/oneEuro'
export { PoseFusion } from './fusion/fusion'
export type { FusedPose, MarkerSample } from './fusion/fusion'
export { createTracker } from './tracker'
export type {
  Tracker,
  TrackerOptions,
  TrackerPose,
  TrackerStatus,
  TrackerError,
  TrackerStats,
  SyncedFrame,
  FrameOutcome,
} from './tracker'
export { intrinsicsFromSize, projectionForCover } from './pose/intrinsics'
export type { Intrinsics } from './pose/intrinsics'
