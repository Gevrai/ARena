import { quatFromAxisAngle, quatMultiply } from '../math/quat'
import type { Quat } from '../math/quat'

const DEG = Math.PI / 180
const Q_BACK: Quat = [-Math.SQRT1_2, 0, 0, Math.SQRT1_2]

/** Intrinsic 'YXZ' euler (radians) to quaternion: qY * qX * qZ. */
function eulerYXZ(x: number, y: number, z: number): Quat {
  const c1 = Math.cos(x / 2)
  const c2 = Math.cos(y / 2)
  const c3 = Math.cos(z / 2)
  const s1 = Math.sin(x / 2)
  const s2 = Math.sin(y / 2)
  const s3 = Math.sin(z / 2)
  return [
    s1 * c2 * c3 + c1 * s2 * s3,
    c1 * s2 * c3 - s1 * c2 * s3,
    c1 * c2 * s3 - s1 * s2 * c3,
    c1 * c2 * c3 + s1 * s2 * s3,
  ]
}

/**
 * three.js DeviceOrientationControls math: euler(beta, alpha, -gamma, 'YXZ') · q(-√½,0,0,√½) ·
 * axisAngle(z, -screenAngle). Angles in degrees in, quaternion out (device camera orientation in an
 * arbitrary IMU world frame, Y up).
 */
export function deviceOrientationToQuat(
  alpha: number,
  beta: number,
  gamma: number,
  screenAngleDeg: number,
): Quat {
  const q = eulerYXZ(beta * DEG, alpha * DEG, -gamma * DEG)
  const q2 = quatMultiply(q, Q_BACK)
  return quatMultiply(q2, quatFromAxisAngle([0, 0, 1], -screenAngleDeg * DEG))
}

type PermissionApi = { requestPermission?: () => Promise<string> }

/** iOS requires a user-gesture permission request for motion events. */
export async function requestMotionPermission(): Promise<'granted' | 'denied' | 'unsupported'> {
  if (typeof window === 'undefined' || typeof DeviceOrientationEvent === 'undefined')
    return 'unsupported'
  const api = DeviceOrientationEvent as unknown as PermissionApi
  if (typeof api.requestPermission !== 'function') return 'granted'
  try {
    return (await api.requestPermission()) === 'granted' ? 'granted' : 'denied'
  } catch {
    return 'denied'
  }
}

/** iOS requires a user-gesture permission request for DeviceMotion (accelerometer) events too. */
export async function requestAccelPermission(): Promise<'granted' | 'denied' | 'unsupported'> {
  if (typeof window === 'undefined' || typeof DeviceMotionEvent === 'undefined')
    return 'unsupported'
  const api = DeviceMotionEvent as unknown as PermissionApi
  if (typeof api.requestPermission !== 'function') return 'granted'
  try {
    return (await api.requestPermission()) === 'granted' ? 'granted' : 'denied'
  } catch {
    return 'denied'
  }
}
