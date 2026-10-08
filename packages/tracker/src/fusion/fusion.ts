import type { Detection } from '../detect/framedQr'
import { ImuHistory } from '../imu/history'
import { quatAngle, quatInvert, quatMultiply, quatNormalize, quatSlerp } from '../math/quat'
import type { Quat } from '../math/quat'
import type { Vec3 } from '../math/vec3'
import type { Intrinsics } from '../pose/intrinsics'
import {
  poseToWorldFromCamera,
  solveTranslationGivenRotation,
  worldFromCameraQuatToCvR,
} from '../pose/pose'
import { OneEuroQuat, OneEuroVec3 } from './oneEuro'

export interface FusedPose {
  position: Vec3
  quaternion: Quat
  source: 'marker' | 'imu' | 'none'
  confidence: number
}

export interface MarkerSample {
  /** Marker-only world-from-camera pose (used when no IMU is available). */
  position: Vec3
  quaternion: Quat
  corners: Detection['corners']
  K: Intrinsics
  markerSizeM: number
  reprojErrorPx: number
}

const IDENTITY: Quat = [0, 0, 0, 1]
const REACQUIRE_ANGLE = (20 * Math.PI) / 180
const IMU_LIVE_MS = 500

/** Keep only the rotation about +Y (swing-twist decomposition). */
function yawTwist(q: Quat): Quat {
  const n = Math.hypot(q[1], q[3])
  return n < 1e-9 ? IDENTITY : [0, q[1] / n, 0, q[3] / n]
}

const smoothstep = (p: number): number => p * p * (3 - 2 * p)

/**
 * Complementary pose filter. Times are milliseconds on one shared clock.
 * With IMU: worldFromCam = offset · imuQ, where offset is yaw-only (gravity-locked); position is
 * re-solved from the detection corners with the fused rotation. Without IMU the smoothed marker
 * pose is used directly.
 */
export class PoseFusion {
  private readonly lostAfterMs: number
  private readonly blendMs: number
  private readonly rate: number
  private readonly useImu: boolean
  private readonly imu = new ImuHistory(120)

  private offset: Quat | null = null
  private blendFrom: Quat | null = null
  private blendStart = 0
  private lastMarkerT: number | null = null // capture time (reacquire gap logic)
  private lastArrivalT: number | null = null // arrival time (freshness / source)
  private lastConfidence = 0

  private position: Vec3 = [0, 0, 0]
  private readonly posFilter = new OneEuroVec3(1.0, 20)
  private readonly rotFilter = new OneEuroQuat(1.0, 2)
  private markerQuat: Quat = IDENTITY

  constructor(
    opts: {
      lostAfterMs?: number
      reacquireBlendMs?: number
      correctionRate?: number
      useImu?: boolean
    } = {},
  ) {
    this.lostAfterMs = opts.lostAfterMs ?? 150
    this.blendMs = opts.reacquireBlendMs ?? 200
    this.rate = opts.correctionRate ?? 4
    this.useImu = opts.useImu ?? true
  }

  onImu(t: number, q: Quat): void {
    this.imu.push(t, q)
  }

  /** Offset as displayed at time `now` (includes the reacquire blend). */
  private displayedOffset(now: number): Quat {
    const off = this.offset ?? IDENTITY
    if (!this.blendFrom) return off
    const p = (now - this.blendStart) / this.blendMs
    if (p >= 1) {
      this.blendFrom = null
      return off
    }
    return quatSlerp(this.blendFrom, off, smoothstep(Math.max(0, p)))
  }

  onMarker(frameTime: number, m: MarkerSample, arrivalTime?: number): void {
    const latest = this.imu.latest()
    this.lastArrivalT = Math.max(
      arrivalTime ?? latest?.t ?? frameTime,
      this.lastArrivalT ?? -Infinity,
    )
    const imuQ = this.useImu && latest ? this.imu.at(frameTime) : null
    const prev = this.lastMarkerT
    const gap = prev === null ? Infinity : frameTime - prev
    this.lastMarkerT = Math.max(frameTime, prev ?? -Infinity)
    this.lastConfidence = Math.max(0.1, Math.min(1, 1 - m.reprojErrorPx / 3))
    const tSec = frameTime / 1000

    if (!latest || !imuQ) {
      // Marker-only mode.
      this.markerQuat = this.rotFilter.filter(m.quaternion, tSec)
      this.position = this.posFilter.filter(m.position, tSec)
      return
    }

    const target = yawTwist(quatMultiply(m.quaternion, quatInvert(imuQ)))
    if (this.offset === null) {
      this.offset = target
      this.blendFrom = null
    } else if (gap > this.lostAfterMs || quatAngle(this.offset, target) > REACQUIRE_ANGLE) {
      this.blendFrom = this.displayedOffset(latest.t)
      this.blendStart = latest.t
      this.offset = target
    } else {
      const a = 1 - Math.exp(-this.rate * (gap / 1000))
      this.offset = yawTwist(quatNormalize(quatSlerp(this.offset, target, a)))
    }

    const fusedQ = quatMultiply(this.offset, imuQ)
    const sol = solveTranslationGivenRotation(
      m.corners,
      m.K,
      m.markerSizeM,
      worldFromCameraQuatToCvR(fusedQ),
    )
    const pos = sol
      ? poseToWorldFromCamera({
          R: worldFromCameraQuatToCvR(fusedQ),
          t: sol.t,
          reprojErrorPx: 0,
        }).position
      : m.position
    this.position = this.posFilter.filter(pos, tSec)
  }

  get(now: number): FusedPose {
    const latest = this.imu.latest()
    const age = this.lastArrivalT === null ? Infinity : now - this.lastArrivalT
    const fresh = age <= this.lostAfterMs
    const imuLive = this.useImu && latest !== null && now - latest.t < IMU_LIVE_MS

    if (this.useImu && latest && this.offset) {
      const q = quatNormalize(quatMultiply(this.displayedOffset(now), latest.q))
      if (fresh)
        return {
          position: this.position,
          quaternion: q,
          source: 'marker',
          confidence: this.lastConfidence,
        }
      if (imuLive) {
        const conf = 0.5 * this.lastConfidence * Math.exp(-age / 3000)
        return { position: this.position, quaternion: q, source: 'imu', confidence: conf }
      }
      return { position: this.position, quaternion: q, source: 'none', confidence: 0 }
    }
    if (this.useImu && latest && this.lastMarkerT === null) {
      return { position: this.position, quaternion: latest.q, source: 'none', confidence: 0 }
    }
    // Marker-only.
    if (this.lastMarkerT === null) {
      return { position: this.position, quaternion: IDENTITY, source: 'none', confidence: 0 }
    }
    return {
      position: this.position,
      quaternion: this.markerQuat,
      source: fresh ? 'marker' : 'none',
      confidence: fresh ? this.lastConfidence : 0,
    }
  }
}
