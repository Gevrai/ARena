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
  /**
   * False when the latest marker sample disagreed with gravity (marker not lying flat), so the
   * pose follows the marker alone for that sample. True otherwise (and when there is no data).
   */
  flat: boolean
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
/** Marker tilt (swing from gravity-up) above which the gravity lock is not trusted. */
const MAX_SWING = (25 * Math.PI) / 180
const MIN_RESID_PX = 2
const RESID_FACTOR = 3
const NOT_FLAT_CONFIDENCE = 0.5

/** Angle of q away from a pure rotation about +Y (the swing part of swing-twist). */
function swingAngle(q: Quat): number {
  return quatAngle(q, yawTwist(q))
}

/** Keep only the rotation about +Y (swing-twist decomposition). */
function yawTwist(q: Quat): Quat {
  const n = Math.hypot(q[1], q[3])
  return n < 1e-9 ? IDENTITY : [0, q[1] / n, 0, q[3] / n]
}

/**
 * Position One Euro parameters for a 0..1 smoothing knob (0 = responsive, 1 = very smooth).
 * dCutoff is high so the speed estimate (and hence the adaptive cutoff) reacts within a frame or
 * two; with the old dCutoff of 1 Hz the filter lagged ~150 ms at motion onset.
 */
export function positionFilterParams(smoothing: number): {
  minCutoff: number
  beta: number
  dCutoff: number
} {
  const s = Math.min(1, Math.max(0, smoothing))
  return { minCutoff: 3 * Math.pow(0.1, s), beta: 40, dCutoff: 4 }
}

const PREDICT_MAX_SPEED_FADE = [0.03, 0.08] // m/s: no extrapolation below, full above
const PREDICT_DECAY_START_MS = 100
const PREDICT_DECAY_MS = 150

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
  private flat = true

  private position: Vec3 = [0, 0, 0]
  private readonly posFilter = new OneEuroVec3(1.0, 20)
  private predictMs: number
  private rawPos: Vec3 | null = null
  private rawT = 0
  private vel: Vec3 = [0, 0, 0]
  private lastFrameT = 0
  private syncPose: { position: Vec3; quaternion: Quat; timestamp: number } | null = null
  private readonly rotFilter = new OneEuroQuat(1.0, 2)
  private markerQuat: Quat = IDENTITY

  constructor(
    opts: {
      lostAfterMs?: number
      reacquireBlendMs?: number
      correctionRate?: number
      useImu?: boolean
      /** 0..1 position smoothing, default 0.5. */
      smoothing?: number
      /** Latency-compensation horizon in ms (0 = off), default 100. */
      predictMs?: number
    } = {},
  ) {
    this.setSmoothing(opts.smoothing ?? 0.5)
    this.predictMs = opts.predictMs ?? 100
    this.lostAfterMs = opts.lostAfterMs ?? 150
    this.blendMs = opts.reacquireBlendMs ?? 200
    this.rate = opts.correctionRate ?? 4
    this.useImu = opts.useImu ?? true
  }

  setSmoothing(smoothing: number): void {
    const p = positionFilterParams(smoothing)
    this.posFilter.setParams(p.minCutoff, p.beta, p.dCutoff)
  }

  setPredictMs(ms: number): void {
    this.predictMs = Math.max(0, ms)
  }

  /**
   * Pose matching the most recent marker frame exactly (unfiltered position, gravity-locked
   * rotation at the frame's capture time), for drawing over that frame's image.
   */
  frameSyncedPose(): { position: Vec3; quaternion: Quat; timestamp: number } | null {
    return this.syncPose
  }

  /** Track raw marker positions to estimate velocity for latency compensation. */
  private observe(pos: Vec3, frameTime: number, q: Quat): void {
    this.syncPose = { position: pos, quaternion: q, timestamp: frameTime }
    const dt = (frameTime - this.rawT) / 1000
    if (this.rawPos && dt > 0.005 && dt < 0.2) {
      const a = 0.3
      for (let i = 0; i < 3; i++) {
        const raw = ((pos[i] as number) - (this.rawPos[i] as number)) / dt
        this.vel[i] = (this.vel[i] as number) + a * (raw - (this.vel[i] as number))
      }
    } else if (!this.rawPos || dt >= 0.2) {
      this.vel = [0, 0, 0]
    }
    if (!this.rawPos || frameTime >= this.rawT) {
      this.rawPos = pos
      this.rawT = frameTime
      this.lastFrameT = frameTime
    }
  }

  /** Position extrapolated from the last marker frame to `now` (constant velocity, clamped). */
  private predictedPosition(now: number): Vec3 {
    if (this.predictMs <= 0 || this.lastArrivalT === null) return this.position
    const speed = Math.hypot(this.vel[0], this.vel[1], this.vel[2])
    const [lo, hi] = PREDICT_MAX_SPEED_FADE as [number, number]
    const gate = smoothstep(Math.min(1, Math.max(0, (speed - lo) / (hi - lo))))
    const since = now - this.lastArrivalT
    const decay = Math.min(1, Math.max(0, 1 - (since - PREDICT_DECAY_START_MS) / PREDICT_DECAY_MS))
    const e = Math.min(this.predictMs, Math.max(0, now - this.lastFrameT)) / 1000
    const k = e * gate * decay
    if (k <= 0) return this.position
    return [
      this.position[0] + this.vel[0] * k,
      this.position[1] + this.vel[1] * k,
      this.position[2] + this.vel[2] * k,
    ]
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
      this.observe(m.position, frameTime, m.quaternion)
      return
    }

    const full = quatMultiply(m.quaternion, quatInvert(imuQ))
    const target = yawTwist(full)
    const fusedForTarget = quatMultiply(target, imuQ)
    const R = worldFromCameraQuatToCvR(fusedForTarget)
    const sol = solveTranslationGivenRotation(m.corners, m.K, m.markerSizeM, R)
    const swingBad = swingAngle(full) > MAX_SWING
    const residBad =
      sol === null || sol.reprojErrorPx > Math.max(MIN_RESID_PX, RESID_FACTOR * m.reprojErrorPx)
    if (swingBad || residBad) {
      // Marker is not flat w.r.t. gravity: trust the marker alone for this sample.
      this.flat = false
      this.lastConfidence *= NOT_FLAT_CONFIDENCE
      this.markerQuat = this.rotFilter.filter(m.quaternion, tSec)
      this.position = this.posFilter.filter(m.position, tSec)
      this.observe(m.position, frameTime, m.quaternion)
      return
    }
    this.flat = true

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

    // Position is solved with the smoothed offset, not the raw target.
    const fusedQ = quatMultiply(this.offset, imuQ)
    const Rf = worldFromCameraQuatToCvR(fusedQ)
    const solF = solveTranslationGivenRotation(m.corners, m.K, m.markerSizeM, Rf)
    const pos = solF
      ? poseToWorldFromCamera({ R: Rf, t: solF.t, reprojErrorPx: 0 }).position
      : m.position
    this.position = this.posFilter.filter(pos, tSec)
    this.observe(pos, frameTime, fusedQ)
  }

  get(now: number): FusedPose {
    const position = this.predictedPosition(now)
    const latest = this.imu.latest()
    const age = this.lastArrivalT === null ? Infinity : now - this.lastArrivalT
    const fresh = age <= this.lostAfterMs
    const imuLive = this.useImu && latest !== null && now - latest.t < IMU_LIVE_MS

    if (this.useImu && latest && this.offset) {
      const q = quatNormalize(quatMultiply(this.displayedOffset(now), latest.q))
      if (fresh && !this.flat)
        return {
          position,
          quaternion: this.markerQuat,
          source: 'marker',
          confidence: this.lastConfidence,
          flat: false,
        }
      if (fresh)
        return {
          position,
          quaternion: q,
          source: 'marker',
          confidence: this.lastConfidence,
          flat: true,
        }
      if (imuLive) {
        const conf = 0.5 * this.lastConfidence * Math.exp(-age / 3000)
        return {
          position,
          quaternion: q,
          source: 'imu',
          confidence: conf,
          flat: this.flat,
        }
      }
      return {
        position,
        quaternion: q,
        source: 'none',
        confidence: 0,
        flat: this.flat,
      }
    }
    if (this.useImu && latest && this.lastMarkerT === null) {
      return {
        position,
        quaternion: latest.q,
        source: 'none',
        confidence: 0,
        flat: true,
      }
    }
    // Marker-only.
    if (this.lastMarkerT === null) {
      return {
        position,
        quaternion: IDENTITY,
        source: 'none',
        confidence: 0,
        flat: true,
      }
    }
    return {
      position,
      quaternion: this.markerQuat,
      source: fresh ? 'marker' : 'none',
      confidence: fresh ? this.lastConfidence : 0,
      flat: this.flat,
    }
  }
}
