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
/** Marker tilt (swing from gravity-up) above which the gravity lock is not trusted (enter). */
const MAX_SWING = (25 * Math.PI) / 180
/** Swing below which a non-flat marker counts as flat again (hysteresis: exit < enter). */
const EXIT_SWING = (18 * Math.PI) / 180
/** Consecutive bad samples needed to leave the flat state (single outliers are ignored). */
const NONFLAT_ENTER_SAMPLES = 3
/** Consecutive good samples needed to return to the flat state. */
const FLAT_EXIT_SAMPLES = 2
/** A gap this long between marker frames restarts the marker-only filters. */
const FILTER_RESET_GAP_MS = 1000
const MIN_RESID_PX = 3
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
  deadband: number
} {
  const s = Math.min(1, Math.max(0, smoothing))
  // The deadband (m/s) keeps the speed estimate of measurement noise (~0.02-0.04 m/s at 30 Hz)
  // from opening the filter: without it beta * noise-speed dominated minCutoff and the knob did
  // nothing at rest.
  return { minCutoff: 3 * Math.pow(0.1, s), beta: 40, dCutoff: 4, deadband: 0.02 + 0.04 * s }
}

const PREDICT_MAX_SPEED_FADE = [0.03, 0.08] // m/s: no extrapolation below, full above
/** Beyond predictMs the extrapolation keeps going but saturates (time constant, s). */
const PREDICT_TAIL_TAU = 0.08
/** The extrapolation freezes this long after the last frame's capture (s). */
const PREDICT_MAX_E = 0.4
/** Velocity regression window (s) and the minimum time span of its samples (s). */
const VEL_WINDOW_S = 0.22
const VEL_MIN_SPAN_S = 0.05
/** Longest gap between two detections over which a velocity is still estimated (s). */
const VEL_MAX_DT = 0.35

interface PosSample {
  t: number
  p: Vec3
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
  private flat = true

  private position: Vec3 = [0, 0, 0]
  private readonly posFilter = new OneEuroVec3(1.0, 20)
  private predictMs: number
  private rawPos: Vec3 | null = null
  private rawT = 0
  private vel: Vec3 = [0, 0, 0]
  private readonly hist: PosSample[] = []
  private lastFrameT = 0
  private syncPose: { position: Vec3; quaternion: Quat; timestamp: number } | null = null
  private readonly rotFilter = new OneEuroQuat(1.0, 2)
  private markerQuat: Quat = IDENTITY
  private badRun = 0
  private goodRun = 0

  constructor(
    opts: {
      lostAfterMs?: number
      reacquireBlendMs?: number
      correctionRate?: number
      useImu?: boolean
      /** 0..1 translation smoothing, default 0.5 (rotation is never smoothed: it follows the gyro). */
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
    this.posFilter.setParams(p.minCutoff, p.beta, p.dCutoff, p.deadband)
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

  /**
   * Track raw marker positions to estimate velocity for latency compensation: least-squares slope
   * over the samples of the last VEL_WINDOW_S. Differencing consecutive frames amplified the
   * measurement noise by e/dt (~4x at 30 Hz) and made the prediction oscillate; the regression is
   * ~6x quieter and also works when only every 2nd/3rd frame is detected.
   */
  private observe(pos: Vec3, frameTime: number, q: Quat): void {
    this.syncPose = { position: pos, quaternion: q, timestamp: frameTime }
    if (this.hist.length && frameTime - (this.hist[this.hist.length - 1] as PosSample).t > VEL_MAX_DT * 1000)
      this.hist.length = 0
    if (!this.hist.length || frameTime > (this.hist[this.hist.length - 1] as PosSample).t) {
      this.hist.push({ t: frameTime, p: pos })
      while (frameTime - (this.hist[0] as PosSample).t > VEL_WINDOW_S * 1000) this.hist.shift()
    }
    const n = this.hist.length
    const first = this.hist[0] as PosSample
    const last = this.hist[n - 1] as PosSample
    if (n >= 2 && last.t - first.t >= VEL_MIN_SPAN_S * 1000) {
      let tm = 0
      const pm: Vec3 = [0, 0, 0]
      for (const h of this.hist) {
        tm += h.t
        for (let i = 0; i < 3; i++) pm[i] = (pm[i] as number) + (h.p[i] as number)
      }
      tm /= n
      for (let i = 0; i < 3; i++) pm[i] = (pm[i] as number) / n
      let stt = 0
      const stp: Vec3 = [0, 0, 0]
      for (const h of this.hist) {
        const dt = (h.t - tm) / 1000
        stt += dt * dt
        for (let i = 0; i < 3; i++) stp[i] = (stp[i] as number) + dt * ((h.p[i] as number) - (pm[i] as number))
      }
      if (stt > 1e-9) this.vel = [stp[0] / stt, stp[1] / stt, stp[2] / stt]
    } else if (n < 2) {
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
    // Time since the last frame's capture: linear up to predictMs, then a saturating tail
    // (detections can be missing for a few frames during motion blur), frozen after PREDICT_MAX_E.
    const e = Math.min(PREDICT_MAX_E, Math.max(0, now - this.lastFrameT) / 1000)
    const P = this.predictMs / 1000
    const eff = e <= P ? e : P + PREDICT_TAIL_TAU * (1 - Math.exp(-(e - P) / PREDICT_TAIL_TAU))
    const k = eff * gate
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
    if (gap > FILTER_RESET_GAP_MS) {
      this.rotFilter.reset()
      this.posFilter.reset()
    }
    // Always fed, so it is warm and continuous whenever the marker-only path is shown.
    this.markerQuat = this.rotFilter.filter(m.quaternion, tSec)

    if (!latest || !imuQ) {
      // Marker-only mode.
      this.position = this.posFilter.filter(m.position, tSec)
      this.observe(m.position, frameTime, m.quaternion)
      return
    }

    const full = quatMultiply(m.quaternion, quatInvert(imuQ))
    const target = yawTwist(full)
    const fusedForTarget = quatMultiply(target, imuQ)
    const R = worldFromCameraQuatToCvR(fusedForTarget)
    const sol = solveTranslationGivenRotation(m.corners, m.K, m.markerSizeM, R)
    const swing = swingAngle(full)
    const residBad =
      sol === null || sol.reprojErrorPx > Math.max(MIN_RESID_PX, RESID_FACTOR * m.reprojErrorPx)
    // Hysteresis on the swing (enter 25 deg, exit 18 deg) plus debounce: one outlier sample from
    // pose-ambiguity noise must not flip the whole pose to the (noisy) marker-only rotation.
    const bad = residBad || swing > (this.flat ? MAX_SWING : EXIT_SWING)
    if (this.flat) {
      if (bad) {
        this.badRun++
        if (this.badRun < NONFLAT_ENTER_SAMPLES) return // ignore the outlier, keep the last pose
        this.flat = false
        this.goodRun = 0
      } else {
        this.badRun = 0
      }
    } else if (bad) {
      this.goodRun = 0
    } else if (++this.goodRun >= FLAT_EXIT_SAMPLES) {
      this.flat = true
      this.badRun = 0
    }
    if (!this.flat) {
      // Marker is not flat w.r.t. gravity: trust the marker alone.
      this.lastConfidence *= NOT_FLAT_CONFIDENCE
      this.position = this.posFilter.filter(m.position, tSec)
      this.observe(m.position, frameTime, m.quaternion)
      return
    }

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
