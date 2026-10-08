import { encode } from 'uqr'

// All sizes are fractions of the frame's outer side S (S = 1).
export const FRAME_THICKNESS = 0.14 // black border width
export const QUIET_ZONE = 0.06 // white gap between frame inner edge and QR
export const QR_SIZE = 1 - 2 * (FRAME_THICKNESS + QUIET_ZONE) // = 0.6
export const DEFAULT_URL = 'https://gevrai.github.io/ARena/'

const QR_ORIGIN = FRAME_THICKNESS + QUIET_ZONE

/** QR matrix, true = dark, row 0 = top. */
export interface MarkerGrid {
  size: number
  modules: boolean[][]
}

export function buildQrGrid(url: string = DEFAULT_URL): MarkerGrid {
  const { data, size } = encode(url, { ecc: 'L', border: 0 })
  return { size, modules: data }
}

/** Sample the ideal marker at normalized (u,v) in [0,1]^2, (0,0) = top-left. true = dark. Outside = white. */
export function sampleMarker(grid: MarkerGrid, u: number, v: number): boolean {
  if (u < 0 || u > 1 || v < 0 || v > 1) return false
  const t = FRAME_THICKNESS
  if (u < t || u > 1 - t || v < t || v > 1 - t) return true
  const qx = Math.floor(((u - QR_ORIGIN) / QR_SIZE) * grid.size)
  const qy = Math.floor(((v - QR_ORIGIN) / QR_SIZE) * grid.size)
  if (qx < 0 || qy < 0 || qx >= grid.size || qy >= grid.size) return false
  return grid.modules[qy]?.[qx] ?? false
}

/** Centres of the 3 QR finder patterns + the 4th (empty) corner, in (u,v). */
export function finderCentres(grid: MarkerGrid): {
  tl: [number, number]
  tr: [number, number]
  bl: [number, number]
  empty: [number, number]
} {
  const map = (m: number): number => QR_ORIGIN + (QR_SIZE * m) / grid.size
  const near = map(3.5)
  const far = map(grid.size - 3.5)
  return { tl: [near, near], tr: [far, near], bl: [near, far], empty: [far, far] }
}
