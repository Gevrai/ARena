import type { Mat3 } from '../math/quat'

/** Hartley normalisation: translate centroid to origin, scale so mean distance is sqrt(2). */
function normalise(pts: [number, number][]): { T: number[]; p: [number, number][] } | null {
  let cx = 0
  let cy = 0
  for (const [x, y] of pts) {
    cx += x
    cy += y
  }
  cx /= pts.length
  cy /= pts.length
  let d = 0
  for (const [x, y] of pts) d += Math.hypot(x - cx, y - cy)
  d /= pts.length
  if (!(d > 1e-12)) return null
  const s = Math.SQRT2 / d
  return {
    T: [s, 0, -s * cx, 0, s, -s * cy, 0, 0, 1],
    p: pts.map(([x, y]) => [s * (x - cx), s * (y - cy)] as [number, number]),
  }
}

/** Solve A x = b (n x n, row-major) by Gaussian elimination with partial pivoting. null if singular. */
function solve(A: number[], b: number[], n: number): number[] | null {
  for (let c = 0; c < n; c++) {
    let piv = c
    for (let r = c + 1; r < n; r++)
      if (Math.abs(A[r * n + c] ?? 0) > Math.abs(A[piv * n + c] ?? 0)) piv = r
    if (Math.abs(A[piv * n + c] ?? 0) < 1e-10) return null
    if (piv !== c) {
      for (let k = 0; k < n; k++) {
        const t = A[c * n + k] ?? 0
        A[c * n + k] = A[piv * n + k] ?? 0
        A[piv * n + k] = t
      }
      const t = b[c] ?? 0
      b[c] = b[piv] ?? 0
      b[piv] = t
    }
    const d = A[c * n + c] ?? 1
    for (let r = c + 1; r < n; r++) {
      const f = (A[r * n + c] ?? 0) / d
      if (f === 0) continue
      for (let k = c; k < n; k++) A[r * n + k] = (A[r * n + k] ?? 0) - f * (A[c * n + k] ?? 0)
      b[r] = (b[r] ?? 0) - f * (b[c] ?? 0)
    }
  }
  const x = new Array<number>(n).fill(0)
  for (let r = n - 1; r >= 0; r--) {
    let s = b[r] ?? 0
    for (let k = r + 1; k < n; k++) s -= (A[r * n + k] ?? 0) * (x[k] ?? 0)
    x[r] = s / (A[r * n + r] ?? 1)
  }
  return x
}

/** 4-point DLT (normalised, h33 = 1). Returns H (row-major) with dst ~ H * src, or null if degenerate. */
export function homographyFromQuad(src: [number, number][], dst: [number, number][]): Mat3 | null {
  if (src.length !== 4 || dst.length !== 4) return null
  const ns = normalise(src)
  const nd = normalise(dst)
  if (!ns || !nd) return null
  const A: number[] = []
  const b: number[] = []
  for (let i = 0; i < 4; i++) {
    const [x, y] = ns.p[i] as [number, number]
    const [u, v] = nd.p[i] as [number, number]
    A.push(x, y, 1, 0, 0, 0, -u * x, -u * y)
    b.push(u)
    A.push(0, 0, 0, x, y, 1, -v * x, -v * y)
    b.push(v)
  }
  const h = solve(A, b, 8)
  if (!h) return null
  const Hn = [...h, 1]
  // H = Td^-1 * Hn * Ts
  const [a, , tx, , , ty] = nd.T as [number, number, number, number, number, number]
  const sInv = 1 / a // uniform scale
  const Tdi = [sInv, 0, -tx * sInv, 0, sInv, -ty * sInv, 0, 0, 1]
  const mul = (P: number[], Q: number[]): number[] => {
    const R = new Array<number>(9).fill(0)
    for (let r = 0; r < 3; r++)
      for (let c = 0; c < 3; c++)
        for (let k = 0; k < 3; k++)
          R[r * 3 + c] = (R[r * 3 + c] ?? 0) + (P[r * 3 + k] ?? 0) * (Q[k * 3 + c] ?? 0)
    return R
  }
  const H = mul(Tdi, mul(Hn, ns.T))
  const w = H[8] ?? 0
  if (!Number.isFinite(w) || Math.abs(w) < 1e-12) return null
  const out = new Float64Array(9)
  for (let i = 0; i < 9; i++) {
    const v = (H[i] ?? 0) / w
    if (!Number.isFinite(v)) return null
    out[i] = v
  }
  return out
}

export function applyHomography(H: Mat3, x: number, y: number): [number, number] {
  const w = (H[6] ?? 0) * x + (H[7] ?? 0) * y + (H[8] ?? 1)
  return [
    ((H[0] ?? 0) * x + (H[1] ?? 0) * y + (H[2] ?? 0)) / w,
    ((H[3] ?? 0) * x + (H[4] ?? 0) * y + (H[5] ?? 0)) / w,
  ]
}
