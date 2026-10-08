export interface Intrinsics {
  fx: number
  fy: number
  cx: number
  cy: number
  width: number
  height: number
}

/**
 * Pinhole intrinsics from an image size and horizontal FOV (square pixels).
 * Pixel convention: pixel (i,j) covers [i,i+1)x[j,j+1), centre at (i+0.5,j+0.5);
 * the principal point is the exact image centre (width/2, height/2).
 */
export function intrinsicsFromSize(width: number, height: number, hfovDeg = 65): Intrinsics {
  const f = width / 2 / Math.tan((hfovDeg * Math.PI) / 360)
  return { fx: f, fy: f, cx: width / 2, cy: height / 2, width, height }
}

/**
 * OpenGL/three.js projection matrix (column-major, NDC z in [-1,1], camera looks down -Z, +Y up)
 * for a canvas of viewW×viewH showing a video with `object-fit: cover`. `K` is in VIDEO pixel units
 * (K.width × K.height is the full video size). A point projecting to video pixel (u,v) lands at
 * screen pixel ((u - w/2)·s + viewW/2, (v - h/2)·s + viewH/2), s = max(viewW/w, viewH/h).
 */
export function projectionForCover(
  K: Intrinsics,
  viewW: number,
  viewH: number,
  near: number,
  far: number,
): Float32Array {
  const s = Math.max(viewW / K.width, viewH / K.height)
  const m = new Float32Array(16)
  m[0] = (2 * s * K.fx) / viewW
  m[8] = (-2 * s * (K.cx - K.width / 2)) / viewW
  m[5] = (2 * s * K.fy) / viewH
  m[9] = (2 * s * (K.cy - K.height / 2)) / viewH
  m[10] = -(far + near) / (far - near)
  m[14] = (-2 * far * near) / (far - near)
  m[11] = -1
  return m
}
