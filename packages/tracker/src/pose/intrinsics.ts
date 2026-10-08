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
