/*
 * Ported from js-aruco2 (https://github.com/damianofalcioni/js-aruco2, src/cv.js),
 * itself derived from js-aruco / OpenCV-style code.
 *
 * Copyright (c) 2011 Juan Mellado
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

import type { GrayImage } from './image'

// Module-level scratch so nothing is allocated per frame once the size is stable.
let integral = new Uint32Array(0)

/**
 * Binary output: 255 where pixel is darker than (local mean - offset), else 0.
 * Local mean via integral image over a (2r+1)^2 box (clamped at borders).
 * Rewritten (js-aruco2 used a stack box blur).
 */
export function adaptiveThreshold(
  src: GrayImage,
  radius: number,
  offset: number,
  out?: GrayImage,
): GrayImage {
  const { width: w, height: h, data: s } = src
  const dst: GrayImage =
    out && out.data.length === w * h ? out : { width: w, height: h, data: new Uint8Array(w * h) }
  dst.width = w
  dst.height = h
  const d = dst.data
  const iw = w + 1
  if (integral.length < iw * (h + 1)) integral = new Uint32Array(iw * (h + 1))
  const I = integral
  for (let x = 0; x < iw; x++) I[x] = 0
  for (let y = 0; y < h; y++) {
    let row = 0
    const o = (y + 1) * iw
    I[o] = 0
    for (let x = 0; x < w; x++) {
      row += s[y * w + x] ?? 0
      I[o + x + 1] = (I[o - iw + x + 1] ?? 0) + row
    }
  }
  for (let y = 0; y < h; y++) {
    const y0 = y - radius < 0 ? 0 : y - radius
    const y1 = y + radius + 1 > h ? h : y + radius + 1
    const r0 = y0 * iw
    const r1 = y1 * iw
    for (let x = 0; x < w; x++) {
      const x0 = x - radius < 0 ? 0 : x - radius
      const x1 = x + radius + 1 > w ? w : x + radius + 1
      const sum = (I[r1 + x1] ?? 0) - (I[r1 + x0] ?? 0) - (I[r0 + x1] ?? 0) + (I[r0 + x0] ?? 0)
      const n = (x1 - x0) * (y1 - y0)
      // pixel < sum/n - offset  <=>  pixel*n < sum - offset*n
      d[y * w + x] = (s[y * w + x] ?? 0) * n < sum - offset * n ? 255 : 0
    }
  }
  return dst
}
