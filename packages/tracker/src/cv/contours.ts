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

export type Contour = Array<{ x: number; y: number }>

// 8-neighbourhood, counter-clockwise starting east (image y points down).
const NX = [1, 1, 0, -1, -1, -1, 0, 1]
const NY = [0, -1, -1, -1, 0, 1, 1, 1]

let scratch = new Int32Array(0)

/**
 * Suzuki-style border following (js-aruco2 CV.findContours). Any non-zero pixel is foreground.
 * Returns outer and hole borders; points are pixel indices (x right, y down).
 */
export function findContours(binary: GrayImage): Contour[] {
  const { width, height, data } = binary
  const stride = width + 2
  const size = stride * (height + 2)
  if (scratch.length < size) scratch = new Int32Array(size)
  const src = scratch
  src.fill(0, 0, size)
  for (let y = 0; y < height; y++) {
    const so = y * width
    const o = (y + 1) * stride + 1
    for (let x = 0; x < width; x++) src[o + x] = data[so + x] === 0 ? 0 : 1
  }
  const deltas: number[] = []
  for (let i = 0; i < 8; i++) deltas.push((NX[i] ?? 0) + (NY[i] ?? 0) * stride)
  for (let i = 0; i < 8; i++) deltas.push(deltas[i] ?? 0)

  const contours: Contour[] = []
  let nbd = 1
  let pos = stride + 1
  for (let i = 0; i < height; i++, pos += 2) {
    for (let j = 0; j < width; j++, pos++) {
      const pix = src[pos] ?? 0
      if (pix === 0) continue
      let outer = false
      let hole = false
      if (pix === 1 && src[pos - 1] === 0) outer = true
      else if (pix >= 1 && src[pos + 1] === 0) hole = true
      if (outer || hole) {
        nbd++
        contours.push(borderFollowing(src, pos, nbd, j, i, hole, deltas))
      }
    }
  }
  return contours
}

function borderFollowing(
  src: Int32Array,
  pos: number,
  nbd: number,
  px: number,
  py: number,
  hole: boolean,
  deltas: number[],
): Contour {
  const contour: Contour = []
  let x = px
  let y = py
  let pos1: number
  let s = hole ? 0 : 4
  const sStart = s
  do {
    s = (s - 1) & 7
    pos1 = pos + (deltas[s] ?? 0)
    if (src[pos1] !== 0) break
  } while (s !== sStart)

  if (s === sStart) {
    src[pos] = -nbd
    contour.push({ x, y })
    return contour
  }

  let pos3 = pos
  let pos4: number
  for (;;) {
    const sEnd = s
    do {
      pos4 = pos3 + (deltas[++s] ?? 0)
    } while (src[pos4] === 0)
    s &= 7
    if ((s - 1) >>> 0 < sEnd >>> 0) src[pos3] = -nbd
    else if (src[pos3] === 1) src[pos3] = nbd
    contour.push({ x, y })
    x += NX[s] ?? 0
    y += NY[s] ?? 0
    if (pos4 === pos && pos3 === pos1) break
    pos3 = pos4
    s = (s + 4) & 7
  }
  return contour
}
