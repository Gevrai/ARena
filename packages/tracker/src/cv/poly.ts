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

import type { Contour } from './contours'

/** Ramer-Douglas-Peucker on a closed contour (js-aruco2 CV.approxPolyDP, from OpenCV). */
export function approxPolyDP(contour: Contour, epsilon: number): Contour {
  const len = contour.length
  const poly: Contour = []
  if (len === 0) return poly
  const at = (i: number): { x: number; y: number } => contour[i] ?? { x: 0, y: 0 }
  const eps2 = epsilon * epsilon

  let slice: { start: number; end: number }
  let rightStart = 0
  let rightEnd: number
  const stack: Array<{ start: number; end: number }> = []

  let k = 0
  let maxDist = 0
  let startPt = at(0)
  for (let i = 0; i < 3; i++) {
    maxDist = 0
    k = (k + rightStart) % len
    startPt = at(k)
    if (++k === len) k = 0
    for (let j = 1; j < len; j++) {
      const pt = at(k)
      if (++k === len) k = 0
      const dx = pt.x - startPt.x
      const dy = pt.y - startPt.y
      const dist = dx * dx + dy * dy
      if (dist > maxDist) {
        maxDist = dist
        rightStart = j
      }
    }
  }

  if (maxDist <= eps2) {
    poly.push({ x: startPt.x, y: startPt.y })
  } else {
    slice = { start: k, end: 0 }
    rightStart += slice.start
    slice.end = rightStart
    rightStart -= rightStart >= len ? len : 0
    rightEnd = slice.start
    if (rightEnd < rightStart) rightEnd += len
    stack.push({ start: rightStart, end: rightEnd })
    stack.push({ start: slice.start, end: slice.end })
  }

  for (let top = stack.pop(); top !== undefined; top = stack.pop()) {
    slice = top
    const endPt = at(slice.end % len)
    startPt = at((k = slice.start % len))
    if (++k === len) k = 0

    let leEps: boolean
    if (slice.end <= slice.start + 1) {
      leEps = true
    } else {
      maxDist = 0
      const dx = endPt.x - startPt.x
      const dy = endPt.y - startPt.y
      for (let i = slice.start + 1; i < slice.end; i++) {
        const pt = at(k)
        if (++k === len) k = 0
        const dist = Math.abs((pt.y - startPt.y) * dx - (pt.x - startPt.x) * dy)
        if (dist > maxDist) {
          maxDist = dist
          rightStart = i
        }
      }
      leEps = maxDist * maxDist <= eps2 * (dx * dx + dy * dy)
    }

    if (leEps) {
      poly.push({ x: startPt.x, y: startPt.y })
    } else {
      rightEnd = slice.end
      stack.push({ start: rightStart, end: rightEnd })
      stack.push({ start: slice.start, end: rightStart })
    }
  }
  return poly
}

export function isContourConvex(contour: Contour): boolean {
  const len = contour.length
  if (len < 3) return false
  let orientation = 0
  let prev = contour[len - 1]
  let cur = contour[0]
  if (!prev || !cur) return false
  let dx0 = cur.x - prev.x
  let dy0 = cur.y - prev.y
  let j = 0
  for (let i = 0; i < len; i++) {
    if (++j === len) j = 0
    prev = cur
    cur = contour[j]
    if (!cur) return false
    const dx = cur.x - prev.x
    const dy = cur.y - prev.y
    const a = dx * dy0
    const b = dy * dx0
    orientation |= b > a ? 1 : b < a ? 2 : 3
    if (orientation === 3) return false
    dx0 = dx
    dy0 = dy
  }
  return true
}

export function perimeter(contour: Contour): number {
  let p = 0
  for (let i = 0, j = contour.length - 1; i < contour.length; j = i++) {
    const a = contour[i]
    const b = contour[j]
    if (!a || !b) continue
    p += Math.hypot(a.x - b.x, a.y - b.y)
  }
  return p
}
