export interface GrayImage {
  width: number
  height: number
  data: Uint8Array
}

/** RGBA (e.g. canvas ImageData) to 8-bit luma (Rec.601, integer approximation). Reuses `out` when its size matches. */
export function toGray(
  rgba: Uint8ClampedArray,
  width: number,
  height: number,
  out?: GrayImage,
): GrayImage {
  const n = width * height
  const img: GrayImage =
    out && out.data.length === n ? out : { width, height, data: new Uint8Array(n) }
  img.width = width
  img.height = height
  const d = img.data
  for (let i = 0, p = 0; i < n; i++, p += 4) {
    const r = rgba[p] ?? 0
    const g = rgba[p + 1] ?? 0
    const b = rgba[p + 2] ?? 0
    d[i] = (r * 77 + g * 150 + b * 29) >> 8
  }
  return img
}
