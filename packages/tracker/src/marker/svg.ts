import { FRAME_THICKNESS, QR_SIZE, QUIET_ZONE, type MarkerGrid } from './layout'

const n = (x: number): string => String(Math.round(x * 1000) / 1000)

/** SVG path data (in the marker's own mm frame) for the black frame + dark QR modules. */
function markerPaths(grid: MarkerGrid, sizeMm: number): { frame: string; modules: string } {
  const t = FRAME_THICKNESS * sizeMm
  const inner = sizeMm - t
  const frame = `M0 0H${n(sizeMm)}V${n(sizeMm)}H0Z M${n(t)} ${n(t)}V${n(inner)}H${n(inner)}V${n(t)}Z`
  const origin = (FRAME_THICKNESS + QUIET_ZONE) * sizeMm
  const m = (QR_SIZE * sizeMm) / grid.size
  let d = ''
  grid.modules.forEach((row, y) => {
    let x = 0
    while (x < row.length) {
      if (!row[x]) {
        x++
        continue
      }
      const start = x
      while (x < row.length && row[x]) x++
      d += `M${n(origin + start * m)} ${n(origin + y * m)}h${n((x - start) * m)}v${n(m)}h${n(-(x - start) * m)}z`
    }
  })
  return { frame, modules: d }
}

function markerGroup(grid: MarkerGrid, sizeMm: number): string {
  const { frame, modules } = markerPaths(grid, sizeMm)
  return (
    `<rect width="${n(sizeMm)}" height="${n(sizeMm)}" fill="#fff"/>` +
    `<path fill="#000" fill-rule="evenodd" d="${frame}"/>` +
    `<path fill="#000" d="${modules}"/>`
  )
}

/** Just the framed QR, with viewBox in mm. Strictly black and white. */
export function renderMarkerSvg(grid: MarkerGrid, sizeMm: number): string {
  return (
    `<svg xmlns="http://www.w3.org/2000/svg" width="${n(sizeMm)}mm" height="${n(sizeMm)}mm" viewBox="0 0 ${n(sizeMm)} ${n(sizeMm)}" shape-rendering="crispEdges">` +
    markerGroup(grid, sizeMm) +
    `</svg>`
  )
}

/** 85x55 mm business card: marker left, "ARena / scan to play" text right. */
export function renderCardSvg(
  grid: MarkerGrid,
  opts: { markerMm?: number; cardW?: number; cardH?: number; label?: string } = {},
): string {
  const markerMm = opts.markerMm ?? 45
  const w = opts.cardW ?? 85
  const h = opts.cardH ?? 55
  const label = opts.label ?? 'ARena'
  const mx = (h - markerMm) / 2
  const tx = mx + markerMm + 5
  const cx = tx + (w - tx - mx) / 2
  const accent = '#ff5a36'
  return (
    `<svg xmlns="http://www.w3.org/2000/svg" width="${n(w)}mm" height="${n(h)}mm" viewBox="0 0 ${n(w)} ${n(h)}">` +
    `<rect width="${n(w)}" height="${n(h)}" fill="#fff"/>` +
    `<g shape-rendering="crispEdges" transform="translate(${n(mx)} ${n(mx)})">${markerGroup(grid, markerMm)}</g>` +
    `<g font-family="'Trebuchet MS','Segoe UI',Arial,sans-serif" text-anchor="middle">` +
    `<text x="${n(cx)}" y="${n(h / 2 - 1)}" font-size="9.5" font-weight="900" fill="#111">${label}</text>` +
    `<rect x="${n(cx - 9)}" y="${n(h / 2 + 1.5)}" width="18" height="1.2" rx="0.6" fill="${accent}"/>` +
    `<text x="${n(cx)}" y="${n(h / 2 + 8)}" font-size="3.6" font-weight="700" fill="${accent}" letter-spacing="0.2">scan to play ▸</text>` +
    `</g></svg>`
  )
}
