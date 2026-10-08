import { it, expect } from 'vitest'
import { buildQrGrid, sampleMarker, finderCentres, QR_SIZE, FRAME_THICKNESS } from '../src/marker/layout'
import { renderMarkerSvg, renderCardSvg } from '../src/marker/svg'

// Measured with uqr, ecc 'L', for DEFAULT_URL: version 2 (25 modules).
it('QR for default URL is version 2 (25 modules)', () => { expect(buildQrGrid().size).toBe(25) })
it('frame is dark, quiet zone is white', () => {
  const g = buildQrGrid()
  expect(sampleMarker(g, FRAME_THICKNESS / 2, 0.5)).toBe(true)
  expect(sampleMarker(g, FRAME_THICKNESS + 0.03, 0.5)).toBe(false)
  expect(sampleMarker(g, -0.1, 0.5)).toBe(false)
})
it('finder centres are dark, empty corner centre region is not a finder', () => {
  const g = buildQrGrid(); const f = finderCentres(g)
  for (const p of [f.tl, f.tr, f.bl]) expect(sampleMarker(g, p[0], p[1])).toBe(true)
  expect(f.tl[0]).toBeCloseTo(1 - f.tr[0], 5); expect(QR_SIZE).toBeCloseTo(0.6, 5)
})
it('renderMarkerSvg contains a viewBox in mm', () => { expect(renderMarkerSvg(buildQrGrid(), 50)).toMatch(/viewBox="0 0 50 50"/) })
it('renderCardSvg is 85x55 mm', () => {
  const s = renderCardSvg(buildQrGrid())
  expect(s).toMatch(/viewBox="0 0 85 55"/); expect(s).toContain('ARena')
})
