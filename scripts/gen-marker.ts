import { mkdirSync, writeFileSync } from 'node:fs'
import { buildQrGrid, renderCardSvg, renderMarkerSvg } from '../packages/tracker/src/index'

const grid = buildQrGrid()
mkdirSync('public', { recursive: true })
writeFileSync('public/marker-card.svg', renderCardSvg(grid))
writeFileSync('public/marker.svg', renderMarkerSvg(grid, 50))
console.log(`wrote public/marker-card.svg and public/marker.svg (QR ${grid.size} modules)`)
