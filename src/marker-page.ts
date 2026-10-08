import { buildQrGrid, renderCardSvg, renderMarkerSvg } from '@arena/tracker'

const grid = buildQrGrid()
const byId = (id: string): HTMLElement => {
  const el = document.getElementById(id)
  if (!el) throw new Error(`missing #${id}`)
  return el
}

byId('card').innerHTML = renderCardSvg(grid)
const screen = byId('screen')
screen.innerHTML = renderMarkerSvg(grid, 50)

byId('print').addEventListener('click', () => window.print())

let wakeLock: WakeLockSentinel | null = null

async function show(): Promise<void> {
  screen.classList.add('on')
  try {
    await screen.requestFullscreen()
  } catch {
    // fullscreen unavailable: the fixed overlay still fills the viewport
  }
  try {
    wakeLock = (await navigator.wakeLock?.request('screen')) ?? null
  } catch {
    wakeLock = null
  }
}

function hide(): void {
  screen.classList.remove('on')
  void wakeLock?.release()
  wakeLock = null
  if (document.fullscreenElement) void document.exitFullscreen()
}

byId('show').addEventListener('click', () => void show())
screen.addEventListener('click', hide)
document.addEventListener('fullscreenchange', () => {
  if (!document.fullscreenElement) hide()
})
