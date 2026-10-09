import {
  AxesHelper,
  BoxGeometry,
  CircleGeometry,
  DoubleSide,
  EdgesGeometry,
  Group,
  LineBasicMaterial,
  LineSegments,
  Mesh,
  MeshBasicMaterial,
  PerspectiveCamera,
  Scene,
  SphereGeometry,
  WebGLRenderer,
} from 'three'
import { createTracker } from '@arena/tracker'
import type { TrackerError } from '@arena/tracker'
import './demo.css'

const byId = <T extends HTMLElement>(id: string): T => {
  const el = document.getElementById(id)
  if (!el) throw new Error(`missing #${id}`)
  return el as T
}

const video = byId<HTMLVideoElement>('video')
const glCanvas = byId<HTMLCanvasElement>('gl')
const overlay = byId<HTMLCanvasElement>('overlay')
const hud = byId('hud')
const badge = byId('badge')
const statsEl = byId('stats')
const strip = byId<HTMLCanvasElement>('strip')
const startPanel = byId('start-panel')
const startMsg = byId('start-msg')
const startBtn = byId<HTMLButtonElement>('start')
const optGyro = byId<HTMLInputElement>('opt-gyro')
const optRes = byId<HTMLSelectElement>('opt-res')
const optHfov = byId<HTMLInputElement>('opt-hfov')
const hfovVal = byId('hfov-val')
const optCorners = byId<HTMLInputElement>('opt-corners')
const optAccel = byId<HTMLInputElement>('opt-accel')

// ---- settings (localStorage, best effort) ----
interface Settings {
  gyro: boolean
  detectWidth: number
  fov: number
  corners: boolean
  accel: boolean
  hudCollapsed: boolean
}
const KEY = 'arena-tracker-demo'
const defaults: Settings = {
  gyro: true,
  detectWidth: 640,
  fov: 65,
  corners: false,
  accel: false,
  hudCollapsed: false,
}
function loadSettings(): Settings {
  try {
    const raw = localStorage.getItem(KEY)
    if (raw) {
      // `hfov` is the pre-rename key (same value, now the long-side FOV): keep it compatible.
      const p = JSON.parse(raw) as Partial<Settings> & { hfov?: number }
      return {
        gyro: typeof p.gyro === 'boolean' ? p.gyro : defaults.gyro,
        detectWidth: [480, 640, 960].includes(Number(p.detectWidth))
          ? Number(p.detectWidth)
          : defaults.detectWidth,
        fov: Math.min(80, Math.max(50, Number(p.fov ?? p.hfov) || defaults.fov)),
        corners: typeof p.corners === 'boolean' ? p.corners : defaults.corners,
        accel: p.accel === true,
        hudCollapsed: p.hudCollapsed === true,
      }
    }
  } catch {
    // storage unavailable or corrupt: use defaults
  }
  return { ...defaults }
}
const settings = loadSettings()
function saveSettings(): void {
  try {
    localStorage.setItem(KEY, JSON.stringify(settings))
  } catch {
    // ignore
  }
}

// ---- tracker ----
const tracker = createTracker({
  video,
  fovDeg: settings.fov,
  detectWidth: settings.detectWidth,
  useImu: settings.gyro,
  useAccel: settings.accel,
})

// ---- three.js scene ----
const renderer = new WebGLRenderer({ canvas: glCanvas, alpha: true, antialias: true })
renderer.setClearColor(0x000000, 0)
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2))
const scene = new Scene()
const camera = new PerspectiveCamera()
camera.matrixAutoUpdate = false

const content = new Group()
content.add(new AxesHelper(0.03))
const cube = new LineSegments(
  new EdgesGeometry(new BoxGeometry(0.05, 0.05, 0.05)),
  new LineBasicMaterial({ color: 0xffffff }),
)
cube.position.y = 0.025
content.add(cube)
const disc = new Mesh(
  new CircleGeometry(0.075, 48),
  new MeshBasicMaterial({ color: 0x33aaff, transparent: true, opacity: 0.25, side: DoubleSide }),
)
disc.rotation.x = -Math.PI / 2
disc.position.y = 0.005
content.add(disc)
const ball = new Mesh(new SphereGeometry(0.01, 24, 16), new MeshBasicMaterial({ color: 0xff5a36 }))
ball.position.set(0.04, 0.01, 0.03)
content.add(ball)
content.visible = false
scene.add(content)

let viewW = 0
let viewH = 0
function resize(): void {
  viewW = window.innerWidth
  viewH = window.innerHeight
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2))
  renderer.setSize(viewW, viewH, false)
  const dpr = Math.min(window.devicePixelRatio, 2)
  overlay.width = Math.round(viewW * dpr)
  overlay.height = Math.round(viewH * dpr)
}
window.addEventListener('resize', resize)
window.addEventListener('orientationchange', resize)
resize()

// ---- HUD ----
optGyro.checked = settings.gyro
optRes.value = String(settings.detectWidth)
optHfov.value = String(settings.fov)
hfovVal.textContent = String(settings.fov)
optCorners.checked = settings.corners
optAccel.checked = settings.accel
hud.classList.toggle('collapsed', settings.hudCollapsed)

byId('hud-toggle').addEventListener('click', () => {
  settings.hudCollapsed = hud.classList.toggle('collapsed')
  saveSettings()
})
optGyro.addEventListener('change', () => {
  settings.gyro = optGyro.checked
  tracker.setOptions({ useImu: settings.gyro })
  saveSettings()
})
optRes.addEventListener('change', () => {
  settings.detectWidth = Number(optRes.value)
  tracker.setOptions({ detectWidth: settings.detectWidth })
  saveSettings()
})
optHfov.addEventListener('input', () => {
  settings.fov = Number(optHfov.value)
  hfovVal.textContent = String(settings.fov)
  tracker.setOptions({ fovDeg: settings.fov })
  saveSettings()
})
optCorners.addEventListener('change', () => {
  settings.corners = optCorners.checked
  saveSettings()
})

optAccel.addEventListener('change', () => {
  settings.accel = optAccel.checked
  tracker.setOptions({ useAccel: settings.accel })
  saveSettings()
})

// ---- start / errors ----
const ERROR_TEXT: Record<TrackerError, string> = {
  'camera-denied':
    'Camera access was denied. Allow the camera for this site in your browser settings, then retry.',
  'no-camera': 'No camera was found on this device.',
  'insecure-context': 'The camera needs a secure (https) connection.',
  'camera-interrupted': 'The camera was interrupted. Tap retry to resume.',
  unknown: 'Could not start the camera. Please retry.',
}
const INSTRUCTION = 'Point the camera at the ARena card'
let started = false

async function start(): Promise<void> {
  // Must be called synchronously inside the gesture; do not await fullscreen
  // (iOS needs the gesture for the motion permission prompt).
  try {
    const p = document.documentElement.requestFullscreen?.()
    p?.catch(() => undefined)
  } catch {
    // ignore
  }
  startBtn.disabled = true
  try {
    await tracker.start()
    started = true
    startPanel.classList.add('hidden')
  } catch (e) {
    const code = (e as { code?: TrackerError } | null)?.code ?? 'unknown'
    startMsg.textContent = ERROR_TEXT[code] ?? ERROR_TEXT.unknown
    startBtn.textContent = 'Retry'
  } finally {
    startBtn.disabled = false
  }
}
startMsg.textContent = INSTRUCTION
startBtn.addEventListener('click', () => {
  void start()
})

// ---- frame loop ----
const ctx2d = overlay.getContext('2d')
let lastStatsText = 0

function drawCorners(): void {
  if (!ctx2d) return
  const dpr = overlay.width / Math.max(1, viewW)
  ctx2d.setTransform(dpr, 0, 0, dpr, 0, 0)
  ctx2d.clearRect(0, 0, viewW, viewH)
  if (!settings.corners) return
  const st = tracker.stats()
  if (!st.corners || st.videoW <= 0 || st.videoH <= 0) return
  // corners are in detect resolution: detectWidth x (detectWidth * videoH / videoW)
  const k = st.videoW / settings.detectWidth
  const s = Math.max(viewW / st.videoW, viewH / st.videoH)
  const pts = st.corners.map((c) => ({
    x: (c[0] * k - st.videoW / 2) * s + viewW / 2,
    y: (c[1] * k - st.videoH / 2) * s + viewH / 2,
  }))
  ctx2d.lineWidth = 2
  ctx2d.strokeStyle = '#0f0'
  ctx2d.beginPath()
  pts.forEach((p, i) => (i === 0 ? ctx2d.moveTo(p.x, p.y) : ctx2d.lineTo(p.x, p.y)))
  ctx2d.closePath()
  ctx2d.stroke()
  const colors = ['#f00', '#ff0', '#0f0', '#0ff']
  pts.forEach((p, i) => {
    ctx2d.fillStyle = colors[i] ?? '#fff'
    ctx2d.beginPath()
    ctx2d.arc(p.x, p.y, 4, 0, Math.PI * 2)
    ctx2d.fill()
  })
}

const STRIP_COLORS = { hit: '#2ecc71', nonflat: '#f5a623', miss: '#e74c3c' } as const
/** Scrolling strip: one tick per analysed frame of the last 3 s, coloured by outcome. */
function drawStrip(
  recent: { t: number; outcome: 'hit' | 'nonflat' | 'miss' }[],
  now: number,
): void {
  const c = strip.getContext('2d')
  if (!c) return
  c.clearRect(0, 0, strip.width, strip.height)
  c.fillStyle = 'rgba(255,255,255,0.12)'
  c.fillRect(0, 0, strip.width, strip.height)
  for (const f of recent) {
    const x = strip.width - ((now - f.t) / 3000) * strip.width
    c.fillStyle = STRIP_COLORS[f.outcome]
    c.fillRect(Math.floor(x) - 1, 0, 3, strip.height)
  }
}

function frame(now: number): void {
  requestAnimationFrame(frame)
  const pose = tracker.getPose()
  const visible = pose.source !== 'none'
  content.visible = visible
  if (visible) {
    camera.matrixWorld.fromArray(pose.matrix)
    camera.matrixWorldInverse.copy(camera.matrixWorld).invert()
    camera.projectionMatrix.fromArray(tracker.projectionMatrix(viewW, viewH, 0.01, 10))
    camera.projectionMatrixInverse.copy(camera.projectionMatrix).invert()
    renderer.render(scene, camera)
  } else {
    renderer.clear()
  }
  drawCorners()

  badge.textContent = pose.source
  badge.className = pose.source
  if (now - lastStatsText > 200) {
    lastStatsText = now
    const st = tracker.stats()
    const hit = Number.isFinite(st.hitRate2s) ? `${Math.round(st.hitRate2s * 100)}%` : '-'
    const lines = [
      `${pose.source}${pose.flat ? '' : ' NOT-FLAT'}  hit ${hit} (2s)`,
      `detect ${st.detectHz.toFixed(1)} Hz  worker ${st.detectMs.toFixed(0)} ms`,
      `latency ${st.latencyMs.toFixed(0)} ms  reproj ${Number.isFinite(st.reprojErrorPx) ? st.reprojErrorPx.toFixed(2) : '-'} px`,
      `imu ${st.imu ? 'on' : 'off'}  video ${st.videoW}x${st.videoH}  conf ${pose.confidence.toFixed(2)}`,
    ]
    if (st.accelHz !== undefined && st.accelWorld) {
      const w = st.accelWorld.map((v) => v.toFixed(1).padStart(5))
      lines.push(
        `accel ${st.accelHz} Hz  disp ${(st.accelDispMm ?? 0).toFixed(0)} mm  world x${w[0]} y${w[1]} z${w[2]} m/s2`,
      )
    }
    drawStrip(st.recent, performance.now())
    if (st.lastError) lines.push(`error   ${st.lastError}`)
    statsEl.textContent = lines.join('\n')
  }
}
requestAnimationFrame(frame)

tracker.onStatus((s) => {
  if (s.state === 'error' && started) {
    startMsg.textContent = ERROR_TEXT[s.reason ?? 'unknown']
    startBtn.textContent = 'Retry'
    startPanel.classList.remove('hidden')
    started = false
  }
})
