# M1 — Tracker Feasibility Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Ship a standalone `packages/tracker` library that tracks a business-card "framed QR" marker from the phone camera, fused with the gyroscope. Also ship a demo page and a marker page, both deployed to GitHub Pages, so the client can judge AR feasibility on a real Android phone.

**Architecture:** The camera frames are downscaled to grayscale on the main thread and posted to a Web Worker. The worker runs our own CV pipeline: adaptive threshold → contours → quads → frame validation + orientation from the QR finder patterns → subpixel corners → homography pose. The main thread fuses marker poses with DeviceOrientation in a complementary filter, using timestamp-matched IMU history and One Euro smoothing, and exposes `getPose()`. The tracker never imports three.js or game code. The demo page uses three.js only to visualise.

**Tech Stack:** Vite 7, TypeScript 5 (strict), Vitest, ESLint 9 + Prettier, npm workspaces, `uqr` (QR matrix), `three` (demo only), `@vitejs/plugin-basic-ssl`, GitHub Actions → Pages.

**Spec:** `docs/superpowers/specs/2026-10-07-ballball-design.md` (§4 and §10 milestone 1). **Research:** `docs/research/2026-10-07-tracker-and-platform.md`.

## Global Constraints

- Site URL: `https://gevrai.github.io/ARena/`. Set Vite `base: '/ARena/'`. The marker QR encodes exactly `https://gevrai.github.io/ARena/`.
- TypeScript `strict: true`, plus `noUncheckedIndexedAccess: true`. No `any` in library code.
- `packages/tracker` must not import `three` or anything outside itself (enforced by ESLint `no-restricted-imports`).
- Matrices are `Float32Array(16)`, column-major (WebGL/three.js convention).
- Coordinate conventions (used everywhere, do not deviate):
  - **Marker/world frame**: origin at the marker centre, **+Y up** (out of the card), +X towards the card's right edge, +Z towards the card's bottom edge (towards the viewer when the card is read upright). Units are **metres**.
  - **Camera frame (output)**: the three.js camera convention: +X right, +Y up, looking down −Z.
  - `getPose().matrix` = **world-from-camera** transform, directly usable as `camera.matrixWorld`.
  - Internally, CV uses OpenCV camera conventions (x right, y down, z forward). Convert at one place only (`pose.ts`).
- Default marker frame outer size: 50 mm (`markerSizeMm: 50`). Default horizontal FOV assumption: 65°.
- Detection must run in a Web Worker. Rendering must never block on detection.
- No new runtime deps beyond `uqr` (tracker) and `three` (demo). Port CV code from js-aruco2 (MIT) and keep its copyright notice in ported files.
- Commit after each task with a conventional message ending with `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`.

## Review Focus

1. **False positives from other rectangles** (business cards, phone bezels, book covers, table tiles): the detector must return `null` unless the inner finder-pattern check passes. Pinned in Task 5 (clutter test).
2. **Marker partially out of frame or occluded by a finger**: no detection with garbage corners, and no NaN poses. Pinned in Task 5 (cropped marker test) and Task 6 (degenerate quad test).
3. **Phone rotated portrait ↔ landscape mid-session**: the pose and projection stay correct. `screen.orientation.angle` is applied to the IMU, and the projection is recomputed from the current video/canvas size. Pinned in Task 8 (orientation-angle test) and Task 9 (projection test for both aspect ratios).
4. **Camera or motion permission denied, or no camera (desktop)**: `start()` rejects with a typed error and status `'error'`, and the demo shows the reason. If motion is denied, the tracker continues marker-only. Pinned in Task 9 (tests with mocked `navigator.mediaDevices`).
5. **Marker shown on a bright phone screen or under glare**: adaptive threshold (not global) must still find it. Pinned in Task 4/5 (synthetic gradient + specular blob test).

---

## File Structure

```
package.json                 workspaces, scripts (dev, build, test, typecheck, lint, gen:marker)
tsconfig.base.json           shared strict options
tsconfig.json                root app (refs packages)
vite.config.ts               base '/ARena/', multipage inputs, basic-ssl, worker format 'es'
eslint.config.js, .prettierrc
index.html                   placeholder landing ("ARena — coming soon", links to demo & marker)
marker/index.html, src/marker-page.ts       marker page: card preview, print, fullscreen "show marker"
demo/tracker.html, src/demo/tracker-demo.ts demo page (three.js visualisation + HUD)
scripts/gen-marker.ts        writes public/marker-card.svg
.github/workflows/deploy.yml build + deploy to Pages on push to main

packages/tracker/
  package.json (name "@arena/tracker", "type":"module", exports "./src/index.ts")
  src/index.ts               public API re-exports
  src/marker/layout.ts       marker geometry constants + module grid builder (shared by generator, detector and tests)
  src/marker/svg.ts          renderMarkerSvg(), renderCardSvg()
  src/cv/image.ts            GrayImage type, toGray()
  src/cv/threshold.ts        adaptiveThreshold()
  src/cv/contours.ts         findContours()
  src/cv/poly.ts             approxPolyDP(), isContourConvex(), perimeter()
  src/cv/homography.ts       homographyFromQuad(), applyHomography()
  src/detect/framedQr.ts     detectFramedQr(): Detection | null
  src/detect/refine.ts       refineCorners() (line fit per edge)
  src/pose/intrinsics.ts     intrinsicsFromVideo(), projectionFromIntrinsics()
  src/pose/pose.ts           estimatePose(), poseToWorldFromCamera()
  src/math/                  mat4.ts, quat.ts, vec3.ts (minimal, tested)
  src/imu/orientation.ts     deviceOrientationToQuat(), requestMotionPermission()
  src/imu/history.ts         ImuHistory ring buffer (timestamp lookup + slerp)
  src/fusion/oneEuro.ts      OneEuroFilter (scalar) + vec3/quat helpers
  src/fusion/fusion.ts       PoseFusion (complementary filter)
  src/worker/protocol.ts     message types
  src/worker/detect.worker.ts
  src/tracker.ts             createTracker()
  test/synth.ts              synthetic renderer: marker under pose → GrayImage
  test/*.test.ts
```

---

### Task 1: Project scaffold + Pages deploy

**Files:** all root config files listed above, `index.html`, `packages/tracker/package.json`, `packages/tracker/src/index.ts` (exports `VERSION = '0.0.1'`), `packages/tracker/test/smoke.test.ts`, `.github/workflows/deploy.yml`, `.gitignore`.

**Interfaces:** Produces: scripts `npm test`, `npm run typecheck`, `npm run lint`, `npm run build`, `npm run dev` (HTTPS, `--host` capable). Workspace import `@arena/tracker` resolves from the root app.

- [ ] Step 1: Create `package.json` with `"workspaces": ["packages/*"]`, devDeps (vite, typescript, vitest, eslint, typescript-eslint, prettier, @vitejs/plugin-basic-ssl, tsx), deps (three, @types/three). Create `packages/tracker/package.json` with dep `uqr`.
- [ ] Step 2: Write `packages/tracker/test/smoke.test.ts`:
```ts
import { describe, it, expect } from 'vitest'
import { VERSION } from '../src/index'
describe('tracker package', () => { it('exports a version', () => { expect(VERSION).toBe('0.0.1') }) })
```
- [ ] Step 3: Configure `vite.config.ts`: `base: '/ARena/'`, `build.rollupOptions.input` = `{ main: 'index.html', marker: 'marker/index.html', demo: 'demo/tracker.html' }` (create stub HTML pages), `plugins: [basicSsl()]`, `worker: { format: 'es' }`, and a `test` block for Vitest (`include: ['packages/**/test/**/*.test.ts', 'src/**/*.test.ts']`).
- [ ] Step 4: ESLint flat config with typescript-eslint strict. Add a `no-restricted-imports` rule for `packages/tracker/**` that forbids `three` and any path matching `../../../*` / `src/*` of the app.
- [ ] Step 5: `.github/workflows/deploy.yml`: on push to `main` → `actions/checkout`, `actions/setup-node` (node 22, npm cache), `npm ci`, `npm test`, `npm run build`, `actions/upload-pages-artifact` (path `dist`), `actions/deploy-pages`. Permissions: `pages: write`, `id-token: write`.
- [ ] Step 6: Run `npm install && npm test && npm run typecheck && npm run lint && npm run build`. Expected: all pass, and `dist/` contains `index.html`, `marker/index.html`, `demo/tracker.html` with asset URLs prefixed `/ARena/`.
- [ ] Step 7: Commit `chore: scaffold workspace, vite, vitest, pages deploy`.
- [ ] Step 8 (orchestrator, not the implementer): push, and enable Pages with "GitHub Actions" as the source (`gh api -X POST repos/Gevrai/ARena/pages -f build_type=workflow`). Confirm that https://gevrai.github.io/ARena/ serves the placeholder.

### Task 2: Marker layout + SVG generation + marker page

**Files:** `packages/tracker/src/marker/layout.ts`, `packages/tracker/src/marker/svg.ts`, `packages/tracker/test/layout.test.ts`, `scripts/gen-marker.ts`, `marker/index.html`, `src/marker-page.ts`.

**Interfaces:**
- Produces:
```ts
// layout.ts — all sizes are fractions of the frame's outer side S (S = 1)
export const FRAME_THICKNESS = 0.14   // black border width
export const QUIET_ZONE = 0.06        // white gap between frame inner edge and QR
export const QR_SIZE = 1 - 2 * (FRAME_THICKNESS + QUIET_ZONE) // = 0.6
export const DEFAULT_URL = 'https://gevrai.github.io/ARena/'
export interface MarkerGrid { size: number; modules: boolean[][] } // QR matrix, true = dark, row 0 = top
export function buildQrGrid(url?: string): MarkerGrid             // uqr encode, ecc 'L', border 0
/** Sample the ideal marker at normalized coords (u,v) ∈ [0,1]², (0,0)=top-left. true = dark. Outside [0,1]² = false (white). */
export function sampleMarker(grid: MarkerGrid, u: number, v: number): boolean
/** Centres of the 3 QR finder patterns + the 4th (empty) corner, in (u,v): order TL, TR, BL, BR(empty). */
export function finderCentres(grid: MarkerGrid): { tl: [number, number]; tr: [number, number]; bl: [number, number]; empty: [number, number] }
// svg.ts
export function renderMarkerSvg(grid: MarkerGrid, sizeMm: number): string    // just the framed QR, with viewBox in mm
export function renderCardSvg(grid: MarkerGrid, opts?: { markerMm?: number; cardW?: number; cardH?: number; label?: string }): string // 85×55 mm card: marker left, "ARena — scan to play" text right
```
- [ ] Step 1: Write the failing tests `layout.test.ts`:
```ts
import { buildQrGrid, sampleMarker, finderCentres, QR_SIZE, FRAME_THICKNESS } from '../src/marker/layout'
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
```
(If the default URL produces a version other than 2, assert on whatever `uqr` gives with ecc L, and document it in a comment. Use the measured value, not a guess.)
- [ ] Step 2: Run `npx vitest run packages/tracker/test/layout.test.ts`. Expected: FAIL (module not found).
- [ ] Step 3: Implement `layout.ts` and `svg.ts`. Finder centre of the TL finder = QR origin + 3.5 modules. TR: x = size − 3.5. BL: y = size − 3.5. The empty corner = (size − 3.5, size − 3.5). Map to (u,v) via `FRAME_THICKNESS + QUIET_ZONE + QR_SIZE * m / size`. The SVG uses one `<path>` for the dark modules (merge runs per row) and a frame drawn as an even-odd path. White background rect.
- [ ] Step 4: Run the tests. Expected: PASS.
- [ ] Step 5: `scripts/gen-marker.ts` (run with `tsx`, npm script `gen:marker`, also run in `prebuild`) writes `public/marker-card.svg` and `public/marker.svg`.
- [ ] Step 6: Marker page (`marker/index.html` + `src/marker-page.ts`):
  - Shows the card SVG at true size for printing (`@media print`, sized in mm; a print button calls `window.print()`).
  - Has a **"Show on this screen"** button: requests fullscreen, white background, marker as large as possible, and `navigator.wakeLock.request('screen')` if available.
  - Has a link back to `/ARena/`.
- [ ] Step 7: `npm test && npm run build`. Then use `npm run dev` and open `/ARena/marker/` to verify it renders (the agent can use a headless screenshot via Vite preview + Playwright if available; otherwise skip and note it).
- [ ] Step 8: Commit `feat(tracker): framed-QR marker layout, SVG card, marker page`.

### Task 3: Synthetic test renderer + math utilities

**Files:** `packages/tracker/src/math/{vec3,quat,mat4}.ts`, `packages/tracker/src/pose/intrinsics.ts`, `packages/tracker/test/synth.ts`, `packages/tracker/test/math.test.ts`, `packages/tracker/test/synth.test.ts`.

**Interfaces:**
- Produces:
```ts
// math: plain tuples, no classes
export type Vec3 = [number, number, number]; export type Quat = [number, number, number, number] // x,y,z,w
export type Mat3 = Float64Array /*9, row-major, internal CV use*/; export type Mat4 = Float32Array /*16, column-major*/
// quat.ts: quatMultiply, quatInvert, quatSlerp, quatFromAxisAngle, quatFromMat3, quatToMat3, quatAngle(a,b) (radians between)
// mat4.ts: mat4FromRotationTranslation(q, t), mat4Invert, mat4Multiply, mat4Identity
// intrinsics.ts
export interface Intrinsics { fx: number; fy: number; cx: number; cy: number; width: number; height: number }
export function intrinsicsFromSize(width: number, height: number, hfovDeg = 65): Intrinsics
// test/synth.ts
export interface SynthPose { R: Mat3; t: Vec3 } // OpenCV camera-from-marker, marker frame in metres (see below)
export interface SynthOptions { width?: number; height?: number; hfovDeg?: number; markerSizeM?: number; background?: number | ((x: number, y: number) => number); noise?: number; blurPx?: number; supersample?: number; clutter?: Array<{ corners: [number, number][]; value: number }> }
export function renderSynthetic(pose: SynthPose, opts?: SynthOptions): { image: GrayImage; K: Intrinsics; cornersPx: [number, number][] /* TL,TR,BR,BL of the frame outer square */ }
export function lookAtPose(opts: { distanceM: number; tiltDeg: number; yawDeg: number; rollDeg: number; offsetPx?: [number, number] }): SynthPose
```
Internal CV marker frame (OpenCV-side): the marker lies in its z=0 plane, x right, y **down** (towards the card bottom), origin at the centre. Normalized marker coords: (u,v) = (x/S + 0.5, y/S + 0.5). `GrayImage` = `{ width: number; height: number; data: Uint8Array }` (define it in `src/cv/image.ts` in this task).
- [ ] Step 1: Write the failing math tests: quat multiply/invert round trip, slerp halfway, mat4 invert × original ≈ identity, `intrinsicsFromSize(640,480,65)` gives `fx ≈ 502.3` and `cx = 320`.
- [ ] Step 2: Write the failing synth tests:
  - A front-facing pose at 0.3 m renders with the marker's `cornersPx` symmetric around the image centre.
  - Pixels at a frame corner are dark (< 60) and the background is light (> 180).
  - `cornersPx` matches projecting the marker corners through K within 0.01 px.
- [ ] Step 3: Run. Expected: FAIL.
- [ ] Step 4: Implement the renderer. For each pixel (with `supersample` × `supersample` subsamples, default 3), back-project the ray, intersect the plane z=0 in marker coords, convert to (u,v), and use `sampleMarker`: dark = 20, white = 235. Otherwise use the background. Then add clutter quads (filled polygons), optional box blur, and Gaussian noise (seeded PRNG, e.g. mulberry32, so tests are deterministic).
- [ ] Step 5: Run. Expected: PASS.
- [ ] Step 6: Commit `test(tracker): synthetic marker renderer and math utils`.

### Task 4: CV primitives (port from js-aruco2)

**Files:** `packages/tracker/src/cv/{image,threshold,contours,poly}.ts`, `packages/tracker/test/cv.test.ts`.

**Interfaces:**
- Consumes: `GrayImage`, `renderSynthetic`.
- Produces:
```ts
export function toGray(rgba: Uint8ClampedArray, width: number, height: number, out?: GrayImage): GrayImage
/** Binary output: 255 where pixel is darker than (local mean − offset), else 0. Local mean via integral image over a (2r+1)² box. */
export function adaptiveThreshold(src: GrayImage, radius: number, offset: number, out?: GrayImage): GrayImage
export type Contour = Array<{ x: number; y: number }>
export function findContours(binary: GrayImage): Contour[]   // Suzuki-style border following as in js-aruco2 CV.findContours
export function approxPolyDP(contour: Contour, epsilon: number): Contour
export function isContourConvex(contour: Contour): boolean
export function perimeter(contour: Contour): number
```
- [ ] Step 1: Write failing tests:
  - A gray image with a filled 100×100 black square on white → `adaptiveThreshold(img, 7, 7)` marks the square border pixels 255. `findContours` + `approxPolyDP(c, 0.05*perimeter)` yields exactly one convex 4-gon whose vertices are within 1.5 px of the true corners.
  - **Glare test (Review Focus 5):** a synthetic front-facing marker with a strong left-to-right brightness gradient (background 255→120) and a bright specular disc (value 255, radius 30 px) over part of the frame still produces a convex 4-gon matching `cornersPx` within 2 px.
  - Blank white image → zero contours of perimeter > 40.
- [ ] Step 2: Run. Expected: FAIL.
- [ ] Step 3: Port the code from js-aruco2 `src/cv.js` (`adaptiveThreshold`, `findContours`, `approxPolyDP`, `isContourConvex`, `perimeter`) to typed TS. Use an integral image for adaptive threshold (O(1) per pixel) instead of stackBoxBlur. Reuse `out` buffers so nothing is allocated per frame. Keep the MIT notice at the top of each ported file.
- [ ] Step 4: Run. Expected: PASS.
- [ ] Step 5: Add a micro-benchmark test (`it.skip` by default, enabled with `BENCH=1`) that logs the ms for threshold + contours on a 640×480 synthetic frame. Target < 8 ms on desktop Node.
- [ ] Step 6: Commit `feat(tracker): CV primitives ported from js-aruco2`.

### Task 5: Framed-QR detector (quads, orientation, corner refinement)

**Files:** `packages/tracker/src/cv/homography.ts`, `packages/tracker/src/detect/framedQr.ts`, `packages/tracker/src/detect/refine.ts`, `packages/tracker/test/detect.test.ts`.

**Interfaces:**
- Consumes: CV primitives, `buildQrGrid`, `finderCentres`, `FRAME_THICKNESS`, `QUIET_ZONE`.
- Produces:
```ts
// Mat3 comes from src/math (Task 3)
export function homographyFromQuad(src: [number, number][], dst: [number, number][]): Mat3 | null // 4-point DLT, null if degenerate
export function applyHomography(H: Mat3, x: number, y: number): [number, number]
export interface Detection { corners: [[number, number], [number, number], [number, number], [number, number]] /* image px, order = marker TL, TR, BR, BL (upright marker) */; score: number /* 0..1 finder confidence */; areaPx: number }
export interface DetectOptions { thresholdRadius?: number; thresholdOffset?: number; minPerimeterPx?: number /*default 120*/ }
export function detectFramedQr(img: GrayImage, grid: MarkerGrid, opts?: DetectOptions, scratch?: DetectScratch): Detection | null
export function refineCorners(img: GrayImage, quad: [number, number][]): [number, number][] // fit a line per edge with gradient-weighted points, intersect adjacent lines
```
Algorithm:
1. Run adaptive threshold (radius ≈ max(7, width/80)) and find contours.
2. For each contour with perimeter ≥ `minPerimeterPx`, run approxPolyDP(0.03·perimeter). Keep convex 4-gons with min edge ≥ 10 px, sorted by area descending.
3. For each candidate (max 5), take the outer quad, compute H from unit square → quad, and validate:
   1. Sample about 16 points at the middle of the frame band (u = FRAME_THICKNESS/2 along each side): ≥ 90% dark.
   2. Sample about 16 quiet-zone points: ≥ 80% light.
   3. Sample a 3×3 patch at each of the 4 corner finder positions under the 4 cyclic rotations. The rotation where 3 positions are "finder-like" (centre dark, the ring at 2.5 modules from the centre light, the ring at 3.5 modules dark) and the 4th is not wins. Score = fraction of checks passed.
4. Reorder the corners to marker TL, TR, BR, BL for the winning rotation.
5. Refine the corners. Return the best candidate with score ≥ 0.75, else `null`.
- [ ] Step 1: Write failing tests (use `renderSynthetic`, 640×480, 65° HFOV, 50 mm marker):
  - Front-facing at 0.3 m: detection exists, and its corners match `cornersPx` within 0.5 px after refinement.
  - Roll 0/90/180/270° (+ 37°): the corner order always matches `cornersPx` (TL stays TL).
  - Tilt 50°, yaw 30°, distance 0.45 m: detected, corner error < 1 px.
  - Small marker: the distance where the marker is about 45 px wide (≈0.55 m) → still detected with noise σ=6 and blur 1 px.
  - **Clutter (Review Focus 1):** 3 extra black-bordered rectangles (cards, a phone bezel shape: a thick dark border with a light inside) plus the marker → exactly the marker is returned. The same clutter without the marker → `null`.
  - **Cropped (Review Focus 2):** the marker shifted so one corner is outside the image → `null` (never a wrong quad).
  - Random noise image → `null`.
- [ ] Step 2: Run. Expected: FAIL.
- [ ] Step 3: Implement the homography (normalized DLT with an 8×8 Gaussian elimination solve), the detector and the refinement.
- [ ] Step 4: Run. Expected: PASS. Then tune the thresholds only via `DetectOptions` defaults, never by weakening the tests.
- [ ] Step 5: Commit `feat(tracker): framed-QR detector with orientation and subpixel corners`.

### Task 6: Pose estimation

**Files:** `packages/tracker/src/pose/pose.ts`, `packages/tracker/test/pose.test.ts`.

**Interfaces:**
- Consumes: `Detection`, `Intrinsics`, `homographyFromQuad`, math utils.
- Produces:
```ts
export interface CvPose { R: Mat3; t: Vec3; reprojErrorPx: number } // OpenCV camera-from-marker (marker y down, see Task 3)
export function estimatePose(corners: Detection['corners'], K: Intrinsics, markerSizeM: number): CvPose | null
/** Converts to the Global-Constraints world (marker, Y up) and camera (three.js) conventions: returns world-from-camera. */
export function poseToWorldFromCamera(p: CvPose): { position: Vec3; quaternion: Quat; matrix: Mat4 }
```
Method:
1. H = homography from the marker-plane corners (±S/2) to K⁻¹-normalized image points.
2. λ = 2/(‖h1‖+‖h2‖); r1 = λh1, r2 = λh2, r3 = r1×r2, t = λh3. If t.z < 0, flip the signs of r1, r2 and t.
3. Orthonormalize R via polar decomposition (3 iterations of R ← (R + R⁻ᵀ)/2).
4. Run 5 Gauss-Newton iterations on 6-DoF reprojection error (numeric Jacobian is fine).
5. Return null if any value is non-finite or reprojErrorPx > 3.

Conversion: world-from-marker-cv has the axes `x_w = x_cv`, `y_w = −z_cv` (up out of the card), `z_w = y_cv`. Camera three-from-cv: flip y and z. world_from_cam = W·(R,t)⁻¹·C.
- [ ] Step 1: Write failing tests:
  - Feed the exact `cornersPx` from `renderSynthetic` for 10 random poses (seeded; distance 0.2–0.7 m, tilt 0–60°, any roll). The recovered t is within 1 mm and the R angle error is < 0.5°.
  - With ±0.5 px corner noise: t within 5 mm and the angle < 2° at 0.4 m.
  - `poseToWorldFromCamera` of a camera 0.4 m straight above the marker centre looking down, image-up = marker top → position ≈ (0, 0.4, 0), and the camera's −Z axis maps to world −Y.
  - **Degenerate (Review Focus 2):** collinear corners → `null`, never NaN.
- [ ] Step 2: Run. Expected: FAIL.
- [ ] Step 3: Implement.
- [ ] Step 4: Run. Expected: PASS.
- [ ] Step 5: Commit `feat(tracker): homography pose with refinement and world conversion`.

### Task 7: Detection worker + frame pump

**Files:** `packages/tracker/src/worker/protocol.ts`, `packages/tracker/src/worker/detect.worker.ts`, `packages/tracker/src/worker/framePump.ts`, `packages/tracker/test/framePump.test.ts`.

**Interfaces:**
- Produces:
```ts
// protocol.ts
export type ToWorker = { type: 'init'; markerSizeM: number; url: string } | { type: 'frame'; id: number; timestamp: number; width: number; height: number; gray: ArrayBuffer /* transferred */; K: Intrinsics }
export type FromWorker = { type: 'result'; id: number; timestamp: number; pose: { position: Vec3; quaternion: Quat } | null; corners: Detection['corners'] | null; reprojErrorPx: number; detectMs: number; gray: ArrayBuffer /* returned for reuse */ }
// framePump.ts — pure scheduling logic, testable without DOM
export class FramePump { constructor(opts: { maxInFlight?: number /*1*/ }); canSend(): boolean; markSent(id: number): void; markDone(id: number): void; stats(): { sent: number; done: number; dropped: number; detectHz: number } }
```
The worker owns a `MarkerGrid` (built from `init.url`) and scratch buffers. Per frame: `detectFramedQr` → `estimatePose` → `poseToWorldFromCamera`. It posts the result and transfers the gray buffer back.
- [ ] Step 1: Write failing FramePump tests: with `maxInFlight` 1, `canSend` is false after `markSent` and true after `markDone`, and the frames skipped while busy count as `dropped`.
- [ ] Step 2: Run. Expected: FAIL.
- [ ] Step 3: Implement FramePump and the worker. Add `worker.handle.test.ts`, which calls the worker's exported `handleMessage(msg)` (factor the logic out of `onmessage`) on a synthetic frame and checks that the pose is ≈ the ground truth.
- [ ] Step 4: Run. Expected: PASS.
- [ ] Step 5: Commit `feat(tracker): detection worker and frame pump`.

### Task 8: IMU orientation, history, One Euro, fusion

**Files:** `packages/tracker/src/imu/{orientation,history}.ts`, `packages/tracker/src/fusion/{oneEuro,fusion}.ts`, `packages/tracker/test/{imu,fusion}.test.ts`.

**Interfaces:**
- Produces:
```ts
/** three.js DeviceOrientationControls math: euler(beta, alpha, -gamma, 'YXZ') · q(-√½,0,0,√½) · axisAngle(z, -screenAngle). Angles in degrees in, quaternion out (device camera orientation in an arbitrary IMU world frame, Y up). */
export function deviceOrientationToQuat(alpha: number, beta: number, gamma: number, screenAngleDeg: number): Quat
export async function requestMotionPermission(): Promise<'granted' | 'denied' | 'unsupported'>
export class ImuHistory { constructor(capacity?: number /*120*/); push(t: number, q: Quat): void; at(t: number): Quat | null /* slerp between neighbours, nearest if outside */; latest(): { t: number; q: Quat } | null }
export class OneEuroFilter { constructor(minCutoff: number, beta: number, dCutoff?: number); filter(x: number, tSec: number): number; reset(): void }
export interface FusedPose { position: Vec3; quaternion: Quat; source: 'marker' | 'imu' | 'none'; confidence: number }
export class PoseFusion {
  constructor(opts?: { lostAfterMs?: number /*150*/; reacquireBlendMs?: number /*200*/; correctionRate?: number /*per second, 4*/; useImu?: boolean })
  onImu(t: number, q: Quat): void
  onMarker(frameTime: number, position: Vec3, quaternion: Quat, reprojErrorPx: number): void
  get(now: number): FusedPose
}
```
**Amendment (2026-10-07, controller ruling — gravity-locked):** a ~60 px marker viewed near-frontally gives 5–13° tilt error from 4 corners (ill-conditioning, measured in Task 6). Since the marker lies flat on a table, when IMU is available **the offset is constrained to a rotation about world +Y (yaw only)**: compute the target as before, then keep only its yaw component (swing-twist decomposition about +Y) before slerping. IMU supplies tilt (gravity-referenced). **Position is re-solved with the fused rotation**: `onMarker` receives the detection corners + intrinsics and calls `solveTranslationGivenRotation(corners, K, markerSizeM, worldFromCameraQuatToCvR(fusedQ))` (Task 6), then `poseToWorldFromCamera` for the camera position, then the One Euro filter. Signature becomes `onMarker(frameTime, marker: { position: Vec3; quaternion: Quat; corners: Detection['corners']; K: Intrinsics; markerSizeM: number; reprojErrorPx: number })`. Without IMU, fall back to the full marker pose as described below. Add a test: IMU tilt is correct and marker rotation is noisy by ±8° in tilt → fused tilt error < 1.5° and the position error stays < 1 cm at 0.4 m.

Fusion model:
- Keep `offset: Quat` such that `worldFromCam ≈ offset · imuQ`.
- On a marker sample at `frameTime`, `target = markerQ · inverse(imuHistory.at(frameTime))`. If no marker was seen in the last `lostAfterMs`, or the angle(offset, target) > 20°, start a `reacquireBlendMs` blend to target. Otherwise slerp offset toward target by `1 − exp(−correctionRate·dt)`.
- Position comes from the marker, through a One Euro filter per axis (minCutoff 1.0, beta 20). It is held when lost.
- `get(now)`: the rotation = offset · latest imuQ, so the IMU gives the rotation at full rate.
- `source` = 'marker' if a marker arrived within `lostAfterMs`, 'imu' if any marker was ever seen and the IMU is live, else 'none'.
- If `useImu` is false or there is no IMU data, rotation comes from the marker quaternion with the One Euro filter (via slerp toward the new sample with alpha from the filter's cutoff) and source = 'marker'/'none'.
- [ ] Step 1: Write failing tests:
  - `deviceOrientationToQuat(0, 90, 0, 0)` (phone upright, portrait) → camera looks along the horizontal −Z. `(0, 0, 0, 0)` (flat, screen up) → camera −Z points to world −Y.
  - **Rotation (Review Focus 3):** the same physical pose with screenAngle 90 differs from 0 by a 90° roll about the camera Z.
  - ImuHistory: interpolation at the midpoint gives the slerp midpoint, and the capacity wraps.
  - OneEuro: a constant input converges, and a step input with large beta follows within 3 samples.
  - PoseFusion:
    - Scripted scenario: the IMU rotates the camera at a constant 30°/s while markers arrive at 15 Hz with a 60 ms latency. The fused rotation error vs the ground truth stays < 1.5° after a 0.5 s warm-up.
    - Markers stop: source turns 'imu' after 150 ms, the rotation keeps following the IMU, and the position stays constant.
    - Markers resume after the IMU drifted 10°: the error drops below 1° within 300 ms, with no single-frame jump > 5°.
    - No IMU: source 'marker', and the output follows the markers.
- [ ] Step 2: Run. Expected: FAIL.
- [ ] Step 3: Implement.
- [ ] Step 4: Run. Expected: PASS.
- [ ] Step 5: Commit `feat(tracker): IMU orientation, history, One Euro and pose fusion`.

### Task 9: `createTracker` public API

**Files:** `packages/tracker/src/tracker.ts`, `packages/tracker/src/index.ts`, `packages/tracker/test/tracker.test.ts`, extend `packages/tracker/src/pose/intrinsics.ts`.

**Interfaces:**
- Produces (public API — exactly this):
```ts
export type TrackerStatus = { state: 'idle' | 'starting' | 'tracking' | 'lost' | 'error'; reason?: TrackerError }
export type TrackerError = 'camera-denied' | 'no-camera' | 'insecure-context' | 'unknown'
export interface TrackerOptions { video: HTMLVideoElement; markerSizeMm?: number /*50*/; hfovDeg?: number /*65*/; detectWidth?: number /*640*/; useImu?: boolean /*true*/; url?: string /*DEFAULT_URL*/ }
export interface TrackerPose { matrix: Float32Array; position: Vec3; quaternion: Quat; source: 'marker' | 'imu' | 'none'; confidence: number }
export interface Tracker {
  start(): Promise<void>            // must be called from a user gesture; requests camera (facingMode environment, ideal 1280×720) + motion permission; rejects with Error whose .code is TrackerError
  stop(): void
  getPose(): TrackerPose
  /** Projection for a canvas of viewW×viewH showing the video with object-fit: cover. */
  projectionMatrix(viewW: number, viewH: number, near: number, far: number): Float32Array
  onStatus(cb: (s: TrackerStatus) => void): () => void
  stats(): { detectHz: number; detectMs: number; reprojErrorPx: number; imu: boolean; corners: Detection['corners'] | null; videoW: number; videoH: number }
  setOptions(o: Partial<Pick<TrackerOptions, 'hfovDeg' | 'detectWidth' | 'useImu'>>): void
}
export function createTracker(opts: TrackerOptions): Tracker
// intrinsics.ts addition
export function projectionForCover(K: Intrinsics, viewW: number, viewH: number, near: number, far: number): Float32Array
```
Loop:
1. A `requestVideoFrameCallback` loop (with a `requestAnimationFrame` fallback) draws the video into an OffscreenCanvas at `detectWidth` × the proportional height, then runs `toGray`.
2. If `FramePump.canSend()`, it posts the frame with `timestamp = performance.now()` and K scaled to the detect size.
3. `deviceorientation` listener → `fusion.onImu(performance.now(), deviceOrientationToQuat(..., screen.orientation?.angle ?? window.orientation ?? 0))`.
4. Status transitions: `tracking`/`lost` derived from the fusion source.
- [ ] Step 1: Write failing tests (Vitest with mocked globals):
  - **Review Focus 4:**
    - `navigator.mediaDevices` undefined → `start()` rejects with code `'no-camera'` and the status becomes `error`.
    - `getUserMedia` rejecting with `NotAllowedError` → `'camera-denied'`.
    - `isSecureContext` false → `'insecure-context'`.
  - Motion permission `'denied'` → `start()` still resolves and `stats().imu === false`.
  - **Review Focus 3:** `projectionForCover` for video 1280×720 shown in a portrait 390×844 view versus a landscape 844×390 view. Project the 4 marker corners for a known pose with each matrix: their screen positions match the positions of the corresponding video pixels under object-fit: cover, within 1 px.
- [ ] Step 2: Run. Expected: FAIL.
- [ ] Step 3: Implement. Export the public API from `index.ts`, plus `renderMarkerSvg`, `renderCardSvg`, `buildQrGrid`, `DEFAULT_URL`.
- [ ] Step 4: `npm test && npm run typecheck && npm run lint`. Expected: PASS.
- [ ] Step 5: Commit `feat(tracker): createTracker public API`.

### Task 10: Tracker demo page

**Files:** `demo/tracker.html`, `src/demo/tracker-demo.ts`, `src/demo/demo.css`.

**Interfaces:** Consumes the public `createTracker` API only (import from `@arena/tracker`).
- [ ] Step 1: Build the layout.
  - A full-viewport `<video playsinline muted autoplay>` with `object-fit: cover`, and a transparent three.js canvas on top.
  - A big **Start** button that, in order: calls `document.documentElement.requestFullscreen?.()` (ignore failure), runs `tracker.start()`, and hides itself.
  - On error, show the reason text and a **Retry** button.
- [ ] Step 2: The three.js scene, in world (marker) coordinates:
  - an RGB axes helper 3 cm long at the origin;
  - a wireframe 5 cm box sitting on the marker;
  - a translucent disc of diameter 15 cm (3× the marker) at y = 0.005 to preview the arena.
  - Each frame: `camera.matrixAutoUpdate = false`, `camera.matrixWorld.fromArray(pose.matrix)`, `camera.matrixWorldInverse` = invert, `camera.projectionMatrix.fromArray(tracker.projectionMatrix(w, h, 0.01, 10))` plus its inverse. Hide the objects while `source === 'none'`.
- [ ] Step 3: The HUD (top-left, small monospace) shows source (coloured: green marker / amber imu / red none), detect Hz, detect ms, reproj px, IMU on/off, and video res.
  - Toggles: **Gyro** on/off (`setOptions({useImu})`), **Detect res** 480/640/960, an **HFOV** slider 50–80°, and **Show corners**, which draws the detected corners on a 2D overlay canvas, mapped through the cover transform.
  - Remember the settings in localStorage with try/catch.
- [ ] Step 4: Handle `resize` and `orientationchange`, which update the renderer size and the projection. Cap the pixel ratio at 2.
- [ ] Step 5: Run `npm run build && npx vite preview`. Check that the desktop page loads without console errors and that Start without a camera shows the 'no-camera' message (use the Chrome extension if available, else Playwright headless).
- [ ] Step 6: Commit `feat(demo): tracker demo page`.
- [ ] Step 7 (orchestrator): push and wait for the Pages deploy.

### Task 11: Client feasibility test on real devices (human checkpoint)

- [ ] Orchestrator sends the client:
  - the link https://gevrai.github.io/ARena/marker/ (print the card, or use "Show on this screen" on a second device);
  - the link https://gevrai.github.io/ARena/demo/tracker.html.
- [ ] The client tests on Android (and optionally the iPhone 6s) and reports:
  1. Does the cube sit on the card without visible swimming?
  2. Max distance and tilt that still track?
  3. Does it work with the marker shown on a phone screen?
  4. With the card hidden, does the cube stay roughly in place while turning the phone, and snap back smoothly?
  5. Detect Hz and ms shown in the HUD.
  6. Portrait ↔ landscape.
- [ ] Decide: framed QR is good enough → M2. Otherwise tune (detect res, thresholds, FOV, filter params), or switch to the ArUco fallback (a new `Detector` behind the same worker protocol), and retest.
- [ ] Update `docs/STATUS.md` with the results and decision. Commit.
