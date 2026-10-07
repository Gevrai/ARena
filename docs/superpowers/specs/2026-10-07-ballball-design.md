# BallBall — Design Spec

Date: 2026-10-07
Status: Draft for review
Source pitch: `PITCH.md`

## 1. Intent

A short, fun, arcade-style multiplayer game for friends sitting around a table, played on their phones. Each player controls a ball and tries to push the others off a small arena. The arena is anchored in **augmented reality** to a business-card-sized marker in the middle of the table (or shown on one phone's screen), so everyone sees the same arena from their own seat.

**Guiding principles**
- **No-fuss**: open a URL, tap Play, you're in. No install, no account, no backend.
- **AR is the headline feature**: it is built and validated first.
- **Start simple, stay extensible**: game modes, items, arena variants and alternate trackers must be addable without rewriting the core.
- **Friendly, not competitive**: no anti-cheat, no validation of clients, no player limit.

**Success criteria for the MVP**
- A printed business card (or a phone showing the marker) on a table is tracked well enough that the arena feels "on the table", and the arena stays visible (gyro-driven) when the marker is briefly lost.
- 2+ friends on the same wifi can host/join with a code or QR, roll around in a shared lobby, and play a best-of-rounds match that feels responsive.
- Runs in Chrome on recent iOS and Android phones, deployed as a static site on GitHub Pages.

## 2. Platform & constraints

- Static site on GitHub Pages; no backend. The public PeerJS cloud server is used for signaling only; gameplay traffic is peer-to-peer WebRTC.
- No TURN relay: same-wifi play is the supported case; the UI recommends it.
- Chrome on iOS is WebKit: no WebXR, no `BarcodeDetector`, and no element fullscreen on iPhone. AR is therefore camera feed + JS computer vision on all platforms.
- Camera requires HTTPS: GitHub Pages in production, `vite --host` with a local HTTPS cert in development.
- Tooling: Vite, TypeScript (`strict`), Three.js, Vitest, ESLint + Prettier, npm workspaces.

## 3. Repository layout

```
/                       game app (Vite multi-page)
  index.html            the game
  demo/tracker.html     tracker demo page
  marker/index.html     printable / on-screen marker page
  src/
    sim/                pure simulation
    net/                transport, protocol, host/client sessions
    render/             Three.js scene
    input/              joystick + buttons
    ui/                 DOM panels and screens
    app/                app state machine, config, tracker adapter
  scripts/gen-marker.ts generates marker SVG/PNG from the Pages URL
packages/tracker/       standalone AR tracking library (no game, no Three.js deps)
.github/workflows/      build + deploy to GitHub Pages on push to main
```

Dependency rule: `sim` imports nothing from other modules. `packages/tracker` imports nothing from the game. `render` reads state, never mutates it. Enforced with ESLint `no-restricted-imports`.

## 4. Tracker package (`packages/tracker`)

### 4.1 Marker: "framed QR"
- Business-card size (≈50 mm square target area). A thick black square frame around a QR code encoding the game URL. The same image is printed or displayed full-screen on a phone (`/marker`).
- Scanning with a normal camera app opens the game.
- Detection: adaptive threshold → contours → quad approximation → candidate quads whose inner region is bordered correctly. The **4 frame corners** give the pose; **orientation** comes from the QR's three finder patterns (sampled at the inner corners — the one corner without a finder pattern identifies rotation). No QR decoding per frame.
- Pose: homography from 4 corners → camera pose (IPPE or homography decomposition) using known marker size and an approximate camera intrinsic matrix (typical phone FOV; configurable).
- Fallback detector, only if the framed QR proves too weak: single ArUco marker on the front of the card, QR on the back. Both detectors implement the same `Detector` interface.

### 4.2 Pipeline
- `getUserMedia` (rear camera) → `<video>` displayed behind the transparent game canvas.
- Frames downscaled to ~640 px wide grayscale and sent to a **Web Worker** for detection; rendering never waits on detection.
- Screen orientation (portrait/landscape) is accounted for in both the image → camera mapping and the IMU axes.

### 4.3 Sensor fusion
- `DeviceOrientationEvent` gives device rotation; it runs **continuously**, not only when the marker is lost.
- Complementary filter: gyro supplies high-frequency rotation, marker detections correct drift and supply position.
- Marker lost: position held at last known value, rotation follows the gyro, so the arena keeps moving around the screen as in AR. No accelerometer-based position.
- Marker re-acquired: blend to tracked pose over ~200 ms.
- Remaining jitter smoothed with a One Euro filter (position) and adaptive slerp (rotation).

### 4.4 Public API
```ts
const t = createTracker({ video, markerSizeMm: 50, detector: framedQr() })
await t.start()                    // camera + motion permission; must be called from a user gesture
t.getPose(): { matrix: Float32Array /*16*/, source: 'marker' | 'imu' | 'none', confidence: number }
t.projectionMatrix(near, far): Float32Array
t.onStatus(cb)                     // 'starting' | 'tracking' | 'lost' | 'error' (+ reason)
t.stop()
```
Matrices are plain column-major arrays; the game wraps them in Three.js types via a tiny adapter.

### 4.5 Testing
- Unit tests (Node/Vitest): synthesize the marker into an image buffer at known poses (distance, tilt, rotation, noise, blur) and assert the detector recovers corners/orientation/pose within tolerance; test the fusion filter with scripted pose/gyro sequences.
- Demo page (`/demo/tracker.html`): camera view with axes and a test cube drawn on the marker, live readouts of detection FPS, pose source, confidence, and a toggle to disable the gyro to compare.

## 5. Simulation (`src/sim`)

- Pure, deterministic-enough, serializable. `step(world, inputs, dt) → world` at a fixed tick rate.
- All physics constants are per-second so tick rate can change without changing feel.
- Arena: flat disc. Balls: equal-mass spheres. Physics: gravity, ground friction, sphere–sphere elastic-ish collisions, sphere–disc contact, falling off the edge. No physics engine.
- `Input`: `{ seq, move: {x, y} (arena space, |move| ≤ 1), jump: boolean, boost: boolean }` — jump/boost are "pressed this tick" edges.
- Jump: only when grounded. Boost: impulse in `move` direction, or current velocity direction if `move` is zero; 1 s cooldown stored in world state.
- A ball is **out** when it falls below a set height.
- `GameMode` interface: `spawnPositions`, `onTick`, `onPlayerOut`, `isRoundOver`, `scoreRound`, plus mode-specific serializable state. Future arena shrink / items / modes plug in here.
- Modes for MVP:
  - **FreePlay** (sandbox and lobby): no rounds; out balls respawn after ~1 s.
  - **LastBallStanding**: 3-2-1 countdown with inputs locked; last ball on the arena scores 1 point; if the last remaining balls all go out on the same tick, **each of them scores 1 point**; first to **3** wins; winner banner; return to lobby.

## 6. Netcode (`src/net`)

- **Transport**: thin wrapper over PeerJS with two data channels per peer: **reliable** (join/leave, name/color, ready, start, config) and **unreliable** (inputs, snapshots).
- **Room code**: 4 characters, mapped to PeerJS id `ballball-<CODE>`. The host shows the code and a QR linking to `<pages-url>/?join=<CODE>`.
- **Config**: `tickRate` (default 60) and `snapshotRate` (default 30) live in `src/app/config.ts`, overridable via URL params (`?tick=&snap=`), chosen by the host and sent to clients at match/lobby start. Also `?lag=<ms>&loss=<%>` to simulate bad networks.
- **Client → host**: one `Input` per tick, each packet carrying the last few unacknowledged inputs for loss resilience.
- **Host**: per-player input queue, consumes one per tick (small jitter buffer, drops backlog if it grows); repeats last input if missing. The host's own player is a zero-latency local client.
- **Host → clients**: full world snapshot at `snapshotRate`, including each player's current input and last processed `seq`. Everything is state (scores, round phase, lobby/ready state), not events.
- **Client presentation — full-world prediction**: on each snapshot, reset local world to it, then re-simulate up to the present using own unacknowledged inputs and each remote player's last known input. All balls are shown in the present, so collisions line up locally. Corrections are hidden by decaying a visual offset over ~100–150 ms.
- **Trust**: none needed; no validation or anti-cheat.
- **Disconnects**: a client leaving removes their ball. Host leaving ends the session for everyone ("Host left") and returns them to the sandbox. No host migration.

## 7. Rendering (`src/render`)

- Three.js, bright low-poly flat-shaded style, transparent canvas over the camera video in AR.
- Camera pose from the tracker adapter; projection from `tracker.projectionMatrix`.
- Arena floats slightly above the marker, diameter ≈ 3× marker size (tunable).
- Blob shadow under each ball (no shadow maps). Pixel ratio capped at 2.
- Diegetic UI: billboarded name tags above balls; floating scoreboard (colored dots + points) above the arena center; 3D text for countdown and "<Color> wins!".
- **No-AR mode**: arena centered on screen, camera slowly orbiting it; everyone sees the same thing.

## 8. Input (`src/input`)

- On-screen joystick bottom-left, **Jump** and **Boost** buttons bottom-right, in both portrait and landscape. Layout leaves room for a third button (future items).
- Boost button shows its cooldown as a ring.
- Joystick vector is rotated into arena space using the camera's yaw around the arena's up axis, so "up" means "away from me" wherever the player is (and follows the orbit in No-AR mode).
- Gestures: out of scope; to iterate on after MVP.

## 9. App flow & UI (`src/app`, `src/ui`)

1. **Landing**: one big **Play** button. The tap triggers camera + motion permission, tracker start, and fullscreen (where supported). On failure: error message with **Retry** and **Play without camera**. On iPhone (no fullscreen): a one-time hint to "Add to Home Screen" (web app manifest with `display: fullscreen`).
2. **Sandbox (home)**: local-only FreePlay world with the player's ball and 3 idle balls to push around — doubles as solo tech demo and AR check. Side panel (right edge in landscape, top in portrait): name (persisted in localStorage), color (auto, tap to change), **Host**, **Join** (code entry). Opening `?join=CODE` goes straight to joining after Play.
3. **Lobby**: shared networked FreePlay arena; everyone can roll around. Host panel shows code + QR. Each player toggles **Ready**; when all are ready, the host's **Start** enables.
4. **Match**: LastBallStanding. Players joining mid-match spectate until the next round. At match end, back to Lobby with Ready reset.

Errors: signaling server unreachable → message + Retry; room not found → message; camera denied → No-AR mode; motion denied → marker-only tracking; host left → back to sandbox with message.

Orientation: portrait and landscape both supported; scene and tracker adapt on rotation.

## 10. Milestones (in order)

1. **Tracker feasibility**: project scaffold, workspace, Pages deploy pipeline, marker generator + `/marker` page, `packages/tracker` (framed-QR detector, worker, pose, IMU fusion, smoothing) with unit tests, and `/demo/tracker.html`. Validate on real iOS and Android phones with a printed card and a phone screen. Decide whether the framed QR is good enough or switch to the ArUco fallback.
2. **Local sandbox in AR**: sim (FreePlay), rendering, controls, landing flow, fullscreen/permissions/fallbacks, No-AR mode. Playable solo with idle balls.
3. **Multiplayer**: PeerJS transport, host/join, lobby with ready, snapshots, full-world prediction + correction smoothing, LastBallStanding mode, disconnect handling, net debug overlay (ping, correction size, snapshot rate).
4. **Polish**: diegetic scoreboard/banners, art pass, tuning (physics feel, tracking, net rates), Add-to-Home-Screen hint.

## 11. Out of scope (MVP)

Bots; items/powerups; arena shrink; other game modes; gestures; host migration; TURN relay / mobile-data play; per-device camera calibration; anti-cheat.
