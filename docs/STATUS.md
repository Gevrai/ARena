# Status

**Current milestone:** M1, Tracker feasibility. Plan: `docs/superpowers/plans/2026-10-07-m1-tracker-feasibility.md`

| Task | State | Notes |
|---|---|---|
| 1 Scaffold + Pages deploy | done | |
| 2 Marker layout + page | done | client OK, live at /ARena/marker/ |
| 3 Synth renderer + math | done | |
| 4 CV primitives | done | |
| 5 Framed-QR detector | done | |
| 6 Pose | done | low-tilt ill-conditioned → gravity-locked fusion |
| 7 Worker + pump | done | |
| 8 IMU + fusion | done | |
| 9 createTracker API | done | |
| 10 Demo page | done | live at /ARena/demo/tracker.html |
| 11 Client device test | in progress | |

## Decisions log
- 2026-10-07: spec approved; AR tracker first; repo Gevrai/ARena; Pages URL https://gevrai.github.io/ARena/
- Client devices: Android (primary), iPhone 6s / iOS 15 (secondary)
- 2026-10-07: gravity-locked tracking (marker assumed flat on table)
- 2026-10-08: tracker behaviour reverted to 68145bb (phone-verified good rotation); only observational diagnostics stats (hit rate, outcome strip, latency) kept; new ideas to return as opt-in toggles.
- 2026-10-08: opt-in `useAccel` (accelerometer dead-reckoning of translation between/after detections, default off, demo checkbox "Accelerometer (experimental)"); needs phone validation of axis signs via HUD vector.
- 2026-10-08: accel rework (no marker-slope velocity seed, continuous leaky velocity, frozen window; fixed 3 s sample-prune jump), camera/grab/pump HUD diagnostics, opt-in `shortExposure` + accel bar meters in demo; needs phone validation.
