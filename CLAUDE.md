# ARena (ballball)

AR party game: push friends' balls off an arena anchored on a business-card marker. Static site → https://gevrai.github.io/ARena/ (repo Gevrai/ARena, Vite `base: '/ARena/'`).

- Spec: `docs/superpowers/specs/2026-10-07-ballball-design.md` (source of truth)
- Plans: `docs/superpowers/plans/` · Research: `docs/research/`
- Status/progress: `docs/STATUS.md` (keep updated after each task)

## Working mode
- Main session = **project manager/orchestrator only**. Delegate all exploring, research, coding, testing and running to subagents with fresh context (Sonnet for real coding/debugging/research, Haiku for mechanical/simple tasks). Give each a self-contained brief: goal, files, constraints, acceptance check, and "report back concisely".
- The user is the client. Show working features when meaningful; ask them to test what agents can't (real phones: Android main, old iPhone 6s/iOS 15).
- Commit after each completed task (small commits, conventional messages). Pushing to `main` (which deploys Pages) is allowed.

## Conventions
- Strict TypeScript, npm workspaces (`packages/tracker` is standalone: no game or three.js imports).
- `src/sim` is pure (no DOM/three/net). Physics constants per-second.
- Tests: Vitest. Run `npm test` and `npm run typecheck` before claiming done.
- Dev on phones: `npm run dev -- --host` (HTTPS via basic-ssl).
