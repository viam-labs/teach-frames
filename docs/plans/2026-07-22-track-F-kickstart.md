# Track F — kickstart for a later brainstorming session

**Date:** 2026-07-22
**Status:** context capture only (NOT a design). A separate workstream *after*
PR #6 merges. Read the remaining-work handoff + roadmap memory first, then
brainstorm each item.

F is the tail of the teach-pendant roadmap (A+B → C → D → E → G all shipped). Two
independent, lower-priority items. **Both need backend + frontend** (unlike the
A–D pure ports; like E). Suggested order below.

## F1 — 2-point "Line" frame-define method (smaller; natural extension of A)

A 4th `define_frame` method (today: `3point` / `point` / `tcp_snapshot`): **2
captured points define a line/axis**. Origin at P0, +X toward P1; the other two
axes are underdetermined by 2 points, so they need a convention.

**Reusable seams (most of the machinery already exists from track A):**
- The guided wizard is already **method-parameterized**
  (`frontend/src/lib/wizard/frameDefine.svelte.ts`): `requiredCaptures` is 3 for
  `3point`, else 1 — extend it to return **2** for `line`.
- The pure preview helper `frontend/src/scene/provisionalTriad.ts` and the
  wizard panel's method selector (`METHOD_LABELS`/`METHOD_HINTS`/prompts) are the
  spots to add the `line` case.
- Backend: a new `frames.ComputeLine(p0, p1)` alongside `ComputeThreePoint` /
  `ComputePoint` in `frames/compute.go`, and a `case "line"` in
  `models/posetracker/docommand.go` `define_frame` (needs `len(buf) == 2`). Add
  `'line'` to `FrameMethod` in `frontend/src/lib/poseTracker.ts`.

**Open questions to brainstorm:**
- **The orientation convention for the free axes.** Options: project a world axis
  (e.g. world +Z) to fix +Z, then +Y = Z×X (fails if the line is vertical);
  or let the operator pick the "up" hint; or align the remaining axes to the
  parent frame. Pick one, document the degenerate case (line parallel to the
  chosen up), and pin it with a non-symmetric backend fixture (as `compute.go`
  frames do). Match the Go convention exactly in `provisionalTriad`.
- **Preview**: a provisional triad at P0 with +X toward P1 (2 markers + a line +
  the triad) — the plugin already draws markers + a preview line for `3point`;
  generalize to 2 captures.
- Naming/UX: is "Line" the right operator-facing label vs "2-point axis"?

## F2 — waypoint record / replay (bigger; new subsystem)

Record a sequence of arm poses (waypoints), then replay them (move through the
sequence). New backend subsystem + a frontend panel.

**What exists to build on:**
- Absolute motion primitives shipped in E1: `move_to_pose` / `move_to_joints`
  (arm-direct; now world-frame per track G), the global **STOP** (`stop_arm` +
  `requestStop`), and `withMove`/`motion.busy` pacing. A replay is essentially a
  sequenced series of `move_to_*` calls.
- The confirm/safety patterns from E1 (two-step confirm, always-visible STOP) and
  the persist layer (`persist/`) if waypoints should survive restarts.

**Open questions to brainstorm (this is the design-heavy one):**
- **Where do waypoints live?** A new backend store + DoCommands
  (`record_waypoint` = capture current pose/joints; `list/delete/clear_waypoints`;
  `replay_waypoints`), persisted to config like frames? Or frontend-only
  (a session list the panel replays via `move_to_*`)? Persistence + a replay
  command is more robust but more backend.
- **Pose vs joints waypoints** (or both) — joints replay is more deterministic;
  pose replay is more portable. Which does the operator want?
- **Replay safety** — replay commands a *sequence* of real moves. Needs: a
  prominent confirm before starting, STOP that halts mid-sequence (the
  press-and-hold jog loop's `stopSeq` pattern generalizes — check `stopSeq`
  between waypoints), speed/pausing, and clear "moving to waypoint N/M" feedback.
  This is the hardest UX (a robot moving unattended through a path).
- **Replay is NOT motion planning** — sequential `move_to_*` calls each plan
  point-to-point; there's no continuous trajectory/blending. Set expectations
  (or scope in a motion-service trajectory later).
- Home: its own panel/toggle, or a mode in the Jog panel next to Move-to?

**Recommendation for the later session:** brainstorm F1 first (small, mostly a
port-extension of the shipped frame wizard, low risk), ship it, then take on F2
(waypoints) as its own design — F2's replay-safety UX is the real design problem
and deserves a dedicated brainstorm like hand-eye (D) got.

## Constraints still to honor
- The `world_state_store` mirror stays configured (main Viam app renders frames);
  the teach-pendant remounts the Visualizer on our frame mutations (Bug-1 stopgap)
  — the proper motion-tools upstream fix is documented at
  `docs/plans/2026-07-20-motion-tools-wss-reconfigure-upstream-patch.md`.
- Keep `@viamrobotics/sdk` aligned with what `@viamrobotics/motion-tools` declares
  (the Bug-2 lesson).
