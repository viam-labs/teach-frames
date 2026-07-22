# Frame-relative jogging (track E2) — design

**Date:** 2026-07-22
**Branch:** `redesign/visualizer-teach-pendant`
**Status:** design approved; ready for an implementation plan.

Track E part 2 (E1 = go-to-pose/joints, shipped). Frame-relative jogging: jog the
TCP *along and about the axes of a chosen reference frame* — World, Tool, or a
taught frame. Backend + frontend (like E1).

## Current behavior

`jog_cartesian` (`models/posetracker/docommand.go`) reads `arm.EndPosition`
(arm-base frame) and:
- translation x/y/z → adds `step` along the **arm-base** axes (doc calls it
  "world");
- rotation roll/pitch/yaw → composes the delta about the **tool** frame;
then `arm.MoveToPosition(next)`. So today the basis is *split*: translate in
world/base, rotate in tool.

## Decisions (brainstorming, 2026-07-22)

1. **The reference governs BOTH translation and rotation** — a clean "jog in the
   selected frame" model (chosen over translation-only).
2. **Default reference = `world`** — translate + rotate in world/arm-base.
   This slightly changes the *default rotation* basis (tool → world); the operator
   selects **Tool** to recover tool-relative rotation (+ tool-relative
   translation). Confirmed acceptable.
3. **Origin-base scope** — taught-frame references assume
   `destFrame ≈ arm-base ≈ world`, the app-wide "constraint G" assumption
   (our scene draws, TcpTriad, etc. already make it). World/Tool references are
   always correct; only taught-frame references depend on it. Documented; joins
   the constraint-G backlog (best fixed holistically, not piecemeal here).

## Backend

`jog_cartesian` gains an **optional** `frame` string param. Resolve a **basis
rotation `R`** (a `spatialmath.Orientation`/rotation), then apply the step in it:

| `frame` value | basis `R` |
|---|---|
| omitted / `"world"` | identity (arm-base) — current translation behavior |
| `"tool"` | `EndPosition` current orientation |
| a taught frame name | `pt.store.GetFrame(name)` orientation (in `destFrame`) — origin-base caveat |

- **Translation** (x/y/z): world/base step vector `v = R.applyTo(axisUnit * step)`;
  `next = NewPose(cur.Point().Add(v), cur.Orientation())`.
- **Rotation** (roll/pitch/yaw): rotate the current pose about `R`'s corresponding
  axis. For `R = tool` this must reduce to the existing tool-frame compose; for
  `R = world` it rotates about the base axes; for a frame, about the frame axes.
  (Implementation: build the axis delta, conjugate/compose so the rotation is
  expressed about the `R` basis — the exact spatialmath form settled in the plan,
  with a fixture test pinning a non-symmetric case.)
- Unknown `frame` name (not world/tool/a stored frame) → descriptive error.
- No `frame` (or `"world"`) → byte-for-byte the current translation result (guard
  with a test that the existing jog behavior is unchanged).

`arm.MoveToPosition(next)` as today. Errors: no arm (existing), unknown frame.

## Frontend

- Builder: `jogCartesian(axis, step, frame?)` — `frame` optional; when set, include
  it in the command (`{ jog_cartesian: { axis, step, frame } }`). Omitted → current
  command shape (back-compat).
- `JogPanel.svelte` Cartesian mode: a **Reference** selector (segmented buttons or
  a select) — `World`, `Tool`, then each taught frame from a `list_frames` query
  (`createResourceQuery`, shared cache with the other panels). Default `World`.
  The jog-hold buttons pass the selected reference:
  `jogCartesian(axis, ±step, reference)`.
- If the reference is a taught frame that no longer exists (deleted), fall back to
  `World` (or disable) — keep the selection valid against the live frame list.
- Reference selector shows only in Cartesian mode; Joint / Move-to modes unaffected.

## Testing

- **Backend Go** (to jog-handler standard, using the `inject.Arm` fake):
  - `frame` omitted / `"world"` → same recorded target as the existing
    `TestJogCartesianTranslation` (no regression).
  - `"tool"` rotation → matches the existing tool-frame rotation result.
  - a taught frame (seed the store with a known non-identity orientation) →
    assert the recorded translation step is rotated by that orientation, and a
    rotation is about the frame axis (non-symmetric fixture).
  - unknown frame name → error, `MoveToPosition` not called.
- **Frontend**: `jogCartesian` builder with/without `frame` (unit test); the
  Reference picker wiring (panel is component-level, verified by check/build +
  hardware).
- **Hardware**: jog along/about a taught frame's axes moves as expected; World and
  Tool references unchanged from before.

## Out of scope

Non-origin arm-base correctness for taught-frame references (constraint-G backlog).
A 3D reference-axes overlay in the scene. Frame-relative go-to-pose (E1 is
arm-base only).
