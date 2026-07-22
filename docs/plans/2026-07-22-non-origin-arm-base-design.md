# Non-origin arm-base correctness (track G) — design

**Date:** 2026-07-22
**Branch:** `redesign/visualizer-teach-pendant`
**Status:** design approved; ready for an implementation plan.

Track G — the "constraint G" that A–E deferred to: on a cell whose arm base is
not at the world origin, our scene overlays are offset. This resolves it and
unblocks general-cell support. **Backend-only** (a pleasant surprise — the
frontend auto-corrects once the data is world-frame).

## Root cause: mixed reference frames

Our overlays mix frames:
- `TcpTriad` + the frame-define preview line read `armState.pose`, which comes
  from `get_arm_state` → `arm.EndPosition` → the **ARM-BASE** frame, drawn at the
  scene root (world).
- The capture markers + provisional triads come from `capture_point` →
  `pt.source.Capture` (`motion.GetPose`, **destFrame/world**).

On an origin-base cell these coincide. Off-origin, the live TcpTriad is offset
from the (correct) capture markers even though they represent the same tool. The
committed frames + arm mesh are already correct (Visualizer frame system).

## Fix: report all TCP poses in the destination (world) frame

Chosen approach (brainstorming 2026-07-22): unify the *data* to world-frame rather
than compensate in rendering. `pt.source` (`MotionSource`, `GetPose(TCP, dest)`)
is the single source of truth `capture_point` already uses.

### Backend changes (all no-ops on an origin-base cell)

1. **`get_arm_state`** — read the TCP pose from `pt.source.Capture(ctx).Pose()`
   (world/destFrame) instead of `arm.EndPosition`. Joints still from
   `arm.JointPositions`. (Keep the arm guard: joints need the arm.)
2. **`jog_cartesian`** — the move computation is UNCHANGED (still reads
   `arm.EndPosition` and moves via `arm.MoveToPosition`); only the **response**
   `pose` becomes world: after the move, `pt.source.Capture(ctx).Pose()`. This
   keeps the live jog readout consistent during a press-and-hold (the poll is
   paused while `motion.busy`, so the response drives the readout).
3. **`move_to_pose`** — the target is now a **world** pose (the readout / "Load
   current" are world). Convert it to the arm-base frame and move arm-direct
   exactly as E1 (preserves the hardware-verified behavior; only the input frame
   changes):
   ```
   T := pt.armInDest.Capture(ctx).Pose()            // arm base in world = GetPose(arm, dest)
   armBaseTarget := Compose(PoseInverse(T), worldTarget)
   arm.MoveToPosition(ctx, armBaseTarget, nil)
   resp.pose := pt.source.Capture(ctx).Pose()       // world
   ```
   On an origin base `T` is identity → `armBaseTarget == worldTarget` → identical
   to today.

`jog_joint` / `move_to_joints` are joint-space and unchanged.

### New field: `armInDest`

Add `armInDest posesource.PoseSource` to `teachTracker`, set at construction
whenever the arm resolves:
`&posesource.MotionSource{Motion: mot, Component: cfg.Arm, DestFrame: dest}`
(the same shape as the existing eye-in-hand `flangeInDest`, but not gated on
`eye_in_hand`). Used only by `move_to_pose`. If it's nil (arm unresolved),
`move_to_pose` already errors on the arm guard.

### Frontend — NO code change

`TcpTriad`, the preview line, and the provisional/marker draws already render
`armState.pose` (and world captures) at the scene root. Once `armState.pose` is
world-frame, they are correct for any arm base and consistent with the capture
markers. Existing frontend tests pass unchanged.

## Two intended consequences

- **Jog readout + Move-to fields now show world coordinates** (were arm-base).
  This is an improvement — they now match the captured/committed frame values.
  Identical numbers on an origin-base cell.
- A small extra `motion.GetPose` round-trip per poll (~2 Hz, negligible) and per
  jog nudge / move (already paced to the arm). Acceptable.

## Testing

- **Backend Go** (fake `pt.source` / `pt.armInDest` via `posesource.Fake`, fake
  `inject.Arm`):
  - `get_arm_state`: pose comes from `pt.source` (queue a known world pose →
    assert the response pose equals it), joints from the arm.
  - `jog_cartesian`: the move still targets the `EndPosition`-derived pose
    (existing assertions unchanged), but the **response** pose equals the
    `pt.source` world pose (queue a distinct value to prove it's sourced from
    `GetPose`, not the move target).
  - `move_to_pose`: seed `pt.armInDest` with a **non-identity** arm base (e.g.
    translated + rotated); send a world target; assert `MoveToPosition` receives
    `Compose(PoseInverse(T), worldTarget)` (fixture with a tolerance); response is
    the `pt.source` world pose. Plus an **identity arm base → target unchanged**
    case (origin-base no-op) and the existing no-arm / bad-pose guards.
- **Frontend**: unchanged; existing suite green.
- **Hardware**: on a non-origin-base cell, TcpTriad/markers coincide with the real
  tool and the committed frames; on an origin-base cell, no visible change; jog +
  go-to-pose still land correctly.

## Out of scope

Rendering-side ECS parenting (the alternative approach, not taken). Changing
`move_to_pose` to motion-service planned motion (kept arm-direct to preserve E1).
Any change to joint-space commands.
