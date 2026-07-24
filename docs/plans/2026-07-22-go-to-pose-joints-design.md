# Go-to-pose / go-to-joints (track E1) — design

**Date:** 2026-07-22
**Branch:** `redesign/visualizer-teach-pendant`
**Status:** design approved; ready for an implementation plan.

Track E of the teach-pendant roadmap, part 1. **E splits into two independent
features**, both needing backend + frontend (unlike the A–D pure-frontend ports):

- **E1 (this doc): editable go-to-pose / go-to-joints** — move the arm to an
  *absolute* target with a confirm step.
- **E2 (deferred): frame-relative jogging** — jog along a taught frame's axes.

E1 chosen first (per brainstorming 2026-07-22).

## Why the backend lift is small

The jog handlers already compute an absolute target and use the exact move
primitives E1 needs (`models/posetracker/docommand.go`):
- `jog_joint` → reads `arm.JointPositions`, sets `next[joint]`, calls
  `arm.MoveToJointPositions(next)`.
- `jog_cartesian` → reads `arm.EndPosition` (arm-base frame), computes `next`
  pose, calls `arm.MoveToPosition(next)`.

So go-to-joints/pose reuse `arm.MoveToJointPositions` / `arm.MoveToPosition` with
an operator-supplied absolute target instead of current+step. The go-to-pose
target is in the **arm-base frame**, matching the `get_arm_state` (`EndPosition`)
readout — no motion-service or frame-composition complexity.

A **`stop_arm`** DoCommand (`arm.Stop`) already exists, and `StatusBar.svelte` has
the STOP-button logic — but **`StatusBar` is not mounted in the new shell**, so
there is currently **no STOP button anywhere**. E1 fills that safety gap.

## Backend (new DoCommands)

Two commands on the pose-tracker, mirroring the jog handlers:

| Command | Payload | Effect | Response |
|---|---|---|---|
| `move_to_joints` | `{"positions":[deg,…]}` | validate `len == len(current joints)`; convert degrees→radians `[]referenceframe.Input`; `arm.MoveToJointPositions` | `{"joints":[deg,…], "moved":true}` |
| `move_to_pose` | `{"pose":{x,y,z,o_x,o_y,o_z,theta}}` | build `spatialmath.NewPose(r3.Vector{x,y,z}, &OrientationVectorDegrees{OX,OY,OZ,Theta})`; `arm.MoveToPosition` (arm-base frame) | `{"pose":{…}, "moved":true}` |

- Both error clearly if no arm is configured (same pattern as `jog_*`).
- `move_to_joints` errors on wrong joint count.
- Both are interruptible by the existing `stop_arm` (`arm.Stop`).
- Tested to the jog-handler standard: known-input → asserts the arm move is called
  with the correctly-converted target (fake arm records the call); error paths
  (no arm, bad joint count, malformed pose).

## Frontend — "Move to" mode in the Jog panel + a global STOP

Extend `frontend/src/panels/JogPanel.svelte` (chosen home — reuses the arm-state
poll, live readout, gating, `motion` plumbing).

**Third mode** `[ Cartesian | Joint | Move to ]`. "Move to" has two target kinds
(matching the existing cartesian/joint split): a **Pose** target (editable x,y,z +
OV `o_x,o_y,o_z,theta`) and a **Joints** target (editable J0…Jn). Flow:
1. **Load current** — prefill the fields from the live `armState.pose` /
   `armState.joints` (the fields are a *frozen editable target*, separate from the
   live readout, so live polling doesn't fight the operator's edits).
2. Edit the fields.
3. **Review target** — a two-step confirm: shows the absolute target + a warning
   ("The arm will move to this absolute target."), with **Execute** and **Cancel**
   (Cancel focused, mirroring the ManageFrames destructive-confirm pattern).
4. **Execute** — `move_to_pose` / `move_to_joints` inside `withMove` (so
   `motion.busy` pauses the poll); disabled while `motion.busy > 0`.

**Global STOP** (safety-critical; the new shell has none): port `StatusBar`'s
`handleStop` — `requestStop()` (bumps `stopSeq` to bail any jog loop) + a
dedicated always-callable `stop` mutation firing `stopArm()`. Render it as a
prominent, always-visible STOP (never gated by `motion.busy`, so it interrupts an
in-flight absolute move via the concurrent `arm.Stop` DoCommand). It also covers
jogging, which currently has no explicit STOP.

## A small "move-to target" store

A pure rune helper (`src/lib/wizard/moveToTarget.svelte.ts` or inline) modelling
the confirm phase + editable fields is worth unit-testing:
```
kind: 'pose' | 'joints'
phase: 'edit' | 'confirm'
poseFields: { x,y,z,o_x,o_y,o_z,theta }; jointFields: number[]
loadFromPose(pose); loadFromJoints(joints); toReview(); backToEdit(); reset()
```
(Exact shape settled in the plan; keep the live readout separate from these fields.)

## Error handling / safety

- Gate the whole "Move to" mode on `armState.hasArm`.
- Validate joint count (`=== armState.joints.length`) and numeric pose fields
  before enabling **Review**.
- **Execute** disabled while `motion.busy > 0`; the two-step confirm prevents
  accidental absolute moves.
- **STOP** always reachable and always-callable (its own mutation), interrupts the
  in-flight move.
- Surface backend errors (motion-plan failure, unreachable target, joint limits).

## Testing

- **Backend Go** (to jog-handler standard): `move_to_joints`/`move_to_pose` call
  the arm move with the correct target; no-arm + bad-count + malformed-pose errors.
- **Frontend**: the move-to target store (phase transitions, load/review/back);
  light panel wiring.
- **Hardware**: absolute pose + joints moves land at the target; **STOP interrupts
  an in-flight move**; the two-step confirm reads clearly.

## Out of scope (E2 / later)

Frame-relative jogging (jog along a taught frame's axes — separate backend
`jog_cartesian` frame param + a reference picker). Go-to-pose in a *taught* frame
(E1 is arm-base only, matching the readout). Trajectory preview in the 3D scene.
