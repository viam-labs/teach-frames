# Non-origin arm-base correctness Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Report all TCP poses in the destination (world) frame so our scene overlays are correct on cells whose arm base isn't at the world origin. Backend-only; no-op on origin-base cells.

**Architecture:** `get_arm_state` and the `jog_cartesian` / `move_to_pose` response poses read the world TCP pose from `pt.source` (`motion.GetPose(TCP, destFrame)` — the source `capture_point` already uses). `move_to_pose` converts its now-world target back to the arm-base frame (via a new `armInDest` source giving the arm base in world) and moves arm-direct exactly as E1. The frontend needs no change — `armState.pose` becomes world, so the scene-root overlays auto-correct.

**Tech Stack:** Go (RDK `arm`/`motion`/`spatialmath`/`referenceframe`), `go test`.

**Design doc:** `docs/plans/2026-07-22-non-origin-arm-base-design.md`

**Commands:** `go build ./... && go test ./models/posetracker/... && go vet ./...`; frontend sanity (in `frontend/`) `npm run check`/`test` (should be unaffected).

**Read first:**
- `models/posetracker/model.go` `teachTracker` struct + `newPoseTracker` (the `cfg.Arm` resolve block where `pt.arm`/`pt.flangeInDest` are set).
- `models/posetracker/docommand.go`: `getArmState` (uses `arm.EndPosition`), `jogCartesian` (returns `poseToMap(next)`), `moveToPose` (`arm.MoveToPosition(pose)` + returns `poseToMap(pose)`), and the `poseToMap`/`inputsToDegrees` helpers.
- `posesource/posesource.go`: `PoseSource` iface, `MotionSource`, and `Fake` (queued poses, one per `Capture`) — the test double.
- `models/posetracker/docommand_test.go`: how tests build the pose-tracker and set `pt.source = &posesource.Fake{...}` and `injArm` (`inject.Arm`).

---

## Task 1: Backend — world-frame pose reporting + `move_to_pose` conversion (Go)

**Files:**
- Modify: `models/posetracker/model.go`
- Modify: `models/posetracker/docommand.go`
- Modify: `models/posetracker/docommand_test.go`

**Step 1: Add the `armInDest` field + wiring** in `model.go`:
- Struct field on `teachTracker`: `armInDest posesource.PoseSource` (doc: "reads the ARM BASE pose in destFrame; used by move_to_pose to convert a world target back to the arm frame").
- In `newPoseTracker`, in the arm-resolved `else` block, right after `pt.arm = a`, add (NOT gated on eye_in_hand):
  ```go
  pt.armInDest = &posesource.MotionSource{Motion: mot, Component: cfg.Arm, DestFrame: dest}
  ```
  Leave the existing `flangeInDest` (eye-in-hand) exactly as is.

**Step 2: Update the tests FIRST** (they encode the frame contract):
- **`get_arm_state`**: if a test exists, update it; else add `TestGetArmStateReportsWorldPose`: set `pt.source = &posesource.Fake{Poses: []spatialmath.Pose{<known world pose>}}` and `injArm.JointPositionsFunc` → known joints; assert the response `pose` equals the fake's world pose (NOT an `EndPosition` value) and `joints` matches. (No-arm test still errors.)
- **`jog_cartesian`**: keep every existing assertion about the `MoveToPosition` *target* (the move computation is unchanged). ADD/adjust so the **response** `pose` is sourced from `pt.source`: set `pt.source = &posesource.Fake{Poses: [<distinct world pose>]}` and assert the returned `pose` equals that distinct world pose — proving it's from `GetPose`, not the move target. (Existing frame-relative jog tests: their `MoveToPosition` assertions stay; add a `pt.source` fake so the handler doesn't nil-panic and, where they check the response pose, update to the fake value.)
- **`move_to_pose`**: 
  - `TestMoveToPoseConvertsWorldTargetToArmBase`: `pt.armInDest = &posesource.Fake{Poses: [T]}` with a **non-identity** `T` (translated + rotated arm base); `pt.source = &posesource.Fake{Poses: [<world result>]}`; `injArm.MoveToPositionFunc` records `p`. Send a world target pose; assert `p ≈ spatialmath.Compose(spatialmath.PoseInverse(T), worldTarget)` (use `spatialmath.PoseAlmostEqual`), and the response `pose` equals the `pt.source` world result.
  - `TestMoveToPoseIdentityBaseNoOp`: `armInDest` returns identity `T`; assert `p ≈ worldTarget` (origin-base no-op).
  - Keep the existing no-arm / partial-pose / non-numeric guards (they must still error before any move).

Run: `go test ./models/posetracker/... -run 'ArmState|JogCartesian|MoveToPose'` → the new assertions FAIL.

**Step 3: Implement in `docommand.go`:**
- `getArmState`: keep the `pt.arm == nil` guard; replace `pose, err := pt.arm.EndPosition(...)` with
  ```go
  pif, err := pt.source.Capture(ctx)
  if err != nil { return nil, err }
  ```
  and use `poseToMap(pif.Pose())` in the response. Joints unchanged.
- `jogCartesian`: leave the whole move computation (EndPosition, basis, `next`, `MoveToPosition(next)`) UNCHANGED. Replace the response line so the pose is the post-move world pose:
  ```go
  if err := pt.arm.MoveToPosition(ctx, next, nil); err != nil { return nil, err }
  worldPIF, err := pt.source.Capture(ctx)
  if err != nil { return nil, err }
  return map[string]interface{}{"pose": poseToMap(worldPIF.Pose()), "moved": true}, nil
  ```
- `moveToPose`: after parsing the (world) target into `worldTarget spatialmath.Pose` (existing parse), convert + move + report world:
  ```go
  if pt.armInDest == nil {
      return nil, errors.New("arm dependency not configured; cannot move")
  }
  basePIF, err := pt.armInDest.Capture(ctx)
  if err != nil { return nil, err }
  armBaseTarget := spatialmath.Compose(spatialmath.PoseInverse(basePIF.Pose()), worldTarget)
  if err := pt.arm.MoveToPosition(ctx, armBaseTarget, nil); err != nil { return nil, err }
  worldPIF, err := pt.source.Capture(ctx)
  if err != nil { return nil, err }
  return map[string]interface{}{"pose": poseToMap(worldPIF.Pose()), "moved": true}, nil
  ```
  (The `pt.arm == nil` guard at the top already covers the arm; `armInDest` is set whenever the arm resolves, so the extra nil-check is defensive.)
- Update the `get_arm_state` / `jog_cartesian` / `move_to_pose` doc-comment lines to note the pose is reported in the destination (world) frame.

**Step 4: Verify**
- `go test ./models/posetracker/...` → all green (new + existing; existing move-computation assertions unchanged).
- `go build ./...`, `go vet ./...`, `gofmt -l` on the three files → clean.
- Frontend sanity (unaffected): `cd frontend && npm run check && npm run test` → green.

**Step 5: Commit**
```bash
git add models/posetracker/model.go models/posetracker/docommand.go models/posetracker/docommand_test.go
git commit -m "fix(posetracker): report TCP poses in world frame (non-origin arm-base correctness)"
```

---

## Task 2: Hardware verification (user-run)

1. **Origin-base cell (regression):** jog, capture/define a frame, go-to-pose, TcpTriad — all behave as before; the readout numbers are unchanged.
2. **Non-origin-base cell (the fix):** the live TcpTriad now coincides with the real tool and with the committed frame triads; captured markers and the live triad agree; jog and go-to-pose land at the intended physical spot; the Jog readout / Move-to fields now show world coordinates matching the committed frames.
Record results; no code commit unless a bug surfaces (TDD the fix).

---

## Definition of done
- [ ] `go build/test/vet ./...` green; frontend `check`/`test` green (unchanged).
- [ ] `get_arm_state` + `jog_cartesian`/`move_to_pose` responses report the world-frame TCP pose; `move_to_pose` converts a world target through a non-identity arm base to the correct arm-base target (fixture-tested); origin-base no-op verified.
- [ ] Frontend overlays auto-correct (no frontend change).
- [ ] Hardware: non-origin-base overlays coincide with the real tool + committed frames; origin-base unchanged. Roadmap memory updated (G done → general-cell support; only F remains).
