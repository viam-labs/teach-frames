# Go-to-pose / go-to-joints Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Move the arm to an *absolute* target (pose or joints) with a two-step confirm, plus a global STOP — via two new pose-tracker DoCommands and a "Move to" mode in the Jog panel.

**Architecture:** Backend adds `move_to_joints` / `move_to_pose` mirroring the existing jog handlers (`arm.MoveToJointPositions` / `arm.MoveToPosition`, arm-base frame) — no motion-service/frame complexity. Frontend extends `JogPanel.svelte` with a third "Move to" mode (editable target fields + Load-current prefill + Review→Execute confirm) and a prominent global STOP (ported from the unmounted `StatusBar`). The Jog panel keeps its all-local-state style — no wizard store.

**Tech Stack:** Go (RDK `arm`/`spatialmath`/`referenceframe`), Svelte 5 runes, `@viamrobotics/svelte-sdk`, Vitest, `go test`.

**Design doc:** `docs/plans/2026-07-22-go-to-pose-joints-design.md`

**Commands:** backend `go build ./... && go test ./models/posetracker/...`; frontend (in `frontend/`) `npm run check` / `npm run test` / `npm run build`.

**Backend precedent (read these) — `models/posetracker/docommand.go`:**
- `jogJoint` (~L494): reads `arm.JointPositions`, sets `next[joint] = cur[joint] + step*π/180` (NOTE `referenceframe.Input` is a float64 alias here), `arm.MoveToJointPositions(next)`.
- `jogCartesian` (~L530): reads `arm.EndPosition`, computes `next` pose, `arm.MoveToPosition(next)`.
- `stop_arm` case (~L124): `pt.arm.Stop`.
- The DoCommand dispatch `switch` + the doc-comment block at the top of the file.
**Test precedent — `models/posetracker/docommand_test.go`:** `TestJogJoint`, `TestJogCartesianTranslation`, `TestStopArm` — use the `inject.Arm` fake with `MoveToJointPositionsFunc`/`MoveToPositionFunc`/`JointPositionsFunc`/`EndPositionFunc` recording the call, and the existing test helper that builds the pose-tracker with an injected arm.

---

## Task 1: Backend — `move_to_joints` + `move_to_pose` (TDD, Go)

**Files:**
- Modify: `models/posetracker/docommand.go`
- Modify: `models/posetracker/docommand_test.go`

**Step 1: Write failing tests** in `docommand_test.go`, following the `TestJogJoint` / `TestJogCartesianTranslation` idiom (inject arm, record the move call, assert the target). Cover:
- `TestMoveToJoints`: `JointPositionsFunc` returns e.g. `[]referenceframe.Input{0,0,0}`; `MoveToJointPositionsFunc` records `p`; send `{"move_to_joints":{"positions":[90.0, 0.0, 45.0]}}`; assert the recorded input equals degrees→radians of the payload; response `moved:true`.
- `TestMoveToJointsWrongCount`: current has 3 joints, send 2 positions → error, and `MoveToJointPositions` NOT called.
- `TestMoveToJointsNoArm`: no arm → clear error.
- `TestMoveToPose`: `MoveToPositionFunc` records the pose; send `{"move_to_pose":{"pose":{"x":100,"y":0,"z":200,"o_x":0,"o_y":0,"o_z":1,"theta":0}}}`; assert the recorded pose's `Point()` ≈ (100,0,200) and orientation matches the OrientationVectorDegrees; response `moved:true`.
- `TestMoveToPoseNoArm`: no arm → clear error.
- `TestMoveToPoseBadPose`: missing/non-object `pose` → error.

Run: `go test ./models/posetracker/... -run 'MoveTo'` → FAIL (unknown command / undefined).

**Step 2: Implement the handlers** in `docommand.go`. Add two dispatch cases in the DoCommand `switch` (next to `jog_joint`/`jog_cartesian`):
```go
	case has(cmd, "move_to_joints"):
		return pt.moveToJoints(ctx, cmd["move_to_joints"])
	case has(cmd, "move_to_pose"):
		return pt.moveToPose(ctx, cmd["move_to_pose"])
```
Handlers (mirror `jogJoint`/`jogCartesian` exactly for the arm-nil guard, arg parsing, `referenceframe.Input` alias usage, and `poseToMap`/`inputsToDegrees` for the response):
```go
// moveToJoints moves the arm to an absolute joint configuration (degrees).
func (pt *teachTracker) moveToJoints(ctx context.Context, raw interface{}) (map[string]interface{}, error) {
	if pt.arm == nil {
		return nil, errors.New("arm dependency not configured; cannot move")
	}
	args, ok := raw.(map[string]interface{})
	if !ok {
		return nil, errors.New("move_to_joints args must be an object")
	}
	rawPositions, ok := args["positions"].([]interface{})
	if !ok {
		return nil, errors.New("move_to_joints requires a 'positions' array (degrees)")
	}
	cur, err := pt.arm.JointPositions(ctx, nil)
	if err != nil {
		return nil, err
	}
	if len(rawPositions) != len(cur) {
		return nil, fmt.Errorf("move_to_joints expects %d joint positions, got %d", len(cur), len(rawPositions))
	}
	next := make([]referenceframe.Input, len(rawPositions))
	for i, p := range rawPositions {
		deg, ok := p.(float64)
		if !ok {
			return nil, fmt.Errorf("move_to_joints position %d is not numeric", i)
		}
		next[i] = referenceframe.Input(deg * math.Pi / 180.0) // Input is a float64 alias (radians)
	}
	if err := pt.arm.MoveToJointPositions(ctx, next, nil); err != nil {
		return nil, err
	}
	return map[string]interface{}{"joints": inputsToDegrees(next), "moved": true}, nil
}

// moveToPose moves the arm's TCP to an absolute pose (mm + OrientationVector
// degrees) in the arm base frame — the same frame get_arm_state/EndPosition
// reports, so the operator edits the values they see.
func (pt *teachTracker) moveToPose(ctx context.Context, raw interface{}) (map[string]interface{}, error) {
	if pt.arm == nil {
		return nil, errors.New("arm dependency not configured; cannot move")
	}
	args, ok := raw.(map[string]interface{})
	if !ok {
		return nil, errors.New("move_to_pose args must be an object")
	}
	poseArg, ok := args["pose"].(map[string]interface{})
	if !ok {
		return nil, errors.New("move_to_pose requires a 'pose' object")
	}
	num := func(k string) (float64, error) {
		v, ok := poseArg[k].(float64)
		if !ok {
			return 0, fmt.Errorf("move_to_pose pose.%s is not numeric", k)
		}
		return v, nil
	}
	x, err := num("x"); if err != nil { return nil, err }
	y, err := num("y"); if err != nil { return nil, err }
	z, err := num("z"); if err != nil { return nil, err }
	ox, err := num("o_x"); if err != nil { return nil, err }
	oy, err := num("o_y"); if err != nil { return nil, err }
	oz, err := num("o_z"); if err != nil { return nil, err }
	th, err := num("theta"); if err != nil { return nil, err }
	pose := spatialmath.NewPose(
		r3.Vector{X: x, Y: y, Z: z},
		&spatialmath.OrientationVectorDegrees{OX: ox, OY: oy, OZ: oz, Theta: th},
	)
	if err := pt.arm.MoveToPosition(ctx, pose, nil); err != nil {
		return nil, err
	}
	return map[string]interface{}{"pose": poseToMap(pose), "moved": true}, nil
}
```
(If a `mapToPose`-style helper already exists in the package, reuse it instead of the inline `num` parsing — check first. Match the existing gofmt style; the compressed `if err != nil` lines above may need expanding to satisfy the linter — follow the repo's convention.)

Add two lines to the top-of-file doc-comment block, in the style of the others:
```
//	{"move_to_joints": {"positions": [deg,…]}}               — move the arm to an absolute joint configuration (degrees); errors if no arm or the count mismatches the arm's joints.
//	{"move_to_pose": {"pose": {"x":0,"y":0,"z":0,"o_x":0,"o_y":0,"o_z":1,"theta":0}}} — move the TCP to an absolute pose (mm + OV degrees) in the arm base frame; errors if no arm.
```

**Step 3: Verify** — `go build ./... && go test ./models/posetracker/...` (all green, incl. the new tests). `go vet ./...`.

**Step 4: Commit**
```bash
git add models/posetracker/docommand.go models/posetracker/docommand_test.go
git commit -m "feat(posetracker): move_to_joints + move_to_pose (absolute go-to) DoCommands"
```

---

## Task 2: Frontend builders + types (TDD)

**Files:**
- Modify: `frontend/src/lib/poseTracker.ts`
- Modify: `frontend/src/lib/poseTracker.test.ts`

**Step 1: Write failing builder tests** in `poseTracker.test.ts` (match the existing builder-test style there):
```ts
it('moveToJoints builds the command', () => {
  expect(moveToJoints([90, 0, 45])).toEqual({ move_to_joints: { positions: [90, 0, 45] } })
})
it('moveToPose builds the command', () => {
  const p = { x: 1, y: 2, z: 3, o_x: 0, o_y: 0, o_z: 1, theta: 0 }
  expect(moveToPose(p)).toEqual({ move_to_pose: { pose: p } })
})
```
Run: `cd frontend && npx vitest run src/lib/poseTracker.test.ts` → FAIL.

**Step 2: Add builders + types** to `poseTracker.ts` near `jogCartesian`/`jogJoint`:
```ts
export const moveToJoints = (positions: number[]) => ({ move_to_joints: { positions } })
export const moveToPose = (pose: PoseMap) => ({ move_to_pose: { pose } })

export interface MoveToJointsResponse { joints: number[]; moved: boolean }
export interface MoveToPoseResponse { pose: PoseMap; moved: boolean }
```
(`PoseMap` already has `x,y,z,o_x,o_y,o_z,theta`.)

**Step 3: Verify** — `npx vitest run src/lib/poseTracker.test.ts` (PASS), `npm run check` (0 errors), `npm run test` (all green).

**Step 4: Commit**
```bash
git add frontend/src/lib/poseTracker.ts frontend/src/lib/poseTracker.test.ts
git commit -m "feat(frontend): move_to_joints/move_to_pose builders + types"
```

---

## Task 3: Jog panel "Move to" mode + global STOP

**Files:**
- Modify: `frontend/src/panels/JogPanel.svelte`

Read `frontend/src/panels/StatusBar.svelte` for the exact STOP logic (`requestStop()` + a dedicated `stop` mutation firing `stopArm()`, never gated by `motion.busy`, using `.mutate` not `.mutateAsync`).

**Step 1: Add the global STOP.** In `JogPanel.svelte`:
- Import `requestStop` from `../lib/motion.svelte` and `stopArm` from `../lib/poseTracker`.
- Add a dedicated `const stop = createResourceMutation(pt, 'doCommand')` (separate from `jog`).
- `function handleStop() { requestStop(); stop.mutate(toCommandArgs(stopArm())) }`.
- Render a prominent **STOP** button — always visible in the panel, `disabled={stop.isPending}` (NOT gated on `motion.busy` or `armState.hasArm` — it must stay callable to interrupt a move; if no arm the backend errors harmlessly). Use a danger recipe (`bg-danger-dark`/`text-white`), `min-h-11`, `aria-label="Stop arm"`. Surface `stop.error`.

**Step 2: Add the "Move to" mode.** Extend the mode toggle from `'cartesian' | 'joint'` to also `'moveto'` (add a third toggle button "Move to"). Add local state:
```ts
type MoveKind = 'pose' | 'joints'
let moveKind = $state<MoveKind>('pose')
let moveConfirm = $state(false) // false = editing, true = review/confirm
let targetPose = $state<PoseMap>({ x: 0, y: 0, z: 0, o_x: 0, o_y: 0, o_z: 1, theta: 0 })
let targetJoints = $state<number[]>([])
const moveM = createResourceMutation(pt, 'doCommand')
```
- **Load current**: `targetPose = { ...armState.pose }` (guard non-undefined) / `targetJoints = [...armState.joints]`; sets `moveConfirm = false`.
- Editable fields: number inputs bound to `targetPose.x/y/z/o_x/o_y/o_z/theta` (pose kind) or each `targetJoints[i]` (joints kind). Use the input recipe + labels from the pose/joints readout tables. Reasonable numeric `step`.
- **Review target** (`disabled` unless the fields are valid: pose = all numeric; joints = `targetJoints.length === armState.joints.length`) → `moveConfirm = true`.
- When `moveConfirm`: a confirm bar (role="alert", like ManageFramesPanel's) showing the target values + "The arm will move to this absolute target." with **Execute** (danger/primary) and **Cancel** (focused, `moveConfirm = false`).
- **Execute** → `await withMove(async () => { await moveM.mutateAsync(toCommandArgs(moveKind === 'pose' ? moveToPose(targetPose) : moveToJoints(targetJoints))) })`; drive the readout from the response if present; on success clear `moveConfirm`. Disable Execute while `motion.busy > 0` or `moveM.isPending`. Surface `moveM.error`. Guard whole mode on `armState.hasArm`.
- Import `withMove` (already imported), `moveToPose`, `moveToJoints`, `MoveToPoseResponse`, `MoveToJointsResponse`.

Keep the existing Cartesian/Joint jog + readout unchanged. The "Move to" fields are a *frozen editable target*, separate from the live readout (Load current snapshots into them; live polling never overwrites them).

**Step 3: Verify** — `npm run check` (0 errors), `npm run test` (all green), `npm run build` (success). Manual smoke optional (`npm run dev`).

**Step 4: Commit**
```bash
git add frontend/src/panels/JogPanel.svelte
git commit -m "feat(frontend): Move-to (absolute go-to-pose/joints) mode + global STOP in Jog panel"
```

---

## Task 4: Hardware verification (user-run)

1. **Go-to-joints:** Load current, edit a joint, Review → Execute → arm moves to the absolute joint config; readout matches.
2. **Go-to-pose:** Load current, edit x/z, Review → Execute → TCP moves to the absolute pose.
3. **STOP:** during an Execute move (or a press-and-hold jog), press STOP → the arm halts immediately.
4. **Safety:** the two-step confirm reads clearly; a bad joint count / non-numeric field blocks Review; Execute is disabled while busy.
Record results; no code commit unless a bug surfaces (TDD the fix).

---

## Definition of done
- [ ] `go build/test ./...` and `npm run check`/`test`/`build` green.
- [ ] `move_to_joints`/`move_to_pose` DoCommands (arm move to the correct absolute target; error paths) — Go-tested.
- [ ] Jog panel "Move to" mode: editable pose/joints target, Load current, Review→Execute confirm.
- [ ] Global STOP button (always-callable) interrupts an in-flight move.
- [ ] Hardware: absolute moves land; STOP halts; roadmap memory updated (E1 done, E2 frame-relative jog next).
