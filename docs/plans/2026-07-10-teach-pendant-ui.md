# Teach Pendant UI Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Ship a browser-based teach pendant (jog + full teaching workflow) as a Viam Application bundled in the `teach-frames` module.

**Architecture:** Jog logic lives in Go as four new pose-tracker DoCommands (`jog_cartesian`, `jog_joint`, `stop_arm`, `get_arm_state`) that reuse the existing optional `arm` dependency. A Svelte 5 + Vite SPA built on `@viamrobotics/svelte-sdk` calls those DoCommands (plus the existing capture/frame/TCP verbs) via TanStack query/mutation hooks, and is bundled into the module tarball as a `single_machine` Viam Application.

**Tech Stack:** Go (RDK, `spatialmath`), Svelte 5, Vite, TypeScript, `@viamrobotics/svelte-sdk@^1.2.3`, `@viamrobotics/sdk`, `@tanstack/svelte-query`, Vitest.

**Design doc:** `docs/plans/2026-07-10-teach-pendant-ui-design.md`

**Conventions established during design:**
- Jog target arm = the existing optional `arm` config attribute (same one TCP teaching uses).
- `jog_cartesian`: world-frame translation (X/Y/Z along base axes) + tool-frame rotation (roll/pitch/yaw about the TCP).
- Orientation math uses quaternion composition via `spatialmath.Compose` — never Euler addition, never manual `OX/OY/OZ/Theta`.
- `step` in the payload carries its own sign; UI arrows send `+step` / `−step`.
- Joint values are radians internally (`referenceframe.Input.Value`); jog steps and readouts are **degrees** at the DoCommand boundary.
- Discrete steps only in v1 (no speed/velocity param).

---

## Phase A — Go jog DoCommands (TDD)

All work is in `models/posetracker/`. Tests use RDK's `inject.NewArm` (already used in `model_test.go`) — no new fake needed. Reference existing patterns: dispatch in `docommand.go:34-107`, `poseToMap` at `docommand.go:387`, `has`/`keysOf` helpers at `docommand.go:372-384`.

### Task 1: Store the raw arm on the model

The constructor currently wraps the arm only as `pt.flange` (a `posesource.FlangeSource`). Jog needs the full `arm.Arm` (for `MoveToPosition`, `JointPositions`, `MoveToJointPositions`, `Stop`). Add an `arm arm.Arm` field and populate it alongside `flange`.

**Files:**
- Modify: `models/posetracker/model.go:66-86` (struct), `models/posetracker/model.go:121-129` (constructor)
- Test: `models/posetracker/model_test.go`

**Step 1: Write the failing test**

Add to `model_test.go` (mirrors `TestNewPoseTrackerWithArmBuildsFlange`):

```go
// TestNewPoseTrackerWithArmStoresArm verifies the raw arm is retained for jogging.
func TestNewPoseTrackerWithArmStoresArm(t *testing.T) {
	motionName := motion.Named(defaultMotionService)
	injArm := inject.NewArm("my-arm")
	deps := resource.Dependencies{
		motionName:          injectmotion.NewMotionService(defaultMotionService),
		arm.Named("my-arm"): injArm,
	}
	conf := resource.Config{
		Name:                "test-tracker",
		API:                 posetracker.API,
		Model:               Model,
		ConvertedAttributes: &Config{TCPComponent: "tool", Arm: "my-arm"},
	}
	res, err := newPoseTracker(context.Background(), deps, conf, logging.NewTestLogger(t))
	test.That(t, err, test.ShouldBeNil)
	tt := res.(*teachTracker)
	test.That(t, tt.arm, test.ShouldEqual, injArm)
}
```

**Step 2: Run to verify it fails**

Run: `go test ./models/posetracker/ -run TestNewPoseTrackerWithArmStoresArm`
Expected: FAIL — `tt.arm` undefined (compile error).

**Step 3: Implement**

In `model.go`, add to the `teachTracker` struct (after `flange` field, ~line 77):

```go
	arm          arm.Arm
```

In the constructor, change the arm wiring block (currently ~lines 121-129) to also store the raw arm:

```go
	if cfg.Arm != "" {
		a, aerr := arm.FromDependencies(deps, cfg.Arm)
		if aerr != nil {
			// Declared dependency should resolve; warn and leave TCP teaching disabled.
			logger.Warnw("TCP teaching disabled: arm dependency not resolvable", "arm", cfg.Arm, "err", aerr)
		} else {
			pt.flange = &posesource.ArmSource{Arm: a}
			pt.arm = a
		}
	}
```

**Step 4: Run to verify it passes**

Run: `go test ./models/posetracker/ -run TestNewPoseTrackerWithArmStoresArm`
Expected: PASS.

**Step 5: Commit**

```bash
git add models/posetracker/model.go models/posetracker/model_test.go
git commit -m "feat(pose-tracker): retain raw arm for jogging"
```

---

### Task 2: `get_arm_state` DoCommand

Returns current TCP pose + joint positions (degrees) in one call for the UI poll loop.

**Files:**
- Modify: `models/posetracker/docommand.go` (add case + helper)
- Test: `models/posetracker/docommand_test.go`

**Step 1: Write the failing test**

```go
func TestGetArmState(t *testing.T) {
	pt := newTestTrackerNoArm(t) // helper below returns tracker with pt.arm == nil
	_, err := pt.DoCommand(context.Background(), map[string]interface{}{"get_arm_state": map[string]interface{}{}})
	test.That(t, err, test.ShouldNotBeNil) // arm not configured

	injArm := inject.NewArm("my-arm")
	injArm.EndPositionFunc = func(context.Context, map[string]interface{}) (spatialmath.Pose, error) {
		return spatialmath.NewPoseFromPoint(r3.Vector{X: 1, Y: 2, Z: 3}), nil
	}
	injArm.JointPositionsFunc = func(context.Context, map[string]interface{}) ([]referenceframe.Input, error) {
		return []referenceframe.Input{{Value: 0}, {Value: math.Pi / 2}}, nil
	}
	pt.arm = injArm

	resp, err := pt.DoCommand(context.Background(), map[string]interface{}{"get_arm_state": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
	pose := resp["pose"].(map[string]interface{})
	test.That(t, pose["x"], test.ShouldAlmostEqual, 1.0)
	joints := resp["joints"].([]float64)
	test.That(t, joints[0], test.ShouldAlmostEqual, 0.0)
	test.That(t, joints[1], test.ShouldAlmostEqual, 90.0) // radians→degrees
}
```

If a `newTestTrackerNoArm` helper does not already exist in the test file, add a minimal one that constructs a `&teachTracker{store: store.New()}` (see how existing tests build trackers near the top of `docommand_test.go` and reuse that constructor/helper). Add imports as needed: `math`, `github.com/golang/geo/r3`, `go.viam.com/rdk/referenceframe`, `go.viam.com/rdk/spatialmath`, `go.viam.com/rdk/components/arm`, `go.viam.com/rdk/testutils/inject`.

**Step 2: Run to verify it fails**

Run: `go test ./models/posetracker/ -run TestGetArmState`
Expected: FAIL — `unknown command: [get_arm_state]`.

**Step 3: Implement**

Add a dispatch case in `docommand.go` (in the `switch` before the final `return`):

```go
	case has(cmd, "get_arm_state"):
		return pt.getArmState(ctx)
```

Add the handler + a joints-to-degrees helper at the end of the file:

```go
// getArmState returns the current TCP pose and joint positions (degrees) for the UI.
func (pt *teachTracker) getArmState(ctx context.Context) (map[string]interface{}, error) {
	if pt.arm == nil {
		return nil, errors.New("arm dependency not configured; cannot read arm state")
	}
	pose, err := pt.arm.EndPosition(ctx, nil)
	if err != nil {
		return nil, err
	}
	joints, err := pt.arm.JointPositions(ctx, nil)
	if err != nil {
		return nil, err
	}
	return map[string]interface{}{
		"pose":   poseToMap(pose),
		"joints": inputsToDegrees(joints),
	}, nil
}

// inputsToDegrees converts joint inputs (radians) to a plain []float64 in degrees.
func inputsToDegrees(inputs []referenceframe.Input) []float64 {
	out := make([]float64, len(inputs))
	for i, in := range inputs {
		out[i] = in.Value * 180.0 / math.Pi
	}
	return out
}
```

Add imports to `docommand.go`: `math`, `go.viam.com/rdk/referenceframe`.

**Step 4: Run to verify it passes**

Run: `go test ./models/posetracker/ -run TestGetArmState`
Expected: PASS.

**Step 5: Commit**

```bash
git add models/posetracker/docommand.go models/posetracker/docommand_test.go
git commit -m "feat(pose-tracker): add get_arm_state DoCommand"
```

---

### Task 3: `stop_arm` DoCommand

**Files:**
- Modify: `models/posetracker/docommand.go`
- Test: `models/posetracker/docommand_test.go`

**Step 1: Write the failing test**

```go
func TestStopArm(t *testing.T) {
	injArm := inject.NewArm("my-arm")
	stopped := false
	injArm.StopFunc = func(context.Context, map[string]interface{}) error { stopped = true; return nil }
	pt := newTestTrackerNoArm(t)
	pt.arm = injArm

	resp, err := pt.DoCommand(context.Background(), map[string]interface{}{"stop_arm": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, resp["stopped"], test.ShouldBeTrue)
	test.That(t, stopped, test.ShouldBeTrue)
}
```

**Step 2: Run — FAIL** (`unknown command: [stop_arm]`).
Run: `go test ./models/posetracker/ -run TestStopArm`

**Step 3: Implement**

Dispatch case:

```go
	case has(cmd, "stop_arm"):
		if pt.arm == nil {
			return nil, errors.New("arm dependency not configured; cannot stop arm")
		}
		if err := pt.arm.Stop(ctx, nil); err != nil {
			return nil, err
		}
		return map[string]interface{}{"stopped": true}, nil
```

**Step 4: Run — PASS.**

**Step 5: Commit**

```bash
git add models/posetracker/docommand.go models/posetracker/docommand_test.go
git commit -m "feat(pose-tracker): add stop_arm DoCommand"
```

---

### Task 4: `jog_joint` DoCommand

**Files:**
- Modify: `models/posetracker/docommand.go`
- Test: `models/posetracker/docommand_test.go`

**Step 1: Write the failing test**

```go
func TestJogJoint(t *testing.T) {
	injArm := inject.NewArm("my-arm")
	start := []referenceframe.Input{{Value: 0}, {Value: 0}, {Value: 0}}
	var moved []referenceframe.Input
	injArm.JointPositionsFunc = func(context.Context, map[string]interface{}) ([]referenceframe.Input, error) {
		return start, nil
	}
	injArm.MoveToJointPositionsFunc = func(_ context.Context, p []referenceframe.Input, _ map[string]interface{}) error {
		moved = p
		return nil
	}
	pt := newTestTrackerNoArm(t)
	pt.arm = injArm

	// jog joint 1 by +90 degrees
	resp, err := pt.DoCommand(context.Background(), map[string]interface{}{
		"jog_joint": map[string]interface{}{"joint": 1.0, "step": 90.0},
	})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, moved[1].Value, test.ShouldAlmostEqual, math.Pi/2) // +90deg in radians
	test.That(t, moved[0].Value, test.ShouldAlmostEqual, 0.0)
	joints := resp["joints"].([]float64)
	test.That(t, joints[1], test.ShouldAlmostEqual, 90.0)

	// out-of-range joint index errors
	_, err = pt.DoCommand(context.Background(), map[string]interface{}{
		"jog_joint": map[string]interface{}{"joint": 9.0, "step": 5.0},
	})
	test.That(t, err, test.ShouldNotBeNil)
}
```

**Step 2: Run — FAIL** (`unknown command: [jog_joint]`).
Run: `go test ./models/posetracker/ -run TestJogJoint`

**Step 3: Implement**

Dispatch case:

```go
	case has(cmd, "jog_joint"):
		return pt.jogJoint(ctx, cmd["jog_joint"])
```

Handler:

```go
// jogJoint nudges a single joint by a signed step (degrees) and moves the arm there.
func (pt *teachTracker) jogJoint(ctx context.Context, raw interface{}) (map[string]interface{}, error) {
	if pt.arm == nil {
		return nil, errors.New("arm dependency not configured; cannot jog")
	}
	args, ok := raw.(map[string]interface{})
	if !ok {
		return nil, errors.New("jog_joint args must be an object")
	}
	jointF, ok := args["joint"].(float64)
	if !ok {
		return nil, errors.New("jog_joint requires a numeric 'joint' index")
	}
	step, ok := args["step"].(float64)
	if !ok {
		return nil, errors.New("jog_joint requires a numeric 'step' (degrees)")
	}
	joint := int(jointF)

	cur, err := pt.arm.JointPositions(ctx, nil)
	if err != nil {
		return nil, err
	}
	if joint < 0 || joint >= len(cur) {
		return nil, fmt.Errorf("joint index %d out of range [0,%d)", joint, len(cur))
	}
	next := make([]referenceframe.Input, len(cur))
	copy(next, cur)
	next[joint] = referenceframe.Input{Value: cur[joint].Value + step*math.Pi/180.0}

	if err := pt.arm.MoveToJointPositions(ctx, next, nil); err != nil {
		return nil, err
	}
	return map[string]interface{}{"joints": inputsToDegrees(next)}, nil
}
```

**Step 4: Run — PASS.**

**Step 5: Commit**

```bash
git add models/posetracker/docommand.go models/posetracker/docommand_test.go
git commit -m "feat(pose-tracker): add jog_joint DoCommand"
```

---

### Task 5: `jog_cartesian` — translation (world frame)

Build the command with translation first, rotation in Task 6.

**Files:**
- Modify: `models/posetracker/docommand.go`
- Test: `models/posetracker/docommand_test.go`

**Step 1: Write the failing test**

```go
func TestJogCartesianTranslation(t *testing.T) {
	injArm := inject.NewArm("my-arm")
	// current pose: at (10,20,30) with a non-identity orientation so we can prove
	// translation is world-frame (orientation must not rotate the delta).
	startOri := &spatialmath.EulerAngles{Yaw: math.Pi / 2} // 90deg about Z
	injArm.EndPositionFunc = func(context.Context, map[string]interface{}) (spatialmath.Pose, error) {
		return spatialmath.NewPose(r3.Vector{X: 10, Y: 20, Z: 30}, startOri), nil
	}
	var moved spatialmath.Pose
	injArm.MoveToPositionFunc = func(_ context.Context, p spatialmath.Pose, _ map[string]interface{}) error {
		moved = p
		return nil
	}
	pt := newTestTrackerNoArm(t)
	pt.arm = injArm

	// jog +5mm in world X
	_, err := pt.DoCommand(context.Background(), map[string]interface{}{
		"jog_cartesian": map[string]interface{}{"axis": "x", "step": 5.0},
	})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, moved.Point().X, test.ShouldAlmostEqual, 15.0) // 10+5 world-frame
	test.That(t, moved.Point().Y, test.ShouldAlmostEqual, 20.0)
	test.That(t, moved.Point().Z, test.ShouldAlmostEqual, 30.0)
	// orientation unchanged by a translation jog
	test.That(t, spatialmath.OrientationAlmostEqual(moved.Orientation(), startOri), test.ShouldBeTrue)
}
```

**Step 2: Run — FAIL** (`unknown command: [jog_cartesian]`).
Run: `go test ./models/posetracker/ -run TestJogCartesianTranslation`

**Step 3: Implement**

Dispatch case:

```go
	case has(cmd, "jog_cartesian"):
		return pt.jogCartesian(ctx, cmd["jog_cartesian"])
```

Handler (translation branch only for now; rotation returns an error until Task 6):

```go
// jogCartesian nudges the TCP by a signed step. Translation axes (x/y/z, mm) are
// world-frame; rotation axes (roll/pitch/yaw, degrees) are tool-frame.
func (pt *teachTracker) jogCartesian(ctx context.Context, raw interface{}) (map[string]interface{}, error) {
	if pt.arm == nil {
		return nil, errors.New("arm dependency not configured; cannot jog")
	}
	args, ok := raw.(map[string]interface{})
	if !ok {
		return nil, errors.New("jog_cartesian args must be an object")
	}
	axis, ok := args["axis"].(string)
	if !ok {
		return nil, errors.New("jog_cartesian requires a string 'axis'")
	}
	step, ok := args["step"].(float64)
	if !ok {
		return nil, errors.New("jog_cartesian requires a numeric 'step'")
	}

	cur, err := pt.arm.EndPosition(ctx, nil)
	if err != nil {
		return nil, err
	}

	var next spatialmath.Pose
	switch axis {
	case "x":
		next = spatialmath.NewPose(cur.Point().Add(r3.Vector{X: step}), cur.Orientation())
	case "y":
		next = spatialmath.NewPose(cur.Point().Add(r3.Vector{Y: step}), cur.Orientation())
	case "z":
		next = spatialmath.NewPose(cur.Point().Add(r3.Vector{Z: step}), cur.Orientation())
	case "roll", "pitch", "yaw":
		return nil, errors.New("rotation jog not yet implemented")
	default:
		return nil, fmt.Errorf("unknown jog axis %q (want x|y|z|roll|pitch|yaw)", axis)
	}

	if err := pt.arm.MoveToPosition(ctx, next, nil); err != nil {
		return nil, err
	}
	return map[string]interface{}{"pose": poseToMap(next), "moved": true}, nil
}
```

Add import `github.com/golang/geo/r3` to `docommand.go`.

**Step 4: Run — PASS.**

**Step 5: Commit**

```bash
git add models/posetracker/docommand.go models/posetracker/docommand_test.go
git commit -m "feat(pose-tracker): add jog_cartesian translation"
```

---

### Task 6: `jog_cartesian` — rotation (tool frame)

Replace the rotation stub with tool-frame quaternion composition.

**Files:**
- Modify: `models/posetracker/docommand.go` (rotation branch)
- Test: `models/posetracker/docommand_test.go`

**Step 1: Write the failing test**

Verify a yaw jog rotates about the **tool's** Z (right-multiply), leaves position fixed, and composes correctly from a non-identity start.

```go
func TestJogCartesianRotationToolFrame(t *testing.T) {
	injArm := inject.NewArm("my-arm")
	startOri := &spatialmath.EulerAngles{Roll: math.Pi / 2} // tool tilted about X
	injArm.EndPositionFunc = func(context.Context, map[string]interface{}) (spatialmath.Pose, error) {
		return spatialmath.NewPose(r3.Vector{X: 1, Y: 2, Z: 3}, startOri), nil
	}
	var moved spatialmath.Pose
	injArm.MoveToPositionFunc = func(_ context.Context, p spatialmath.Pose, _ map[string]interface{}) error {
		moved = p
		return nil
	}
	pt := newTestTrackerNoArm(t)
	pt.arm = injArm

	_, err := pt.DoCommand(context.Background(), map[string]interface{}{
		"jog_cartesian": map[string]interface{}{"axis": "yaw", "step": 30.0},
	})
	test.That(t, err, test.ShouldBeNil)

	// position unchanged by a pure rotation jog
	test.That(t, moved.Point().X, test.ShouldAlmostEqual, 1.0)
	test.That(t, moved.Point().Y, test.ShouldAlmostEqual, 2.0)
	test.That(t, moved.Point().Z, test.ShouldAlmostEqual, 3.0)

	// expected orientation = startOri (right-multiplied) tool-frame yaw of 30deg:
	// Compose(start, deltaAboutZ)
	delta := spatialmath.NewPoseFromOrientation(&spatialmath.EulerAngles{Yaw: 30 * math.Pi / 180})
	expected := spatialmath.Compose(spatialmath.NewPose(r3.Vector{X: 1, Y: 2, Z: 3}, startOri), delta)
	test.That(t, spatialmath.OrientationAlmostEqual(moved.Orientation(), expected.Orientation()), test.ShouldBeTrue)
}
```

**Step 2: Run — FAIL** (`rotation jog not yet implemented`).
Run: `go test ./models/posetracker/ -run TestJogCartesianRotationToolFrame`

**Step 3: Implement**

Replace the `case "roll", "pitch", "yaw":` branch:

```go
	case "roll", "pitch", "yaw":
		rad := step * math.Pi / 180.0
		var delta *spatialmath.EulerAngles
		switch axis {
		case "roll":
			delta = &spatialmath.EulerAngles{Roll: rad}
		case "pitch":
			delta = &spatialmath.EulerAngles{Pitch: rad}
		case "yaw":
			delta = &spatialmath.EulerAngles{Yaw: rad}
		}
		// Tool-frame rotation: right-multiply the current pose by the delta
		// (delta has zero translation, so the point is preserved).
		next = spatialmath.Compose(cur, spatialmath.NewPoseFromOrientation(delta))
```

Note: `next` is declared with `var next spatialmath.Pose` above; this branch assigns it, and the shared `MoveToPosition(ctx, next, nil)` at the bottom handles the move. Ensure the `switch` no longer early-returns for rotation.

**Step 4: Run — PASS.** Also run the full package:

Run: `go test ./models/posetracker/`
Expected: PASS (all jog + existing tests).

**Step 5: Commit**

```bash
git add models/posetracker/docommand.go models/posetracker/docommand_test.go
git commit -m "feat(pose-tracker): add jog_cartesian tool-frame rotation"
```

---

### Task 7: Document the new DoCommands

**Files:**
- Modify: `models/posetracker/docommand.go:14-28` (doc comment)
- Modify: `README.md` (DoCommand reference table)

**Step 1:** Add the four verbs to the `DoCommand` godoc block, matching the existing bullet style:

```
//	{"get_arm_state": {}}                                    — return the current TCP pose and joint positions (degrees).
//	{"jog_cartesian": {"axis": "x|y|z|roll|pitch|yaw", "step": <mm-or-deg>}} — nudge the TCP: x/y/z translate in world frame (mm), roll/pitch/yaw rotate about the TCP (degrees).
//	{"jog_joint": {"joint": <index>, "step": <deg>}}         — nudge a single joint by a signed step (degrees).
//	{"stop_arm": {}}                                         — stop arm motion immediately.
```

**Step 2:** Add rows to the README DoCommand table (under "Teaching workflow: DoCommand reference"), and add a short "Jogging" subsection noting: requires the `arm` config attribute; world-frame translation + tool-frame rotation; discrete steps only.

**Step 3: Commit**

```bash
git add models/posetracker/docommand.go README.md
git commit -m "docs(pose-tracker): document jog and arm-state DoCommands"
```

---

## Phase B — Frontend Viam Application

Directory: `frontend/`. The app is a Svelte 5 + Vite SPA using `@viamrobotics/svelte-sdk`. Panels call DoCommands via `createResourceMutation` and poll via `createResourceQuery` + `usePolling`.

### Task 8: Scaffold the Vite + Svelte + TS app

**Files (create):** `frontend/package.json`, `frontend/vite.config.ts`, `frontend/tsconfig.json`, `frontend/svelte.config.js`, `frontend/index.html`, `frontend/src/main.ts`, `frontend/src/App.svelte`, `frontend/.gitignore`

**Step 1:** `frontend/package.json`:

```json
{
  "name": "teach-pendant",
  "private": true,
  "type": "module",
  "scripts": {
    "dev": "vite",
    "build": "vite build",
    "preview": "vite preview",
    "check": "svelte-check --tsconfig ./tsconfig.json",
    "test": "vitest run"
  },
  "dependencies": {
    "@viamrobotics/sdk": "^0.51.0",
    "@viamrobotics/svelte-sdk": "^1.2.3",
    "@tanstack/svelte-query": "^6.0.8",
    "js-cookie": "^3.0.5"
  },
  "devDependencies": {
    "@sveltejs/vite-plugin-svelte": "^5.0.0",
    "@testing-library/svelte": "^5.2.0",
    "@types/js-cookie": "^3.0.6",
    "svelte": "^5.0.0",
    "svelte-check": "^4.0.0",
    "typescript": "^5.6.0",
    "vite": "^6.0.0",
    "vitest": "^2.1.0",
    "jsdom": "^25.0.0"
  }
}
```

> Pin exact versions during install; confirm `@viamrobotics/sdk` peer range against `@viamrobotics/svelte-sdk@1.2.3` (peer is `>=0.51`).

**Step 2:** `frontend/vite.config.ts` — build to `dist/`, single entry, relative base (Viam serves the app under a path prefix, so `base: './'` keeps asset URLs relative):

```ts
import { defineConfig } from 'vite'
import { svelte } from '@sveltejs/vite-plugin-svelte'

export default defineConfig({
  base: './',
  plugins: [svelte()],
  build: { outDir: 'dist', emptyOutDir: true },
  test: { environment: 'jsdom', globals: true },
})
```

**Step 3:** `frontend/index.html`, `frontend/svelte.config.js` (vitePreprocess), `frontend/tsconfig.json` (extend `@tsconfig/svelte` or a standard Svelte 5 config), `frontend/src/main.ts` mounting `App.svelte`, and a placeholder `App.svelte` that renders `<h1>Teach Pendant</h1>`. `frontend/.gitignore` ignores `node_modules/` and `dist/`.

**Step 4: Verify build**

Run: `cd frontend && npm install && npm run build`
Expected: `frontend/dist/index.html` produced, no type errors.

**Step 5: Commit**

```bash
git add frontend/ && git commit -m "chore(frontend): scaffold Svelte + Vite teach pendant app"
```

---

### Task 9: Build integration — Makefile + meta.json

**Files:**
- Modify: `Makefile`, `meta.json`

**Step 1:** Add Makefile targets:

```make
.PHONY: frontend
frontend:
	cd frontend && npm ci && npm run build
```

Make the existing `module` target depend on `frontend` (so `frontend/dist` exists before packaging). Inspect the current `module` target and prepend `frontend` as a prerequisite.

**Step 2:** Add an `applications` array to `meta.json` (schema confirmed from viam-docs `docs/build-apps/hosting/meta-json-reference.md`). Fields: `name` (required, lowercase alphanumeric + hyphens, no leading/trailing hyphen), `type` (required, `"single_machine"` | `"multi_machine"` — use `single_machine`), `entrypoint` (required, path to the built HTML **relative to the archive root**). Optional: `fragmentIds`, `allowedOrgIds`, `logoPath`, `customizations`.

```json
"applications": [
  {
    "name": "teach-pendant",
    "type": "single_machine",
    "entrypoint": "frontend/dist/index.html"
  }
]
```

Requirements confirmed by docs:
- `visibility` **must be `"public"`** — private modules cannot host applications. `meta.json` already has `"visibility": "public"`. Good.
- `meta.json` must sit at the **root of the uploaded archive**; `entrypoint` is resolved relative to that root, so `frontend/dist/index.html` must ship at exactly that path inside the tarball.
- Static files only — no server-side code (this app is a static SPA, fine).
- On upload, use `--platform any` (web apps are platform-independent). NOTE: the existing module `meta.json` `build.arch` lists Go platforms for the binary; the application upload is orthogonal — the app ships in the same tarball via `entrypoint`.

**Step 3:** Ensure the packaged tarball includes `frontend/dist` **at the path `frontend/dist/index.html`** (matching `entrypoint`). Inspect the `module` target's `tar` invocation / packaging file list and confirm `frontend/dist/**` is added (it is git-ignored per Task 8, so it must be included explicitly by the build, not via git). Add it to the tar file list if the packaging uses an explicit list.

**Step 4: Verify**

Run: `make frontend && make module`
Expected: build succeeds; the tarball contains `frontend/dist/index.html` (verify: `tar tzf bin/module.tar.gz | grep frontend/dist/index.html`), and `meta.json` is at the archive root (`tar tzf bin/module.tar.gz | grep -x meta.json`).

**Step 5: Commit**

```bash
git add Makefile meta.json && git commit -m "build: bundle teach-pendant app into module"
```

---

### Task 10: Connection layer (ViamProvider)

Wire the SDK provider using the machine identity Viam injects. **Injection mechanism (confirmed from viam-docs `docs/build-apps/hosting/hosting-reference.md`):** a `single_machine` Viam Application receives the machine's connection info via a **browser cookie whose name is the machine ID**, and the machine ID appears in the URL path as `/machine/{id}/...`. The cookie value is JSON:

```json
{
  "apiKey": { "id": "api-key-id", "key": "api-key-secret" },
  "credentials": { "type": "api-key", "payload": "api-key-secret", "authEntity": "api-key-id" },
  "hostname": "machine-main.xxxx.viam.cloud",
  "machineId": "machine-uuid",
  "timestamp": 1712620800
}
```

`viam module local-app-testing` injects the exact same cookie in dev, so there is a single code path.

The svelte-sdk's `ViamProvider` takes `dialConfigs: Record<key, DialConf>`; the `key` is an opaque lookup id shared between the provider and the resource clients (it does not have to be the Viam part id — it just must match what we pass to `createResourceClient`). We key everything by the machine id read from the URL/cookie.

**Files (create):** `frontend/src/lib/machine.ts`, modify `frontend/src/App.svelte`

**Step 1:** `frontend/src/lib/machine.ts` — read the machine id from the URL path, parse the cookie, and build a `DialConf`:

```ts
import Cookies from 'js-cookie'
import type { DialConf, Credentials } from '@viamrobotics/sdk'

export interface MachineIdentity {
  id: string
  dialConf: DialConf
}

interface MachineCookie {
  hostname: string
  credentials: Credentials
  machineId: string
}

// Viam Applications (single_machine) inject a cookie keyed by the machine id,
// which is the third URL path segment: /machine/{id}/...
// `viam module local-app-testing` injects the same cookie in dev.
export function currentMachine(): MachineIdentity {
  const id = window.location.pathname.split('/')[2]
  if (!id) throw new Error('no machine id in URL path (expected /machine/{id}/...)')
  const raw = Cookies.get(id)
  if (!raw) throw new Error(`no Viam credentials cookie for machine ${id}`)
  const cookie = JSON.parse(raw) as MachineCookie
  return {
    id,
    dialConf: {
      host: cookie.hostname,
      credentials: cookie.credentials,
      signalingAddress: 'https://app.viam.com:443',
    },
  }
}
```

> Confirm the exact `DialConf` field names against the installed `@viamrobotics/sdk` types (`host`, `credentials`, `signalingAddress` are the documented `createRobotClient` fields; `DialConf` should mirror them). Adjust if the installed type differs.

**Step 2:** `App.svelte` wraps content in `ViamProvider`, keyed by the machine id:

```svelte
<script lang="ts">
  import { ViamProvider } from '@viamrobotics/svelte-sdk'
  import { currentMachine } from './lib/machine'
  const machine = currentMachine()
  const dialConfigs = { [machine.id]: machine.dialConf }
</script>

<ViamProvider {dialConfigs}>
  <!-- panels go here (Task 17); pass machine.id as the partID lookup key -->
</ViamProvider>
```

**Step 3: Verify** with `viam module local-app-testing` (see Task 19) that a connection is established (StatusBar indicator, once built). For now assert `npm run build` + `npm run check` pass.

**Step 4: Commit**

```bash
git add frontend/src && git commit -m "feat(frontend): ViamProvider connection layer"
```

---

### Task 11: `poseTracker.ts` helpers + resource clients

Centralize resource-client creation, DoCommand payload builders, and typed response parsers so panels stay thin. Pure functions here are the unit-test surface (Task 18).

**Files (create):** `frontend/src/lib/poseTracker.ts`, `frontend/src/lib/clients.ts`

**Step 1:** `clients.ts` — expose the pose-tracker + arm resource clients:

```ts
import { createResourceClient } from '@viamrobotics/svelte-sdk'
import { PoseTrackerClient } from '@viamrobotics/sdk'

export function poseTrackerClient(partID: () => string, name = 'pose-tracker') {
  return createResourceClient(PoseTrackerClient, partID, () => name)
}
```

**Step 2:** `poseTracker.ts` — payload builders + parsers (framework-free, unit-testable):

```ts
export type CartesianAxis = 'x' | 'y' | 'z' | 'roll' | 'pitch' | 'yaw'

export const jogCartesian = (axis: CartesianAxis, step: number) =>
  ({ jog_cartesian: { axis, step } })

export const jogJoint = (joint: number, step: number) =>
  ({ jog_joint: { joint, step } })

export const stopArm = () => ({ stop_arm: {} })
export const getArmState = () => ({ get_arm_state: {} })
export const capturePoint = () => ({ capture_point: {} })
export const defineFrame = (name: string, method: string) =>
  ({ define_frame: { name, method } })
export const listFrames = () => ({ list_frames: {} })
export const deleteFrame = (name: string) => ({ delete_frame: { name } })

export interface ArmState { pose: PoseMap; joints: number[] }
export interface PoseMap { x: number; y: number; z: number; o_x: number; o_y: number; o_z: number; theta: number }

export function parseArmState(resp: Record<string, unknown>): ArmState {
  return { pose: resp.pose as PoseMap, joints: (resp.joints as number[]) ?? [] }
}
```

**Step 3: Verify:** `cd frontend && npm run check` passes.

**Step 4: Commit**

```bash
git add frontend/src/lib && git commit -m "feat(frontend): pose-tracker clients and payload helpers"
```

---

### Task 12: StatusBar panel (live readouts + Stop)

**Files (create):** `frontend/src/panels/StatusBar.svelte`

**Step 1:** Implement using `createResourceQuery(poseTracker, 'doCommand', () => [getArmState()])` + `usePolling(query.queryKey, () => isMoving ? false : 500)`, and a Stop button wired to `createResourceMutation(poseTracker, 'doCommand')` calling `stopArm()`. Display TCP pose (x/y/z + o_x/o_y/o_z/theta) and joint degrees. Show `useConnectionStatus` state. Stop stays enabled even while other mutations are pending; expose an `isMoving` store (shared via context or props) that jog mutations set.

**Step 2: Verify:** `npm run check` + `npm run build` pass.

**Step 3: Commit**

```bash
git add frontend/src/panels/StatusBar.svelte && git commit -m "feat(frontend): status bar with live readouts and stop"
```

---

### Task 13: JogPanel

**Files (create):** `frontend/src/panels/JogPanel.svelte`

**Step 1:** Implement:
- Mode toggle: Cartesian ↔ Joint.
- Step-size presets: translation `[0.1, 1, 10]` mm, rotation `[1, 5]` deg, joint `[1, 5]` deg — selected value drives the jog.
- Cartesian: 6-axis pad — X± Y± Z± (send `jogCartesian(axis, ±transStep)`), roll± pitch± yaw± (send `jogCartesian(axis, ±rotStep)`).
- Joint: one ± control per joint; joint count from the latest `get_arm_state` `joints.length`.
- Each button = a `createResourceMutation(poseTracker,'doCommand')`; while `isPending`, disable jog buttons and set the shared `isMoving` true (pauses polling). Render `error` inline.

**Step 2: Verify:** `npm run check` + `npm run build`.

**Step 3: Commit**

```bash
git add frontend/src/panels/JogPanel.svelte && git commit -m "feat(frontend): jog panel (cartesian + joint)"
```

---

### Task 14: CapturePanel

**Files (create):** `frontend/src/panels/CapturePanel.svelte`

**Step 1:** "Capture point" button → `capturePoint()` mutation; buffer list via `createResourceQuery` on `{get_buffer:{}}` (invalidate/refetch after capture/clear); "Clear" → `{clear_buffer:{}}`. Show index + pose per row.

**Step 2: Verify / Step 3: Commit**

```bash
git add frontend/src/panels/CapturePanel.svelte && git commit -m "feat(frontend): capture panel"
```

---

### Task 15: FramePanel

**Files (create):** `frontend/src/panels/FramePanel.svelte`

**Step 1:** Name input + method picker (`3point` / `point` / `tcp_snapshot`) with buffer-requirement hints; "Define" → `defineFrame(name, method)`, surfacing `committed`/`replaced` and distinguishing "computed but not persisted" (`committed:false`). Committed-frames list via `{list_frames:{}}` with per-row delete (`deleteFrame(name)`) and clear-all (`{clear_frames:{}}`).

**Step 2: Verify / Step 3: Commit**

```bash
git add frontend/src/panels/FramePanel.svelte && git commit -m "feat(frontend): frame panel"
```

---

### Task 16: TcpPanel

**Files (create):** `frontend/src/panels/TcpPanel.svelte`

**Step 1:** TCP capture button (`{capture_tcp_point:{}}`) + TCP buffer view (`{get_tcp_buffer:{}}`); `teach_tcp_position` (show `residual_rms` on success) and `teach_tcp_orientation` (o_x/o_y/o_z/theta inputs). Disable the whole panel with an explanatory note when no arm is configured — detect via a failed `get_arm_state` / "arm dependency not configured" error.

**Step 2: Verify / Step 3: Commit**

```bash
git add frontend/src/panels/TcpPanel.svelte && git commit -m "feat(frontend): tcp teaching panel"
```

---

### Task 17: Assemble App layout

**Files:** modify `frontend/src/App.svelte`

**Step 1:** Inside `ViamProvider`, lay out StatusBar (pinned top), JogPanel (primary region), and Capture→Frame→Tcp panels alongside. Provide the shared `isMoving` store and `partID` via Svelte context so panels don't re-derive them. Responsive, tablet-friendly hit targets.

**Step 2: Verify:** `npm run build` + `npm run check`.

**Step 3: Commit**

```bash
git add frontend/src/App.svelte && git commit -m "feat(frontend): assemble teach pendant layout"
```

---

### Task 18: Frontend unit tests

**Files (create):** `frontend/src/lib/poseTracker.test.ts`

**Step 1:** Vitest tests for the payload builders and `parseArmState` — assert exact DoCommand shapes (e.g. `jogCartesian('x', -1)` → `{ jog_cartesian: { axis: 'x', step: -1 } }`) and that `parseArmState` tolerates missing joints. (Panel-level tests optional; keep v1 light per the design.)

**Step 2: Run:** `cd frontend && npm test` — Expected: PASS.

**Step 3: Commit**

```bash
git add frontend/src/lib/poseTracker.test.ts && git commit -m "test(frontend): payload builder unit tests"
```

---

### Task 19: Manual verification + docs

**Step 1:** Build everything: `make frontend && go build ./...`.

**Step 2:** Run against a machine (real arm or a fake arm module) using the local app proxy. Flags confirmed from viam-docs `docs/build-apps/tasks/test-locally.md` — the proxy serves at `http://localhost:8012` and injects the same cookies as production:

```bash
# Start the Vite dev server (or `npm run preview` on the built dist) first, e.g. on :5173
cd frontend && npm run dev &

# Then, from a logged-in CLI session, proxy + inject the machine cookie:
viam login
viam module local-app-testing \
  --app-url http://localhost:5173 \
  --machine-id YOUR-MACHINE-ID
# opens http://localhost:8012/start which sets the cookie and redirects to the app
```

Notes: `--app-url` is **required** (the URL incl. port where the app runs); `--machine-id` selects single-machine mode (the CLI fetches an API key + FQDN for that machine and writes the machine-id-keyed cookie). Port 8012 is hardcoded.

Confirm the end-to-end loop: connection indicator green → jog (cartesian + joint) moves the arm → Stop halts → capture 3 points → define a `3point` frame → frame appears in the list. If the machine-identity cookie parsing (Task 10) needs adjustment, fix `machine.ts` here.

**Step 3:** Update `README.md` with a "Teach Pendant application" section: what it is, that it ships with the module, how to open it, and the local-app-testing command for development.

**Step 4: Commit**

```bash
git add README.md frontend/src/lib/machine.ts && git commit -m "docs: teach pendant application usage"
```

---

## Verification checklist (before merge)

- [ ] `go test ./...` passes (jog + arm-state handlers covered).
- [ ] `cd frontend && npm run check && npm run build && npm test` all pass.
- [ ] `make module` produces a tarball containing `frontend/dist/index.html`.
- [ ] Manual: jog (cartesian world-translation + tool-rotation, joint), Stop, capture→define loop verified via `viam module local-app-testing`.
- [ ] Rotation jog verified to rotate about the **tool** frame (not world) and to leave position fixed.
- [ ] `arm`-not-configured path: jog/stop/arm-state return clear errors; TcpPanel and jog UI degrade gracefully.

## Open items to confirm during execution

1. ~~Machine-identity injection~~ — **RESOLVED** (viam-docs): cookie keyed by machine id (from URL path `/machine/{id}/...`), read with `js-cookie`; same in prod and under `local-app-testing`. Implemented in Task 10.
2. ~~`meta.json` `applications` schema~~ — **RESOLVED** (viam-docs): `name`/`type`/`entrypoint` (+ optional `fragmentIds`/`allowedOrgIds`/`logoPath`/`customizations`); `visibility: public` required; `meta.json` at archive root; `entrypoint` relative to root. Implemented in Task 9.
3. **`@viamrobotics/sdk` version** that satisfies the `svelte-sdk@1.2.3` peer (`>=0.51`) and exports `PoseTrackerClient` / `ArmClient` / `DialConf` / `Credentials` (Task 8/11). Confirm the exact `DialConf` field names at install time (Task 10).
