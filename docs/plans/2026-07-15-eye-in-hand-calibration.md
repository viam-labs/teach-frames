# Eye-in-Hand Calibration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add wrist-mounted (eye-in-hand) camera calibration to the `pose-tracker` component, solving camera→flange from manually-picked RGBD points and persisting it as the camera's frame with `parent = <arm>`.

**Architecture:** Eye-in-hand reduces to the *same* Kabsch solve as the shipped eye-to-hand feature. Given a known target `P` and flange pose `F_i`, `X ∘ c_i = F_i⁻¹ ∘ P`, so aligning the camera points to the target expressed in each flange frame recovers camera→flange. The shipped solver is generalized into a shared core; eye-in-hand adds a thin adapter, a second buffer, two new commands, and its own panel.

**Tech Stack:** Go 1.x, `go.viam.com/rdk@v0.131.0`, `gonum/mat` (SVD), Svelte 5 runes, `@viamrobotics/svelte-sdk`, vitest.

**Spec:** `docs/plans/2026-07-15-eye-in-hand-calibration-design.md` — read the "Frame conventions" and "Shipped bug" sections before starting.

---

## Read this first

Three things have already bitten this feature. Do not rediscover them.

1. **`spatialmath.Compose` applies the TRANSPOSE** of what `RotationMatrix.Col()` reads back. Documented in `frames/compute.go` and `frames/tcp.go`. The round-trip test is the only guard. Write it first, every time.
2. **The fakes hide real bugs.** `posesource.Fake.Capture` hardcodes `referenceframe.World` regardless of `DestFrame`, so no fake-based test proves frame-name correctness. `posesource.FakeCamera.Snapshot` returns a *fixed* snapshot, so every deprojected point is identical — a degenerate configuration. A green fake-based test means the plumbing is wired, not that the geometry is right.
3. **`arm.EndPosition` and `motion.GetPose(arm, destFrame)` return DIFFERENT frames** (arm base vs destination frame). The touched point arrives in `destination_frame`. Mixing them passes every test on a machine whose arm sits at the world origin and silently miscalibrates elsewhere. Use `motion.GetPose`; do **not** reuse `posesource.FlangeSource`.

**Already done on this branch — do NOT redo:** the two-sided degeneracy guard fix in `frames/handeye.go` (`ee96cd9`). `minSpreadEps`, `maxSpreadRatio`, and the `spread()` helper already exist and are tested.

## File structure

| File | Responsibility |
|---|---|
| `frames/handeye.go` | *(modify)* Shared Kabsch core + `Deproject`. Rename to honest names. |
| `frames/eyeinhand.go` | *(create)* `EyeInHandObservation`, `ComputeCameraToFlange`. Adapter only — no math. |
| `frames/eyeinhand_test.go` | *(create)* Geometry tests, incl. degeneracy with noise. |
| `store/store.go` | *(modify)* Second buffer for eye-in-hand observations. |
| `models/posetracker/model.go` | *(modify)* `camera_mount` config, `Validate`, tracker state, constructor wiring. |
| `models/posetracker/handeye.go` | *(modify)* Snapshot gating, target/view capture, solve branching. |
| `models/posetracker/docommand.go` | *(modify)* Dispatch + canonical command doc comment. |
| `frontend/src/lib/poseTracker.ts` | *(modify)* Command builders + types. **Do NOT rename `world`** — it types the wire. |
| `frontend/src/panels/EyeInHandPanel.svelte` | *(create)* Eye-in-hand operator flow. |
| `frontend/src/panels/HandEyePanel.svelte` | *(modify)* Mode gating only. |
| `frontend/src/App.svelte` | *(modify)* Mount the new panel. |
| `README.md` | *(modify)* Config, verbs, walkthrough. |

---

### Task 1: Rename the solver core to honest names

Mechanical, no behavior change. `Reference` is accurate for both eye-to-hand (a `destination_frame` point) and eye-in-hand (a flange-frame point); `World` would be a lie in one of them.

**Files:**
- Modify: `frames/handeye.go`, `frames/handeye_test.go`
- Modify: `store/store.go`, `store/store_test.go`
- Modify: `models/posetracker/handeye.go`, `models/posetracker/docommand.go`, `models/posetracker/docommand_test.go`
- Modify: `README.md`

**DO NOT touch `frontend/src/lib/poseTracker.ts`.** Its `HandEyePair` interface types the *wire*, which keeps the `world` key for compatibility with the shipped `HandEyePanel`. The Go rename must not leak to the wire.

- [ ] **Step 1: Rename the type and function**

In `frames/handeye.go`:
- `type HandEyePair struct { World, Camera r3.Vector }` → `type PointPair struct { Reference, Camera r3.Vector }`
- `func ComputeCameraToWorld(pairs []HandEyePair)` → `func ComputeRigidTransform(pairs []PointPair)`
- All `p.World` → `p.Reference`; `worldMean` → `refMean`; `wCentered` → `refCentered`; `worldSpread` → `refSpread`.
- Error message `"world points are coincident; move the arm between captures"` → `"reference points are coincident; move the arm between captures"`.

Update the doc comment to pin the unit contract and keep the degeneracy rationale:

```go
// PointPair is one 3D correspondence: the same physical point expressed in the
// camera frame and in the frame we are solving the camera's pose relative to
// (the destination frame for eye-to-hand, the flange frame for eye-in-hand).
type PointPair struct {
	Reference r3.Vector
	Camera    r3.Vector
}

// ComputeRigidTransform solves the rigid transform T such that reference ≈ T·camera
// (Kabsch/Umeyama, rotation only). Returns an error for too-few, collinear, or
// degenerate inputs. Coplanar (but non-collinear) inputs are accepted.
//
// Units: input points are in mm, the returned residual is the RMS fit error in
// mm, and minSpreadEps is in mm. The type is otherwise unit-agnostic, so this
// contract lives here or nowhere.
//
// Degeneracy is checked on BOTH clouds, not just the camera one. If the reference
// points collapse to a single point — the operator never moved the arm between
// captures — then reference-refMean is zero for every pair, so H is exactly zero
// and the solve returns identity rotation no matter the truth. Camera noise keeps
// the camera cloud looking healthy, so a camera-side check cannot see it, and the
// residual stays small enough to read as a good calibration while the answer is
// hundreds of mm wrong.
```

- [ ] **Step 2: Update every call site**

`store/store.go`: `handeyeBuffer []frames.PointPair`; `AddHandEyePair(p frames.PointPair)`; `HandEyeBuffer() []frames.PointPair`. **Keep the method names** — they name the hand-eye buffer, not the type.

`models/posetracker/handeye.go`: `frames.PointPair{Reference: worldPt, Camera: camPt}`; `frames.ComputeRigidTransform(buf)`.

`models/posetracker/docommand.go`, in `get_handeye_buffer` — the Go field changes, the **wire key does not**:

```go
pts[i] = map[string]interface{}{"world": vecToMap(p.Reference), "camera": vecToMap(p.Camera)}
```

Test files: rename **all ten** `TestComputeCameraToWorld*` function names in
`frames/handeye_test.go` to `TestComputeRigidTransform*`, and every
`HandEyePair{World: ...}` literal to `PointPair{Reference: ...}`. (The spec says
six — it predates `ee96cd9`, which added four degeneracy tests on this branch.
Trust the grep in Step 3, not the count.)

Also update the comment in `frames/handeye.go` that names
`TestComputeCameraToWorldRoundTrip`.

`README.md`: the one mention of `ComputeCameraToWorld`.

- [ ] **Step 3: Verify no expected values changed**

Run: `go build ./... && go test ./... 2>&1 | grep -v "^ok"`
Expected: no output (all pass). Every existing assertion must pass **unedited apart from names and literals** — that is what proves this was behavior-preserving. If any *expected value* needed changing, stop: the rename broke something.

Run: `grep -rn "HandEyePair\|ComputeCameraToWorld" --include="*.go" .`
Expected: no matches.

Run: `grep -n "world" frontend/src/lib/poseTracker.ts`
Expected: still present. If you renamed it, revert that file.

- [ ] **Step 4: Commit**

```bash
git add -A && git commit -m "refactor(frames): rename HandEyePair/ComputeCameraToWorld to honest names

The solver is about to be shared with eye-in-hand, where the reference
point is in the flange frame, not the world. PointPair{Reference} is true
in both modes; HandEyePair{World} would be a lie in one -- the same class
of naming trap that hid the destination_frame parent bug.

The wire keeps its \"world\" key for compatibility with the shipped panel,
so poseTracker.ts is deliberately untouched."
```

---

### Task 2: `ComputeCameraToFlange`

**Files:**
- Create: `frames/eyeinhand.go`
- Create: `frames/eyeinhand_test.go`

- [ ] **Step 1: Write the round-trip test FIRST**

This is the transpose-convention guard. Nothing downstream is worth debugging until it passes.

```go
package frames

import (
	"math"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
)

// eyeInHandTruth is a camera->flange transform with a non-trivial rotation, so a
// solver that wrongly returns identity cannot accidentally look correct.
func eyeInHandTruth() spatialmath.Pose {
	return spatialmath.NewPose(
		r3.Vector{X: 30, Y: -20, Z: 50},
		&spatialmath.EulerAngles{Roll: math.Pi / 2},
	)
}

// buildObservations synthesizes observations of targets viewed from flange poses,
// inverting the real data flow: given truth X, the camera point that WOULD have
// been deprojected is X^-1 o F^-1 o Target.
func buildObservations(truth spatialmath.Pose, targets []r3.Vector, flanges []spatialmath.Pose) []EyeInHandObservation {
	xInv := spatialmath.PoseInverse(truth)
	obs := make([]EyeInHandObservation, len(flanges))
	for i, f := range flanges {
		target := targets[i%len(targets)]
		q := spatialmath.Compose(spatialmath.PoseInverse(f), spatialmath.NewPoseFromPoint(target)).Point()
		obs[i] = EyeInHandObservation{
			Target: target,
			Flange: f,
			Camera: spatialmath.Compose(xInv, spatialmath.NewPoseFromPoint(q)).Point(),
		}
	}
	return obs
}

func TestComputeCameraToFlangeRoundTrip(t *testing.T) {
	truth := eyeInHandTruth()
	targets := []r3.Vector{{X: 400, Y: 100, Z: 0}}
	flanges := []spatialmath.Pose{
		spatialmath.NewPose(r3.Vector{X: 300, Y: 0, Z: 400}, &spatialmath.EulerAngles{Yaw: 0}),
		spatialmath.NewPose(r3.Vector{X: 350, Y: 80, Z: 420}, &spatialmath.EulerAngles{Yaw: 0.3, Pitch: 0.2}),
		spatialmath.NewPose(r3.Vector{X: 280, Y: -60, Z: 380}, &spatialmath.EulerAngles{Yaw: -0.4, Roll: 0.25}),
		spatialmath.NewPose(r3.Vector{X: 320, Y: 40, Z: 450}, &spatialmath.EulerAngles{Pitch: -0.35, Roll: -0.2}),
	}

	got, residual, err := ComputeCameraToFlange(buildObservations(truth, targets, flanges))
	test.That(t, err, test.ShouldBeNil)
	test.That(t, residual, test.ShouldBeLessThan, 1e-6)
	test.That(t, spatialmath.PoseAlmostEqualEps(got, truth, 1e-6), test.ShouldBeTrue)
}
```

- [ ] **Step 2: Run it to watch it fail**

Run: `go test ./frames/ -run TestComputeCameraToFlangeRoundTrip -v`
Expected: build failure — `undefined: ComputeCameraToFlange`, `undefined: EyeInHandObservation`.

- [ ] **Step 3: Implement**

Create `frames/eyeinhand.go`:

```go
// Package frames — eye-in-hand (wrist-mounted camera) hand-eye calibration.
package frames

import (
	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
)

// EyeInHandObservation is one eye-in-hand observation: a target point touched by
// the calibrated TCP, the flange pose at the moment the camera viewed it, and the
// deprojected click.
//
// Target and Flange MUST be expressed in the same frame (the configured
// destination_frame). The underlying equation is frame-agnostic, but only if both
// terms agree — mixing an arm-base flange pose (arm.EndPosition) with a
// destination-frame target silently miscalibrates on any machine whose arm base
// is not at the world origin.
type EyeInHandObservation struct {
	Target r3.Vector
	Flange spatialmath.Pose
	Camera r3.Vector
}

// ComputeCameraToFlange solves the camera->flange transform X from observations of
// known targets viewed from varying flange poses. Returns the pose, the RMS fit
// residual (mm), and an error for too-few or degenerate inputs.
//
// The camera is rigidly mounted to the flange, so X is constant and
// Target = Flange o X o Camera, hence X o Camera = Flange^-1 o Target. The
// right-hand side is computable, so this is an ordinary rigid point-cloud
// alignment between the camera points and each target expressed in its flange
// frame — the same Kabsch solve eye-to-hand uses, with no AX=XB machinery.
//
// Deprojection yields full 3D points rather than bearings, so pure-translation
// vantages recover the rotation too. Do NOT add a rotational-diversity check; it
// would reject valid data. See ComputeRigidTransform for the degeneracy guards
// that DO apply.
func ComputeCameraToFlange(obs []EyeInHandObservation) (spatialmath.Pose, float64, error) {
	pairs := make([]PointPair, len(obs))
	for i, o := range obs {
		q := spatialmath.Compose(
			spatialmath.PoseInverse(o.Flange),
			spatialmath.NewPoseFromPoint(o.Target),
		).Point()
		pairs[i] = PointPair{Reference: q, Camera: o.Camera}
	}
	return ComputeRigidTransform(pairs)
}
```

- [ ] **Step 4: Run it to watch it pass**

Run: `go test ./frames/ -run TestComputeCameraToFlangeRoundTrip -v`
Expected: PASS. If the orientation is wrong but the translation is right, you have hit the transpose convention — read `frames/compute.go`.

- [ ] **Step 5: Commit**

```bash
git add frames/eyeinhand.go frames/eyeinhand_test.go
git commit -m "feat(frames): add ComputeCameraToFlange for eye-in-hand"
```

---

### Task 3: Eye-in-hand geometry tests

The round-trip proves the happy path. These pin the properties that are easy to "fix" into wrongness.

**Files:**
- Modify: `frames/eyeinhand_test.go`

**Add `"math/rand"` to the import block** — this task is the first to use it.

**Why these and not the spec's full list:** the spec also lists coincident camera
points, near-degenerate δ=0.01/1 mm rejection, δ=10/70 mm acceptance, and bounded
noisy residual. Those all live in `frames/handeye_test.go` already, on the shared
core, from `ee96cd9`. `ComputeCameraToFlange` delegates to that core, so
re-testing them here would duplicate coverage. `RejectsNeverJogged` below is the
exception: it is worth having at *this* layer because it is the eye-in-hand
operator's specific mistake, and it proves the delegation actually reaches the
guard.

- [ ] **Step 1: Write the tests**

```go
// TestComputeCameraToFlangePureTranslation is a documentation test as much as a
// correctness one. Classic AX=XB cannot recover rotation without rotational
// motion, so this property looks wrong and invites someone to "fix" it by adding
// a rotational-diversity check. It is correct: deprojection gives 3D points, not
// bearings, so each view contributes 3 constraints. A diversity check would reject
// valid data.
func TestComputeCameraToFlangePureTranslation(t *testing.T) {
	truth := eyeInHandTruth()
	same := &spatialmath.EulerAngles{Yaw: 0.1}
	flanges := []spatialmath.Pose{
		spatialmath.NewPose(r3.Vector{X: 300, Y: 0, Z: 400}, same),
		spatialmath.NewPose(r3.Vector{X: 380, Y: 0, Z: 400}, same),
		spatialmath.NewPose(r3.Vector{X: 300, Y: 90, Z: 400}, same),
		spatialmath.NewPose(r3.Vector{X: 300, Y: 0, Z: 470}, same),
	}

	got, _, err := ComputeCameraToFlange(buildObservations(truth, []r3.Vector{{X: 400, Y: 100, Z: 0}}, flanges))
	test.That(t, err, test.ShouldBeNil)
	test.That(t, spatialmath.PoseAlmostEqualEps(got, truth, 1e-6), test.ShouldBeTrue)
}

// TestComputeCameraToFlangeMixedTargets validates the flexible observation
// protocol: each observation is an independent constraint regardless of which
// target it came from, so any mix of targets and vantages must solve.
func TestComputeCameraToFlangeMixedTargets(t *testing.T) {
	truth := eyeInHandTruth()
	targets := []r3.Vector{{X: 400, Y: 100, Z: 0}, {X: 350, Y: -120, Z: 40}}
	flanges := []spatialmath.Pose{
		spatialmath.NewPose(r3.Vector{X: 300, Y: 0, Z: 400}, &spatialmath.EulerAngles{Yaw: 0}),
		spatialmath.NewPose(r3.Vector{X: 350, Y: 80, Z: 420}, &spatialmath.EulerAngles{Yaw: 0.3, Pitch: 0.2}),
		spatialmath.NewPose(r3.Vector{X: 280, Y: -60, Z: 380}, &spatialmath.EulerAngles{Yaw: -0.4, Roll: 0.25}),
		spatialmath.NewPose(r3.Vector{X: 320, Y: 40, Z: 450}, &spatialmath.EulerAngles{Pitch: -0.35, Roll: -0.2}),
	}

	got, residual, err := ComputeCameraToFlange(buildObservations(truth, targets, flanges))
	test.That(t, err, test.ShouldBeNil)
	test.That(t, residual, test.ShouldBeLessThan, 1e-6)
	test.That(t, spatialmath.PoseAlmostEqualEps(got, truth, 1e-6), test.ShouldBeTrue)
}

func TestComputeCameraToFlangeRejectsTooFew(t *testing.T) {
	truth := eyeInHandTruth()
	flanges := []spatialmath.Pose{
		spatialmath.NewPose(r3.Vector{X: 300, Y: 0, Z: 400}, &spatialmath.EulerAngles{Yaw: 0}),
		spatialmath.NewPose(r3.Vector{X: 350, Y: 80, Z: 420}, &spatialmath.EulerAngles{Yaw: 0.3}),
	}
	_, _, err := ComputeCameraToFlange(buildObservations(truth, []r3.Vector{{X: 400, Y: 100, Z: 0}}, flanges))
	test.That(t, err, test.ShouldNotBeNil)
}

// TestComputeCameraToFlangeRejectsNeverJogged is the operator's most likely
// mistake: snapshot and click repeatedly without moving the arm. Every q_i is then
// identical. The noise is load-bearing -- without it the camera points coincide
// too and a camera-side check alone would catch it, proving nothing about the
// reference-side guard.
func TestComputeCameraToFlangeRejectsNeverJogged(t *testing.T) {
	truth := eyeInHandTruth()
	rng := rand.New(rand.NewSource(11))
	f := spatialmath.NewPose(r3.Vector{X: 300, Y: 0, Z: 400}, &spatialmath.EulerAngles{Yaw: 0.1})

	obs := buildObservations(truth, []r3.Vector{{X: 400, Y: 100, Z: 0}},
		[]spatialmath.Pose{f, f, f, f}) // same flange pose every time
	for i := range obs {
		obs[i].Camera = r3.Vector{
			X: obs[i].Camera.X + rng.NormFloat64(),
			Y: obs[i].Camera.Y + rng.NormFloat64(),
			Z: obs[i].Camera.Z + rng.NormFloat64(),
		}
	}

	_, _, err := ComputeCameraToFlange(obs)
	test.That(t, err, test.ShouldNotBeNil)
}

// TestComputeCameraToFlangeRejectsCollinearVantages covers vantages along a line.
func TestComputeCameraToFlangeRejectsCollinearVantages(t *testing.T) {
	truth := eyeInHandTruth()
	same := &spatialmath.EulerAngles{Yaw: 0.1}
	flanges := []spatialmath.Pose{
		spatialmath.NewPose(r3.Vector{X: 300, Y: 0, Z: 400}, same),
		spatialmath.NewPose(r3.Vector{X: 340, Y: 0, Z: 400}, same),
		spatialmath.NewPose(r3.Vector{X: 380, Y: 0, Z: 400}, same),
		spatialmath.NewPose(r3.Vector{X: 420, Y: 0, Z: 400}, same),
	}
	_, _, err := ComputeCameraToFlange(buildObservations(truth, []r3.Vector{{X: 400, Y: 100, Z: 0}}, flanges))
	test.That(t, err, test.ShouldNotBeNil)
}
```

- [ ] **Step 2: Run them**

Run: `go test ./frames/ -run TestComputeCameraToFlange -v`
Expected: all PASS. These exercise guards that already exist in `ComputeRigidTransform`, so they should pass without new production code. **If `RejectsNeverJogged` fails, stop** — the degeneracy guard is not working and everything downstream inherits a silent miscalibration.

- [ ] **Step 3: Commit**

```bash
git add frames/eyeinhand_test.go
git commit -m "test(frames): pin eye-in-hand geometry properties

Includes the never-jogged case with camera noise injected. Without noise
the camera points coincide and a camera-side check catches it, which
would certify nothing about the reference-side guard."
```

---

### Task 4: Store buffer for eye-in-hand observations

Two buffers, not one — the element types differ, and mount mode is fixed by config so a machine only ever uses one.

**Files:**
- Modify: `store/store.go`, `store/store_test.go`

- [ ] **Step 1: Write the failing test**

In `store/store_test.go`, mirroring the existing hand-eye buffer tests:

```go
func TestEyeInHandBuffer(t *testing.T) {
	s := New()
	test.That(t, s.EyeInHandBufferLen(), test.ShouldEqual, 0)

	o := frames.EyeInHandObservation{
		Target: r3.Vector{X: 1},
		Flange: spatialmath.NewPoseFromPoint(r3.Vector{Z: 3}),
		Camera: r3.Vector{Y: 2},
	}
	test.That(t, s.AddEyeInHandObservation(o), test.ShouldEqual, 0)
	test.That(t, s.AddEyeInHandObservation(o), test.ShouldEqual, 1)
	test.That(t, s.EyeInHandBufferLen(), test.ShouldEqual, 2)

	buf := s.EyeInHandBuffer()
	test.That(t, len(buf), test.ShouldEqual, 2)
	buf[0].Target = r3.Vector{X: 99} // copy-on-read: must not affect the store
	test.That(t, s.EyeInHandBuffer()[0].Target.X, test.ShouldEqual, 1.0)

	test.That(t, s.ClearEyeInHandBuffer(), test.ShouldEqual, 2)
	test.That(t, s.EyeInHandBufferLen(), test.ShouldEqual, 0)
}
```

- [ ] **Step 2: Run it to watch it fail**

Run: `go test ./store/ -run TestEyeInHandBuffer -v`
Expected: build failure — `s.EyeInHandBufferLen undefined`.

- [ ] **Step 3: Implement**

Add the field alongside `handeyeBuffer`:

```go
eyeInHandBuffer []frames.EyeInHandObservation
```

And the four methods, mirroring the hand-eye ones exactly (same mutex, same copy-on-read):

```go
// AddEyeInHandObservation appends an eye-in-hand observation and returns its index.
func (s *FrameStore) AddEyeInHandObservation(o frames.EyeInHandObservation) int {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.eyeInHandBuffer = append(s.eyeInHandBuffer, o)
	return len(s.eyeInHandBuffer) - 1
}

// EyeInHandBuffer returns a copy of the current eye-in-hand buffer.
// Mutating the returned slice does not affect the store.
func (s *FrameStore) EyeInHandBuffer() []frames.EyeInHandObservation {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return append([]frames.EyeInHandObservation(nil), s.eyeInHandBuffer...)
}

// EyeInHandBufferLen returns the number of observations in the eye-in-hand buffer.
func (s *FrameStore) EyeInHandBufferLen() int {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return len(s.eyeInHandBuffer)
}

// ClearEyeInHandBuffer discards all observations and returns the count removed.
func (s *FrameStore) ClearEyeInHandBuffer() int {
	s.mu.Lock()
	defer s.mu.Unlock()
	n := len(s.eyeInHandBuffer)
	s.eyeInHandBuffer = nil
	return n
}
```

- [ ] **Step 4: Run it to watch it pass**

Run: `go test ./store/ -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add store/store.go store/store_test.go
git commit -m "feat(store): add eye-in-hand observation buffer"
```

---

### Task 5: `camera_mount` config + validation

**Files:**
- Modify: `models/posetracker/model.go`, `models/posetracker/model_test.go`

- [ ] **Step 1: Write the failing tests**

```go
func TestValidateCameraMountDefaultsToEyeToHand(t *testing.T) {
	c := &Config{TCPComponent: "arm"}
	_, _, err := c.Validate("")
	test.That(t, err, test.ShouldBeNil)
	test.That(t, c.cameraMountOrDefault(), test.ShouldEqual, mountEyeToHand)
}

func TestValidateRejectsUnknownCameraMount(t *testing.T) {
	c := &Config{TCPComponent: "arm", CameraMount: "eye_on_stalk"}
	_, _, err := c.Validate("")
	test.That(t, err, test.ShouldNotBeNil)
}

func TestValidateEyeInHandRequiresArm(t *testing.T) {
	c := &Config{TCPComponent: "arm", CameraMount: mountEyeInHand, Camera: "cam"}
	_, _, err := c.Validate("")
	test.That(t, err, test.ShouldNotBeNil)
}

func TestValidateEyeInHandRequiresCamera(t *testing.T) {
	c := &Config{TCPComponent: "arm", CameraMount: mountEyeInHand, Arm: "ur5"}
	_, _, err := c.Validate("")
	test.That(t, err, test.ShouldNotBeNil)
}

func TestValidateEyeInHandAccepted(t *testing.T) {
	c := &Config{TCPComponent: "tcp", CameraMount: mountEyeInHand, Arm: "ur5", Camera: "cam"}
	deps, _, err := c.Validate("")
	test.That(t, err, test.ShouldBeNil)
	test.That(t, deps, test.ShouldContain, "ur5")
	test.That(t, deps, test.ShouldContain, "cam")
}
```

- [ ] **Step 2: Run to watch fail**

Run: `go test ./models/posetracker/ -run TestValidate -v`
Expected: build failure — `mountEyeToHand undefined`, `CameraMount undefined`.

- [ ] **Step 3: Implement**

Add the constants and config field:

```go
// Camera mount modes. The mount is a physical fact that never changes at
// runtime, so it belongs in config rather than being inferred per command —
// which also lets the module refuse a flow that contradicts the hardware
// instead of silently solving garbage.
const (
	mountEyeToHand = "eye_to_hand"
	mountEyeInHand = "eye_in_hand"
)
```

In `Config`, after `Camera`:

```go
CameraMount string `json:"camera_mount"`
```

```go
// cameraMountOrDefault returns the configured mount, defaulting to eye_to_hand so
// configs written before this attribute existed keep working unchanged.
func (c *Config) cameraMountOrDefault() string {
	if c.CameraMount == "" {
		return mountEyeToHand
	}
	return c.CameraMount
}
```

In `Validate`, after the existing dep collection and before `return`:

```go
switch c.cameraMountOrDefault() {
case mountEyeToHand:
case mountEyeInHand:
	// Eye-in-hand needs the arm name to ask the motion service for flange
	// poses, and the camera to attach the solved frame to.
	if c.Arm == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "arm")
	}
	if c.Camera == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "camera")
	}
default:
	return nil, nil, resource.NewConfigValidationError(path, fmt.Errorf(
		"camera_mount must be %q or %q, got %q", mountEyeToHand, mountEyeInHand, c.CameraMount))
}
```

Add `"fmt"` to imports if absent.

- [ ] **Step 4: Run to watch pass**

Run: `go test ./models/posetracker/ -v 2>&1 | grep -E "FAIL|ok"`
Expected: ok.

- [ ] **Step 5: Commit**

```bash
git add models/posetracker/model.go models/posetracker/model_test.go
git commit -m "feat(posetracker): add camera_mount config attribute"
```

---

### Task 6: Tracker state and constructor wiring

**Files:**
- Modify: `models/posetracker/model.go`

- [ ] **Step 1: Add the fields**

In `teachTracker`, near the camera fields:

```go
cameraMount string

// flangeInDest reads the arm flange pose in destFrame. It is deliberately NOT
// posesource.FlangeSource: that wraps arm.EndPosition, which reports in the ARM
// BASE frame, while the touched target arrives in destFrame. The solve is
// frame-agnostic but only if both terms agree, so mixing them would pass every
// test on a machine whose arm sits at the world origin and silently
// miscalibrate everywhere else. nil unless mount is eye_in_hand AND the arm
// resolved.
flangeInDest posesource.PoseSource

targetMu      sync.Mutex
currentTarget *r3.Vector
```

Alongside `lastSnapshot`, under the existing `snapshotMu` comment:

```go
// lastFlange is the flange pose read at snapshot time, cached under snapshotMu
// with lastSnapshot. The operator jogs, snapshots, THEN clicks, so a flange read
// at click time could describe a pose the pixels never came from.
lastFlange spatialmath.Pose
```

Add `"github.com/golang/geo/r3"` and `"go.viam.com/rdk/spatialmath"` to imports if absent.

- [ ] **Step 3: Wire the constructor**

After `pt.tcpComponent = cfg.TCPComponent`:

```go
pt.cameraMount = cfg.cameraMountOrDefault()
```

Inside the existing `if cfg.Arm != ""` block, in the `else` branch after `pt.arm = a`:

```go
		// Gated on the arm actually resolving, not just on cfg.Arm being set:
		// MotionSource holds only a name string, so it would be non-nil even for
		// an arm that does not exist, turning a clear "arm not configured" error
		// into an opaque GetPose failure at snapshot time.
		if pt.cameraMount == mountEyeInHand {
			pt.flangeInDest = &posesource.MotionSource{Motion: mot, Component: cfg.Arm, DestFrame: dest}
		}
```

- [ ] **Step 2: Write the test that pins the wiring — DO NOT SKIP**

This is the only test in the entire plan that exercises the constructor line
above, and it is the one guard against trap #3. Every later task injects
`pt.flangeInDest` directly over the field, so without this test an implementer can
write `Component: cfg.TCPComponent`, or reuse `pt.flange` (`&posesource.ArmSource{}`
— i.e. `arm.EndPosition`, in the **arm base frame**), and get a **fully green
suite** plus a silent miscalibration on any machine whose arm base is not at the
world origin. That is the exact failure shape of `4afb61e`.

`newForTest` cannot be used: it injects only the motion service
(`model_test.go:25`), so `arm.FromDependencies` always fails and `flangeInDest`
stays nil. Follow `TestNewPoseTrackerWithArmBuildsFlange` (`model_test.go:199`),
which injects a resolvable arm.

In `models/posetracker/model_test.go`:

```go
func TestNewPoseTrackerEyeInHandBuildsFlangeInDest(t *testing.T) {
	motionName := motion.Named(defaultMotionService)
	deps := resource.Dependencies{
		motionName:          injectmotion.NewMotionService(defaultMotionService),
		arm.Named("my-arm"): inject.NewArm("my-arm"),
	}

	conf := resource.Config{
		Name:  "test-tracker",
		API:   posetracker.API,
		Model: Model,
		ConvertedAttributes: &Config{
			TCPComponent:     "tool",
			Arm:              "my-arm",
			Camera:           "cam",
			CameraMount:      mountEyeInHand,
			DestinationFrame: "base",
		},
	}

	res, err := newPoseTracker(context.Background(), deps, conf, logging.NewTestLogger(t))
	test.That(t, err, test.ShouldBeNil)

	tt := res.(*teachTracker)
	test.That(t, tt.flangeInDest, test.ShouldNotBeNil)

	// Assert the concrete type and its wiring. A literal ArmSource substitution
	// is already impossible here (it implements FlangeSource.CaptureFlange, not
	// PoseSource.Capture, so it would not compile), but reusing pt.source
	// (Component = tcp_component) or passing the wrong DestFrame both compile
	// fine and both miscalibrate silently. Those are what these assertions catch.
	ms, ok := tt.flangeInDest.(*posesource.MotionSource)
	test.That(t, ok, test.ShouldBeTrue)
	test.That(t, ms.Component, test.ShouldEqual, "my-arm")   // the ARM, not tcp_component
	test.That(t, ms.DestFrame, test.ShouldEqual, "base")     // same frame as the target
}

// Eye-to-hand needs no arm and must not build flangeInDest.
func TestNewPoseTrackerEyeToHandHasNoFlangeInDest(t *testing.T) {
	motionName := motion.Named(defaultMotionService)
	deps := resource.Dependencies{
		motionName:          injectmotion.NewMotionService(defaultMotionService),
		arm.Named("my-arm"): inject.NewArm("my-arm"),
	}
	conf := resource.Config{
		Name:                "test-tracker",
		API:                 posetracker.API,
		Model:               Model,
		ConvertedAttributes: &Config{TCPComponent: "tool", Arm: "my-arm"},
	}

	res, err := newPoseTracker(context.Background(), deps, conf, logging.NewTestLogger(t))
	test.That(t, err, test.ShouldBeNil)
	test.That(t, res.(*teachTracker).flangeInDest, test.ShouldBeNil)
}
```

- [ ] **Step 4: Run to watch it pass**

Run: `go test ./models/posetracker/ -run TestNewPoseTrackerEyeInHand -v`
Expected: PASS. (You watched it fail at the end of Step 2, before wiring.)

Then the full suite: `go build ./... && go test ./... 2>&1 | grep -v "^ok"`
Expected: no output.

- [ ] **Step 5: Commit**

```bash
git add models/posetracker/model.go models/posetracker/model_test.go
git commit -m "feat(posetracker): wire eye-in-hand tracker state

flangeInDest uses motion.GetPose rather than FlangeSource: EndPosition
reports in the arm base frame while the target arrives in destFrame, and
mixing the two miscalibrates on any machine whose arm base is not at the
world origin -- while passing every test on one where it is. The
constructor test asserts the concrete MotionSource type and its
Component/DestFrame, because every other test injects the field directly
and would not catch the substitution."
```

---

### Task 7: Gate the snapshot's flange read on mount

**Files:**
- Modify: `models/posetracker/handeye.go`, `models/posetracker/docommand_test.go`

- [ ] **Step 1: Extract the shared camera fixtures FIRST**

`testRGB()`, `testDepth()`, and `testIntr()` do **not** exist yet — this step
creates them, and Tasks 9, 10, and 12 depend on them.

Two inline fixtures exist today and they **disagree on which pixel carries
depth**: `TestHandEyeSnapshot` (`docommand_test.go:959`) sets `dm.Set(1, 1, 500)`,
while `TestCaptureHandEyePoint` (`docommand_test.go:996`) sets `dm.Set(3, 1, 1000)`.
Every test in this plan clicks `(u=1, v=1)`, so **extract the (1,1) variant**.
Extracting the other one makes every Task 9 test fail with `no depth at pixel
(1,1)` — a failure that looks like a logic bug in `captureHandEyeView` and isn't.

Add to `models/posetracker/docommand_test.go`:

```go
// Shared 4x4 RGBD fixture for hand-eye tests. Depth is set at pixel (1,1) ONLY,
// so tests must click u=1, v=1; any other pixel deprojects to "no depth".
func testIntr() *transform.PinholeCameraIntrinsics {
	return &transform.PinholeCameraIntrinsics{Width: 4, Height: 4, Fx: 2, Fy: 2, Ppx: 2, Ppy: 2}
}

func testDepth() *rimage.DepthMap {
	dm := rimage.NewEmptyDepthMap(4, 4)
	dm.Set(1, 1, rimage.Depth(500))
	return dm
}

func testRGB() image.Image {
	return image.NewRGBA(image.Rect(0, 0, 4, 4))
}
```

Refactor `TestHandEyeSnapshot` to use them, but **bind locals and reuse them**:

```go
intr, dm := testIntr(), testDepth()
pt.cameraSrc = &posesource.FakeCamera{RGB: testRGB(), Depth: dm, Intr: intr}
// ...
test.That(t, pt.lastSnapshot.Intr, test.ShouldEqual, intr) // pointer identity
```

That test asserts `pt.lastSnapshot.Intr ShouldEqual intr` — **pointer identity**,
deliberately, to prove the cache holds the snapshot just acquired rather than a
fresh one. Inlining `testIntr()` into the assertion constructs a *second* pointer
and fails with a confusing "Both the actual and expected values render equally"
message.

**Leave `TestCaptureHandEyePoint` alone** — it clicks (3,1) and asserts against
that depth; rewriting it to the shared fixture would change what it tests. Run `go test ./models/posetracker/` and
confirm still green before continuing.

- [ ] **Step 2: Write the failing tests**

Note `models/posetracker/handeye.go` does **not** currently import `spatialmath`;
the implementation below needs it.

```go
// Eye-to-hand requires no arm at all, so the shared snapshot verb must not read
// the flange unconditionally -- that would break every arm-less machine.
func TestHandeyeSnapshotEyeToHandWorksWithoutArm(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	pt.cameraSrc = &posesource.FakeCamera{RGB: testRGB(), Depth: testDepth(), Intr: testIntr()}
	pt.cameraMount = mountEyeToHand
	pt.flangeInDest = nil // no arm configured

	_, err := pt.DoCommand(context.Background(), map[string]interface{}{"handeye_snapshot": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
}

func TestHandeyeSnapshotEyeInHandCachesFlange(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	pt.cameraSrc = &posesource.FakeCamera{RGB: testRGB(), Depth: testDepth(), Intr: testIntr()}
	pt.cameraMount = mountEyeInHand
	f1 := spatialmath.NewPoseFromPoint(r3.Vector{X: 11})
	pt.flangeInDest = &posesource.Fake{Poses: []spatialmath.Pose{f1}}

	_, err := pt.DoCommand(context.Background(), map[string]interface{}{"handeye_snapshot": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, pt.lastFlange, test.ShouldNotBeNil)
	test.That(t, pt.lastFlange.Point().X, test.ShouldAlmostEqual, 11.0)
}

func TestHandeyeSnapshotEyeInHandErrorsWithoutArm(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	pt.cameraSrc = &posesource.FakeCamera{RGB: testRGB(), Depth: testDepth(), Intr: testIntr()}
	pt.cameraMount = mountEyeInHand
	pt.flangeInDest = nil

	_, err := pt.DoCommand(context.Background(), map[string]interface{}{"handeye_snapshot": map[string]interface{}{}})
	test.That(t, err, test.ShouldNotBeNil)
}
```

- [ ] **Step 3: Run to watch fail**

Run: `go test ./models/posetracker/ -run TestHandeyeSnapshot -v`
Expected: `TestHandeyeSnapshotEyeInHandCachesFlange` fails (`pt.lastFlange` is nil), and the errors-without-arm test fails (no error returned).

- [ ] **Step 4: Implement**

In `handeyeSnapshot`, after the JPEG encode and before the cache write:

```go
	// Eye-in-hand only: freeze the flange pose WITH the image. The operator jogs,
	// snapshots, then clicks; a flange read at click time could describe a pose
	// the pixels never came from. Eye-to-hand touches and clicks at the same arm
	// pose and requires no arm at all, so this must stay gated.
	var flange spatialmath.Pose
	if pt.cameraMount == mountEyeInHand {
		if pt.flangeInDest == nil {
			return nil, errors.New("arm dependency not configured or not resolvable; cannot run eye-in-hand calibration")
		}
		pif, ferr := pt.flangeInDest.Capture(ctx)
		if ferr != nil {
			return nil, fmt.Errorf("could not read flange pose: %w", ferr)
		}
		flange = pif.Pose()
	}

	pt.snapshotMu.Lock()
	pt.lastSnapshot = snap
	pt.lastFlange = flange
	pt.snapshotMu.Unlock()
```

(Replace the existing three-line cache write.)

- [ ] **Step 5: Run to watch pass**

Run: `go test ./models/posetracker/ -v 2>&1 | grep -E "FAIL|ok"`
Expected: ok.

- [ ] **Step 6: Commit**

```bash
git add models/posetracker/handeye.go models/posetracker/docommand_test.go
git commit -m "feat(posetracker): freeze flange pose with the snapshot in eye-in-hand"
```

---

### Task 8: `capture_handeye_target`

**Files:**
- Modify: `models/posetracker/handeye.go`, `models/posetracker/docommand.go`, `models/posetracker/docommand_test.go`

- [ ] **Step 1: Write the failing tests**

```go
func TestCaptureHandEyeTargetSetsTarget(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	pt.cameraMount = mountEyeInHand
	pt.source = &posesource.Fake{Poses: []spatialmath.Pose{spatialmath.NewPoseFromPoint(r3.Vector{X: 7, Y: 8, Z: 9})}}

	resp, err := pt.DoCommand(context.Background(), map[string]interface{}{"capture_handeye_target": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, resp["target"].(map[string]interface{})["x"], test.ShouldAlmostEqual, 7.0)
	test.That(t, pt.currentTarget, test.ShouldNotBeNil)
}

func TestCaptureHandEyeTargetRejectedInEyeToHand(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	pt.cameraMount = mountEyeToHand

	_, err := pt.DoCommand(context.Background(), map[string]interface{}{"capture_handeye_target": map[string]interface{}{}})
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "capture_handeye_point")
}
```

- [ ] **Step 2: Run to watch fail**

Run: `go test ./models/posetracker/ -run TestCaptureHandEyeTarget -v`
Expected: FAIL — `unknown command: [capture_handeye_target]`.

- [ ] **Step 3: Implement**

In `handeye.go`:

```go
// captureHandEyeTarget reads the current TCP pose and stores it as the target that
// subsequent views will be paired against. Eye-in-hand only: the arm cannot touch
// a point and view it from a distance at the same time, so touching and viewing
// are separate steps.
func (pt *teachTracker) captureHandEyeTarget(ctx context.Context) (map[string]interface{}, error) {
	if pt.cameraMount != mountEyeInHand {
		return nil, fmt.Errorf("camera_mount is %q; use capture_handeye_point instead", pt.cameraMount)
	}
	pif, err := pt.source.Capture(ctx)
	if err != nil {
		return nil, err
	}
	p := pif.Pose().Point()

	pt.targetMu.Lock()
	pt.currentTarget = &p
	pt.targetMu.Unlock()

	return map[string]interface{}{"target": vecToMap(p)}, nil
}
```

In `docommand.go`, add before `case has(cmd, "solve_handeye")`:

```go
	case has(cmd, "capture_handeye_target"):
		return pt.captureHandEyeTarget(ctx)
```

- [ ] **Step 4: Run to watch pass**

Run: `go test ./models/posetracker/ -run TestCaptureHandEyeTarget -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add models/posetracker/ && git commit -m "feat(posetracker): add capture_handeye_target"
```

---

### Task 9: `capture_handeye_view`

**Files:**
- Modify: `models/posetracker/handeye.go`, `models/posetracker/docommand.go`, `models/posetracker/docommand_test.go`

- [ ] **Step 1: Write the failing tests**

The freeze test is the important one. **Use a separate `Fake` for `source` and `flangeInDest`** — one fake backing both shares a queue and the assertion stops meaning what it says.

```go
// The snapshot must freeze the flange pose. If the implementation re-reads the
// flange at click time it gets F2 instead of F1 and this fails -- which is the
// race we cannot see on hardware.
func TestCaptureHandEyeViewUsesCachedFlange(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	pt.cameraMount = mountEyeInHand
	pt.cameraSrc = &posesource.FakeCamera{RGB: testRGB(), Depth: testDepth(), Intr: testIntr()}
	// SEPARATE fakes: a shared queue would make this assertion meaningless.
	pt.source = &posesource.Fake{Poses: []spatialmath.Pose{spatialmath.NewPoseFromPoint(r3.Vector{X: 400})}}
	f1 := spatialmath.NewPoseFromPoint(r3.Vector{X: 11})
	f2 := spatialmath.NewPoseFromPoint(r3.Vector{X: 22})
	pt.flangeInDest = &posesource.Fake{Poses: []spatialmath.Pose{f1, f2}}

	ctx := context.Background()
	_, err := pt.DoCommand(ctx, map[string]interface{}{"capture_handeye_target": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
	_, err = pt.DoCommand(ctx, map[string]interface{}{"handeye_snapshot": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
	_, err = pt.DoCommand(ctx, map[string]interface{}{"capture_handeye_view": map[string]interface{}{"u": 1.0, "v": 1.0}})
	test.That(t, err, test.ShouldBeNil)

	buf := pt.store.EyeInHandBuffer()
	test.That(t, len(buf), test.ShouldEqual, 1)
	test.That(t, buf[0].Flange.Point().X, test.ShouldAlmostEqual, 11.0) // F1, not F2
}

func TestCaptureHandEyeViewRequiresTarget(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	pt.cameraMount = mountEyeInHand
	pt.cameraSrc = &posesource.FakeCamera{RGB: testRGB(), Depth: testDepth(), Intr: testIntr()}
	pt.flangeInDest = &posesource.Fake{Poses: []spatialmath.Pose{spatialmath.NewPoseFromPoint(r3.Vector{X: 11})}}

	ctx := context.Background()
	_, err := pt.DoCommand(ctx, map[string]interface{}{"handeye_snapshot": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
	_, err = pt.DoCommand(ctx, map[string]interface{}{"capture_handeye_view": map[string]interface{}{"u": 1.0, "v": 1.0}})
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "capture_handeye_target")
}

// One view per snapshot. Without this an operator can click twice without
// re-snapshotting, producing two identical observations -- exactly the degenerate
// set the solver must reject.
func TestCaptureHandEyeViewConsumesSnapshot(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	pt.cameraMount = mountEyeInHand
	pt.cameraSrc = &posesource.FakeCamera{RGB: testRGB(), Depth: testDepth(), Intr: testIntr()}
	pt.source = &posesource.Fake{Poses: []spatialmath.Pose{spatialmath.NewPoseFromPoint(r3.Vector{X: 400})}}
	pt.flangeInDest = &posesource.Fake{Poses: []spatialmath.Pose{spatialmath.NewPoseFromPoint(r3.Vector{X: 11})}}

	ctx := context.Background()
	_, _ = pt.DoCommand(ctx, map[string]interface{}{"capture_handeye_target": map[string]interface{}{}})
	_, _ = pt.DoCommand(ctx, map[string]interface{}{"handeye_snapshot": map[string]interface{}{}})
	_, err := pt.DoCommand(ctx, map[string]interface{}{"capture_handeye_view": map[string]interface{}{"u": 1.0, "v": 1.0}})
	test.That(t, err, test.ShouldBeNil)

	_, err = pt.DoCommand(ctx, map[string]interface{}{"capture_handeye_view": map[string]interface{}{"u": 1.0, "v": 1.0}})
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, pt.store.EyeInHandBufferLen(), test.ShouldEqual, 1)
}

func TestCaptureHandEyeViewRejectedInEyeToHand(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	pt.cameraMount = mountEyeToHand
	_, err := pt.DoCommand(context.Background(), map[string]interface{}{"capture_handeye_view": map[string]interface{}{"u": 1.0, "v": 1.0}})
	test.That(t, err, test.ShouldNotBeNil)
}
```

- [ ] **Step 2: Run to watch fail**

Run: `go test ./models/posetracker/ -run TestCaptureHandEyeView -v`
Expected: FAIL — `unknown command: [capture_handeye_view]`.

- [ ] **Step 3: Implement**

```go
// captureHandEyeView deprojects the clicked pixel against the cached snapshot and
// pairs it with the current target and the flange pose frozen at snapshot time.
func (pt *teachTracker) captureHandEyeView(ctx context.Context, raw interface{}) (map[string]interface{}, error) {
	if pt.cameraMount != mountEyeInHand {
		return nil, fmt.Errorf("camera_mount is %q; use capture_handeye_point instead", pt.cameraMount)
	}
	args, ok := raw.(map[string]interface{})
	if !ok {
		return nil, errors.New("capture_handeye_view args must be an object")
	}
	uF, ok := args["u"].(float64)
	if !ok {
		return nil, errors.New("capture_handeye_view requires a numeric 'u' (pixel column)")
	}
	vF, ok := args["v"].(float64)
	if !ok {
		return nil, errors.New("capture_handeye_view requires a numeric 'v' (pixel row)")
	}

	pt.targetMu.Lock()
	target := pt.currentTarget
	pt.targetMu.Unlock()
	if target == nil {
		return nil, errors.New("no target set; run capture_handeye_target before capturing a view")
	}

	// Take-and-clear in one critical section: each snapshot yields at most one
	// observation. Two clicks on one frozen frame would produce identical
	// observations, which is a degenerate set the solve must reject. Clearing
	// here rather than after a successful deproject keeps this race-free; a
	// failed deproject just means the operator re-snapshots.
	pt.snapshotMu.Lock()
	snap := pt.lastSnapshot
	flange := pt.lastFlange
	pt.lastSnapshot = nil
	pt.lastFlange = nil
	pt.snapshotMu.Unlock()
	if snap == nil {
		return nil, errors.New("no snapshot cached; run handeye_snapshot before each view")
	}

	camPt, err := frames.Deproject(snap.Intr, snap.Depth, int(uF), int(vF))
	if err != nil {
		return nil, err
	}

	idx := pt.store.AddEyeInHandObservation(frames.EyeInHandObservation{
		Target: *target,
		Flange: flange,
		Camera: camPt,
	})
	return map[string]interface{}{
		"index":      idx,
		"buffer_len": pt.store.EyeInHandBufferLen(),
		"target":     vecToMap(*target),
		"flange":     poseToMap(flange),
		"camera":     vecToMap(camPt),
	}, nil
}
```

Dispatch:

```go
	case has(cmd, "capture_handeye_view"):
		return pt.captureHandEyeView(ctx, cmd["capture_handeye_view"])
```

- [ ] **Step 4: Run to watch pass**

Run: `go test ./models/posetracker/ -v 2>&1 | grep -E "FAIL|ok"`
Expected: ok.

- [ ] **Step 5: Commit**

```bash
git add models/posetracker/ && git commit -m "feat(posetracker): add capture_handeye_view

Consumes the snapshot per view: two clicks on one frozen frame would
produce identical observations, which is the degenerate set the solver
must reject."
```

---

### Task 10: Guard `capture_handeye_point` against eye-in-hand

**Files:**
- Modify: `models/posetracker/handeye.go`, `models/posetracker/docommand_test.go`

- [ ] **Step 1: Write the failing test**

```go
func TestCaptureHandEyePointRejectedInEyeInHand(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	pt.cameraMount = mountEyeInHand
	pt.cameraSrc = &posesource.FakeCamera{RGB: testRGB(), Depth: testDepth(), Intr: testIntr()}

	_, err := pt.DoCommand(context.Background(), map[string]interface{}{"capture_handeye_point": map[string]interface{}{"u": 1.0, "v": 1.0}})
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "capture_handeye_target")
}
```

- [ ] **Step 2: Run to watch fail**

Run: `go test ./models/posetracker/ -run TestCaptureHandEyePointRejectedInEyeInHand -v`
Expected: FAIL — no error returned.

- [ ] **Step 3: Implement**

At the top of `captureHandEyePoint`, before the `cameraSrc` nil check:

```go
	if pt.cameraMount != mountEyeToHand {
		return nil, fmt.Errorf("camera_mount is %q; use capture_handeye_target and capture_handeye_view instead", pt.cameraMount)
	}
```

- [ ] **Step 4: Run to watch pass**

Run: `go test ./models/posetracker/ -v 2>&1 | grep -E "FAIL|ok"`
Expected: ok.

- [ ] **Step 5: Commit**

```bash
git add models/posetracker/ && git commit -m "feat(posetracker): reject eye-to-hand capture on a wrist camera"
```

---

### Task 11: Branch `solve_handeye` on mount

**Files:**
- Modify: `models/posetracker/handeye.go`, `models/posetracker/docommand_test.go`

- [ ] **Step 1: Write the failing test**

The parent-frame assertion is the direct analogue of the `destination_frame` test that caught the shipped bug.

```go
func TestSolveHandEyeEyeInHandPersistsArmAsParent(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "tcp", DestinationFrame: "world"})
	pt.cameraMount = mountEyeInHand
	pt.cameraName = "wrist_cam"
	pt.armName = "ur5"
	fp := &persist.Fake{}
	pt.persist = fp

	// Non-degenerate synthetic observations: reuse the frames package's geometry.
	truth := spatialmath.NewPose(r3.Vector{X: 30, Y: -20, Z: 50}, &spatialmath.EulerAngles{Roll: math.Pi / 2})
	xInv := spatialmath.PoseInverse(truth)
	target := r3.Vector{X: 400, Y: 100, Z: 0}
	flanges := []spatialmath.Pose{
		spatialmath.NewPose(r3.Vector{X: 300, Y: 0, Z: 400}, &spatialmath.EulerAngles{Yaw: 0}),
		spatialmath.NewPose(r3.Vector{X: 350, Y: 80, Z: 420}, &spatialmath.EulerAngles{Yaw: 0.3, Pitch: 0.2}),
		spatialmath.NewPose(r3.Vector{X: 280, Y: -60, Z: 380}, &spatialmath.EulerAngles{Yaw: -0.4, Roll: 0.25}),
		spatialmath.NewPose(r3.Vector{X: 320, Y: 40, Z: 450}, &spatialmath.EulerAngles{Pitch: -0.35, Roll: -0.2}),
	}
	for _, f := range flanges {
		q := spatialmath.Compose(spatialmath.PoseInverse(f), spatialmath.NewPoseFromPoint(target)).Point()
		pt.store.AddEyeInHandObservation(frames.EyeInHandObservation{
			Target: target,
			Flange: f,
			Camera: spatialmath.Compose(xInv, spatialmath.NewPoseFromPoint(q)).Point(),
		})
	}

	resp, err := pt.DoCommand(context.Background(), map[string]interface{}{"solve_handeye": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, resp["parent"], test.ShouldEqual, "ur5") // NOT "world"
	test.That(t, fp.SavedParent, test.ShouldEqual, "ur5")
	test.That(t, pt.store.EyeInHandBufferLen(), test.ShouldEqual, 0)
	test.That(t, pt.currentTarget, test.ShouldBeNil) // cleared on success
}
```

**Check `persist.Fake`'s actual recorded-field names** before writing this (`persist/persist.go`) and use whatever it exposes; if it records nothing, add a recorded parent field in this task.

- [ ] **Step 2: Run to watch fail**

Run: `go test ./models/posetracker/ -run TestSolveHandEyeEyeInHand -v`
Expected: FAIL at `test.That(t, err, test.ShouldBeNil)` with `camera calibration
requires at least 3 points, have 0` — pre-implementation, `solveHandEye` reads the
**eye-to-hand** buffer, which this test never filled. It does not reach the parent
assertion until the branch exists. That is the correct failure; if you see the
parent assertion fire instead, the branch is already half-written.

- [ ] **Step 3: Implement**

(`handeye.go` already imports `spatialmath` as of Task 7.)

First, add a guard after the existing `cameraName` check. `Validate` enforces this
in production, but tests set fields directly and a blank parent would persist
`frame {parent: ""}` without complaint:

```go
	if pt.cameraMount == mountEyeInHand && pt.armName == "" {
		return nil, errors.New("arm not configured; cannot solve eye-in-hand calibration")
	}
```

Then rewrite the solve body between the guards and `commitMu`:

```go
	var pose spatialmath.Pose
	var residual float64
	var err error
	var parent string

	if pt.cameraMount == mountEyeInHand {
		// Observations pair a destFrame target with a destFrame flange pose, so
		// the solve yields camera->flange. The frame system attaches a child with
		// parent = <arm> at the arm's kinematic tip (the flange), so that is the
		// correct parent -- NOT destFrame.
		pose, residual, err = frames.ComputeCameraToFlange(pt.store.EyeInHandBuffer())
		parent = pt.armName
	} else {
		// Captured points come from pt.source.Capture in pt.destFrame, so the
		// solve yields camera->destFrame.
		pose, residual, err = frames.ComputeRigidTransform(pt.store.HandEyeBuffer())
		parent = pt.destFrame
	}
	if err != nil {
		return nil, fmt.Errorf("hand-eye solve failed: %w", err)
	}
```

Replace `pt.destFrame` with `parent` in the `SaveComponentFrame` call and in the response map. Replace the buffer clear with:

```go
	if pt.cameraMount == mountEyeInHand {
		pt.store.ClearEyeInHandBuffer()
		pt.targetMu.Lock()
		pt.currentTarget = nil
		pt.targetMu.Unlock()
	} else {
		pt.store.ClearHandEyeBuffer()
	}
```

- [ ] **Step 4: Run to watch pass**

Run: `go test ./models/posetracker/ -v 2>&1 | grep -E "FAIL|ok"`
Expected: ok. The existing eye-to-hand solve tests must still pass unchanged.

- [ ] **Step 5: Commit**

```bash
git add models/posetracker/ && git commit -m "feat(posetracker): solve camera->flange with parent=arm for eye-in-hand"
```

---

### Task 12: `get_handeye_mode`, and mount-aware buffer verbs

**Files:**
- Modify: `models/posetracker/docommand.go`, `models/posetracker/docommand_test.go`

- [ ] **Step 1: Write the failing tests**

```go
func TestGetHandEyeMode(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	pt.cameraMount = mountEyeInHand
	pt.cameraSrc = &posesource.FakeCamera{RGB: testRGB(), Depth: testDepth(), Intr: testIntr()}

	resp, err := pt.DoCommand(context.Background(), map[string]interface{}{"get_handeye_mode": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, resp["camera_mount"], test.ShouldEqual, mountEyeInHand)
	test.That(t, resp["camera_configured"], test.ShouldBeTrue)
}

func TestGetHandEyeBufferEyeInHandShape(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	pt.cameraMount = mountEyeInHand
	pt.store.AddEyeInHandObservation(frames.EyeInHandObservation{
		Target: r3.Vector{X: 1},
		Flange: spatialmath.NewPoseFromPoint(r3.Vector{Z: 3}),
		Camera: r3.Vector{Y: 2},
	})

	resp, err := pt.DoCommand(context.Background(), map[string]interface{}{"get_handeye_buffer": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
	pts := resp["points"].([]interface{})
	test.That(t, len(pts), test.ShouldEqual, 1)
	p := pts[0].(map[string]interface{})
	test.That(t, p["target"], test.ShouldNotBeNil)
	test.That(t, p["camera"], test.ShouldNotBeNil)
	// flange is a POSE, not a vector: dropping the orientation would discard
	// exactly what the solve depends on.
	test.That(t, p["flange"].(map[string]interface{})["theta"], test.ShouldNotBeNil)
}

func TestClearHandEyeBufferEyeInHandClearsTarget(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	pt.cameraMount = mountEyeInHand
	p := r3.Vector{X: 1}
	pt.currentTarget = &p
	pt.store.AddEyeInHandObservation(frames.EyeInHandObservation{Flange: spatialmath.NewZeroPose()})

	resp, err := pt.DoCommand(context.Background(), map[string]interface{}{"clear_handeye_buffer": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, resp["cleared"], test.ShouldEqual, 1)
	test.That(t, pt.currentTarget, test.ShouldBeNil)
}
```

- [ ] **Step 2: Run to watch fail**

Run: `go test ./models/posetracker/ -run "TestGetHandEye|TestClearHandEyeBufferEyeInHand" -v`
Expected: FAIL — `unknown command: [get_handeye_mode]`, and the eye-in-hand buffer returns the wrong shape.

- [ ] **Step 3: Implement**

Replace the `get_handeye_buffer` and `clear_handeye_buffer` cases and add the mode verb:

```go
	case has(cmd, "get_handeye_mode"):
		return map[string]interface{}{
			"camera_mount":      pt.cameraMount,
			"camera_configured": pt.cameraSrc != nil,
			"arm_configured":    pt.arm != nil,
		}, nil

	case has(cmd, "get_handeye_buffer"):
		// Shape differs by mount. Mount is fixed by config, so a given machine
		// only ever sees one shape. The eye-to-hand "world" key is retained for
		// compatibility with the shipped panel despite the Go field rename.
		if pt.cameraMount == mountEyeInHand {
			buf := pt.store.EyeInHandBuffer()
			pts := make([]interface{}, len(buf))
			for i, o := range buf {
				pts[i] = map[string]interface{}{
					"target": vecToMap(o.Target),
					"flange": poseToMap(o.Flange),
					"camera": vecToMap(o.Camera),
				}
			}
			return map[string]interface{}{"points": pts}, nil
		}
		buf := pt.store.HandEyeBuffer()
		pts := make([]interface{}, len(buf))
		for i, p := range buf {
			pts[i] = map[string]interface{}{"world": vecToMap(p.Reference), "camera": vecToMap(p.Camera)}
		}
		return map[string]interface{}{"points": pts}, nil

	case has(cmd, "clear_handeye_buffer"):
		if pt.cameraMount == mountEyeInHand {
			n := pt.store.ClearEyeInHandBuffer()
			// Drop the target too: a stale one would silently poison the next session.
			pt.targetMu.Lock()
			pt.currentTarget = nil
			pt.targetMu.Unlock()
			return map[string]interface{}{"cleared": n}, nil
		}
		return map[string]interface{}{"cleared": pt.store.ClearHandEyeBuffer()}, nil
```

- [ ] **Step 4: Run to watch pass**

Run: `go test ./... 2>&1 | grep -v "^ok"`
Expected: no output.

- [ ] **Step 5: Commit**

```bash
git add models/posetracker/ && git commit -m "feat(posetracker): add get_handeye_mode, branch buffer verbs on mount"
```

---

### Task 13: Frontend command builders and types

**Files:**
- Modify: `frontend/src/lib/poseTracker.ts`, `frontend/src/lib/poseTracker.test.ts`

- [ ] **Step 1: Write the failing tests**

```ts
describe('eye-in-hand commands', () => {
  it('builds capture_handeye_target', () => {
    expect(captureHandeyeTarget()).toEqual({ capture_handeye_target: {} })
  })
  it('builds capture_handeye_view with pixel coords', () => {
    expect(captureHandeyeView(412, 233)).toEqual({ capture_handeye_view: { u: 412, v: 233 } })
  })
  it('builds get_handeye_mode', () => {
    expect(getHandeyeMode()).toEqual({ get_handeye_mode: {} })
  })
})
```

- [ ] **Step 2: Run to watch fail**

Run: `cd frontend && npx vitest run src/lib/poseTracker.test.ts`
Expected: FAIL — `captureHandeyeTarget is not defined`.

- [ ] **Step 3: Implement**

Append to `poseTracker.ts`, after the existing hand-eye builders. **Do not modify the existing `HandEyePair` interface** — it types the wire, which still sends `world`.

```ts
export const captureHandeyeTarget = () => ({ capture_handeye_target: {} })
export const captureHandeyeView = (u: number, v: number) => ({ capture_handeye_view: { u, v } })
export const getHandeyeMode = () => ({ get_handeye_mode: {} })

export type CameraMount = 'eye_to_hand' | 'eye_in_hand'

export interface HandEyeModeResponse {
  camera_mount: CameraMount
  camera_configured: boolean
  arm_configured: boolean
}

export interface CaptureHandEyeTargetResponse {
  target: { x: number; y: number; z: number }
}

// `flange` is a full pose, not a vector: the orientation is what the solve
// depends on.
export interface EyeInHandObservation {
  target: { x: number; y: number; z: number }
  flange: PoseMap
  camera: { x: number; y: number; z: number }
}

export interface CaptureHandEyeViewResponse extends EyeInHandObservation {
  index: number
  buffer_len: number
}

export interface EyeInHandBufferResponse {
  points: EyeInHandObservation[]
}
```

- [ ] **Step 4: Run to watch pass**

Run: `cd frontend && npx vitest run`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add frontend/src/lib/ && git commit -m "feat(frontend): add eye-in-hand command builders and types"
```

---

### Task 14: `EyeInHandPanel.svelte`

**Files:**
- Create: `frontend/src/panels/EyeInHandPanel.svelte`

Read `frontend/src/panels/HandEyePanel.svelte` first and follow its structure, query/mutation patterns, and styling. Some markup will be duplicated — that is a deliberate decision recorded in the spec, not an oversight. Do **not** refactor `HandEyePanel`.

- [ ] **Step 1: Build the panel**

Requirements:
- Calls `get_handeye_mode` on mount; renders **nothing** unless `camera_mount === 'eye_in_hand'`.
- Step 1 button: "Touch target" → `capture_handeye_target`. Show the returned target coords, and that a target is set.
- Step 2 button: "Snapshot" → `handeye_snapshot`, renders the frozen base64 JPEG with `object-fit: fill`.
- Click on the image → `displayToNativePixel(...)` → `capture_handeye_view(u, v)`. Reuse the helper as-is.
- **The UI must make "same physical point, new vantage" obvious.** Unlike eye-to-hand, the operator clicks the *same* point from each new view. Label the step accordingly (e.g. "Jog the arm, snapshot, then click the SAME point").
- After a view is captured, the snapshot is consumed server-side — clear the displayed frame and prompt for a new snapshot rather than leaving a stale image that looks clickable.
- Buffer table from `get_handeye_buffer` (target / flange / camera).
- "Solve" disabled below 3 observations; on success show `residual_rms` and `parent`.
- Surface errors from every mutation, matching `HandEyePanel`'s error handling.
- **Do not present `residual_rms` as proof of a good calibration.** It is flat across the never-jogged failure. If you show a hint, recommend varying the arm pose between views.

- [ ] **Step 2: Verify it compiles clean**

Run: `cd frontend && npx svelte-check --threshold warning 2>&1 | tail -3`
Expected: `0 errors, 0 warnings`. Note `svelte-ignore` codes must be **comma-separated** in runes mode; space-separated only suppresses the first.

Run: `cd frontend && npm run build`
Expected: builds.

- [ ] **Step 3: Commit**

```bash
git add frontend/src/panels/EyeInHandPanel.svelte
git commit -m "feat(frontend): add EyeInHandPanel"
```

---

### Task 15: Mount the panel and gate both panels

**Files:**
- Modify: `frontend/src/App.svelte`, `frontend/src/panels/HandEyePanel.svelte`

- [ ] **Step 1: Gate `HandEyePanel`**

Add the same `get_handeye_mode` query; render nothing unless `camera_mount === 'eye_to_hand'`. An operator should never see a panel that can only error.

- [ ] **Step 2: Mount `EyeInHandPanel`**

In `App.svelte`, import it and add after the existing `HandEyePanel` section:

```svelte
<section class="panel"><EyeInHandPanel /></section>
```

- [ ] **Step 3: Verify**

Run: `cd frontend && npx svelte-check --threshold warning 2>&1 | tail -3 && npm run build && npx vitest run`
Expected: 0 errors, 0 warnings; builds; tests pass.

- [ ] **Step 4: Commit**

```bash
git add frontend/src/ && git commit -m "feat(frontend): mount EyeInHandPanel, gate both panels on camera_mount"
```

---

### Task 16: Documentation

**Files:**
- Modify: `README.md`, `models/posetracker/docommand.go`

- [ ] **Step 1: Update the canonical command doc comment**

`docommand.go`'s doc comment is the authoritative command list. Add `capture_handeye_target`, `capture_handeye_view`, and `get_handeye_mode`.

Also fix `solveHandEye`'s own doc comment in `handeye.go` (currently around
line 94): it hardcodes "parent = pt.destFrame", which is now only true for
eye-to-hand.

- [ ] **Step 1b: CORRECT the README's residual claim — this is a deletion, not an addition**

`README.md:207` currently reads:

> On success, `solve_handeye` reports `residual_rms` — the RMS fit error in
> mm — **as the trust signal for calibration quality**; treat a large residual as a
> sign to re-touch points more carefully or capture a better-spread set.

That is now known to be false, and it is worse than a missing caveat: it actively
tells operators to trust the one number that does not move. The residual is flat
(~1.6–1.8 mm) across the entire never-jogged failure range, including cases that
are 850 mm wrong. Rewrite the sentence so residual is *a* signal that catches
sloppy touches, not *the* signal that certifies a calibration — and say plainly
that a low residual does not by itself mean the calibration is good.

Also soften `README.md:320` (walkthrough step 12: "Confirm `committed: true` and a
low `residual_rms`"). It is milder — a low residual is necessary-but-not-sufficient,
so it works as a checklist item — but it should not be the only thing the
walkthrough tells an operator to check.

Leave `README.md:140` (TCP teaching) alone: `ComputePivotTCP` is an
over-determined least-squares fit where the residual genuinely does measure
consistency. The claim is only wrong for hand-eye.

- [ ] **Step 2: Update the README**

- Config table: `camera_mount` (default `eye_to_hand`; `eye_in_hand` requires `arm` and `camera`).
- DoCommand table: the three new verbs; note `solve_handeye`'s `parent` differs by mount.
- New "Hand-eye calibration (eye-in-hand)" section: the touch-target → jog → snapshot → click-same-point loop; ≥3 observations, 4+ for a meaningful residual; any mix of targets and vantages is fine.
- **Recommend varying wrist orientation between vantages.** Pure translation is valid and must not be rejected, but it is measurably worse-conditioned (2.42° vs 0.77° worst-case rotation error under 1mm noise) and the residual will not reveal it.
- **State plainly that `residual_rms` alone does not certify a calibration.** It is flat across the never-jogged failure.
- Persistence: eye-in-hand patches the camera's `frame` with `parent = <arm>`.
- Caveat: the tracker is `AlwaysRebuild`, so any config edit drops the buffer and current target mid-calibration; restart the calibration after editing config.
- Prerequisite: run `teach_tcp_position` first — target accuracy is bounded by the TCP calibration, and it matters more here since flange poses feed the solve directly.

- [ ] **Step 3: Verify**

Run: `grep -rn "ComputeCameraToWorld" README.md`
Expected: no matches (renamed in Task 1).

- [ ] **Step 4: Commit**

```bash
git add README.md models/posetracker/docommand.go
git commit -m "docs: document eye-in-hand calibration"
```

---

### Task 17: Final verification and holistic review

- [ ] **Step 1: Full suite**

```bash
go build ./... && go test ./... && go vet ./...
gofmt -l frames/ store/ posesource/ models/
cd frontend && npx svelte-check --threshold warning && npm run build && npx vitest run
```

Expected: all green; `gofmt` silent. **Known pre-existing:** `persist/appclient_test.go` fails `gofmt -l` on `main` — not yours, leave it. `make lint` fails only on that.

- [ ] **Step 2: Holistic review — MANDATORY**

Both bugs that escaped the eye-to-hand feature were **integration** bugs (a mislabeled parent frame, a missing resolution check), and both passed per-task reviews that were individually correct. The coincident-points bug then escaped that review too. Per-task review does not see cross-task seams.

Review the whole diff against the spec, specifically hunting:
- Frame mismatches: is every `Target`/`Flange` pair in the same frame, on every path?
- Does any test's green depend on a fake's fixed/hardcoded behavior rather than real geometry?
- Do the wire shapes match what the panels actually read?
- Is `parent` right for both mounts?

- [ ] **Step 3: Confirm the hardware gap in the PR description**

Everything here is verified against fakes. Eye-to-hand's math has still never run against a real RGBD camera, and eye-in-hand sits on top of it. Say so plainly in the PR rather than letting green CI imply otherwise.
