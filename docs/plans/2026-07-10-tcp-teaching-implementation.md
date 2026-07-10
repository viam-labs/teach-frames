# TCP Teaching Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Add TCP (tool center point) teaching to the `pose-tracker` component: a 4-point pivot method that least-squares-solves a tool's tip offset from arm flange poses, with optional explicit orientation, persisted to the tool component's `frame` config.

**Architecture:** A new pivot solver lives in the `frames/` package beside `compute.go`. The `pose-tracker` component gains an optional `arm` dependency; a new flange `PoseSource` reads `arm.EndPosition`. Captured flange poses accumulate in a dedicated TCP buffer in the `store`. New `DoCommand` verbs (`capture_tcp_point`, `teach_tcp_position`, `teach_tcp_orientation`, `get_tcp_buffer`, `clear_tcp_buffer`) drive the flow. Persistence is generalized so it can patch a *different* component's `frame` (translation and/or orientation) via the existing `GetRobotPart`/`UpdateRobotPart` machinery. TCP results never enter the tracked `frames` set or `Poses()`.

**Tech Stack:** Go, `go.viam.com/rdk` (v0.102.0), `gonum.org/v1/gonum/mat` (least squares), `github.com/golang/geo/r3`, `go.viam.com/test` (assertions).

**Design reference:** `docs/plans/2026-07-10-tcp-teaching-design.md`

---

## Conventions used in this plan

- Run tests for a single package with `go test ./<pkg>/... -run <Name> -v`.
- Poses are in **millimeters** (Viam convention); residuals are reported in mm.
- Follow the existing code's style: table-driven where the existing tests are, `go.viam.com/test` assertions (`test.That(t, got, test.ShouldEqual, want)`), doc comments on every exported symbol.
- Commit after each task with the message shown in that task's final step.

---

## Task 1: Pivot TCP solver in the `frames` package

**Files:**
- Create: `frames/tcp.go`
- Test: `frames/tcp_test.go`

**Background:** For each capture `i`, the flange pose has rotation `R_i` and translation `p_i` (flange origin in the arm base frame). The tool tip is a fixed point `t` in the flange frame, so its base-frame position `R_i·t + p_i` equals one common touched point `c` for every capture. Rearranged: `[R_i | -I]·[t; c] = -p_i`. Stacking all captures gives a `3N×6` system solved in the least-squares sense. The RMS of `‖R_i·t + p_i − c‖` over all captures is the fit residual (the "TCP error").

**Step 1: Write the failing test**

```go
// frames/tcp_test.go
package frames

import (
	"math"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
)

// makeFlangePose returns a flange pose (rotation about Z by degrees, at the given
// base-frame flange origin) such that a tool tip `tip` in the flange frame lands
// on the fixed base point `target`. It solves p = target - R*tip so every
// synthesized capture is consistent with one tip and one touched point.
func makeFlangePose(zDeg float64, tip, target r3.Vector) spatialmath.Pose {
	ov := &spatialmath.OrientationVectorDegrees{OZ: 1, Theta: zDeg}
	r := spatialmath.NewPose(r3.Vector{}, ov).Orientation().RotationMatrix()
	// R * tip
	col0, col1, col2 := r.Col(0), r.Col(1), r.Col(2)
	rTip := r3.Vector{
		X: col0.X*tip.X + col1.X*tip.Y + col2.X*tip.Z,
		Y: col0.Y*tip.X + col1.Y*tip.Y + col2.Y*tip.Z,
		Z: col0.Z*tip.X + col1.Z*tip.Y + col2.Z*tip.Z,
	}
	p := target.Sub(rTip)
	return spatialmath.NewPose(p, ov)
}

func TestComputePivotTCPRecoversTip(t *testing.T) {
	tip := r3.Vector{X: 10, Y: -5, Z: 120}
	target := r3.Vector{X: 400, Y: 0, Z: 300}
	poses := []spatialmath.Pose{
		makeFlangePose(0, tip, target),
		makeFlangePose(30, tip, target),
		makeFlangePose(75, tip, target),
		makeFlangePose(150, tip, target),
	}

	offset, residual, err := ComputePivotTCP(poses)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, spatialmath.R3VectorAlmostEqual(offset, tip, 1e-6), test.ShouldBeTrue)
	test.That(t, math.Abs(residual) < 1e-6, test.ShouldBeTrue)
}

func TestComputePivotTCPTooFewPoints(t *testing.T) {
	_, _, err := ComputePivotTCP(make([]spatialmath.Pose, 3))
	test.That(t, err, test.ShouldNotBeNil)
}
```

**Step 2: Run test to verify it fails**

Run: `go test ./frames/... -run TestComputePivotTCP -v`
Expected: FAIL — `undefined: ComputePivotTCP`.

**Step 3: Write minimal implementation**

```go
// frames/tcp.go

package frames

import (
	"errors"
	"fmt"
	"math"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
	"gonum.org/v1/gonum/mat"
)

// minPivotPoints is the smallest number of flange captures accepted by the
// 4-point pivot method. Four gives redundancy over the 6 unknowns and a
// meaningful RMS residual.
const minPivotPoints = 4

// ComputePivotTCP solves for a tool tip offset from a set of arm flange poses.
// Each pose places the (fixed) tip at one common point in the arm base frame, so
// it solves the least-squares system [R_i | -I]·[t; c] = -p_i, where t is the
// tip offset in the flange frame and c is the common touched point. It returns
// the tip offset and the RMS residual (mm) of how consistently the captures hit
// one point. Returns an error for fewer than minPivotPoints captures or a
// rank-deficient system (arm orientations too similar).
func ComputePivotTCP(poses []spatialmath.Pose) (r3.Vector, float64, error) {
	if len(poses) < minPivotPoints {
		return r3.Vector{}, 0, fmt.Errorf("pivot requires at least %d captures, have %d", minPivotPoints, len(poses))
	}

	n := len(poses)
	a := mat.NewDense(3*n, 6, nil)
	b := mat.NewVecDense(3*n, nil)

	for i, pose := range poses {
		rm := pose.Orientation().RotationMatrix()
		c0, c1, c2 := rm.Col(0), rm.Col(1), rm.Col(2)
		// Rows 3i..3i+2: [ R_i | -I ] and rhs -p_i.
		// R_i column vectors give entries: row r, col c.
		a.Set(3*i+0, 0, c0.X)
		a.Set(3*i+0, 1, c1.X)
		a.Set(3*i+0, 2, c2.X)
		a.Set(3*i+1, 0, c0.Y)
		a.Set(3*i+1, 1, c1.Y)
		a.Set(3*i+1, 2, c2.Y)
		a.Set(3*i+2, 0, c0.Z)
		a.Set(3*i+2, 1, c1.Z)
		a.Set(3*i+2, 2, c2.Z)
		a.Set(3*i+0, 3, -1)
		a.Set(3*i+1, 4, -1)
		a.Set(3*i+2, 5, -1)

		p := pose.Point()
		b.SetVec(3*i+0, -p.X)
		b.SetVec(3*i+1, -p.Y)
		b.SetVec(3*i+2, -p.Z)
	}

	var x mat.Dense
	if err := x.Solve(a, b); err != nil {
		// mat.Condition (ill-conditioned) surfaces here when orientations are
		// near-parallel and the system is effectively rank-deficient.
		return r3.Vector{}, 0, fmt.Errorf("pivot solve failed (vary arm orientations more): %w", err)
	}

	tip := r3.Vector{X: x.At(0, 0), Y: x.At(1, 0), Z: x.At(2, 0)}
	common := r3.Vector{X: x.At(3, 0), Y: x.At(4, 0), Z: x.At(5, 0)}

	// RMS residual of ||R_i*tip + p_i - common||.
	var sumSq float64
	for _, pose := range poses {
		hit := spatialmath.Compose(pose, spatialmath.NewPoseFromPoint(tip)).Point()
		d := hit.Sub(common)
		sumSq += d.Norm2()
	}
	residual := math.Sqrt(sumSq / float64(n))

	if math.IsNaN(residual) || math.IsInf(residual, 0) {
		return r3.Vector{}, 0, errors.New("pivot produced a non-finite result")
	}
	return tip, residual, nil
}
```

**Step 4: Run test to verify it passes**

Run: `go test ./frames/... -run TestComputePivotTCP -v`
Expected: PASS. If `gonum.org/v1/gonum/mat` is not resolvable, run `go get gonum.org/v1/gonum/mat` then `go mod tidy` and re-run.

**Note:** `spatialmath.Compose(pose, NewPoseFromPoint(tip)).Point()` computes `R_i·tip + p_i` using the SDK rather than re-deriving matrix math — prefer it over hand-rolled multiplication in the residual loop. Verify `spatialmath.NewPoseFromPoint` exists in v0.102.0; if not, use `spatialmath.NewPose(tip, nil)`.

**Step 5: Commit**

```bash
git add frames/tcp.go frames/tcp_test.go go.mod go.sum
git commit -m "feat: add pivot TCP solver"
```

---

## Task 2: Dedicated TCP capture buffer in the store

**Files:**
- Modify: `store/store.go`
- Test: `store/store_test.go`

**Background:** The existing `buffer []spatialmath.Pose` holds world TCP poses for frame teaching. TCP teaching needs a *separate* buffer of flange poses so the two flows never interfere. Mirror the existing buffer methods exactly.

**Step 1: Write the failing test**

```go
// append to store/store_test.go
func TestTCPBuffer(t *testing.T) {
	s := New()
	test.That(t, s.TCPBufferLen(), test.ShouldEqual, 0)

	p0 := spatialmath.NewZeroPose()
	idx := s.AddTCPCapture(p0)
	test.That(t, idx, test.ShouldEqual, 0)
	test.That(t, s.TCPBufferLen(), test.ShouldEqual, 1)

	s.AddTCPCapture(spatialmath.NewZeroPose())
	test.That(t, len(s.TCPBuffer()), test.ShouldEqual, 2)

	// TCP buffer is independent of the frame buffer.
	s.AddCapture(spatialmath.NewZeroPose())
	test.That(t, s.TCPBufferLen(), test.ShouldEqual, 2)
	test.That(t, s.BufferLen(), test.ShouldEqual, 1)

	test.That(t, s.ClearTCPBuffer(), test.ShouldEqual, 2)
	test.That(t, s.TCPBufferLen(), test.ShouldEqual, 0)
}
```

(Ensure `spatialmath` is imported in the test file; it already is via existing tests.)

**Step 2: Run test to verify it fails**

Run: `go test ./store/... -run TestTCPBuffer -v`
Expected: FAIL — `s.TCPBufferLen undefined`.

**Step 3: Write minimal implementation**

Add a `tcpBuffer []spatialmath.Pose` field to `FrameStore` and these methods (mirror the existing buffer methods, guarded by the same `s.mu`):

```go
// AddTCPCapture appends a flange pose to the TCP capture buffer and returns its
// zero-based index.
func (s *FrameStore) AddTCPCapture(p spatialmath.Pose) int {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.tcpBuffer = append(s.tcpBuffer, p)
	return len(s.tcpBuffer) - 1
}

// TCPBuffer returns a shallow copy of the current TCP capture buffer.
func (s *FrameStore) TCPBuffer() []spatialmath.Pose {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return append([]spatialmath.Pose(nil), s.tcpBuffer...)
}

// TCPBufferLen returns the number of poses in the TCP capture buffer.
func (s *FrameStore) TCPBufferLen() int {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return len(s.tcpBuffer)
}

// ClearTCPBuffer discards all buffered flange poses and returns the count removed.
func (s *FrameStore) ClearTCPBuffer() int {
	s.mu.Lock()
	defer s.mu.Unlock()
	n := len(s.tcpBuffer)
	s.tcpBuffer = nil
	return n
}
```

**Step 4: Run test to verify it passes**

Run: `go test ./store/... -run TestTCPBuffer -v`
Expected: PASS.

**Step 5: Commit**

```bash
git add store/store.go store/store_test.go
git commit -m "feat: add dedicated TCP capture buffer to store"
```

---

## Task 3: Flange pose source

**Files:**
- Modify: `posesource/posesource.go`
- Test: `posesource/posesource_test.go`

**Background:** TCP capture reads the flange pose from the arm via `arm.EndPosition`, a different source than the motion-service `MotionSource`. Add a small interface plus a real and a fake implementation, mirroring the `PoseSource`/`Fake` pattern already in this file.

**Step 1: Write the failing test**

```go
// append to posesource/posesource_test.go
func TestFakeFlangeSource(t *testing.T) {
	want := spatialmath.NewPoseFromPoint(r3.Vector{X: 1, Y: 2, Z: 3})
	f := &FakeFlange{Poses: []spatialmath.Pose{want}}

	got, err := f.CaptureFlange(context.Background())
	test.That(t, err, test.ShouldBeNil)
	test.That(t, spatialmath.PoseAlmostEqual(got, want), test.ShouldBeTrue)

	_, err = f.CaptureFlange(context.Background())
	test.That(t, err, test.ShouldNotBeNil) // exhausted
}
```

(Add imports `context`, `github.com/golang/geo/r3` if not already present in the test file.)

**Step 2: Run test to verify it fails**

Run: `go test ./posesource/... -run TestFakeFlangeSource -v`
Expected: FAIL — `undefined: FakeFlange`.

**Step 3: Write minimal implementation**

```go
// append to posesource/posesource.go

import (
	// add to the existing import block:
	"go.viam.com/rdk/components/arm"
)

// FlangeSource reads the arm flange pose (end position in the arm base frame).
type FlangeSource interface {
	CaptureFlange(ctx context.Context) (spatialmath.Pose, error)
}

// ArmSource wraps an arm's EndPosition call.
type ArmSource struct {
	Arm arm.Arm
}

// CaptureFlange returns the arm's current end position (flange pose in the arm
// base frame).
func (a *ArmSource) CaptureFlange(ctx context.Context) (spatialmath.Pose, error) {
	return a.Arm.EndPosition(ctx, nil)
}

// FakeFlange returns queued poses, popping one per call. For tests.
type FakeFlange struct {
	Poses []spatialmath.Pose
	idx   int
}

// CaptureFlange returns the next queued flange pose.
func (f *FakeFlange) CaptureFlange(_ context.Context) (spatialmath.Pose, error) {
	if f.idx >= len(f.Poses) {
		return nil, errors.New("fake: no more queued flange poses")
	}
	p := f.Poses[f.idx]
	f.idx++
	return p, nil
}
```

**Step 4: Run test to verify it passes**

Run: `go test ./posesource/... -run TestFakeFlangeSource -v`
Expected: PASS.

**Step 5: Commit**

```bash
git add posesource/posesource.go posesource/posesource_test.go
git commit -m "feat: add arm flange pose source"
```

---

## Task 4: Generalize persistence to patch a component's frame

**Files:**
- Modify: `persist/persist.go` (interface + `Fake`)
- Modify: `persist/appclient.go` (pure helper + `AppPersister` method)
- Test: `persist/appclient_test.go` (pure helper), `persist/persist_test.go` (Fake)

**Background:** Today `persist` only patches the module's own `attributes.frames`. TCP teaching writes translation and/or orientation into a *different* component's `frame`. Add a pure read-modify-write helper (mirroring `setComponentFrames`) plus interface/Fake/AppPersister methods. Translation and orientation are independently optional (position teaching sets translation only; orientation teaching sets orientation only).

**Frame JSON shape written:**
```json
"frame": {
  "parent": "<arm-name>",
  "translation": {"x": .., "y": .., "z": ..},
  "orientation": {"type": "ov_degrees", "value": {"x": .., "y": .., "z": .., "th": ..}}
}
```
Preserve any existing `frame` keys (e.g. `geometry`). Preserve an existing non-empty `parent`; otherwise set it to the provided `parent`.

**Step 1: Write the failing test (pure helper)**

```go
// append to persist/appclient_test.go
func TestSetComponentFrameTranslationAndOrientation(t *testing.T) {
	cfg := map[string]interface{}{
		"components": []interface{}{
			map[string]interface{}{"name": "tool", "attributes": map[string]interface{}{}},
		},
	}
	tr := &r3.Vector{X: 10, Y: -5, Z: 120}
	ov := &spatialmath.OrientationVectorDegrees{OZ: 1, Theta: 45}

	out, err := setComponentFrame(cfg, "tool", "my-arm", tr, ov)
	test.That(t, err, test.ShouldBeNil)

	comp := out["components"].([]interface{})[0].(map[string]interface{})
	frame := comp["frame"].(map[string]interface{})
	test.That(t, frame["parent"], test.ShouldEqual, "my-arm")
	transl := frame["translation"].(map[string]interface{})
	test.That(t, transl["x"], test.ShouldEqual, 10.0)
	test.That(t, transl["z"], test.ShouldEqual, 120.0)
	orient := frame["orientation"].(map[string]interface{})
	test.That(t, orient["type"], test.ShouldEqual, "ov_degrees")

	// Original config is not mutated.
	origComp := cfg["components"].([]interface{})[0].(map[string]interface{})
	_, hadFrame := origComp["frame"]
	test.That(t, hadFrame, test.ShouldBeFalse)
}

func TestSetComponentFrameMissingComponent(t *testing.T) {
	cfg := map[string]interface{}{"components": []interface{}{}}
	_, err := setComponentFrame(cfg, "nope", "arm", &r3.Vector{}, nil)
	test.That(t, err, test.ShouldNotBeNil)
}

func TestSetComponentFramePreservesExistingParentAndGeometry(t *testing.T) {
	cfg := map[string]interface{}{
		"components": []interface{}{
			map[string]interface{}{
				"name": "tool",
				"frame": map[string]interface{}{
					"parent":   "existing-parent",
					"geometry": map[string]interface{}{"type": "box"},
				},
			},
		},
	}
	out, err := setComponentFrame(cfg, "tool", "my-arm", &r3.Vector{X: 1}, nil)
	test.That(t, err, test.ShouldBeNil)
	frame := out["components"].([]interface{})[0].(map[string]interface{})["frame"].(map[string]interface{})
	test.That(t, frame["parent"], test.ShouldEqual, "existing-parent") // preserved
	test.That(t, frame["geometry"], test.ShouldNotBeNil)               // preserved
}
```

(Ensure imports `github.com/golang/geo/r3` and `go.viam.com/rdk/spatialmath` are present in the test file.)

**Step 2: Run test to verify it fails**

Run: `go test ./persist/... -run TestSetComponentFrame -v`
Expected: FAIL — `undefined: setComponentFrame`.

**Step 3: Write minimal implementation**

Add to `persist/appclient.go`:

```go
// setComponentFrame returns a copy of robotConfig with the named component's
// `frame.translation` and/or `frame.orientation` set. A nil translation or
// orientation leaves that sub-field untouched. Existing frame keys (geometry, a
// non-empty parent) are preserved; parent falls back to the provided value when
// absent or empty. Like setComponentFrames, it does not mutate the input.
func setComponentFrame(
	robotConfig map[string]interface{},
	componentName, parent string,
	translation *r3.Vector,
	orientation spatialmath.Orientation,
) (map[string]interface{}, error) {
	newConfig := make(map[string]interface{}, len(robotConfig))
	for k, v := range robotConfig {
		newConfig[k] = v
	}

	rawComponents, ok := newConfig["components"]
	if !ok {
		rawComponents = []interface{}{}
	}
	components, ok := rawComponents.([]interface{})
	if !ok {
		return nil, fmt.Errorf("robot config \"components\" is not a list")
	}

	found := false
	newComponents := make([]interface{}, len(components))
	for i, raw := range components {
		comp, ok := raw.(map[string]interface{})
		if !ok {
			newComponents[i] = raw
			continue
		}
		if name, _ := comp["name"].(string); name != componentName {
			newComponents[i] = comp
			continue
		}

		newComp := make(map[string]interface{}, len(comp)+1)
		for k, v := range comp {
			newComp[k] = v
		}

		frame := map[string]interface{}{}
		if rawFrame, ok := newComp["frame"].(map[string]interface{}); ok {
			for k, v := range rawFrame {
				frame[k] = v
			}
		}
		if existingParent, _ := frame["parent"].(string); existingParent == "" {
			frame["parent"] = parent
		}
		if translation != nil {
			frame["translation"] = map[string]interface{}{
				"x": translation.X, "y": translation.Y, "z": translation.Z,
			}
		}
		if orientation != nil {
			ov := orientation.OrientationVectorDegrees()
			frame["orientation"] = map[string]interface{}{
				"type": "ov_degrees",
				"value": map[string]interface{}{
					"x": ov.OX, "y": ov.OY, "z": ov.OZ, "th": ov.Theta,
				},
			}
		}
		newComp["frame"] = frame
		newComponents[i] = newComp
		found = true
	}
	if !found {
		return nil, fmt.Errorf("component %q not found in robot config", componentName)
	}

	newConfig["components"] = newComponents
	return newConfig, nil
}
```

Add the `AppPersister` method (mirrors `Save`):

```go
// SaveComponentFrame does a read-modify-write of the named component's frame
// (translation and/or orientation) in the robot part config. A nil translation
// or orientation leaves that sub-field unchanged. Needs a live cloud connection;
// covered by the manual integration checklist, not unit tests.
func (p *AppPersister) SaveComponentFrame(
	ctx context.Context,
	componentName, parent string,
	translation *r3.Vector,
	orientation spatialmath.Orientation,
) error {
	viamClient, err := app.CreateViamClientWithAPIKey(ctx, app.Options{}, p.apiKey, p.apiKeyID, p.logger)
	if err != nil {
		return fmt.Errorf("create viam client: %w", err)
	}
	defer viamClient.Close()

	appClient := viamClient.AppClient()
	part, _, err := appClient.GetRobotPart(ctx, p.partID)
	if err != nil {
		return fmt.Errorf("get robot part %s: %w", p.partID, err)
	}

	newConfig, err := setComponentFrame(part.RobotConfig, componentName, parent, translation, orientation)
	if err != nil {
		return fmt.Errorf("patch frame for component %q: %w", componentName, err)
	}
	if _, err := appClient.UpdateRobotPart(ctx, p.partID, part.Name, newConfig); err != nil {
		return fmt.Errorf("update robot part %s: %w", p.partID, err)
	}
	return nil
}
```

Add imports `github.com/golang/geo/r3` and `go.viam.com/rdk/spatialmath` to `appclient.go`.

Extend the interface and `Fake` in `persist/persist.go`:

```go
// ConfigPersister persists taught state durably.
type ConfigPersister interface {
	Save(ctx context.Context, frames []config.FrameSpec) error
	SaveComponentFrame(ctx context.Context, componentName, parent string, translation *r3.Vector, orientation spatialmath.Orientation) error
}

// add fields to Fake:
//   SavedComponent   string
//   SavedParent      string
//   SavedTranslation *r3.Vector
//   SavedOrientation spatialmath.Orientation
//   FrameErr         error

// SaveComponentFrame records the component-frame write (or returns FrameErr).
func (f *Fake) SaveComponentFrame(_ context.Context, componentName, parent string, translation *r3.Vector, orientation spatialmath.Orientation) error {
	if f.FrameErr != nil {
		return f.FrameErr
	}
	f.SavedComponent = componentName
	f.SavedParent = parent
	f.SavedTranslation = translation
	f.SavedOrientation = orientation
	return nil
}
```

Add imports `github.com/golang/geo/r3` and `go.viam.com/rdk/spatialmath` to `persist.go`.

**Step 4: Run test to verify it passes**

Run: `go test ./persist/... -v`
Expected: PASS (existing tests still green).

**Step 5: Commit**

```bash
git add persist/appclient.go persist/persist.go persist/appclient_test.go persist/persist_test.go
git commit -m "feat: persist arbitrary component frame translation/orientation"
```

---

## Task 5: Add the `arm` dependency and wire it into the component

**Files:**
- Modify: `models/posetracker/model.go`
- Test: `models/posetracker/model_test.go`

**Background:** Add an optional `arm` config attribute. When set, declare it as a dependency, resolve it in the constructor, and build a flange source. When absent, TCP teaching is disabled (commands error later). Store `tcpComponent` and `armName` on the struct for persistence targeting.

**Step 1: Write the failing test**

```go
// append to models/posetracker/model_test.go
func TestValidateDeclaresArmDependency(t *testing.T) {
	cfg := &Config{TCPComponent: "tool", Arm: "my-arm"}
	req, _, err := cfg.Validate("")
	test.That(t, err, test.ShouldBeNil)
	test.That(t, req, test.ShouldContain, "my-arm")
}

func TestValidateNoArmIsValid(t *testing.T) {
	cfg := &Config{TCPComponent: "tool"}
	req, _, err := cfg.Validate("")
	test.That(t, err, test.ShouldBeNil)
	// motion service dep present, arm absent.
	test.That(t, req, test.ShouldNotContain, "")
}
```

**Step 2: Run test to verify it fails**

Run: `go test ./models/posetracker/... -run TestValidate -v`
Expected: FAIL — `Config has no field Arm` (compile error).

**Step 3: Write minimal implementation**

In `model.go`:

1. Add the field to `Config`:
```go
	Arm string `json:"arm"`
```

2. In `Validate`, add the arm to required deps when set (it is a hard dependency only when configured):
```go
func (c *Config) Validate(path string) ([]string, []string, error) {
	if c.TCPComponent == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "tcp_component")
	}
	deps := []string{c.motionServiceOrDefault()}
	if c.Arm != "" {
		deps = append(deps, c.Arm)
	}
	return deps, nil, nil
}
```

3. Add fields to `teachTracker`:
```go
	flange       posesource.FlangeSource
	tcpComponent string
	armName      string
```

4. In `newPoseTracker`, after building `pt`, resolve the arm when configured (import `go.viam.com/rdk/components/arm`):
```go
	pt.tcpComponent = cfg.TCPComponent
	pt.armName = cfg.Arm
	if cfg.Arm != "" {
		a, aerr := arm.FromDependencies(deps, cfg.Arm)
		if aerr != nil {
			// Declared dependency should resolve; warn and leave TCP teaching disabled.
			logger.Warnw("TCP teaching disabled: arm dependency not resolvable", "arm", cfg.Arm, "err", aerr)
		} else {
			pt.flange = &posesource.ArmSource{Arm: a}
		}
	}
```

**Step 4: Run test to verify it passes**

Run: `go test ./models/posetracker/... -run TestValidate -v`
Expected: PASS.

**Step 5: Commit**

```bash
git add models/posetracker/model.go models/posetracker/model_test.go
git commit -m "feat: add optional arm dependency to pose-tracker"
```

---

## Task 6: `capture_tcp_point`, `get_tcp_buffer`, `clear_tcp_buffer` commands

**Files:**
- Modify: `models/posetracker/docommand.go`
- Test: `models/posetracker/docommand_test.go`

**Background:** These mirror the existing `capture_point`/`get_buffer`/`clear_buffer`, but read from `pt.flange` and the TCP buffer. `capture_tcp_point` errors clearly when no arm is configured.

**Step 1: Write the failing test**

First check how existing DoCommand tests build a `teachTracker` (fields, fakes) and follow that pattern. Then:

```go
// append to models/posetracker/docommand_test.go
func TestCaptureTCPPoint(t *testing.T) {
	pt := newTestTracker(t) // helper used by existing tests; adapt as needed
	pt.flange = &posesource.FakeFlange{Poses: []spatialmath.Pose{
		spatialmath.NewPoseFromPoint(r3.Vector{X: 1, Y: 2, Z: 3}),
	}}

	resp, err := pt.DoCommand(context.Background(), map[string]interface{}{"capture_tcp_point": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, resp["index"], test.ShouldEqual, 0)
	test.That(t, resp["buffer_len"], test.ShouldEqual, 1)
}

func TestCaptureTCPPointNoArm(t *testing.T) {
	pt := newTestTracker(t)
	pt.flange = nil
	_, err := pt.DoCommand(context.Background(), map[string]interface{}{"capture_tcp_point": map[string]interface{}{}})
	test.That(t, err, test.ShouldNotBeNil)
}
```

If no `newTestTracker` helper exists, construct a `&teachTracker{store: store.New(), logger: logging.NewTestLogger(t)}` directly, matching how the existing tests instantiate it.

**Step 2: Run test to verify it fails**

Run: `go test ./models/posetracker/... -run TestCaptureTCP -v`
Expected: FAIL — unknown command / nil deref.

**Step 3: Write minimal implementation**

Add cases to the `DoCommand` switch and update the doc comment to list the new verbs:

```go
	case has(cmd, "capture_tcp_point"):
		if pt.flange == nil {
			return nil, errors.New("arm dependency not configured; cannot capture TCP point")
		}
		pose, err := pt.flange.CaptureFlange(ctx)
		if err != nil {
			return nil, err
		}
		idx := pt.store.AddTCPCapture(pose)
		return map[string]interface{}{
			"index":      idx,
			"buffer_len": pt.store.TCPBufferLen(),
			"pose":       poseToMap(pose),
		}, nil

	case has(cmd, "get_tcp_buffer"):
		buf := pt.store.TCPBuffer()
		pts := make([]interface{}, len(buf))
		for i, p := range buf {
			pts[i] = poseToMap(p)
		}
		return map[string]interface{}{"points": pts}, nil

	case has(cmd, "clear_tcp_buffer"):
		return map[string]interface{}{"cleared": pt.store.ClearTCPBuffer()}, nil
```

**Step 4: Run test to verify it passes**

Run: `go test ./models/posetracker/... -run TestCaptureTCP -v`
Expected: PASS.

**Step 5: Commit**

```bash
git add models/posetracker/docommand.go models/posetracker/docommand_test.go
git commit -m "feat: add TCP capture-buffer commands"
```

---

## Task 7: `teach_tcp_position` command

**Files:**
- Modify: `models/posetracker/docommand.go`
- Test: `models/posetracker/docommand_test.go`

**Background:** Solve the pivot over the TCP buffer, persist the tip as `tcp_component.frame.translation` (parent = arm), clear the TCP buffer on success, and report the residual. Guard on `pt.persist == nil` like `defineFrame`. Serialize with `commitMu`.

**Step 1: Write the failing test**

```go
// append to models/posetracker/docommand_test.go
func TestTeachTCPPositionPersistsTranslation(t *testing.T) {
	pt := newTestTracker(t)
	fake := &persist.Fake{}
	pt.persist = fake
	pt.tcpComponent = "tool"
	pt.armName = "my-arm"

	tip := r3.Vector{X: 10, Y: -5, Z: 120}
	target := r3.Vector{X: 400, Y: 0, Z: 300}
	for _, zDeg := range []float64{0, 30, 75, 150} {
		pt.store.AddTCPCapture(makeFlangePoseForTest(zDeg, tip, target))
	}

	resp, err := pt.DoCommand(context.Background(), map[string]interface{}{"teach_tcp_position": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, resp["committed"], test.ShouldEqual, true)
	test.That(t, fake.SavedComponent, test.ShouldEqual, "tool")
	test.That(t, fake.SavedParent, test.ShouldEqual, "my-arm")
	test.That(t, fake.SavedTranslation, test.ShouldNotBeNil)
	test.That(t, pt.store.TCPBufferLen(), test.ShouldEqual, 0) // cleared on success
}

func TestTeachTCPPositionPersistenceDisabled(t *testing.T) {
	pt := newTestTracker(t)
	pt.persist = nil
	for _, zDeg := range []float64{0, 30, 75, 150} {
		pt.store.AddTCPCapture(spatialmath.NewZeroPose())
	}
	_, err := pt.DoCommand(context.Background(), map[string]interface{}{"teach_tcp_position": map[string]interface{}{}})
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, pt.store.TCPBufferLen(), test.ShouldEqual, 4) // buffer preserved on failure
}
```

Add a local `makeFlangePoseForTest` helper (copy of `makeFlangePose` from `frames/tcp_test.go`) to the test file, or export the helper from a shared test location. Keep it local to avoid cross-package test coupling.

**Step 2: Run test to verify it fails**

Run: `go test ./models/posetracker/... -run TestTeachTCPPosition -v`
Expected: FAIL — unknown command.

**Step 3: Write minimal implementation**

Add the dispatch case and method:

```go
	case has(cmd, "teach_tcp_position"):
		return pt.teachTCPPosition(ctx)
```

```go
// teachTCPPosition solves the pivot over the TCP buffer, persists the tool tip as
// the tcp_component's frame translation, and clears the buffer on success.
func (pt *teachTracker) teachTCPPosition(ctx context.Context) (map[string]interface{}, error) {
	if pt.persist == nil {
		return nil, errors.New("persistence disabled (missing platform-API env vars); cannot teach TCP")
	}
	buf := pt.store.TCPBuffer()
	offset, residual, err := frames.ComputePivotTCP(buf)
	if err != nil {
		return nil, fmt.Errorf("pivot solve failed: %w", err)
	}

	pt.commitMu.Lock()
	defer pt.commitMu.Unlock()

	off := offset // take address of a local
	if perr := pt.persist.SaveComponentFrame(ctx, pt.tcpComponent, pt.armName, &off, nil); perr != nil {
		return nil, fmt.Errorf("persist failed, TCP not committed: %w", perr)
	}

	pt.store.ClearTCPBuffer()
	return map[string]interface{}{
		"committed":    true,
		"offset":       map[string]interface{}{"x": offset.X, "y": offset.Y, "z": offset.Z},
		"residual_rms": residual,
	}, nil
}
```

**Step 4: Run test to verify it passes**

Run: `go test ./models/posetracker/... -run TestTeachTCPPosition -v`
Expected: PASS.

**Step 5: Commit**

```bash
git add models/posetracker/docommand.go models/posetracker/docommand_test.go
git commit -m "feat: add teach_tcp_position command"
```

---

## Task 8: `teach_tcp_orientation` command

**Files:**
- Modify: `models/posetracker/docommand.go`
- Test: `models/posetracker/docommand_test.go`

**Background:** Persist an explicit tool orientation (OV degrees) to `tcp_component.frame.orientation`, leaving the translation intact. Payload keys `o_x`, `o_y`, `o_z`, `theta` (all optional floats defaulting to 0; the OV-degrees identity is `{OZ:1, Theta:0}`, so require at least an approach axis — reject an all-zero orientation vector as ambiguous).

**Step 1: Write the failing test**

```go
// append to models/posetracker/docommand_test.go
func TestTeachTCPOrientationPersistsOrientation(t *testing.T) {
	pt := newTestTracker(t)
	fake := &persist.Fake{}
	pt.persist = fake
	pt.tcpComponent = "tool"
	pt.armName = "my-arm"

	resp, err := pt.DoCommand(context.Background(), map[string]interface{}{
		"teach_tcp_orientation": map[string]interface{}{"o_x": 0.0, "o_y": 0.0, "o_z": 1.0, "theta": 45.0},
	})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, resp["committed"], test.ShouldEqual, true)
	test.That(t, fake.SavedOrientation, test.ShouldNotBeNil)
	test.That(t, fake.SavedTranslation, test.ShouldBeNil) // translation untouched
}

func TestTeachTCPOrientationRejectsZeroVector(t *testing.T) {
	pt := newTestTracker(t)
	pt.persist = &persist.Fake{}
	_, err := pt.DoCommand(context.Background(), map[string]interface{}{
		"teach_tcp_orientation": map[string]interface{}{"o_x": 0.0, "o_y": 0.0, "o_z": 0.0, "theta": 0.0},
	})
	test.That(t, err, test.ShouldNotBeNil)
}
```

**Step 2: Run test to verify it fails**

Run: `go test ./models/posetracker/... -run TestTeachTCPOrientation -v`
Expected: FAIL — unknown command.

**Step 3: Write minimal implementation**

```go
	case has(cmd, "teach_tcp_orientation"):
		return pt.teachTCPOrientation(ctx, cmd["teach_tcp_orientation"])
```

```go
// teachTCPOrientation persists an explicit tool orientation (OV degrees) to the
// tcp_component's frame, leaving the taught translation intact. Tool orientation
// cannot be derived from tip touches, so it is supplied by the operator.
func (pt *teachTracker) teachTCPOrientation(ctx context.Context, raw interface{}) (map[string]interface{}, error) {
	args, ok := raw.(map[string]interface{})
	if !ok {
		return nil, errors.New("teach_tcp_orientation args must be an object")
	}
	if pt.persist == nil {
		return nil, errors.New("persistence disabled (missing platform-API env vars); cannot teach TCP orientation")
	}

	num := func(k string) float64 {
		if v, ok := args[k].(float64); ok {
			return v
		}
		return 0
	}
	ov := &spatialmath.OrientationVectorDegrees{OX: num("o_x"), OY: num("o_y"), OZ: num("o_z"), Theta: num("theta")}
	if ov.OX == 0 && ov.OY == 0 && ov.OZ == 0 {
		return nil, errors.New("orientation vector (o_x,o_y,o_z) must be non-zero")
	}

	pt.commitMu.Lock()
	defer pt.commitMu.Unlock()

	if perr := pt.persist.SaveComponentFrame(ctx, pt.tcpComponent, pt.armName, nil, ov); perr != nil {
		return nil, fmt.Errorf("persist failed, TCP orientation not committed: %w", perr)
	}
	return map[string]interface{}{
		"committed":   true,
		"orientation": map[string]interface{}{"o_x": ov.OX, "o_y": ov.OY, "o_z": ov.OZ, "theta": ov.Theta},
	}, nil
}
```

Add `go.viam.com/rdk/spatialmath` to the imports if not already present in `docommand.go` (it is).

**Step 4: Run test to verify it passes**

Run: `go test ./models/posetracker/... -run TestTeachTCPOrientation -v`
Expected: PASS.

**Step 5: Commit**

```bash
git add models/posetracker/docommand.go models/posetracker/docommand_test.go
git commit -m "feat: add teach_tcp_orientation command"
```

---

## Task 9: Full build, lint, docs, and manual integration checklist

**Files:**
- Modify: `README.md`
- Modify: `docs/plans/2026-07-10-tcp-teaching-design.md` (append a "verified" note after manual testing)

**Step 1: Full test + build**

Run:
```bash
go test ./... && make module
```
Expected: all packages PASS and the module binary builds. Fix any `go vet`/lint issues the Makefile surfaces.

**Step 2: Document the new commands and config in the README**

Add an `arm` row to the pose-tracker config table (optional; enables TCP teaching) and a new "TCP teaching" section to the DoCommand reference documenting `capture_tcp_point`, `teach_tcp_position`, `teach_tcp_orientation`, `get_tcp_buffer`, `clear_tcp_buffer`, plus a worked example:

```json
{"capture_tcp_point": {}}
{"capture_tcp_point": {}}
{"capture_tcp_point": {}}
{"capture_tcp_point": {}}
{"teach_tcp_position": {}}
{"teach_tcp_orientation": {"o_x": 0, "o_y": 0, "o_z": 1, "theta": 0}}
```

Document that `teach_tcp_position` writes `tcp_component.frame.translation` (parent = the configured `arm`), reports `residual_rms` (mm), and that orientation is optional and supplied explicitly.

**Step 3: Manual integration checklist (requires a real machine + cloud creds)**

This resolves the two open risks from the design doc. Perform on a robot with an arm and a tool component:

1. Configure the tool as `tcp_component`, the arm as `arm`. Jog the tip to a fixed point from ≥4 varied orientations, `capture_tcp_point` each, then `teach_tcp_position`. Confirm `residual_rms` is small (e.g. < 1–2 mm for careful touches).
2. **Frame-parent semantics (open risk):** After teaching, inspect the tool component's `frame` in the app config and verify in the visualizer that the tool tip now renders at the correct physical location. If the tip renders relative to the arm *base* instead of the *flange*, the solved offset needs transforming before persistence — record the correction and open a follow-up.
3. **Existing tool offset (open risk):** Repeat teaching on a tool that already has a non-zero `frame`. Confirm `arm.EndPosition` returns the flange pose *without* folding in the tool offset (i.e. re-teaching converges to the same physical tip, not a doubled offset). If it double-counts, document that the tool `frame` must be cleared before re-teaching.
4. `teach_tcp_orientation` with a known rotation; confirm it writes `frame.orientation` and leaves `translation` unchanged.

Record results in the design doc under a new "## Integration verification (YYYY-MM-DD)" heading.

**Step 4: Commit**

```bash
git add README.md docs/plans/2026-07-10-tcp-teaching-design.md
git commit -m "docs: document TCP teaching commands and integration checklist"
```

---

## Out of scope (deferred)

Per the design doc: additional workcell-geometry methods (plane/line/bore-center), the alignment-capture orientation method, the visual teach-pendant application, and freedrive / force-torque features. Do not implement these here.
```
