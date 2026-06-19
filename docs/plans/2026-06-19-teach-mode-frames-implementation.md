# Teach-Mode Frames Module — Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Build a Viam Go module that lets an operator define arbitrary named workcell frames by jogging a robot arm (3-point plane / TCP snapshot / single point), exposes them via `PoseTracker.Poses()`, persists them to config, and mirrors them to a `world_state_store` service for live visualization.

**Architecture:** Two registered resources in one module process. A **PoseTracker** component is the authoritative store: it handles the teach workflow via `DoCommand`, reads capture poses through the motion service's `GetPose`, and persists committed frames by patching its own component config via the Viam app API. A dependent **world_state_store** service mirrors the PoseTracker's frames as `Transform`s (with `showAxesHelper`) for the visualization tool. Both models use `AlwaysRebuild`, so config patches reload frames via the constructor — config is the single source of truth.

**Tech Stack:** Go, Viam RDK SDK (`go.viam.com/rdk`), Viam API protos (`go.viam.com/api`), `go.viam.com/api/app/v1` (config persistence), `github.com/google/uuid` (stable UUIDv5).

**Design reference:** `docs/plans/2026-06-19-teach-mode-frames-design.md`.

**Conventions for the whole plan:**
- Module path placeholder: `github.com/viam-labs/teach-frames` — change to the real org before Task 0.
- Model namespace placeholder: `viam-labs:teach-frames:*` — change to match.
- Run `go test ./... ` and `make lint` between tasks; commit at the end of each task.
- TDD: write the failing test first, watch it fail, implement minimally, watch it pass, commit.

---

### Task 0: Scaffold the module repo

**Files:**
- Create: `go.mod`, `Makefile`, `meta.json`, `cmd/module/main.go`, `.gitignore`

**Step 1: Initialize the Go module**

Run:
```bash
cd ~/src/teach-frames
go mod init github.com/viam-labs/teach-frames
go get go.viam.com/rdk@latest go.viam.com/api@latest github.com/google/uuid
```

**Step 2: Create `.gitignore`**

```
/bin/
/teach-frames
*.tar.gz
```

**Step 3: Create a minimal compiling `cmd/module/main.go`**

```go
// Package main is the entrypoint for the teach-frames module.
package main

import (
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
	"go.viam.com/utils"
)

func main() {
	utils.ContextualMain(mainWithArgs, module.NewLoggerFromArgs("teach-frames"))
}

func mainWithArgs(ctx context.Context, args []string, logger logging.Logger) error {
	m, err := module.NewModuleFromArgs(ctx)
	if err != nil {
		return err
	}
	// Models are added in Task 13.
	if err := m.Start(ctx); err != nil {
		return err
	}
	<-ctx.Done()
	return nil
}
```

> Note: imports (`context`, `go.viam.com/rdk/logging`) will be completed in Task 13 when models are registered. For now, get it compiling — you may temporarily stub `mainWithArgs` to just start an empty module.

**Step 4: Create `Makefile`**

```makefile
BIN = bin/teach-frames

build:
	go build -o $(BIN) ./cmd/module

test:
	go test ./...

lint:
	gofmt -l .
	go vet ./...

module.tar.gz: build
	tar czf module.tar.gz $(BIN) meta.json
```

**Step 5: Verify it builds**

Run: `make build`
Expected: compiles, produces `bin/teach-frames`.

**Step 6: Commit**

```bash
git add -A
git commit -m "chore: scaffold teach-frames Go module"
```

---

### Task 1: Frame math (pure functions)

**Files:**
- Create: `frames/compute.go`
- Test: `frames/compute_test.go`

This is the mathematical core: turn captured points into a `spatialmath.Pose`. Keep it pure (no Viam resources) so it's trivially testable.

**Step 1: Write the failing test**

```go
package frames

import (
	"math"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
)

func TestComputeThreePoint(t *testing.T) {
	// p1 origin, p2 on +X (1m), p3 on +Y (1m). Expect identity-ish orientation:
	// X axis -> world +X, Z axis -> world +Z.
	p1 := r3.Vector{X: 0, Y: 0, Z: 0}
	p2 := r3.Vector{X: 1, Y: 0, Z: 0}
	p3 := r3.Vector{X: 0, Y: 1, Z: 0}

	pose, err := ComputeThreePoint(p1, p2, p3)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, spatialmath.R3VectorAlmostEqual(pose.Point(), p1, 1e-6), test.ShouldBeTrue)

	// Orientation should map local X to world X.
	rm := pose.Orientation().RotationMatrix()
	xCol := rm.Col(0)
	test.That(t, math.Abs(xCol.X-1) < 1e-6, test.ShouldBeTrue)
}

func TestComputeThreePointRejectsCollinear(t *testing.T) {
	p1 := r3.Vector{X: 0, Y: 0, Z: 0}
	p2 := r3.Vector{X: 1, Y: 0, Z: 0}
	p3 := r3.Vector{X: 2, Y: 0, Z: 0} // collinear
	_, err := ComputeThreePoint(p1, p2, p3)
	test.That(t, err, test.ShouldNotBeNil)
}
```

**Step 2: Run it and watch it fail**

Run: `go test ./frames/ -run TestComputeThreePoint -v`
Expected: FAIL (`ComputeThreePoint` undefined).

**Step 3: Implement `frames/compute.go`**

```go
// Package frames computes spatial poses from captured arm positions.
package frames

import (
	"errors"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
)

const collinearEpsilon = 1e-9

// ComputeThreePoint builds a pose whose origin is p1, +X points toward p2, and
// p3 lies in the +XY half-plane (UR-style plane teaching). Returns an error if
// the three points are collinear or coincident.
func ComputeThreePoint(p1, p2, p3 r3.Vector) (spatialmath.Pose, error) {
	xAxis := p2.Sub(p1)
	if xAxis.Norm() < collinearEpsilon {
		return nil, errors.New("p1 and p2 are coincident; cannot define X axis")
	}
	xHat := xAxis.Normalize()

	inPlane := p3.Sub(p1)
	zAxis := xHat.Cross(inPlane)
	if zAxis.Norm() < collinearEpsilon {
		return nil, errors.New("points are collinear; cannot define a plane")
	}
	zHat := zAxis.Normalize()
	yHat := zHat.Cross(xHat) // right-handed

	rm, err := spatialmath.NewRotationMatrix([]float64{
		xHat.X, yHat.X, zHat.X,
		xHat.Y, yHat.Y, zHat.Y,
		xHat.Z, yHat.Z, zHat.Z,
	})
	if err != nil {
		return nil, err
	}
	return spatialmath.NewPose(p1, rm), nil
}

// ComputePoint builds a pose at p with identity orientation.
func ComputePoint(p r3.Vector) spatialmath.Pose {
	return spatialmath.NewPoseFromPoint(p)
}
```

> Verify `spatialmath.NewRotationMatrix` column/row ordering against the SDK version (the matrix above is column-major: columns are the basis vectors). If the SDK expects row-major, transpose. The `TestComputeThreePoint` assertion on `rm.Col(0)` will catch a wrong convention.

**Step 4: Run tests and watch them pass**

Run: `go test ./frames/ -v`
Expected: PASS.

**Step 5: Commit**

```bash
git add frames/
git commit -m "feat: add frame computation (3-point, point)"
```

---

### Task 2: Config frame type + (de)serialization

**Files:**
- Create: `config/frame.go`
- Test: `config/frame_test.go`

A `FrameSpec` is the JSON-serializable form stored in `attributes.frames` and converted to/from `spatialmath.Pose`.

**Step 1: Write the failing test**

```go
package config

import (
	"testing"

	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
)

func TestFrameSpecRoundTrip(t *testing.T) {
	pose := spatialmath.NewPose(
		spatialmath.NewPoseFromPoint(spatialmath.Vector{X: 0.1, Y: 0.2, Z: 0.3}).Point(),
		&spatialmath.OrientationVectorDegrees{OX: 0, OY: 0, OZ: 1, Theta: 35},
	)
	spec := FrameSpecFromPose("fixture_a", "3point", "world", pose)
	got, err := spec.ToPose()
	test.That(t, err, test.ShouldBeNil)
	test.That(t, spatialmath.PoseAlmostEqual(got, pose), test.ShouldBeTrue)
	test.That(t, spec.Name, test.ShouldEqual, "fixture_a")
}
```

**Step 2: Run and watch fail**

Run: `go test ./config/ -run TestFrameSpec -v`
Expected: FAIL (undefined).

**Step 3: Implement `config/frame.go`**

```go
// Package config holds the JSON-serializable config types for the module.
package config

import (
	"go.viam.com/rdk/spatialmath"
)

// PoseSpec is the JSON form of a pose (orientation-vector degrees).
type PoseSpec struct {
	X     float64 `json:"x"`
	Y     float64 `json:"y"`
	Z     float64 `json:"z"`
	OX    float64 `json:"o_x"`
	OY    float64 `json:"o_y"`
	OZ    float64 `json:"o_z"`
	Theta float64 `json:"theta"`
}

// FrameSpec is one taught frame as stored in config.
type FrameSpec struct {
	Name   string   `json:"name"`
	Method string   `json:"method"`
	Parent string   `json:"parent"`
	Pose   PoseSpec `json:"pose"`
}

// FrameSpecFromPose builds a FrameSpec from a spatialmath.Pose.
func FrameSpecFromPose(name, method, parent string, pose spatialmath.Pose) FrameSpec {
	pt := pose.Point()
	ov := pose.Orientation().OrientationVectorDegrees()
	return FrameSpec{
		Name: name, Method: method, Parent: parent,
		Pose: PoseSpec{X: pt.X, Y: pt.Y, Z: pt.Z, OX: ov.OX, OY: ov.OY, OZ: ov.OZ, Theta: ov.Theta},
	}
}

// ToPose converts a FrameSpec back to a spatialmath.Pose.
func (f FrameSpec) ToPose() (spatialmath.Pose, error) {
	ov := &spatialmath.OrientationVectorDegrees{OX: f.Pose.OX, OY: f.Pose.OY, OZ: f.Pose.OZ, Theta: f.Pose.Theta}
	return spatialmath.NewPose(spatialmath.Vector{X: f.Pose.X, Y: f.Pose.Y, Z: f.Pose.Z}, ov), nil
}
```

> Verify `spatialmath.Vector` vs `r3.Vector` usage against the SDK; `spatialmath.NewPose` takes an `r3.Vector`. Adjust imports/types so it compiles (likely `import "github.com/golang/geo/r3"` and use `r3.Vector`).

**Step 4: Run and watch pass**

Run: `go test ./config/ -v`
Expected: PASS.

**Step 5: Commit**

```bash
git add config/
git commit -m "feat: add FrameSpec config (de)serialization"
```

---

### Task 3: FrameStore (in-memory state + capture buffer)

**Files:**
- Create: `store/store.go`
- Test: `store/store_test.go`

Holds committed frames (`map[name]PoseInFrame`) and the capture buffer, mutex-guarded. Pure in-memory; no Viam deps beyond `referenceframe`/`spatialmath`.

**Step 1: Write the failing test**

```go
package store

import (
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
)

func TestBufferCaptureAndClear(t *testing.T) {
	s := New()
	s.AddCapture(spatialmath.NewPoseFromPoint(r3.Vector{X: 1}))
	s.AddCapture(spatialmath.NewPoseFromPoint(r3.Vector{X: 2}))
	test.That(t, s.BufferLen(), test.ShouldEqual, 2)
	s.ClearBuffer()
	test.That(t, s.BufferLen(), test.ShouldEqual, 0)
}

func TestSetAndListFrames(t *testing.T) {
	s := New()
	s.SetFrame("a", "world", spatialmath.NewPoseFromPoint(r3.Vector{X: 1}))
	s.SetFrame("b", "world", spatialmath.NewPoseFromPoint(r3.Vector{X: 2}))
	names := s.FrameNames()
	test.That(t, len(names), test.ShouldEqual, 2)
	_, ok := s.GetFrame("a")
	test.That(t, ok, test.ShouldBeTrue)
	test.That(t, s.DeleteFrame("a"), test.ShouldBeTrue)
	_, ok = s.GetFrame("a")
	test.That(t, ok, test.ShouldBeFalse)
}
```

**Step 2: Run and watch fail**

Run: `go test ./store/ -v`
Expected: FAIL (undefined).

**Step 3: Implement `store/store.go`**

```go
// Package store holds the in-memory taught-frame state and capture buffer.
package store

import (
	"sort"
	"sync"

	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

// FrameStore is the authoritative in-memory state, guarded by a mutex.
type FrameStore struct {
	mu     sync.RWMutex
	frames map[string]*referenceframe.PoseInFrame
	buffer []spatialmath.Pose
}

// New creates an empty FrameStore.
func New() *FrameStore {
	return &FrameStore{frames: map[string]*referenceframe.PoseInFrame{}}
}

// AddCapture appends a captured pose to the buffer and returns its index.
func (s *FrameStore) AddCapture(p spatialmath.Pose) int {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.buffer = append(s.buffer, p)
	return len(s.buffer) - 1
}

// Buffer returns a copy of the capture buffer.
func (s *FrameStore) Buffer() []spatialmath.Pose {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return append([]spatialmath.Pose(nil), s.buffer...)
}

// BufferLen returns the number of buffered captures.
func (s *FrameStore) BufferLen() int {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return len(s.buffer)
}

// ClearBuffer discards buffered captures and returns how many were cleared.
func (s *FrameStore) ClearBuffer() int {
	s.mu.Lock()
	defer s.mu.Unlock()
	n := len(s.buffer)
	s.buffer = nil
	return n
}

// SetFrame upserts a committed frame. Returns true if it replaced an existing one.
func (s *FrameStore) SetFrame(name, parent string, pose spatialmath.Pose) bool {
	s.mu.Lock()
	defer s.mu.Unlock()
	_, existed := s.frames[name]
	s.frames[name] = referenceframe.NewPoseInFrame(parent, pose)
	return existed
}

// GetFrame returns a committed frame by name.
func (s *FrameStore) GetFrame(name string) (*referenceframe.PoseInFrame, bool) {
	s.mu.RLock()
	defer s.mu.RUnlock()
	f, ok := s.frames[name]
	return f, ok
}

// DeleteFrame removes a frame; returns true if it existed.
func (s *FrameStore) DeleteFrame(name string) bool {
	s.mu.Lock()
	defer s.mu.Unlock()
	_, ok := s.frames[name]
	delete(s.frames, name)
	return ok
}

// ClearFrames removes all frames and returns how many were removed.
func (s *FrameStore) ClearFrames() int {
	s.mu.Lock()
	defer s.mu.Unlock()
	n := len(s.frames)
	s.frames = map[string]*referenceframe.PoseInFrame{}
	return n
}

// FrameNames returns sorted committed frame names.
func (s *FrameStore) FrameNames() []string {
	s.mu.RLock()
	defer s.mu.RUnlock()
	out := make([]string, 0, len(s.frames))
	for n := range s.frames {
		out = append(out, n)
	}
	sort.Strings(out)
	return out
}

// Snapshot returns a copy of all committed frames, filtered by names if provided.
func (s *FrameStore) Snapshot(names []string) referenceframe.FrameSystemPoses {
	s.mu.RLock()
	defer s.mu.RUnlock()
	out := referenceframe.FrameSystemPoses{}
	if len(names) == 0 {
		for n, f := range s.frames {
			out[n] = f
		}
		return out
	}
	for _, n := range names {
		if f, ok := s.frames[n]; ok {
			out[n] = f
		}
	}
	return out
}
```

**Step 4: Run and watch pass**

Run: `go test ./store/ -v`
Expected: PASS.

**Step 5: Commit**

```bash
git add store/
git commit -m "feat: add in-memory FrameStore with capture buffer"
```

---

### Task 4: PoseSource interface (motion wrapper + fake)

**Files:**
- Create: `posesource/posesource.go`
- Test: `posesource/posesource_test.go`

Abstracts "read the TCP pose" so teach logic is testable without a robot.

**Step 1: Write the failing test (fake satisfies the interface)**

```go
package posesource

import (
	"context"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
)

func TestFakeSourceReturnsQueuedPoses(t *testing.T) {
	f := &Fake{Poses: []spatialmath.Pose{spatialmath.NewPoseFromPoint(r3.Vector{X: 1})}}
	pif, err := f.Capture(context.Background())
	test.That(t, err, test.ShouldBeNil)
	test.That(t, pif.Parent(), test.ShouldEqual, referenceframe.World)
}
```

**Step 2: Run and watch fail**

Run: `go test ./posesource/ -v`
Expected: FAIL.

**Step 3: Implement `posesource/posesource.go`**

```go
// Package posesource abstracts reading the TCP pose for capture.
package posesource

import (
	"context"
	"errors"

	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
)

// PoseSource reads the current TCP pose in the destination frame.
type PoseSource interface {
	Capture(ctx context.Context) (*referenceframe.PoseInFrame, error)
}

// MotionSource wraps a motion service GetPose call.
type MotionSource struct {
	Motion        motion.Service
	Component     string
	DestФrame     string
}

// Capture queries motion.GetPose for the configured component/destination.
func (m *MotionSource) Capture(ctx context.Context) (*referenceframe.PoseInFrame, error) {
	return m.Motion.GetPose(ctx, m.Component, m.DestФrame, nil, nil)
}

// Fake returns queued poses, popping one per Capture. For tests.
type Fake struct {
	Poses []spatialmath.Pose
	idx   int
}

// Capture returns the next queued pose as a world-framed PoseInFrame.
func (f *Fake) Capture(_ context.Context) (*referenceframe.PoseInFrame, error) {
	if f.idx >= len(f.Poses) {
		return nil, errors.New("fake: no more queued poses")
	}
	p := f.Poses[f.idx]
	f.idx++
	return referenceframe.NewPoseInFrame(referenceframe.World, p), nil
}
```

> Fix the typo'd field names (`DestФrame` → `DestFrame`) — they're mangled here intentionally so you notice and rename consistently. Confirm `referenceframe.World` is the exported "world" frame constant in the SDK; if not, use the literal `"world"`.

**Step 4: Run and watch pass**

Run: `go test ./posesource/ -v`
Expected: PASS.

**Step 5: Commit**

```bash
git add posesource/
git commit -m "feat: add PoseSource interface (motion impl + fake)"
```

---

### Task 5: ConfigPersister interface (app API + fake)

**Files:**
- Create: `persist/persist.go`
- Test: `persist/persist_test.go`

Abstracts "write the frames back to component config" so define/delete/clear are testable, and the app-API connection detail is isolated in one place.

**Step 1: Write the failing test (fake records the saved set)**

```go
package persist

import (
	"context"
	"testing"

	"github.com/viam-labs/teach-frames/config"
	"go.viam.com/test"
)

func TestFakePersisterRecordsFrames(t *testing.T) {
	f := &Fake{}
	frames := []config.FrameSpec{{Name: "a"}, {Name: "b"}}
	err := f.Save(context.Background(), frames)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(f.Saved), test.ShouldEqual, 2)
}
```

**Step 2: Run and watch fail**

Run: `go test ./persist/ -v`
Expected: FAIL.

**Step 3: Implement `persist/persist.go`**

```go
// Package persist writes taught frames back to the module's component config.
package persist

import (
	"context"

	"github.com/viam-labs/teach-frames/config"
)

// ConfigPersister persists the full set of taught frames durably.
type ConfigPersister interface {
	Save(ctx context.Context, frames []config.FrameSpec) error
}

// Fake records the last saved set. For tests.
type Fake struct {
	Saved []config.FrameSpec
	Err   error
}

// Save records frames (or returns the configured error).
func (f *Fake) Save(_ context.Context, frames []config.FrameSpec) error {
	if f.Err != nil {
		return f.Err
	}
	f.Saved = append([]config.FrameSpec(nil), frames...)
	return nil
}
```

**Step 4: Run and watch pass**

Run: `go test ./persist/ -v`
Expected: PASS.

**Step 5: Implement the real app-API persister (no unit test; integration-only)**

Create `persist/appclient.go`:

```go
package persist

import (
	"context"
	"fmt"
	"os"

	"github.com/viam-labs/teach-frames/config"
)

// AppPersister patches this component's `attributes.frames` in the robot part
// config via the Viam app API, authenticating with module-injected env vars.
type AppPersister struct {
	ComponentName string
	// connection params read from env at construction:
	apiKey   string
	apiKeyID string
	partID   string
}

// NewAppPersister reads platform-API credentials from the standard module env vars.
// See https://docs.viam.com/build-modules/platform-apis/ for the env var names and
// the SDK helper used to build the app client. Returns an error if creds are absent.
func NewAppPersister(componentName string) (*AppPersister, error) {
	p := &AppPersister{
		ComponentName: componentName,
		apiKey:        os.Getenv("VIAM_API_KEY"),
		apiKeyID:      os.Getenv("VIAM_API_KEY_ID"),
		partID:        os.Getenv("VIAM_MACHINE_PART_ID"),
	}
	if p.apiKey == "" || p.apiKeyID == "" || p.partID == "" {
		return nil, fmt.Errorf("missing platform-API env vars (VIAM_API_KEY/_ID, VIAM_MACHINE_PART_ID); "+
			"cannot persist frames for %s", componentName)
	}
	return p, nil
}

// Save does a read-modify-write of the robot part config:
//  1. app.GetRobotPart(partID) -> current robot config
//  2. locate the component entry whose name == ComponentName
//  3. set its attributes.frames = frames
//  4. app.UpdateRobotPart(partID, updatedConfig)
//
// IMPLEMENTATION NOTE: build the app client from the env creds using the Viam Go
// SDK helper (see platform-apis docs / `app` package). GetRobotPart returns the
// part with a `RobotConfig` *structpb.Struct; mutate only this component's
// `attributes.frames` within it and pass it back to UpdateRobotPart. Verify the
// exact app client constructor + request/response shapes against
// go.viam.com/api@<version>/app/v1 before writing this.
func (p *AppPersister) Save(ctx context.Context, frames []config.FrameSpec) error {
	return fmt.Errorf("AppPersister.Save not yet implemented")
}
```

> This real impl is intentionally left as a stub with a precise recipe — it requires a live cloud connection to test, so it's covered by the **manual integration checklist** (Task 14), not unit tests. Implement the read-modify-write against the actual `app/v1` protos. Mutate **only** this component's `attributes.frames` inside the fetched part config to minimize clobber risk.

**Step 6: Commit**

```bash
git add persist/
git commit -m "feat: add ConfigPersister (fake + app-API skeleton)"
```

---

### Task 6: PoseTracker model — registration, config, constructor, Poses()

**Files:**
- Create: `models/posetracker/model.go`
- Test: `models/posetracker/model_test.go`

**Step 1: Write the failing test (constructor loads frames; Poses filters)**

```go
package posetracker

import (
	"context"
	"testing"

	"github.com/viam-labs/teach-frames/config"
	"go.viam.com/test"
)

func TestPosesLoadsConfiguredFrames(t *testing.T) {
	cfg := &Config{
		MotionService:    "builtin",
		TCPComponent:     "arm",
		DestinationFrame: "world",
		Frames: []config.FrameSpec{
			{Name: "a", Parent: "world", Pose: config.PoseSpec{X: 1, OZ: 1, Theta: 0}},
		},
	}
	pt := newForTest(t, cfg)
	got, err := pt.Poses(context.Background(), nil, nil)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(got), test.ShouldEqual, 1)
	_, ok := got["a"]
	test.That(t, ok, test.ShouldBeTrue)

	// filter by a non-existent name -> empty
	got, err = pt.Poses(context.Background(), []string{"zzz"}, nil)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(got), test.ShouldEqual, 0)
}
```

**Step 2: Run and watch fail**

Run: `go test ./models/posetracker/ -v`
Expected: FAIL.

**Step 3: Implement `models/posetracker/model.go` (config + constructor + Poses + test helper)**

```go
// Package posetracker implements the teach-mode PoseTracker model.
package posetracker

import (
	"context"
	"testing"

	"github.com/viam-labs/teach-frames/config"
	"github.com/viam-labs/teach-frames/persist"
	"github.com/viam-labs/teach-frames/posesource"
	"github.com/viam-labs/teach-frames/store"
	"go.viam.com/rdk/components/posetracker"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/motion"
)

// Model is the resource model triple. CHANGE the namespace/family.
var Model = resource.NewModel("viam-labs", "teach-frames", "pose-tracker")

// Config is the PoseTracker config.
type Config struct {
	MotionService    string             `json:"motion_service"`
	TCPComponent     string             `json:"tcp_component"`
	DestinationFrame string             `json:"destination_frame"`
	Frames           []config.FrameSpec `json:"frames"`
}

// Validate checks required fields and returns the motion service as a dependency.
func (c *Config) Validate(path string) ([]string, error) {
	if c.MotionService == "" {
		return nil, resource.NewConfigValidationFieldRequiredError(path, "motion_service")
	}
	if c.TCPComponent == "" {
		return nil, resource.NewConfigValidationFieldRequiredError(path, "tcp_component")
	}
	return []string{c.MotionService}, nil
}

func init() {
	resource.RegisterComponent(posetracker.API, Model,
		resource.Registration[posetracker.PoseTracker, *Config]{Constructor: newPoseTracker})
}

type teachTracker struct {
	resource.Named
	resource.AlwaysRebuild // every config change rebuilds; constructor reloads frames
	logger logging.Logger

	store   *store.FrameStore
	source  posesource.PoseSource
	persist persist.ConfigPersister
	destFrame string
}

func newPoseTracker(
	ctx context.Context, deps resource.Dependencies, conf resource.Config, logger logging.Logger,
) (posetracker.PoseTracker, error) {
	cfg, err := resource.NativeConfig[*Config](conf)
	if err != nil {
		return nil, err
	}
	mot, err := motion.FromDependencies(deps, cfg.MotionService)
	if err != nil {
		return nil, err
	}
	dest := cfg.DestinationFrame
	if dest == "" {
		dest = referenceframe.World
	}
	pt := &teachTracker{
		Named:     conf.ResourceName().AsNamed(),
		logger:    logger,
		store:     store.New(),
		source:    &posesource.MotionSource{Motion: mot, Component: cfg.TCPComponent, DestFrame: dest},
		destFrame: dest,
	}
	// Persistence: app API. Warn (don't fail) if creds absent — Poses/capture still work.
	if p, perr := persist.NewAppPersister(conf.ResourceName().Name); perr != nil {
		logger.Warnw("frame persistence disabled", "reason", perr)
	} else {
		pt.persist = p
	}
	// Load committed frames from config (single source of truth).
	for _, fs := range cfg.Frames {
		pose, perr := fs.ToPose()
		if perr != nil {
			return nil, perr
		}
		parent := fs.Parent
		if parent == "" {
			parent = dest
		}
		pt.store.SetFrame(fs.Name, parent, pose)
	}
	return pt, nil
}

// Poses returns committed taught frames, filtered by bodyNames if provided.
func (pt *teachTracker) Poses(
	_ context.Context, bodyNames []string, _ map[string]interface{},
) (referenceframe.FrameSystemPoses, error) {
	return pt.store.Snapshot(bodyNames), nil
}

func (pt *teachTracker) Close(_ context.Context) error { return nil }

// newForTest builds a tracker with injected fakes (used by tests).
func newForTest(t *testing.T, cfg *Config) *teachTracker {
	t.Helper()
	pt := &teachTracker{
		logger:    logging.NewTestLogger(t),
		store:     store.New(),
		source:    &posesource.Fake{},
		persist:   &persist.Fake{},
		destFrame: cfg.DestinationFrame,
	}
	for _, fs := range cfg.Frames {
		pose, err := fs.ToPose()
		test.That(t, err, test.ShouldBeNil)
		pt.store.SetFrame(fs.Name, fs.Parent, pose)
	}
	return pt
}
```

> Verify SDK helpers: `resource.NativeConfig`, `motion.FromDependencies`, `resource.NewConfigValidationFieldRequiredError`, `resource.NewModel`, and that embedding `resource.AlwaysRebuild` provides the `Reconfigure` method (confirmed in rdk `resource/resource.go:282`). The `newForTest` helper importing `testing` in a non-`_test.go` file is acceptable here but consider moving it to `model_test.go` to keep `testing` out of the build graph.

**Step 4: Run and watch pass**

Run: `go test ./models/posetracker/ -v`
Expected: PASS.

**Step 5: Commit**

```bash
git add models/posetracker/
git commit -m "feat: PoseTracker model registration, config load, Poses()"
```

---

### Task 7: DoCommand — capture_point / get_buffer / clear_buffer

**Files:**
- Modify: `models/posetracker/model.go` (add `DoCommand`)
- Create/extend test: `models/posetracker/docommand_test.go`

**Step 1: Write the failing test**

```go
func TestCapturePointBuffers(t *testing.T) {
	pt := newForTest(t, &Config{DestinationFrame: "world"})
	pt.source = &posesource.Fake{Poses: []spatialmath.Pose{
		spatialmath.NewPoseFromPoint(r3.Vector{X: 1}),
	}}
	resp, err := pt.DoCommand(context.Background(), map[string]interface{}{"capture_point": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, resp["buffer_len"], test.ShouldEqual, 1)
}
```

**Step 2: Run and watch fail** — `go test ./models/posetracker/ -run TestCapturePoint -v` → FAIL (DoCommand undefined).

**Step 3: Implement the `DoCommand` dispatcher with the three buffer verbs**

```go
func (pt *teachTracker) DoCommand(
	ctx context.Context, cmd map[string]interface{},
) (map[string]interface{}, error) {
	switch {
	case has(cmd, "capture_point"):
		pif, err := pt.source.Capture(ctx)
		if err != nil {
			return nil, err
		}
		idx := pt.store.AddCapture(pif.Pose())
		return map[string]interface{}{"index": idx, "buffer_len": pt.store.BufferLen(),
			"pose": poseToMap(pif.Pose())}, nil
	case has(cmd, "clear_buffer"):
		return map[string]interface{}{"cleared": pt.store.ClearBuffer()}, nil
	case has(cmd, "get_buffer"):
		buf := pt.store.Buffer()
		pts := make([]interface{}, len(buf))
		for i, p := range buf {
			pts[i] = poseToMap(p)
		}
		return map[string]interface{}{"points": pts}, nil
	}
	return nil, fmt.Errorf("unknown command: %v", keysOf(cmd))
}

func has(m map[string]interface{}, k string) bool { _, ok := m[k]; return ok }
func keysOf(m map[string]interface{}) []string {
	ks := make([]string, 0, len(m))
	for k := range m {
		ks = append(ks, k)
	}
	return ks
}
func poseToMap(p spatialmath.Pose) map[string]interface{} {
	pt := p.Point()
	ov := p.Orientation().OrientationVectorDegrees()
	return map[string]interface{}{"x": pt.X, "y": pt.Y, "z": pt.Z,
		"o_x": ov.OX, "o_y": ov.OY, "o_z": ov.OZ, "theta": ov.Theta}
}
```

Add imports: `fmt`, `github.com/golang/geo/r3` (tests), `go.viam.com/rdk/spatialmath`.

**Step 4: Run and watch pass** — `go test ./models/posetracker/ -v` → PASS.

**Step 5: Commit**

```bash
git add models/posetracker/
git commit -m "feat: DoCommand capture_point/get_buffer/clear_buffer"
```

---

### Task 8: DoCommand — define_frame

**Files:**
- Modify: `models/posetracker/model.go`
- Test: `models/posetracker/docommand_test.go`

**Step 1: Write the failing test (3-point happy path + insufficient buffer)**

```go
func TestDefineFrameThreePoint(t *testing.T) {
	pt := newForTest(t, &Config{DestinationFrame: "world"})
	fake := &persist.Fake{}
	pt.persist = fake
	pt.source = &posesource.Fake{Poses: []spatialmath.Pose{
		spatialmath.NewPoseFromPoint(r3.Vector{X: 0, Y: 0, Z: 0}),
		spatialmath.NewPoseFromPoint(r3.Vector{X: 1, Y: 0, Z: 0}),
		spatialmath.NewPoseFromPoint(r3.Vector{X: 0, Y: 1, Z: 0}),
	}}
	for i := 0; i < 3; i++ {
		_, err := pt.DoCommand(context.Background(), map[string]interface{}{"capture_point": map[string]interface{}{}})
		test.That(t, err, test.ShouldBeNil)
	}
	resp, err := pt.DoCommand(context.Background(), map[string]interface{}{
		"define_frame": map[string]interface{}{"name": "fixture_a", "method": "3point"}})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, resp["committed"], test.ShouldBeTrue)
	test.That(t, len(fake.Saved), test.ShouldEqual, 1)
	test.That(t, pt.store.BufferLen(), test.ShouldEqual, 0) // buffer cleared
}

func TestDefineFrameInsufficientBuffer(t *testing.T) {
	pt := newForTest(t, &Config{DestinationFrame: "world"})
	pt.persist = &persist.Fake{}
	_, err := pt.DoCommand(context.Background(), map[string]interface{}{
		"define_frame": map[string]interface{}{"name": "x", "method": "3point"}})
	test.That(t, err, test.ShouldNotBeNil)
}
```

**Step 2: Run and watch fail** — FAIL.

**Step 3: Implement the `define_frame` case + helper**

Add to the `switch` in `DoCommand`:

```go
	case has(cmd, "define_frame"):
		return pt.defineFrame(ctx, cmd["define_frame"])
```

And the method:

```go
func (pt *teachTracker) defineFrame(ctx context.Context, raw interface{}) (map[string]interface{}, error) {
	args, _ := raw.(map[string]interface{})
	name, _ := args["name"].(string)
	method, _ := args["method"].(string)
	if name == "" {
		return nil, errors.New("define_frame requires a non-empty name")
	}
	if pt.persist == nil {
		return nil, errors.New("persistence disabled (missing platform-API env vars); cannot commit frame")
	}

	buf := pt.store.Buffer()
	var pose spatialmath.Pose
	var err error
	switch method {
	case "3point":
		if len(buf) < 3 {
			return nil, fmt.Errorf("3point requires 3 captured points, have %d", len(buf))
		}
		pose, err = frames.ComputeThreePoint(buf[0].Point(), buf[1].Point(), buf[2].Point())
	case "point":
		if len(buf) < 1 {
			return nil, fmt.Errorf("point requires 1 captured point, have %d", len(buf))
		}
		pose = frames.ComputePoint(buf[0].Point())
	case "tcp_snapshot":
		if len(buf) < 1 {
			return nil, fmt.Errorf("tcp_snapshot requires 1 captured point, have %d", len(buf))
		}
		pose = buf[0]
	default:
		return nil, fmt.Errorf("unknown method %q (want 3point|point|tcp_snapshot)", method)
	}
	if err != nil {
		return nil, err
	}

	replaced := pt.store.SetFrame(name, pt.destFrame, pose)
	if perr := pt.saveFrames(ctx); perr != nil {
		// roll back the optimistic in-memory write so state matches durable config
		pt.store.DeleteFrame(name)
		return nil, fmt.Errorf("persist failed, frame not committed: %w", perr)
	}
	pt.store.ClearBuffer()
	return map[string]interface{}{"name": name, "committed": true, "replaced": replaced,
		"pose": poseToMap(pose)}, nil
}

// saveFrames serializes the full committed set and persists it.
func (pt *teachTracker) saveFrames(ctx context.Context) error {
	names := pt.store.FrameNames()
	specs := make([]config.FrameSpec, 0, len(names))
	for _, n := range names {
		pif, _ := pt.store.GetFrame(n)
		specs = append(specs, config.FrameSpecFromPose(n, "", pif.Parent(), pif.Pose()))
	}
	return pt.persist.Save(ctx, specs)
}
```

Add imports: `errors`, `github.com/viam-labs/teach-frames/frames`, `github.com/viam-labs/teach-frames/config`.

> Note on method tracking: `FrameSpecFromPose` is called with `""` method here because the store doesn't track the method that produced a frame. If you want to preserve `method` in config, extend `FrameStore` to store it alongside the pose. YAGNI for v1 unless you need it for re-teaching UX.

**Step 4: Run and watch pass** — `go test ./models/posetracker/ -v` → PASS.

**Step 5: Commit**

```bash
git add models/posetracker/ ; git commit -m "feat: DoCommand define_frame (3point/point/tcp_snapshot + persist)"
```

---

### Task 9: DoCommand — list_frames / delete_frame / clear_frames

**Files:**
- Modify: `models/posetracker/model.go`
- Test: `models/posetracker/docommand_test.go`

**Step 1: Write the failing test**

```go
func TestListDeleteClearFrames(t *testing.T) {
	pt := newForTest(t, &Config{DestinationFrame: "world",
		Frames: []config.FrameSpec{{Name: "a", Parent: "world", Pose: config.PoseSpec{OZ: 1}}}})
	pt.persist = &persist.Fake{}

	resp, err := pt.DoCommand(context.Background(), map[string]interface{}{"list_frames": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
	frames := resp["frames"].(map[string]interface{})
	test.That(t, len(frames), test.ShouldEqual, 1)

	resp, err = pt.DoCommand(context.Background(), map[string]interface{}{
		"delete_frame": map[string]interface{}{"name": "a"}})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, resp["deleted"], test.ShouldBeTrue)

	resp, err = pt.DoCommand(context.Background(), map[string]interface{}{"clear_frames": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, resp["deleted"], test.ShouldEqual, 0)
}
```

**Step 2: Run and watch fail** — FAIL.

**Step 3: Implement the three cases**

```go
	case has(cmd, "list_frames"):
		out := map[string]interface{}{}
		for n, pif := range pt.store.Snapshot(nil) {
			out[n] = poseToMap(pif.Pose())
		}
		return map[string]interface{}{"frames": out}, nil
	case has(cmd, "delete_frame"):
		args, _ := cmd["delete_frame"].(map[string]interface{})
		name, _ := args["name"].(string)
		if pt.persist == nil {
			return nil, errors.New("persistence disabled; cannot delete")
		}
		existed := pt.store.DeleteFrame(name)
		if err := pt.saveFrames(ctx); err != nil {
			return nil, fmt.Errorf("persist failed: %w", err)
		}
		return map[string]interface{}{"deleted": existed}, nil
	case has(cmd, "clear_frames"):
		if pt.persist == nil {
			return nil, errors.New("persistence disabled; cannot clear")
		}
		n := pt.store.ClearFrames()
		if err := pt.saveFrames(ctx); err != nil {
			return nil, fmt.Errorf("persist failed: %w", err)
		}
		return map[string]interface{}{"deleted": n}, nil
```

**Step 4: Run and watch pass** — `go test ./models/posetracker/ -v` → PASS.

**Step 5: Commit**

```bash
git add models/posetracker/ ; git commit -m "feat: DoCommand list/delete/clear frames"
```

---

### Task 10: WSS — UUID derivation + frame→Transform mapping (pure)

**Files:**
- Create: `models/worldstatestore/mapping.go`
- Test: `models/worldstatestore/mapping_test.go`

**Step 1: Write the failing test**

```go
package worldstatestore

import (
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
)

func TestUUIDStableForName(t *testing.T) {
	test.That(t, uuidForName("fixture_a"), test.ShouldResemble, uuidForName("fixture_a"))
	test.That(t, uuidForName("a"), test.ShouldNotResemble, uuidForName("b"))
}

func TestFrameToTransformSetsAxesHelper(t *testing.T) {
	pif := referenceframe.NewPoseInFrame("world", spatialmath.NewPoseFromPoint(r3.Vector{X: 1}))
	tf := frameToTransform("fixture_a", pif)
	test.That(t, tf.ReferenceFrame, test.ShouldEqual, "fixture_a")
	test.That(t, tf.Metadata.Fields["showAxesHelper"].GetBoolValue(), test.ShouldBeTrue)
}
```

**Step 2: Run and watch fail** — FAIL.

**Step 3: Implement `models/worldstatestore/mapping.go`**

```go
// Package worldstatestore implements the teach-mode world_state_store mirror.
package worldstatestore

import (
	"github.com/google/uuid"
	commonpb "go.viam.com/api/common/v1"
	"go.viam.com/rdk/referenceframe"
	"google.golang.org/protobuf/types/known/structpb"
)

// namespaceUUID is a fixed namespace so name->UUID is stable across runs.
var namespaceUUID = uuid.MustParse("6f0e1a3c-0000-4000-8000-000000000000")

func uuidForName(name string) []byte {
	u := uuid.NewSHA1(namespaceUUID, []byte(name))
	b, _ := u.MarshalBinary()
	return b
}

// frameToTransform maps a taught frame to a world_state_store Transform with an
// axes-helper hint so the visualization renders it as a coordinate triad.
func frameToTransform(name string, pif *referenceframe.PoseInFrame) *commonpb.Transform {
	meta, _ := structpb.NewStruct(map[string]interface{}{"showAxesHelper": true})
	return &commonpb.Transform{
		ReferenceFrame:      name,
		PoseInObserverFrame: referenceframe.PoseInFrameToProtobuf(pif),
		Metadata:            meta,
		Uuid:                uuidForName(name),
	}
}
```

> Verify field names on `commonpb.Transform` (`Uuid`, `ReferenceFrame`, `PoseInObserverFrame`, `Metadata`) against `go.viam.com/api/common/v1` — adjust casing if the generated struct differs. `referenceframe.PoseInFrameToProtobuf` exists (used in `components/posetracker/server.go`).

**Step 4: Run and watch pass** — `go test ./models/worldstatestore/ -v` → PASS.

**Step 5: Commit**

```bash
git add models/worldstatestore/ ; git commit -m "feat: WSS frame->Transform mapping + stable UUIDs"
```

---

### Task 11: WSS model — registration, dependency, ListUUIDs, GetTransform

**Files:**
- Modify: `models/worldstatestore/mapping.go` → add `model.go`
- Create: `models/worldstatestore/model.go`
- Test: `models/worldstatestore/model_test.go`

The WSS depends on the PoseTracker and reads its current frames via the `PoseTracker` interface (`Poses`). Same-module deps are delivered as direct pointers, so this is an in-process call.

**Step 1: Write the failing test (inject a fake PoseTracker)**

```go
func TestListAndGetTransform(t *testing.T) {
	fakePT := &fakePoseTracker{poses: referenceframe.FrameSystemPoses{
		"a": referenceframe.NewPoseInFrame("world", spatialmath.NewPoseFromPoint(r3.Vector{X: 1})),
	}}
	svc := newForTest(t, fakePT)

	uuids, err := svc.ListUUIDs(context.Background(), nil)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(uuids), test.ShouldEqual, 1)

	tf, err := svc.GetTransform(context.Background(), uuidForName("a"), nil)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, tf.ReferenceFrame, test.ShouldEqual, "a")
}
```

Add a `fakePoseTracker` in the test file implementing `posetracker.PoseTracker` (only `Poses` needs real behavior; embed `resource.Named`/return zero values for the rest, or embed a `posetracker.PoseTracker` nil with overridden `Poses`). Simplest: define a minimal struct with `Poses` and the embedded `resource.Named` + no-op `Close`/`DoCommand`/`Reconfigure`.

**Step 2: Run and watch fail** — FAIL.

**Step 3: Implement `models/worldstatestore/model.go`**

```go
package worldstatestore

import (
	"context"
	"fmt"
	"testing"

	commonpb "go.viam.com/api/common/v1"
	"go.viam.com/rdk/components/posetracker"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/worldstatestore"
)

// Model — CHANGE the namespace/family.
var Model = resource.NewModel("viam-labs", "teach-frames", "world-state-store")

// Config references the PoseTracker to mirror.
type Config struct {
	PoseTracker string `json:"pose_tracker"`
}

// Validate requires and depends on the PoseTracker.
func (c *Config) Validate(path string) ([]string, error) {
	if c.PoseTracker == "" {
		return nil, resource.NewConfigValidationFieldRequiredError(path, "pose_tracker")
	}
	return []string{c.PoseTracker}, nil
}

func init() {
	resource.RegisterService(worldstatestore.API, Model,
		resource.Registration[worldstatestore.Service, *Config]{Constructor: newWSS})
}

type mirror struct {
	resource.Named
	resource.AlwaysRebuild
	logger logging.Logger
	pt     posetracker.PoseTracker
}

func newWSS(
	ctx context.Context, deps resource.Dependencies, conf resource.Config, logger logging.Logger,
) (worldstatestore.Service, error) {
	cfg, err := resource.NativeConfig[*Config](conf)
	if err != nil {
		return nil, err
	}
	pt, err := posetracker.FromDependencies(deps, cfg.PoseTracker)
	if err != nil {
		return nil, err
	}
	return &mirror{Named: conf.ResourceName().AsNamed(), logger: logger, pt: pt}, nil
}

func (m *mirror) ListUUIDs(ctx context.Context, _ map[string]any) ([][]byte, error) {
	poses, err := m.pt.Poses(ctx, nil, nil)
	if err != nil {
		return nil, err
	}
	out := make([][]byte, 0, len(poses))
	for name := range poses {
		out = append(out, uuidForName(name))
	}
	return out, nil
}

func (m *mirror) GetTransform(ctx context.Context, id []byte, _ map[string]any) (*commonpb.Transform, error) {
	poses, err := m.pt.Poses(ctx, nil, nil)
	if err != nil {
		return nil, err
	}
	for name, pif := range poses {
		if string(uuidForName(name)) == string(id) {
			return frameToTransform(name, pif), nil
		}
	}
	return nil, fmt.Errorf("no transform for uuid")
}

func (m *mirror) Close(_ context.Context) error { return nil }

func newForTest(t *testing.T, pt posetracker.PoseTracker) *mirror {
	t.Helper()
	return &mirror{logger: logging.NewTestLogger(t), pt: pt}
}
```

> Verify `posetracker.FromDependencies` exists (the SDK provides `FromDependencies`/`FromRobot` helpers per API). If only `resource.FromDependencies[posetracker.PoseTracker]` is available, use that. Also confirm `worldstatestore.API` and `worldstatestore.Service` import paths.

**Step 4: Run and watch pass** — `go test ./models/worldstatestore/ -v` → PASS.

**Step 5: Commit**

```bash
git add models/worldstatestore/ ; git commit -m "feat: WSS model ListUUIDs/GetTransform over PoseTracker dep"
```

---

### Task 12: WSS — StreamTransformChanges

**Files:**
- Modify: `models/worldstatestore/model.go`
- Test: `models/worldstatestore/model_test.go`

With Option 1 (rebuild-and-reseed), the stream stays open and seeds via `ListUUIDs`/`GetTransform`; it does not need to emit deltas (commits rebuild the resource, the viz reconnects and reseeds). Implement a minimal correct stream that blocks until the context ends, so the contract is satisfied without leaking.

**Step 1: Write the failing test (stream returns and closes cleanly on ctx cancel)**

```go
func TestStreamClosesOnContextCancel(t *testing.T) {
	svc := newForTest(t, &fakePoseTracker{})
	ctx, cancel := context.WithCancel(context.Background())
	stream, err := svc.StreamTransformChanges(ctx, nil)
	test.That(t, err, test.ShouldBeNil)
	cancel()
	_, err = stream.Next()
	test.That(t, err, test.ShouldNotBeNil) // ctx.Err() or io.EOF
}
```

**Step 2: Run and watch fail** — FAIL (method undefined).

**Step 3: Implement**

```go
func (m *mirror) StreamTransformChanges(
	ctx context.Context, _ map[string]any,
) (*worldstatestore.TransformChangeStream, error) {
	// Option 1: no live deltas — the resource rebuilds on every committed change,
	// so the viz reconnects and reseeds via ListUUIDs/GetTransform. The stream
	// simply stays open until the context ends.
	ch := make(chan worldstatestore.TransformChange) // never written; closes via ctx
	return worldstatestore.NewTransformChangeStreamFromChannel(ctx, ch), nil
}
```

> If, during integration, the visualization requires an initial ADDED burst over the stream (rather than seeding purely from ListUUIDs/GetTransform), emit one ADDED `TransformChange` per current frame into a buffered channel before returning. Confirm against the visualization's `useWorldState` behavior (it seeds via ListUUIDs first, so deltas-only is expected to suffice).

**Step 4: Run and watch pass** — `go test ./models/worldstatestore/ -v` → PASS.

**Step 5: Commit**

```bash
git add models/worldstatestore/ ; git commit -m "feat: WSS StreamTransformChanges (open, reseed-on-rebuild)"
```

---

### Task 13: Wire both models into the module entrypoint

**Files:**
- Modify: `cmd/module/main.go`

**Step 1: Complete `main.go`**

```go
package main

import (
	"context"

	pt "github.com/viam-labs/teach-frames/models/posetracker"
	wss "github.com/viam-labs/teach-frames/models/worldstatestore"
	"go.viam.com/rdk/components/posetracker"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/worldstatestore"
	"go.viam.com/utils"
)

func main() {
	utils.ContextualMain(mainWithArgs, module.NewLoggerFromArgs("teach-frames"))
}

func mainWithArgs(ctx context.Context, args []string, logger logging.Logger) error {
	m, err := module.NewModuleFromArgs(ctx)
	if err != nil {
		return err
	}
	if err := m.AddModelFromRegistry(ctx, posetracker.API, pt.Model); err != nil {
		return err
	}
	if err := m.AddModelFromRegistry(ctx, worldstatestore.API, wss.Model); err != nil {
		return err
	}
	if err := m.Start(ctx); err != nil {
		return err
	}
	defer m.Close(ctx)
	<-ctx.Done()
	return nil
}

var _ = resource.Model{} // keep resource import if unused; remove if not needed
```

> The blank import-keeper line is a hint; remove it. The model packages must be imported so their `init()` registration runs — the aliased imports above do that.

**Step 2: Verify build**

Run: `make build`
Expected: compiles.

**Step 3: Run the whole suite**

Run: `make test`
Expected: all green.

**Step 4: Commit**

```bash
git add cmd/ ; git commit -m "feat: register both models in module entrypoint"
```

---

### Task 14: meta.json, README, manual integration checklist

**Files:**
- Create: `meta.json`, `README.md`

**Step 1: Write `meta.json`** (adjust module id/org)

```json
{
  "module_id": "viam-labs:teach-frames",
  "visibility": "private",
  "url": "https://github.com/viam-labs/teach-frames",
  "description": "Teach arbitrary workcell frames by jogging an arm; mirror to world_state_store for visualization.",
  "models": [
    { "api": "rdk:component:pose_tracker", "model": "viam-labs:teach-frames:pose-tracker" },
    { "api": "rdk:service:world_state_store", "model": "viam-labs:teach-frames:world-state-store" }
  ],
  "entrypoint": "bin/teach-frames"
}
```

**Step 2: Write `README.md`** — document config for both models, the DoCommand verbs (table from the design doc), required env vars for persistence, and the manual integration checklist:

1. Configure an `arm` + builtin `motion` service on a machine.
2. Add the `pose-tracker` model with `motion_service`, `tcp_component`, `destination_frame`.
3. Add the `world-state-store` model with `pose_tracker` set to the tracker's name.
4. Jog the arm; call `capture_point` ×3 via the DoCommand UI; call `define_frame {name, method:"3point"}`.
5. Confirm the component config gained a `frames` entry (durability).
6. Open the visualization; confirm an axes triad appears at the taught frame.
7. Restart the machine; confirm the frame reloads from config.

**Step 3: Commit**

```bash
git add meta.json README.md ; git commit -m "docs: meta.json + README + integration checklist"
```

---

### Task 15: Implement the real AppPersister + final verification

**Files:**
- Modify: `persist/appclient.go`

**Step 1:** Implement `AppPersister.Save` against `go.viam.com/api/app/v1`:
- Build the app client from env creds (see platform-APIs docs for the SDK helper).
- `GetRobotPart(partID)` → part with `RobotConfig` struct.
- Walk `components`, find the entry whose `name == ComponentName`, set `attributes.frames` to the serialized specs (marshal `[]config.FrameSpec` into the struct).
- `UpdateRobotPart(partID, name, robotConfig)`.

**Step 2:** Manual test using the Task 14 checklist on a real machine (this path has no unit test by design — it needs cloud).

**Step 3:** Final gates:

```bash
make lint
make test
```
Expected: clean lint, all tests pass.

**Step 4: Commit + tag**

```bash
git add persist/ ; git commit -m "feat: implement app-API config persistence"
git tag v0.1.0
```

---

## Build sequence summary

Pure logic first (Tasks 1–3, 10), then the testable seams (4–5), then the PoseTracker model and its DoCommand surface (6–9), then the WSS model (11–12), then wiring and docs (13–14), and finally the cloud-dependent persistence (15). Every task is TDD with a commit at the end. The only code without unit coverage is the live app-API call, covered by the manual checklist.

## Open items to confirm during execution

- **Rebuild propagation:** verify that rebuilding the PoseTracker rebuilds the dependent WSS (Option 1's reseed basis). If it doesn't, the viz won't refresh on commit — fall back to the design doc's Option 2 (shared store) noted under "Known limitations."
- **SDK symbol drift:** `resource.NativeConfig`, `motion.FromDependencies`, `posetracker.FromDependencies`, `commonpb.Transform` field names, `spatialmath.NewRotationMatrix` ordering — all flagged inline; confirm against the pinned SDK version.
- **app/v1 shapes:** `GetRobotPart`/`UpdateRobotPart` request/response and the `RobotConfig` struct path for `attributes.frames`.
