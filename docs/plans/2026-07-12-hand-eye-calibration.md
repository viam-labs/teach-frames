# Hand-Eye Calibration Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Add eye-to-hand camera calibration to the `pose-tracker` component: touch physical points with the calibrated TCP, click the same points in a frozen RGBD camera frame, and solve the fixed camera's pose in the world (`camera → world`), persisting it to the camera component's own `frame` config.

**Architecture:** Extend the existing `pose-tracker` component (no new model). A new optional `camera` dependency enables five DoCommands that mirror the existing capture→define shape. The camera is abstracted behind a `posesource.CameraSource` interface (same pattern as `PoseSource`/`FlangeSource`) so the deprojection and solve math are pure, unit-testable functions. The solve is a closed-form Kabsch/Umeyama rigid transform over (world, camera) point pairs. Persistence reuses the existing `SaveComponentFrame` app-API path.

**Tech Stack:** Go (RDK v0.131.0, `gonum/mat`, `spatialmath`, `rimage`, `rimage/transform`), Svelte 5 + Vite + `@viamrobotics/svelte-sdk` frontend, vitest.

---

## Background: verified RDK APIs (v0.131.0)

Confirmed against the local module cache — use these exact signatures:

```go
// components/camera
Camera.Images(ctx, filterSourceNames []string, extra map[string]interface{}) ([]NamedImage, resource.ResponseMetadata, error)
Camera.Properties(ctx) (camera.Properties, error)   // .IntrinsicParams *transform.PinholeCameraIntrinsics
NamedImage.Image(ctx) (image.Image, error)
NamedImage.SourceName string        // "color" / "depth" by convention
NamedImage.MimeType() string        // rutils.MimeTypeJPEG / rutils.MimeTypeRawDepth

// rimage
rimage.ConvertImageToDepthMap(ctx, img image.Image) (*rimage.DepthMap, error)
DepthMap.GetDepth(x, y int) rimage.Depth
DepthMap.Width() int; DepthMap.Height() int; DepthMap.Contains(x, y int) bool
rimage.EncodeImage(ctx, img image.Image, mimeType string) ([]byte, error)

// rimage/transform.PinholeCameraIntrinsics { Width, Height int; Fx, Fy, Ppx, Ppy float64 }
intr.ImagePointTo3DPoint(pt image.Point, d rimage.Depth) (r3.Vector, error)
```

`camera.FromDependencies(deps, name)` resolves the dependency (import `go.viam.com/rdk/components/camera`).

## Solve math (reference for Tasks 1 & 8)

Pairs are `(world, camera)` r3 vectors (mm). Solve `world ≈ R·camera + t`:

1. `camMean`, `worldMean` = centroids.
2. `H = Σ (cameraᵢ − camMean)(worldᵢ − worldMean)ᵀ` — a 3×3 matrix.
3. SVD `H = U S Vᵀ` (`gonum` `mat.SVD`).
4. `d = sign(det(V·Uᵀ))`; `R = V · diag(1,1,d) · Uᵀ` (the `d` term prevents a reflection).
5. `t = worldMean − R·camMean`.
6. `residual_rms = sqrt(mean_i |R·cameraᵢ + t − worldᵢ|²)`.

**Degeneracy:** 3 non-collinear points fully determine `(R,t)` — coplanar is fine. Reject **collinear** inputs (the 2nd singular value of the centered camera points ≈ 0 → rotation about the line is unconstrained). Require ≥3 points; recommend ≥4 non-coplanar for meaningful residual.

---

## Task 1: Kabsch solve + deprojection (pure functions)

**Files:**
- Create: `frames/handeye.go`
- Test: `frames/handeye_test.go`

**Step 1: Write the failing tests**

```go
package frames

import (
	"image"
	"math"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/rimage"
	"go.viam.com/rdk/rimage/transform"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
)

func TestComputeCameraToWorldRoundTrip(t *testing.T) {
	// Known camera->world transform: rotate 90° about Z, translate (100,-50,300).
	rot := spatialmath.NewPoseFromOrientation(&spatialmath.EulerAngles{Yaw: math.Pi / 2})
	truth := spatialmath.NewPose(r3.Vector{X: 100, Y: -50, Z: 300}, rot.Orientation())

	camPts := []r3.Vector{{X: 0, Y: 0, Z: 0}, {X: 100, Y: 0, Z: 0}, {X: 0, Y: 100, Z: 0}, {X: 0, Y: 0, Z: 100}}
	pairs := make([]HandEyePair, len(camPts))
	for i, c := range camPts {
		w := spatialmath.Compose(truth, spatialmath.NewPoseFromPoint(c)).Point()
		pairs[i] = HandEyePair{Camera: c, World: w}
	}

	got, residual, err := ComputeCameraToWorld(pairs)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, residual, test.ShouldBeLessThan, 1e-6)
	test.That(t, spatialmath.PoseAlmostEqualEps(got, truth, 1e-6), test.ShouldBeTrue)
}

func TestComputeCameraToWorldRejectsCollinear(t *testing.T) {
	pairs := []HandEyePair{
		{Camera: r3.Vector{X: 0}, World: r3.Vector{X: 0}},
		{Camera: r3.Vector{X: 1}, World: r3.Vector{X: 1}},
		{Camera: r3.Vector{X: 2}, World: r3.Vector{X: 2}},
	}
	_, _, err := ComputeCameraToWorld(pairs)
	test.That(t, err, test.ShouldNotBeNil)
}

func TestComputeCameraToWorldRejectsTooFew(t *testing.T) {
	_, _, err := ComputeCameraToWorld([]HandEyePair{{}, {}})
	test.That(t, err, test.ShouldNotBeNil)
}

func TestDeproject(t *testing.T) {
	intr := &transform.PinholeCameraIntrinsics{Width: 100, Height: 100, Fx: 50, Fy: 50, Ppx: 50, Ppy: 50}
	dm := rimage.NewEmptyDepthMap(100, 100)
	dm.Set(60, 50, rimage.Depth(1000)) // 1 m at pixel (60,50)

	p, err := Deproject(intr, dm, 60, 50)
	test.That(t, err, test.ShouldBeNil)
	// x = (60-50)/50 * 1000 = 200; y = (50-50)/50*1000 = 0; z = 1000
	test.That(t, p.X, test.ShouldAlmostEqual, 200.0)
	test.That(t, p.Y, test.ShouldAlmostEqual, 0.0)
	test.That(t, p.Z, test.ShouldAlmostEqual, 1000.0)
}

func TestDeprojectZeroDepthErrors(t *testing.T) {
	intr := &transform.PinholeCameraIntrinsics{Width: 100, Height: 100, Fx: 50, Fy: 50, Ppx: 50, Ppy: 50}
	dm := rimage.NewEmptyDepthMap(100, 100)
	_, err := Deproject(intr, dm, 10, 10) // depth 0
	test.That(t, err, test.ShouldNotBeNil)
}

func TestDeprojectOutOfBoundsErrors(t *testing.T) {
	intr := &transform.PinholeCameraIntrinsics{Width: 100, Height: 100, Fx: 50, Fy: 50, Ppx: 50, Ppy: 50}
	dm := rimage.NewEmptyDepthMap(100, 100)
	_, err := Deproject(intr, dm, 200, 10)
	test.That(t, err, test.ShouldNotBeNil)
	_ = image.Point{} // keep image import used if trimmed
}
```

**Step 2: Run to verify they fail**

Run: `go test ./frames/ -run 'CameraToWorld|Deproject' -v`
Expected: FAIL (undefined: `HandEyePair`, `ComputeCameraToWorld`, `Deproject`).

**Step 3: Write the implementation**

```go
// Package frames — hand-eye (eye-to-hand) calibration helpers.
package frames

import (
	"errors"
	"fmt"
	"image"
	"math"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/rimage"
	"go.viam.com/rdk/rimage/transform"
	"go.viam.com/rdk/spatialmath"
	"gonum.org/v1/gonum/mat"
)

// HandEyePair is one correspondence: a physical point's coordinate in the world
// frame (from the calibrated TCP) and in the camera frame (from deprojection).
type HandEyePair struct {
	World  r3.Vector
	Camera r3.Vector
}

// collinearSingularEps flags a near-zero second singular value (points span < 2 dims).
const collinearSingularEps = 1e-6

// ComputeCameraToWorld solves the rigid transform T such that world ≈ T·camera
// (Kabsch/Umeyama, rotation only). Returns the pose, the RMS fit residual (mm),
// and an error for too-few or collinear inputs. Coplanar (but non-collinear)
// inputs are accepted.
func ComputeCameraToWorld(pairs []HandEyePair) (spatialmath.Pose, float64, error) {
	n := len(pairs)
	if n < 3 {
		return nil, 0, fmt.Errorf("camera calibration requires at least 3 points, have %d", n)
	}

	var camMean, worldMean r3.Vector
	for _, p := range pairs {
		camMean = camMean.Add(p.Camera)
		worldMean = worldMean.Add(p.World)
	}
	camMean = camMean.Mul(1.0 / float64(n))
	worldMean = worldMean.Mul(1.0 / float64(n))

	// H = Σ (cam-camMean)(world-worldMean)ᵀ, and the centered camera matrix C
	// (3×n) whose singular values reveal collinearity.
	h := mat.NewDense(3, 3, nil)
	cCentered := mat.NewDense(3, n, nil)
	for i, p := range pairs {
		c := p.Camera.Sub(camMean)
		w := p.World.Sub(worldMean)
		cCentered.Set(0, i, c.X)
		cCentered.Set(1, i, c.Y)
		cCentered.Set(2, i, c.Z)
		h.Set(0, 0, h.At(0, 0)+c.X*w.X)
		h.Set(0, 1, h.At(0, 1)+c.X*w.Y)
		h.Set(0, 2, h.At(0, 2)+c.X*w.Z)
		h.Set(1, 0, h.At(1, 0)+c.Y*w.X)
		h.Set(1, 1, h.At(1, 1)+c.Y*w.Y)
		h.Set(1, 2, h.At(1, 2)+c.Y*w.Z)
		h.Set(2, 0, h.At(2, 0)+c.Z*w.X)
		h.Set(2, 1, h.At(2, 1)+c.Z*w.Y)
		h.Set(2, 2, h.At(2, 2)+c.Z*w.Z)
	}

	// Reject collinear inputs: the centered camera points must span ≥ 2 dims.
	var csvd mat.SVD
	if !csvd.Factorize(cCentered, mat.SVDNone) {
		return nil, 0, errors.New("camera point SVD failed")
	}
	csv := csvd.Values(nil)
	if len(csv) < 2 || csv[1] < collinearSingularEps*csv[0] {
		return nil, 0, errors.New("camera points are collinear; capture points spanning at least a plane")
	}

	var svd mat.SVD
	if !svd.Factorize(h, mat.SVDFull) {
		return nil, 0, errors.New("hand-eye SVD failed")
	}
	var u, v mat.Dense
	svd.UTo(&u)
	svd.VTo(&v)

	// R = V · diag(1,1, det(V·Uᵀ)) · Uᵀ
	var vut mat.Dense
	vut.Mul(&v, u.T())
	d := mat.Det(&vut)
	dsign := 1.0
	if d < 0 {
		dsign = -1.0
	}
	diag := mat.NewDense(3, 3, []float64{1, 0, 0, 0, 1, 0, 0, 0, dsign})
	var vd, r mat.Dense
	vd.Mul(&v, diag)
	r.Mul(&vd, u.T())

	rm, err := spatialmath.NewRotationMatrix([]float64{
		r.At(0, 0), r.At(0, 1), r.At(0, 2),
		r.At(1, 0), r.At(1, 1), r.At(1, 2),
		r.At(2, 0), r.At(2, 1), r.At(2, 2),
	})
	if err != nil {
		return nil, 0, err
	}

	// t = worldMean − R·camMean
	rc := rotate(&r, camMean)
	t := worldMean.Sub(rc)

	pose := spatialmath.NewPose(t, rm)

	// Residual RMS.
	var sumSq float64
	for _, p := range pairs {
		pred := rotate(&r, p.Camera).Add(t)
		sumSq += pred.Sub(p.World).Norm2()
	}
	residual := math.Sqrt(sumSq / float64(n))

	return pose, residual, nil
}

// rotate applies a 3×3 rotation matrix to a vector.
func rotate(r *mat.Dense, v r3.Vector) r3.Vector {
	return r3.Vector{
		X: r.At(0, 0)*v.X + r.At(0, 1)*v.Y + r.At(0, 2)*v.Z,
		Y: r.At(1, 0)*v.X + r.At(1, 1)*v.Y + r.At(1, 2)*v.Z,
		Z: r.At(2, 0)*v.X + r.At(2, 1)*v.Y + r.At(2, 2)*v.Z,
	}
}

// Deproject converts pixel (u,v) with its depth-map depth into a 3D point in the
// camera frame (mm). Errors on out-of-bounds pixels or zero/absent depth.
func Deproject(intr *transform.PinholeCameraIntrinsics, dm *rimage.DepthMap, u, v int) (r3.Vector, error) {
	if intr == nil {
		return r3.Vector{}, errors.New("camera intrinsics unavailable; cannot deproject")
	}
	if dm == nil {
		return r3.Vector{}, errors.New("no depth frame available; snapshot first")
	}
	if !dm.Contains(u, v) {
		return r3.Vector{}, fmt.Errorf("pixel (%d,%d) out of depth bounds %dx%d", u, v, dm.Width(), dm.Height())
	}
	d := dm.GetDepth(u, v)
	if d == 0 {
		return r3.Vector{}, fmt.Errorf("no depth at pixel (%d,%d); aim at a surface with valid depth", u, v)
	}
	return intr.ImagePointTo3DPoint(image.Pt(u, v), d)
}
```

> **Note on the `RotationMatrix` row/column convention:** `frames/compute.go`
> documents that `spatialmath.Compose` applies the *transpose* of the matrix
> `RotationMatrix.Col()`/`Mul()` reads back. The round-trip test
> (`TestComputeCameraToWorldRoundTrip`) is the guard here: if the recovered pose
> does not match `truth`, the `R` rows/cols are transposed — swap to
> `r.At(col,row)` ordering. Get the test green before moving on.

**Step 4: Run to verify pass**

Run: `go test ./frames/ -run 'CameraToWorld|Deproject' -v`
Expected: PASS. If the round-trip fails on orientation only, apply the transpose note above.

**Step 5: Commit**

```bash
git add frames/handeye.go frames/handeye_test.go
git commit -m "feat(frames): add Kabsch camera->world solve and pixel deprojection"
```

---

## Task 2: Hand-eye buffer in the store

**Files:**
- Modify: `store/store.go`
- Test: `store/store_test.go`

**Step 1: Write the failing test** (append to `store/store_test.go`)

```go
func TestHandEyeBuffer(t *testing.T) {
	s := store.New()
	test.That(t, s.HandEyeBufferLen(), test.ShouldEqual, 0)

	i0 := s.AddHandEyePair(frames.HandEyePair{World: r3.Vector{X: 1}, Camera: r3.Vector{Z: 2}})
	test.That(t, i0, test.ShouldEqual, 0)
	i1 := s.AddHandEyePair(frames.HandEyePair{World: r3.Vector{X: 3}, Camera: r3.Vector{Z: 4}})
	test.That(t, i1, test.ShouldEqual, 1)
	test.That(t, s.HandEyeBufferLen(), test.ShouldEqual, 2)

	buf := s.HandEyeBuffer()
	test.That(t, len(buf), test.ShouldEqual, 2)
	test.That(t, buf[0].World.X, test.ShouldEqual, 1)

	// Copy semantics: mutating the returned slice must not affect the store.
	buf[0].World.X = 999
	test.That(t, s.HandEyeBuffer()[0].World.X, test.ShouldEqual, 1)

	n := s.ClearHandEyeBuffer()
	test.That(t, n, test.ShouldEqual, 2)
	test.That(t, s.HandEyeBufferLen(), test.ShouldEqual, 0)
}
```

Add imports to the test file if missing: `"github.com/golang/geo/r3"` and `"github.com/viam-labs/teach-frames/frames"`.

**Step 2: Run to verify fail**

Run: `go test ./store/ -run HandEye -v`
Expected: FAIL (undefined methods).

**Step 3: Implement** (in `store/store.go`)

Add the import `"github.com/viam-labs/teach-frames/frames"`, a field, and methods:

```go
// in FrameStore struct:
	handeyeBuffer []frames.HandEyePair

// AddHandEyePair appends a hand-eye correspondence and returns its zero-based index.
func (s *FrameStore) AddHandEyePair(p frames.HandEyePair) int {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.handeyeBuffer = append(s.handeyeBuffer, p)
	return len(s.handeyeBuffer) - 1
}

// HandEyeBuffer returns a copy of the current hand-eye buffer.
func (s *FrameStore) HandEyeBuffer() []frames.HandEyePair {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return append([]frames.HandEyePair(nil), s.handeyeBuffer...)
}

// HandEyeBufferLen returns the number of pairs in the hand-eye buffer.
func (s *FrameStore) HandEyeBufferLen() int {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return len(s.handeyeBuffer)
}

// ClearHandEyeBuffer discards all hand-eye pairs and returns the count removed.
func (s *FrameStore) ClearHandEyeBuffer() int {
	s.mu.Lock()
	defer s.mu.Unlock()
	n := len(s.handeyeBuffer)
	s.handeyeBuffer = nil
	return n
}
```

> `HandEyePair` is a value type (two `r3.Vector`s, themselves value types), so the
> `append(nil, ...)` copy is a full deep copy — the mutation test passes without
> extra copying.

**Step 4: Run to verify pass**

Run: `go test ./store/ -run HandEye -v` → PASS
Run: `go test ./store/ ./frames/` → PASS (no regressions)

**Step 5: Commit**

```bash
git add store/store.go store/store_test.go
git commit -m "feat(store): add hand-eye correspondence buffer"
```

---

## Task 3: CameraSource abstraction + Fake

**Files:**
- Modify: `posesource/posesource.go`
- Test: `posesource/posesource_test.go`

**Step 1: Write the failing test** (append)

```go
func TestFakeCameraSourceSnapshot(t *testing.T) {
	intr := &transform.PinholeCameraIntrinsics{Width: 4, Height: 4, Fx: 2, Fy: 2, Ppx: 2, Ppy: 2}
	dm := rimage.NewEmptyDepthMap(4, 4)
	dm.Set(1, 1, rimage.Depth(500))
	rgb := image.NewRGBA(image.Rect(0, 0, 4, 4))

	src := &posesource.FakeCamera{RGB: rgb, Depth: dm, Intr: intr}
	snap, err := src.Snapshot(context.Background())
	test.That(t, err, test.ShouldBeNil)
	test.That(t, snap.Intr.Fx, test.ShouldEqual, 2.0)
	test.That(t, snap.Depth.GetDepth(1, 1), test.ShouldEqual, rimage.Depth(500))
	test.That(t, snap.RGB, test.ShouldNotBeNil)
}
```

Add test imports: `"context"`, `"image"`, `"go.viam.com/rdk/rimage"`, `"go.viam.com/rdk/rimage/transform"`.

**Step 2: Run to verify fail** → `go test ./posesource/ -run FakeCamera -v` — FAIL.

**Step 3: Implement** (append to `posesource/posesource.go`; add imports `"image"`, `"go.viam.com/rdk/components/camera"`, `"go.viam.com/rdk/rimage"`, `"go.viam.com/rdk/rimage/transform"`, `rutils "go.viam.com/rdk/utils"`)

```go
// Snapshot is one RGBD acquisition: a color image, an aligned depth map, and the
// camera intrinsics used to deproject depth pixels.
type Snapshot struct {
	RGB   image.Image
	Depth *rimage.DepthMap
	Intr  *transform.PinholeCameraIntrinsics
}

// CameraSource acquires an aligned RGB+depth snapshot with intrinsics.
type CameraSource interface {
	Snapshot(ctx context.Context) (*Snapshot, error)
}

// RGBDCamera adapts an RDK camera to CameraSource.
type RGBDCamera struct {
	Cam camera.Camera
}

// Snapshot fetches color+depth via Images() and intrinsics via Properties().
// It selects the color and depth images by mime type, falling back to source
// name ("color"/"depth") when mime types are ambiguous.
func (c *RGBDCamera) Snapshot(ctx context.Context) (*Snapshot, error) {
	imgs, _, err := c.Cam.Images(ctx, nil, nil)
	if err != nil {
		return nil, err
	}
	var colorImg, depthImg image.Image
	for i := range imgs {
		ni := &imgs[i]
		img, ierr := ni.Image(ctx)
		if ierr != nil {
			return nil, ierr
		}
		switch {
		case ni.MimeType() == rutils.MimeTypeRawDepth || ni.SourceName == "depth":
			depthImg = img
		case ni.SourceName == "color" || colorImg == nil:
			colorImg = img
		}
	}
	if colorImg == nil {
		return nil, errors.New("camera returned no color image")
	}
	if depthImg == nil {
		return nil, errors.New("camera returned no depth image; an RGBD/depth camera is required")
	}
	dm, err := rimage.ConvertImageToDepthMap(ctx, depthImg)
	if err != nil {
		return nil, fmt.Errorf("could not read depth map: %w", err)
	}
	props, err := c.Cam.Properties(ctx)
	if err != nil {
		return nil, err
	}
	if props.IntrinsicParams == nil {
		return nil, errors.New("camera exposes no intrinsics; cannot deproject (configure intrinsic_parameters on the camera)")
	}
	return &Snapshot{RGB: colorImg, Depth: dm, Intr: props.IntrinsicParams}, nil
}

// FakeCamera returns a fixed snapshot. For tests.
type FakeCamera struct {
	RGB   image.Image
	Depth *rimage.DepthMap
	Intr  *transform.PinholeCameraIntrinsics
	Err   error
}

// Snapshot returns the canned snapshot or the configured error.
func (f *FakeCamera) Snapshot(_ context.Context) (*Snapshot, error) {
	if f.Err != nil {
		return nil, f.Err
	}
	return &Snapshot{RGB: f.RGB, Depth: f.Depth, Intr: f.Intr}, nil
}
```

Add `"errors"` and `"fmt"` to the file imports if not present (`errors` is already imported).

**Step 4: Run to verify pass** → `go test ./posesource/ -v` — PASS.

**Step 5: Commit**

```bash
git add posesource/posesource.go posesource/posesource_test.go
git commit -m "feat(posesource): add RGBD CameraSource abstraction and fake"
```

---

## Task 4: Config `camera` attribute + constructor wiring

**Files:**
- Modify: `models/posetracker/model.go`
- Test: `models/posetracker/model_test.go`

**Step 1: Write the failing test** (append; follow existing Validate tests for style)

```go
func TestConfigValidateCameraDependency(t *testing.T) {
	cfg := &posetracker.Config{TCPComponent: "gripper", Camera: "cam"}
	deps, _, err := cfg.Validate("")
	test.That(t, err, test.ShouldBeNil)
	test.That(t, deps, test.ShouldContain, "cam")
}
```

**Step 2: Run to verify fail** → `go test ./models/posetracker/ -run CameraDependency -v` — FAIL (unknown field `Camera`).

**Step 3: Implement** (in `models/posetracker/model.go`)

Add the config field:
```go
	Arm              string               `json:"arm"`
	Camera           string               `json:"camera"`
	Frames           []tfconfig.FrameSpec `json:"frames"`
```

Extend `Validate` to declare the dependency when set (after the `Arm` block):
```go
	if c.Camera != "" {
		deps = append(deps, c.Camera)
	}
```

Add a struct field and constructor wiring. In `teachTracker`:
```go
	cameraSrc  posesource.CameraSource
	cameraName string

	// snapshotMu guards lastSnapshot; the cached depth+intrinsics MUST be the
	// exact frame the operator clicked on, so capture deprojects against this,
	// not a fresh acquisition.
	snapshotMu   sync.Mutex
	lastSnapshot *posesource.Snapshot
```

In `newPoseTracker`, after the arm block:
```go
	pt.cameraName = cfg.Camera
	if cfg.Camera != "" {
		cam, cerr := camera.FromDependencies(deps, cfg.Camera)
		if cerr != nil {
			logger.Warnw("hand-eye calibration disabled: camera dependency not resolvable", "camera", cfg.Camera, "err", cerr)
		} else {
			pt.cameraSrc = &posesource.RGBDCamera{Cam: cam}
		}
	}
```

Add `"go.viam.com/rdk/components/camera"` to the model imports.

**Step 4: Run to verify pass** → `go test ./models/posetracker/ -v` — PASS. Also `go build ./...`.

**Step 5: Commit**

```bash
git add models/posetracker/model.go models/posetracker/model_test.go
git commit -m "feat(posetracker): add optional camera dependency"
```

---

## Task 5: `handeye_snapshot` DoCommand

**Files:**
- Modify: `models/posetracker/docommand.go`
- Test: `models/posetracker/docommand_test.go`

**Step 1: Write the failing test**

Look at how existing `docommand_test.go` builds a `teachTracker` for tests (it wires a `posesource.Fake` and `persist.Fake`). Add a helper or extend the constructor used there to also set `cameraSrc`. Then:

```go
func TestHandEyeSnapshot(t *testing.T) {
	intr := &transform.PinholeCameraIntrinsics{Width: 4, Height: 4, Fx: 2, Fy: 2, Ppx: 2, Ppy: 2}
	dm := rimage.NewEmptyDepthMap(4, 4)
	dm.Set(1, 1, rimage.Depth(500))
	rgb := image.NewRGBA(image.Rect(0, 0, 4, 4))
	pt := newTestTracker(t) // existing/extended helper
	pt.cameraSrc = &posesource.FakeCamera{RGB: rgb, Depth: dm, Intr: intr}

	resp, err := pt.DoCommand(context.Background(), map[string]interface{}{"handeye_snapshot": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, resp["width"], test.ShouldEqual, 4)
	test.That(t, resp["height"], test.ShouldEqual, 4)
	test.That(t, resp["image"], test.ShouldNotBeNil) // base64 JPEG string
	// cached for capture:
	test.That(t, pt.lastSnapshot, test.ShouldNotBeNil)
}

func TestHandEyeSnapshotNoCameraErrors(t *testing.T) {
	pt := newTestTracker(t)
	pt.cameraSrc = nil
	_, err := pt.DoCommand(context.Background(), map[string]interface{}{"handeye_snapshot": map[string]interface{}{}})
	test.That(t, err, test.ShouldNotBeNil)
}
```

> If `docommand_test.go` has no reusable constructor, add `newTestTracker(t)` there
> that returns a `*teachTracker` with `store.New()`, a `posesource.Fake` source, a
> `persist.Fake`, and `destFrame = "world"` — mirroring the existing per-test setup.

**Step 2: Run to verify fail** → `go test ./models/posetracker/ -run HandEyeSnapshot -v` — FAIL.

**Step 3: Implement**

Add the dispatch case in `DoCommand`'s switch (near the other cases):
```go
	case has(cmd, "handeye_snapshot"):
		return pt.handeyeSnapshot(ctx)
```

Add the method (new file `models/posetracker/handeye.go` to keep `docommand.go` lean; package `posetracker`):
```go
package posetracker

import (
	"context"
	"encoding/base64"
	"errors"
	"fmt"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/rimage"
	rutils "go.viam.com/rdk/utils"

	"github.com/viam-labs/teach-frames/frames"
)

// handeyeSnapshot acquires an RGBD frame, caches the depth+intrinsics for the
// subsequent capture, and returns the RGB as a base64 JPEG for the UI to freeze.
func (pt *teachTracker) handeyeSnapshot(ctx context.Context) (map[string]interface{}, error) {
	if pt.cameraSrc == nil {
		return nil, errors.New("camera dependency not configured; cannot run hand-eye calibration")
	}
	snap, err := pt.cameraSrc.Snapshot(ctx)
	if err != nil {
		return nil, err
	}
	jpeg, err := rimage.EncodeImage(ctx, snap.RGB, rutils.MimeTypeJPEG)
	if err != nil {
		return nil, fmt.Errorf("could not encode snapshot: %w", err)
	}

	pt.snapshotMu.Lock()
	pt.lastSnapshot = snap
	pt.snapshotMu.Unlock()

	b := snap.RGB.Bounds()
	return map[string]interface{}{
		"image":  base64.StdEncoding.EncodeToString(jpeg),
		"width":  b.Dx(),
		"height": b.Dy(),
	}, nil
}
```

**Step 4: Run to verify pass** → `go test ./models/posetracker/ -run HandEyeSnapshot -v` — PASS.

**Step 5: Commit**

```bash
git add models/posetracker/handeye.go models/posetracker/docommand.go models/posetracker/docommand_test.go
git commit -m "feat(posetracker): add handeye_snapshot DoCommand"
```

---

## Task 6: `capture_handeye_point` DoCommand

**Files:**
- Modify: `models/posetracker/handeye.go`, `models/posetracker/docommand.go`
- Test: `models/posetracker/docommand_test.go`

**Step 1: Write the failing test**

```go
func TestCaptureHandEyePoint(t *testing.T) {
	intr := &transform.PinholeCameraIntrinsics{Width: 4, Height: 4, Fx: 2, Fy: 2, Ppx: 2, Ppy: 2}
	dm := rimage.NewEmptyDepthMap(4, 4)
	dm.Set(3, 1, rimage.Depth(1000))
	rgb := image.NewRGBA(image.Rect(0, 0, 4, 4))

	pt := newTestTracker(t)
	pt.cameraSrc = &posesource.FakeCamera{RGB: rgb, Depth: dm, Intr: intr}
	// The Fake PoseSource returns a queued world pose for Capture():
	pt.source = &posesource.Fake{Poses: []spatialmath.Pose{
		spatialmath.NewPoseFromPoint(r3.Vector{X: 10, Y: 20, Z: 30}),
	}}

	// Must snapshot before capture (populates the cache).
	_, err := pt.DoCommand(context.Background(), map[string]interface{}{"handeye_snapshot": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)

	resp, err := pt.DoCommand(context.Background(),
		map[string]interface{}{"capture_handeye_point": map[string]interface{}{"u": 3.0, "v": 1.0}})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, resp["buffer_len"], test.ShouldEqual, 1)
	world := resp["world"].(map[string]interface{})
	test.That(t, world["x"], test.ShouldEqual, 10.0)
	cam := resp["camera"].(map[string]interface{})
	// x = (3-2)/2 * 1000 = 500
	test.That(t, cam["x"], test.ShouldEqual, 500.0)
}

func TestCaptureHandEyeNoSnapshotErrors(t *testing.T) {
	pt := newTestTracker(t)
	pt.cameraSrc = &posesource.FakeCamera{} // no snapshot taken yet
	_, err := pt.DoCommand(context.Background(),
		map[string]interface{}{"capture_handeye_point": map[string]interface{}{"u": 1.0, "v": 1.0}})
	test.That(t, err, test.ShouldNotBeNil)
}
```

**Step 2: Run to verify fail** → FAIL.

**Step 3: Implement**

Dispatch case:
```go
	case has(cmd, "capture_handeye_point"):
		return pt.captureHandEyePoint(ctx, cmd["capture_handeye_point"])
```

Method (in `handeye.go`):
```go
// captureHandEyePoint deprojects the clicked pixel against the cached snapshot,
// reads the current TCP world pose, and stores the (world, camera) pair.
func (pt *teachTracker) captureHandEyePoint(ctx context.Context, raw interface{}) (map[string]interface{}, error) {
	if pt.cameraSrc == nil {
		return nil, errors.New("camera dependency not configured; cannot capture hand-eye point")
	}
	args, ok := raw.(map[string]interface{})
	if !ok {
		return nil, errors.New("capture_handeye_point args must be an object")
	}
	uF, ok := args["u"].(float64)
	if !ok {
		return nil, errors.New("capture_handeye_point requires a numeric 'u' (pixel column)")
	}
	vF, ok := args["v"].(float64)
	if !ok {
		return nil, errors.New("capture_handeye_point requires a numeric 'v' (pixel row)")
	}

	pt.snapshotMu.Lock()
	snap := pt.lastSnapshot
	pt.snapshotMu.Unlock()
	if snap == nil {
		return nil, errors.New("no snapshot cached; run handeye_snapshot before capturing")
	}

	camPt, err := frames.Deproject(snap.Intr, snap.Depth, int(uF), int(vF))
	if err != nil {
		return nil, err
	}

	pif, err := pt.source.Capture(ctx)
	if err != nil {
		return nil, err
	}
	worldPt := pif.Pose().Point()

	idx := pt.store.AddHandEyePair(frames.HandEyePair{World: worldPt, Camera: camPt})
	return map[string]interface{}{
		"index":      idx,
		"buffer_len": pt.store.HandEyeBufferLen(),
		"world":      vecToMap(worldPt),
		"camera":     vecToMap(camPt),
	}, nil
}

// vecToMap renders an r3.Vector as a plain xyz map for DoCommand responses.
func vecToMap(v r3.Vector) map[string]interface{} {
	return map[string]interface{}{"x": v.X, "y": v.Y, "z": v.Z}
}
```

**Step 4: Run to verify pass** → PASS.

**Step 5: Commit**

```bash
git add models/posetracker/handeye.go models/posetracker/docommand.go models/posetracker/docommand_test.go
git commit -m "feat(posetracker): add capture_handeye_point DoCommand"
```

---

## Task 7: `get_handeye_buffer` + `clear_handeye_buffer`

**Files:**
- Modify: `models/posetracker/handeye.go`, `models/posetracker/docommand.go`
- Test: `models/posetracker/docommand_test.go`

**Step 1: Write the failing test**

```go
func TestGetAndClearHandEyeBuffer(t *testing.T) {
	pt := newTestTracker(t)
	pt.store.AddHandEyePair(frames.HandEyePair{World: r3.Vector{X: 1}, Camera: r3.Vector{Z: 2}})

	got, err := pt.DoCommand(context.Background(), map[string]interface{}{"get_handeye_buffer": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
	pts := got["points"].([]interface{})
	test.That(t, len(pts), test.ShouldEqual, 1)

	cleared, err := pt.DoCommand(context.Background(), map[string]interface{}{"clear_handeye_buffer": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, cleared["cleared"], test.ShouldEqual, 1)
	test.That(t, pt.store.HandEyeBufferLen(), test.ShouldEqual, 0)
}
```

**Step 2: Run to verify fail** → FAIL.

**Step 3: Implement**

Dispatch cases:
```go
	case has(cmd, "get_handeye_buffer"):
		buf := pt.store.HandEyeBuffer()
		pts := make([]interface{}, len(buf))
		for i, p := range buf {
			pts[i] = map[string]interface{}{"world": vecToMap(p.World), "camera": vecToMap(p.Camera)}
		}
		return map[string]interface{}{"points": pts}, nil

	case has(cmd, "clear_handeye_buffer"):
		return map[string]interface{}{"cleared": pt.store.ClearHandEyeBuffer()}, nil
```

**Step 4: Run to verify pass** → PASS.

**Step 5: Commit**

```bash
git add models/posetracker/docommand.go models/posetracker/docommand_test.go
git commit -m "feat(posetracker): add get/clear handeye buffer DoCommands"
```

---

## Task 8: `solve_handeye` DoCommand

**Files:**
- Modify: `models/posetracker/handeye.go`, `models/posetracker/docommand.go`
- Test: `models/posetracker/docommand_test.go`

**Step 1: Write the failing test**

```go
func TestSolveHandEye(t *testing.T) {
	pt := newTestTracker(t) // persist is a *persist.Fake set on pt.persist
	// Identity transform: world == camera, so R=I, t=0.
	pts := []frames.HandEyePair{
		{Camera: r3.Vector{X: 0, Y: 0, Z: 0}, World: r3.Vector{X: 0, Y: 0, Z: 0}},
		{Camera: r3.Vector{X: 100, Y: 0, Z: 0}, World: r3.Vector{X: 100, Y: 0, Z: 0}},
		{Camera: r3.Vector{X: 0, Y: 100, Z: 0}, World: r3.Vector{X: 0, Y: 100, Z: 0}},
		{Camera: r3.Vector{X: 0, Y: 0, Z: 100}, World: r3.Vector{X: 0, Y: 0, Z: 100}},
	}
	for _, p := range pts {
		pt.store.AddHandEyePair(p)
	}
	pt.cameraName = "cam"

	resp, err := pt.DoCommand(context.Background(), map[string]interface{}{"solve_handeye": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, resp["committed"], test.ShouldBeTrue)
	test.That(t, resp["residual_rms"].(float64), test.ShouldBeLessThan, 1e-6)
	// Buffer cleared on success:
	test.That(t, pt.store.HandEyeBufferLen(), test.ShouldEqual, 0)
	// Persisted to the camera's frame, parent world:
	fake := pt.persist.(*persist.Fake)
	test.That(t, fake.SavedComponent, test.ShouldEqual, "cam")
	test.That(t, fake.SavedParent, test.ShouldEqual, "world")
	test.That(t, fake.SavedTranslation, test.ShouldNotBeNil)
}

func TestSolveHandEyePersistDisabledErrors(t *testing.T) {
	pt := newTestTracker(t)
	pt.persist = nil
	for i := 0; i < 3; i++ {
		pt.store.AddHandEyePair(frames.HandEyePair{})
	}
	_, err := pt.DoCommand(context.Background(), map[string]interface{}{"solve_handeye": map[string]interface{}{}})
	test.That(t, err, test.ShouldNotBeNil)
	// Buffer preserved on failure:
	test.That(t, pt.store.HandEyeBufferLen(), test.ShouldEqual, 3)
}
```

**Step 2: Run to verify fail** → FAIL.

**Step 3: Implement**

Dispatch case:
```go
	case has(cmd, "solve_handeye"):
		return pt.solveHandEye(ctx)
```

Method (in `handeye.go` — add imports `"go.viam.com/rdk/spatialmath"`):
```go
// solveHandEye runs the Kabsch camera->world solve over the buffer, persists the
// result to the camera component's own frame (parent world), reports the residual,
// and clears the buffer on success. On any failure the buffer is preserved.
func (pt *teachTracker) solveHandEye(ctx context.Context) (map[string]interface{}, error) {
	if pt.persist == nil {
		return nil, errors.New("persistence disabled (missing platform-API env vars); cannot commit camera calibration")
	}
	if pt.cameraName == "" {
		return nil, errors.New("camera dependency not configured; cannot solve hand-eye")
	}

	buf := pt.store.HandEyeBuffer()
	pose, residual, err := frames.ComputeCameraToWorld(buf)
	if err != nil {
		return nil, fmt.Errorf("hand-eye solve failed: %w", err)
	}

	// Serialize against other config-persisting commits.
	pt.commitMu.Lock()
	defer pt.commitMu.Unlock()

	translation := pose.Point()
	orientation := pose.Orientation()
	if perr := pt.persist.SaveComponentFrame(ctx, pt.cameraName, referenceframe.World, &translation, orientation); perr != nil {
		return nil, fmt.Errorf("persist failed, camera calibration not committed: %w", perr)
	}

	pt.store.ClearHandEyeBuffer()
	ov := orientation.OrientationVectorDegrees()
	return map[string]interface{}{
		"committed":    true,
		"pose":         poseToMap(pose),
		"residual_rms": residual,
		"parent":       referenceframe.World,
		"orientation":  map[string]interface{}{"o_x": ov.OX, "o_y": ov.OY, "o_z": ov.OZ, "theta": ov.Theta},
		_ = translation,
	}, nil
}
```

> Remove the stray `_ = translation` line above — it's a marker to remind you
> `translation` is already used in `SaveComponentFrame`. Return only the real
> keys. Add `"go.viam.com/rdk/referenceframe"` to the `handeye.go` imports.

**Step 4: Run to verify pass** → `go test ./models/posetracker/ -v` — PASS. Then full `go test ./...` and `make lint`.

**Step 5: Commit**

```bash
git add models/posetracker/handeye.go models/posetracker/docommand.go models/posetracker/docommand_test.go
git commit -m "feat(posetracker): add solve_handeye DoCommand"
```

---

## Task 9: Frontend builders + pixel-mapping helper

**Files:**
- Modify: `frontend/src/lib/poseTracker.ts`
- Test: `frontend/src/lib/poseTracker.test.ts`

**Step 1: Write the failing tests** (append to `poseTracker.test.ts`)

```ts
import {
  handeyeSnapshot, captureHandeyePoint, getHandeyeBuffer, clearHandeyeBuffer,
  solveHandeye, displayToNativePixel,
} from './poseTracker'

test('handeye command builders', () => {
  expect(handeyeSnapshot()).toEqual({ handeye_snapshot: {} })
  expect(captureHandeyePoint(12, 34)).toEqual({ capture_handeye_point: { u: 12, v: 34 } })
  expect(getHandeyeBuffer()).toEqual({ get_handeye_buffer: {} })
  expect(clearHandeyeBuffer()).toEqual({ clear_handeye_buffer: {} })
  expect(solveHandeye()).toEqual({ solve_handeye: {} })
})

test('displayToNativePixel maps click coords to source pixels', () => {
  // A 640x480 image rendered into a 320x240 box: a click at (160,120) → (320,240).
  expect(displayToNativePixel(160, 120, 320, 240, 640, 480)).toEqual({ u: 320, v: 240 })
  // Clamp to bounds.
  expect(displayToNativePixel(400, 300, 320, 240, 640, 480)).toEqual({ u: 639, v: 479 })
  expect(displayToNativePixel(-5, -5, 320, 240, 640, 480)).toEqual({ u: 0, v: 0 })
})
```

**Step 2: Run to verify fail** → `cd frontend && npm test -- poseTracker` — FAIL.

**Step 3: Implement** (append to `poseTracker.ts`)

```ts
export const handeyeSnapshot = () => ({ handeye_snapshot: {} })
export const captureHandeyePoint = (u: number, v: number) => ({ capture_handeye_point: { u, v } })
export const getHandeyeBuffer = () => ({ get_handeye_buffer: {} })
export const clearHandeyeBuffer = () => ({ clear_handeye_buffer: {} })
export const solveHandeye = () => ({ solve_handeye: {} })

// Maps a click at (displayX, displayY) inside an <img> of rendered size
// (clientW, clientH) to the native source pixel of an image (naturalW, naturalH).
// Assumes the image fills the box (object-fit: fill/contain with matching aspect).
// Result is clamped to valid pixel indices [0, natural-1].
export function displayToNativePixel(
  displayX: number, displayY: number,
  clientW: number, clientH: number,
  naturalW: number, naturalH: number,
): { u: number; v: number } {
  const u = Math.round((displayX / clientW) * naturalW)
  const v = Math.round((displayY / clientH) * naturalH)
  return {
    u: Math.max(0, Math.min(naturalW - 1, u)),
    v: Math.max(0, Math.min(naturalH - 1, v)),
  }
}

export interface HandEyePair { world: { x: number; y: number; z: number }; camera: { x: number; y: number; z: number } }
export interface HandEyeSnapshotResponse { image: string; width: number; height: number }
export interface CaptureHandEyeResponse { index: number; buffer_len: number; world: { x: number; y: number; z: number }; camera: { x: number; y: number; z: number } }
export interface HandEyeBufferResponse { points: HandEyePair[] }
export interface SolveHandEyeResponse {
  committed: boolean
  pose: PoseMap
  residual_rms: number
  parent: string
  orientation: { o_x: number; o_y: number; o_z: number; theta: number }
}
```

> The `<img>` will be styled `object-fit: fill` in Task 11 so the mapping in
> `displayToNativePixel` (a straight ratio, no letterbox offset) is exact. If you
> switch to `object-fit: contain`, this helper must add the letterbox offsets —
> keep them in lockstep.

**Step 4: Run to verify pass** → `npm test -- poseTracker` — PASS. Also `npm run check`.

**Step 5: Commit**

```bash
git add frontend/src/lib/poseTracker.ts frontend/src/lib/poseTracker.test.ts
git commit -m "feat(frontend): add hand-eye command builders and pixel-mapping helper"
```

---

## Task 10: `HandEyePanel.svelte`

**Files:**
- Create: `frontend/src/panels/HandEyePanel.svelte`

Model this on `TcpPanel.svelte` (same query/mutation patterns, panel chrome, and CSS). Key behaviors:

- **Gating:** attempt `handeye_snapshot`; if it errors with "camera dependency not configured", render `<p class="panel-warning">Requires a configured RGBD camera.</p>` and disable the controls. (There is no camera-presence flag in `get_arm_state`; deriving disabled-state from the snapshot error keeps parity with the backend's own gating.)
- **Frozen frame:** a **Refresh view** button runs `handeye_snapshot`; bind the returned base64 to `<img src={`data:image/jpeg;base64,${snapshot.image}`}>`. Store `snapshot.width/height` for pixel mapping.
- **Click to capture:** an `onclick` on the image wrapper reads `event.offsetX/offsetY` and the image's `clientWidth/clientHeight`, calls `displayToNativePixel(...)`, then `capture_handeye_point({u, v})`. On success push a dot marker at the *display* coords and refetch the buffer.
- **Buffer table:** world xyz / camera xyz per row (from `get_handeye_buffer`).
- **Solve:** a **Solve calibration** button runs `solve_handeye`; on success show `committed`, `residual_rms` (mm) with the same success styling as TcpPanel, and clear the overlay dots.

Full component:

```svelte
<script lang="ts">
  import { createResourceMutation, createResourceQuery } from '@viamrobotics/svelte-sdk'
  import { poseTrackerClient } from '../lib/clients'
  import { selectedResource } from '../lib/resource.svelte'
  import { useMachineId } from '../lib/machine'
  import {
    handeyeSnapshot, captureHandeyePoint, getHandeyeBuffer, clearHandeyeBuffer,
    solveHandeye, displayToNativePixel, toCommandArgs,
    type HandEyeSnapshotResponse, type HandEyeBufferResponse, type SolveHandEyeResponse,
  } from '../lib/poseTracker'

  const machineId = useMachineId()
  const pt = poseTrackerClient(machineId, () => selectedResource.name ?? '')

  const buffer = createResourceQuery(pt, 'doCommand', () => toCommandArgs(getHandeyeBuffer()), () => ({}))
  const snap = createResourceMutation(pt, 'doCommand')
  const capture = createResourceMutation(pt, 'doCommand')
  const clear = createResourceMutation(pt, 'doCommand')
  const solve = createResourceMutation(pt, 'doCommand')

  let snapshot = $state<HandEyeSnapshotResponse | undefined>(undefined)
  let dots = $state<{ x: number; y: number }[]>([])
  let solveResult = $state<SolveHandEyeResponse | undefined>(undefined)
  let noCamera = $state(false)
  let imgEl = $state<HTMLImageElement | undefined>(undefined)

  const points = $derived((buffer.data as HandEyeBufferResponse | undefined)?.points ?? [])

  async function handleRefresh() {
    solveResult = undefined
    try {
      const resp = (await snap.mutateAsync(toCommandArgs(handeyeSnapshot()))) as unknown as HandEyeSnapshotResponse
      snapshot = resp
      noCamera = false
    } catch (err) {
      if (err instanceof Error && /camera dependency not configured/.test(err.message)) noCamera = true
    }
  }

  async function handleImageClick(event: MouseEvent) {
    if (!snapshot || !imgEl) return
    const { u, v } = displayToNativePixel(
      event.offsetX, event.offsetY, imgEl.clientWidth, imgEl.clientHeight, snapshot.width, snapshot.height,
    )
    try {
      await capture.mutateAsync(toCommandArgs(captureHandeyePoint(u, v)))
      dots = [...dots, { x: event.offsetX, y: event.offsetY }]
      await buffer.refetch()
    } catch {
      // Surfaced via capture.error.
    }
  }

  async function handleClear() {
    try {
      await clear.mutateAsync(toCommandArgs(clearHandeyeBuffer()))
      dots = []
      await buffer.refetch()
    } catch {
      // Surfaced via clear.error.
    }
  }

  async function handleSolve() {
    solveResult = undefined
    try {
      const resp = (await solve.mutateAsync(toCommandArgs(solveHandeye()))) as unknown as SolveHandEyeResponse
      solveResult = resp
      dots = []
      await buffer.refetch()
    } catch {
      // Surfaced via solve.error.
    }
  }

  function fmt(n: number): string {
    return n.toFixed(2)
  }
</script>

<section class="handeye-panel">
  <header class="panel-header">
    <div class="panel-titles">
      <h2>Camera Calibration</h2>
      <p class="panel-subtitle">Eye-to-hand: touch points with the TCP, click them in the camera view, solve camera&nbsp;&rarr;&nbsp;world.</p>
    </div>
    <div class="panel-actions">
      <button type="button" onclick={handleRefresh} disabled={snap.isPending}>
        {snap.isPending ? 'Refreshing…' : 'Refresh view'}
      </button>
    </div>
  </header>

  {#if noCamera}
    <p class="panel-warning">Requires a configured RGBD camera.</p>
  {/if}

  {#if snap.error && !noCamera}
    <p class="error">{snap.error.message}</p>
  {/if}
  {#if capture.error}
    <p class="error">{capture.error.message}</p>
  {/if}

  {#if snapshot}
    <div class="view">
      <!-- svelte-ignore a11y_click_events_have_key_events a11y_no_noninteractive_element_interactions -->
      <img
        bind:this={imgEl}
        src={`data:image/jpeg;base64,${snapshot.image}`}
        alt="Camera view — click a touched point"
        onclick={handleImageClick}
      />
      {#each dots as dot, i (i)}
        <span class="dot" style={`left:${dot.x}px; top:${dot.y}px`}></span>
      {/each}
    </div>
  {:else if !noCamera}
    <p class="panel-empty">Refresh the view, jog the TCP to touch a point, then click it in the image.</p>
  {/if}

  <div class="handeye-actions">
    <button type="button" class="secondary" onclick={handleClear} disabled={clear.isPending || points.length === 0}>Clear points</button>
    <button type="button" onclick={handleSolve} disabled={solve.isPending || points.length < 3}>
      {solve.isPending ? 'Solving…' : 'Solve calibration'}
    </button>
  </div>

  {#if solve.error}
    <p class="error">{solve.error.message}</p>
  {/if}
  {#if solveResult}
    <p class="success">Committed · residual RMS {fmt(solveResult.residual_rms)} mm</p>
  {/if}

  {#if points.length > 0}
    <div class="table-scroll">
      <table>
        <thead>
          <tr><th>#</th><th>World XYZ</th><th>Camera XYZ</th></tr>
        </thead>
        <tbody>
          {#each points as p, i (i)}
            <tr>
              <td>{i}</td>
              <td>{fmt(p.world.x)}, {fmt(p.world.y)}, {fmt(p.world.z)}</td>
              <td>{fmt(p.camera.x)}, {fmt(p.camera.y)}, {fmt(p.camera.z)}</td>
            </tr>
          {/each}
        </tbody>
      </table>
    </div>
  {/if}
</section>

<style>
  .handeye-panel { display: flex; flex-direction: column; gap: 1rem; padding: 1rem; }
  .view { position: relative; display: inline-block; max-width: 100%; }
  .view img { display: block; max-width: 100%; height: auto; object-fit: fill; cursor: crosshair; border-radius: var(--radius-md); }
  .dot {
    position: absolute; width: 10px; height: 10px; margin: -5px 0 0 -5px;
    border-radius: 50%; background: var(--accent); border: 2px solid var(--ink-on-accent); pointer-events: none;
  }
  .handeye-actions { display: flex; gap: 0.5rem; flex-wrap: wrap; }
  button { min-height: 44px; padding: 0.5rem 1.1rem; border-radius: var(--radius-md); border: 1px solid var(--border-control); background: var(--accent); color: var(--ink-on-accent); cursor: pointer; }
  button.secondary { background: var(--surface-control); color: inherit; }
  button:disabled { opacity: 0.5; cursor: not-allowed; }
  table { border-collapse: collapse; width: 100%; font-variant-numeric: tabular-nums; font-size: 0.9rem; }
  th, td { padding: 0.35rem 0.6rem; text-align: right; border-bottom: 1px solid var(--border-panel); }
  th:first-child, td:first-child { text-align: left; }
  .success { color: var(--success); font-size: 0.9rem; }
  .error { color: var(--error-text); font-size: 0.85rem; }
</style>
```

> **`object-fit: fill`** is deliberate — it makes `displayToNativePixel` a pure
> ratio with no letterbox math. Because `height: auto` preserves aspect anyway,
> `fill` and `contain` render identically here, but `fill` keeps the click math
> honest even if the box is later constrained.

**Verify:** `cd frontend && npm run check` (types) — PASS. No unit test for the Svelte component itself (logic lives in the tested `poseTracker.ts` helpers).

**Commit:**

```bash
git add frontend/src/panels/HandEyePanel.svelte
git commit -m "feat(frontend): add HandEyePanel for camera calibration"
```

---

## Task 11: Mount `HandEyePanel` in the app shell

**Files:**
- Modify: `frontend/src/App.svelte`

**Step 1:** Import and render it in the `side-region`, after `TcpPanel`:

```svelte
  import HandEyePanel from './panels/HandEyePanel.svelte'
```
```svelte
            <section class="panel">
              <TcpPanel />
            </section>
            <section class="panel">
              <HandEyePanel />
            </section>
```

**Step 2:** `cd frontend && npm run check && npm run build` — PASS.

**Step 3: Commit**

```bash
git add frontend/src/App.svelte
git commit -m "feat(frontend): mount HandEyePanel in the pendant layout"
```

---

## Task 12: Documentation

**Files:**
- Modify: `README.md`

Update these sections:
1. **Overview / models** — note the new optional `camera` attribute on `pose-tracker`.
2. **Config attributes table** — add the `camera` row (type string, no, —, "Name of the RGBD camera to calibrate. Enables the hand-eye DoCommands when set.").
3. **DoCommand reference table** — add rows for `handeye_snapshot`, `capture_handeye_point`, `get_handeye_buffer`, `clear_handeye_buffer`, `solve_handeye` with their payloads and response keys (from Tasks 5–8).
4. **New "Hand-eye calibration (eye-to-hand)" section** — explain the touch+click procedure, the RGBD requirement, the TCP-calibration prerequisite, the ≥3-non-collinear / 4+-recommended point guidance, `residual_rms` as the trust signal, and that the result persists to `camera.frame` (parent `world`) via the same credentials as frame teaching.
5. **Teach Pendant application** — note the new Camera Calibration panel and that it needs the `camera` attribute set.
6. **Manual integration checklist** — add a hand-eye subsection: configure an RGBD camera, run the touch/click loop for 4+ spread points, solve, confirm `committed` + a low `residual_rms`, verify the camera component's `frame` was patched in config, and confirm the camera renders at the correct world location in the visualizer.

**Commit:**

```bash
git add README.md
git commit -m "docs: document eye-to-hand hand-eye calibration"
```

---

## Final verification

```bash
make test          # all Go tests
make lint          # go vet + gofmt
cd frontend && npm run check && npm test && npm run build
```

All green → the feature is complete. Optionally run `make module` to confirm the bundled build still packages.

## Out of scope (tracked for later)

- Eye-in-hand configuration (reuses `Deproject`; needs an AX=XB-style solver, parent = flange).
- Fiducial/AX=XB solving for full 6-DOF robustness.
- Live camera streaming (frozen snapshots only).
- Automatic point detection (manual pick only).
