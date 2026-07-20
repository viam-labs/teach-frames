package frames

import (
	"image"
	"math"
	"math/rand"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/rimage"
	"go.viam.com/rdk/rimage/transform"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
)

func TestComputeRigidTransformRoundTrip(t *testing.T) {
	// Known camera->world transform: rotate 90° about Z, translate (100,-50,300).
	rot := spatialmath.NewPoseFromOrientation(&spatialmath.EulerAngles{Yaw: math.Pi / 2})
	truth := spatialmath.NewPose(r3.Vector{X: 100, Y: -50, Z: 300}, rot.Orientation())

	camPts := []r3.Vector{{X: 0, Y: 0, Z: 0}, {X: 100, Y: 0, Z: 0}, {X: 0, Y: 100, Z: 0}, {X: 0, Y: 0, Z: 100}}
	pairs := make([]PointPair, len(camPts))
	for i, c := range camPts {
		w := spatialmath.Compose(truth, spatialmath.NewPoseFromPoint(c)).Point()
		pairs[i] = PointPair{Camera: c, Reference: w}
	}

	got, residual, err := ComputeRigidTransform(pairs)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, residual, test.ShouldBeLessThan, 1e-6)
	test.That(t, spatialmath.PoseAlmostEqualEps(got, truth, 1e-6), test.ShouldBeTrue)
}

// degenerateTruth is a non-identity transform used by the degeneracy tests, so a
// solver that wrongly returns identity cannot accidentally look correct.
func degenerateTruth() spatialmath.Pose {
	return spatialmath.NewPose(
		r3.Vector{X: 100, Y: -50, Z: 300},
		&spatialmath.EulerAngles{Roll: 0.4, Pitch: 0.3, Yaw: 0.65},
	)
}

// TestComputeRigidTransformRejectsCoincidentCameraPoints covers the case where
// every camera point is identical. The centered camera matrix is then all zeros,
// so the relative collinearity test reduces to 0 < 1e-6*0, which is false — it
// cannot catch this on its own.
func TestComputeRigidTransformRejectsCoincidentCameraPoints(t *testing.T) {
	truth := degenerateTruth()
	c := r3.Vector{X: 10, Y: 20, Z: 500}
	w := spatialmath.Compose(truth, spatialmath.NewPoseFromPoint(c)).Point()

	pairs := make([]PointPair, 4)
	for i := range pairs {
		pairs[i] = PointPair{Camera: c, Reference: w}
	}

	_, _, err := ComputeRigidTransform(pairs)
	test.That(t, err, test.ShouldNotBeNil)
}

// TestComputeRigidTransformRejectsReferenceSideDegeneracy is the important one.
// When the world points are identical but the camera points scatter by sensor
// noise, w-refMean is zero for every pair, so H is exactly zero and the solve
// returns identity rotation regardless of the truth. The camera cloud looks
// perfectly healthy, so no camera-side check can see this. The residual is no
// help either: it reports ~1mm, which reads as an excellent calibration while
// the answer is hundreds of mm wrong.
//
// The noise is load-bearing. Without it the camera points coincide too, and the
// case would be caught by a camera-side check alone — proving nothing.
func TestComputeRigidTransformRejectsReferenceSideDegeneracy(t *testing.T) {
	truth := degenerateTruth()
	rng := rand.New(rand.NewSource(42))
	baseCam := r3.Vector{X: 10, Y: 20, Z: 500}
	w := spatialmath.Compose(truth, spatialmath.NewPoseFromPoint(baseCam)).Point()

	pairs := make([]PointPair, 4)
	for i := range pairs {
		pairs[i] = PointPair{
			Camera: r3.Vector{
				X: baseCam.X + rng.NormFloat64(),
				Y: baseCam.Y + rng.NormFloat64(),
				Z: baseCam.Z + rng.NormFloat64(),
			},
			Reference: w, // identical for every pair: the arm never moved
		}
	}

	_, _, err := ComputeRigidTransform(pairs)
	test.That(t, err, test.ShouldNotBeNil)
}

// buildJoggedPairs models the real geometry: a fixed target viewed from n flange
// poses that differ by jog mm, with sigma mm of camera noise. A jog of 0 means
// the operator never moved the arm.
func buildJoggedPairs(truth spatialmath.Pose, jog, sigma float64, n int, seed int64) []PointPair {
	rng := rand.New(rand.NewSource(seed))
	pairs := make([]PointPair, n)
	base := r3.Vector{X: 10, Y: 20, Z: 500}
	for i := 0; i < n; i++ {
		c := r3.Vector{
			X: base.X + jog*float64(i%3),
			Y: base.Y + jog*float64((i/3)%3),
			Z: base.Z + jog*float64(i%2),
		}
		w := spatialmath.Compose(truth, spatialmath.NewPoseFromPoint(c)).Point()
		pairs[i] = PointPair{
			Camera: r3.Vector{
				X: c.X + rng.NormFloat64()*sigma,
				Y: c.Y + rng.NormFloat64()*sigma,
				Z: c.Z + rng.NormFloat64()*sigma,
			},
			Reference: w,
		}
	}
	return pairs
}

// TestComputeRigidTransformRejectsNearDegenerateSpread covers the hardware form of
// the failure. An arm that was never deliberately jogged still reports slightly
// different poses (encoder quantization), so the world spread is small but not
// zero — which any absolute floor conservative enough to be safe would miss.
func TestComputeRigidTransformRejectsNearDegenerateSpread(t *testing.T) {
	truth := degenerateTruth()
	for _, jog := range []float64{0.01, 0.1, 1.0} {
		pairs := buildJoggedPairs(truth, jog, 1.0, 8, 7)
		_, _, err := ComputeRigidTransform(pairs)
		test.That(t, err, test.ShouldNotBeNil)
	}
}

// TestComputeRigidTransformAcceptsRealisticSpread is the guard on the guard: real
// calibration data must still be accepted, and must still solve accurately.
func TestComputeRigidTransformAcceptsRealisticSpread(t *testing.T) {
	truth := degenerateTruth()
	for _, jog := range []float64{10.0, 70.0} {
		pairs := buildJoggedPairs(truth, jog, 1.0, 8, 7)
		got, _, err := ComputeRigidTransform(pairs)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, got.Point().Sub(truth.Point()).Norm(), test.ShouldBeLessThan, 40.0)
	}
}

// rotDet returns the determinant of an orientation's 3x3 rotation matrix
// (+1 for a proper right-handed rotation, -1 for a reflection).
func rotDet(o spatialmath.Orientation) float64 {
	rm := o.RotationMatrix()
	return rm.Row(0).Cross(rm.Row(1)).Dot(rm.Row(2))
}

func TestComputeRigidTransformCoplanarAccepted(t *testing.T) {
	// A general (out-of-plane) rotation + translation. 3 non-collinear but
	// COPLANAR camera points (all Z=0) fully determine the rigid transform.
	rot := spatialmath.NewPoseFromOrientation(&spatialmath.EulerAngles{Roll: 0.3, Pitch: -0.4, Yaw: 1.1})
	truth := spatialmath.NewPose(r3.Vector{X: 30, Y: 60, Z: -20}, rot.Orientation())

	camPts := []r3.Vector{{X: 0, Y: 0, Z: 0}, {X: 100, Y: 0, Z: 0}, {X: 0, Y: 100, Z: 0}}
	pairs := make([]PointPair, len(camPts))
	for i, c := range camPts {
		w := spatialmath.Compose(truth, spatialmath.NewPoseFromPoint(c)).Point()
		pairs[i] = PointPair{Camera: c, Reference: w}
	}

	got, residual, err := ComputeRigidTransform(pairs)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, residual, test.ShouldBeLessThan, 1e-6)
	test.That(t, spatialmath.PoseAlmostEqualEps(got, truth, 1e-6), test.ShouldBeTrue)
	test.That(t, rotDet(got.Orientation()), test.ShouldAlmostEqual, 1.0)
}

func TestComputeRigidTransformNoisyResidual(t *testing.T) {
	// Same truth as the round-trip test, but with one world point perturbed by
	// 5 mm. The solve must succeed and report a sensible POSITIVE residual.
	rot := spatialmath.NewPoseFromOrientation(&spatialmath.EulerAngles{Yaw: math.Pi / 2})
	truth := spatialmath.NewPose(r3.Vector{X: 100, Y: -50, Z: 300}, rot.Orientation())

	camPts := []r3.Vector{{X: 0, Y: 0, Z: 0}, {X: 100, Y: 0, Z: 0}, {X: 0, Y: 100, Z: 0}, {X: 0, Y: 0, Z: 100}}
	pairs := make([]PointPair, len(camPts))
	for i, c := range camPts {
		w := spatialmath.Compose(truth, spatialmath.NewPoseFromPoint(c)).Point()
		pairs[i] = PointPair{Camera: c, Reference: w}
	}
	pairs[1].Reference = pairs[1].Reference.Add(r3.Vector{X: 5}) // inject 5 mm of noise

	_, residual, err := ComputeRigidTransform(pairs)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, residual, test.ShouldBeGreaterThan, 0.1)
	test.That(t, residual, test.ShouldBeLessThan, 5.0)
}

func TestComputeRigidTransformReflectionCorrected(t *testing.T) {
	// World points are a mirror image of the camera points (Z negated). The raw
	// SVD best-fit orthogonal matrix here is a reflection (det -1); the diag(1,1,d)
	// correction must return the nearest PROPER rotation (det +1) instead.
	camPts := []r3.Vector{{X: 0, Y: 0, Z: 0}, {X: 100, Y: 0, Z: 0}, {X: 0, Y: 100, Z: 0}, {X: 0, Y: 0, Z: 100}}
	pairs := make([]PointPair, len(camPts))
	for i, c := range camPts {
		pairs[i] = PointPair{Camera: c, Reference: r3.Vector{X: c.X, Y: c.Y, Z: -c.Z}}
	}

	got, residual, err := ComputeRigidTransform(pairs)
	test.That(t, err, test.ShouldBeNil)
	// A proper rotation cannot fit a reflection, so residual is non-trivial.
	test.That(t, residual, test.ShouldBeGreaterThan, 0.0)
	// The result must be a valid right-handed rotation, not a reflection.
	test.That(t, rotDet(got.Orientation()), test.ShouldAlmostEqual, 1.0)
}

func TestComputeRigidTransformRejectsCollinear(t *testing.T) {
	pairs := []PointPair{
		{Camera: r3.Vector{X: 0}, Reference: r3.Vector{X: 0}},
		{Camera: r3.Vector{X: 1}, Reference: r3.Vector{X: 1}},
		{Camera: r3.Vector{X: 2}, Reference: r3.Vector{X: 2}},
	}
	_, _, err := ComputeRigidTransform(pairs)
	test.That(t, err, test.ShouldNotBeNil)
}

func TestComputeRigidTransformRejectsTooFew(t *testing.T) {
	_, _, err := ComputeRigidTransform([]PointPair{{}, {}})
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
	pt := image.Pt(200, 10) // column beyond the 100px width
	_, err := Deproject(intr, dm, pt.X, pt.Y)
	test.That(t, err, test.ShouldNotBeNil)
}
