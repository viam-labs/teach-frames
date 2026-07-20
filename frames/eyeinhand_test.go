package frames

import (
	"math"
	"math/rand"
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
