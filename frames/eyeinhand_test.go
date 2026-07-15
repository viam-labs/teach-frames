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
