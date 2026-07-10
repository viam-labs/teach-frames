package frames

import (
	"math"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
)

// makeFlangePose returns a flange pose (given orientation, at the given
// base-frame flange origin) such that a tool tip `tip` in the flange frame lands
// on the fixed base point `target`. It solves p = target - R*tip so every
// synthesized capture is consistent with one tip and one touched point. R*tip
// is computed via spatialmath.Compose (not by hand-multiplying
// RotationMatrix() columns) so the synthesized pose matches how the SDK
// actually composes real captured poses.
func makeFlangePose(ov *spatialmath.OrientationVectorDegrees, tip, target r3.Vector) spatialmath.Pose {
	rotationOnly := spatialmath.NewPoseFromOrientation(ov)
	rTip := spatialmath.Compose(rotationOnly, spatialmath.NewPoseFromPoint(tip)).Point()
	p := target.Sub(rTip)
	return spatialmath.NewPose(p, ov)
}

func TestComputePivotTCPRecoversTip(t *testing.T) {
	tip := r3.Vector{X: 10, Y: -5, Z: 120}
	target := r3.Vector{X: 400, Y: 0, Z: 300}
	// Orientations must tilt the flange Z axis away from the base Z axis (not
	// just spin about it): with OX=OY=0, R's third column is always (0,0,1),
	// which leaves the tip's Z-offset and the common point's Z-coordinate
	// linearly coupled (only their difference is constrained) and the 6x6
	// least-squares system rank-deficient. Nonzero OX/OY breaks that coupling.
	poses := []spatialmath.Pose{
		makeFlangePose(&spatialmath.OrientationVectorDegrees{OZ: 1, Theta: 0}, tip, target),
		makeFlangePose(&spatialmath.OrientationVectorDegrees{OX: 0.3, OZ: 1, Theta: 30}, tip, target),
		makeFlangePose(&spatialmath.OrientationVectorDegrees{OY: 0.3, OZ: 1, Theta: 75}, tip, target),
		makeFlangePose(&spatialmath.OrientationVectorDegrees{OX: -0.2, OY: 0.2, OZ: 1, Theta: 150}, tip, target),
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
