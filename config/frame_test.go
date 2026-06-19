package config

import (
	"encoding/json"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
)

func TestFrameSpecRoundTrip(t *testing.T) {
	pose := spatialmath.NewPose(
		r3.Vector{X: 0.1, Y: 0.2, Z: 0.3},
		&spatialmath.OrientationVectorDegrees{OX: 0, OY: 0, OZ: 1, Theta: 35},
	)
	spec := FrameSpecFromPose("fixture_a", "3point", "world", pose)
	got, err := spec.ToPose()
	test.That(t, err, test.ShouldBeNil)
	test.That(t, spatialmath.PoseAlmostEqual(got, pose), test.ShouldBeTrue)
	test.That(t, spec.Name, test.ShouldEqual, "fixture_a")
	test.That(t, spec.Method, test.ShouldEqual, "3point")
	test.That(t, spec.Parent, test.ShouldEqual, "world")
}

func TestFrameSpecJSONRoundTrip(t *testing.T) {
	pose := spatialmath.NewPose(
		r3.Vector{X: 1.5, Y: -2.7, Z: 0.0},
		&spatialmath.OrientationVectorDegrees{OX: 0, OY: 1, OZ: 0, Theta: 90},
	)
	original := FrameSpecFromPose("fixture_b", "point", "base", pose)

	// Marshal to JSON.
	data, err := json.Marshal(original)
	test.That(t, err, test.ShouldBeNil)

	// Unmarshal back.
	var restored FrameSpec
	err = json.Unmarshal(data, &restored)
	test.That(t, err, test.ShouldBeNil)

	// Metadata preserved.
	test.That(t, restored.Name, test.ShouldEqual, original.Name)
	test.That(t, restored.Method, test.ShouldEqual, original.Method)
	test.That(t, restored.Parent, test.ShouldEqual, original.Parent)

	// Pose round-trips within float tolerance.
	restoredPose, err := restored.ToPose()
	test.That(t, err, test.ShouldBeNil)
	test.That(t, spatialmath.PoseAlmostEqual(restoredPose, pose), test.ShouldBeTrue)
}
