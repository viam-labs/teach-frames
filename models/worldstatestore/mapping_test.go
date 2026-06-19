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

func TestUUIDIsSixteenBytes(t *testing.T) {
	u := uuidForName("some_frame")
	test.That(t, len(u), test.ShouldEqual, 16)
}

func TestUUIDIsDeterministicAcrossCalls(t *testing.T) {
	name := "deterministic_frame"
	first := uuidForName(name)
	second := uuidForName(name)
	third := uuidForName(name)
	test.That(t, first, test.ShouldResemble, second)
	test.That(t, second, test.ShouldResemble, third)
}

func TestFrameToTransformSetsAxesHelper(t *testing.T) {
	pif := referenceframe.NewPoseInFrame("world", spatialmath.NewPoseFromPoint(r3.Vector{X: 1}))
	tf := frameToTransform("fixture_a", pif)
	test.That(t, tf.ReferenceFrame, test.ShouldEqual, "fixture_a")
	test.That(t, tf.Metadata.Fields["showAxesHelper"].GetBoolValue(), test.ShouldBeTrue)
}

func TestFrameToTransformSetsUUID(t *testing.T) {
	pif := referenceframe.NewPoseInFrame("world", spatialmath.NewPoseFromPoint(r3.Vector{X: 1}))
	tf := frameToTransform("fixture_a", pif)
	test.That(t, tf.Uuid, test.ShouldResemble, uuidForName("fixture_a"))
}

func TestFrameToTransformHasNonNilPose(t *testing.T) {
	pif := referenceframe.NewPoseInFrame("world", spatialmath.NewPoseFromPoint(r3.Vector{X: 1, Y: 2, Z: 3}))
	tf := frameToTransform("my_frame", pif)
	test.That(t, tf.PoseInObserverFrame, test.ShouldNotBeNil)
	test.That(t, tf.PoseInObserverFrame.Pose.X, test.ShouldAlmostEqual, 1.0)
}
