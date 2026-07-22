package worldstatestore

import (
	"testing"

	"github.com/golang/geo/r3"
	pb "go.viam.com/api/service/worldstatestore/v1"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
)

func TestDiffTransformsPureAdd(t *testing.T) {
	prev := referenceframe.FrameSystemPoses{}
	cur := referenceframe.FrameSystemPoses{
		"a": referenceframe.NewPoseInFrame("world", spatialmath.NewPoseFromPoint(r3.Vector{X: 1})),
	}

	changes := diffTransforms(prev, cur)

	test.That(t, len(changes), test.ShouldEqual, 1)
	test.That(t, changes[0].ChangeType, test.ShouldEqual, pb.TransformChangeType_TRANSFORM_CHANGE_TYPE_ADDED)
	test.That(t, changes[0].Transform.ReferenceFrame, test.ShouldEqual, "a")
	test.That(t, changes[0].Transform.Uuid, test.ShouldResemble, uuidForName("a"))
}

func TestDiffTransformsPureRemove(t *testing.T) {
	prev := referenceframe.FrameSystemPoses{
		"a": referenceframe.NewPoseInFrame("world", spatialmath.NewPoseFromPoint(r3.Vector{X: 1})),
	}
	cur := referenceframe.FrameSystemPoses{}

	changes := diffTransforms(prev, cur)

	test.That(t, len(changes), test.ShouldEqual, 1)
	test.That(t, changes[0].ChangeType, test.ShouldEqual, pb.TransformChangeType_TRANSFORM_CHANGE_TYPE_REMOVED)
	test.That(t, changes[0].Transform.ReferenceFrame, test.ShouldEqual, "a")
	test.That(t, changes[0].Transform.Uuid, test.ShouldResemble, uuidForName("a"))
}

func TestDiffTransformsPoseChangeIsUpdated(t *testing.T) {
	prev := referenceframe.FrameSystemPoses{
		"a": referenceframe.NewPoseInFrame("world", spatialmath.NewPoseFromPoint(r3.Vector{X: 1})),
	}
	cur := referenceframe.FrameSystemPoses{
		"a": referenceframe.NewPoseInFrame("world", spatialmath.NewPoseFromPoint(r3.Vector{X: 2})),
	}

	changes := diffTransforms(prev, cur)

	test.That(t, len(changes), test.ShouldEqual, 1)
	test.That(t, changes[0].ChangeType, test.ShouldEqual, pb.TransformChangeType_TRANSFORM_CHANGE_TYPE_UPDATED)
	test.That(t, changes[0].Transform.ReferenceFrame, test.ShouldEqual, "a")
	test.That(t, changes[0].UpdatedFields, test.ShouldResemble, []string{"pose_in_observer_frame"})
}

func TestDiffTransformsParentChangeIsUpdated(t *testing.T) {
	prev := referenceframe.FrameSystemPoses{
		"a": referenceframe.NewPoseInFrame("world", spatialmath.NewPoseFromPoint(r3.Vector{X: 1})),
	}
	cur := referenceframe.FrameSystemPoses{
		"a": referenceframe.NewPoseInFrame("gripper", spatialmath.NewPoseFromPoint(r3.Vector{X: 1})),
	}

	changes := diffTransforms(prev, cur)

	test.That(t, len(changes), test.ShouldEqual, 1)
	test.That(t, changes[0].ChangeType, test.ShouldEqual, pb.TransformChangeType_TRANSFORM_CHANGE_TYPE_UPDATED)
	test.That(t, changes[0].UpdatedFields, test.ShouldResemble, []string{"pose_in_observer_frame"})
}

func TestDiffTransformsUnchangedIsEmpty(t *testing.T) {
	prev := referenceframe.FrameSystemPoses{
		"a": referenceframe.NewPoseInFrame("world", spatialmath.NewPoseFromPoint(r3.Vector{X: 1})),
	}
	cur := referenceframe.FrameSystemPoses{
		"a": referenceframe.NewPoseInFrame("world", spatialmath.NewPoseFromPoint(r3.Vector{X: 1})),
	}

	changes := diffTransforms(prev, cur)

	test.That(t, len(changes), test.ShouldEqual, 0)
}

func TestDiffTransformsMixed(t *testing.T) {
	prev := referenceframe.FrameSystemPoses{
		"unchanged": referenceframe.NewPoseInFrame("world", spatialmath.NewPoseFromPoint(r3.Vector{X: 1})),
		"updated":   referenceframe.NewPoseInFrame("world", spatialmath.NewPoseFromPoint(r3.Vector{X: 1})),
		"removed":   referenceframe.NewPoseInFrame("world", spatialmath.NewPoseFromPoint(r3.Vector{X: 1})),
	}
	cur := referenceframe.FrameSystemPoses{
		"unchanged": referenceframe.NewPoseInFrame("world", spatialmath.NewPoseFromPoint(r3.Vector{X: 1})),
		"updated":   referenceframe.NewPoseInFrame("world", spatialmath.NewPoseFromPoint(r3.Vector{X: 5})),
		"added":     referenceframe.NewPoseInFrame("world", spatialmath.NewPoseFromPoint(r3.Vector{X: 9})),
	}

	changes := diffTransforms(prev, cur)

	byType := map[pb.TransformChangeType]int{}
	for _, c := range changes {
		byType[c.ChangeType]++
	}
	test.That(t, len(changes), test.ShouldEqual, 3)
	test.That(t, byType[pb.TransformChangeType_TRANSFORM_CHANGE_TYPE_ADDED], test.ShouldEqual, 1)
	test.That(t, byType[pb.TransformChangeType_TRANSFORM_CHANGE_TYPE_REMOVED], test.ShouldEqual, 1)
	test.That(t, byType[pb.TransformChangeType_TRANSFORM_CHANGE_TYPE_UPDATED], test.ShouldEqual, 1)
}
