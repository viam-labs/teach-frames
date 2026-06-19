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
	f := &Fake{Poses: []spatialmath.Pose{
		spatialmath.NewPoseFromPoint(r3.Vector{X: 1}),
		spatialmath.NewPoseFromPoint(r3.Vector{X: 2}),
	}}
	pif, err := f.Capture(context.Background())
	test.That(t, err, test.ShouldBeNil)
	test.That(t, pif.Parent(), test.ShouldEqual, referenceframe.World)
	test.That(t, pif.Pose().Point().X, test.ShouldEqual, 1.0)

	pif2, err := f.Capture(context.Background())
	test.That(t, err, test.ShouldBeNil)
	test.That(t, pif2.Pose().Point().X, test.ShouldEqual, 2.0)
}

func TestFakeSourceExhausted(t *testing.T) {
	f := &Fake{}
	_, err := f.Capture(context.Background())
	test.That(t, err, test.ShouldNotBeNil)
}

func TestMotionSourceImplementsInterface(t *testing.T) {
	var _ PoseSource = (*MotionSource)(nil)
	var _ PoseSource = (*Fake)(nil)
}
