package worldstatestore

import (
	"context"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/testutils/inject"
	"go.viam.com/test"
)

// newForTest returns a mirror with test-scoped logger and the given PoseTracker.
func newForTest(t *testing.T, pt *inject.PoseTracker) *mirror {
	t.Helper()
	return &mirror{
		logger: logging.NewTestLogger(t),
		pt:     pt,
	}
}

// fakePTWith returns an inject.PoseTracker whose Poses always returns the given map.
func fakePTWith(poses referenceframe.FrameSystemPoses) *inject.PoseTracker {
	pt := inject.NewPoseTracker("test-pt")
	pt.PosesFunc = func(ctx context.Context, bodyNames []string, extra map[string]interface{}) (referenceframe.FrameSystemPoses, error) {
		return poses, nil
	}
	return pt
}

func TestListAndGetTransform(t *testing.T) {
	poses := referenceframe.FrameSystemPoses{
		"a": referenceframe.NewPoseInFrame("world", spatialmath.NewPoseFromPoint(r3.Vector{X: 1})),
	}
	svc := newForTest(t, fakePTWith(poses))

	uuids, err := svc.ListUUIDs(context.Background(), nil)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(uuids), test.ShouldEqual, 1)

	tf, err := svc.GetTransform(context.Background(), uuidForName("a"), nil)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, tf.ReferenceFrame, test.ShouldEqual, "a")
}

func TestGetTransformUnknownUUID(t *testing.T) {
	svc := newForTest(t, fakePTWith(referenceframe.FrameSystemPoses{}))
	_, err := svc.GetTransform(context.Background(), uuidForName("nope"), nil)
	test.That(t, err, test.ShouldNotBeNil)
}
