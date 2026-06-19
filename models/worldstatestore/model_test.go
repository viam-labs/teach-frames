package worldstatestore

import (
	"context"
	"testing"
	"time"

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

func TestStreamClosesOnContextCancel(t *testing.T) {
	svc := newForTest(t, inject.NewPoseTracker("test-pt"))
	ctx, cancel := context.WithCancel(context.Background())
	stream, err := svc.StreamTransformChanges(ctx, nil)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, stream, test.ShouldNotBeNil)
	cancel()
	_, nextErr := stream.Next()
	test.That(t, nextErr, test.ShouldNotBeNil) // ctx.Err() or io.EOF after cancel
}

func TestStreamBlocksWhileContextAlive(t *testing.T) {
	svc := newForTest(t, inject.NewPoseTracker("test-pt"))
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	stream, err := svc.StreamTransformChanges(ctx, nil)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, stream, test.ShouldNotBeNil)

	done := make(chan struct{})
	go func() {
		stream.Next() //nolint:errcheck
		close(done)
	}()

	select {
	case <-done:
		t.Fatal("stream.Next() returned before context was cancelled; expected it to block")
	case <-time.After(50 * time.Millisecond):
		// Good: Next is still blocking after 50 ms with a live context.
	}
}
