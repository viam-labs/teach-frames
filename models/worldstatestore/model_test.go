package worldstatestore

import (
	"context"
	"sync/atomic"
	"testing"
	"time"

	"github.com/golang/geo/r3"
	pb "go.viam.com/api/service/worldstatestore/v1"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/services/worldstatestore"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/testutils/inject"
	"go.viam.com/test"
)

// testPollInterval is used in place of the production default so stream tests observe
// deltas quickly instead of waiting a full second per tick.
const testPollInterval = 5 * time.Millisecond

// newForTest returns a mirror with test-scoped logger and the given PoseTracker.
func newForTest(t *testing.T, pt *inject.PoseTracker) *mirror {
	t.Helper()
	return &mirror{
		logger:       logging.NewTestLogger(t),
		pt:           pt,
		pollInterval: testPollInterval,
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

// fakePTDynamic returns an inject.PoseTracker plus a setter that changes what Poses
// reports. The current snapshot is held behind an atomic.Pointer rather than mutating
// PosesFunc directly, so tests can update it concurrently with the poll goroutine's
// reads without racing.
func fakePTDynamic(initial referenceframe.FrameSystemPoses) (*inject.PoseTracker, func(referenceframe.FrameSystemPoses)) {
	var current atomic.Pointer[referenceframe.FrameSystemPoses]
	current.Store(&initial)

	pt := inject.NewPoseTracker("test-pt")
	pt.PosesFunc = func(ctx context.Context, bodyNames []string, extra map[string]interface{}) (referenceframe.FrameSystemPoses, error) {
		return *current.Load(), nil
	}
	set := func(poses referenceframe.FrameSystemPoses) {
		current.Store(&poses)
	}
	return pt, set
}

// nextWithTimeout calls stream.Next() on a goroutine and fails the test if it doesn't
// return within timeout, instead of letting a stuck stream hang the whole suite.
func nextWithTimeout(t *testing.T, stream *worldstatestore.TransformChangeStream, timeout time.Duration) worldstatestore.TransformChange {
	t.Helper()
	type result struct {
		change worldstatestore.TransformChange
		err    error
	}
	resultCh := make(chan result, 1)
	go func() {
		change, err := stream.Next()
		resultCh <- result{change, err}
	}()

	select {
	case r := <-resultCh:
		test.That(t, r.err, test.ShouldBeNil)
		return r.change
	case <-time.After(timeout):
		t.Fatal("stream.Next() timed out")
		return worldstatestore.TransformChange{}
	}
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
	// A static (constant) PosesFunc is required here: under polling, the stream's
	// background goroutine calls Poses() immediately to seed its initial snapshot, and
	// inject.PoseTracker panics if PosesFunc is unset (it falls through to a nil
	// embedded PoseTracker).
	svc := newForTest(t, fakePTWith(referenceframe.FrameSystemPoses{}))
	ctx, cancel := context.WithCancel(context.Background())
	stream, err := svc.StreamTransformChanges(ctx, nil)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, stream, test.ShouldNotBeNil)
	cancel()
	_, nextErr := stream.Next()
	test.That(t, nextErr, test.ShouldNotBeNil) // ctx.Err() or io.EOF after cancel
}

func TestStreamBlocksWhileContextAlive(t *testing.T) {
	// A static (constant) PosesFunc means every poll tick sees the same snapshot, so
	// diffTransforms produces no changes and Next() correctly keeps blocking. (Also
	// avoids the nil-embedded-PoseTracker panic described in TestStreamClosesOnContextCancel.)
	svc := newForTest(t, fakePTWith(referenceframe.FrameSystemPoses{}))
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

func TestStreamEmitsAddedForNewFrame(t *testing.T) {
	pt, setPoses := fakePTDynamic(referenceframe.FrameSystemPoses{})
	svc := newForTest(t, pt)
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()

	stream, err := svc.StreamTransformChanges(ctx, nil)
	test.That(t, err, test.ShouldBeNil)

	// Give the goroutine time to seed its initial (empty) snapshot before the pose
	// tracker starts reporting the new frame, so the addition shows up as a real delta.
	time.Sleep(testPollInterval)
	setPoses(referenceframe.FrameSystemPoses{
		"a": referenceframe.NewPoseInFrame("world", spatialmath.NewPoseFromPoint(r3.Vector{X: 1})),
	})

	change := nextWithTimeout(t, stream, 2*time.Second)
	test.That(t, change.ChangeType, test.ShouldEqual, pb.TransformChangeType_TRANSFORM_CHANGE_TYPE_ADDED)
	test.That(t, change.Transform.ReferenceFrame, test.ShouldEqual, "a")
	test.That(t, change.Transform.Uuid, test.ShouldResemble, uuidForName("a"))
}

func TestStreamEmitsRemovedForDeletedFrame(t *testing.T) {
	pt, setPoses := fakePTDynamic(referenceframe.FrameSystemPoses{
		"a": referenceframe.NewPoseInFrame("world", spatialmath.NewPoseFromPoint(r3.Vector{X: 1})),
	})
	svc := newForTest(t, pt)
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()

	stream, err := svc.StreamTransformChanges(ctx, nil)
	test.That(t, err, test.ShouldBeNil)

	// Give the goroutine time to seed its initial snapshot (containing "a") before the
	// pose tracker stops reporting it, so the removal shows up as a real delta.
	time.Sleep(testPollInterval)
	setPoses(referenceframe.FrameSystemPoses{})

	change := nextWithTimeout(t, stream, 2*time.Second)
	test.That(t, change.ChangeType, test.ShouldEqual, pb.TransformChangeType_TRANSFORM_CHANGE_TYPE_REMOVED)
	test.That(t, change.Transform.ReferenceFrame, test.ShouldEqual, "a")
	test.That(t, change.Transform.Uuid, test.ShouldResemble, uuidForName("a"))
}
