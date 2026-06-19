package posetracker

import (
	"context"
	"errors"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"

	"github.com/viam-labs/teach-frames/persist"
	"github.com/viam-labs/teach-frames/posesource"
)

func TestCapturePointBuffers(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	pt.source = &posesource.Fake{Poses: []spatialmath.Pose{
		spatialmath.NewPoseFromPoint(r3.Vector{X: 1}),
	}}

	resp, err := pt.DoCommand(context.Background(), map[string]interface{}{"capture_point": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, resp["buffer_len"], test.ShouldEqual, 1)
}

func TestCapturePointTwice(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	pt.source = &posesource.Fake{Poses: []spatialmath.Pose{
		spatialmath.NewPoseFromPoint(r3.Vector{X: 1}),
		spatialmath.NewPoseFromPoint(r3.Vector{X: 2}),
	}}

	// First capture: index 0, buffer_len 1.
	resp, err := pt.DoCommand(context.Background(), map[string]interface{}{"capture_point": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, resp["index"], test.ShouldEqual, 0)
	test.That(t, resp["buffer_len"], test.ShouldEqual, 1)

	// Second capture: index 1, buffer_len 2.
	resp, err = pt.DoCommand(context.Background(), map[string]interface{}{"capture_point": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, resp["index"], test.ShouldEqual, 1)
	test.That(t, resp["buffer_len"], test.ShouldEqual, 2)
}

func TestGetBuffer(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	pt.source = &posesource.Fake{Poses: []spatialmath.Pose{
		spatialmath.NewPoseFromPoint(r3.Vector{X: 1}),
		spatialmath.NewPoseFromPoint(r3.Vector{X: 2}),
	}}

	// Capture two points.
	_, err := pt.DoCommand(context.Background(), map[string]interface{}{"capture_point": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
	_, err = pt.DoCommand(context.Background(), map[string]interface{}{"capture_point": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)

	// get_buffer should return both points.
	resp, err := pt.DoCommand(context.Background(), map[string]interface{}{"get_buffer": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
	points, ok := resp["points"].([]interface{})
	test.That(t, ok, test.ShouldBeTrue)
	test.That(t, len(points), test.ShouldEqual, 2)
}

func TestClearBuffer(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	pt.source = &posesource.Fake{Poses: []spatialmath.Pose{
		spatialmath.NewPoseFromPoint(r3.Vector{X: 1}),
		spatialmath.NewPoseFromPoint(r3.Vector{X: 2}),
	}}

	// Capture two points.
	_, err := pt.DoCommand(context.Background(), map[string]interface{}{"capture_point": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
	_, err = pt.DoCommand(context.Background(), map[string]interface{}{"capture_point": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)

	// clear_buffer should report 2 cleared.
	resp, err := pt.DoCommand(context.Background(), map[string]interface{}{"clear_buffer": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, resp["cleared"], test.ShouldEqual, 2)

	// Buffer should now be empty.
	resp, err = pt.DoCommand(context.Background(), map[string]interface{}{"get_buffer": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
	points, ok := resp["points"].([]interface{})
	test.That(t, ok, test.ShouldBeTrue)
	test.That(t, len(points), test.ShouldEqual, 0)
}

func TestUnknownCommand(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})

	_, err := pt.DoCommand(context.Background(), map[string]interface{}{"unknown_verb": nil})
	test.That(t, err, test.ShouldNotBeNil)
}

// TestMultiKeyCommandRejected verifies the dispatcher rejects ambiguous multi-key maps.
func TestMultiKeyCommandRejected(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})

	_, err := pt.DoCommand(context.Background(), map[string]interface{}{
		"capture_point": map[string]interface{}{},
		"get_buffer":    map[string]interface{}{},
	})
	test.That(t, err, test.ShouldNotBeNil)
}

// TestEmptyCommandRejected verifies the dispatcher rejects empty maps.
func TestEmptyCommandRejected(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})

	_, err := pt.DoCommand(context.Background(), map[string]interface{}{})
	test.That(t, err, test.ShouldNotBeNil)
}

// --- define_frame tests ---

func TestDefineFrameThreePoint(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	fake := &persist.Fake{}
	pt.persist = fake
	pt.source = &posesource.Fake{Poses: []spatialmath.Pose{
		spatialmath.NewPoseFromPoint(r3.Vector{X: 0, Y: 0, Z: 0}),
		spatialmath.NewPoseFromPoint(r3.Vector{X: 1, Y: 0, Z: 0}),
		spatialmath.NewPoseFromPoint(r3.Vector{X: 0, Y: 1, Z: 0}),
	}}
	for i := 0; i < 3; i++ {
		_, err := pt.DoCommand(context.Background(), map[string]interface{}{"capture_point": map[string]interface{}{}})
		test.That(t, err, test.ShouldBeNil)
	}
	resp, err := pt.DoCommand(context.Background(), map[string]interface{}{
		"define_frame": map[string]interface{}{"name": "fixture_a", "method": "3point"}})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, resp["committed"], test.ShouldBeTrue)
	test.That(t, len(fake.Saved), test.ShouldEqual, 1)
	test.That(t, fake.Saved[0].Name, test.ShouldEqual, "fixture_a")
	test.That(t, pt.store.BufferLen(), test.ShouldEqual, 0) // buffer cleared

	// Frame should be in the store.
	_, ok := pt.store.GetFrame("fixture_a")
	test.That(t, ok, test.ShouldBeTrue)
}

func TestDefineFrameInsufficientBuffer(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	pt.persist = &persist.Fake{}
	_, err := pt.DoCommand(context.Background(), map[string]interface{}{
		"define_frame": map[string]interface{}{"name": "x", "method": "3point"}})
	test.That(t, err, test.ShouldNotBeNil)
}

func TestDefineFramePointMethod(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	fake := &persist.Fake{}
	pt.persist = fake
	pt.source = &posesource.Fake{Poses: []spatialmath.Pose{
		spatialmath.NewPoseFromPoint(r3.Vector{X: 5, Y: 10, Z: 0}),
	}}
	_, err := pt.DoCommand(context.Background(), map[string]interface{}{"capture_point": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)

	resp, err := pt.DoCommand(context.Background(), map[string]interface{}{
		"define_frame": map[string]interface{}{"name": "point_frame", "method": "point"}})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, resp["committed"], test.ShouldBeTrue)
	test.That(t, len(fake.Saved), test.ShouldEqual, 1)
	test.That(t, fake.Saved[0].Name, test.ShouldEqual, "point_frame")
	test.That(t, pt.store.BufferLen(), test.ShouldEqual, 0)

	// ComputePoint returns identity orientation.
	pif, ok := pt.store.GetFrame("point_frame")
	test.That(t, ok, test.ShouldBeTrue)
	ov := pif.Pose().Orientation().OrientationVectorDegrees()
	test.That(t, ov.OX, test.ShouldEqual, 0.0)
	test.That(t, ov.OY, test.ShouldEqual, 0.0)
	test.That(t, ov.OZ, test.ShouldEqual, 1.0) // identity OV has OZ=1, Theta=0
	test.That(t, ov.Theta, test.ShouldEqual, 0.0)
}

func TestDefineFrameTCPSnapshot(t *testing.T) {
	capturedPose := spatialmath.NewPoseFromPoint(r3.Vector{X: 3, Y: 7, Z: 2})
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	fake := &persist.Fake{}
	pt.persist = fake
	pt.source = &posesource.Fake{Poses: []spatialmath.Pose{capturedPose}}

	_, err := pt.DoCommand(context.Background(), map[string]interface{}{"capture_point": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)

	resp, err := pt.DoCommand(context.Background(), map[string]interface{}{
		"define_frame": map[string]interface{}{"name": "tcp_snap", "method": "tcp_snapshot"}})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, resp["committed"], test.ShouldBeTrue)
	test.That(t, pt.store.BufferLen(), test.ShouldEqual, 0)

	// The stored pose should equal the captured pose.
	pif, ok := pt.store.GetFrame("tcp_snap")
	test.That(t, ok, test.ShouldBeTrue)
	test.That(t, spatialmath.PoseAlmostEqual(pif.Pose(), capturedPose), test.ShouldBeTrue)
}

func TestDefineFrameEmptyName(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	pt.persist = &persist.Fake{}
	pt.source = &posesource.Fake{Poses: []spatialmath.Pose{
		spatialmath.NewPoseFromPoint(r3.Vector{X: 1}),
	}}
	_, err := pt.DoCommand(context.Background(), map[string]interface{}{"capture_point": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)

	_, err = pt.DoCommand(context.Background(), map[string]interface{}{
		"define_frame": map[string]interface{}{"name": "", "method": "point"}})
	test.That(t, err, test.ShouldNotBeNil)
}

func TestDefineFrameUnknownMethod(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	pt.persist = &persist.Fake{}
	pt.source = &posesource.Fake{Poses: []spatialmath.Pose{
		spatialmath.NewPoseFromPoint(r3.Vector{X: 1}),
	}}
	_, err := pt.DoCommand(context.Background(), map[string]interface{}{"capture_point": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)

	_, err = pt.DoCommand(context.Background(), map[string]interface{}{
		"define_frame": map[string]interface{}{"name": "f", "method": "bogus"}})
	test.That(t, err, test.ShouldNotBeNil)
}

func TestDefineFramePersistDisabled(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	pt.persist = nil // disabled
	pt.source = &posesource.Fake{Poses: []spatialmath.Pose{
		spatialmath.NewPoseFromPoint(r3.Vector{X: 1}),
	}}
	_, err := pt.DoCommand(context.Background(), map[string]interface{}{"capture_point": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)

	_, err = pt.DoCommand(context.Background(), map[string]interface{}{
		"define_frame": map[string]interface{}{"name": "f", "method": "point"}})
	test.That(t, err, test.ShouldNotBeNil)

	// Frame must NOT have been added to the store.
	_, ok := pt.store.GetFrame("f")
	test.That(t, ok, test.ShouldBeFalse)
}

func TestDefineFramePersistError(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	pt.persist = &persist.Fake{Err: errors.New("boom")}
	pt.source = &posesource.Fake{Poses: []spatialmath.Pose{
		spatialmath.NewPoseFromPoint(r3.Vector{X: 1}),
	}}
	_, err := pt.DoCommand(context.Background(), map[string]interface{}{"capture_point": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)

	_, err = pt.DoCommand(context.Background(), map[string]interface{}{
		"define_frame": map[string]interface{}{"name": "new_frame", "method": "point"}})
	test.That(t, err, test.ShouldNotBeNil)

	// The frame should be rolled back (not in store).
	_, ok := pt.store.GetFrame("new_frame")
	test.That(t, ok, test.ShouldBeFalse)
	// Buffer should remain (not cleared on failure).
	test.That(t, pt.store.BufferLen(), test.ShouldEqual, 1)
}

// TestDefineFramePersistErrorRollbackUpsert verifies that when a persist error occurs
// during an UPSERT (replacing an existing frame), the OLD frame is restored — not deleted.
func TestDefineFramePersistErrorRollbackUpsert(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})

	// Pre-populate an existing frame for "existing".
	oldPose := spatialmath.NewPoseFromPoint(r3.Vector{X: 99, Y: 0, Z: 0})
	pt.store.SetFrame("existing", "world", oldPose)

	// Now try to redefine "existing" but persist fails.
	pt.persist = &persist.Fake{Err: errors.New("disk full")}
	pt.source = &posesource.Fake{Poses: []spatialmath.Pose{
		spatialmath.NewPoseFromPoint(r3.Vector{X: 1, Y: 0, Z: 0}),
	}}
	_, err := pt.DoCommand(context.Background(), map[string]interface{}{"capture_point": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)

	_, err = pt.DoCommand(context.Background(), map[string]interface{}{
		"define_frame": map[string]interface{}{"name": "existing", "method": "point"}})
	test.That(t, err, test.ShouldNotBeNil)

	// The OLD frame must still be there, unchanged.
	pif, ok := pt.store.GetFrame("existing")
	test.That(t, ok, test.ShouldBeTrue)
	test.That(t, spatialmath.PoseAlmostEqual(pif.Pose(), oldPose), test.ShouldBeTrue)
}

// TestDefineFramePersistsFullSet verifies that saveFrames serializes the entire
// committed set (not just the newly defined frame) on each successful commit.
func TestDefineFramePersistsFullSet(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	fake := &persist.Fake{}
	pt.persist = fake

	// Pre-populate two frames directly in the store.
	pt.store.SetFrame("frame_a", "world", spatialmath.NewPoseFromPoint(r3.Vector{X: 1}))
	pt.store.SetFrame("frame_b", "world", spatialmath.NewPoseFromPoint(r3.Vector{X: 2}))

	// Capture a point and define a third frame.
	pt.source = &posesource.Fake{Poses: []spatialmath.Pose{
		spatialmath.NewPoseFromPoint(r3.Vector{X: 3}),
	}}
	_, err := pt.DoCommand(context.Background(), map[string]interface{}{"capture_point": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)

	resp, err := pt.DoCommand(context.Background(), map[string]interface{}{
		"define_frame": map[string]interface{}{"name": "frame_c", "method": "point"}})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, resp["committed"], test.ShouldBeTrue)

	// All three frames must appear in the persisted snapshot.
	test.That(t, len(fake.Saved), test.ShouldEqual, 3)
	savedNames := make(map[string]bool, len(fake.Saved))
	for _, s := range fake.Saved {
		savedNames[s.Name] = true
	}
	test.That(t, savedNames["frame_a"], test.ShouldBeTrue)
	test.That(t, savedNames["frame_b"], test.ShouldBeTrue)
	test.That(t, savedNames["frame_c"], test.ShouldBeTrue)
}

// TestDefineFrameThreePointCollinear verifies that collinear points produce an
// error, leave the buffer intact, and do not commit any frame to the store.
func TestDefineFrameThreePointCollinear(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	fake := &persist.Fake{}
	pt.persist = fake
	pt.source = &posesource.Fake{Poses: []spatialmath.Pose{
		spatialmath.NewPoseFromPoint(r3.Vector{X: 0, Y: 0, Z: 0}),
		spatialmath.NewPoseFromPoint(r3.Vector{X: 1, Y: 0, Z: 0}),
		spatialmath.NewPoseFromPoint(r3.Vector{X: 2, Y: 0, Z: 0}), // collinear with the first two
	}}
	for i := 0; i < 3; i++ {
		_, err := pt.DoCommand(context.Background(), map[string]interface{}{"capture_point": map[string]interface{}{}})
		test.That(t, err, test.ShouldBeNil)
	}

	_, err := pt.DoCommand(context.Background(), map[string]interface{}{
		"define_frame": map[string]interface{}{"name": "bad_frame", "method": "3point"}})
	test.That(t, err, test.ShouldNotBeNil)

	// Buffer must still be intact (not cleared on failure).
	test.That(t, pt.store.BufferLen(), test.ShouldEqual, 3)

	// No frame should have been committed to the store.
	_, ok := pt.store.GetFrame("bad_frame")
	test.That(t, ok, test.ShouldBeFalse)

	// Nothing should have been persisted.
	test.That(t, len(fake.Saved), test.ShouldEqual, 0)
}
