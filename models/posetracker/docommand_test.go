package posetracker

import (
	"context"
	"errors"
	"math"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/testutils/inject"
	"go.viam.com/test"

	tfconfig "github.com/viam-labs/teach-frames/config"
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

// --- list_frames / delete_frame / clear_frames tests ---

func TestListDeleteClearFrames(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world",
		Frames: []tfconfig.FrameSpec{{Name: "a", Parent: "world", Pose: tfconfig.PoseSpec{OZ: 1}}}})
	pt.persist = &persist.Fake{}

	resp, err := pt.DoCommand(context.Background(), map[string]interface{}{"list_frames": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
	frames := resp["frames"].(map[string]interface{})
	test.That(t, len(frames), test.ShouldEqual, 1)

	resp, err = pt.DoCommand(context.Background(), map[string]interface{}{"delete_frame": map[string]interface{}{"name": "a"}})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, resp["deleted"], test.ShouldBeTrue)

	resp, err = pt.DoCommand(context.Background(), map[string]interface{}{"clear_frames": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, resp["deleted"], test.ShouldEqual, 0)
}

func TestDeleteFrameNonExistent(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	pt.persist = &persist.Fake{}

	resp, err := pt.DoCommand(context.Background(), map[string]interface{}{"delete_frame": map[string]interface{}{"name": "nope"}})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, resp["deleted"], test.ShouldBeFalse)
}

func TestDeleteFramePersistDisabled(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world",
		Frames: []tfconfig.FrameSpec{{Name: "a", Parent: "world", Pose: tfconfig.PoseSpec{OZ: 1}}}})
	pt.persist = nil

	_, err := pt.DoCommand(context.Background(), map[string]interface{}{"delete_frame": map[string]interface{}{"name": "a"}})
	test.That(t, err, test.ShouldNotBeNil)
}

func TestDeleteFramePersistErrorRollback(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world",
		Frames: []tfconfig.FrameSpec{{Name: "a", Parent: "world", Pose: tfconfig.PoseSpec{OZ: 1}}}})
	pt.persist = &persist.Fake{Err: errors.New("disk full")}

	_, err := pt.DoCommand(context.Background(), map[string]interface{}{"delete_frame": map[string]interface{}{"name": "a"}})
	test.That(t, err, test.ShouldNotBeNil)

	// Frame must be restored after persist failure.
	_, ok := pt.store.GetFrame("a")
	test.That(t, ok, test.ShouldBeTrue)
}

func TestClearFramesTwoFrames(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world",
		Frames: []tfconfig.FrameSpec{
			{Name: "a", Parent: "world", Pose: tfconfig.PoseSpec{OZ: 1}},
			{Name: "b", Parent: "world", Pose: tfconfig.PoseSpec{OZ: 1}},
		}})
	fake := &persist.Fake{}
	pt.persist = fake

	resp, err := pt.DoCommand(context.Background(), map[string]interface{}{"clear_frames": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, resp["deleted"], test.ShouldEqual, 2)

	// Store must be empty.
	names := pt.store.FrameNames()
	test.That(t, len(names), test.ShouldEqual, 0)

	// Persist must have been called with an empty set.
	test.That(t, len(fake.Saved), test.ShouldEqual, 0)
}

func TestClearFramesPersistErrorRollback(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world",
		Frames: []tfconfig.FrameSpec{
			{Name: "a", Parent: "world", Pose: tfconfig.PoseSpec{OZ: 1}},
			{Name: "b", Parent: "world", Pose: tfconfig.PoseSpec{OZ: 1}},
		}})
	pt.persist = &persist.Fake{Err: errors.New("disk full")}

	_, err := pt.DoCommand(context.Background(), map[string]interface{}{"clear_frames": map[string]interface{}{}})
	test.That(t, err, test.ShouldNotBeNil)

	// Both frames must be restored after rollback.
	_, okA := pt.store.GetFrame("a")
	test.That(t, okA, test.ShouldBeTrue)
	_, okB := pt.store.GetFrame("b")
	test.That(t, okB, test.ShouldBeTrue)
}

func TestClearFramesPersistDisabled(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world",
		Frames: []tfconfig.FrameSpec{{Name: "a", Parent: "world", Pose: tfconfig.PoseSpec{OZ: 1}}}})
	pt.persist = nil

	_, err := pt.DoCommand(context.Background(), map[string]interface{}{"clear_frames": map[string]interface{}{}})
	test.That(t, err, test.ShouldNotBeNil)
}

func TestDeleteFrameEmptyName(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	pt.persist = &persist.Fake{}

	_, err := pt.DoCommand(context.Background(), map[string]interface{}{"delete_frame": map[string]interface{}{"name": ""}})
	test.That(t, err, test.ShouldNotBeNil)
}

func TestDeleteFrameMissingName(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	pt.persist = &persist.Fake{}

	_, err := pt.DoCommand(context.Background(), map[string]interface{}{"delete_frame": map[string]interface{}{}})
	test.That(t, err, test.ShouldNotBeNil)
}

// --- capture_tcp_point / get_tcp_buffer / clear_tcp_buffer tests ---

func TestCaptureTCPPoint(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	pt.flange = &posesource.FakeFlange{Poses: []spatialmath.Pose{
		spatialmath.NewPoseFromPoint(r3.Vector{X: 1, Y: 2, Z: 3}),
	}}

	resp, err := pt.DoCommand(context.Background(), map[string]interface{}{"capture_tcp_point": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, resp["index"], test.ShouldEqual, 0)
	test.That(t, resp["buffer_len"], test.ShouldEqual, 1)
}

func TestCaptureTCPPointNoArm(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	pt.flange = nil
	_, err := pt.DoCommand(context.Background(), map[string]interface{}{"capture_tcp_point": map[string]interface{}{}})
	test.That(t, err, test.ShouldNotBeNil)
}

func TestGetTCPBuffer(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	pt.flange = &posesource.FakeFlange{Poses: []spatialmath.Pose{
		spatialmath.NewPoseFromPoint(r3.Vector{X: 1}),
		spatialmath.NewPoseFromPoint(r3.Vector{X: 2}),
	}}

	_, err := pt.DoCommand(context.Background(), map[string]interface{}{"capture_tcp_point": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
	_, err = pt.DoCommand(context.Background(), map[string]interface{}{"capture_tcp_point": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)

	resp, err := pt.DoCommand(context.Background(), map[string]interface{}{"get_tcp_buffer": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
	points, ok := resp["points"].([]interface{})
	test.That(t, ok, test.ShouldBeTrue)
	test.That(t, len(points), test.ShouldEqual, 2)
}

func TestClearTCPBuffer(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	pt.flange = &posesource.FakeFlange{Poses: []spatialmath.Pose{
		spatialmath.NewPoseFromPoint(r3.Vector{X: 1}),
		spatialmath.NewPoseFromPoint(r3.Vector{X: 2}),
	}}

	_, err := pt.DoCommand(context.Background(), map[string]interface{}{"capture_tcp_point": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
	_, err = pt.DoCommand(context.Background(), map[string]interface{}{"capture_tcp_point": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)

	resp, err := pt.DoCommand(context.Background(), map[string]interface{}{"clear_tcp_buffer": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, resp["cleared"], test.ShouldEqual, 2)

	resp, err = pt.DoCommand(context.Background(), map[string]interface{}{"get_tcp_buffer": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
	points, ok := resp["points"].([]interface{})
	test.That(t, ok, test.ShouldBeTrue)
	test.That(t, len(points), test.ShouldEqual, 0)
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

// --- teach_tcp_position tests ---

// makeFlangePoseForTest is a local copy of makeFlangePose from frames/tcp_test.go
// (kept local to avoid cross-package test coupling). It returns a flange pose
// (given orientation, at the given base-frame flange origin) such that a tool
// tip `tip` in the flange frame lands on the fixed base point `target`. It
// solves p = target - R*tip so every synthesized capture is consistent with
// one tip and one touched point. R*tip is computed via spatialmath.Compose
// (not by hand-multiplying RotationMatrix() columns) so the synthesized pose
// matches how the SDK actually composes real captured poses.
func makeFlangePoseForTest(ov *spatialmath.OrientationVectorDegrees, tip, target r3.Vector) spatialmath.Pose {
	rotationOnly := spatialmath.NewPoseFromOrientation(ov)
	rTip := spatialmath.Compose(rotationOnly, spatialmath.NewPoseFromPoint(tip)).Point()
	p := target.Sub(rTip)
	return spatialmath.NewPose(p, ov)
}

func TestTeachTCPPositionPersistsTranslation(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	fake := &persist.Fake{}
	pt.persist = fake
	pt.tcpComponent = "tool"
	pt.armName = "my-arm"

	tip := r3.Vector{X: 10, Y: -5, Z: 120}
	target := r3.Vector{X: 400, Y: 0, Z: 300}
	// Orientations must tilt the flange Z axis away from the base Z axis (not
	// just spin about it) so the 6x6 least-squares system is well-conditioned;
	// see frames/tcp_test.go TestComputePivotTCPRecoversTip for details.
	ovs := []*spatialmath.OrientationVectorDegrees{
		{OZ: 1, Theta: 0},
		{OX: 0.3, OZ: 1, Theta: 30},
		{OY: 0.3, OZ: 1, Theta: 75},
		{OX: -0.2, OY: 0.2, OZ: 1, Theta: 150},
	}
	for _, ov := range ovs {
		pt.store.AddTCPCapture(makeFlangePoseForTest(ov, tip, target))
	}

	resp, err := pt.DoCommand(context.Background(), map[string]interface{}{"teach_tcp_position": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, resp["committed"], test.ShouldEqual, true)
	test.That(t, fake.SavedComponent, test.ShouldEqual, "tool")
	test.That(t, fake.SavedParent, test.ShouldEqual, "my-arm")
	test.That(t, fake.SavedTranslation, test.ShouldNotBeNil)
	// The persisted translation must be the SOLVED tip, not the common touched
	// point, a zero vector, or the RHS — proves teachTCPPosition plumbs the
	// pivot solution through to persistence.
	test.That(t, spatialmath.R3VectorAlmostEqual(*fake.SavedTranslation, tip, 1e-6), test.ShouldBeTrue)
	test.That(t, pt.store.TCPBufferLen(), test.ShouldEqual, 0) // cleared on success

	residual, ok := resp["residual_rms"].(float64)
	test.That(t, ok, test.ShouldBeTrue)
	test.That(t, residual < 1.0, test.ShouldBeTrue)
}

func TestTeachTCPPositionPersistenceDisabled(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	pt.persist = nil
	for i := 0; i < 4; i++ {
		pt.store.AddTCPCapture(spatialmath.NewZeroPose())
	}
	_, err := pt.DoCommand(context.Background(), map[string]interface{}{"teach_tcp_position": map[string]interface{}{}})
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, pt.store.TCPBufferLen(), test.ShouldEqual, 4) // buffer preserved on failure
}

// --- teach_tcp_orientation tests ---

func TestTeachTCPOrientationPersistsOrientation(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	fake := &persist.Fake{}
	pt.persist = fake
	pt.tcpComponent = "tool"
	pt.armName = "my-arm"

	resp, err := pt.DoCommand(context.Background(), map[string]interface{}{
		"teach_tcp_orientation": map[string]interface{}{"o_x": 0.0, "o_y": 0.0, "o_z": 1.0, "theta": 45.0},
	})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, resp["committed"], test.ShouldEqual, true)
	test.That(t, fake.SavedOrientation, test.ShouldNotBeNil)
	test.That(t, fake.SavedTranslation, test.ShouldBeNil) // translation untouched
}

func TestTeachTCPOrientationRejectsZeroVector(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	pt.persist = &persist.Fake{}
	_, err := pt.DoCommand(context.Background(), map[string]interface{}{
		"teach_tcp_orientation": map[string]interface{}{"o_x": 0.0, "o_y": 0.0, "o_z": 0.0, "theta": 0.0},
	})
	test.That(t, err, test.ShouldNotBeNil)
}

func TestTeachTCPOrientationRejectsWrongTypedValue(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	pt.persist = &persist.Fake{}
	// A present-but-wrong-typed value on a real key (o_x as a string, not a
	// number) must error rather than silently coercing to 0. o_z is a valid
	// nonzero float here so the all-zero guard cannot mask the type error —
	// without the type check this would silently persist the WRONG orientation.
	_, err := pt.DoCommand(context.Background(), map[string]interface{}{
		"teach_tcp_orientation": map[string]interface{}{"o_x": "bad", "o_z": 1.0},
	})
	test.That(t, err, test.ShouldNotBeNil)
}

func TestTeachTCPOrientationPersistenceDisabled(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	pt.persist = nil
	_, err := pt.DoCommand(context.Background(), map[string]interface{}{
		"teach_tcp_orientation": map[string]interface{}{"o_x": 0.0, "o_y": 0.0, "o_z": 1.0, "theta": 45.0},
	})
	test.That(t, err, test.ShouldNotBeNil)
}

// --- armName guard tests (M3) ---

// TestTeachTCPPositionNoArm verifies teach_tcp_position errors when no arm is
// configured (pt.armName == ""), and — critically — that nothing is persisted.
// Without this guard, an empty armName would flow through to
// persist.SaveComponentFrame as an empty parent string and get silently
// written into the tool's frame config. Captures are seeded so a solve would
// otherwise succeed, isolating this test to the armName guard regardless of
// whether it is placed before or after the buffer-length check.
func TestTeachTCPPositionNoArm(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	fake := &persist.Fake{}
	pt.persist = fake
	pt.tcpComponent = "tool"
	pt.armName = "" // no arm configured

	tip := r3.Vector{X: 10, Y: -5, Z: 120}
	target := r3.Vector{X: 400, Y: 0, Z: 300}
	ovs := []*spatialmath.OrientationVectorDegrees{
		{OZ: 1, Theta: 0},
		{OX: 0.3, OZ: 1, Theta: 30},
		{OY: 0.3, OZ: 1, Theta: 75},
		{OX: -0.2, OY: 0.2, OZ: 1, Theta: 150},
	}
	for _, ov := range ovs {
		pt.store.AddTCPCapture(makeFlangePoseForTest(ov, tip, target))
	}

	_, err := pt.DoCommand(context.Background(), map[string]interface{}{"teach_tcp_position": map[string]interface{}{}})
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, fake.SavedComponent, test.ShouldEqual, "") // nothing persisted
}

// TestTeachTCPOrientationNoArm verifies teach_tcp_orientation errors when no
// arm is configured, and that nothing is persisted.
func TestTeachTCPOrientationNoArm(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	fake := &persist.Fake{}
	pt.persist = fake
	pt.tcpComponent = "tool"
	pt.armName = "" // no arm configured

	_, err := pt.DoCommand(context.Background(), map[string]interface{}{
		"teach_tcp_orientation": map[string]interface{}{"o_x": 0.0, "o_y": 0.0, "o_z": 1.0, "theta": 45.0},
	})
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, fake.SavedComponent, test.ShouldEqual, "") // nothing persisted
}

// --- get_arm_state tests ---

func TestGetArmState(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm"})
	_, err := pt.DoCommand(context.Background(), map[string]interface{}{"get_arm_state": map[string]interface{}{}})
	test.That(t, err, test.ShouldNotBeNil) // arm not configured

	injArm := inject.NewArm("my-arm")
	injArm.EndPositionFunc = func(context.Context, map[string]interface{}) (spatialmath.Pose, error) {
		return spatialmath.NewPoseFromPoint(r3.Vector{X: 1, Y: 2, Z: 3}), nil
	}
	injArm.JointPositionsFunc = func(context.Context, map[string]interface{}) ([]referenceframe.Input, error) {
		return []referenceframe.Input{0, math.Pi / 2}, nil
	}
	pt.arm = injArm

	resp, err := pt.DoCommand(context.Background(), map[string]interface{}{"get_arm_state": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
	pose := resp["pose"].(map[string]interface{})
	test.That(t, pose["x"], test.ShouldAlmostEqual, 1.0)
	joints := resp["joints"].([]float64)
	test.That(t, joints[0], test.ShouldAlmostEqual, 0.0)
	test.That(t, joints[1], test.ShouldAlmostEqual, 90.0) // radians→degrees
}

// --- stop_arm tests ---

func TestStopArm(t *testing.T) {
	injArm := inject.NewArm("my-arm")
	stopped := false
	injArm.StopFunc = func(context.Context, map[string]interface{}) error { stopped = true; return nil }
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm"})
	pt.arm = injArm

	resp, err := pt.DoCommand(context.Background(), map[string]interface{}{"stop_arm": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, resp["stopped"], test.ShouldBeTrue)
	test.That(t, stopped, test.ShouldBeTrue)
}

func TestStopArmNoArm(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm"})
	_, err := pt.DoCommand(context.Background(), map[string]interface{}{"stop_arm": map[string]interface{}{}})
	test.That(t, err, test.ShouldNotBeNil)
}

// --- jog_joint tests ---

func TestJogJoint(t *testing.T) {
	injArm := inject.NewArm("my-arm")
	start := []referenceframe.Input{0, 0, 0}
	var moved []referenceframe.Input
	injArm.JointPositionsFunc = func(context.Context, map[string]interface{}) ([]referenceframe.Input, error) {
		return start, nil
	}
	injArm.MoveToJointPositionsFunc = func(_ context.Context, p []referenceframe.Input, _ map[string]interface{}) error {
		moved = p
		return nil
	}
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm"})
	pt.arm = injArm

	// jog joint 1 by +90 degrees
	resp, err := pt.DoCommand(context.Background(), map[string]interface{}{
		"jog_joint": map[string]interface{}{"joint": 1.0, "step": 90.0},
	})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, moved[1], test.ShouldAlmostEqual, math.Pi/2) // +90deg in radians
	test.That(t, moved[0], test.ShouldAlmostEqual, 0.0)
	joints := resp["joints"].([]float64)
	test.That(t, joints[1], test.ShouldAlmostEqual, 90.0)

	// out-of-range joint index errors
	_, err = pt.DoCommand(context.Background(), map[string]interface{}{
		"jog_joint": map[string]interface{}{"joint": 9.0, "step": 5.0},
	})
	test.That(t, err, test.ShouldNotBeNil)
}

func TestJogJointNoArm(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm"})
	_, err := pt.DoCommand(context.Background(), map[string]interface{}{
		"jog_joint": map[string]interface{}{"joint": 0.0, "step": 5.0},
	})
	test.That(t, err, test.ShouldNotBeNil)
}

func TestJogCartesianTranslation(t *testing.T) {
	injArm := inject.NewArm("my-arm")
	// non-identity orientation to prove translation is world-frame (not rotated by tool orientation)
	startOri := &spatialmath.EulerAngles{Yaw: math.Pi / 2}
	injArm.EndPositionFunc = func(context.Context, map[string]interface{}) (spatialmath.Pose, error) {
		return spatialmath.NewPose(r3.Vector{X: 10, Y: 20, Z: 30}, startOri), nil
	}
	var moved spatialmath.Pose
	injArm.MoveToPositionFunc = func(_ context.Context, p spatialmath.Pose, _ map[string]interface{}) error {
		moved = p
		return nil
	}
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm"})
	pt.arm = injArm

	_, err := pt.DoCommand(context.Background(), map[string]interface{}{
		"jog_cartesian": map[string]interface{}{"axis": "x", "step": 5.0},
	})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, moved.Point().X, test.ShouldAlmostEqual, 15.0) // 10+5 world-frame
	test.That(t, moved.Point().Y, test.ShouldAlmostEqual, 20.0)
	test.That(t, moved.Point().Z, test.ShouldAlmostEqual, 30.0)
	test.That(t, spatialmath.OrientationAlmostEqual(moved.Orientation(), startOri), test.ShouldBeTrue)
}

func TestJogCartesianNoArm(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm"})
	_, err := pt.DoCommand(context.Background(), map[string]interface{}{
		"jog_cartesian": map[string]interface{}{"axis": "x", "step": 5.0},
	})
	test.That(t, err, test.ShouldNotBeNil)
}

func TestJogCartesianUnknownAxis(t *testing.T) {
	injArm := inject.NewArm("my-arm")
	injArm.EndPositionFunc = func(context.Context, map[string]interface{}) (spatialmath.Pose, error) {
		return spatialmath.NewPose(r3.Vector{X: 1, Y: 2, Z: 3}, &spatialmath.EulerAngles{}), nil
	}
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm"})
	pt.arm = injArm

	_, err := pt.DoCommand(context.Background(), map[string]interface{}{
		"jog_cartesian": map[string]interface{}{"axis": "bogus", "step": 5.0},
	})
	test.That(t, err, test.ShouldNotBeNil)
}

func TestJogCartesianRotationToolFrame(t *testing.T) {
	injArm := inject.NewArm("my-arm")
	startOri := &spatialmath.EulerAngles{Roll: math.Pi / 2}
	injArm.EndPositionFunc = func(context.Context, map[string]interface{}) (spatialmath.Pose, error) {
		return spatialmath.NewPose(r3.Vector{X: 1, Y: 2, Z: 3}, startOri), nil
	}
	var moved spatialmath.Pose
	injArm.MoveToPositionFunc = func(_ context.Context, p spatialmath.Pose, _ map[string]interface{}) error {
		moved = p
		return nil
	}
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm"})
	pt.arm = injArm

	_, err := pt.DoCommand(context.Background(), map[string]interface{}{
		"jog_cartesian": map[string]interface{}{"axis": "yaw", "step": 30.0},
	})
	test.That(t, err, test.ShouldBeNil)

	// pure rotation: position unchanged
	test.That(t, moved.Point().X, test.ShouldAlmostEqual, 1.0)
	test.That(t, moved.Point().Y, test.ShouldAlmostEqual, 2.0)
	test.That(t, moved.Point().Z, test.ShouldAlmostEqual, 3.0)

	// orientation == tool-frame compose (right-multiply) of a 30deg yaw
	delta := spatialmath.NewPoseFromOrientation(&spatialmath.EulerAngles{Yaw: 30 * math.Pi / 180})
	expected := spatialmath.Compose(spatialmath.NewPose(r3.Vector{X: 1, Y: 2, Z: 3}, startOri), delta)
	test.That(t, spatialmath.OrientationAlmostEqual(moved.Orientation(), expected.Orientation()), test.ShouldBeTrue)
}

// TestJogCartesianAllAxes covers every translation and rotation axis (x/y/z and
// roll/pitch/yaw) with a non-identity start pose, closing the coverage gap where
// only x and yaw were exercised for the math. Translation axes must move the world
// point and preserve orientation; rotation axes must preserve position and equal the
// tool-frame (right-multiply) compose of a single-axis Euler delta.
func TestJogCartesianAllAxes(t *testing.T) {
	start := spatialmath.NewPose(r3.Vector{X: 1, Y: 2, Z: 3}, &spatialmath.EulerAngles{Roll: math.Pi / 2})
	const step = 30.0

	transCases := map[string]r3.Vector{
		"x": {X: step}, "y": {Y: step}, "z": {Z: step},
	}
	for axis, delta := range transCases {
		t.Run("translate_"+axis, func(t *testing.T) {
			injArm := inject.NewArm("my-arm")
			injArm.EndPositionFunc = func(context.Context, map[string]interface{}) (spatialmath.Pose, error) {
				return start, nil
			}
			var moved spatialmath.Pose
			injArm.MoveToPositionFunc = func(_ context.Context, p spatialmath.Pose, _ map[string]interface{}) error {
				moved = p
				return nil
			}
			pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm"})
			pt.arm = injArm

			_, err := pt.DoCommand(context.Background(), map[string]interface{}{
				"jog_cartesian": map[string]interface{}{"axis": axis, "step": step},
			})
			test.That(t, err, test.ShouldBeNil)
			want := start.Point().Add(delta)
			test.That(t, moved.Point().X, test.ShouldAlmostEqual, want.X)
			test.That(t, moved.Point().Y, test.ShouldAlmostEqual, want.Y)
			test.That(t, moved.Point().Z, test.ShouldAlmostEqual, want.Z)
			test.That(t, spatialmath.OrientationAlmostEqual(moved.Orientation(), start.Orientation()), test.ShouldBeTrue)
		})
	}

	rad := step * math.Pi / 180
	rotCases := map[string]*spatialmath.EulerAngles{
		"roll":  {Roll: rad},
		"pitch": {Pitch: rad},
		"yaw":   {Yaw: rad},
	}
	for axis, euler := range rotCases {
		t.Run("rotate_"+axis, func(t *testing.T) {
			injArm := inject.NewArm("my-arm")
			injArm.EndPositionFunc = func(context.Context, map[string]interface{}) (spatialmath.Pose, error) {
				return start, nil
			}
			var moved spatialmath.Pose
			injArm.MoveToPositionFunc = func(_ context.Context, p spatialmath.Pose, _ map[string]interface{}) error {
				moved = p
				return nil
			}
			pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm"})
			pt.arm = injArm

			_, err := pt.DoCommand(context.Background(), map[string]interface{}{
				"jog_cartesian": map[string]interface{}{"axis": axis, "step": step},
			})
			test.That(t, err, test.ShouldBeNil)
			// position preserved
			test.That(t, moved.Point().X, test.ShouldAlmostEqual, 1.0)
			test.That(t, moved.Point().Y, test.ShouldAlmostEqual, 2.0)
			test.That(t, moved.Point().Z, test.ShouldAlmostEqual, 3.0)
			// tool-frame (right-multiply) orientation
			expected := spatialmath.Compose(start, spatialmath.NewPoseFromOrientation(euler))
			test.That(t, spatialmath.OrientationAlmostEqual(moved.Orientation(), expected.Orientation()), test.ShouldBeTrue)
		})
	}
}
