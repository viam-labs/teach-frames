package posetracker

import (
	"bytes"
	"context"
	"encoding/base64"
	"errors"
	"image"
	_ "image/jpeg" // register the JPEG decoder for image.DecodeConfig
	"math"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/rimage"
	"go.viam.com/rdk/rimage/transform"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/testutils/inject"
	"go.viam.com/test"

	tfconfig "github.com/viam-labs/teach-frames/config"
	"github.com/viam-labs/teach-frames/frames"
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

// Shared 4x4 RGBD fixture for hand-eye tests. Depth is set at pixel (1,1) ONLY,
// so tests must click u=1, v=1; any other pixel deprojects to "no depth".
func testIntr() *transform.PinholeCameraIntrinsics {
	return &transform.PinholeCameraIntrinsics{Width: 4, Height: 4, Fx: 2, Fy: 2, Ppx: 2, Ppy: 2}
}

func testDepth() *rimage.DepthMap {
	dm := rimage.NewEmptyDepthMap(4, 4)
	dm.Set(1, 1, rimage.Depth(500))
	return dm
}

func testRGB() image.Image {
	return image.NewRGBA(image.Rect(0, 0, 4, 4))
}

func TestHandEyeSnapshot(t *testing.T) {
	intr, dm := testIntr(), testDepth()
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	pt.cameraSrc = &posesource.FakeCamera{RGB: testRGB(), Depth: dm, Intr: intr}

	resp, err := pt.DoCommand(context.Background(), map[string]interface{}{"handeye_snapshot": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, resp["width"], test.ShouldEqual, 4)
	test.That(t, resp["height"], test.ShouldEqual, 4)

	// The returned image must be a valid 4x4 JPEG, not just a non-nil string.
	encoded, ok := resp["image"].(string)
	test.That(t, ok, test.ShouldBeTrue)
	decoded, derr := base64.StdEncoding.DecodeString(encoded)
	test.That(t, derr, test.ShouldBeNil)
	cfg, format, cerr := image.DecodeConfig(bytes.NewReader(decoded))
	test.That(t, cerr, test.ShouldBeNil)
	test.That(t, format, test.ShouldEqual, "jpeg")
	test.That(t, cfg.Width, test.ShouldEqual, 4)
	test.That(t, cfg.Height, test.ShouldEqual, 4)

	// The cache must hold THE snapshot just acquired (identity, not a fresh/empty one).
	test.That(t, pt.lastSnapshot, test.ShouldNotBeNil)
	test.That(t, pt.lastSnapshot.Intr, test.ShouldEqual, intr)
	test.That(t, pt.lastSnapshot.Depth, test.ShouldEqual, dm)
}

func TestHandEyeSnapshotNoCameraErrors(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	pt.cameraSrc = nil
	_, err := pt.DoCommand(context.Background(), map[string]interface{}{"handeye_snapshot": map[string]interface{}{}})
	test.That(t, err, test.ShouldNotBeNil)
}

// Eye-to-hand requires no arm at all, so the shared snapshot verb must not read
// the flange unconditionally -- that would break every arm-less machine.
func TestHandeyeSnapshotEyeToHandWorksWithoutArm(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	pt.cameraSrc = &posesource.FakeCamera{RGB: testRGB(), Depth: testDepth(), Intr: testIntr()}
	pt.cameraMount = mountEyeToHand
	pt.flangeInDest = nil // no arm configured

	_, err := pt.DoCommand(context.Background(), map[string]interface{}{"handeye_snapshot": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
}

func TestHandeyeSnapshotEyeInHandCachesFlange(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	pt.cameraSrc = &posesource.FakeCamera{RGB: testRGB(), Depth: testDepth(), Intr: testIntr()}
	pt.cameraMount = mountEyeInHand
	f1 := spatialmath.NewPoseFromPoint(r3.Vector{X: 11})
	pt.flangeInDest = &posesource.Fake{Poses: []spatialmath.Pose{f1}}

	_, err := pt.DoCommand(context.Background(), map[string]interface{}{"handeye_snapshot": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, pt.lastFlange, test.ShouldNotBeNil)
	test.That(t, pt.lastFlange.Point().X, test.ShouldAlmostEqual, 11.0)
}

func TestHandeyeSnapshotEyeInHandErrorsWithoutArm(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	pt.cameraSrc = &posesource.FakeCamera{RGB: testRGB(), Depth: testDepth(), Intr: testIntr()}
	pt.cameraMount = mountEyeInHand
	pt.flangeInDest = nil

	_, err := pt.DoCommand(context.Background(), map[string]interface{}{"handeye_snapshot": map[string]interface{}{}})
	test.That(t, err, test.ShouldNotBeNil)
	// Pin the content: a bare not-nil assertion would be satisfied by any unrelated
	// earlier failure, reporting coverage of the arm check that it does not have.
	test.That(t, err.Error(), test.ShouldContainSubstring, "arm dependency not configured")
}

func TestCaptureHandEyePoint(t *testing.T) {
	intr := &transform.PinholeCameraIntrinsics{Width: 4, Height: 4, Fx: 2, Fy: 2, Ppx: 2, Ppy: 2}
	dm := rimage.NewEmptyDepthMap(4, 4)
	dm.Set(3, 1, rimage.Depth(1000))
	rgb := image.NewRGBA(image.Rect(0, 0, 4, 4))

	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	pt.cameraSrc = &posesource.FakeCamera{RGB: rgb, Depth: dm, Intr: intr}
	// The Fake PoseSource returns a queued world pose for Capture():
	pt.source = &posesource.Fake{Poses: []spatialmath.Pose{
		spatialmath.NewPoseFromPoint(r3.Vector{X: 10, Y: 20, Z: 30}),
	}}

	// Must snapshot before capture (populates the cache).
	_, err := pt.DoCommand(context.Background(), map[string]interface{}{"handeye_snapshot": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)

	resp, err := pt.DoCommand(context.Background(),
		map[string]interface{}{"capture_handeye_point": map[string]interface{}{"u": 3.0, "v": 1.0}})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, resp["index"], test.ShouldEqual, 0)
	test.That(t, resp["buffer_len"], test.ShouldEqual, 1)
	world := resp["world"].(map[string]interface{})
	test.That(t, world["x"], test.ShouldEqual, 10.0)
	cam := resp["camera"].(map[string]interface{})
	// x = (3-2)/2 * 1000 = 500
	test.That(t, cam["x"], test.ShouldEqual, 500.0)
	// y = (1-2)/2 * 1000 = -500
	test.That(t, cam["y"], test.ShouldEqual, -500.0)
	// z = depth = 1000
	test.That(t, cam["z"], test.ShouldEqual, 1000.0)
}

// The guard must fire before the snapshot-cache check: this test never calls
// handeye_snapshot, so if the mount guard were deleted the next check reached
// would be "no snapshot cached" -- a different error that does NOT mention
// capture_handeye_target, which the substring assertion below would catch.
func TestCaptureHandEyePointRejectedInEyeInHand(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	pt.cameraMount = mountEyeInHand
	pt.cameraSrc = &posesource.FakeCamera{RGB: testRGB(), Depth: testDepth(), Intr: testIntr()}

	_, err := pt.DoCommand(context.Background(), map[string]interface{}{"capture_handeye_point": map[string]interface{}{"u": 1.0, "v": 1.0}})
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "capture_handeye_target")
}

func TestCaptureHandEyeNoCameraErrors(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	pt.cameraSrc = nil
	_, err := pt.DoCommand(context.Background(),
		map[string]interface{}{"capture_handeye_point": map[string]interface{}{"u": 1.0, "v": 1.0}})
	test.That(t, err, test.ShouldNotBeNil)
}

func TestCaptureHandEyeNoSnapshotErrors(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	pt.cameraSrc = &posesource.FakeCamera{} // no snapshot taken yet
	_, err := pt.DoCommand(context.Background(),
		map[string]interface{}{"capture_handeye_point": map[string]interface{}{"u": 1.0, "v": 1.0}})
	test.That(t, err, test.ShouldNotBeNil)
}

// --- get_handeye_buffer / clear_handeye_buffer tests ---

func TestGetAndClearHandEyeBuffer(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	pt.store.AddHandEyePair(frames.PointPair{Reference: r3.Vector{X: 1}, Camera: r3.Vector{Z: 2}})

	got, err := pt.DoCommand(context.Background(), map[string]interface{}{"get_handeye_buffer": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
	pts := got["points"].([]interface{})
	test.That(t, len(pts), test.ShouldEqual, 1)

	cleared, err := pt.DoCommand(context.Background(), map[string]interface{}{"clear_handeye_buffer": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, cleared["cleared"], test.ShouldEqual, 1)
	test.That(t, pt.store.HandEyeBufferLen(), test.ShouldEqual, 0)
}

// --- capture_handeye_target tests ---

func TestCaptureHandEyeTargetSetsTarget(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	pt.cameraMount = mountEyeInHand
	pt.source = &posesource.Fake{Poses: []spatialmath.Pose{spatialmath.NewPoseFromPoint(r3.Vector{X: 7, Y: 8, Z: 9})}}

	resp, err := pt.DoCommand(context.Background(), map[string]interface{}{"capture_handeye_target": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, resp["target"].(map[string]interface{})["x"], test.ShouldAlmostEqual, 7.0)
	test.That(t, pt.currentTarget, test.ShouldNotBeNil)
}

func TestCaptureHandEyeTargetRejectedInEyeToHand(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	pt.cameraMount = mountEyeToHand

	_, err := pt.DoCommand(context.Background(), map[string]interface{}{"capture_handeye_target": map[string]interface{}{}})
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "capture_handeye_point")
}

// --- capture_handeye_view tests ---

// The snapshot must freeze the flange pose. If the implementation re-reads the
// flange at click time it gets F2 instead of F1 and this fails -- which is the
// race we cannot see on hardware.
func TestCaptureHandEyeViewUsesCachedFlange(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	pt.cameraMount = mountEyeInHand
	pt.cameraSrc = &posesource.FakeCamera{RGB: testRGB(), Depth: testDepth(), Intr: testIntr()}
	// SEPARATE fakes: a shared queue would make this assertion meaningless.
	pt.source = &posesource.Fake{Poses: []spatialmath.Pose{spatialmath.NewPoseFromPoint(r3.Vector{X: 400})}}
	f1 := spatialmath.NewPoseFromPoint(r3.Vector{X: 11})
	f2 := spatialmath.NewPoseFromPoint(r3.Vector{X: 22})
	pt.flangeInDest = &posesource.Fake{Poses: []spatialmath.Pose{f1, f2}}

	ctx := context.Background()
	_, err := pt.DoCommand(ctx, map[string]interface{}{"capture_handeye_target": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
	_, err = pt.DoCommand(ctx, map[string]interface{}{"handeye_snapshot": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
	_, err = pt.DoCommand(ctx, map[string]interface{}{"capture_handeye_view": map[string]interface{}{"u": 1.0, "v": 1.0}})
	test.That(t, err, test.ShouldBeNil)

	buf := pt.store.EyeInHandBuffer()
	test.That(t, len(buf), test.ShouldEqual, 1)
	test.That(t, buf[0].Flange.Point().X, test.ShouldAlmostEqual, 11.0) // F1, not F2
}

func TestCaptureHandEyeViewRequiresTarget(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	pt.cameraMount = mountEyeInHand
	pt.cameraSrc = &posesource.FakeCamera{RGB: testRGB(), Depth: testDepth(), Intr: testIntr()}
	pt.flangeInDest = &posesource.Fake{Poses: []spatialmath.Pose{spatialmath.NewPoseFromPoint(r3.Vector{X: 11})}}

	ctx := context.Background()
	_, err := pt.DoCommand(ctx, map[string]interface{}{"handeye_snapshot": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
	_, err = pt.DoCommand(ctx, map[string]interface{}{"capture_handeye_view": map[string]interface{}{"u": 1.0, "v": 1.0}})
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "capture_handeye_target")
}

// One view per snapshot. Without this an operator can click twice without
// re-snapshotting, producing two identical observations -- exactly the degenerate
// set the solver must reject.
func TestCaptureHandEyeViewConsumesSnapshot(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	pt.cameraMount = mountEyeInHand
	pt.cameraSrc = &posesource.FakeCamera{RGB: testRGB(), Depth: testDepth(), Intr: testIntr()}
	pt.source = &posesource.Fake{Poses: []spatialmath.Pose{spatialmath.NewPoseFromPoint(r3.Vector{X: 400})}}
	pt.flangeInDest = &posesource.Fake{Poses: []spatialmath.Pose{spatialmath.NewPoseFromPoint(r3.Vector{X: 11})}}

	ctx := context.Background()
	_, _ = pt.DoCommand(ctx, map[string]interface{}{"capture_handeye_target": map[string]interface{}{}})
	_, _ = pt.DoCommand(ctx, map[string]interface{}{"handeye_snapshot": map[string]interface{}{}})
	_, err := pt.DoCommand(ctx, map[string]interface{}{"capture_handeye_view": map[string]interface{}{"u": 1.0, "v": 1.0}})
	test.That(t, err, test.ShouldBeNil)

	_, err = pt.DoCommand(ctx, map[string]interface{}{"capture_handeye_view": map[string]interface{}{"u": 1.0, "v": 1.0}})
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, pt.store.EyeInHandBufferLen(), test.ShouldEqual, 1)
}

// The fixture is deliberately COMPLETE -- target, snapshot, and frozen flange all
// present -- so the mount guard is the only thing left that can reject the view.
// With a bare fixture this test passes on "no target set" whether or not the guard
// exists, asserting merely that something failed rather than that the right thing
// did. Fully armed, deleting the guard makes the command succeed outright.
func TestCaptureHandEyeViewRejectedInEyeToHand(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	pt.cameraMount = mountEyeToHand
	pt.cameraSrc = &posesource.FakeCamera{RGB: testRGB(), Depth: testDepth(), Intr: testIntr()}
	pt.currentTarget = &r3.Vector{X: 400}
	pt.lastSnapshot = &posesource.Snapshot{RGB: testRGB(), Depth: testDepth(), Intr: testIntr()}
	pt.lastFlange = spatialmath.NewPoseFromPoint(r3.Vector{X: 11})

	_, err := pt.DoCommand(context.Background(), map[string]interface{}{"capture_handeye_view": map[string]interface{}{"u": 1.0, "v": 1.0}})
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "capture_handeye_point")
	test.That(t, pt.store.EyeInHandBufferLen(), test.ShouldEqual, 0) // nothing recorded
}

// --- solve_handeye tests ---

func TestSolveHandEye(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	// Identity transform: world == camera, so R=I, t=0.
	pts := []frames.PointPair{
		{Camera: r3.Vector{X: 0, Y: 0, Z: 0}, Reference: r3.Vector{X: 0, Y: 0, Z: 0}},
		{Camera: r3.Vector{X: 100, Y: 0, Z: 0}, Reference: r3.Vector{X: 100, Y: 0, Z: 0}},
		{Camera: r3.Vector{X: 0, Y: 100, Z: 0}, Reference: r3.Vector{X: 0, Y: 100, Z: 0}},
		{Camera: r3.Vector{X: 0, Y: 0, Z: 100}, Reference: r3.Vector{X: 0, Y: 0, Z: 100}},
	}
	for _, p := range pts {
		pt.store.AddHandEyePair(p)
	}
	pt.cameraName = "cam"

	resp, err := pt.DoCommand(context.Background(), map[string]interface{}{"solve_handeye": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, resp["committed"], test.ShouldBeTrue)
	test.That(t, resp["residual_rms"].(float64), test.ShouldBeLessThan, 1e-6)
	// Buffer cleared on success:
	test.That(t, pt.store.HandEyeBufferLen(), test.ShouldEqual, 0)
	// Persisted to the camera's frame, parent world:
	fake := pt.persist.(*persist.Fake)
	test.That(t, fake.SavedComponent, test.ShouldEqual, "cam")
	test.That(t, fake.SavedParent, test.ShouldEqual, "world")
	test.That(t, fake.SavedTranslation, test.ShouldNotBeNil)
	// The persisted translation must be the SOLVED transform, not just non-nil:
	// for identity input (world == camera) the translation is ~{0,0,0}. A
	// transposed-rotation or wrong-pose bug would still be non-nil but wrong.
	test.That(t, fake.SavedTranslation.X, test.ShouldAlmostEqual, 0.0, 1e-6)
	test.That(t, fake.SavedTranslation.Y, test.ShouldAlmostEqual, 0.0, 1e-6)
	test.That(t, fake.SavedTranslation.Z, test.ShouldAlmostEqual, 0.0, 1e-6)
}

// solveHandEyeIdentityPairs are the identity-transform (world == camera) points
// reused across solve_handeye tests.
func solveHandEyeIdentityPairs() []frames.PointPair {
	return []frames.PointPair{
		{Camera: r3.Vector{X: 0, Y: 0, Z: 0}, Reference: r3.Vector{X: 0, Y: 0, Z: 0}},
		{Camera: r3.Vector{X: 100, Y: 0, Z: 0}, Reference: r3.Vector{X: 100, Y: 0, Z: 0}},
		{Camera: r3.Vector{X: 0, Y: 100, Z: 0}, Reference: r3.Vector{X: 0, Y: 100, Z: 0}},
		{Camera: r3.Vector{X: 0, Y: 0, Z: 100}, Reference: r3.Vector{X: 0, Y: 0, Z: 100}},
	}
}

// TestSolveHandEyePersistsCaptureFrameAsParent guards against silently persisting
// the camera under "world" when the captured world points are actually expressed
// in a non-world destination_frame. The Kabsch solve yields camera->destFrame, so
// the persisted parent (and the response's "parent") must be destFrame.
func TestSolveHandEyePersistsCaptureFrameAsParent(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "base"})
	// Confirm the constructor propagated DestinationFrame into destFrame.
	test.That(t, pt.destFrame, test.ShouldEqual, "base")
	pt.cameraName = "cam"
	for _, p := range solveHandEyeIdentityPairs() {
		pt.store.AddHandEyePair(p)
	}

	resp, err := pt.DoCommand(context.Background(), map[string]interface{}{"solve_handeye": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, resp["parent"], test.ShouldEqual, "base")
	test.That(t, pt.persist.(*persist.Fake).SavedParent, test.ShouldEqual, "base")
}

func TestSolveHandEyePersistErrorPreservesBuffer(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	pt.cameraName = "cam"
	// The solve will succeed, but the component-frame write fails: this exercises
	// the clear-after-persist ordering (buffer must survive a persist failure).
	pt.persist.(*persist.Fake).FrameErr = errors.New("boom")

	// Identity transform: solve succeeds, so we reach the persist step.
	pts := []frames.PointPair{
		{Camera: r3.Vector{X: 0, Y: 0, Z: 0}, Reference: r3.Vector{X: 0, Y: 0, Z: 0}},
		{Camera: r3.Vector{X: 100, Y: 0, Z: 0}, Reference: r3.Vector{X: 100, Y: 0, Z: 0}},
		{Camera: r3.Vector{X: 0, Y: 100, Z: 0}, Reference: r3.Vector{X: 0, Y: 100, Z: 0}},
		{Camera: r3.Vector{X: 0, Y: 0, Z: 100}, Reference: r3.Vector{X: 0, Y: 0, Z: 100}},
	}
	for _, p := range pts {
		pt.store.AddHandEyePair(p)
	}

	_, err := pt.DoCommand(context.Background(), map[string]interface{}{"solve_handeye": map[string]interface{}{}})
	test.That(t, err, test.ShouldNotBeNil)
	// Buffer preserved because the persist failed after a successful solve:
	test.That(t, pt.store.HandEyeBufferLen(), test.ShouldEqual, 4)
}

func TestSolveHandEyePersistDisabledErrors(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "arm", DestinationFrame: "world"})
	pt.persist = nil
	for i := 0; i < 3; i++ {
		pt.store.AddHandEyePair(frames.PointPair{})
	}
	_, err := pt.DoCommand(context.Background(), map[string]interface{}{"solve_handeye": map[string]interface{}{}})
	test.That(t, err, test.ShouldNotBeNil)
	// Buffer preserved on failure:
	test.That(t, pt.store.HandEyeBufferLen(), test.ShouldEqual, 3)
}

// TestSolveHandEyeEyeInHandPersistsArmAsParent is the direct analogue of
// TestSolveHandEyePersistsCaptureFrameAsParent for eye-in-hand: the persisted
// parent must be the arm, NOT destFrame. Getting this wrong is the same class of
// silent-miscalibration bug fixed in 4afb61e for eye-to-hand.
func TestSolveHandEyeEyeInHandPersistsArmAsParent(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "tcp", DestinationFrame: "world"})
	pt.cameraMount = mountEyeInHand
	pt.cameraName = "wrist_cam"
	pt.armName = "ur5"
	fp := &persist.Fake{}
	pt.persist = fp

	// Non-degenerate synthetic observations: reuse the frames package's geometry.
	truth := spatialmath.NewPose(r3.Vector{X: 30, Y: -20, Z: 50}, &spatialmath.EulerAngles{Roll: math.Pi / 2})
	xInv := spatialmath.PoseInverse(truth)
	target := r3.Vector{X: 400, Y: 100, Z: 0}
	flanges := []spatialmath.Pose{
		spatialmath.NewPose(r3.Vector{X: 300, Y: 0, Z: 400}, &spatialmath.EulerAngles{Yaw: 0}),
		spatialmath.NewPose(r3.Vector{X: 350, Y: 80, Z: 420}, &spatialmath.EulerAngles{Yaw: 0.3, Pitch: 0.2}),
		spatialmath.NewPose(r3.Vector{X: 280, Y: -60, Z: 380}, &spatialmath.EulerAngles{Yaw: -0.4, Roll: 0.25}),
		spatialmath.NewPose(r3.Vector{X: 320, Y: 40, Z: 450}, &spatialmath.EulerAngles{Pitch: -0.35, Roll: -0.2}),
	}
	for _, f := range flanges {
		q := spatialmath.Compose(spatialmath.PoseInverse(f), spatialmath.NewPoseFromPoint(target)).Point()
		pt.store.AddEyeInHandObservation(frames.EyeInHandObservation{
			Target: target,
			Flange: f,
			Camera: spatialmath.Compose(xInv, spatialmath.NewPoseFromPoint(q)).Point(),
		})
	}

	resp, err := pt.DoCommand(context.Background(), map[string]interface{}{"solve_handeye": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, resp["parent"], test.ShouldEqual, "ur5") // NOT "world"
	test.That(t, fp.SavedParent, test.ShouldEqual, "ur5")
	test.That(t, pt.store.EyeInHandBufferLen(), test.ShouldEqual, 0)
	test.That(t, pt.currentTarget, test.ShouldBeNil) // cleared on success
}

// TestSolveHandEyeEyeInHandRequiresArm guards against persisting frame
// {parent: ""} when armName is blank. Validate enforces this in production, but
// tests (and any future caller) set fields directly, so solveHandEye must
// re-check.
func TestSolveHandEyeEyeInHandRequiresArm(t *testing.T) {
	pt := newForTest(t, &Config{MotionService: "builtin", TCPComponent: "tcp", DestinationFrame: "world"})
	pt.cameraMount = mountEyeInHand
	pt.cameraName = "wrist_cam"
	pt.armName = ""
	for i := 0; i < 3; i++ {
		pt.store.AddEyeInHandObservation(frames.EyeInHandObservation{Flange: spatialmath.NewZeroPose()})
	}

	_, err := pt.DoCommand(context.Background(), map[string]interface{}{"solve_handeye": map[string]interface{}{}})
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "arm")
}
