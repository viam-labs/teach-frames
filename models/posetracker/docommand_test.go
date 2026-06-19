package posetracker

import (
	"context"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"

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
