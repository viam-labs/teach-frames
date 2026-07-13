package posesource

import (
	"context"
	"image"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/rimage"
	"go.viam.com/rdk/rimage/transform"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"

	injectmotion "go.viam.com/rdk/testutils/inject/motion"
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

func TestMotionSourceEmptyDestFrameUsesWorld(t *testing.T) {
	var capturedDest string
	injMotion := injectmotion.NewMotionService("builtin")
	injMotion.GetPoseFunc = func(
		_ context.Context,
		_ string,
		destinationFrame string,
		_ []*referenceframe.LinkInFrame,
		_ map[string]interface{},
	) (*referenceframe.PoseInFrame, error) {
		capturedDest = destinationFrame
		return referenceframe.NewPoseInFrame(referenceframe.World, spatialmath.NewZeroPose()), nil
	}

	ms := &MotionSource{
		Motion:    injMotion,
		Component: "arm",
		DestFrame: "", // deliberately empty
	}
	_, err := ms.Capture(context.Background())
	test.That(t, err, test.ShouldBeNil)
	test.That(t, capturedDest, test.ShouldEqual, referenceframe.World)
}

func TestFakeFlangeSource(t *testing.T) {
	want := spatialmath.NewPoseFromPoint(r3.Vector{X: 1, Y: 2, Z: 3})
	f := &FakeFlange{Poses: []spatialmath.Pose{want}}

	got, err := f.CaptureFlange(context.Background())
	test.That(t, err, test.ShouldBeNil)
	test.That(t, spatialmath.PoseAlmostEqual(got, want), test.ShouldBeTrue)

	_, err = f.CaptureFlange(context.Background())
	test.That(t, err, test.ShouldNotBeNil) // exhausted
}

func TestFlangeSourceImplementsInterface(t *testing.T) {
	var _ FlangeSource = (*ArmSource)(nil)
	var _ FlangeSource = (*FakeFlange)(nil)
}

func TestFakeCameraSourceSnapshot(t *testing.T) {
	intr := &transform.PinholeCameraIntrinsics{Width: 4, Height: 4, Fx: 2, Fy: 2, Ppx: 2, Ppy: 2}
	dm := rimage.NewEmptyDepthMap(4, 4)
	dm.Set(1, 1, rimage.Depth(500))
	rgb := image.NewRGBA(image.Rect(0, 0, 4, 4))

	src := &FakeCamera{RGB: rgb, Depth: dm, Intr: intr}
	snap, err := src.Snapshot(context.Background())
	test.That(t, err, test.ShouldBeNil)
	test.That(t, snap.Intr.Fx, test.ShouldEqual, 2.0)
	test.That(t, snap.Depth.GetDepth(1, 1), test.ShouldEqual, rimage.Depth(500))
	test.That(t, snap.RGB, test.ShouldNotBeNil)
}
