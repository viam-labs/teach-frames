package posesource

import (
	"context"
	"image"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/data"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/rimage"
	"go.viam.com/rdk/rimage/transform"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"

	injectmotion "go.viam.com/rdk/testutils/inject/motion"
	rutils "go.viam.com/rdk/utils"
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

func TestCameraSourceImplementsInterface(t *testing.T) {
	var _ CameraSource = (*RGBDCamera)(nil)
	var _ CameraSource = (*FakeCamera)(nil)
}

// namedImg builds a NamedImage whose backing image is uniquely identifiable so
// tests can assert exactly which stream was selected.
func namedImg(t *testing.T, src, mime string) (camera.NamedImage, image.Image) {
	t.Helper()
	img := image.NewRGBA(image.Rect(0, 0, 2, 2))
	ni, err := camera.NamedImageFromImage(img, src, mime, data.Annotations{})
	test.That(t, err, test.ShouldBeNil)
	return ni, img
}

func TestSelectColorDepth(t *testing.T) {
	t.Run("color-first order", func(t *testing.T) {
		color, cImg := namedImg(t, "color", rutils.MimeTypeJPEG)
		depth, dImg := namedImg(t, "depth", rutils.MimeTypeRawDepth)
		gotC, gotD, err := selectColorDepth(context.Background(), []camera.NamedImage{color, depth})
		test.That(t, err, test.ShouldBeNil)
		test.That(t, gotC, test.ShouldEqual, cImg)
		test.That(t, gotD, test.ShouldEqual, dImg)
	})

	t.Run("depth-first order", func(t *testing.T) {
		depth, dImg := namedImg(t, "depth", rutils.MimeTypeRawDepth)
		color, cImg := namedImg(t, "color", rutils.MimeTypeJPEG)
		gotC, gotD, err := selectColorDepth(context.Background(), []camera.NamedImage{depth, color})
		test.That(t, err, test.ShouldBeNil)
		test.That(t, gotC, test.ShouldEqual, cImg)
		test.That(t, gotD, test.ShouldEqual, dImg)
	})

	t.Run("depth identified by raw-depth mime", func(t *testing.T) {
		color, cImg := namedImg(t, "color", rutils.MimeTypeJPEG)
		depth, dImg := namedImg(t, "", rutils.MimeTypeRawDepth) // depth by mime, not name
		gotC, gotD, err := selectColorDepth(context.Background(), []camera.NamedImage{color, depth})
		test.That(t, err, test.ShouldBeNil)
		test.That(t, gotC, test.ShouldEqual, cImg)
		test.That(t, gotD, test.ShouldEqual, dImg)
	})

	t.Run("depth identified by source name", func(t *testing.T) {
		color, cImg := namedImg(t, "color", rutils.MimeTypeJPEG)
		depth, dImg := namedImg(t, "depth", rutils.MimeTypeJPEG) // depth by name, non-depth mime
		gotC, gotD, err := selectColorDepth(context.Background(), []camera.NamedImage{color, depth})
		test.That(t, err, test.ShouldBeNil)
		test.That(t, gotC, test.ShouldEqual, cImg)
		test.That(t, gotD, test.ShouldEqual, dImg)
	})

	t.Run("labeled color wins over earlier unlabeled stream", func(t *testing.T) {
		unlabeled, _ := namedImg(t, "", rutils.MimeTypeJPEG)
		color, cImg := namedImg(t, "color", rutils.MimeTypeJPEG)
		depth, _ := namedImg(t, "depth", rutils.MimeTypeRawDepth)
		gotC, _, err := selectColorDepth(context.Background(), []camera.NamedImage{unlabeled, color, depth})
		test.That(t, err, test.ShouldBeNil)
		test.That(t, gotC, test.ShouldEqual, cImg)
	})

	t.Run("infrared + depth, no color, errors", func(t *testing.T) {
		ir, _ := namedImg(t, "infrared", rutils.MimeTypeJPEG)
		depth, _ := namedImg(t, "depth", rutils.MimeTypeRawDepth)
		_, _, err := selectColorDepth(context.Background(), []camera.NamedImage{ir, depth})
		test.That(t, err, test.ShouldNotBeNil)
	})

	t.Run("color, no depth, errors", func(t *testing.T) {
		color, _ := namedImg(t, "color", rutils.MimeTypeJPEG)
		_, _, err := selectColorDepth(context.Background(), []camera.NamedImage{color})
		test.That(t, err, test.ShouldNotBeNil)
	})
}

func TestCheckResolutions(t *testing.T) {
	t.Run("all agree", func(t *testing.T) {
		test.That(t, checkResolutions(1280, 720, 1280, 720, 1280, 720), test.ShouldBeNil)
	})
	t.Run("depth differs from color", func(t *testing.T) {
		err := checkResolutions(1280, 720, 848, 480, 1280, 720)
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err.Error(), test.ShouldContainSubstring, "848x480")
	})
	t.Run("intrinsics differ from color", func(t *testing.T) {
		err := checkResolutions(1280, 720, 1280, 720, 848, 480)
		test.That(t, err, test.ShouldNotBeNil)
	})
	t.Run("height mismatch only", func(t *testing.T) {
		err := checkResolutions(640, 480, 640, 400, 640, 480)
		test.That(t, err, test.ShouldNotBeNil)
	})
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
