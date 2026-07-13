package frames

import (
	"image"
	"math"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/rimage"
	"go.viam.com/rdk/rimage/transform"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
)

func TestComputeCameraToWorldRoundTrip(t *testing.T) {
	// Known camera->world transform: rotate 90° about Z, translate (100,-50,300).
	rot := spatialmath.NewPoseFromOrientation(&spatialmath.EulerAngles{Yaw: math.Pi / 2})
	truth := spatialmath.NewPose(r3.Vector{X: 100, Y: -50, Z: 300}, rot.Orientation())

	camPts := []r3.Vector{{X: 0, Y: 0, Z: 0}, {X: 100, Y: 0, Z: 0}, {X: 0, Y: 100, Z: 0}, {X: 0, Y: 0, Z: 100}}
	pairs := make([]HandEyePair, len(camPts))
	for i, c := range camPts {
		w := spatialmath.Compose(truth, spatialmath.NewPoseFromPoint(c)).Point()
		pairs[i] = HandEyePair{Camera: c, World: w}
	}

	got, residual, err := ComputeCameraToWorld(pairs)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, residual, test.ShouldBeLessThan, 1e-6)
	test.That(t, spatialmath.PoseAlmostEqualEps(got, truth, 1e-6), test.ShouldBeTrue)
}

func TestComputeCameraToWorldRejectsCollinear(t *testing.T) {
	pairs := []HandEyePair{
		{Camera: r3.Vector{X: 0}, World: r3.Vector{X: 0}},
		{Camera: r3.Vector{X: 1}, World: r3.Vector{X: 1}},
		{Camera: r3.Vector{X: 2}, World: r3.Vector{X: 2}},
	}
	_, _, err := ComputeCameraToWorld(pairs)
	test.That(t, err, test.ShouldNotBeNil)
}

func TestComputeCameraToWorldRejectsTooFew(t *testing.T) {
	_, _, err := ComputeCameraToWorld([]HandEyePair{{}, {}})
	test.That(t, err, test.ShouldNotBeNil)
}

func TestDeproject(t *testing.T) {
	intr := &transform.PinholeCameraIntrinsics{Width: 100, Height: 100, Fx: 50, Fy: 50, Ppx: 50, Ppy: 50}
	dm := rimage.NewEmptyDepthMap(100, 100)
	dm.Set(60, 50, rimage.Depth(1000)) // 1 m at pixel (60,50)

	p, err := Deproject(intr, dm, 60, 50)
	test.That(t, err, test.ShouldBeNil)
	// x = (60-50)/50 * 1000 = 200; y = (50-50)/50*1000 = 0; z = 1000
	test.That(t, p.X, test.ShouldAlmostEqual, 200.0)
	test.That(t, p.Y, test.ShouldAlmostEqual, 0.0)
	test.That(t, p.Z, test.ShouldAlmostEqual, 1000.0)
}

func TestDeprojectZeroDepthErrors(t *testing.T) {
	intr := &transform.PinholeCameraIntrinsics{Width: 100, Height: 100, Fx: 50, Fy: 50, Ppx: 50, Ppy: 50}
	dm := rimage.NewEmptyDepthMap(100, 100)
	_, err := Deproject(intr, dm, 10, 10) // depth 0
	test.That(t, err, test.ShouldNotBeNil)
}

func TestDeprojectOutOfBoundsErrors(t *testing.T) {
	intr := &transform.PinholeCameraIntrinsics{Width: 100, Height: 100, Fx: 50, Fy: 50, Ppx: 50, Ppy: 50}
	dm := rimage.NewEmptyDepthMap(100, 100)
	_, err := Deproject(intr, dm, 200, 10)
	test.That(t, err, test.ShouldNotBeNil)
	_ = image.Point{} // keep image import used if trimmed
}
