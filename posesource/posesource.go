// Package posesource abstracts reading the TCP pose for capture.
package posesource

import (
	"context"
	"errors"
	"fmt"
	"image"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/rimage"
	"go.viam.com/rdk/rimage/transform"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
	rutils "go.viam.com/rdk/utils"
)

// PoseSource reads the current TCP pose in the destination frame.
type PoseSource interface {
	Capture(ctx context.Context) (*referenceframe.PoseInFrame, error)
}

// MotionSource wraps a motion service GetPose call.
type MotionSource struct {
	Motion    motion.Service
	Component string
	// DestFrame is the reference frame for GetPose. Defaults to world if empty.
	DestFrame string
}

// Capture queries motion.GetPose for the configured component/destination frame.
// If DestFrame is empty it defaults to referenceframe.World rather than passing
// an empty string to GetPose.
func (m *MotionSource) Capture(ctx context.Context) (*referenceframe.PoseInFrame, error) {
	dest := m.DestFrame
	if dest == "" {
		dest = referenceframe.World
	}
	return m.Motion.GetPose(ctx, m.Component, dest, nil, nil)
}

// Fake returns queued poses, popping one per Capture. For tests.
type Fake struct {
	Poses []spatialmath.Pose
	idx   int
}

// Capture returns the next queued pose as a world-framed PoseInFrame.
func (f *Fake) Capture(_ context.Context) (*referenceframe.PoseInFrame, error) {
	if f.idx >= len(f.Poses) {
		return nil, errors.New("fake: no more queued poses")
	}
	p := f.Poses[f.idx]
	f.idx++
	return referenceframe.NewPoseInFrame(referenceframe.World, p), nil
}

// FlangeSource reads the arm flange pose (end position in the arm base frame).
type FlangeSource interface {
	CaptureFlange(ctx context.Context) (spatialmath.Pose, error)
}

// ArmSource wraps an arm's EndPosition call.
type ArmSource struct {
	Arm arm.Arm
}

// CaptureFlange returns the arm's current end position (flange pose in the arm
// base frame).
func (a *ArmSource) CaptureFlange(ctx context.Context) (spatialmath.Pose, error) {
	return a.Arm.EndPosition(ctx, nil)
}

// FakeFlange returns queued poses, popping one per call. For tests.
type FakeFlange struct {
	Poses []spatialmath.Pose
	idx   int
}

// CaptureFlange returns the next queued flange pose.
func (f *FakeFlange) CaptureFlange(_ context.Context) (spatialmath.Pose, error) {
	if f.idx >= len(f.Poses) {
		return nil, errors.New("fake: no more queued flange poses")
	}
	p := f.Poses[f.idx]
	f.idx++
	return p, nil
}

// Snapshot is one RGBD acquisition: a color image, an aligned depth map, and the
// camera intrinsics used to deproject depth pixels.
type Snapshot struct {
	RGB   image.Image
	Depth *rimage.DepthMap
	Intr  *transform.PinholeCameraIntrinsics
}

// CameraSource acquires an aligned RGB+depth snapshot with intrinsics.
type CameraSource interface {
	Snapshot(ctx context.Context) (*Snapshot, error)
}

// RGBDCamera adapts an RDK camera to CameraSource.
type RGBDCamera struct {
	Cam camera.Camera
}

// Snapshot fetches color+depth via Images() and intrinsics via Properties().
// It selects the color and depth images by mime type, falling back to source
// name ("color"/"depth") when mime types are ambiguous.
func (c *RGBDCamera) Snapshot(ctx context.Context) (*Snapshot, error) {
	imgs, _, err := c.Cam.Images(ctx, nil, nil)
	if err != nil {
		return nil, err
	}
	var colorImg, depthImg image.Image
	for i := range imgs {
		ni := &imgs[i]
		img, ierr := ni.Image(ctx)
		if ierr != nil {
			return nil, ierr
		}
		switch {
		case ni.MimeType() == rutils.MimeTypeRawDepth || ni.SourceName == "depth":
			depthImg = img
		case ni.SourceName == "color" || colorImg == nil:
			colorImg = img
		}
	}
	if colorImg == nil {
		return nil, errors.New("camera returned no color image")
	}
	if depthImg == nil {
		return nil, errors.New("camera returned no depth image; an RGBD/depth camera is required")
	}
	dm, err := rimage.ConvertImageToDepthMap(ctx, depthImg)
	if err != nil {
		return nil, fmt.Errorf("could not read depth map: %w", err)
	}
	props, err := c.Cam.Properties(ctx)
	if err != nil {
		return nil, err
	}
	if props.IntrinsicParams == nil {
		return nil, errors.New("camera exposes no intrinsics; cannot deproject (configure intrinsic_parameters on the camera)")
	}
	return &Snapshot{RGB: colorImg, Depth: dm, Intr: props.IntrinsicParams}, nil
}

// FakeCamera returns a fixed snapshot. For tests.
type FakeCamera struct {
	RGB   image.Image
	Depth *rimage.DepthMap
	Intr  *transform.PinholeCameraIntrinsics
	Err   error
}

// Snapshot returns the canned snapshot or the configured error.
func (f *FakeCamera) Snapshot(_ context.Context) (*Snapshot, error) {
	if f.Err != nil {
		return nil, f.Err
	}
	return &Snapshot{RGB: f.RGB, Depth: f.Depth, Intr: f.Intr}, nil
}
