package posetracker

import (
	"context"
	"encoding/base64"
	"errors"
	"fmt"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/rimage"
	"go.viam.com/rdk/spatialmath"
	rutils "go.viam.com/rdk/utils"

	"github.com/viam-labs/teach-frames/frames"
)

// handeyeSnapshot acquires an RGBD frame, caches the depth+intrinsics for the
// subsequent capture, and returns the RGB as a base64 JPEG for the UI to freeze.
func (pt *teachTracker) handeyeSnapshot(ctx context.Context) (map[string]interface{}, error) {
	if pt.cameraSrc == nil {
		return nil, errors.New("camera dependency not configured; cannot run hand-eye calibration")
	}
	snap, err := pt.cameraSrc.Snapshot(ctx)
	if err != nil {
		return nil, err
	}
	jpeg, err := rimage.EncodeImage(ctx, snap.RGB, rutils.MimeTypeJPEG)
	if err != nil {
		return nil, fmt.Errorf("could not encode snapshot: %w", err)
	}

	// Eye-in-hand only: freeze the flange pose WITH the image. The operator jogs,
	// snapshots, then clicks; a flange read at click time could describe a pose
	// the pixels never came from. Eye-to-hand touches and clicks at the same arm
	// pose and requires no arm at all, so this must stay gated.
	var flange spatialmath.Pose
	if pt.cameraMount == mountEyeInHand {
		if pt.flangeInDest == nil {
			return nil, errors.New("arm dependency not configured or not resolvable; cannot run eye-in-hand calibration")
		}
		pif, ferr := pt.flangeInDest.Capture(ctx)
		if ferr != nil {
			return nil, fmt.Errorf("could not read flange pose: %w", ferr)
		}
		flange = pif.Pose()
	}

	pt.snapshotMu.Lock()
	pt.lastSnapshot = snap
	pt.lastFlange = flange
	pt.snapshotMu.Unlock()

	b := snap.RGB.Bounds()
	return map[string]interface{}{
		"image":  base64.StdEncoding.EncodeToString(jpeg),
		"width":  b.Dx(),
		"height": b.Dy(),
	}, nil
}

// captureHandEyeTarget reads the current TCP pose and stores it as the target that
// subsequent views will be paired against. Eye-in-hand only: the arm cannot touch
// a point and view it from a distance at the same time, so touching and viewing
// are separate steps.
func (pt *teachTracker) captureHandEyeTarget(ctx context.Context) (map[string]interface{}, error) {
	if pt.cameraMount != mountEyeInHand {
		return nil, fmt.Errorf("camera_mount is %q; use capture_handeye_point instead", pt.cameraMount)
	}
	pif, err := pt.source.Capture(ctx)
	if err != nil {
		return nil, err
	}
	p := pif.Pose().Point()

	pt.targetMu.Lock()
	pt.currentTarget = &p
	pt.targetMu.Unlock()

	return map[string]interface{}{"target": vecToMap(p)}, nil
}

// captureHandEyePoint deprojects the clicked pixel against the cached snapshot,
// reads the current TCP world pose, and stores the (world, camera) pair.
func (pt *teachTracker) captureHandEyePoint(ctx context.Context, raw interface{}) (map[string]interface{}, error) {
	if pt.cameraMount != mountEyeToHand {
		return nil, fmt.Errorf("camera_mount is %q; use capture_handeye_target and capture_handeye_view instead", pt.cameraMount)
	}
	if pt.cameraSrc == nil {
		return nil, errors.New("camera dependency not configured; cannot capture hand-eye point")
	}
	args, ok := raw.(map[string]interface{})
	if !ok {
		return nil, errors.New("capture_handeye_point args must be an object")
	}
	uF, ok := args["u"].(float64)
	if !ok {
		return nil, errors.New("capture_handeye_point requires a numeric 'u' (pixel column)")
	}
	vF, ok := args["v"].(float64)
	if !ok {
		return nil, errors.New("capture_handeye_point requires a numeric 'v' (pixel row)")
	}

	pt.snapshotMu.Lock()
	snap := pt.lastSnapshot
	pt.snapshotMu.Unlock()
	if snap == nil {
		return nil, errors.New("no snapshot cached; run handeye_snapshot before capturing")
	}

	camPt, err := frames.Deproject(snap.Intr, snap.Depth, int(uF), int(vF))
	if err != nil {
		return nil, err
	}

	pif, err := pt.source.Capture(ctx)
	if err != nil {
		return nil, err
	}
	worldPt := pif.Pose().Point()

	idx := pt.store.AddHandEyePair(frames.PointPair{Reference: worldPt, Camera: camPt})
	return map[string]interface{}{
		"index":      idx,
		"buffer_len": pt.store.HandEyeBufferLen(),
		"world":      vecToMap(worldPt),
		"camera":     vecToMap(camPt),
	}, nil
}

// captureHandEyeView deprojects the clicked pixel against the cached snapshot and
// pairs it with the current target and the flange pose frozen at snapshot time.
func (pt *teachTracker) captureHandEyeView(ctx context.Context, raw interface{}) (map[string]interface{}, error) {
	if pt.cameraMount != mountEyeInHand {
		return nil, fmt.Errorf("camera_mount is %q; use capture_handeye_point instead", pt.cameraMount)
	}
	args, ok := raw.(map[string]interface{})
	if !ok {
		return nil, errors.New("capture_handeye_view args must be an object")
	}
	uF, ok := args["u"].(float64)
	if !ok {
		return nil, errors.New("capture_handeye_view requires a numeric 'u' (pixel column)")
	}
	vF, ok := args["v"].(float64)
	if !ok {
		return nil, errors.New("capture_handeye_view requires a numeric 'v' (pixel row)")
	}

	pt.targetMu.Lock()
	target := pt.currentTarget
	pt.targetMu.Unlock()
	if target == nil {
		return nil, errors.New("no target set; run capture_handeye_target before capturing a view")
	}

	// Take-and-clear in one critical section: each snapshot yields at most one
	// observation. Two clicks on one frozen frame would produce identical
	// observations, which is a degenerate set the solve must reject. Clearing
	// here rather than after a successful deproject keeps this race-free; a
	// failed deproject just means the operator re-snapshots.
	pt.snapshotMu.Lock()
	snap := pt.lastSnapshot
	flange := pt.lastFlange
	pt.lastSnapshot = nil
	pt.lastFlange = nil
	pt.snapshotMu.Unlock()
	if snap == nil {
		return nil, errors.New("no snapshot cached; run handeye_snapshot before each view")
	}

	camPt, err := frames.Deproject(snap.Intr, snap.Depth, int(uF), int(vF))
	if err != nil {
		return nil, err
	}

	idx := pt.store.AddEyeInHandObservation(frames.EyeInHandObservation{
		Target: *target,
		Flange: flange,
		Camera: camPt,
	})
	return map[string]interface{}{
		"index":      idx,
		"buffer_len": pt.store.EyeInHandBufferLen(),
		"target":     vecToMap(*target),
		"flange":     poseToMap(flange),
		"camera":     vecToMap(camPt),
	}, nil
}

// vecToMap renders an r3.Vector as a plain xyz map for DoCommand responses.
func vecToMap(v r3.Vector) map[string]interface{} {
	return map[string]interface{}{"x": v.X, "y": v.Y, "z": v.Z}
}

// solveHandEye runs the Kabsch solve over the mode's buffer -- camera->destFrame
// for eye-to-hand, camera->flange for eye-in-hand -- and persists the result to
// the camera component's own frame. The parent differs by mount: pt.destFrame
// (the frame the captured world points are expressed in) for eye-to-hand, or
// pt.armName (the frame system attaches a child at the arm's kinematic tip) for
// eye-in-hand. Reports the residual and clears the buffer (and, for eye-in-hand,
// the current target) on success. On any failure the buffer is preserved.
func (pt *teachTracker) solveHandEye(ctx context.Context) (map[string]interface{}, error) {
	if pt.persist == nil {
		return nil, errors.New("persistence disabled (missing platform-API env vars); cannot commit camera calibration")
	}
	if pt.cameraName == "" {
		return nil, errors.New("camera dependency not configured; cannot solve hand-eye")
	}
	if pt.cameraMount == mountEyeInHand && pt.armName == "" {
		return nil, errors.New("arm not configured; cannot solve eye-in-hand calibration")
	}

	var pose spatialmath.Pose
	var residual float64
	var err error
	var parent string

	if pt.cameraMount == mountEyeInHand {
		// Observations pair a destFrame target with a destFrame flange pose, so
		// the solve yields camera->flange. The frame system attaches a child with
		// parent = <arm> at the arm's kinematic tip (the flange), so that is the
		// correct parent -- NOT destFrame.
		pose, residual, err = frames.ComputeCameraToFlange(pt.store.EyeInHandBuffer())
		parent = pt.armName
	} else {
		// Captured points come from pt.source.Capture in pt.destFrame, so the
		// solve yields camera->destFrame.
		pose, residual, err = frames.ComputeRigidTransform(pt.store.HandEyeBuffer())
		parent = pt.destFrame
	}
	if err != nil {
		return nil, fmt.Errorf("hand-eye solve failed: %w", err)
	}

	// Serialize against other config-persisting commits.
	pt.commitMu.Lock()
	defer pt.commitMu.Unlock()

	// The captured world points come from pt.source.Capture, which expresses
	// poses in the configured destination frame (pt.destFrame). The Kabsch solve
	// therefore yields camera->destFrame, so persist that as the camera's parent —
	// NOT the literal "world" (which would silently miscalibrate when
	// destination_frame is set to something other than the default "world").
	translation := pose.Point()
	orientation := pose.Orientation()
	if perr := pt.persist.SaveComponentFrame(ctx, pt.cameraName, parent, &translation, orientation); perr != nil {
		return nil, fmt.Errorf("persist failed, camera calibration not committed: %w", perr)
	}

	if pt.cameraMount == mountEyeInHand {
		pt.store.ClearEyeInHandBuffer()
		pt.targetMu.Lock()
		pt.currentTarget = nil
		pt.targetMu.Unlock()
	} else {
		pt.store.ClearHandEyeBuffer()
	}
	ov := orientation.OrientationVectorDegrees()
	return map[string]interface{}{
		"committed":    true,
		"pose":         poseToMap(pose),
		"residual_rms": residual,
		"parent":       parent,
		"orientation":  map[string]interface{}{"o_x": ov.OX, "o_y": ov.OY, "o_z": ov.OZ, "theta": ov.Theta},
	}, nil
}
