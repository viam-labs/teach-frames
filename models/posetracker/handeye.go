package posetracker

import (
	"context"
	"encoding/base64"
	"errors"
	"fmt"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/rimage"
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

	pt.snapshotMu.Lock()
	pt.lastSnapshot = snap
	pt.snapshotMu.Unlock()

	b := snap.RGB.Bounds()
	return map[string]interface{}{
		"image":  base64.StdEncoding.EncodeToString(jpeg),
		"width":  b.Dx(),
		"height": b.Dy(),
	}, nil
}

// captureHandEyePoint deprojects the clicked pixel against the cached snapshot,
// reads the current TCP world pose, and stores the (world, camera) pair.
func (pt *teachTracker) captureHandEyePoint(ctx context.Context, raw interface{}) (map[string]interface{}, error) {
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

	idx := pt.store.AddHandEyePair(frames.HandEyePair{World: worldPt, Camera: camPt})
	return map[string]interface{}{
		"index":      idx,
		"buffer_len": pt.store.HandEyeBufferLen(),
		"world":      vecToMap(worldPt),
		"camera":     vecToMap(camPt),
	}, nil
}

// vecToMap renders an r3.Vector as a plain xyz map for DoCommand responses.
func vecToMap(v r3.Vector) map[string]interface{} {
	return map[string]interface{}{"x": v.X, "y": v.Y, "z": v.Z}
}

// solveHandEye runs the Kabsch camera->world solve over the buffer, persists the
// result to the camera component's own frame (parent world), reports the residual,
// and clears the buffer on success. On any failure the buffer is preserved.
func (pt *teachTracker) solveHandEye(ctx context.Context) (map[string]interface{}, error) {
	if pt.persist == nil {
		return nil, errors.New("persistence disabled (missing platform-API env vars); cannot commit camera calibration")
	}
	if pt.cameraName == "" {
		return nil, errors.New("camera dependency not configured; cannot solve hand-eye")
	}

	buf := pt.store.HandEyeBuffer()
	pose, residual, err := frames.ComputeCameraToWorld(buf)
	if err != nil {
		return nil, fmt.Errorf("hand-eye solve failed: %w", err)
	}

	// Serialize against other config-persisting commits.
	pt.commitMu.Lock()
	defer pt.commitMu.Unlock()

	translation := pose.Point()
	orientation := pose.Orientation()
	if perr := pt.persist.SaveComponentFrame(ctx, pt.cameraName, referenceframe.World, &translation, orientation); perr != nil {
		return nil, fmt.Errorf("persist failed, camera calibration not committed: %w", perr)
	}

	pt.store.ClearHandEyeBuffer()
	ov := orientation.OrientationVectorDegrees()
	return map[string]interface{}{
		"committed":    true,
		"pose":         poseToMap(pose),
		"residual_rms": residual,
		"parent":       referenceframe.World,
		"orientation":  map[string]interface{}{"o_x": ov.OX, "o_y": ov.OY, "o_z": ov.OZ, "theta": ov.Theta},
	}, nil
}
