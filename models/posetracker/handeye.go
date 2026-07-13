package posetracker

import (
	"context"
	"encoding/base64"
	"errors"
	"fmt"

	"go.viam.com/rdk/rimage"
	rutils "go.viam.com/rdk/utils"
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
