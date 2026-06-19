package posetracker

import (
	"context"
	"errors"
	"fmt"

	"go.viam.com/rdk/spatialmath"

	"github.com/viam-labs/teach-frames/config"
	"github.com/viam-labs/teach-frames/frames"
)

// DoCommand dispatches buffer-management verbs for the teach-frames pose tracker.
// Supported commands (single-key maps):
//
//	{"capture_point": {}}  — read the current TCP pose and append it to the capture buffer.
//	{"get_buffer": {}}     — return all poses currently in the capture buffer.
//	{"clear_buffer": {}}   — empty the capture buffer and return the count removed.
//	{"define_frame": {...}} — compute a frame from the buffer, persist it, and clear the buffer.
func (pt *teachTracker) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	if len(cmd) != 1 {
		return nil, fmt.Errorf("expected exactly one command key, got %d: %v", len(cmd), keysOf(cmd))
	}

	switch {
	case has(cmd, "capture_point"):
		pif, err := pt.source.Capture(ctx)
		if err != nil {
			return nil, err
		}
		idx := pt.store.AddCapture(pif.Pose())
		return map[string]interface{}{
			"index":      idx,
			"buffer_len": pt.store.BufferLen(),
			"pose":       poseToMap(pif.Pose()),
		}, nil

	case has(cmd, "get_buffer"):
		buf := pt.store.Buffer()
		pts := make([]interface{}, len(buf))
		for i, p := range buf {
			pts[i] = poseToMap(p)
		}
		return map[string]interface{}{"points": pts}, nil

	case has(cmd, "clear_buffer"):
		return map[string]interface{}{"cleared": pt.store.ClearBuffer()}, nil

	case has(cmd, "define_frame"):
		return pt.defineFrame(ctx, cmd["define_frame"])
	}

	return nil, fmt.Errorf("unknown command: %v", keysOf(cmd))
}

// defineFrame computes a frame from the capture buffer using the given method,
// persists the full committed set, and clears the buffer on success.
func (pt *teachTracker) defineFrame(ctx context.Context, raw interface{}) (map[string]interface{}, error) {
	args, _ := raw.(map[string]interface{})
	name, _ := args["name"].(string)
	method, _ := args["method"].(string)

	if name == "" {
		return nil, errors.New("define_frame requires a non-empty name")
	}
	if pt.persist == nil {
		return nil, errors.New("persistence disabled (missing platform-API env vars); cannot commit frame")
	}

	buf := pt.store.Buffer()
	var pose spatialmath.Pose
	var err error
	switch method {
	case "3point":
		if len(buf) < 3 {
			return nil, fmt.Errorf("3point requires 3 captured points, have %d", len(buf))
		}
		pose, err = frames.ComputeThreePoint(buf[0].Point(), buf[1].Point(), buf[2].Point())
	case "point":
		if len(buf) < 1 {
			return nil, fmt.Errorf("point requires 1 captured point, have %d", len(buf))
		}
		pose = frames.ComputePoint(buf[0].Point())
	case "tcp_snapshot":
		if len(buf) < 1 {
			return nil, fmt.Errorf("tcp_snapshot requires 1 captured point, have %d", len(buf))
		}
		pose = buf[0]
	default:
		return nil, fmt.Errorf("unknown method %q (want 3point|point|tcp_snapshot)", method)
	}
	if err != nil {
		return nil, err
	}

	// Capture the previous frame (if any) so we can roll back on persist failure.
	prev, hadPrev := pt.store.GetFrame(name)

	replaced := pt.store.SetFrame(name, pt.destFrame, pose)
	if perr := pt.saveFrames(ctx); perr != nil {
		// Roll back the optimistic in-memory write so in-memory state matches durable config.
		if hadPrev {
			// Restore the prior frame — do NOT just delete, since the prior frame was valid.
			pt.store.SetFrame(name, prev.Parent(), prev.Pose())
		} else {
			pt.store.DeleteFrame(name)
		}
		return nil, fmt.Errorf("persist failed, frame not committed: %w", perr)
	}

	pt.store.ClearBuffer()
	return map[string]interface{}{
		"name":      name,
		"committed": true,
		"replaced":  replaced,
		"pose":      poseToMap(pose),
	}, nil
}

// saveFrames serializes the full committed set and persists it.
// Note: method is passed as "" because the store does not currently track the compute method.
// This is acceptable for v1; method-tracking can be added to the store in a future task.
func (pt *teachTracker) saveFrames(ctx context.Context) error {
	names := pt.store.FrameNames()
	specs := make([]config.FrameSpec, 0, len(names))
	for _, n := range names {
		pif, ok := pt.store.GetFrame(n)
		if !ok {
			continue
		}
		specs = append(specs, config.FrameSpecFromPose(n, "", pif.Parent(), pif.Pose()))
	}
	return pt.persist.Save(ctx, specs)
}

// has reports whether key k is present in m.
func has(m map[string]interface{}, k string) bool {
	_, ok := m[k]
	return ok
}

// keysOf returns the keys of m as a slice (for error messages).
func keysOf(m map[string]interface{}) []string {
	ks := make([]string, 0, len(m))
	for k := range m {
		ks = append(ks, k)
	}
	return ks
}

// poseToMap converts a spatialmath.Pose to a plain map for DoCommand responses.
func poseToMap(p spatialmath.Pose) map[string]interface{} {
	pt := p.Point()
	ov := p.Orientation().OrientationVectorDegrees()
	return map[string]interface{}{
		"x":     pt.X,
		"y":     pt.Y,
		"z":     pt.Z,
		"o_x":   ov.OX,
		"o_y":   ov.OY,
		"o_z":   ov.OZ,
		"theta": ov.Theta,
	}
}
