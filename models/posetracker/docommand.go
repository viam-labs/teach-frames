package posetracker

import (
	"context"
	"fmt"

	"go.viam.com/rdk/spatialmath"
)

// DoCommand dispatches buffer-management verbs for the teach-frames pose tracker.
// Supported commands (single-key maps):
//
//	{"capture_point": {}} — read the current TCP pose and append it to the capture buffer.
//	{"get_buffer": {}}    — return all poses currently in the capture buffer.
//	{"clear_buffer": {}}  — empty the capture buffer and return the count removed.
func (pt *teachTracker) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
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
	}

	return nil, fmt.Errorf("unknown command: %v", keysOf(cmd))
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
