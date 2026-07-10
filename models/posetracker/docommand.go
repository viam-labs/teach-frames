package posetracker

import (
	"context"
	"errors"
	"fmt"
	"math"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"

	"github.com/viam-labs/teach-frames/config"
	"github.com/viam-labs/teach-frames/frames"
)

// DoCommand dispatches buffer-management and frame-management verbs for the
// teach-frames pose tracker. Supported commands (single-key maps):
//
//	{"capture_point": {}}                                    — read the current TCP pose and append it to the capture buffer.
//	{"get_buffer": {}}                                       — return all poses currently in the capture buffer.
//	{"clear_buffer": {}}                                     — empty the capture buffer and return the count removed.
//	{"capture_tcp_point": {}}                                — read the current arm flange pose and append it to the TCP capture buffer (errors if no arm is configured).
//	{"get_tcp_buffer": {}}                                   — return all poses currently in the TCP capture buffer.
//	{"clear_tcp_buffer": {}}                                 — empty the TCP capture buffer and return the count removed.
//	{"define_frame": {"name": "<n>", "method": "<m>"}}       — compute a frame from the buffer (methods: 3point|point|tcp_snapshot), persist it, and clear the buffer.
//	{"list_frames": {}}                                      — return all currently committed frames.
//	{"delete_frame": {"name": "<n>"}}                        — remove a single named frame and persist the updated set.
//	{"clear_frames": {}}                                     — remove all committed frames and persist the empty set.
//	{"teach_tcp_position": {}}                               — solve the pivot over the TCP buffer, persist the tool tip as the tcp_component's frame translation, and clear the TCP buffer.
//	{"teach_tcp_orientation": {"o_x": 0, "o_y": 0, "o_z": 1, "theta": 0}} — persist an explicit tool orientation (OV degrees) to the tcp_component's frame, leaving the taught translation intact. Run after teach_tcp_position, since it writes orientation only and does not re-derive the translation.
//	{"get_arm_state": {}}                                    — return the current TCP pose and joint positions (degrees) in one call, for the UI poll loop (errors if no arm is configured).
//	{"stop_arm": {}}                                          — stop arm motion immediately (errors if no arm is configured).
//	{"jog_joint": {"joint": 0, "step": 5}}                    — nudge a single joint by a signed step (degrees) and move the arm there (errors if no arm is configured or the joint index is out of range).
//	{"jog_cartesian": {"axis": "x", "step": 5}}               — nudge the TCP by a signed step: x/y/z (mm) translate along the world frame, roll/pitch/yaw (degrees) rotate about the tool frame (errors if no arm is configured or the axis is unknown).
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

	case has(cmd, "capture_tcp_point"):
		if pt.flange == nil {
			return nil, errors.New("arm dependency not configured; cannot capture TCP point")
		}
		pose, err := pt.flange.CaptureFlange(ctx)
		if err != nil {
			return nil, err
		}
		idx := pt.store.AddTCPCapture(pose)
		return map[string]interface{}{
			"index":      idx,
			"buffer_len": pt.store.TCPBufferLen(),
			"pose":       poseToMap(pose),
		}, nil

	case has(cmd, "get_tcp_buffer"):
		buf := pt.store.TCPBuffer()
		pts := make([]interface{}, len(buf))
		for i, p := range buf {
			pts[i] = poseToMap(p)
		}
		return map[string]interface{}{"points": pts}, nil

	case has(cmd, "clear_tcp_buffer"):
		return map[string]interface{}{"cleared": pt.store.ClearTCPBuffer()}, nil

	case has(cmd, "define_frame"):
		return pt.defineFrame(ctx, cmd["define_frame"])

	case has(cmd, "list_frames"):
		out := map[string]interface{}{}
		for n, pif := range pt.store.Snapshot(nil) {
			out[n] = poseToMap(pif.Pose())
		}
		return map[string]interface{}{"frames": out}, nil

	case has(cmd, "delete_frame"):
		return pt.deleteFrame(ctx, cmd["delete_frame"])

	case has(cmd, "clear_frames"):
		return pt.clearFrames(ctx)

	case has(cmd, "teach_tcp_position"):
		return pt.teachTCPPosition(ctx)

	case has(cmd, "teach_tcp_orientation"):
		return pt.teachTCPOrientation(ctx, cmd["teach_tcp_orientation"])

	case has(cmd, "get_arm_state"):
		return pt.getArmState(ctx)

	case has(cmd, "stop_arm"):
		if pt.arm == nil {
			return nil, errors.New("arm dependency not configured; cannot stop arm")
		}
		if err := pt.arm.Stop(ctx, nil); err != nil {
			return nil, err
		}
		return map[string]interface{}{"stopped": true}, nil

	case has(cmd, "jog_joint"):
		return pt.jogJoint(ctx, cmd["jog_joint"])

	case has(cmd, "jog_cartesian"):
		return pt.jogCartesian(ctx, cmd["jog_cartesian"])
	}

	return nil, fmt.Errorf("unknown command: %v", keysOf(cmd))
}

// defineFrame computes a frame from the capture buffer using the given method,
// persists the full committed set, and clears the buffer on success.
func (pt *teachTracker) defineFrame(ctx context.Context, raw interface{}) (map[string]interface{}, error) {
	args, ok := raw.(map[string]interface{})
	if !ok {
		return nil, errors.New("define_frame args must be an object")
	}

	var name string
	if nameVal, hasName := args["name"]; hasName {
		name, ok = nameVal.(string)
		if !ok {
			return nil, errors.New("name must be a string")
		}
	}

	var method string
	if methodVal, hasMethod := args["method"]; hasMethod {
		method, ok = methodVal.(string)
		if !ok {
			return nil, errors.New("method must be a string")
		}
	}

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
		if err != nil {
			return nil, fmt.Errorf("3point compute failed: %w", err)
		}
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

	// commitMu serializes the get→set→persist→rollback sequence so concurrent
	// define calls (and future delete/clear) cannot interleave and corrupt the
	// persisted set or produce an incorrect rollback.
	pt.commitMu.Lock()
	defer pt.commitMu.Unlock()

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

// deleteFrame removes a single named frame from the store, persists the updated
// set, and restores the frame on persist failure (rollback).
func (pt *teachTracker) deleteFrame(ctx context.Context, raw interface{}) (map[string]interface{}, error) {
	args, ok := raw.(map[string]interface{})
	if !ok {
		return nil, errors.New("delete_frame args must be an object")
	}
	nameVal, present := args["name"]
	if !present {
		return nil, errors.New("delete_frame requires a non-empty name")
	}
	name, isStr := nameVal.(string)
	if !isStr {
		return nil, errors.New("name must be a string")
	}
	if name == "" {
		return nil, errors.New("delete_frame requires a non-empty name")
	}
	if pt.persist == nil {
		return nil, errors.New("persistence disabled (missing platform-API env vars); cannot delete frame")
	}

	pt.commitMu.Lock()
	defer pt.commitMu.Unlock()

	prev, hadPrev := pt.store.GetFrame(name)
	existed := pt.store.DeleteFrame(name)
	if err := pt.saveFrames(ctx); err != nil {
		if hadPrev {
			pt.store.SetFrame(name, prev.Parent(), prev.Pose())
		}
		return nil, fmt.Errorf("persist failed, delete not committed: %w", err)
	}
	return map[string]interface{}{"deleted": existed}, nil
}

// clearFrames removes all committed frames, persists the empty set, and restores
// all frames on persist failure (rollback).
func (pt *teachTracker) clearFrames(ctx context.Context) (map[string]interface{}, error) {
	if pt.persist == nil {
		return nil, errors.New("persistence disabled (missing platform-API env vars); cannot clear frames")
	}

	pt.commitMu.Lock()
	defer pt.commitMu.Unlock()

	prev := pt.store.Snapshot(nil) // defensive copies for rollback
	n := pt.store.ClearFrames()
	if err := pt.saveFrames(ctx); err != nil {
		for name, pif := range prev {
			pt.store.SetFrame(name, pif.Parent(), pif.Pose())
		}
		return nil, fmt.Errorf("persist failed, clear not committed: %w", err)
	}
	return map[string]interface{}{"deleted": n}, nil
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

// teachTCPPosition solves the pivot over the TCP capture buffer, persists the
// resolved tool tip as the tcp_component's frame translation (parent = arm),
// and clears the TCP buffer on success.
func (pt *teachTracker) teachTCPPosition(ctx context.Context) (map[string]interface{}, error) {
	if pt.persist == nil {
		return nil, errors.New("persistence disabled (missing platform-API env vars); cannot teach TCP")
	}
	if pt.armName == "" {
		return nil, errors.New("arm dependency not configured; cannot teach TCP")
	}

	buf := pt.store.TCPBuffer()
	offset, residual, err := frames.ComputePivotTCP(buf)
	if err != nil {
		return nil, fmt.Errorf("pivot solve failed: %w", err)
	}

	// commitMu serializes this persist against concurrent define/delete/clear/teach
	// commits so the persisted state stays consistent.
	pt.commitMu.Lock()
	defer pt.commitMu.Unlock()

	off := offset
	if perr := pt.persist.SaveComponentFrame(ctx, pt.tcpComponent, pt.armName, &off, nil); perr != nil {
		return nil, fmt.Errorf("persist failed, TCP not committed: %w", perr)
	}

	pt.store.ClearTCPBuffer()
	return map[string]interface{}{
		"committed":    true,
		"offset":       map[string]interface{}{"x": offset.X, "y": offset.Y, "z": offset.Z},
		"residual_rms": residual,
	}, nil
}

// teachTCPOrientation persists an explicit tool orientation (OV degrees) to the
// tcp_component's frame, leaving the taught translation intact. Tool orientation
// cannot be derived from tip touches, so it is supplied by the operator.
func (pt *teachTracker) teachTCPOrientation(ctx context.Context, raw interface{}) (map[string]interface{}, error) {
	args, ok := raw.(map[string]interface{})
	if !ok {
		return nil, errors.New("teach_tcp_orientation args must be an object")
	}
	if pt.persist == nil {
		return nil, errors.New("persistence disabled (missing platform-API env vars); cannot teach TCP orientation")
	}
	if pt.armName == "" {
		return nil, errors.New("arm dependency not configured; cannot teach TCP")
	}

	// num reads an optional numeric key: a MISSING key defaults to 0, but a key
	// that is PRESENT with a non-float64 value is a typo/type error and must
	// surface rather than silently coercing to 0 (which could persist the wrong
	// orientation). A mistyped key NAME is indistinguishable from a missing key
	// and cannot be caught here.
	num := func(k string) (float64, error) {
		v, present := args[k]
		if !present {
			return 0, nil
		}
		f, ok := v.(float64)
		if !ok {
			return 0, fmt.Errorf("%s must be a number", k)
		}
		return f, nil
	}
	ox, err := num("o_x")
	if err != nil {
		return nil, err
	}
	oy, err := num("o_y")
	if err != nil {
		return nil, err
	}
	oz, err := num("o_z")
	if err != nil {
		return nil, err
	}
	theta, err := num("theta")
	if err != nil {
		return nil, err
	}
	ov := &spatialmath.OrientationVectorDegrees{OX: ox, OY: oy, OZ: oz, Theta: theta}
	if ov.OX == 0 && ov.OY == 0 && ov.OZ == 0 {
		return nil, errors.New("orientation vector (o_x,o_y,o_z) must be non-zero")
	}

	// commitMu serializes this persist against concurrent teach/define commits on
	// the same tcp_component frame so the persisted state stays consistent.
	pt.commitMu.Lock()
	defer pt.commitMu.Unlock()

	if perr := pt.persist.SaveComponentFrame(ctx, pt.tcpComponent, pt.armName, nil, ov); perr != nil {
		return nil, fmt.Errorf("persist failed, TCP orientation not committed: %w", perr)
	}
	return map[string]interface{}{
		"committed":   true,
		"orientation": map[string]interface{}{"o_x": ov.OX, "o_y": ov.OY, "o_z": ov.OZ, "theta": ov.Theta},
	}, nil
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

// getArmState returns the current TCP pose and joint positions (degrees) for the UI.
func (pt *teachTracker) getArmState(ctx context.Context) (map[string]interface{}, error) {
	if pt.arm == nil {
		return nil, errors.New("arm dependency not configured; cannot read arm state")
	}
	pose, err := pt.arm.EndPosition(ctx, nil)
	if err != nil {
		return nil, err
	}
	joints, err := pt.arm.JointPositions(ctx, nil)
	if err != nil {
		return nil, err
	}
	return map[string]interface{}{
		"pose":   poseToMap(pose),
		"joints": inputsToDegrees(joints),
	}, nil
}

// jogJoint nudges a single joint by a signed step (degrees) and moves the arm there.
func (pt *teachTracker) jogJoint(ctx context.Context, raw interface{}) (map[string]interface{}, error) {
	if pt.arm == nil {
		return nil, errors.New("arm dependency not configured; cannot jog")
	}
	args, ok := raw.(map[string]interface{})
	if !ok {
		return nil, errors.New("jog_joint args must be an object")
	}
	jointF, ok := args["joint"].(float64)
	if !ok {
		return nil, errors.New("jog_joint requires a numeric 'joint' index")
	}
	step, ok := args["step"].(float64)
	if !ok {
		return nil, errors.New("jog_joint requires a numeric 'step' (degrees)")
	}
	joint := int(jointF)

	cur, err := pt.arm.JointPositions(ctx, nil)
	if err != nil {
		return nil, err
	}
	if joint < 0 || joint >= len(cur) {
		return nil, fmt.Errorf("joint index %d out of range [0,%d)", joint, len(cur))
	}
	next := make([]referenceframe.Input, len(cur))
	copy(next, cur)
	next[joint] = cur[joint] + step*math.Pi/180.0 // Input is a float64 alias (radians)

	if err := pt.arm.MoveToJointPositions(ctx, next, nil); err != nil {
		return nil, err
	}
	return map[string]interface{}{"joints": inputsToDegrees(next)}, nil
}

// jogCartesian nudges the TCP by a signed step. Translation axes (x/y/z, mm) are
// world-frame; rotation axes (roll/pitch/yaw, degrees) are tool-frame.
func (pt *teachTracker) jogCartesian(ctx context.Context, raw interface{}) (map[string]interface{}, error) {
	if pt.arm == nil {
		return nil, errors.New("arm dependency not configured; cannot jog")
	}
	args, ok := raw.(map[string]interface{})
	if !ok {
		return nil, errors.New("jog_cartesian args must be an object")
	}
	axis, ok := args["axis"].(string)
	if !ok {
		return nil, errors.New("jog_cartesian requires a string 'axis'")
	}
	step, ok := args["step"].(float64)
	if !ok {
		return nil, errors.New("jog_cartesian requires a numeric 'step'")
	}

	cur, err := pt.arm.EndPosition(ctx, nil)
	if err != nil {
		return nil, err
	}

	var next spatialmath.Pose
	switch axis {
	case "x":
		next = spatialmath.NewPose(cur.Point().Add(r3.Vector{X: step}), cur.Orientation())
	case "y":
		next = spatialmath.NewPose(cur.Point().Add(r3.Vector{Y: step}), cur.Orientation())
	case "z":
		next = spatialmath.NewPose(cur.Point().Add(r3.Vector{Z: step}), cur.Orientation())
	case "roll", "pitch", "yaw":
		return nil, errors.New("rotation jog not yet implemented")
	default:
		return nil, fmt.Errorf("unknown jog axis %q (want x|y|z|roll|pitch|yaw)", axis)
	}

	if err := pt.arm.MoveToPosition(ctx, next, nil); err != nil {
		return nil, err
	}
	return map[string]interface{}{"pose": poseToMap(next), "moved": true}, nil
}

// inputsToDegrees converts joint inputs (radians) to a plain []float64 in degrees.
func inputsToDegrees(inputs []referenceframe.Input) []float64 {
	out := make([]float64, len(inputs))
	for i, in := range inputs {
		out[i] = float64(in) * 180.0 / math.Pi
	}
	return out
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
