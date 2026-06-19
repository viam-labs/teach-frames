// Package posesource abstracts reading the TCP pose for capture.
package posesource

import (
	"context"
	"errors"

	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
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
