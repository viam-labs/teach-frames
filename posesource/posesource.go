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
	DestFrame string
}

// Capture queries motion.GetPose for the configured component/destination frame.
func (m *MotionSource) Capture(ctx context.Context) (*referenceframe.PoseInFrame, error) {
	return m.Motion.GetPose(ctx, m.Component, m.DestFrame, nil, nil)
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
