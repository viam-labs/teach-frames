// Package config holds the JSON-serializable config types for the module.
package config

import (
	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
)

// PoseSpec is the JSON form of a pose (orientation-vector degrees).
type PoseSpec struct {
	X     float64 `json:"x"`
	Y     float64 `json:"y"`
	Z     float64 `json:"z"`
	OX    float64 `json:"o_x"`
	OY    float64 `json:"o_y"`
	OZ    float64 `json:"o_z"`
	Theta float64 `json:"theta"`
}

// FrameSpec is one taught frame as stored in config.
type FrameSpec struct {
	Name   string   `json:"name"`
	Method string   `json:"method"`
	Parent string   `json:"parent"`
	Pose   PoseSpec `json:"pose"`
}

// FrameSpecFromPose builds a FrameSpec from a spatialmath.Pose.
func FrameSpecFromPose(name, method, parent string, pose spatialmath.Pose) FrameSpec {
	pt := pose.Point()
	ov := pose.Orientation().OrientationVectorDegrees()
	return FrameSpec{
		Name: name, Method: method, Parent: parent,
		Pose: PoseSpec{X: pt.X, Y: pt.Y, Z: pt.Z, OX: ov.OX, OY: ov.OY, OZ: ov.OZ, Theta: ov.Theta},
	}
}

// ToPose converts a FrameSpec back to a spatialmath.Pose.
func (f FrameSpec) ToPose() (spatialmath.Pose, error) {
	ov := &spatialmath.OrientationVectorDegrees{OX: f.Pose.OX, OY: f.Pose.OY, OZ: f.Pose.OZ, Theta: f.Pose.Theta}
	return spatialmath.NewPose(r3.Vector{X: f.Pose.X, Y: f.Pose.Y, Z: f.Pose.Z}, ov), nil
}
