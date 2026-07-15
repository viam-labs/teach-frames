// Eye-in-hand (wrist-mounted camera) hand-eye calibration.

package frames

import (
	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
)

// EyeInHandObservation is one eye-in-hand observation: a target point touched by
// the calibrated TCP, the flange pose at the moment the camera viewed it, and the
// deprojected click.
//
// Target and Flange MUST be expressed in the same frame (the configured
// destination_frame). The underlying equation is frame-agnostic, but only if both
// terms agree — mixing an arm-base flange pose (arm.EndPosition) with a
// destination-frame target silently miscalibrates on any machine whose arm base
// is not at the world origin.
type EyeInHandObservation struct {
	Target r3.Vector
	Flange spatialmath.Pose
	Camera r3.Vector
}

// ComputeCameraToFlange solves the camera->flange transform X from observations of
// known targets viewed from varying flange poses. Returns the pose, the RMS fit
// residual (mm), and an error for too-few or degenerate inputs.
//
// The camera is rigidly mounted to the flange, so X is constant and
// Target = Flange o X o Camera, hence X o Camera = Flange^-1 o Target. The
// right-hand side is computable, so this is an ordinary rigid point-cloud
// alignment between the camera points and each target expressed in its flange
// frame — the same Kabsch solve eye-to-hand uses, with no AX=XB machinery.
//
// Deprojection yields full 3D points rather than bearings, so pure-translation
// vantages recover the rotation too. Do NOT add a rotational-diversity check; it
// would reject valid data. See ComputeRigidTransform for the degeneracy guards
// that DO apply.
func ComputeCameraToFlange(obs []EyeInHandObservation) (spatialmath.Pose, float64, error) {
	pairs := make([]PointPair, len(obs))
	for i, o := range obs {
		q := spatialmath.Compose(
			spatialmath.PoseInverse(o.Flange),
			spatialmath.NewPoseFromPoint(o.Target),
		).Point()
		pairs[i] = PointPair{Reference: q, Camera: o.Camera}
	}
	return ComputeRigidTransform(pairs)
}
