// Package frames computes spatial poses from captured arm positions.
package frames

import (
	"errors"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
)

const collinearEpsilon = 1e-9

// ComputeThreePoint builds a pose whose origin is p1, +X points toward p2, and
// p3 lies in the +XY half-plane (UR-style plane teaching). Returns an error if
// the three points are collinear or coincident.
func ComputeThreePoint(p1, p2, p3 r3.Vector) (spatialmath.Pose, error) {
	xAxis := p2.Sub(p1)
	if xAxis.Norm() < collinearEpsilon {
		return nil, errors.New("p1 and p2 are coincident; cannot define X axis")
	}
	xHat := xAxis.Normalize()

	inPlane := p3.Sub(p1)
	zAxis := xHat.Cross(inPlane)
	if zAxis.Norm() < collinearEpsilon {
		return nil, errors.New("points are collinear; cannot define a plane")
	}
	zHat := zAxis.Normalize()
	yHat := zHat.Cross(xHat) // right-handed

	// NewRotationMatrix stores values row-major: m[3*r + c].
	// Col(c) returns {mat[c], mat[3+c], mat[6+c]}, so to set Col(0)=xHat,
	// Col(1)=yHat, Col(2)=zHat we lay out rows as [xHat yHat zHat] transposed:
	//   row 0: xHat.X, yHat.X, zHat.X
	//   row 1: xHat.Y, yHat.Y, zHat.Y
	//   row 2: xHat.Z, yHat.Z, zHat.Z
	rm, err := spatialmath.NewRotationMatrix([]float64{
		xHat.X, yHat.X, zHat.X,
		xHat.Y, yHat.Y, zHat.Y,
		xHat.Z, yHat.Z, zHat.Z,
	})
	if err != nil {
		return nil, err
	}
	return spatialmath.NewPose(p1, rm), nil
}

// ComputePoint builds a pose at p with identity orientation.
func ComputePoint(p r3.Vector) spatialmath.Pose {
	return spatialmath.NewPoseFromPoint(p)
}
