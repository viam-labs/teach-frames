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
// the three points are collinear or coincident. Inputs are assumed finite;
// NaN or Inf coordinates are not validated and the caller is responsible for ensuring valid values.
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

	// The stored matrix must be laid out so that spatialmath.Compose maps the
	// frame's local +X/+Y/+Z onto xHat/yHat/zHat in the parent. Compose (and the
	// whole quaternion-based motion/visualization stack) applies the TRANSPOSE of
	// the matrix that RotationMatrix.Col()/Mul() read back, so the basis vectors
	// go in as ROWS here, not columns: with rows = [xHat; yHat; zHat], Compose
	// applies the transpose (columns xHat/yHat/zHat) and rotates local axes to the
	// intended world axes. TestComputeThreePointComposition is the regression guard.
	//   row 0: xHat.X, xHat.Y, xHat.Z
	//   row 1: yHat.X, yHat.Y, yHat.Z
	//   row 2: zHat.X, zHat.Y, zHat.Z
	rm, err := spatialmath.NewRotationMatrix([]float64{
		xHat.X, xHat.Y, xHat.Z,
		yHat.X, yHat.Y, yHat.Z,
		zHat.X, zHat.Y, zHat.Z,
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
