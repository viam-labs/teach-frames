package frames

import (
	"errors"
	"fmt"
	"math"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
	"gonum.org/v1/gonum/mat"
)

// minPivotPoints is the smallest number of flange captures accepted by the
// 4-point pivot method. Four gives redundancy over the 6 unknowns and a
// meaningful RMS residual.
const minPivotPoints = 4

// ComputePivotTCP solves for a tool tip offset from a set of arm flange poses.
// Each pose places the (fixed) tip at one common point in the arm base frame, so
// it solves the least-squares system [R_i | -I]·[t; c] = -p_i, where t is the
// tip offset in the flange frame and c is the common touched point. It returns
// the tip offset and the RMS residual (mm) of how consistently the captures hit
// one point. Returns an error for fewer than minPivotPoints captures or a
// rank-deficient system (arm orientations too similar).
func ComputePivotTCP(poses []spatialmath.Pose) (r3.Vector, float64, error) {
	if len(poses) < minPivotPoints {
		return r3.Vector{}, 0, fmt.Errorf("pivot requires at least %d captures, have %d", minPivotPoints, len(poses))
	}

	n := len(poses)
	a := mat.NewDense(3*n, 6, nil)
	b := mat.NewVecDense(3*n, nil)

	for i, pose := range poses {
		c0, c1, c2 := rotationColumns(pose)
		// Rows 3i..3i+2: [ R_i | -I ] and rhs -p_i.
		// c0, c1, c2 are R_i's columns, so (R_i*t) row r = c0[r]*t.X + c1[r]*t.Y + c2[r]*t.Z.
		a.Set(3*i+0, 0, c0.X)
		a.Set(3*i+0, 1, c1.X)
		a.Set(3*i+0, 2, c2.X)
		a.Set(3*i+1, 0, c0.Y)
		a.Set(3*i+1, 1, c1.Y)
		a.Set(3*i+1, 2, c2.Y)
		a.Set(3*i+2, 0, c0.Z)
		a.Set(3*i+2, 1, c1.Z)
		a.Set(3*i+2, 2, c2.Z)
		a.Set(3*i+0, 3, -1)
		a.Set(3*i+1, 4, -1)
		a.Set(3*i+2, 5, -1)

		p := pose.Point()
		b.SetVec(3*i+0, -p.X)
		b.SetVec(3*i+1, -p.Y)
		b.SetVec(3*i+2, -p.Z)
	}

	var x mat.Dense
	if err := x.Solve(a, b); err != nil {
		// mat.Condition (ill-conditioned) surfaces here when orientations are
		// near-parallel and the system is effectively rank-deficient.
		return r3.Vector{}, 0, fmt.Errorf("pivot solve failed (vary arm orientations more): %w", err)
	}

	tip := r3.Vector{X: x.At(0, 0), Y: x.At(1, 0), Z: x.At(2, 0)}
	common := r3.Vector{X: x.At(3, 0), Y: x.At(4, 0), Z: x.At(5, 0)}

	// RMS residual of ||R_i*tip + p_i - common||.
	var sumSq float64
	for _, pose := range poses {
		hit := spatialmath.Compose(pose, spatialmath.NewPoseFromPoint(tip)).Point()
		d := hit.Sub(common)
		sumSq += d.Norm2()
	}
	residual := math.Sqrt(sumSq / float64(n))

	if math.IsNaN(residual) || math.IsInf(residual, 0) {
		return r3.Vector{}, 0, errors.New("pivot produced a non-finite result")
	}
	return tip, residual, nil
}

// rotationColumns returns the columns of pose's rotation matrix, i.e. the
// images of the base-frame unit X/Y/Z vectors under the pose's orientation.
// These are derived via spatialmath.Compose against a zero-translation copy
// of the pose rather than by reading RotationMatrix()'s Col() accessor
// directly: Col() indexes the matrix's raw row-major storage, which does not
// correspond to how Compose actually rotates a point, so hand-multiplying
// with it silently applies the inverse rotation. Compose is the ground truth
// for how the SDK composes real captured poses, so we build against it.
func rotationColumns(pose spatialmath.Pose) (c0, c1, c2 r3.Vector) {
	rotationOnly := spatialmath.NewPoseFromOrientation(pose.Orientation())
	c0 = spatialmath.Compose(rotationOnly, spatialmath.NewPoseFromPoint(r3.Vector{X: 1})).Point()
	c1 = spatialmath.Compose(rotationOnly, spatialmath.NewPoseFromPoint(r3.Vector{Y: 1})).Point()
	c2 = spatialmath.Compose(rotationOnly, spatialmath.NewPoseFromPoint(r3.Vector{Z: 1})).Point()
	return c0, c1, c2
}
