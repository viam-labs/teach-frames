// Package frames — hand-eye (eye-to-hand) calibration helpers.
package frames

import (
	"errors"
	"fmt"
	"image"
	"math"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/rimage"
	"go.viam.com/rdk/rimage/transform"
	"go.viam.com/rdk/spatialmath"
	"gonum.org/v1/gonum/mat"
)

// HandEyePair is one correspondence: a physical point's coordinate in the world
// frame (from the calibrated TCP) and in the camera frame (from deprojection).
type HandEyePair struct {
	World  r3.Vector
	Camera r3.Vector
}

// collinearSingularEps flags a near-zero second singular value (points span < 2 dims).
const collinearSingularEps = 1e-6

// minSpreadEps is the smallest accepted point spread, in mm. It only catches
// exactly-degenerate input (every point identical); near-degenerate input is
// caught by maxSpreadRatio instead.
const minSpreadEps = 1e-3

// maxSpreadRatio is how far the camera and reference clouds' spreads may
// disagree. A rigid transform preserves the centered spectrum, so for valid data
// the two spreads match; they diverge when one cloud collapses. For true spread S
// and camera noise σ the ratio runs about √(1+(σ/S)²), so this rejects data whose
// spread has fallen to roughly the noise floor — without needing to know σ.
const maxSpreadRatio = 1.25

// spread returns a centered point cloud's extent along its dominant axis, in mm.
// The largest singular value is √n × RMS spread, so dividing by √n makes this
// independent of the number of points.
func spread(singularValues []float64, n int) float64 {
	return singularValues[0] / math.Sqrt(float64(n))
}

// ComputeCameraToWorld solves the rigid transform T such that world ≈ T·camera
// (Kabsch/Umeyama, rotation only). Points are in mm. Returns the pose, the RMS
// fit residual (mm), and an error for too-few, collinear, or degenerate inputs.
// Coplanar (but non-collinear) inputs are accepted.
//
// Degeneracy is checked on BOTH clouds, not just the camera one. If the world
// points collapse to a single point — the operator never moved the arm between
// captures — then world-worldMean is zero for every pair, so H is exactly zero
// and the solve returns identity rotation no matter the truth. Camera noise
// keeps the camera cloud looking healthy, so a camera-side check cannot see it,
// and the residual stays small enough to read as a good calibration while the
// answer is hundreds of mm wrong.
func ComputeCameraToWorld(pairs []HandEyePair) (spatialmath.Pose, float64, error) {
	n := len(pairs)
	if n < 3 {
		return nil, 0, fmt.Errorf("camera calibration requires at least 3 points, have %d", n)
	}

	var camMean, worldMean r3.Vector
	for _, p := range pairs {
		camMean = camMean.Add(p.Camera)
		worldMean = worldMean.Add(p.World)
	}
	camMean = camMean.Mul(1.0 / float64(n))
	worldMean = worldMean.Mul(1.0 / float64(n))

	// H = Σ (cam-camMean)(world-worldMean)ᵀ, plus the centered camera and world
	// matrices (3×n) whose singular values reveal collinearity and degeneracy.
	h := mat.NewDense(3, 3, nil)
	cCentered := mat.NewDense(3, n, nil)
	wCentered := mat.NewDense(3, n, nil)
	for i, p := range pairs {
		c := p.Camera.Sub(camMean)
		w := p.World.Sub(worldMean)
		cCentered.Set(0, i, c.X)
		cCentered.Set(1, i, c.Y)
		cCentered.Set(2, i, c.Z)
		wCentered.Set(0, i, w.X)
		wCentered.Set(1, i, w.Y)
		wCentered.Set(2, i, w.Z)
		h.Set(0, 0, h.At(0, 0)+c.X*w.X)
		h.Set(0, 1, h.At(0, 1)+c.X*w.Y)
		h.Set(0, 2, h.At(0, 2)+c.X*w.Z)
		h.Set(1, 0, h.At(1, 0)+c.Y*w.X)
		h.Set(1, 1, h.At(1, 1)+c.Y*w.Y)
		h.Set(1, 2, h.At(1, 2)+c.Y*w.Z)
		h.Set(2, 0, h.At(2, 0)+c.Z*w.X)
		h.Set(2, 1, h.At(2, 1)+c.Z*w.Y)
		h.Set(2, 2, h.At(2, 2)+c.Z*w.Z)
	}

	var csvd, wsvd mat.SVD
	if !csvd.Factorize(cCentered, mat.SVDNone) {
		return nil, 0, errors.New("camera point SVD failed")
	}
	if !wsvd.Factorize(wCentered, mat.SVDNone) {
		return nil, 0, errors.New("world point SVD failed")
	}
	csv := csvd.Values(nil)
	wsv := wsvd.Values(nil)

	// Reject exactly-degenerate input: every point in a cloud identical. This
	// must run before the ratio check below, which would otherwise divide by zero.
	camSpread := spread(csv, n)
	worldSpread := spread(wsv, n)
	if camSpread < minSpreadEps {
		return nil, 0, errors.New("camera points are coincident; capture points from distinct positions")
	}
	if worldSpread < minSpreadEps {
		return nil, 0, errors.New("world points are coincident; move the arm between captures")
	}

	// Reject clouds whose spreads disagree: a rigid transform preserves the
	// centered spectrum, so this means one cloud has collapsed toward the noise
	// floor. The residual will NOT reveal this, so the error has to.
	if math.Max(camSpread, worldSpread)/math.Min(camSpread, worldSpread) > maxSpreadRatio {
		return nil, 0, errors.New("camera and world point spreads disagree; vary the arm pose between captures")
	}

	// Reject collinear inputs: the centered camera points must span ≥ 2 dims.
	if csv[1] < collinearSingularEps*csv[0] {
		return nil, 0, errors.New("camera points are collinear; capture points spanning at least a plane")
	}

	var svd mat.SVD
	if !svd.Factorize(h, mat.SVDFull) {
		return nil, 0, errors.New("hand-eye SVD failed")
	}
	var u, v mat.Dense
	svd.UTo(&u)
	svd.VTo(&v)

	// R = V · diag(1,1, det(V·Uᵀ)) · Uᵀ
	var vut mat.Dense
	vut.Mul(&v, u.T())
	d := mat.Det(&vut)
	dsign := 1.0
	if d < 0 {
		dsign = -1.0
	}
	diag := mat.NewDense(3, 3, []float64{1, 0, 0, 0, 1, 0, 0, 0, dsign})
	var vd, r mat.Dense
	vd.Mul(&v, diag)
	r.Mul(&vd, u.T())

	// spatialmath.Compose applies the TRANSPOSE of the matrix RotationMatrix.Col()/
	// Mul() reads back (see frames/compute.go), so R must be stored transposed
	// (columns of R laid out as rows here) for Compose to apply R itself.
	// TestComputeCameraToWorldRoundTrip is the regression guard.
	rm, err := spatialmath.NewRotationMatrix([]float64{
		r.At(0, 0), r.At(1, 0), r.At(2, 0),
		r.At(0, 1), r.At(1, 1), r.At(2, 1),
		r.At(0, 2), r.At(1, 2), r.At(2, 2),
	})
	if err != nil {
		return nil, 0, err
	}

	// t = worldMean − R·camMean
	rc := rotate(&r, camMean)
	t := worldMean.Sub(rc)

	pose := spatialmath.NewPose(t, rm)

	// Residual RMS.
	var sumSq float64
	for _, p := range pairs {
		pred := rotate(&r, p.Camera).Add(t)
		sumSq += pred.Sub(p.World).Norm2()
	}
	residual := math.Sqrt(sumSq / float64(n))

	if math.IsNaN(residual) || math.IsInf(residual, 0) {
		return nil, 0, errors.New("hand-eye solve produced a non-finite result")
	}
	return pose, residual, nil
}

// rotate applies a 3×3 rotation matrix to a vector.
func rotate(r *mat.Dense, v r3.Vector) r3.Vector {
	return r3.Vector{
		X: r.At(0, 0)*v.X + r.At(0, 1)*v.Y + r.At(0, 2)*v.Z,
		Y: r.At(1, 0)*v.X + r.At(1, 1)*v.Y + r.At(1, 2)*v.Z,
		Z: r.At(2, 0)*v.X + r.At(2, 1)*v.Y + r.At(2, 2)*v.Z,
	}
}

// Deproject converts pixel (u,v) with its depth-map depth into a 3D point in the
// camera frame (mm). Errors on out-of-bounds pixels or zero/absent depth.
func Deproject(intr *transform.PinholeCameraIntrinsics, dm *rimage.DepthMap, u, v int) (r3.Vector, error) {
	if intr == nil {
		return r3.Vector{}, errors.New("camera intrinsics unavailable; cannot deproject")
	}
	if dm == nil {
		return r3.Vector{}, errors.New("no depth frame available; snapshot first")
	}
	if !dm.Contains(u, v) {
		return r3.Vector{}, fmt.Errorf("pixel (%d,%d) out of depth bounds %dx%d", u, v, dm.Width(), dm.Height())
	}
	d := dm.GetDepth(u, v)
	if d == 0 {
		return r3.Vector{}, fmt.Errorf("no depth at pixel (%d,%d); aim at a surface with valid depth", u, v)
	}
	return intr.ImagePointTo3DPoint(image.Pt(u, v), d)
}
