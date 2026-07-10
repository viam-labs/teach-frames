package frames

import (
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
)

func TestComputeThreePoint(t *testing.T) {
	// p1 origin, p2 on +X (1m), p3 on +Y (1m). Expect identity rotation matrix.
	p1 := r3.Vector{X: 0, Y: 0, Z: 0}
	p2 := r3.Vector{X: 1, Y: 0, Z: 0}
	p3 := r3.Vector{X: 0, Y: 1, Z: 0}

	pose, err := ComputeThreePoint(p1, p2, p3)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, spatialmath.R3VectorAlmostEqual(pose.Point(), p1, 1e-6), test.ShouldBeTrue)

	rm := pose.Orientation().RotationMatrix()
	test.That(t, spatialmath.R3VectorAlmostEqual(rm.Col(0), r3.Vector{X: 1}, 1e-6), test.ShouldBeTrue)
	test.That(t, spatialmath.R3VectorAlmostEqual(rm.Col(1), r3.Vector{Y: 1}, 1e-6), test.ShouldBeTrue)
	test.That(t, spatialmath.R3VectorAlmostEqual(rm.Col(2), r3.Vector{Z: 1}, 1e-6), test.ShouldBeTrue)
}

func TestComputeThreePointGeneralOrthonormal(t *testing.T) {
	// Non-axis-aligned fixture: p2 is off-axis, p3 is out of plane.
	// p2-p1 = {3,4,0} (not aligned to any world axis), p3 forces zHat into Z hemisphere.
	// This exercises the full cross-product path and ensures the correct handedness
	// (zHat.Cross(xHat)) is used for yHat rather than the wrong xHat.Cross(zHat).
	p1 := r3.Vector{X: 0, Y: 0, Z: 0}
	p2 := r3.Vector{X: 3, Y: 4, Z: 0}
	p3 := r3.Vector{X: 0, Y: 0, Z: 1}

	pose, err := ComputeThreePoint(p1, p2, p3)
	test.That(t, err, test.ShouldBeNil)

	rm := pose.Orientation().RotationMatrix()
	col0 := rm.Col(0)
	col1 := rm.Col(1)
	col2 := rm.Col(2)

	// Each column must have unit norm.
	test.That(t, spatialmath.R3VectorAlmostEqual(r3.Vector{X: col0.Norm()}, r3.Vector{X: 1}, 1e-6), test.ShouldBeTrue)
	test.That(t, spatialmath.R3VectorAlmostEqual(r3.Vector{X: col1.Norm()}, r3.Vector{X: 1}, 1e-6), test.ShouldBeTrue)
	test.That(t, spatialmath.R3VectorAlmostEqual(r3.Vector{X: col2.Norm()}, r3.Vector{X: 1}, 1e-6), test.ShouldBeTrue)

	// Columns must be pairwise orthogonal.
	test.That(t, spatialmath.R3VectorAlmostEqual(r3.Vector{X: col0.Dot(col1)}, r3.Vector{}, 1e-6), test.ShouldBeTrue)
	test.That(t, spatialmath.R3VectorAlmostEqual(r3.Vector{X: col0.Dot(col2)}, r3.Vector{}, 1e-6), test.ShouldBeTrue)
	test.That(t, spatialmath.R3VectorAlmostEqual(r3.Vector{X: col1.Dot(col2)}, r3.Vector{}, 1e-6), test.ShouldBeTrue)

	// Right-handed: Col(0) x Col(1) must equal Col(2).
	test.That(t, spatialmath.R3VectorAlmostEqual(col0.Cross(col1), col2, 1e-6), test.ShouldBeTrue)

	// NOTE: this test intentionally asserts only orthonormality/handedness — all
	// convention-independent properties of the stored matrix. It does NOT assert
	// which physical direction each column points, because Col() reads the raw
	// row-major storage, which is the TRANSPOSE of how the pose actually composes.
	// Directional correctness (local axes -> intended world axes) is verified via
	// spatialmath.Compose in TestComputeThreePointComposition.
}

// TestComputeThreePointComposition is the acid test the suite previously lacked:
// it composes points expressed in the taught frame's LOCAL coordinates into the
// parent frame via spatialmath.Compose — exactly what the frame system, motion
// planner, and world_state_store visualizer do — and asserts the taught frame's
// local axes land on the physically intended xHat/yHat/zHat. Asserting on
// rm.Col(c) (as the other tests do) checks the matrix that was built, not how it
// actually composes, and hid a transposed-orientation bug: Compose applies the
// transpose of the Col()/Mul() convention.
func TestComputeThreePointComposition(t *testing.T) {
	// Non-axis-aligned frame: origin p1, +X toward p2, p3 fixes the +XY plane.
	p1 := r3.Vector{X: 0, Y: 0, Z: 0}
	p2 := r3.Vector{X: 3, Y: 4, Z: 0}
	p3 := r3.Vector{X: 0, Y: 0, Z: 1}

	pose, err := ComputeThreePoint(p1, p2, p3)
	test.That(t, err, test.ShouldBeNil)

	// Reconstruct the intended basis the same way compute.go does.
	xHat := p2.Sub(p1).Normalize()
	zHat := xHat.Cross(p3.Sub(p1)).Normalize()
	yHat := zHat.Cross(xHat)

	// Compose local unit axes into the parent frame and compare to the intended basis.
	composeLocal := func(local r3.Vector) r3.Vector {
		return spatialmath.Compose(pose, spatialmath.NewPoseFromPoint(local)).Point().Sub(p1)
	}
	test.That(t, spatialmath.R3VectorAlmostEqual(composeLocal(r3.Vector{X: 1}), xHat, 1e-9), test.ShouldBeTrue)
	test.That(t, spatialmath.R3VectorAlmostEqual(composeLocal(r3.Vector{Y: 1}), yHat, 1e-9), test.ShouldBeTrue)
	test.That(t, spatialmath.R3VectorAlmostEqual(composeLocal(r3.Vector{Z: 1}), zHat, 1e-9), test.ShouldBeTrue)
}

func TestComputeThreePointRejectsCollinear(t *testing.T) {
	p1 := r3.Vector{X: 0, Y: 0, Z: 0}
	p2 := r3.Vector{X: 1, Y: 0, Z: 0}
	p3 := r3.Vector{X: 2, Y: 0, Z: 0} // collinear
	_, err := ComputeThreePoint(p1, p2, p3)
	test.That(t, err, test.ShouldNotBeNil)
}

func TestComputeThreePointRejectsCoincident(t *testing.T) {
	// p1 == p2: cannot define X axis
	p1 := r3.Vector{X: 1, Y: 2, Z: 3}
	p2 := r3.Vector{X: 1, Y: 2, Z: 3}
	p3 := r3.Vector{X: 0, Y: 1, Z: 0}
	_, err := ComputeThreePoint(p1, p2, p3)
	test.That(t, err, test.ShouldNotBeNil)
}

func TestComputePoint(t *testing.T) {
	p := r3.Vector{X: 1, Y: 2, Z: 3}
	pose := ComputePoint(p)

	// Position must match
	test.That(t, spatialmath.R3VectorAlmostEqual(pose.Point(), p, 1e-6), test.ShouldBeTrue)

	// Orientation must be identity: Col(0)=(1,0,0), Col(1)=(0,1,0), Col(2)=(0,0,1)
	rm := pose.Orientation().RotationMatrix()
	test.That(t, spatialmath.R3VectorAlmostEqual(rm.Col(0), r3.Vector{X: 1, Y: 0, Z: 0}, 1e-6), test.ShouldBeTrue)
	test.That(t, spatialmath.R3VectorAlmostEqual(rm.Col(1), r3.Vector{X: 0, Y: 1, Z: 0}, 1e-6), test.ShouldBeTrue)
	test.That(t, spatialmath.R3VectorAlmostEqual(rm.Col(2), r3.Vector{X: 0, Y: 0, Z: 1}, 1e-6), test.ShouldBeTrue)
}
