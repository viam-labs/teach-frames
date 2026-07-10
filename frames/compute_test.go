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

	// X̂ must equal norm(p2-p1) = {0.6, 0.8, 0}.
	xHat := p2.Sub(p1).Normalize()
	test.That(t, spatialmath.R3VectorAlmostEqual(col0, xHat, 1e-6), test.ShouldBeTrue)

	// Ŷ must equal zHat.Cross(xHat) (right-handed), NOT xHat.Cross(zHat) (left-handed).
	// For this fixture zHat = norm(xHat x inPlane) where inPlane = p3-p1 = {0,0,1}.
	// xHat x {0,0,1} = {0*1-0*0, 0*0-0.6*1, 0.6*0-0*0} ... computed below symbolically.
	// We derive the expected yHat the same way compute.go does and check equality.
	inPlane := p3.Sub(p1)
	zHat := xHat.Cross(inPlane).Normalize()
	yHat := zHat.Cross(xHat)
	test.That(t, spatialmath.R3VectorAlmostEqual(col1, yHat, 1e-6), test.ShouldBeTrue)
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
