package frames

import (
	"math"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
)

func TestComputeThreePoint(t *testing.T) {
	// p1 origin, p2 on +X (1m), p3 on +Y (1m). Expect X axis -> world +X.
	p1 := r3.Vector{X: 0, Y: 0, Z: 0}
	p2 := r3.Vector{X: 1, Y: 0, Z: 0}
	p3 := r3.Vector{X: 0, Y: 1, Z: 0}

	pose, err := ComputeThreePoint(p1, p2, p3)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, spatialmath.R3VectorAlmostEqual(pose.Point(), p1, 1e-6), test.ShouldBeTrue)

	rm := pose.Orientation().RotationMatrix()
	xCol := rm.Col(0)
	test.That(t, math.Abs(xCol.X-1) < 1e-6, test.ShouldBeTrue)
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
