package store

import (
	"sync"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"

	"github.com/viam-labs/teach-frames/frames"
)

func TestBufferCaptureAndClear(t *testing.T) {
	s := New()
	idx0 := s.AddCapture(spatialmath.NewPoseFromPoint(r3.Vector{X: 1}))
	test.That(t, idx0, test.ShouldEqual, 0)
	idx1 := s.AddCapture(spatialmath.NewPoseFromPoint(r3.Vector{X: 2}))
	test.That(t, idx1, test.ShouldEqual, 1)
	test.That(t, s.BufferLen(), test.ShouldEqual, 2)
	cleared := s.ClearBuffer()
	test.That(t, cleared, test.ShouldEqual, 2)
	test.That(t, s.BufferLen(), test.ShouldEqual, 0)
}

func TestSetAndListFrames(t *testing.T) {
	s := New()
	s.SetFrame("a", "world", spatialmath.NewPoseFromPoint(r3.Vector{X: 1}))
	s.SetFrame("b", "world", spatialmath.NewPoseFromPoint(r3.Vector{X: 2}))
	names := s.FrameNames()
	test.That(t, len(names), test.ShouldEqual, 2)
	_, ok := s.GetFrame("a")
	test.That(t, ok, test.ShouldBeTrue)
	test.That(t, s.DeleteFrame("a"), test.ShouldBeTrue)
	_, ok = s.GetFrame("a")
	test.That(t, ok, test.ShouldBeFalse)
	// Deleting a non-existent frame returns false.
	test.That(t, s.DeleteFrame("missing"), test.ShouldBeFalse)
}

// TestGetFrameRoundTrip verifies that the returned PoseInFrame carries the correct
// parent and pose values.
func TestGetFrameRoundTrip(t *testing.T) {
	s := New()
	pt := r3.Vector{X: 42, Y: 7, Z: 3}
	s.SetFrame("body", "world", spatialmath.NewPoseFromPoint(pt))
	f, ok := s.GetFrame("body")
	test.That(t, ok, test.ShouldBeTrue)
	test.That(t, f.Parent(), test.ShouldEqual, "world")
	test.That(t, f.Pose().Point().X, test.ShouldAlmostEqual, pt.X)
	test.That(t, f.Pose().Point().Y, test.ShouldAlmostEqual, pt.Y)
	test.That(t, f.Pose().Point().Z, test.ShouldAlmostEqual, pt.Z)
}

// TestGetFrameReturnsCopy verifies that mutating the returned PoseInFrame does
// not corrupt the store's internal copy.
func TestGetFrameReturnsCopy(t *testing.T) {
	s := New()
	s.SetFrame("body", "world", spatialmath.NewPoseFromPoint(r3.Vector{X: 1}))

	a, ok := s.GetFrame("body")
	test.That(t, ok, test.ShouldBeTrue)

	// Mutate the returned copy.
	a.SetParent("hacked")

	// The store must be unaffected.
	b, ok := s.GetFrame("body")
	test.That(t, ok, test.ShouldBeTrue)
	test.That(t, b.Parent(), test.ShouldEqual, "world")

	// Two successive GetFrame calls must return distinct pointers.
	c, _ := s.GetFrame("body")
	test.That(t, b != c, test.ShouldBeTrue)
}

// TestConcurrentAccess launches many goroutines doing mixed reads and writes
// concurrently. The test is meaningful only under -race; passing without a
// detected data race is the assertion.
func TestConcurrentAccess(t *testing.T) {
	s := New()
	s.SetFrame("seed", "world", spatialmath.NewPoseFromPoint(r3.Vector{}))

	const goroutines = 50
	var wg sync.WaitGroup
	wg.Add(goroutines)
	for i := 0; i < goroutines; i++ {
		i := i
		go func() {
			defer wg.Done()
			switch i % 4 {
			case 0:
				s.AddCapture(spatialmath.NewPoseFromPoint(r3.Vector{X: float64(i)}))
			case 1:
				s.SetFrame("body", "world", spatialmath.NewPoseFromPoint(r3.Vector{X: float64(i)}))
			case 2:
				s.Snapshot(nil)
			case 3:
				s.FrameNames()
			}
		}()
	}
	wg.Wait()
}

// TestSetFrameReplace verifies SetFrame returns false for new names and true when replacing.
func TestSetFrameReplace(t *testing.T) {
	s := New()
	replaced := s.SetFrame("x", "world", spatialmath.NewPoseFromPoint(r3.Vector{X: 1}))
	test.That(t, replaced, test.ShouldBeFalse)

	replaced = s.SetFrame("x", "world", spatialmath.NewPoseFromPoint(r3.Vector{X: 99}))
	test.That(t, replaced, test.ShouldBeTrue)
}

// TestSnapshot verifies that nil names returns all frames, a name filter works,
// and non-existent names are silently ignored.
func TestSnapshot(t *testing.T) {
	s := New()
	s.SetFrame("a", "world", spatialmath.NewPoseFromPoint(r3.Vector{X: 1}))
	s.SetFrame("b", "world", spatialmath.NewPoseFromPoint(r3.Vector{X: 2}))
	s.SetFrame("c", "world", spatialmath.NewPoseFromPoint(r3.Vector{X: 3}))

	// nil → all frames
	all := s.Snapshot(nil)
	test.That(t, len(all), test.ShouldEqual, 3)

	// filter to just "a"
	filtered := s.Snapshot([]string{"a"})
	test.That(t, len(filtered), test.ShouldEqual, 1)
	_, ok := filtered["a"]
	test.That(t, ok, test.ShouldBeTrue)

	// non-existent name is ignored
	partial := s.Snapshot([]string{"a", "nonexistent"})
	test.That(t, len(partial), test.ShouldEqual, 1)
}

// TestClearFrames verifies ClearFrames returns the count of removed frames.
func TestClearFrames(t *testing.T) {
	s := New()
	s.SetFrame("a", "world", spatialmath.NewPoseFromPoint(r3.Vector{X: 1}))
	s.SetFrame("b", "world", spatialmath.NewPoseFromPoint(r3.Vector{X: 2}))
	removed := s.ClearFrames()
	test.That(t, removed, test.ShouldEqual, 2)
	test.That(t, len(s.FrameNames()), test.ShouldEqual, 0)
}

// TestBufferReturnsCopy verifies that mutating the slice returned by Buffer()
// does NOT affect the store's internal buffer.
func TestBufferReturnsCopy(t *testing.T) {
	s := New()
	p1 := spatialmath.NewPoseFromPoint(r3.Vector{X: 1})
	p2 := spatialmath.NewPoseFromPoint(r3.Vector{X: 2})
	s.AddCapture(p1)
	s.AddCapture(p2)

	got := s.Buffer()
	test.That(t, len(got), test.ShouldEqual, 2)

	// Mutate the returned slice — set a nil element and truncate it.
	got[0] = nil
	got = got[:1]

	// The store must be unaffected.
	test.That(t, s.BufferLen(), test.ShouldEqual, 2)
	internal := s.Buffer()
	test.That(t, internal[0], test.ShouldNotBeNil)
}

func TestTCPBuffer(t *testing.T) {
	s := New()
	test.That(t, s.TCPBufferLen(), test.ShouldEqual, 0)

	p0 := spatialmath.NewZeroPose()
	idx := s.AddTCPCapture(p0)
	test.That(t, idx, test.ShouldEqual, 0)
	test.That(t, s.TCPBufferLen(), test.ShouldEqual, 1)

	s.AddTCPCapture(spatialmath.NewZeroPose())
	test.That(t, len(s.TCPBuffer()), test.ShouldEqual, 2)

	// TCP buffer is independent of the frame buffer.
	s.AddCapture(spatialmath.NewZeroPose())
	test.That(t, s.TCPBufferLen(), test.ShouldEqual, 2)
	test.That(t, s.BufferLen(), test.ShouldEqual, 1)

	test.That(t, s.ClearTCPBuffer(), test.ShouldEqual, 2)
	test.That(t, s.TCPBufferLen(), test.ShouldEqual, 0)
}

// TestTCPBufferReturnsCopy verifies that mutating the slice returned by
// TCPBuffer() does NOT affect the store's internal buffer.
func TestTCPBufferReturnsCopy(t *testing.T) {
	s := New()
	p1 := spatialmath.NewPoseFromPoint(r3.Vector{X: 1})
	p2 := spatialmath.NewPoseFromPoint(r3.Vector{X: 2})
	s.AddTCPCapture(p1)
	s.AddTCPCapture(p2)

	got := s.TCPBuffer()
	test.That(t, len(got), test.ShouldEqual, 2)

	// Mutate the returned slice — set a nil element and truncate it.
	got[0] = nil
	got = got[:1]

	// The store must be unaffected.
	test.That(t, s.TCPBufferLen(), test.ShouldEqual, 2)
	internal := s.TCPBuffer()
	test.That(t, internal[0], test.ShouldNotBeNil)
}

func TestHandEyeBuffer(t *testing.T) {
	s := New()
	test.That(t, s.HandEyeBufferLen(), test.ShouldEqual, 0)

	i0 := s.AddHandEyePair(frames.HandEyePair{World: r3.Vector{X: 1}, Camera: r3.Vector{Z: 2}})
	test.That(t, i0, test.ShouldEqual, 0)
	i1 := s.AddHandEyePair(frames.HandEyePair{World: r3.Vector{X: 3}, Camera: r3.Vector{Z: 4}})
	test.That(t, i1, test.ShouldEqual, 1)
	test.That(t, s.HandEyeBufferLen(), test.ShouldEqual, 2)

	// Hand-eye buffer is independent of the frame and TCP buffers.
	test.That(t, s.BufferLen(), test.ShouldEqual, 0)
	test.That(t, s.TCPBufferLen(), test.ShouldEqual, 0)

	buf := s.HandEyeBuffer()
	test.That(t, len(buf), test.ShouldEqual, 2)
	test.That(t, buf[0].World.X, test.ShouldEqual, 1)

	// Copy semantics: mutating the returned slice must not affect the store.
	buf[0].World.X = 999
	test.That(t, s.HandEyeBuffer()[0].World.X, test.ShouldEqual, 1)

	n := s.ClearHandEyeBuffer()
	test.That(t, n, test.ShouldEqual, 2)
	test.That(t, s.HandEyeBufferLen(), test.ShouldEqual, 0)
}
