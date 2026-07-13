// Package store holds the in-memory taught-frame state and capture buffer.
package store

import (
	"sort"
	"sync"

	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"

	"github.com/viam-labs/teach-frames/frames"
)

// FrameStore is the authoritative in-memory state, guarded by a mutex.
type FrameStore struct {
	mu            sync.RWMutex
	frames        map[string]*referenceframe.PoseInFrame
	buffer        []spatialmath.Pose
	tcpBuffer     []spatialmath.Pose
	handeyeBuffer []frames.HandEyePair
}

// New returns an initialised, empty FrameStore.
func New() *FrameStore {
	return &FrameStore{frames: map[string]*referenceframe.PoseInFrame{}}
}

// AddCapture appends a pose to the capture buffer and returns the zero-based
// index at which it was stored.
func (s *FrameStore) AddCapture(p spatialmath.Pose) int {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.buffer = append(s.buffer, p)
	return len(s.buffer) - 1
}

// Buffer returns a shallow copy of the current capture buffer.
// Mutating the returned slice does not affect the store.
func (s *FrameStore) Buffer() []spatialmath.Pose {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return append([]spatialmath.Pose(nil), s.buffer...)
}

// BufferLen returns the number of poses currently in the capture buffer.
func (s *FrameStore) BufferLen() int {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return len(s.buffer)
}

// ClearBuffer discards all captured poses and returns the count that was removed.
func (s *FrameStore) ClearBuffer() int {
	s.mu.Lock()
	defer s.mu.Unlock()
	n := len(s.buffer)
	s.buffer = nil
	return n
}

// AddTCPCapture appends a flange pose to the TCP capture buffer and returns its
// zero-based index.
func (s *FrameStore) AddTCPCapture(p spatialmath.Pose) int {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.tcpBuffer = append(s.tcpBuffer, p)
	return len(s.tcpBuffer) - 1
}

// TCPBuffer returns a shallow copy of the current TCP capture buffer.
// Mutating the returned slice does not affect the store.
func (s *FrameStore) TCPBuffer() []spatialmath.Pose {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return append([]spatialmath.Pose(nil), s.tcpBuffer...)
}

// TCPBufferLen returns the number of poses in the TCP capture buffer.
func (s *FrameStore) TCPBufferLen() int {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return len(s.tcpBuffer)
}

// ClearTCPBuffer discards all buffered flange poses and returns the count removed.
func (s *FrameStore) ClearTCPBuffer() int {
	s.mu.Lock()
	defer s.mu.Unlock()
	n := len(s.tcpBuffer)
	s.tcpBuffer = nil
	return n
}

// AddHandEyePair appends a hand-eye correspondence and returns its zero-based index.
func (s *FrameStore) AddHandEyePair(p frames.HandEyePair) int {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.handeyeBuffer = append(s.handeyeBuffer, p)
	return len(s.handeyeBuffer) - 1
}

// HandEyeBuffer returns a copy of the current hand-eye buffer.
// Mutating the returned slice does not affect the store.
func (s *FrameStore) HandEyeBuffer() []frames.HandEyePair {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return append([]frames.HandEyePair(nil), s.handeyeBuffer...)
}

// HandEyeBufferLen returns the number of pairs in the hand-eye buffer.
func (s *FrameStore) HandEyeBufferLen() int {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return len(s.handeyeBuffer)
}

// ClearHandEyeBuffer discards all hand-eye pairs and returns the count removed.
func (s *FrameStore) ClearHandEyeBuffer() int {
	s.mu.Lock()
	defer s.mu.Unlock()
	n := len(s.handeyeBuffer)
	s.handeyeBuffer = nil
	return n
}

// SetFrame stores a taught frame under the given name.
// It returns true if a frame with that name already existed (replace),
// false if this is a new entry.
func (s *FrameStore) SetFrame(name, parent string, pose spatialmath.Pose) bool {
	s.mu.Lock()
	defer s.mu.Unlock()
	_, existed := s.frames[name]
	s.frames[name] = referenceframe.NewPoseInFrame(parent, pose)
	return existed
}

// GetFrame retrieves the PoseInFrame stored under name.
// The second return value is false when no such frame exists.
// A defensive copy is returned so callers cannot mutate stored state.
func (s *FrameStore) GetFrame(name string) (*referenceframe.PoseInFrame, bool) {
	s.mu.RLock()
	defer s.mu.RUnlock()
	f, ok := s.frames[name]
	if !ok {
		return nil, false
	}
	return copyPIF(f), true
}

// DeleteFrame removes the named frame and returns true if it was present.
func (s *FrameStore) DeleteFrame(name string) bool {
	s.mu.Lock()
	defer s.mu.Unlock()
	_, ok := s.frames[name]
	delete(s.frames, name)
	return ok
}

// ClearFrames removes all committed frames and returns the count removed.
func (s *FrameStore) ClearFrames() int {
	s.mu.Lock()
	defer s.mu.Unlock()
	n := len(s.frames)
	s.frames = map[string]*referenceframe.PoseInFrame{}
	return n
}

// FrameNames returns a sorted list of all committed frame names.
func (s *FrameStore) FrameNames() []string {
	s.mu.RLock()
	defer s.mu.RUnlock()
	out := make([]string, 0, len(s.frames))
	for n := range s.frames {
		out = append(out, n)
	}
	sort.Strings(out)
	return out
}

// Snapshot returns a FrameSystemPoses map for the requested names.
// When names is nil or empty, all committed frames are returned.
// Non-existent names are silently ignored.
// Defensive copies are returned so callers cannot mutate stored state.
func (s *FrameStore) Snapshot(names []string) referenceframe.FrameSystemPoses {
	s.mu.RLock()
	defer s.mu.RUnlock()
	out := referenceframe.FrameSystemPoses{}
	if len(names) == 0 {
		for n, f := range s.frames {
			out[n] = copyPIF(f)
		}
		return out
	}
	for _, n := range names {
		if f, ok := s.frames[n]; ok {
			out[n] = copyPIF(f)
		}
	}
	return out
}

// copyPIF returns a fresh PoseInFrame so callers cannot mutate stored state.
// Note: NewPoseInFrame sets only parent and pose; the internal name field (distinct
// from parent) is intentionally left empty because the store keyed by body name does
// not rely on the PIF's own name field.
func copyPIF(f *referenceframe.PoseInFrame) *referenceframe.PoseInFrame {
	return referenceframe.NewPoseInFrame(f.Parent(), f.Pose())
}
