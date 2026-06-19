// Package store holds the in-memory taught-frame state and capture buffer.
package store

import (
	"sort"
	"sync"

	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

// FrameStore is the authoritative in-memory state, guarded by a mutex.
type FrameStore struct {
	mu     sync.RWMutex
	frames map[string]*referenceframe.PoseInFrame
	buffer []spatialmath.Pose
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
func (s *FrameStore) GetFrame(name string) (*referenceframe.PoseInFrame, bool) {
	s.mu.RLock()
	defer s.mu.RUnlock()
	f, ok := s.frames[name]
	return f, ok
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
func (s *FrameStore) Snapshot(names []string) referenceframe.FrameSystemPoses {
	s.mu.RLock()
	defer s.mu.RUnlock()
	out := referenceframe.FrameSystemPoses{}
	if len(names) == 0 {
		for n, f := range s.frames {
			out[n] = f
		}
		return out
	}
	for _, n := range names {
		if f, ok := s.frames[n]; ok {
			out[n] = f
		}
	}
	return out
}
