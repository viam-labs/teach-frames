// Package persist writes taught frames back to the module's component config.
package persist

import (
	"context"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"

	"github.com/viam-labs/teach-frames/config"
)

// ConfigPersister persists taught state durably.
type ConfigPersister interface {
	Save(ctx context.Context, frames []config.FrameSpec) error
	SaveComponentFrame(ctx context.Context, componentName, parent string, translation *r3.Vector, orientation spatialmath.Orientation) error
}

// Fake records the last saved set. For tests.
type Fake struct {
	Saved []config.FrameSpec
	Err   error

	SavedComponent   string
	SavedParent      string
	SavedTranslation *r3.Vector
	SavedOrientation spatialmath.Orientation
	FrameErr         error
}

// Save records frames (or returns the configured error without recording).
func (f *Fake) Save(_ context.Context, frames []config.FrameSpec) error {
	if f.Err != nil {
		return f.Err
	}
	f.Saved = append([]config.FrameSpec(nil), frames...)
	return nil
}

// SaveComponentFrame records the component-frame write (or returns FrameErr).
func (f *Fake) SaveComponentFrame(_ context.Context, componentName, parent string, translation *r3.Vector, orientation spatialmath.Orientation) error {
	if f.FrameErr != nil {
		return f.FrameErr
	}
	f.SavedComponent = componentName
	f.SavedParent = parent
	f.SavedTranslation = translation
	f.SavedOrientation = orientation
	return nil
}
