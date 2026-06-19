// Package persist writes taught frames back to the module's component config.
package persist

import (
	"context"

	"github.com/viam-labs/teach-frames/config"
)

// ConfigPersister persists the full set of taught frames durably.
type ConfigPersister interface {
	Save(ctx context.Context, frames []config.FrameSpec) error
}

// Fake records the last saved set. For tests.
type Fake struct {
	Saved []config.FrameSpec
	Err   error
}

// Save records frames (or returns the configured error without recording).
func (f *Fake) Save(_ context.Context, frames []config.FrameSpec) error {
	if f.Err != nil {
		return f.Err
	}
	f.Saved = append([]config.FrameSpec(nil), frames...)
	return nil
}
