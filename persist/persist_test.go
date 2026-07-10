package persist

import (
	"context"
	"errors"
	"testing"

	"github.com/viam-labs/teach-frames/config"
	"go.viam.com/test"
)

func TestFakePersisterRecordsFrames(t *testing.T) {
	f := &Fake{}
	frames := []config.FrameSpec{{Name: "a"}, {Name: "b"}}
	err := f.Save(context.Background(), frames)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(f.Saved), test.ShouldEqual, 2)
}

func TestFakePersisterReturnsConfiguredError(t *testing.T) {
	want := errors.New("boom")
	f := &Fake{Err: want}
	err := f.Save(context.Background(), []config.FrameSpec{{Name: "a"}})
	test.That(t, err, test.ShouldBeError, want)
	test.That(t, len(f.Saved), test.ShouldEqual, 0) // not recorded on error
}

func TestFakePersisterCopiesInput(t *testing.T) {
	f := &Fake{}
	frames := []config.FrameSpec{{Name: "a"}}
	_ = f.Save(context.Background(), frames)
	frames[0].Name = "mutated"
	test.That(t, f.Saved[0].Name, test.ShouldEqual, "a") // Fake stored a copy
}

func TestPersistersImplementInterface(t *testing.T) {
	var _ ConfigPersister = (*Fake)(nil)
	var _ ConfigPersister = (*AppPersister)(nil)
}
