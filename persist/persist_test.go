package persist

import (
	"context"
	"errors"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"

	"github.com/viam-labs/teach-frames/config"
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

func TestFakePersisterRecordsComponentFrame(t *testing.T) {
	f := &Fake{}
	tr := &r3.Vector{X: 1, Y: 2, Z: 3}
	ov := &spatialmath.OrientationVectorDegrees{OZ: 1, Theta: 90}

	err := f.SaveComponentFrame(context.Background(), "tool", "my-arm", tr, ov)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, f.SavedComponent, test.ShouldEqual, "tool")
	test.That(t, f.SavedParent, test.ShouldEqual, "my-arm")
	test.That(t, f.SavedTranslation, test.ShouldEqual, tr)
	test.That(t, f.SavedOrientation, test.ShouldEqual, ov)
}

func TestFakePersisterSaveComponentFrameReturnsConfiguredError(t *testing.T) {
	want := errors.New("boom")
	f := &Fake{FrameErr: want}
	err := f.SaveComponentFrame(context.Background(), "tool", "my-arm", &r3.Vector{}, nil)
	test.That(t, err, test.ShouldBeError, want)
	test.That(t, f.SavedComponent, test.ShouldEqual, "") // not recorded on error
}
