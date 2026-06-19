package posetracker

import (
	"context"
	"testing"

	"go.viam.com/rdk/logging"
	"go.viam.com/test"

	tfconfig "github.com/viam-labs/teach-frames/config"
	"github.com/viam-labs/teach-frames/persist"
	"github.com/viam-labs/teach-frames/posesource"
	"github.com/viam-labs/teach-frames/store"
)

// newForTest constructs a teachTracker directly for unit tests, bypassing the
// RDK dependency injection. It uses a Fake pose source and Fake persister so
// tests have no external dependencies.
func newForTest(t *testing.T, cfg *Config) *teachTracker {
	t.Helper()

	fs := store.New()

	dest := cfg.DestinationFrame
	if dest == "" {
		dest = "world"
	}

	pt := &teachTracker{
		logger:    logging.NewTestLogger(t),
		store:     fs,
		source:    &posesource.Fake{},
		persist:   &persist.Fake{},
		destFrame: dest,
	}

	// Load committed frames from config, mirroring the constructor logic.
	for _, frameSpec := range cfg.Frames {
		pose, err := frameSpec.ToPose()
		if err != nil {
			t.Fatalf("newForTest: ToPose failed for frame %q: %v", frameSpec.Name, err)
		}
		parent := frameSpec.Parent
		if parent == "" {
			parent = dest
		}
		fs.SetFrame(frameSpec.Name, parent, pose)
	}

	return pt
}

func TestPosesLoadsConfiguredFrames(t *testing.T) {
	cfg := &Config{
		MotionService:    "builtin",
		TCPComponent:     "arm",
		DestinationFrame: "world",
		Frames: []tfconfig.FrameSpec{
			{Name: "a", Parent: "world", Pose: tfconfig.PoseSpec{X: 1, OZ: 1}},
		},
	}
	pt := newForTest(t, cfg)

	// All frames returned when bodyNames is nil.
	got, err := pt.Poses(context.Background(), nil, nil)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(got), test.ShouldEqual, 1)
	_, ok := got["a"]
	test.That(t, ok, test.ShouldBeTrue)

	// Filter by unknown name returns empty map.
	got, err = pt.Poses(context.Background(), []string{"zzz"}, nil)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(got), test.ShouldEqual, 0)
}

func TestValidate(t *testing.T) {
	const path = "components.0"

	// Missing motion_service.
	cfg := &Config{TCPComponent: "arm"}
	_, _, err := cfg.Validate(path)
	test.That(t, err, test.ShouldNotBeNil)

	// Missing tcp_component.
	cfg = &Config{MotionService: "builtin"}
	_, _, err = cfg.Validate(path)
	test.That(t, err, test.ShouldNotBeNil)

	// Valid config returns motion_service as required dep.
	cfg = &Config{MotionService: "builtin", TCPComponent: "arm"}
	deps, optional, err := cfg.Validate(path)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, optional, test.ShouldBeNil)
	test.That(t, len(deps), test.ShouldEqual, 1)
	test.That(t, deps[0], test.ShouldEqual, "builtin")
}
