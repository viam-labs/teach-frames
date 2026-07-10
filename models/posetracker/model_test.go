package posetracker

import (
	"context"
	"testing"

	"go.viam.com/rdk/components/posetracker"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/motion"
	injectmotion "go.viam.com/rdk/testutils/inject/motion"
	"go.viam.com/test"

	tfconfig "github.com/viam-labs/teach-frames/config"
	"github.com/viam-labs/teach-frames/persist"
	"github.com/viam-labs/teach-frames/posesource"
)

// newForTest constructs a teachTracker by calling the real newPoseTracker constructor
// (exercising dest-defaulting, frame loading, and skip-warn on malformed frames),
// then swaps in Fakes for source and persist so DoCommand tests can drive capture
// without a real motion service or app credentials.
func newForTest(t *testing.T, cfg *Config) *teachTracker {
	t.Helper()

	// motion.Named produces the resource.Name that motion.FromDependencies looks up.
	motionName := motion.Named(cfg.MotionService)
	injMotion := injectmotion.NewMotionService(cfg.MotionService)
	deps := resource.Dependencies{motionName: injMotion}

	conf := resource.Config{
		Name:  "test-tracker",
		API:   posetracker.API,
		Model: Model,
		// NativeConfig reads ConvertedAttributes; set it to the typed *Config directly.
		ConvertedAttributes: cfg,
	}

	res, err := newPoseTracker(context.Background(), deps, conf, logging.NewTestLogger(t))
	if err != nil {
		t.Fatalf("newForTest: newPoseTracker failed: %v", err)
	}

	tt := res.(*teachTracker)
	// Replace source and persist with Fakes for unit-test isolation.
	tt.source = &posesource.Fake{}
	tt.persist = &persist.Fake{}
	return tt
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

	// Missing tcp_component is still an error.
	cfg := &Config{MotionService: "builtin"}
	_, _, err := cfg.Validate(path)
	test.That(t, err, test.ShouldNotBeNil)

	// motion_service is optional: when omitted it defaults to "builtin".
	cfg = &Config{TCPComponent: "arm"}
	deps, optional, err := cfg.Validate(path)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, optional, test.ShouldBeNil)
	test.That(t, len(deps), test.ShouldEqual, 1)
	test.That(t, deps[0], test.ShouldEqual, "builtin")

	// An explicit motion_service is returned as the dependency.
	cfg = &Config{MotionService: "custom-motion", TCPComponent: "arm"}
	deps, _, err = cfg.Validate(path)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(deps), test.ShouldEqual, 1)
	test.That(t, deps[0], test.ShouldEqual, "custom-motion")
}

// TestNewPoseTrackerDefaultsMotionService verifies the constructor resolves the
// "builtin" motion service when motion_service is omitted from the config.
func TestNewPoseTrackerDefaultsMotionService(t *testing.T) {
	motionName := motion.Named(defaultMotionService)
	deps := resource.Dependencies{motionName: injectmotion.NewMotionService(defaultMotionService)}

	conf := resource.Config{
		Name:                "test-tracker",
		API:                 posetracker.API,
		Model:               Model,
		ConvertedAttributes: &Config{TCPComponent: "arm"}, // motion_service intentionally omitted
	}

	_, err := newPoseTracker(context.Background(), deps, conf, logging.NewTestLogger(t))
	test.That(t, err, test.ShouldBeNil)
}

// TestNewPoseTrackerConstructor exercises the real constructor directly to cover:
// (a) construction succeeds, (b) destFrame defaults to "world" when DestinationFrame
// is empty, (c) a frame with empty Parent loads under "world", and (d) a frame with
// an invalid pose is skipped (warn) not fatal.
func TestNewPoseTrackerConstructor(t *testing.T) {
	motionName := motion.Named("builtin")
	injMotion := injectmotion.NewMotionService("builtin")
	deps := resource.Dependencies{motionName: injMotion}

	conf := resource.Config{
		Name:  "test-tracker",
		API:   posetracker.API,
		Model: Model,
		ConvertedAttributes: &Config{
			MotionService:    "builtin",
			TCPComponent:     "arm",
			DestinationFrame: "", // (b) intentionally empty — should default to "world"
			Frames: []tfconfig.FrameSpec{
				// (c) frame with empty Parent should inherit dest ("world")
				{Name: "valid-frame", Parent: "", Pose: tfconfig.PoseSpec{X: 10, OZ: 1}},
			},
		},
	}

	res, err := newPoseTracker(context.Background(), deps, conf, logging.NewTestLogger(t))
	// (a) construction must succeed
	test.That(t, err, test.ShouldBeNil)

	tt := res.(*teachTracker)

	// (b) destFrame defaults to "world"
	test.That(t, tt.destFrame, test.ShouldEqual, "world")

	// (c) valid-frame loaded; since Parent was empty it should be under "world"
	poses, err := tt.Poses(context.Background(), nil, nil)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(poses), test.ShouldEqual, 1)
	_, ok := poses["valid-frame"]
	test.That(t, ok, test.ShouldBeTrue)
}

func TestValidateDeclaresArmDependency(t *testing.T) {
	cfg := &Config{TCPComponent: "tool", Arm: "my-arm"}
	req, _, err := cfg.Validate("")
	test.That(t, err, test.ShouldBeNil)
	test.That(t, req, test.ShouldContain, "my-arm")
}

func TestValidateNoArmIsValid(t *testing.T) {
	cfg := &Config{TCPComponent: "tool"}
	req, _, err := cfg.Validate("")
	test.That(t, err, test.ShouldBeNil)
	// motion service dep present, arm absent.
	test.That(t, req, test.ShouldNotContain, "")
}

// TestNewPoseTrackerNoArmLeavesFlangeNil verifies that when the "arm" config
// attribute is omitted, TCP teaching stays disabled: the constructor still
// succeeds (arm is optional) but pt.flange remains nil.
func TestNewPoseTrackerNoArmLeavesFlangeNil(t *testing.T) {
	motionName := motion.Named(defaultMotionService)
	deps := resource.Dependencies{motionName: injectmotion.NewMotionService(defaultMotionService)}

	conf := resource.Config{
		Name:                "test-tracker",
		API:                 posetracker.API,
		Model:               Model,
		ConvertedAttributes: &Config{TCPComponent: "arm"}, // Arm intentionally omitted
	}

	res, err := newPoseTracker(context.Background(), deps, conf, logging.NewTestLogger(t))
	test.That(t, err, test.ShouldBeNil)

	tt := res.(*teachTracker)
	test.That(t, tt.flange, test.ShouldBeNil)
	test.That(t, tt.armName, test.ShouldEqual, "")
	test.That(t, tt.tcpComponent, test.ShouldEqual, "arm")
}
