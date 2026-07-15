// Package posetracker implements the teach-frames PoseTracker model.
package posetracker

import (
	"context"
	"fmt"
	"sync"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/components/posetracker"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"

	tfconfig "github.com/viam-labs/teach-frames/config"
	"github.com/viam-labs/teach-frames/persist"
	"github.com/viam-labs/teach-frames/posesource"
	"github.com/viam-labs/teach-frames/store"
)

// Model is the resource model triple for teach-frames pose tracker.
var Model = resource.NewModel("viam-labs", "teach-frames", "pose-tracker")

// defaultMotionService is the motion service name used when motion_service is omitted.
const defaultMotionService = "builtin"

// Camera mount modes. The mount is a physical fact that never changes at
// runtime, so it belongs in config rather than being inferred per command —
// which also lets the module refuse a flow that contradicts the hardware
// instead of silently solving garbage.
const (
	mountEyeToHand = "eye_to_hand"
	mountEyeInHand = "eye_in_hand"
)

func init() {
	resource.RegisterComponent(posetracker.API, Model,
		resource.Registration[posetracker.PoseTracker, *Config]{
			Constructor: newPoseTracker,
		},
	)
}

// Config holds the JSON-serializable configuration for the teach-frames PoseTracker.
type Config struct {
	MotionService    string               `json:"motion_service"`
	TCPComponent     string               `json:"tcp_component"`
	DestinationFrame string               `json:"destination_frame"`
	Arm              string               `json:"arm"`
	Camera           string               `json:"camera"`
	CameraMount      string               `json:"camera_mount"`
	Frames           []tfconfig.FrameSpec `json:"frames"`
}

// Validate ensures required fields are present and returns the motion service as a dependency.
// Returns (requiredDeps, optionalDeps, error).
func (c *Config) Validate(path string) ([]string, []string, error) {
	if c.TCPComponent == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "tcp_component")
	}
	deps := []string{c.motionServiceOrDefault()}
	if c.Arm != "" {
		deps = append(deps, c.Arm)
	}
	if c.Camera != "" {
		deps = append(deps, c.Camera)
	}

	switch c.cameraMountOrDefault() {
	case mountEyeToHand:
	case mountEyeInHand:
		// Eye-in-hand needs the arm name to ask the motion service for flange
		// poses, and the camera to attach the solved frame to.
		if c.Arm == "" {
			return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "arm")
		}
		if c.Camera == "" {
			return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "camera")
		}
	default:
		return nil, nil, resource.NewConfigValidationError(path, fmt.Errorf(
			"camera_mount must be %q or %q, got %q", mountEyeToHand, mountEyeInHand, c.CameraMount))
	}

	return deps, nil, nil
}

// motionServiceOrDefault returns the configured motion service name, defaulting
// to the builtin motion service when motion_service is omitted.
func (c *Config) motionServiceOrDefault() string {
	if c.MotionService == "" {
		return defaultMotionService
	}
	return c.MotionService
}

// cameraMountOrDefault returns the configured mount, defaulting to eye_to_hand so
// configs written before this attribute existed keep working unchanged.
func (c *Config) cameraMountOrDefault() string {
	if c.CameraMount == "" {
		return mountEyeToHand
	}
	return c.CameraMount
}

// teachTracker is the concrete PoseTracker implementation.
type teachTracker struct {
	resource.Named
	resource.AlwaysRebuild

	logger    logging.Logger
	store     *store.FrameStore
	source    posesource.PoseSource
	persist   persist.ConfigPersister
	destFrame string

	flange       posesource.FlangeSource
	arm          arm.Arm
	tcpComponent string
	armName      string

	cameraSrc   posesource.CameraSource
	cameraName  string
	cameraMount string

	// flangeInDest reads the arm flange pose in destFrame. It is deliberately NOT
	// posesource.FlangeSource: that wraps arm.EndPosition, which reports in the ARM
	// BASE frame, while the touched target arrives in destFrame. The solve is
	// frame-agnostic but only if both terms agree, so mixing them would pass every
	// test on a machine whose arm sits at the world origin and silently
	// miscalibrate everywhere else. nil unless mount is eye_in_hand AND the arm
	// resolved.
	flangeInDest posesource.PoseSource

	// targetMu guards currentTarget.
	targetMu      sync.Mutex
	currentTarget *r3.Vector

	// snapshotMu guards lastSnapshot; the cached depth+intrinsics MUST be the
	// exact frame the operator clicked on, so capture deprojects against this,
	// not a fresh acquisition.
	snapshotMu   sync.Mutex
	lastSnapshot *posesource.Snapshot
	// lastFlange is the flange pose read at snapshot time, cached under snapshotMu
	// with lastSnapshot. The operator jogs, snapshots, THEN clicks, so a flange read
	// at click time could describe a pose the pixels never came from.
	lastFlange spatialmath.Pose

	// commitMu serializes all config-persisting mutations (define/delete/clear) so
	// the get→set→persist→rollback sequence remains atomic w.r.t. concurrent commits
	// and the persisted frame set stays consistent. This is intentionally separate from
	// FrameStore's internal RWMutex, which guards individual store ops.
	commitMu sync.Mutex
}

// newPoseTracker is the resource constructor registered with the RDK.
func newPoseTracker(
	ctx context.Context,
	deps resource.Dependencies,
	conf resource.Config,
	logger logging.Logger,
) (posetracker.PoseTracker, error) {
	cfg, err := resource.NativeConfig[*Config](conf)
	if err != nil {
		return nil, err
	}

	mot, err := motion.FromDependencies(deps, cfg.motionServiceOrDefault())
	if err != nil {
		return nil, err
	}

	dest := cfg.DestinationFrame
	if dest == "" {
		dest = referenceframe.World
	}

	fs := store.New()
	pt := &teachTracker{
		Named:     conf.ResourceName().AsNamed(),
		logger:    logger,
		store:     fs,
		source:    &posesource.MotionSource{Motion: mot, Component: cfg.TCPComponent, DestFrame: dest},
		destFrame: dest,
	}

	pt.tcpComponent = cfg.TCPComponent
	pt.cameraMount = cfg.cameraMountOrDefault()
	pt.armName = cfg.Arm
	if cfg.Arm != "" {
		a, aerr := arm.FromDependencies(deps, cfg.Arm)
		if aerr != nil {
			// Declared dependency should resolve; warn and leave TCP teaching disabled.
			logger.Warnw("TCP teaching disabled: arm dependency not resolvable", "arm", cfg.Arm, "err", aerr)
		} else {
			pt.flange = &posesource.ArmSource{Arm: a}
			pt.arm = a

			// Gated on the arm actually resolving, not just on cfg.Arm being set:
			// MotionSource holds only a name string, so it would be non-nil even for
			// an arm that does not exist, turning a clear "arm not configured" error
			// into an opaque GetPose failure at snapshot time.
			if pt.cameraMount == mountEyeInHand {
				pt.flangeInDest = &posesource.MotionSource{Motion: mot, Component: cfg.Arm, DestFrame: dest}
			}
		}
	}

	pt.cameraName = cfg.Camera
	if cfg.Camera != "" {
		cam, cerr := camera.FromDependencies(deps, cfg.Camera)
		if cerr != nil {
			// Declared dependency should resolve; warn and leave hand-eye calibration disabled.
			logger.Warnw("hand-eye calibration disabled: camera dependency not resolvable", "camera", cfg.Camera, "err", cerr)
		} else {
			pt.cameraSrc = &posesource.RGBDCamera{Cam: cam}
		}
	}

	// Persistence: warn and continue if creds are absent; do not fail construction.
	p, perr := persist.NewAppPersister(conf.ResourceName().Name, logger)
	if perr != nil {
		logger.Warnw("frame persistence disabled", "component", conf.ResourceName().Name, "reason", perr)
	} else {
		pt.persist = p
	}

	// Load committed frames from config.
	// One malformed frame should not brick the component or the live capture path,
	// matching the warn-and-continue stance used for persistence above.
	for _, frameSpec := range cfg.Frames {
		pose, ferr := frameSpec.ToPose()
		if ferr != nil {
			logger.Warnw("skipping invalid taught frame", "name", frameSpec.Name, "err", ferr)
			continue
		}
		parent := frameSpec.Parent
		if parent == "" {
			parent = dest
		}
		pt.store.SetFrame(frameSpec.Name, parent, pose)
	}

	return pt, nil
}

// Poses returns the stored frame poses, filtered by bodyNames when non-empty.
func (pt *teachTracker) Poses(_ context.Context, bodyNames []string, _ map[string]interface{}) (referenceframe.FrameSystemPoses, error) {
	return pt.store.Snapshot(bodyNames), nil
}

// Close is a no-op for this resource (no goroutines or external connections to clean up here).
func (pt *teachTracker) Close(context.Context) error {
	return nil
}
