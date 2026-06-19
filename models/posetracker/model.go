// Package posetracker implements the teach-frames PoseTracker model.
package posetracker

import (
	"context"

	"go.viam.com/rdk/components/posetracker"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/motion"

	tfconfig "github.com/viam-labs/teach-frames/config"
	"github.com/viam-labs/teach-frames/persist"
	"github.com/viam-labs/teach-frames/posesource"
	"github.com/viam-labs/teach-frames/store"
)

// Model is the resource model triple for teach-frames pose tracker.
var Model = resource.NewModel("viam-labs", "teach-frames", "pose-tracker")

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
	Frames           []tfconfig.FrameSpec `json:"frames"`
}

// Validate ensures required fields are present and returns the motion service as a dependency.
// Returns (requiredDeps, optionalDeps, error).
func (c *Config) Validate(path string) ([]string, []string, error) {
	if c.MotionService == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "motion_service")
	}
	if c.TCPComponent == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "tcp_component")
	}
	return []string{c.MotionService}, nil, nil
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

	mot, err := motion.FromDependencies(deps, cfg.MotionService)
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

	// Persistence: warn and continue if creds are absent; do not fail construction.
	p, perr := persist.NewAppPersister(conf.ResourceName().Name)
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
