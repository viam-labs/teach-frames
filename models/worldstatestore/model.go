package worldstatestore

import (
	"bytes"
	"context"
	"fmt"
	"time"

	commonpb "go.viam.com/api/common/v1"
	"go.viam.com/rdk/components/posetracker"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/worldstatestore"
)

// defaultPollInterval is how often StreamTransformChanges polls the pose-tracker for
// deltas when no other interval has been configured.
const defaultPollInterval = time.Second

// Model is the resource model triple for the teach-frames world_state_store mirror.
var Model = resource.NewModel("viam-labs", "teach-frames", "world-state-store")

func init() {
	resource.RegisterService(worldstatestore.API, Model,
		resource.Registration[worldstatestore.Service, *Config]{Constructor: newWSS})
}

// Config holds the JSON-serializable configuration for the WSS mirror.
type Config struct {
	PoseTracker string `json:"pose_tracker"`
}

// Validate ensures required fields are present and returns the pose tracker as a dependency.
// Returns (requiredDeps, optionalDeps, error).
func (c *Config) Validate(path string) ([]string, []string, error) {
	if c.PoseTracker == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "pose_tracker")
	}
	return []string{c.PoseTracker}, nil, nil
}

// mirror is the concrete worldstatestore.Service implementation.
type mirror struct {
	resource.Named
	resource.AlwaysRebuild

	logger       logging.Logger
	pt           posetracker.PoseTracker
	pollInterval time.Duration
}

func newWSS(
	ctx context.Context,
	deps resource.Dependencies,
	conf resource.Config,
	logger logging.Logger,
) (worldstatestore.Service, error) {
	cfg, err := resource.NativeConfig[*Config](conf)
	if err != nil {
		return nil, err
	}
	pt, err := posetracker.FromDependencies(deps, cfg.PoseTracker)
	if err != nil {
		return nil, err
	}
	return &mirror{
		Named:        conf.ResourceName().AsNamed(),
		logger:       logger,
		pt:           pt,
		pollInterval: defaultPollInterval,
	}, nil
}

// ListUUIDs returns the stable UUID for each frame currently tracked by the PoseTracker.
func (m *mirror) ListUUIDs(ctx context.Context, _ map[string]any) ([][]byte, error) {
	poses, err := m.pt.Poses(ctx, nil, nil)
	if err != nil {
		return nil, err
	}
	out := make([][]byte, 0, len(poses))
	for name := range poses {
		out = append(out, uuidForName(name))
	}
	return out, nil
}

// GetTransform returns the Transform for the frame identified by the given UUID.
func (m *mirror) GetTransform(ctx context.Context, id []byte, _ map[string]any) (*commonpb.Transform, error) {
	poses, err := m.pt.Poses(ctx, nil, nil)
	if err != nil {
		return nil, err
	}
	for name, pif := range poses {
		if bytes.Equal(uuidForName(name), id) {
			return frameToTransform(name, pif), nil
		}
	}
	return nil, fmt.Errorf("no transform for uuid %x", id)
}

// StreamTransformChanges returns a stream of live ADDED/REMOVED/UPDATED deltas for the
// PoseTracker's taught frames. A single background goroutine seeds an initial snapshot
// (without emitting — the client already has the initial set via ListUUIDs/GetTransform),
// then polls the PoseTracker on m.pollInterval, diffs each snapshot against the last one
// seen, and emits the resulting changes. The goroutine exits and closes the channel when
// ctx is done.
func (m *mirror) StreamTransformChanges(ctx context.Context, _ map[string]any) (*worldstatestore.TransformChangeStream, error) {
	ch := make(chan worldstatestore.TransformChange)

	go func() {
		defer close(ch)

		prev, err := m.pt.Poses(ctx, nil, nil)
		if err != nil {
			m.logger.CDebugw(ctx, "failed to seed initial poses for transform change stream", "error", err)
			prev = nil
		}

		ticker := time.NewTicker(m.pollInterval)
		defer ticker.Stop()

		for {
			select {
			case <-ctx.Done():
				return
			case <-ticker.C:
				cur, err := m.pt.Poses(ctx, nil, nil)
				if err != nil {
					m.logger.CDebugw(ctx, "failed to poll poses for transform change stream", "error", err)
					continue
				}
				for _, change := range diffTransforms(prev, cur) {
					select {
					case ch <- change:
					case <-ctx.Done():
						return
					}
				}
				prev = cur
			}
		}
	}()

	return worldstatestore.NewTransformChangeStreamFromChannel(ctx, ch), nil
}

// Close is a no-op.
func (m *mirror) Close(_ context.Context) error { return nil }
