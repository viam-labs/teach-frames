package worldstatestore

import (
	"bytes"
	"context"
	"fmt"

	commonpb "go.viam.com/api/common/v1"
	"go.viam.com/rdk/components/posetracker"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/worldstatestore"
)

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

	logger logging.Logger
	pt     posetracker.PoseTracker
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
	return &mirror{Named: conf.ResourceName().AsNamed(), logger: logger, pt: pt}, nil
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

// StreamTransformChanges returns a stream that stays open until the context ends.
// Option-1 (rebuild-and-reseed): this service carries no live deltas. When a frame
// change is committed, the PoseTracker (and this WSS) are rebuilt, causing the
// visualizer to reconnect and reseed via ListUUIDs/GetTransform. The stream simply
// blocks until the caller's context is cancelled, at which point Next returns
// ctx.Err(). No goroutine is spawned — NewTransformChangeStreamFromChannel is
// purely closure-based.
func (m *mirror) StreamTransformChanges(ctx context.Context, _ map[string]any) (*worldstatestore.TransformChangeStream, error) {
	ch := make(chan worldstatestore.TransformChange) // never written; unblocks only via ctx
	return worldstatestore.NewTransformChangeStreamFromChannel(ctx, ch), nil
}

// Close is a no-op.
func (m *mirror) Close(_ context.Context) error { return nil }
