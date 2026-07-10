package persist

import (
	"context"
	"encoding/json"
	"fmt"
	"os"

	"go.viam.com/rdk/app"
	"go.viam.com/rdk/logging"

	"github.com/viam-labs/teach-frames/config"
)

// AppPersister patches this component's attributes.frames in the robot part
// config via the Viam app API, authenticating with module-injected env vars.
type AppPersister struct {
	ComponentName string
	logger        logging.Logger
	apiKey        string
	apiKeyID      string
	partID        string
}

// NewAppPersister reads platform-API credentials from the standard module env
// vars. See https://docs.viam.com/build-modules/platform-apis/ for the env var
// names and the SDK helper used to build the app client. The provided logger is
// stored and used inside Save; pass the module's own logger so log lines carry
// the correct component name. Returns an error if any credential is absent.
func NewAppPersister(componentName string, logger logging.Logger) (*AppPersister, error) {
	p := &AppPersister{
		ComponentName: componentName,
		logger:        logger,
		apiKey:        os.Getenv("VIAM_API_KEY"),
		apiKeyID:      os.Getenv("VIAM_API_KEY_ID"),
		partID:        os.Getenv("VIAM_MACHINE_PART_ID"),
	}
	if p.apiKey == "" || p.apiKeyID == "" || p.partID == "" {
		return nil, fmt.Errorf(
			"missing platform-API env vars (VIAM_API_KEY/VIAM_API_KEY_ID/VIAM_MACHINE_PART_ID); "+
				"cannot persist frames for %s", componentName)
	}
	return p, nil
}

// setComponentFrames returns a copy of robotConfig with the named component's
// attributes.frames replaced by the serialized frames slice. It is a pure
// function — the input is not mutated in a way that corrupts other components.
//
// robotConfig is the map[string]interface{} returned by AppClient.GetRobotPart.
// The function locates the "components" list, finds the entry whose "name"
// equals componentName, ensures it has an "attributes" map, and sets
// attributes["frames"] to the JSON-round-tripped representation of frames.
func setComponentFrames(
	robotConfig map[string]interface{},
	componentName string,
	frames []config.FrameSpec,
) (map[string]interface{}, error) {
	// Marshal frames to JSON then back into a []interface{} so the result is
	// structpb-compatible (no custom Go types that protoutils.StructToStructPb
	// cannot handle).
	framesJSON, err := json.Marshal(frames)
	if err != nil {
		return nil, fmt.Errorf("marshal frames: %w", err)
	}
	var framesVal []interface{}
	if err := json.Unmarshal(framesJSON, &framesVal); err != nil {
		return nil, fmt.Errorf("unmarshal frames: %w", err)
	}

	// Deep-copy the top-level map so we don't accidentally alias the original.
	// We only need a shallow copy at the top level; we'll re-assign the
	// components slice with a fresh slice that replaces just the target entry.
	newConfig := make(map[string]interface{}, len(robotConfig))
	for k, v := range robotConfig {
		newConfig[k] = v
	}

	// Locate the components list.
	rawComponents, ok := newConfig["components"]
	if !ok {
		rawComponents = []interface{}{}
	}
	components, ok := rawComponents.([]interface{})
	if !ok {
		return nil, fmt.Errorf("robot config \"components\" is not a list")
	}

	// Find the named component and patch it.
	found := false
	newComponents := make([]interface{}, len(components))
	for i, raw := range components {
		comp, ok := raw.(map[string]interface{})
		if !ok {
			newComponents[i] = raw
			continue
		}
		name, _ := comp["name"].(string)
		if name != componentName {
			newComponents[i] = comp
			continue
		}

		// Copy this component's map and update attributes.frames.
		newComp := make(map[string]interface{}, len(comp))
		for k, v := range comp {
			newComp[k] = v
		}

		// Ensure the attributes sub-map exists and is a copy.
		var attrs map[string]interface{}
		if rawAttrs, hasAttrs := newComp["attributes"]; hasAttrs {
			if existing, ok := rawAttrs.(map[string]interface{}); ok {
				attrs = make(map[string]interface{}, len(existing))
				for k, v := range existing {
					attrs[k] = v
				}
			}
		}
		if attrs == nil {
			attrs = make(map[string]interface{})
		}
		attrs["frames"] = framesVal
		newComp["attributes"] = attrs
		newComponents[i] = newComp
		found = true
	}
	if !found {
		return nil, fmt.Errorf("component %q not found in robot config", componentName)
	}

	newConfig["components"] = newComponents
	return newConfig, nil
}

// Save does a read-modify-write of this component's attributes.frames in the
// robot part config (GetRobotPart -> mutate only this component's entry ->
// UpdateRobotPart). This path needs a live cloud connection and is covered by
// the manual integration checklist rather than unit tests.
func (p *AppPersister) Save(ctx context.Context, frames []config.FrameSpec) error {
	viamClient, err := app.CreateViamClientWithAPIKey(
		ctx,
		app.Options{},
		p.apiKey,
		p.apiKeyID,
		p.logger,
	)
	if err != nil {
		return fmt.Errorf("create viam client: %w", err)
	}
	defer viamClient.Close()

	appClient := viamClient.AppClient()

	part, _, err := appClient.GetRobotPart(ctx, p.partID)
	if err != nil {
		return fmt.Errorf("get robot part %s: %w", p.partID, err)
	}

	newConfig, err := setComponentFrames(part.RobotConfig, p.ComponentName, frames)
	if err != nil {
		return fmt.Errorf("patch config for component %q: %w", p.ComponentName, err)
	}

	if _, err := appClient.UpdateRobotPart(ctx, p.partID, part.Name, newConfig); err != nil {
		return fmt.Errorf("update robot part %s: %w", p.partID, err)
	}

	return nil
}
