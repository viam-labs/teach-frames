package persist

import (
	"context"
	"encoding/json"
	"fmt"
	"os"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/app"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/spatialmath"

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

// setComponentFrame returns a copy of robotConfig with the named component's
// `frame.translation` and/or `frame.orientation` set. A nil translation or
// orientation leaves that sub-field untouched. Existing frame keys (geometry, a
// non-empty parent) are preserved; parent falls back to the provided value when
// absent or empty. Like setComponentFrames, it does not mutate the input.
func setComponentFrame(
	robotConfig map[string]interface{},
	componentName, parent string,
	translation *r3.Vector,
	orientation spatialmath.Orientation,
) (map[string]interface{}, error) {
	newConfig := make(map[string]interface{}, len(robotConfig))
	for k, v := range robotConfig {
		newConfig[k] = v
	}

	rawComponents, ok := newConfig["components"]
	if !ok {
		rawComponents = []interface{}{}
	}
	components, ok := rawComponents.([]interface{})
	if !ok {
		return nil, fmt.Errorf("robot config \"components\" is not a list")
	}

	found := false
	newComponents := make([]interface{}, len(components))
	for i, raw := range components {
		comp, ok := raw.(map[string]interface{})
		if !ok {
			newComponents[i] = raw
			continue
		}
		if name, _ := comp["name"].(string); name != componentName {
			newComponents[i] = comp
			continue
		}

		newComp := make(map[string]interface{}, len(comp)+1)
		for k, v := range comp {
			newComp[k] = v
		}

		frame := map[string]interface{}{}
		if rawFrame, ok := newComp["frame"].(map[string]interface{}); ok {
			for k, v := range rawFrame {
				frame[k] = v
			}
		}
		if existingParent, _ := frame["parent"].(string); existingParent == "" {
			frame["parent"] = parent
		}
		if translation != nil {
			frame["translation"] = map[string]interface{}{
				"x": translation.X, "y": translation.Y, "z": translation.Z,
			}
		}
		if orientation != nil {
			ov := orientation.OrientationVectorDegrees()
			frame["orientation"] = map[string]interface{}{
				"type": "ov_degrees",
				"value": map[string]interface{}{
					"x": ov.OX, "y": ov.OY, "z": ov.OZ, "th": ov.Theta,
				},
			}
		}
		newComp["frame"] = frame
		newComponents[i] = newComp
		found = true
	}
	if !found {
		return nil, fmt.Errorf("component %q not found in robot config", componentName)
	}

	newConfig["components"] = newComponents
	return newConfig, nil
}

// SaveComponentFrame does a read-modify-write of the named component's frame
// (translation and/or orientation) in the robot part config. A nil translation
// or orientation leaves that sub-field unchanged. Needs a live cloud connection;
// covered by the manual integration checklist, not unit tests.
func (p *AppPersister) SaveComponentFrame(
	ctx context.Context,
	componentName, parent string,
	translation *r3.Vector,
	orientation spatialmath.Orientation,
) error {
	viamClient, err := app.CreateViamClientWithAPIKey(ctx, app.Options{}, p.apiKey, p.apiKeyID, p.logger)
	if err != nil {
		return fmt.Errorf("create viam client: %w", err)
	}
	defer viamClient.Close()

	appClient := viamClient.AppClient()
	part, _, err := appClient.GetRobotPart(ctx, p.partID)
	if err != nil {
		return fmt.Errorf("get robot part %s: %w", p.partID, err)
	}

	newConfig, err := setComponentFrame(part.RobotConfig, componentName, parent, translation, orientation)
	if err != nil {
		return fmt.Errorf("patch frame for component %q: %w", componentName, err)
	}
	if _, err := appClient.UpdateRobotPart(ctx, p.partID, part.Name, newConfig); err != nil {
		return fmt.Errorf("update robot part %s: %w", p.partID, err)
	}
	return nil
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
