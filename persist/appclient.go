package persist

import (
	"context"
	"fmt"
	"os"

	"github.com/viam-labs/teach-frames/config"
)

// AppPersister patches this component's attributes.frames in the robot part
// config via the Viam app API, authenticating with module-injected env vars.
type AppPersister struct {
	ComponentName string
	apiKey        string
	apiKeyID      string
	partID        string
}

// NewAppPersister reads platform-API credentials from the standard module env
// vars. See https://docs.viam.com/build-modules/platform-apis/ for the env var
// names and the SDK helper used to build the app client. Returns an error if
// any credential is absent.
func NewAppPersister(componentName string) (*AppPersister, error) {
	p := &AppPersister{
		ComponentName: componentName,
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

// Save does a read-modify-write of this component's attributes.frames in the
// robot part config (GetRobotPart -> mutate only this component's entry ->
// UpdateRobotPart). Implemented in Task 15; needs a live cloud connection so it
// has no unit test (covered by the manual integration checklist).
func (p *AppPersister) Save(ctx context.Context, frames []config.FrameSpec) error {
	return fmt.Errorf("AppPersister.Save not yet implemented")
}
