// Package main is the entrypoint for the teach-frames Viam module.
package main

import (
	"context"

	"go.viam.com/utils"

	"go.viam.com/rdk/components/posetracker"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/module"

	// Importing the model package triggers its init(), which registers the model.
	teachtracker "github.com/viam-labs/teach-frames/models/posetracker"
)

func main() {
	utils.ContextualMainWithSIGPIPE(mainWithArgs, module.NewLoggerFromArgs("teach-frames"))
}

func mainWithArgs(ctx context.Context, args []string, logger logging.Logger) error {
	logger.Info("teach-frames module starting")

	m, err := module.NewModuleFromArgs(ctx)
	if err != nil {
		return err
	}
	defer m.Close(ctx)

	if err = m.AddModelFromRegistry(ctx, posetracker.API, teachtracker.Model); err != nil {
		return err
	}

	if err = m.Start(ctx); err != nil {
		return err
	}
	<-ctx.Done()
	return nil
}
