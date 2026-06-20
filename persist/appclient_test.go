package persist

import (
	"testing"

	"go.viam.com/test"

	"github.com/viam-labs/teach-frames/config"
)

// buildRobotConfig constructs a minimal robot config map that mirrors what
// AppClient.GetRobotPart returns in its RobotPart.RobotConfig field.
func buildRobotConfig() map[string]interface{} {
	return map[string]interface{}{
		"components": []interface{}{
			// An unrelated component that must remain untouched.
			map[string]interface{}{
				"name":       "arm",
				"type":       "arm",
				"attributes": map[string]interface{}{"model": "fake"},
			},
			// The target component with a pre-existing attribute that must be
			// preserved alongside the new frames key.
			map[string]interface{}{
				"name": "teach-tracker",
				"type": "generic",
				"attributes": map[string]interface{}{
					"existing_key": "existing_value",
					// Simulate old frames that will be replaced.
					"frames": []interface{}{
						map[string]interface{}{"name": "old_frame"},
					},
				},
			},
		},
	}
}

func TestSetComponentFrames_UpdatesTarget(t *testing.T) {
	cfg := buildRobotConfig()
	newFrames := []config.FrameSpec{
		{
			Name:   "a",
			Parent: "world",
			Pose:   config.PoseSpec{X: 1, OZ: 1},
		},
	}

	got, err := setComponentFrames(cfg, "teach-tracker", newFrames)
	test.That(t, err, test.ShouldBeNil)

	components, ok := got["components"].([]interface{})
	test.That(t, ok, test.ShouldBeTrue)
	test.That(t, len(components), test.ShouldEqual, 2)

	// Find the teach-tracker component.
	var tracker map[string]interface{}
	for _, raw := range components {
		comp, ok := raw.(map[string]interface{})
		if !ok {
			continue
		}
		if comp["name"] == "teach-tracker" {
			tracker = comp
			break
		}
	}
	test.That(t, tracker, test.ShouldNotBeNil)

	attrs, ok := tracker["attributes"].(map[string]interface{})
	test.That(t, ok, test.ShouldBeTrue)

	// The frames list must contain exactly one entry named "a".
	framesRaw, ok := attrs["frames"].([]interface{})
	test.That(t, ok, test.ShouldBeTrue)
	test.That(t, len(framesRaw), test.ShouldEqual, 1)

	frame, ok := framesRaw[0].(map[string]interface{})
	test.That(t, ok, test.ShouldBeTrue)
	test.That(t, frame["name"], test.ShouldEqual, "a")
	test.That(t, frame["parent"], test.ShouldEqual, "world")

	pose, ok := frame["pose"].(map[string]interface{})
	test.That(t, ok, test.ShouldBeTrue)
	test.That(t, pose["x"], test.ShouldEqual, float64(1))
}

func TestSetComponentFrames_PreservesExistingAttributes(t *testing.T) {
	cfg := buildRobotConfig()
	newFrames := []config.FrameSpec{{Name: "b", Parent: "base"}}

	got, err := setComponentFrames(cfg, "teach-tracker", newFrames)
	test.That(t, err, test.ShouldBeNil)

	components, _ := got["components"].([]interface{})
	var tracker map[string]interface{}
	for _, raw := range components {
		comp, _ := raw.(map[string]interface{})
		if comp["name"] == "teach-tracker" {
			tracker = comp
			break
		}
	}

	attrs, ok := tracker["attributes"].(map[string]interface{})
	test.That(t, ok, test.ShouldBeTrue)

	// Pre-existing attribute must still be present.
	test.That(t, attrs["existing_key"], test.ShouldEqual, "existing_value")

	// And frames was replaced (not the old value).
	framesRaw, _ := attrs["frames"].([]interface{})
	test.That(t, len(framesRaw), test.ShouldEqual, 1)
	frame, _ := framesRaw[0].(map[string]interface{})
	test.That(t, frame["name"], test.ShouldEqual, "b")
}

func TestSetComponentFrames_OtherComponentUntouched(t *testing.T) {
	cfg := buildRobotConfig()
	newFrames := []config.FrameSpec{{Name: "x"}}

	got, err := setComponentFrames(cfg, "teach-tracker", newFrames)
	test.That(t, err, test.ShouldBeNil)

	components, _ := got["components"].([]interface{})
	var arm map[string]interface{}
	for _, raw := range components {
		comp, _ := raw.(map[string]interface{})
		if comp["name"] == "arm" {
			arm = comp
			break
		}
	}
	test.That(t, arm, test.ShouldNotBeNil)

	attrs, ok := arm["attributes"].(map[string]interface{})
	test.That(t, ok, test.ShouldBeTrue)
	test.That(t, attrs["model"], test.ShouldEqual, "fake")

	// arm must have no "frames" key.
	_, hasFrames := attrs["frames"]
	test.That(t, hasFrames, test.ShouldBeFalse)
}

func TestSetComponentFrames_ComponentNotFound(t *testing.T) {
	cfg := buildRobotConfig()
	_, err := setComponentFrames(cfg, "nonexistent-component", nil)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "nonexistent-component")
}

func TestSetComponentFrames_InputNotMutated(t *testing.T) {
	cfg := buildRobotConfig()
	// Capture original arm attributes pointer.
	origComponents, _ := cfg["components"].([]interface{})
	origArm, _ := origComponents[0].(map[string]interface{})
	origArmAttrs, _ := origArm["attributes"].(map[string]interface{})

	newFrames := []config.FrameSpec{{Name: "z"}}
	_, err := setComponentFrames(cfg, "teach-tracker", newFrames)
	test.That(t, err, test.ShouldBeNil)

	// The original arm attributes must be unchanged.
	test.That(t, origArmAttrs["model"], test.ShouldEqual, "fake")

	// The original cfg map must still point at the old components slice.
	test.That(t, len(origComponents), test.ShouldEqual, 2)
}

func TestSetComponentFrames_EmptyFramesAllowed(t *testing.T) {
	cfg := buildRobotConfig()
	got, err := setComponentFrames(cfg, "teach-tracker", []config.FrameSpec{})
	test.That(t, err, test.ShouldBeNil)

	components, _ := got["components"].([]interface{})
	var tracker map[string]interface{}
	for _, raw := range components {
		comp, _ := raw.(map[string]interface{})
		if comp["name"] == "teach-tracker" {
			tracker = comp
			break
		}
	}
	attrs, _ := tracker["attributes"].(map[string]interface{})
	framesRaw, ok := attrs["frames"].([]interface{})
	test.That(t, ok, test.ShouldBeTrue)
	test.That(t, len(framesRaw), test.ShouldEqual, 0)
}
