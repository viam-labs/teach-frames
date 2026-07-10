package persist

import (
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
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

	// Capture the TARGET component's original attributes and frames value so we
	// can assert that in-place mutation of the target is also guarded.
	origTracker, _ := origComponents[1].(map[string]interface{})
	origTrackerAttrs, _ := origTracker["attributes"].(map[string]interface{})
	origTrackerFrames := origTrackerAttrs["frames"]

	newFrames := []config.FrameSpec{{Name: "z"}}
	_, err := setComponentFrames(cfg, "teach-tracker", newFrames)
	test.That(t, err, test.ShouldBeNil)

	// The original arm attributes must be unchanged.
	test.That(t, origArmAttrs["model"], test.ShouldEqual, "fake")

	// The original cfg map must still point at the old components slice.
	test.That(t, len(origComponents), test.ShouldEqual, 2)

	// The original target's attributes map must not have been mutated — the
	// patched frames must NOT have leaked back into the input map.
	// The original frames value should still be the old single-element slice,
	// not the new "z" slice.
	oldFrames, ok := origTrackerFrames.([]interface{})
	test.That(t, ok, test.ShouldBeTrue)
	test.That(t, len(oldFrames), test.ShouldEqual, 1)
	oldFrame, _ := oldFrames[0].(map[string]interface{})
	test.That(t, oldFrame["name"], test.ShouldEqual, "old_frame")
	// Confirm the attrs map entry still points at the original slice.
	test.That(t, origTrackerAttrs["frames"], test.ShouldResemble, origTrackerFrames)
}

// --- Guard-branch tests (Fix I1) ---

// TestSetComponentFrames_ComponentsNotAList covers the guard that fires when
// the "components" key holds a value that is not a []interface{}.
func TestSetComponentFrames_ComponentsNotAList(t *testing.T) {
	for _, badVal := range []interface{}{
		"not-a-list",
		map[string]interface{}{"key": "value"},
		42,
	} {
		cfg := map[string]interface{}{
			"components": badVal,
		}
		_, err := setComponentFrames(cfg, "teach-tracker", nil)
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err.Error(), test.ShouldContainSubstring, "not a list")
	}
}

// TestSetComponentFrames_NonMapEntrySkipped verifies that a non-map entry in
// the components slice (e.g. a bare string) is silently skipped and does not
// panic. The real target component must still be patched successfully.
func TestSetComponentFrames_NonMapEntrySkipped(t *testing.T) {
	cfg := map[string]interface{}{
		"components": []interface{}{
			// A malformed entry that is not a map — must be skipped.
			"not-a-map-entry",
			// The real target.
			map[string]interface{}{
				"name":       "teach-tracker",
				"type":       "generic",
				"attributes": map[string]interface{}{},
			},
		},
	}
	newFrames := []config.FrameSpec{{Name: "q", Parent: "world"}}

	got, err := setComponentFrames(cfg, "teach-tracker", newFrames)
	test.That(t, err, test.ShouldBeNil)

	components, ok := got["components"].([]interface{})
	test.That(t, ok, test.ShouldBeTrue)
	test.That(t, len(components), test.ShouldEqual, 2)

	// First entry is the malformed string — must be preserved as-is.
	test.That(t, components[0], test.ShouldEqual, "not-a-map-entry")

	// Second entry is the patched target.
	tracker, ok := components[1].(map[string]interface{})
	test.That(t, ok, test.ShouldBeTrue)
	attrs, ok := tracker["attributes"].(map[string]interface{})
	test.That(t, ok, test.ShouldBeTrue)
	framesRaw, ok := attrs["frames"].([]interface{})
	test.That(t, ok, test.ShouldBeTrue)
	test.That(t, len(framesRaw), test.ShouldEqual, 1)
}

// TestSetComponentFrames_NonMapAttributes verifies that when a target
// component's "attributes" value is not a map (e.g. a string), it is silently
// replaced with a fresh map containing the new frames key — no panic.
func TestSetComponentFrames_NonMapAttributes(t *testing.T) {
	cfg := map[string]interface{}{
		"components": []interface{}{
			map[string]interface{}{
				"name":       "teach-tracker",
				"type":       "generic",
				"attributes": "this-is-not-a-map",
			},
		},
	}
	newFrames := []config.FrameSpec{{Name: "r", Parent: "base"}}

	got, err := setComponentFrames(cfg, "teach-tracker", newFrames)
	test.That(t, err, test.ShouldBeNil)

	components, ok := got["components"].([]interface{})
	test.That(t, ok, test.ShouldBeTrue)
	tracker, ok := components[0].(map[string]interface{})
	test.That(t, ok, test.ShouldBeTrue)

	// attributes must now be a proper map with frames set.
	attrs, ok := tracker["attributes"].(map[string]interface{})
	test.That(t, ok, test.ShouldBeTrue)
	framesRaw, ok := attrs["frames"].([]interface{})
	test.That(t, ok, test.ShouldBeTrue)
	test.That(t, len(framesRaw), test.ShouldEqual, 1)
	frame, _ := framesRaw[0].(map[string]interface{})
	test.That(t, frame["name"], test.ShouldEqual, "r")
}

// TestSetComponentFrames_NoComponentsKey documents that when the "components"
// key is absent entirely the helper returns a graceful "not found" error — not
// a panic and not a nil error.
func TestSetComponentFrames_NoComponentsKey(t *testing.T) {
	cfg := map[string]interface{}{
		"other_key": "value",
	}
	_, err := setComponentFrames(cfg, "teach-tracker", nil)
	test.That(t, err, test.ShouldNotBeNil)
	// Component is not found because the components list is empty.
	test.That(t, err.Error(), test.ShouldContainSubstring, "teach-tracker")
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

func TestSetComponentFrame_TranslationAndOrientation(t *testing.T) {
	cfg := map[string]interface{}{
		"components": []interface{}{
			map[string]interface{}{"name": "tool", "attributes": map[string]interface{}{}},
		},
	}
	tr := &r3.Vector{X: 10, Y: -5, Z: 120}
	ov := &spatialmath.OrientationVectorDegrees{OZ: 1, Theta: 45}

	out, err := setComponentFrame(cfg, "tool", "my-arm", tr, ov)
	test.That(t, err, test.ShouldBeNil)

	comp := out["components"].([]interface{})[0].(map[string]interface{})
	frame := comp["frame"].(map[string]interface{})
	test.That(t, frame["parent"], test.ShouldEqual, "my-arm")
	transl := frame["translation"].(map[string]interface{})
	test.That(t, transl["x"], test.ShouldEqual, 10.0)
	test.That(t, transl["z"], test.ShouldEqual, 120.0)
	orient := frame["orientation"].(map[string]interface{})
	test.That(t, orient["type"], test.ShouldEqual, "ov_degrees")

	// Original config is not mutated.
	origComp := cfg["components"].([]interface{})[0].(map[string]interface{})
	_, hadFrame := origComp["frame"]
	test.That(t, hadFrame, test.ShouldBeFalse)
}

func TestSetComponentFrame_MissingComponent(t *testing.T) {
	cfg := map[string]interface{}{"components": []interface{}{}}
	_, err := setComponentFrame(cfg, "nope", "arm", &r3.Vector{}, nil)
	test.That(t, err, test.ShouldNotBeNil)
}

// TestSetComponentFrame_OverridesParentPreservesGeometry locks in the current
// contract: the provided parent is AUTHORITATIVE for TCP-teaching writes (it
// overrides any differing existing parent), while other existing frame keys
// (e.g. geometry) are preserved untouched.
func TestSetComponentFrame_OverridesParentPreservesGeometry(t *testing.T) {
	cfg := map[string]interface{}{
		"components": []interface{}{
			map[string]interface{}{
				"name": "tool",
				"frame": map[string]interface{}{
					"parent":   "existing-parent",
					"geometry": map[string]interface{}{"type": "box"},
				},
			},
		},
	}
	out, err := setComponentFrame(cfg, "tool", "my-arm", &r3.Vector{X: 1}, nil)
	test.That(t, err, test.ShouldBeNil)
	frame := out["components"].([]interface{})[0].(map[string]interface{})["frame"].(map[string]interface{})
	test.That(t, frame["parent"], test.ShouldEqual, "my-arm")           // overridden
	test.That(t, frame["geometry"], test.ShouldNotBeNil)                // preserved
}

// TestSetComponentFrame_OverridesExistingWorldParent is a regression test for
// bug I1: a tool with a pre-existing frame.parent of "world" (a common RDK
// default) must have its parent forcibly replaced by the provided arm parent
// when TCP teaching writes the flange-relative translation. Leaving "world" in
// place would silently place the tool 120mm from the WORLD origin instead of
// off the arm flange.
func TestSetComponentFrame_OverridesExistingWorldParent(t *testing.T) {
	cfg := map[string]interface{}{
		"components": []interface{}{
			map[string]interface{}{
				"name": "tool",
				"frame": map[string]interface{}{
					"parent":      "world",
					"translation": map[string]interface{}{"x": 0.0, "y": 0.0, "z": 0.0},
				},
			},
		},
	}
	newTranslation := &r3.Vector{X: 10, Y: -5, Z: 120}
	out, err := setComponentFrame(cfg, "tool", "my-arm", newTranslation, nil)
	test.That(t, err, test.ShouldBeNil)

	frame := out["components"].([]interface{})[0].(map[string]interface{})["frame"].(map[string]interface{})
	test.That(t, frame["parent"], test.ShouldEqual, "my-arm")

	transl := frame["translation"].(map[string]interface{})
	test.That(t, transl["x"], test.ShouldEqual, 10.0)
	test.That(t, transl["y"], test.ShouldEqual, -5.0)
	test.That(t, transl["z"], test.ShouldEqual, 120.0)
}

// TestSetComponentFrame_NilLeavesExistingSubfield locks in the headline
// contract: a nil sub-field argument leaves that sub-field untouched. Here a
// non-nil translation is written while a nil orientation preserves the
// component's pre-existing orientation unchanged.
func TestSetComponentFrame_NilLeavesExistingSubfield(t *testing.T) {
	existingOrientation := map[string]interface{}{
		"type": "ov_degrees",
		"value": map[string]interface{}{
			"x": 0.0, "y": 0.0, "z": 1.0, "th": 90.0,
		},
	}
	cfg := map[string]interface{}{
		"components": []interface{}{
			map[string]interface{}{
				"name": "tool",
				"frame": map[string]interface{}{
					"parent":      "my-arm",
					"translation": map[string]interface{}{"x": 1.0, "y": 2.0, "z": 3.0},
					"orientation": existingOrientation,
				},
			},
		},
	}

	// Update translation only; nil orientation must leave the existing one intact.
	out, err := setComponentFrame(cfg, "tool", "my-arm", &r3.Vector{X: 10, Y: 20, Z: 30}, nil)
	test.That(t, err, test.ShouldBeNil)

	frame := out["components"].([]interface{})[0].(map[string]interface{})["frame"].(map[string]interface{})

	// (a) Translation was updated to the new value.
	transl := frame["translation"].(map[string]interface{})
	test.That(t, transl["x"], test.ShouldEqual, 10.0)
	test.That(t, transl["y"], test.ShouldEqual, 20.0)
	test.That(t, transl["z"], test.ShouldEqual, 30.0)

	// (b) Pre-existing orientation is preserved unchanged.
	test.That(t, frame["orientation"], test.ShouldResemble, existingOrientation)
}
