# teach-frames

A Viam module that lets you define named workcell coordinate frames by jogging an arm to reference points — similar to a UR teach-pendant plane calibration. Taught frames are exposed via the PoseTracker API and mirrored to a `world_state_store` service so they render as axes triads in the Viam visualizer.

## Overview

The typical workflow is:

1. Jog the arm TCP to one or more reference points.
2. Issue `capture_point` DoCommands to record each point into a capture buffer.
3. Issue `define_frame` with a name and method (e.g. `3point`) to compute and persist the frame.
4. The frame immediately appears in `Poses()` output and is written back into the component's own config via the Viam app API so it survives restarts.
5. The companion `world-state-store` service mirrors all frames to the Viam visualizer as axes triads.

## Models

### `viam-labs:teach-frames:pose-tracker` (`rdk:component:pose_tracker`)

Implements the PoseTracker API. Captures TCP poses from a motion service and manages a set of named coordinate frames.

**Config attributes**

| Attribute | Type | Required | Default | Description |
|---|---|---|---|---|
| `motion_service` | string | yes | — | Name of the motion service dependency used to query the TCP pose. |
| `tcp_component` | string | yes | — | Name of the component (e.g. an arm) whose TCP pose is captured. |
| `destination_frame` | string | no | `"world"` | The reference frame that captured poses and taught frames are expressed relative to. |
| `frames` | array | no | `[]` | Module-managed list of taught frames. Do not hand-edit unless you understand the `FrameSpec` schema (each entry has `name`, `method`, `parent`, and `pose` with `x`, `y`, `z`, `o_x`, `o_y`, `o_z`, `theta`). |

The component declares `motion_service` as a required dependency so the RDK wires it up before construction.

### `viam-labs:teach-frames:world-state-store` (`rdk:service:world_state_store`)

Read-only mirror that syncs the pose-tracker's frame set into the Viam visualization layer. Exists solely for visualization — it does not store state of its own.

**Config attributes**

| Attribute | Type | Required | Description |
|---|---|---|---|
| `pose_tracker` | string | yes | Name of the `teach-frames:pose-tracker` component to mirror. |

The service declares `pose_tracker` as a required dependency. When the pose-tracker is rebuilt (e.g. after `define_frame` persists a new frame), the visualizer reconnects and refreshes via `ListUUIDs`/`GetTransform`.

## Teaching workflow: DoCommand reference

All DoCommands are sent to the **pose-tracker** component. Each command is a single-key map.

| Command key | Payload | Effect | Response keys |
|---|---|---|---|
| `capture_point` | `{}` | Reads the current TCP pose and appends it to the in-memory capture buffer. | `index` (0-based position), `buffer_len`, `pose` (x/y/z/o_x/o_y/o_z/theta) |
| `get_buffer` | `{}` | Returns all poses currently in the capture buffer. | `points` (array of pose maps) |
| `clear_buffer` | `{}` | Empties the capture buffer. | `cleared` (count removed) |
| `define_frame` | `{"name": "<name>", "method": "3point"\|"point"\|"tcp_snapshot"}` | Computes a frame from the buffer, persists it (requires platform-API creds), and clears the buffer on success. | `name`, `committed` (bool), `replaced` (bool), `pose` |
| `list_frames` | `{}` | Returns all currently committed frames. | `frames` (map of name → pose) |
| `delete_frame` | `{"name": "<name>"}` | Removes a single named frame and persists the updated set. | `deleted` (bool) |
| `clear_frames` | `{}` | Removes all committed frames and persists the empty set. | `deleted` (count) |

### `define_frame` methods

| Method | Buffer requirement | How the frame origin/orientation is computed |
|---|---|---|
| `3point` | 3 points | Point 0 is the frame origin. Point 1 defines the +X direction. Point 2 is a point in the +XY half-plane (used to derive +Y and then +Z via cross-product). |
| `point` | 1 point | Frame origin is set to the captured point; orientation is identity (inherits `destination_frame` orientation). |
| `tcp_snapshot` | 1 point | The full TCP pose (position + orientation) is stored verbatim as the frame. |

**Example — define a 3-point fixture frame:**

```json
{"capture_point": {}}
{"capture_point": {}}
{"capture_point": {}}
{"define_frame": {"name": "fixture_a", "method": "3point"}}
```

## Persistence and credentials

Taught frames are persisted by patching this component's own `attributes.frames` in the robot part config via the Viam app API (read-modify-write on `GetRobotPart` / `UpdateRobotPart`).

The module reads credentials from the standard platform-API environment variables injected by the Viam agent:

| Env var | Purpose |
|---|---|
| `VIAM_API_KEY` | API key secret |
| `VIAM_API_KEY_ID` | API key ID |
| `VIAM_MACHINE_PART_ID` | Part ID of this machine part |

If any of these variables is absent at startup, persistence is **disabled** — a warning is logged and the component still starts. `capture_point` and `Poses()` continue to work. `define_frame`, `delete_frame`, and `clear_frames` return an error ("persistence disabled") rather than silently losing state.

See https://docs.viam.com/build-modules/platform-apis/ for the env var names and the SDK helper used to build the app client.

> **Status note:** `AppPersister.Save` (the actual API call in `persist/appclient.go`) returns "not yet implemented" and will be completed in Task 15. Until then, `define_frame`, `delete_frame`, and `clear_frames` will fail at the persist step even when credentials are present. `capture_point`, `get_buffer`, `clear_buffer`, and `list_frames` work normally.

## Example config

```json
{
  "components": [
    {
      "name": "my-arm",
      "api": "rdk:component:arm",
      "model": "viam:arm:ur5e",
      "attributes": {}
    },
    {
      "name": "teach-tracker",
      "api": "rdk:component:pose_tracker",
      "model": "viam-labs:teach-frames:pose-tracker",
      "attributes": {
        "motion_service": "builtin",
        "tcp_component": "my-arm",
        "destination_frame": "world"
      }
    }
  ],
  "services": [
    {
      "name": "frames-vis",
      "api": "rdk:service:world_state_store",
      "model": "viam-labs:teach-frames:world-state-store",
      "attributes": {
        "pose_tracker": "teach-tracker"
      }
    }
  ]
}
```

## Manual integration checklist

The app-API persistence path requires a live cloud connection and has no unit test. Use this checklist to validate end-to-end behavior:

1. **Configure prerequisites.** Add an arm and a motion service (`builtin`) to the machine config.
2. **Add both models.** Add `viam-labs:teach-frames:pose-tracker` (referencing the motion service and arm) and `viam-labs:teach-frames:world-state-store` (referencing the pose-tracker by name).
3. **Check startup logs.** Confirm no "persistence disabled" warning (all three env vars present). If warned, check that the module is running under the Viam agent with the correct machine credentials.
4. **Jog and capture.** Move the arm to three reference positions. After each position, send `{"capture_point": {}}` and confirm the response contains the expected buffer index.
5. **Define a frame.** Send `{"define_frame": {"name": "fixture_a", "method": "3point"}}`. Confirm `"committed": true` in the response.
6. **Verify config persistence.** Open the machine config in the Viam app. Confirm that the pose-tracker component's `attributes.frames` array now contains an entry with `name: "fixture_a"`.
7. **Verify visualization.** Open the Viam visualization panel. Confirm an axes triad labeled `fixture_a` appears at the expected location in the scene.
8. **Restart the machine.** Restart viam-server (or reload the module). Confirm `{"list_frames": {}}` still returns `fixture_a` — i.e. the frame was reloaded from config on startup.

## Build and development

```sh
# Build the module binary
make build

# Run all tests
make test

# Lint (gofmt + go vet)
make lint

# Package for upload to the Viam registry
make module.tar.gz
```

`make module.tar.gz` produces `module.tar.gz` containing `bin/teach-frames` and `meta.json`. The tarball is gitignored.
