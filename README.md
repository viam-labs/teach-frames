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
| `motion_service` | string | no | `"builtin"` | Name of the motion service dependency used to query the TCP pose. |
| `tcp_component` | string | yes | — | Name of the component (e.g. an arm) whose TCP pose is captured. |
| `destination_frame` | string | no | `"world"` | The reference frame that captured poses and taught frames are expressed relative to. |
| `arm` | string | no | — | Name of the arm component used for TCP pivot teaching. Enables the TCP-teaching DoCommands when set. |
| `camera` | string | no | — | Name of the RGBD camera to calibrate. When set, it is declared as a dependency and enables the hand-eye calibration DoCommands. |
| `frames` | array | no | `[]` | Module-managed list of taught frames. Do not hand-edit unless you understand the `FrameSpec` schema (each entry has `name`, `method`, `parent`, and `pose` with `x`, `y`, `z`, `o_x`, `o_y`, `o_z`, `theta`). |

The component declares its motion service (the value of `motion_service`, defaulting to `builtin`) as a dependency so the RDK wires it up before construction. When `arm` is set, the named arm is declared as a required dependency too and is used exclusively for TCP pivot teaching (see [TCP teaching](#tcp-teaching) below) — it is independent of `tcp_component`, which is the component whose TCP/tool frame gets taught. When `camera` is set, the named RGBD camera is declared as a required dependency too and is used exclusively for hand-eye calibration (see [Hand-eye calibration (eye-to-hand)](#hand-eye-calibration-eye-to-hand) below).

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
| `capture_tcp_point` | `{}` | Reads the current arm flange pose (`arm.EndPosition`, in the arm base frame) and appends it to a separate TCP capture buffer. Errors if no `arm` is configured. | `index` (0-based position), `buffer_len`, `pose` (x/y/z/o_x/o_y/o_z/theta) |
| `teach_tcp_position` | `{}` | Least-squares pivot solve over the TCP buffer (requires ≥4 captures spanning varied orientations), persists the solved tip offset as `tcp_component.frame.translation` (parent = the configured `arm`; requires platform-API creds), and clears the TCP buffer on success. | `committed` (bool), `offset` (x/y/z), `residual_rms` (fit residual, mm) |
| `teach_tcp_orientation` | `{"o_x": <n>, "o_y": <n>, "o_z": <n>, "theta": <n>}` | Persists an explicit tool orientation (orientation-vector degrees) to `tcp_component.frame.orientation`, leaving the taught translation unchanged. Rejects an all-zero `(o_x, o_y, o_z)` vector. Requires platform-API creds. | `committed` (bool), `orientation` (o_x/o_y/o_z/theta) |
| `get_tcp_buffer` | `{}` | Returns all poses currently in the TCP capture buffer. | `points` (array of pose maps) |
| `clear_tcp_buffer` | `{}` | Empties the TCP capture buffer. | `cleared` (count removed) |
| `get_arm_state` | `{}` | Reads the current TCP pose and joint positions in a single call (for a UI poll loop). Errors if no `arm` is configured. | `pose` (x/y/z/o_x/o_y/o_z/theta), `joints` (array of degrees) |
| `jog_cartesian` | `{"axis": "x"\|"y"\|"z"\|"roll"\|"pitch"\|"yaw", "step": <mm-or-deg>}` | Nudges the TCP by a signed step: `x`/`y`/`z` translate along the **world** frame (mm); `roll`/`pitch`/`yaw` rotate about the **tool** frame (degrees). Reads `EndPosition`, applies the delta, and calls `MoveToPosition`. Errors if no `arm` is configured or the axis is unknown. | `pose` (resulting TCP pose), `moved` (bool) |
| `jog_joint` | `{"joint": <index>, "step": <deg>}` | Nudges a single joint by a signed step (degrees) and calls `MoveToJointPositions`. Errors if no `arm` is configured or the joint index is out of range. | `joints` (resulting positions, degrees) |
| `stop_arm` | `{}` | Stops arm motion immediately (`arm.Stop`). Errors if no `arm` is configured. | `stopped` (bool) |
| `handeye_snapshot` | `{}` | Acquires an RGBD frame from the configured `camera`, caches the depth map + intrinsics for the next capture, and returns the color image as a base64 JPEG. Errors if no `camera` is configured. | `image` (base64 JPEG string), `width`, `height` |
| `capture_handeye_point` | `{"u": <pixel-col>, "v": <pixel-row>}` | Deprojects the clicked pixel against the cached snapshot (requires a prior `handeye_snapshot`), reads the current TCP world pose, and appends the `(world, camera)` pair to the hand-eye buffer. Errors if no `camera` is configured, no snapshot is cached, `u`/`v` are missing or non-numeric, or the pixel has no valid depth. | `index` (0-based position), `buffer_len`, `world` (x/y/z), `camera` (x/y/z) |
| `get_handeye_buffer` | `{}` | Returns all `(world, camera)` pairs currently in the hand-eye buffer. | `points` (array of `{"world": {x,y,z}, "camera": {x,y,z}}`) |
| `clear_handeye_buffer` | `{}` | Empties the hand-eye buffer. | `cleared` (count removed) |
| `solve_handeye` | `{}` | Solves the camera→world transform over the hand-eye buffer (Kabsch, ≥3 non-collinear points), persists it to the `camera` component's own `frame` (parent `world`; requires platform-API creds), reports the fit residual, and clears the buffer on success. Errors (buffer preserved) if persistence or the `camera` dependency is not configured, or the solve fails (too few or collinear points). | `committed` (bool), `pose` (x/y/z/o_x/o_y/o_z/theta), `residual_rms` (fit residual, mm), `parent` (`"world"`), `orientation` (o_x/o_y/o_z/theta) |

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

### Jogging

The `jog_cartesian`, `jog_joint`, `stop_arm`, and `get_arm_state` commands drive
the arm named by the optional `arm` config attribute — the same dependency TCP
teaching uses. Without it, these commands return an "arm dependency not
configured" error.

Jogging is **discrete**: each command applies exactly one signed step and blocks
until the move completes; the `step` value carries its own sign (send a negative
`step` to move the opposite direction). The frame convention is **world-frame
translation, tool-frame rotation**:

- `jog_cartesian` `x`/`y`/`z` translate the TCP along the world/base axes (mm).
- `jog_cartesian` `roll`/`pitch`/`yaw` rotate about the TCP's own axes (degrees),
  composed via quaternion math so the orientation stays well-formed.
- `jog_joint` nudges one joint by a signed angle (degrees).

The arm's own joint/reachability limits remain the safety authority — an
unreachable or out-of-limit jog surfaces as the underlying `MoveToPosition` /
`MoveToJointPositions` error. Use `stop_arm` to halt immediately.

**Example — nudge +5 mm in world X, then rotate the tool −10° about its Z:**

```json
{"jog_cartesian": {"axis": "x", "step": 5}}
{"jog_cartesian": {"axis": "yaw", "step": -10}}
```

### TCP teaching

TCP teaching calibrates a tool's tip offset (its TCP) by touching a single fixed
world point from several arm orientations — the same calibration a teach
pendant performs for a new tool. It requires the optional `arm` config
attribute; without it, the `capture_tcp_point`/`teach_tcp_*` commands return an
"arm dependency not configured" error.

Unlike `capture_point` (which reads the world TCP pose via the motion service
into the frame-capture buffer used by `define_frame`), `capture_tcp_point`
reads the **arm flange pose** directly via `arm.EndPosition` into a separate,
dedicated TCP capture buffer. A taught TCP is never added to the `frames` set
and never appears in `Poses()` — it is a one-shot calibration written directly
to the `tcp_component`'s own `frame` config (translation and/or orientation),
with `tcp_component.frame.parent` set to the configured `arm`.

`teach_tcp_position` requires **at least 4** captured points spanning varied
arm orientations (the pivot solve is a least-squares fit; too few points, or
orientations that are too similar, leave the system rank-deficient and the
command returns a descriptive error without clearing the buffer). On success it
persists the solved offset to `tcp_component.frame.translation` and reports
`residual_rms` — the RMS fit residual in mm — as a trust signal for how
consistently the touches hit one physical point (aim for well under 1–2 mm with
careful touches).

`teach_tcp_orientation` is independent of the pivot solve: tool orientation
cannot be derived from tip touches, so it takes an explicit orientation vector
`(o_x, o_y, o_z, theta)` and persists it verbatim to
`tcp_component.frame.orientation`, leaving `translation` untouched. It is
optional — omitting it leaves the taught tool's orientation at identity — and
should be run after `teach_tcp_position` (or alongside a previously-taught
tip), since it never re-derives the translation. An all-zero `(o_x, o_y, o_z)`
vector is rejected.

Both `teach_tcp_position` and `teach_tcp_orientation` persist via the same
platform-API credentials as frame teaching (see
[Persistence and credentials](#persistence-and-credentials)); if persistence is
disabled, they return an error rather than silently discarding the result.

**Example — teach a TCP from 4 touches, then set its orientation:**

```json
{"capture_tcp_point": {}}
{"capture_tcp_point": {}}
{"capture_tcp_point": {}}
{"capture_tcp_point": {}}
{"teach_tcp_position": {}}
{"teach_tcp_orientation": {"o_x": 0, "o_y": 0, "o_z": 1, "theta": 0}}
```

### Hand-eye calibration (eye-to-hand)

Hand-eye calibration solves the pose of a **fixed** RGBD camera in the world
(`camera → world`) — the "eye-to-hand" configuration, where the camera watches
the workcell rather than riding on the arm. It requires the optional `camera`
config attribute; without it, `handeye_snapshot`, `capture_handeye_point`, and
`solve_handeye` return a "camera dependency not configured" error.

**Prerequisite: calibrate the TCP first.** The solve pairs each touched point's
**world** coordinate (read from the motion service via the same TCP the
frame-teaching commands use) with its **camera** coordinate (deprojected from
the clicked pixel). If the TCP's tip offset hasn't been taught (see
[TCP teaching](#tcp-teaching)), the world points are only as accurate as an
untaught tool frame — run `teach_tcp_position` (and, if needed,
`teach_tcp_orientation`) before hand-eye calibration for accurate results.

**Procedure — touch and click, repeated for each point:**

1. Jog the TCP to physically touch a point in the workspace.
2. Send `{"handeye_snapshot": {}}` to freeze the camera's current RGBD view
   and get back a base64 JPEG of the color image.
3. Click the touched point's tip in that frozen image (in the Teach Pendant
   app, or by computing the pixel yourself) and send
   `{"capture_handeye_point": {"u": <col>, "v": <row>}}`. This deprojects the
   pixel against the cached depth map/intrinsics and reads the TCP's current
   world pose, storing the `(world, camera)` pair.
4. Repeat for at least 3 points; **4 or more points spread across the
   workspace, varying X, Y, and Z (not all on one plane or line), are
   recommended** for a well-conditioned solve and a meaningful residual.
5. Send `{"solve_handeye": {}}` to compute and persist the result.

The solve (`ComputeCameraToWorld`, a closed-form Kabsch/Umeyama fit) requires
**at least 3 points and rejects collinear inputs** — points spanning only a
line leave rotation about that line unconstrained, and the command returns a
descriptive error with the buffer preserved. Coplanar (but non-collinear)
points are accepted, but 4+ points spanning varied X/Y/Z give a residual that
actually reflects 3D fit quality rather than a coincidentally perfect 2D fit.
On success, `solve_handeye` reports `residual_rms` — the RMS fit error in
mm — as the trust signal for calibration quality; treat a large residual as a
sign to re-touch points more carefully or capture a better-spread set.

Point capture is **manual pick only** (no automatic fiducial or marker
detection) against a single frozen RGBD snapshot per point — there is no live
camera streaming. An RGBD (color + depth) camera exposing intrinsics is
required; a color-only camera cannot be deprojected and `handeye_snapshot`
surfaces a descriptive error in that case (see `posesource.RGBDCamera`).

On success, the solved `camera → world` pose is persisted directly to the
**camera component's own `frame` config** (`translation` and `orientation`,
with `parent` set to `world`) via the same platform-API read-modify-write path
and credentials as frame and TCP teaching (see
[Persistence and credentials](#persistence-and-credentials)), and the hand-eye
buffer is cleared. If persistence is disabled, or the `camera` dependency is
not configured, `solve_handeye` returns an error and preserves the buffer.

**Example — capture 4 points and solve:**

```json
{"handeye_snapshot": {}}
{"capture_handeye_point": {"u": 210, "v": 340}}
{"handeye_snapshot": {}}
{"capture_handeye_point": {"u": 480, "v": 120}}
{"handeye_snapshot": {}}
{"capture_handeye_point": {"u": 95, "v": 260}}
{"handeye_snapshot": {}}
{"capture_handeye_point": {"u": 300, "v": 400}}
{"solve_handeye": {}}
```

**Out of scope (this version):** eye-in-hand configurations (camera mounted on
the arm; would reuse `Deproject` but needs an AX=XB-style solver with
`parent = flange`) and fiducial/marker-based AX=XB solving for full 6-DOF
robustness independent of a taught TCP. See the implementation plan for
tracked follow-ups.

## Persistence and credentials

Taught frames are persisted by patching this component's own `attributes.frames` in the robot part config via the Viam app API (read-modify-write on `GetRobotPart` / `UpdateRobotPart`). TCP teaching (`teach_tcp_position` / `teach_tcp_orientation`) uses the same read-modify-write mechanism and credentials, but patches the `frame` of the configured `tcp_component` instead. Hand-eye calibration (`solve_handeye`) also uses the same mechanism and credentials, but patches the `frame` of the configured `camera` instead (parent `world`).

The module reads credentials from the standard platform-API environment variables injected by the Viam agent:

| Env var | Purpose |
|---|---|
| `VIAM_API_KEY` | API key secret |
| `VIAM_API_KEY_ID` | API key ID |
| `VIAM_MACHINE_PART_ID` | Part ID of this machine part |

If any of these variables is absent at startup, persistence is **disabled** — a warning is logged and the component still starts. `capture_point`, `capture_tcp_point`, `capture_handeye_point`, and `Poses()` continue to work. `define_frame`, `delete_frame`, `clear_frames`, `teach_tcp_position`, `teach_tcp_orientation`, and `solve_handeye` return an error ("persistence disabled") rather than silently losing state.

See https://docs.viam.com/build-modules/platform-apis/ for the env var names and the SDK helper used to build the app client.

## Example config

```json
{
  "components": [
    {
      "name": "my-arm",
      "api": "rdk:component:arm",
      "model": "viam:universal-robots:ur5e",
      "attributes": {}
    },
    {
      "name": "my-gripper",
      "api": "rdk:component:gripper",
      "model": "viam:some-gripper:model",
      "attributes": {}
    },
    {
      "name": "teach-tracker",
      "api": "rdk:component:pose_tracker",
      "model": "viam-labs:teach-frames:pose-tracker",
      "attributes": {
        "motion_service": "builtin",
        "tcp_component": "my-gripper",
        "destination_frame": "world",
        "arm": "my-arm"
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

### Hand-eye calibration

9. **Configure a camera.** Add an RGBD camera component (exposing depth and intrinsics) to the machine config, then set the pose-tracker's `camera` attribute to its name.
10. **Calibrate the TCP first.** Run the [TCP teaching](#tcp-teaching) checklist (or otherwise ensure `tcp_component.frame` is already taught) so captured world points are accurate.
11. **Touch and click.** For at least 4 points spread across the workspace (varied X, Y, *and* Z — not all on one plane or line): jog the TCP to touch the point, send `{"handeye_snapshot": {}}`, then send `{"capture_handeye_point": {"u": ..., "v": ...}}` for the clicked pixel. Confirm each response's `buffer_len` increments.
12. **Solve.** Send `{"solve_handeye": {}}`. Confirm `"committed": true` and a low `residual_rms` (well under a few mm for careful touches) in the response.
13. **Verify config persistence.** Open the machine config in the Viam app. Confirm the camera component's `frame` was patched with `parent: "world"` and the solved `translation`/`orientation`.
14. **Verify visualization.** Open the Viam visualizer. Confirm the camera renders at the correct world location/orientation in the scene (e.g. its frustum or pose triad lines up with its physical mounting).

## Teach Pendant application

This module ships a browser-based **teach pendant** as a Viam Application — a
static web UI, registered in `meta.json` under `applications` (type
`single_machine`), that Viam hosts and opens in the context of one machine. It
is a full pendant: jog the arm, capture points, define frames, run TCP
teaching, and run camera hand-eye calibration from a single screen. Everything
the UI does goes through the pose-tracker's DoCommands (jogging included), so
it needs no extra configuration beyond the `pose-tracker` component — set its
optional `arm` attribute to enable the jog and TCP-teaching controls, and its
optional `camera` attribute to enable the **Camera Calibration** panel.

The **Camera Calibration** panel gates itself the same way the TCP panel gates
on `arm`: it attempts `handeye_snapshot` and, on a "camera dependency not
configured" error, shows a "Requires a configured RGBD camera." warning and
disables its controls (there is no separate camera-presence flag to check —
deriving disabled-state from the DoCommand error keeps the UI in parity with
the backend's own gating).

The app source lives in `frontend/` (Svelte 5 + Vite, built with
[`@viamrobotics/svelte-sdk`](https://github.com/viamrobotics/viam-svelte-sdk)).
`make module` builds it (`frontend/dist/`) and bundles it into the module
tarball.

### v1 scope

Numeric readouts only — live TCP pose and joint angles, capture-buffer and
frame tables. There is no in-app 3D scene or live camera feed in v1; the
Camera Calibration panel shows only a frozen snapshot per `handeye_snapshot`
call, refreshed on demand. Use the Viam visualizer (fed by the companion
`world-state-store` service) to see taught frames as axes triads. Jogging is
discrete (world-frame translation, tool-frame rotation), with a step-size
selector and an always-available **Stop**.

### Local development

`viam module local-app-testing` proxies your local dev server and injects the
same machine-credential cookie that the hosted platform provides in production —
so there is a single connection code path.

```sh
# 1. Run the app's dev server (Vite, defaults to :5173)
cd frontend && npm install && npm run dev

# 2. In another shell, from a logged-in CLI session, proxy + inject the cookie:
viam login
viam module local-app-testing \
  --app-url http://localhost:5173 \
  --machine-id YOUR-MACHINE-ID
```

The CLI opens `http://localhost:8012/start`, sets the machine cookie, and
redirects to the proxied app. `--machine-id` (from the machine's Fleet page)
selects single-machine mode.

### Frontend checks

```sh
cd frontend
npm run check   # svelte-check (types)
npm test        # vitest unit tests (DoCommand payload builders)
npm run build   # production build → frontend/dist/
```

## Build and development

```sh
# Build the module binary (bin/teach-frames)
make

# Run all tests
make test

# Lint (go vet + gofmt)
make lint

# Package for upload to the Viam registry
make module

# Remove build artifacts
make clean
```

`make module` builds the frontend (`make frontend`) and produces `bin/module.tar.gz` containing `bin/teach-frames`, `meta.json`, and `frontend/dist/` (the teach pendant application, whose `entrypoint` is `frontend/dist/index.html`). The `bin/` directory is gitignored. Cross-compilation honors `VIAM_BUILD_OS`/`VIAM_BUILD_ARCH`, which the Viam build action sets automatically.
