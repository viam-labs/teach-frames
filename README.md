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
| `arm` | string | no | — | Name of the arm component used for TCP pivot teaching. Enables the TCP-teaching DoCommands when set. Required (along with `camera`) when `camera_mount` is `eye_in_hand`, where it is also used to read flange poses for the solve. |
| `camera` | string | no | — | Name of the RGBD camera to calibrate. When set, it is declared as a dependency and enables the hand-eye calibration DoCommands. Required (along with `arm`) when `camera_mount` is `eye_in_hand`. |
| `camera_mount` | string | no | `"eye_to_hand"` | Physical mounting of the calibrated camera: `"eye_to_hand"` (fixed camera watching the workcell; solves camera→`destination_frame`) or `"eye_in_hand"` (camera bolted to the arm's wrist; solves camera→flange). `eye_in_hand` requires both `arm` and `camera`; any other value fails config validation. |
| `frames` | array | no | `[]` | Module-managed list of taught frames. Do not hand-edit unless you understand the `FrameSpec` schema (each entry has `name`, `method`, `parent`, and `pose` with `x`, `y`, `z`, `o_x`, `o_y`, `o_z`, `theta`). |

The component declares its motion service (the value of `motion_service`, defaulting to `builtin`) as a dependency so the RDK wires it up before construction. When `arm` is set, the named arm is declared as a required dependency too and is used for TCP pivot teaching (see [TCP teaching](#tcp-teaching) below) and, in `eye_in_hand` mode, for hand-eye calibration — it is independent of `tcp_component`, which is the component whose TCP/tool frame gets taught. When `camera` is set, the named RGBD camera is declared as a required dependency too and is used exclusively for hand-eye calibration, either the [eye-to-hand](#hand-eye-calibration-eye-to-hand) or [eye-in-hand](#hand-eye-calibration-eye-in-hand) flow depending on `camera_mount`.

### `viam-labs:teach-frames:world-state-store` (`rdk:service:world_state_store`)

Read-only mirror that syncs the pose-tracker's frame set into the Viam visualization layer. Exists solely for visualization — it does not store state of its own.

**Config attributes**

| Attribute | Type | Required | Description |
|---|---|---|---|
| `pose_tracker` | string | yes | Name of the `teach-frames:pose-tracker` component to mirror. |

The service declares `pose_tracker` as a required dependency. When the pose-tracker is rebuilt (e.g. after `define_frame` persists a new frame), the visualizer reconnects and refreshes via `ListUUIDs`/`GetTransform`.

> **Known issue and stopgap.** The `world_state_store` mirror's WSS stream to the Viam visualizer/frontend goes permanently stale after the pose-tracker's first `AlwaysRebuild` reconfigure (see `docs/plans/2026-07-20-motion-tools-wss-reconfigure-upstream-patch.md` for the root cause), so deletions and edits to taught frames stop reaching an already-connected client. The `teach-pendant` frontend in this repo works around this by remounting the `<Visualizer>` after a define/delete/clear made through its own UI, which forces a fresh `world_state_store` snapshot. Two known costs: (1) a brief scene refresh on those actions — camera and floating-panel positions may reset, and the Manage Frames panel closes; and (2) **direct edits to the pose-tracker config made outside this UI still require a manual page reload** to show up, until the upstream fix lands.

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
| `jog_cartesian` | `{"axis": "x"\|"y"\|"z"\|"roll"\|"pitch"\|"yaw", "step": <mm-or-deg>, "frame": <optional>}` | Nudges the TCP by a signed step along/about the axes of `frame` (`"world"`, `"tool"`, or a taught frame name). Omitting `frame` is back-compat: translation along **world** axes, rotation about **tool** axes. See [Jogging](#jogging) below for the full frame convention. Reads `EndPosition`, applies the delta, and calls `MoveToPosition`. Errors if no `arm` is configured or the axis is unknown. | `pose` (resulting TCP pose), `moved` (bool) |
| `jog_joint` | `{"joint": <index>, "step": <deg>}` | Nudges a single joint by a signed step (degrees) and calls `MoveToJointPositions`. Errors if no `arm` is configured or the joint index is out of range. | `joints` (resulting positions, degrees) |
| `stop_arm` | `{}` | Stops arm motion immediately (`arm.Stop`). Errors if no `arm` is configured. | `stopped` (bool) |
| `handeye_snapshot` | `{}` | Acquires an RGBD frame from the configured `camera` and caches the depth map + intrinsics for the next capture; in `eye_in_hand` mode, also reads and caches the current flange pose in the same call, so the pixels and the flange pose that produced them stay paired. Returns the color image as a base64 JPEG. Errors if no `camera` is configured, or (`eye_in_hand` only) if the `arm` dependency isn't configured/resolvable. | `image` (base64 JPEG string), `width`, `height` |
| `capture_handeye_point` | `{"u": <pixel-col>, "v": <pixel-row>}` | **`eye_to_hand` only.** Deprojects the clicked pixel against the cached snapshot (requires a prior `handeye_snapshot`), reads the current TCP world pose, and appends the `(world, camera)` pair to the hand-eye buffer. Errors if no `camera` is configured, no snapshot is cached, `u`/`v` are missing or non-numeric, the pixel has no valid depth, or `camera_mount` is `eye_in_hand` (use `capture_handeye_target`/`capture_handeye_view` instead). | `index` (0-based position), `buffer_len`, `world` (x/y/z), `camera` (x/y/z) |
| `get_handeye_mode` | `{}` | Returns the configured `camera_mount` and whether the `camera`/`arm` dependencies resolved, so a client can gate its UI without guessing. | `camera_mount` (`"eye_to_hand"` \| `"eye_in_hand"`), `camera_configured` (bool), `arm_configured` (bool) |
| `get_handeye_buffer` | `{}` | Returns the buffered observations. Shape depends on `camera_mount`. | `eye_to_hand`: `points` (array of `{"world": {x,y,z}, "camera": {x,y,z}}`). `eye_in_hand`: `points` (array of `{"target": {x,y,z}, "flange": {x,y,z,o_x,o_y,o_z,theta}, "camera": {x,y,z}}`) |
| `clear_handeye_buffer` | `{}` | Empties the hand-eye buffer. In `eye_in_hand` mode, also discards the current target set by `capture_handeye_target` (a stale target would silently poison the next session). | `cleared` (count removed) |
| `capture_handeye_target` | `{}` | **`eye_in_hand` only.** Reads the current TCP pose and stores its point as the target that subsequent `capture_handeye_view` calls pair against. Errors if `camera_mount` is `eye_to_hand` (use `capture_handeye_point` instead). | `target` (x/y/z) |
| `capture_handeye_view` | `{"u": <pixel-col>, "v": <pixel-row>}` | **`eye_in_hand` only.** Deprojects the clicked pixel against the snapshot cached by the last `handeye_snapshot` and pairs it with the current target and the flange pose frozen at that same snapshot, appending the observation to the buffer. Each snapshot is consumed by (at most) one view — re-run `handeye_snapshot` before the next click. Errors if no target is set, no snapshot is cached, `u`/`v` are missing or non-numeric, the pixel has no valid depth, or `camera_mount` is `eye_to_hand`. | `index` (0-based position), `buffer_len`, `target` (x/y/z), `flange` (x/y/z/o_x/o_y/o_z/theta), `camera` (x/y/z) |
| `solve_handeye` | `{}` | Solves the camera→`destination_frame` transform (`eye_to_hand`) or camera→flange transform (`eye_in_hand`) over the mode's buffer (Kabsch, ≥3 observations; rejects collinear or degenerate input), persists it to the `camera` component's own `frame` — **`parent` differs by mount**: the configured `destination_frame` (default `world`) for `eye_to_hand`, the configured `arm` for `eye_in_hand` — reports the fit residual, and clears the buffer (and, in `eye_in_hand` mode, the current target) on success. Errors (buffer preserved) if persistence or the required dependency (`camera`, and `arm` for `eye_in_hand`) is not configured, or the solve fails. | `committed` (bool), `pose` (x/y/z/o_x/o_y/o_z/theta), `residual_rms` (fit residual, mm — one signal among several, see below), `parent` (the frame parent that was persisted; mount-dependent), `orientation` (o_x/o_y/o_z/theta) |

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
`step` to move the opposite direction).

`jog_cartesian` takes an optional `frame` param selecting the jog basis:

- Omitted → back-compat: `x`/`y`/`z` translate along the world/base axes (mm);
  `roll`/`pitch`/`yaw` rotate about the tool/TCP's own axes (degrees).
- `"world"` → translate **and** rotate about the world/arm-base axes.
- `"tool"` → translate **and** rotate about the tool/TCP axes.
- A taught frame name → translate **and** rotate along/about that frame's axes
  (assumes the arm base is at/near the world origin — a known limitation
  tracked as the "constraint-G" backlog item, not specific to jogging).

All rotation is composed via quaternion math so the orientation stays
well-formed. `jog_joint` nudges one joint by a signed angle (degrees), and is
unaffected by `frame`.

The teach-pendant UI's Cartesian jog panel exposes this as a **Reference**
selector (World / Tool / any taught frame), defaulting to **World** — so out
of the box, roll/pitch/yaw rotate about the world/base axes; pick **Tool** to
get the old tool-relative rotation.

The arm's own joint/reachability limits remain the safety authority — an
unreachable or out-of-limit jog surfaces as the underlying `MoveToPosition` /
`MoveToJointPositions` error. Use `stop_arm` to halt immediately.

**Example — nudge +5 mm in world X, then rotate the tool −10° about its Z:**

```json
{"jog_cartesian": {"axis": "x", "step": 5}}
{"jog_cartesian": {"axis": "yaw", "step": -10}}
```

**Example — jog relative to a taught frame:**

```json
{"jog_cartesian": {"axis": "x", "step": 5, "frame": "fixture_a"}}
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

The solve (`ComputeRigidTransform`, a closed-form Kabsch/Umeyama fit) requires
**at least 3 points and rejects collinear inputs** — points spanning only a
line leave rotation about that line unconstrained, and the command returns a
descriptive error with the buffer preserved. Coplanar (but non-collinear)
points are accepted, but 4+ points spanning varied X/Y/Z give a residual that
actually reflects 3D fit quality rather than a coincidentally perfect 2D fit.
The solve also guards against **degenerate input on either side** of the
pairing, not just the camera side — e.g. touching points without ever moving
the arm between them — and rejects it outright rather than returning a
misleadingly good fit.

**`residual_rms` (the RMS fit error, in mm, reported on success) is one signal,
not *the* trust signal — a low residual does not by itself mean the
calibration is good.** A large residual is a genuine red flag (re-touch points
more carefully or capture a better-spread set), but the converse doesn't hold:
input that is *close to* degenerate without tripping the guard above can still
produce tens of millimetres of real-world translation error while
`residual_rms` reads as an excellent ~1.5–2 mm fit — the residual simply has no
way to see error that both point clouds agree on. Use a genuinely well-spread
set of touches (varied X, Y, *and* Z, not clustered near one spot) and
cross-check the persisted pose against an independent measurement (e.g. the
visualizer) before trusting a calibration.

Point capture is **manual pick only** (no automatic fiducial or marker
detection) against a single frozen RGBD snapshot per point — there is no live
camera streaming. An RGBD (color + depth) camera exposing intrinsics is
required; a color-only camera cannot be deprojected and `handeye_snapshot`
surfaces a descriptive error in that case (see `posesource.RGBDCamera`).

On success, the solved `camera → destination-frame` pose is persisted directly to the
**camera component's own `frame` config** (`translation` and `orientation`,
with `parent` set to the configured `destination_frame`, default `world`) via the same platform-API read-modify-write path
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

Wrist-mounted cameras are not configured this way — see
[Hand-eye calibration (eye-in-hand)](#hand-eye-calibration-eye-in-hand) below,
which uses the same `ComputeRigidTransform` solver under a different adapter.

**Out of scope (this version):** fiducial/marker-based AX=XB solving for full
6-DOF robustness independent of a taught TCP. See the implementation plan for
tracked follow-ups.

### Hand-eye calibration (eye-in-hand)

Hand-eye calibration in **eye-in-hand** mode solves the pose of a camera
**bolted to the arm's wrist** relative to the flange (`camera → flange`) —
the configuration where the camera rides on the arm rather than watching it
from a fixed mount. Set `camera_mount: "eye_in_hand"` and both `arm` and
`camera` to enable it; the module rejects config where either is missing.
`camera_mount` is a physical fact about the hardware, so it does not change at
runtime: `capture_handeye_point` errors in this mode, and
`capture_handeye_target`/`capture_handeye_view` (below) error in `eye_to_hand`
mode. Send `{"get_handeye_mode": {}}` if a client needs to detect which mode a
given tracker is configured for rather than assuming.

Because the camera is rigidly mounted to the flange, the transform between
them is constant regardless of where the arm moves — so this reduces to the
same rigid point-cloud alignment (`ComputeRigidTransform`) that eye-to-hand
uses, just with the flange pose standing in for the fixed camera mount as the
reference frame. No fiducial and no AX=XB machinery is needed.

**Prerequisite: calibrate the TCP first.** Even more than in eye-to-hand mode,
accuracy here is bounded by the TCP calibration: the touched target comes from
the same TCP-teaching path (see [TCP teaching](#tcp-teaching)), and the flange
pose read at every snapshot feeds directly into the solve. Run
`teach_tcp_position` (and, if needed, `teach_tcp_orientation`) before starting
eye-in-hand calibration.

**Procedure — touch a target, then view it from several vantages:**

1. Jog the TCP to physically touch a point in the workspace and send
   `{"capture_handeye_target": {}}`. This reads the current TCP pose and
   stores its position as the target that every view below must click.
2. Jog the arm to a new vantage point — anywhere the camera can still see the
   touched target, without disturbing the target itself. **Spread vantages by
   tens of centimetres, roughly the scale of the working volume, not by a few
   millimetres.** A small jog is still "a new vantage point" as far as this
   step is concerned — the solve will accept it and report a clean residual —
   but it is measurably worse-conditioned and neither `residual_rms` nor the
   degeneracy guard will catch it (see the conditioning note below).
3. Send `{"handeye_snapshot": {}}` to freeze the camera's current RGBD view
   *and* the flange pose at that same instant, and get back a base64 JPEG of
   the color image.
4. Click the target's location in that frozen image and send
   `{"capture_handeye_view": {"u": <col>, "v": <row>}}`. This deprojects the
   pixel against the cached depth map/intrinsics and pairs it with the target
   and the flange pose frozen in step 3. Each snapshot is consumed by one
   view — snapshot again before the next click.
5. Repeat steps 2–4 from additional vantages, or go back to step 1 and touch a
   *different* target and continue viewing it. Any mix of targets and
   vantages is a valid observation set — each `(target, flange, camera)`
   triple is an independent constraint regardless of which target it came
   from.
6. Send `{"solve_handeye": {}}` once at least 3 observations are buffered.

The solve requires **at least 3 observations**; **4 or more, from genuinely
varied vantages, are recommended** for a well-conditioned solve and a
meaningful residual — the same degeneracy guards described above apply here
too (an operator who snapshots and clicks repeatedly without actually moving
the arm produces near-identical observations, which the solve rejects rather
than silently miscalibrating).

**Spread vantages by tens of centimetres, and vary wrist orientation between
vantages, not just position.** Pure-translation vantages
(jogging position only, never rotating the wrist) are **valid input and are
not rejected** — the deprojected clicks are full 3D points, not bearings, so
translation alone still constrains the rotation. But translation-only
vantages are measurably worse-conditioned than vantages that also vary
orientation, and the effect is large enough to matter:

- At a **~70 mm vantage spread**, worst-case rotation error under 1 mm of
  camera noise measures 2.42° for pure translation vs. 0.77° when orientation
  is varied too. Those figures hold at that spread — they are *not* a
  universal bound, and get worse as vantages get closer together.
- Pure-translation vantages jogged only **5–10 mm apart** — still "a new
  vantage point" by the letter of step 2 above — have been measured, under
  the same 1 mm camera noise, producing worst-case *translation* error of
  **64–118 mm across 60 trials, with none rejected** by the solve or the
  degeneracy guard. A small jog and a clean residual can still mean the
  calibration is off by a decimetre.

Per the residual caveat above, `residual_rms` will not reveal any of this —
it stays small regardless of vantage spread. Jog through a mix of positions
*and* wrist angles, spread by tens of centimetres, when practical.

**Example — one target, four vantages, then solve:**

```json
{"capture_handeye_target": {}}
{"handeye_snapshot": {}}
{"capture_handeye_view": {"u": 210, "v": 340}}
{"handeye_snapshot": {}}
{"capture_handeye_view": {"u": 480, "v": 120}}
{"handeye_snapshot": {}}
{"capture_handeye_view": {"u": 95, "v": 260}}
{"handeye_snapshot": {}}
{"capture_handeye_view": {"u": 300, "v": 400}}
{"solve_handeye": {}}
```

On success, the solved `camera → flange` pose is persisted the same way as
eye-to-hand, via the same platform-API read-modify-write path and credentials
(see [Persistence and credentials](#persistence-and-credentials)) — but to a
**different parent**: the camera component's own `frame` config is patched
with `parent` set to the configured `arm` (not `destination_frame`), since the
frame system attaches a camera mounted on the wrist as a child of the arm at
its kinematic tip. The buffer and current target are cleared on success. If
persistence is disabled, or the `camera`/`arm` dependency is not configured,
`solve_handeye` returns an error and preserves the buffer.

**Caveat: the pose-tracker rebuilds on any config edit.** The component is
`resource.AlwaysRebuild`, so editing *any* attribute of this component's
config — not just hand-eye-related ones — tears down and reconstructs the
tracker, which drops the in-memory hand-eye buffer and the current target.
If you need to change config mid-calibration (e.g. to fix a typo), expect to
restart the calibration (touch a target, re-jog, re-snapshot) afterward.

## Persistence and credentials

Taught frames are persisted by patching this component's own `attributes.frames` in the robot part config via the Viam app API (read-modify-write on `GetRobotPart` / `UpdateRobotPart`). TCP teaching (`teach_tcp_position` / `teach_tcp_orientation`) uses the same read-modify-write mechanism and credentials, but patches the `frame` of the configured `tcp_component` instead. Hand-eye calibration (`solve_handeye`) also uses the same mechanism and credentials, but patches the `frame` of the configured `camera` instead — with `parent` set to the configured `destination_frame` (default `world`) in `eye_to_hand` mode, or to the configured `arm` in `eye_in_hand` mode.

The module reads credentials from the standard platform-API environment variables injected by the Viam agent:

| Env var | Purpose |
|---|---|
| `VIAM_API_KEY` | API key secret |
| `VIAM_API_KEY_ID` | API key ID |
| `VIAM_MACHINE_PART_ID` | Part ID of this machine part |

If any of these variables is absent at startup, persistence is **disabled** — a warning is logged and the component still starts. `capture_point`, `capture_tcp_point`, `capture_handeye_point`, `capture_handeye_target`, `capture_handeye_view`, and `Poses()` continue to work. `define_frame`, `delete_frame`, `clear_frames`, `teach_tcp_position`, `teach_tcp_orientation`, and `solve_handeye` return an error ("persistence disabled") rather than silently losing state.

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

### Hand-eye calibration (eye-to-hand)

9. **Configure a camera.** Add an RGBD camera component (exposing depth and intrinsics) to the machine config, then set the pose-tracker's `camera` attribute to its name (leave `camera_mount` unset or `"eye_to_hand"`).
10. **Calibrate the TCP first.** Run the [TCP teaching](#tcp-teaching) checklist (or otherwise ensure `tcp_component.frame` is already taught) so captured world points are accurate.
11. **Touch and click.** For at least 4 points spread across the workspace (varied X, Y, *and* Z — not all on one plane or line): jog the TCP to touch the point, send `{"handeye_snapshot": {}}`, then send `{"capture_handeye_point": {"u": ..., "v": ...}}` for the clicked pixel. Confirm each response's `buffer_len` increments.
12. **Solve.** Send `{"solve_handeye": {}}`. Confirm `"committed": true` in the response, and a `residual_rms` in the expected range for careful touches (well under a few mm) — but treat that as one check among several, not proof the calibration is correct (see the residual caveat above); step 14 is what actually confirms it.
13. **Verify config persistence.** Open the machine config in the Viam app. Confirm the camera component's `frame` was patched with `parent` set to the configured `destination_frame` (default `"world"`) and the solved `translation`/`orientation`.
14. **Verify visualization.** Open the Viam visualizer. Confirm the camera renders at the correct world location/orientation in the scene (e.g. its frustum or pose triad lines up with its physical mounting).

### Hand-eye calibration (eye-in-hand)

15. **Configure a wrist camera.** Add an RGBD camera mounted on the arm's wrist to the machine config, then set the pose-tracker's `camera` attribute to its name and `camera_mount` to `"eye_in_hand"` (both `arm` and `camera` must be set — config validation rejects the mode otherwise).
16. **Calibrate the TCP first.** As in step 10 — the touched target and every flange pose feed the solve directly here, so TCP accuracy matters even more.
17. **Touch a target, then view it from several vantages.** Jog the TCP to touch a point and send `{"capture_handeye_target": {}}`; then, for at least 3 vantages (4+, with varied wrist orientation, recommended): jog the arm to a new vantage, send `{"handeye_snapshot": {}}`, then send `{"capture_handeye_view": {"u": ..., "v": ...}}` for the clicked pixel. Confirm each response's `buffer_len` increments.
18. **Solve.** Send `{"solve_handeye": {}}`. Confirm `"committed": true` in the response — again, do not treat `residual_rms` alone as sufficient; step 20 is what confirms it.
19. **Verify config persistence.** Open the machine config in the Viam app. Confirm the camera component's `frame` was patched with `parent` set to the configured `arm` (not `destination_frame`) and the solved `translation`/`orientation`.
20. **Verify visualization.** Open the Viam visualizer. Jog the arm to a couple of different poses and confirm the camera's rendered pose (frustum or pose triad) moves rigidly with the wrist rather than staying fixed in the world.

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

There are two Camera Calibration panels, one per `camera_mount` value, and
both query `get_handeye_mode` to decide which one to render — the eye-to-hand
panel shows when `camera_mount` is `eye_to_hand` (the default), the eye-in-hand
panel when it's `eye_in_hand`; exactly one is visible at a time. Either panel
attempts `handeye_snapshot` and, on a "camera dependency not configured" (or,
for eye-in-hand, "arm dependency not configured") error, shows a warning and
disables its controls, so the UI stays in parity with the backend's own
gating even before a snapshot is taken.

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
