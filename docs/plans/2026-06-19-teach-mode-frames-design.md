# Teach-Mode Frames Module — Design

**Date:** 2026-06-19
**Status:** Approved design, pending implementation plan

## Purpose

A Viam Go module that lets an operator define arbitrary named coordinate frames
in a workcell by jogging a robot arm to points in space — a "teach mode"
analogous to Universal Robots' teach-pendant *Planes/Features*. Captured frames
are exposed for programmatic motion use and rendered live in Viam's
visualization tooling.

The module replaces the need for a camera/vision feedback loop to establish
workcell reference frames: the arm's own kinematics (via the motion service)
are the measurement source.

## Background & key findings

Investigation of the RDK codebase and the visualization repo established the
constraints this design is built on:

- **`PoseTracker` API is observation-mechanism-agnostic.** Its only method is
  `Poses(ctx, bodyNames, extra) -> map[name]*PoseInFrame`. Nothing ties it to a
  camera; a taught fixture is just "a named frame in space," which fits the
  contract. (`components/posetracker/pose_tracker.go:44`)
- **Motion does NOT implicitly consume `world_state_store`.** The builtin motion
  service builds its planning `WorldState` solely from `MoveReq.WorldState`
  passed by the caller. A repo-wide search finds zero references to
  `worldstatestore` in `services/motion/`, `motionplan/`, or
  `robot/framesystem/`; auto-consume is a documented *future* intent only
  (`motionplan/armplanning/api.go:344`). So consumers must read frames and pass
  them into `Move` explicitly.
- **The visualization tool is built around `world_state_store`.** It
  auto-discovers every `world_state_store` service, seeds via
  `ListUUIDs`/`GetTransform`, and subscribes to `StreamTransformChanges`.
  Transforms carrying `metadata.showAxesHelper` render as XYZ axes triads —
  exactly the primitive for displaying taught frames. `PoseTracker` is **not**
  consumed by the viz. (visualization `useWorldState.svelte.ts`, `draw.ts`)
- **Both `PoseTracker` and `world_state_store` are read-only at the API
  boundary.** Writes happen inside the implementation; the teach/capture action
  must ride on `DoCommand`.
- **A module can implement `world_state_store`** — registered via
  `resource.RegisterAPI`; a model registers with `resource.RegisterService`
  (`services/worldstatestore/fake/fake.go:61`). `StreamTransformChanges` is
  channel-based via `NewTransformChangeStreamFromChannel`
  (`services/worldstatestore/world_state_store.go:142`).
- **`Reconfigure` is effectively deprecated.** Modern modules use
  `AlwaysRebuild`, so every config change calls the constructor fresh (new
  instance). This shapes the wiring below.

## Decisions (from brainstorming)

| Decision | Choice |
|---|---|
| Resource topology | `PoseTracker` (authoritative) + dependent `world_state_store` mirror |
| Teach methods | `3point` plane, `tcp_snapshot`, `point` |
| Pose source | Motion service `GetPose` for the TCP component, in a configurable destination frame (default `world`) |
| Persistence | Module patches its own component config via the app API, using env-injected machine credentials |
| Language | Go |
| Mirror wiring | **Option 1** — WSS depends on PoseTracker; commits rebuild both; viz reseeds (rebuild-and-reseed), simplest & idiomatic |

## Architecture

One Go module, one process, two registered resources, plus external touchpoints.

```
+-- teach-frames module (one binary) -----------------------------+
|                                                                 |
|  PoseTracker model  (authoritative)                             |
|   - owns committed taught frames (loaded from config)           |
|   - DoCommand: capture/define/list/delete/clear (teach surface) |
|   - Poses() -> map[name]PoseInFrame   (programmatic readers)    |
|   - capture: motion.GetPose(tcp, dest="world")                  |
|   - persist: patches own component config via app API           |
|   - AlwaysRebuild: constructor reloads frames from config       |
|                          ^ direct local dependency pointer      |
|  WorldStateStore model   | (Viam dep on PoseTracker)            |
|   - reads current frames from its PoseTracker on (re)build      |
|   - ListUUIDs / GetTransform  (seed)                            |
|   - StreamTransformChanges    (open; reseed on rebuild)         |
|   - maps frames -> commonpb.Transform (+ showAxesHelper)        |
+-----------------------------------------------------------------+
        | motion.GetPose         | app API patch        | WSS RPC
        v                        v                       v
   motion service          Viam Cloud (config)     Visualization web app
   (frame system)          = durable persistence    (axes triads)

   PoseTracker.Poses()  -> your motion program -> assemble WorldState -> Move
```

### AlwaysRebuild semantics (the wiring basis)

- Every config change re-runs the constructor; there is no `Reconfigure`.
- `define_frame` / `delete_frame` / `clear_frames` each patch config → the
  PoseTracker is rebuilt, and its constructor reloads `attributes.frames`.
  **Config is the single source of truth.**
- `capture_point` does **not** touch config, so the in-memory capture buffer
  survives across captures; `define_frame` consumes it and is the rebuild
  trigger.
- Rebuilding the PoseTracker propagates a rebuild to the dependent WSS (the
  basis for "reseed"). *To be validated during implementation.*

## Components

### PoseTracker model (`<org>:teach-frames:pose-tracker`)

Config attributes:

```json
{
  "motion_service": "builtin",
  "tcp_component": "arm",
  "destination_frame": "world",
  "frames": [
    {
      "name": "fixture_a",
      "method": "3point",
      "parent": "world",
      "pose": { "x": 0.42, "y": -0.10, "z": 0.05,
                "o_x": 0, "o_y": 0, "o_z": 1, "theta": 35.0 }
    }
  ]
}
```

- `frames` is human-readable but **module-managed** — teaching rewrites it via
  the app API; hand-edits load on (re)build.
- Pose uses Viam orientation-vector form (`o_x,o_y,o_z,theta`) to match
  `PoseInFrame`/`spatialmath`.
- Credentials (`VIAM_API_KEY`, `VIAM_API_KEY_ID`, machine part id) come from
  **env vars**, not config.
- Declared dependency: `motion_service`.

### WorldStateStore model (`<org>:teach-frames:world-state-store`)

```json
{ "pose_tracker": "taught-frames" }
```

- Declared dependency: the named PoseTracker (delivered as a direct local Go
  pointer for same-module deps).
- Read/mirror only. Derives each transform's UUID deterministically from the
  frame name (UUIDv5) so identity is stable across reseed rebuilds.
- Maps each frame → `commonpb.Transform{ reference_frame: name,
  pose_in_observer_frame: PoseInFrame("world", pose),
  metadata: { showAxesHelper: true } }`.

## DoCommand surface (teach API)

All single-key maps; responses echo useful state.

| Command | Payload | Effect | Returns |
|---|---|---|---|
| `capture_point` | `{}` | `motion.GetPose(tcp, "world")` → append to buffer | `{index, pose, buffer_len}` |
| `clear_buffer` | `{}` | Discard in-progress captures | `{cleared}` |
| `get_buffer` | `{}` | Debug: list buffered points | `{points}` |
| `define_frame` | `{name, method}` | Validate buffer, compute frame, **patch config**, clear buffer | `{name, pose, committed, replaced}` |
| `list_frames` | `{}` | Current committed frames | `{frames}` |
| `delete_frame` | `{name}` | Remove one, **patch config** | `{deleted}` |
| `clear_frames` | `{}` | Remove all, **patch config** | `{deleted}` |

### Method → buffer contract for `define_frame`

- `point` — needs ≥1 point; uses `buffer[0].position`, orientation = identity.
- `tcp_snapshot` — needs ≥1 point; uses `buffer[0]` full pose as-is.
- `3point` — needs exactly 3 points: `p1`=origin, `p2`=+X direction, `p3`=in
  +XY half-plane.
  - `X̂ = norm(p2−p1)`, `temp = p3−p1`, `Ẑ = norm(X̂ × temp)`, `Ŷ = Ẑ × X̂`,
    `origin = p1`.
  - Rotation `[X̂ Ŷ Ẑ]` → orientation vector. Reject collinear/near-degenerate
    (cross-product magnitude below epsilon).

The buffer is purely in-memory and never persisted; only committed frames hit
config.

## Data flow

**Capture:**
```
capture_point
  -> motion.GetPose(ctx, tcp_component, dest="world")
  -> PoseInFrame(parent="world", pose)
  -> append to in-memory buffer
  -> return {index, pose}
```

**Define + persist (config = single writer):**
```
define_frame{name, method}
  -> validate buffer -> compute world-frame Pose
  -> connect to app via env creds
  -> read latest part config -> locate this component by name
  -> set attributes.frames[name] -> UpdateRobotPart
  -> return {name, pose, committed:true}   (synchronous; errors propagate)

      (Cloud persists, pushes config)
  constructor re-runs (AlwaysRebuild)
  -> load attributes.frames as the live set
  -> dependent WSS rebuilds, reloads frames, viz reconnects/reseeds
```

**Mirror to visualization:**
```
viz -> ListUUIDs() -> [uuid(name) ...]
    -> GetTransform(uuid) -> Transform{reference_frame, pose_in_observer_frame,
                                       metadata.showAxesHelper:true}  (axes triad)
    -> StreamTransformChanges()  (held open; reseed on next rebuild)
```

**Programmatic consumption (motion):**
```
program -> PoseTracker.Poses(bodyNames) -> map[name]PoseInFrame
        -> referenceframe.WorldState{transforms: taught frames}
        -> motion.Move(MoveReq{..., WorldState})   (explicit; unchanged here)
```

## Error handling & edge cases

- **Capture:** `motion.GetPose` failure → error, buffer untouched.
- **Define:** insufficient buffer → error, buffer preserved; degenerate `3point`
  geometry → reject (Gram-Schmidt basis, force right-handed); empty name →
  error; existing name → upsert (`replaced:true`).
- **Persistence:** missing env creds → warn at construction, hard-error on
  `define/delete/clear`; app API failure → error, nothing committed (no config
  patch → no rebuild). Read-modify-write race is last-writer-wins over the part
  config (mutating only our component entry); acceptable for single-operator,
  human-paced teaching — documented, not engineered around in v1.
- **Lifecycle:** a rebuild mid-session loses the in-progress buffer (documented;
  recapture). WSS `Close` closes its change channel / cancels stream ctx to
  avoid goroutine leaks. Stable UUIDv5-from-name keeps viz entity identity
  consistent across reseeds.
- **Concurrency:** frames + buffer are mutex-guarded.

## Testing

Two seams abstracted behind interfaces to keep teach logic pure:

- `PoseSource{ Capture(ctx) (PoseInFrame, error) }` — real impl wraps
  `motion.GetPose`; fake returns canned poses.
- `ConfigPersister{ Save(ctx, frames) error }` — real impl wraps the app API;
  fake records calls.

**Unit (no robot):** frame math (`3point` known-input → expected orientation;
collinear rejection; `point`/`tcp_snapshot`); buffer state machine; config
round-trip; UUID stability; transform mapping (+`showAxesHelper`).

**Component (injected deps):** fake `PoseSource` + fake `ConfigPersister`:
`capture_point`×3 → `define_frame{3point}`; assert computed frame and persisted
set. `Poses()` filtering by `bodyNames`.

**WSS:** `ListUUIDs`/`GetTransform` over a populated store;
`StreamTransformChanges` via the channel helper (mirror
`services/worldstatestore/fake` test pattern); reseed-on-rebuild.

**Integration / manual (README checklist):** real arm + motion service → teach a
3-point plane → confirm `attributes.frames` patched → confirm viz renders the
triad → restart → confirm reload.

**CI:** module's own `make lint` / `make test` (Go), following rdk conventions.

## Known limitations / future work

- Config read-modify-write is last-writer-wins (see above).
- Mirror uses rebuild-and-reseed (Option 1). If per-commit viz churn proves
  annoying, revisit **Option 2** (process-level shared `FrameStore`, no
  dependency edge) for smooth live deltas.
- Motion does not auto-consume the store today; if the
  `armplanning/api.go:344` future lands, the WSS frames could be picked up by
  the planner implicitly.
- TCP offset relies on the existing frame-system TCP frame; a configurable
  in-module tool offset is out of scope for v1.
