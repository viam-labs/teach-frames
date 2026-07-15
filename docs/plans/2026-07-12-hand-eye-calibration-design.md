# Hand-Eye Calibration — Design

**Date:** 2026-07-12
**Status:** Approved design, pending implementation plan

## Purpose

Extend the `teach-frames` module so an operator can calibrate a fixed
workcell camera to the robot's world/base frame — the transform that lets the
robot reason about what the camera sees in its own coordinates. This is the
**eye-to-hand** hand-eye problem: the camera is mounted statically in the cell
(not on the arm), and we solve **camera → world**.

It reuses the module's existing "touch a point with the calibrated TCP"
paradigm: the arm's kinematics give each calibration point's *world* coordinate,
and the operator marks the same physical point in the camera image to give its
*camera-frame* coordinate. With ≥3 such pairs we solve a single rigid transform.

Scope of this phase: **eye-to-hand only**, **RGBD camera**, **manual point
picking**, **point-correspondence (Kabsch/Umeyama) solve**. Eye-in-hand and
fiducial/AX=XB solving are explicitly deferred (see [Extensibility](#extensibility)).

## Why point-correspondence, not AX=XB

The general hand-eye formulation (Tsai-Lenz, Park-Martin, Daniilidis) solves
`AX = XB` from a fiducial observed across many poses and recovers a full 6-DOF
chain. It is more general but needs a pose-returning fiducial and more
machinery.

Point-correspondence is a better fit here because:

- It reuses the existing touch-to-capture flow almost verbatim — the world
  coordinate of each point is just the current TCP pose, exactly like
  `capture_point`.
- With an RGBD camera, a single clicked pixel deprojects directly to a 3D
  camera-frame point; no fiducial, no ML, no multi-view triangulation.
- The math is a closed-form SVD solve (Kabsch/Umeyama) with a clean residual
  trust signal, mirroring the existing pivot-solve idiom.

**Prerequisite (documented, not enforced):** world-point accuracy is bounded by
the TCP calibration, so hand-eye should be run *after* `teach_tcp_position`. The
existing TCP-teaching feature feeds this one.

## Architecture — extend `pose-tracker`, no new model

The feature slots into the existing `pose-tracker` component rather than adding
a model, reusing its buffer/store, persistence, credential, and rollback
machinery.

### New config attribute

| Attribute | Type | Required | Description |
|---|---|---|---|
| `camera` | string | no | Name of the RGBD camera to calibrate. When set, it is declared as a required dependency and enables the hand-eye DoCommands (same pattern as `arm`). |

Camera intrinsics are read from the camera's `GetProperties`. If the camera does
not expose intrinsics, the relevant commands error clearly rather than guessing
(a future config-level intrinsics override can slot in).

### New DoCommands (sent to `pose-tracker`)

Mirrors the existing capture → define shape.

| Command | Payload | Effect | Response |
|---|---|---|---|
| `handeye_snapshot` | `{}` | Grab a fresh RGB+depth frame, **cache it** in the component, and return the RGB (base64) for the UI to freeze and display. | `image` (base64 RGB), `width`, `height` |
| `capture_handeye_point` | `{"u": <px>, "v": <px>}` | Deproject pixel `(u,v)` against the **cached** depth + intrinsics into a camera-frame 3D point; read the current TCP world pose; append the `(world, camera)` pair to the hand-eye buffer. | `index`, `buffer_len`, `world` (xyz), `camera` (xyz) |
| `get_handeye_buffer` | `{}` | Return all pairs currently in the hand-eye buffer. | `points` (array of `{world, camera}`) |
| `clear_handeye_buffer` | `{}` | Empty the hand-eye buffer. | `cleared` (count) |
| `solve_handeye` | `{}` | Kabsch solve over the buffer → camera→world pose; persist to `camera.frame` (parent `world`) via the existing app-API path; report residual; clear the buffer on success. | `committed`, `pose` (camera→world), `residual_rms` |

All commands error cleanly when no `camera` is configured, matching the `arm`
gating on the jog/TCP commands.

### The snapshot cache — why it matters for correctness

The depth frame used for deprojection **must** be the exact frame the operator
clicked on, not a fresh one taken Δt later. `handeye_snapshot` captures RGB+depth
together and caches them; the UI freezes and displays that RGB; the click
deprojects against the **same cached depth**. The scene is static during
calibration anyway, but caching removes the race entirely and keeps RGB, depth,
and click provably aligned.

### Reused wholesale

`SaveComponentFrame` (app-API persistence to a component's `frame`), the
credential/`persist == nil` gating, the buffer/store pattern, `poseToMap`, and
the `commitMu` + rollback discipline.

## The physical procedure

Per calibration point:

1. Jog the TCP tip to touch a physical feature visible to the camera.
2. `handeye_snapshot` — freeze the current view in the UI.
3. Click the tip in the frozen image → the `(world, camera)` pair is stored.

Repeat **4+ times**, spread across the workspace with varied X/Y **and Z**.
Coplanar points leave the out-of-plane axis under-determined; the solve rejects
degenerate configurations rather than returning a bad transform.

## Solve & quality gates

Kabsch/Umeyama, rotation-only (no scale — both sides are metric mm):

1. Centroid-subtract both point sets.
2. `H = Σ cameraᵢ · worldᵢᵀ`.
3. SVD `H = U S Vᵀ`.
4. `R = V · diag(1, 1, det(V Uᵀ)) · Uᵀ` (the `det` term prevents a reflection).
5. `t = worldₘ − R · cameraₘ`.

Guards, matching the pivot solve's discipline:

- Rank deficiency (collinear/coplanar points) → descriptive error, **buffer
  preserved**.
- `residual_rms` (mm) reported as the trust signal — how consistently the pairs
  agree on one rigid transform.

The result `(R, t)` is persisted as `camera.frame` translation + orientation
vector, `parent: world`.

## Frontend — `HandEyePanel`

The one genuinely new UI surface (v1 shipped no camera feed). Gated on a
configured `camera` exactly as `TcpPanel` gates on `arm` (warning line when
absent).

- **Frozen-frame workflow, single image path.** A **Refresh view** button calls
  `handeye_snapshot`; the returned base64 RGB is shown in an `<img>`. The
  operator jogs (existing JogPanel) to touch the point, refreshes, and clicks the
  tip in the frozen image. No live WebRTC stream and no second image source to
  keep in sync — the frame shown is the frame the depth aligns to.
- **Click → pixel mapping** is the fiddly correctness bit: convert display
  coordinates to *native* image pixels via `naturalWidth/clientWidth`
  (accounting for `object-fit` and DPR) before sending `u,v`. Extracted into a
  unit-tested helper.
- **Feedback:** dots overlaid at captured pixels, a small pairs table
  (world xyz / camera xyz), and `residual_rms` after solve with the same
  trust-signal framing as TCP teaching. Capture / Clear / Solve buttons mirror
  the TCP panel's rhythm.

## Testing

- **Go unit tests** (pure functions, like `compute_test.go` / `tcp_test.go`):
  - Kabsch round-trip: apply a known `R,t` to synthetic world points, recover
    it, assert `< ε` and residual ≈ 0.
  - Degeneracy: collinear/coplanar inputs → descriptive error, buffer preserved.
  - Deprojection: pixel + depth + synthetic intrinsics → known 3D point.
- **Frontend vitest:** the pixel-mapping helper and the DoCommand payload
  builders (extends the existing builder tests).
- **Manual integration checklist** addendum (live camera + cloud persist),
  matching the existing checklist style: configure camera, run the touch/click
  loop for 4+ points, solve, verify `camera.frame` was patched in config and the
  camera renders in the correct world location in the visualizer.

## Extensibility

Keep **deprojection** and **solve** as separate, interface-boundaried modules so
future work slots in without disturbing the buffer/persist plumbing:

- **Eye-in-hand** (future): reuses deprojection unchanged; needs a different
  solver (camera moves with the arm, so a fixed world point observed from N poses
  becomes an AX=XB-style problem, parent = flange). The seam is already present.
- **AX=XB with a pose-returning fiducial** (future): an alternate solver +
  observation source behind the same buffer/persist machinery — enables full
  6-DOF and better robustness when a fiducial vision module is available.
- **Pluggable deprojection** also lets point-cloud-only or RGB-only multi-view
  deprojection drop in later.

## Out of scope (this phase)

- Eye-in-hand configuration.
- Fiducial/AX=XB solving.
- Live camera streaming in the UI (frozen snapshots only).
- Automatic point detection (manual pick only).
