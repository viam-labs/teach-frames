# Eye-in-hand hand-eye calibration — design

**Status:** Approved, not yet implemented.

**Scope:** wrist-mounted (eye-in-hand) camera, **RGBD**, **manual pixel
picking**, **point-correspondence (Kabsch/Umeyama) solve**. Extends the shipped
eye-to-hand feature (`4372c5b`, PR #4). Fiducial/AX=XB solving remains out of
scope.

Supersedes `2026-07-15-eye-in-hand-handoff.md`, which asserted that Kabsch could
not be reused here. That assertion was wrong; see [The reduction](#the-reduction).

## The reduction

Eye-in-hand reduces to the **same Kabsch solve** the eye-to-hand feature already
uses. No AX=XB machinery, no fiducial, no new solver.

Let `X` be the unknown camera→flange transform, `F_i` the flange pose at
observation `i`, `c_i` the deprojected click in the camera frame, and `P` the
touched target point. Because the camera is rigidly attached to the flange, `X`
is constant, so:

```
P = F_i ∘ X ∘ c_i     ⟹     X ∘ c_i = F_i⁻¹ ∘ P  =:  q_i
```

`P` is known (the existing `capture_point` path gives it via the calibrated TCP)
and `F_i` is known (from the arm), so `q_i` is computable. That makes
`{c_i} → {q_i}` an ordinary rigid point-cloud alignment — Kabsch — recovering `X`
directly.

### Why the "different camera frames" objection is wrong

The superseded handoff argued that a moving camera puts each observation in a
different frame, so no static point cloud exists to align. This conflates two
things. The camera moves *in the world*, but it is bolted to the flange, so the
camera frame is fixed *relative to the flange* — and the flange is the only
frame the solve references. Every `c_i` lands in one consistent camera frame;
every `q_i` lands in one consistent flange frame. Kabsch applies.

### Verified properties

Checked numerically against the real `ComputeRigidTransform` before this design
was written:

| Property | Result |
|---|---|
| Recovers a known `X` from diverse flange poses | Yes, residual < 1e-6 |
| **Pure-translation vantages (no wrist rotation) recover `X`** | **Yes** |
| Collinear vantages | Rejected by the existing collinear guard |

The pure-translation result is counter-intuitive and worth stating loudly:
classic AX=XB *cannot* recover rotation without rotational motion, which is why
the superseded handoff predicted the same limitation here. It does not apply,
because deprojection yields full 3D points rather than bearings — the depth
camera supplies the constraint that wrist rotation would otherwise have to.

**Do not add a rotational-diversity check.** It would reject valid data. The
existing second-singular-value (collinearity) guard is both necessary and
sufficient for the degeneracies operators actually hit: taking every vantage
along a line, and forgetting to jog between views (all `c_i` coincide).

## Frame conventions (load-bearing)

The RDK frame system gives each component **two** frames. For an arm named
`arm1`:

- `arm1_origin` — static, placed by the component's `frame` config. The arm
  **base**.
- `arm1` — the kinematic model, whose tip is the **end effector** (flange).

A child with `frame.parent: "arm1"` therefore attaches at the **flange**, not
the base. This is what makes `parent = <arm>` correct for a wrist camera, and it
matches the shipped TCP-teaching precedent (`tcp_component.frame.parent = arm`).

### The mismatch this creates

Two sources report the flange in **different frames**:

| Source | Frame |
|---|---|
| `arm.EndPosition` (what `posesource.FlangeSource` wraps) | arm base (`arm1_origin`) |
| `motion.GetPose(arm, destFrame)` | `destination_frame` |

`P` arrives in `destination_frame` (via `source.Capture`). The solve is only
correct when `P` and `F_i` are in the **same** frame — the equation
`P = F_i ∘ X ∘ c_i` is frame-agnostic, but only if both terms agree.

**So eye-in-hand must not reuse `FlangeSource`.** It uses
`motion.GetPose(arm, destFrame)`, which puts `F_i` and `P` in the same frame by
construction. Reusing `FlangeSource` would pass every test on a machine whose
arm sits at the world origin and silently miscalibrate everywhere else — the
same failure shape as the `destination_frame` parent bug found in eye-to-hand's
final review.

No new source type is needed: `motion.Service.GetPose` takes a plain
`componentName string`, so the existing, tested `posesource.MotionSource` works
verbatim with `Component: cfg.Arm`.

TCP teaching is unaffected — `ComputePivotTCP` consumes only flange poses, so it
is frame-agnostic and self-consistent.

## Architecture

Extends `pose-tracker`. No new model.

### Config

One new attribute:

| Attribute | Type | Required | Default | Notes |
|---|---|---|---|---|
| `camera_mount` | string | no | `"eye_to_hand"` | One of `eye_to_hand`, `eye_in_hand`. |

`Validate` rejects any other value, and requires both `arm` and `camera` when
the mount is `eye_in_hand`. The default preserves every existing config
unchanged.

### Tracker state

```go
flangeInDest posesource.PoseSource // &MotionSource{Motion: mot, Component: cfg.Arm, DestFrame: dest}
cameraMount  string
currentTarget *r3.Vector           // set by capture_handeye_target
lastFlange   spatialmath.Pose      // cached alongside lastSnapshot, under snapshotMu
```

### Solver

`frames/handeye.go` — generalized core, honestly named:

```go
// PointPair is one 3D correspondence: the same physical point in the camera
// frame and in the frame we are solving the camera's pose relative to.
type PointPair struct { Reference, Camera r3.Vector }

func ComputeRigidTransform(pairs []PointPair) (spatialmath.Pose, float64, error)
```

`frames/eyeinhand.go` — new:

```go
type EyeInHandObservation struct {
    Target r3.Vector        // touched point, in destination_frame
    Flange spatialmath.Pose // flange in destination_frame, at view time
    Camera r3.Vector        // deprojected click, in camera frame
}

func ComputeCameraToFlange(obs []EyeInHandObservation) (spatialmath.Pose, float64, error)
```

`ComputeCameraToFlange` builds `q_i = Flange⁻¹ ∘ Target` and delegates to
`ComputeRigidTransform`. One Kabsch implementation, one place the transposed-
rotation convention lives.

**Renames from the shipped code:** `HandEyePair{World, Camera}` →
`PointPair{Reference, Camera}`; `ComputeCameraToWorld` → `ComputeRigidTransform`.
Mechanical, and covered by the existing tests. `Reference` is accurate in both
modes, where `World` would be a lie in one — the shipped name was already
imprecise (the value is a `destination_frame` point, not necessarily world), and
generalizing it is what makes the reuse honest rather than a trap.

### Command surface

`handeye_snapshot`, `get_handeye_buffer`, and `clear_handeye_buffer` are shared;
the latter two branch on mount to return the right shape. Because mount is fixed
by config, a given machine only ever sees one shape.

| Verb | `eye_to_hand` | `eye_in_hand` |
|---|---|---|
| `capture_handeye_point` | OK | **error** — names the right verb |
| `capture_handeye_target` | **error** | OK — reads TCP pose, sets current target |
| `capture_handeye_view {u,v}` | **error** | OK — pairs click against current target |
| `solve_handeye` | camera→destFrame, parent = `destination_frame` | camera→flange, parent = `arm` |

## Data flow

```
capture_handeye_target
  P = source.Capture()                 -> TCP pose in destination_frame
  store as current target

handeye_snapshot                       [operator has jogged to a vantage]
  snap = cameraSrc.Snapshot()          -> RGB + depth + intrinsics
  F    = flangeInDest.Capture()        -> flange in destination_frame
  cache BOTH under snapshotMu
  return {image, width, height}

capture_handeye_view {u, v}            [operator clicks the target in the frozen frame]
  c = Deproject(cached depth, u, v)    -> point in camera frame
  F = cached flange                    -> NOT a fresh read
  append EyeInHandObservation{Target: P, Flange: F, Camera: c}

  ... repeat snapshot+view per vantage, or set a new target ...

solve_handeye
  X, residual = ComputeCameraToFlange(observations)
  persist camera.frame {parent: arm, translation, orientation}
  clear buffer only on success
```

### The snapshot must freeze the flange pose

`handeye_snapshot` caches `F` **alongside** the image, under the same
`snapshotMu`. Eye-to-hand never needed this: it touches and clicks at the same
arm pose. Eye-in-hand jogs, snapshots, *then* clicks — so a flange pose read at
click time could describe a pose the pixels never came from.

This is the same race the frozen snapshot already solves for depth, with the
same fix. It is invisible in the field: the calibration comes out slightly
wrong with a plausible residual.

### Observation protocol

The buffer accepts **any mix** of targets and vantages — one target from four
vantages, four targets from one vantage each, or anything between. Each
observation is an independent constraint regardless of which target it came
from, so this falls out of the math at no cost.

Minimum 3 observations; 4+ for a meaningful residual (3 is exactly determined).

## Error handling

All of these preserve the buffer and refuse rather than guess:

- Wrong verb for the configured mount → error naming the correct verb.
- `capture_handeye_view` with no target → "call capture_handeye_target first".
- `capture_handeye_view` with no cached snapshot → as eye-to-hand today.
- `solve_handeye` with <3 observations, or collinear camera points → the core's
  existing guards.
- Persist failure → error, buffer kept, nothing committed.
- Missing camera or arm dependency → error at `Validate`, not at solve time.

Resolution-mismatch and IR-as-color guards are inherited unchanged from
`posesource`.

## Frontend

A separate `EyeInHandPanel.svelte` alongside `HandEyePanel.svelte`. The
snapshot viewer, dot overlay, and buffer table are **extracted into shared
components** used by both, rather than duplicated. Each panel renders only when
the configured `camera_mount` matches, so an operator never sees a panel that
can only error. `displayToNativePixel` is reused as-is.

## Testing

TDD. **The round-trip test is written first** — it is the guard for the
transposed-rotation convention, which has already caused one bug on this
feature. If it fails, nothing downstream is worth debugging.

`frames/eyeinhand_test.go`:

- Round-trip: known `X`, known `P`, diverse flange poses → recovers `X`.
- **Pure translation recovers `X`.** A documentation test as much as a
  correctness one: the property is false for AX=XB, and without a test pinning
  it, someone will "fix" this non-bug by adding a rotational-diversity check and
  begin rejecting valid data.
- Mixed targets (2 targets × 2 vantages) → recovers `X`. This is what validates
  the flexible protocol.
- Collinear vantages → error. Coincident vantages (forgot to jog) → error.
- <3 observations → error. Noisy input → residual > 0 and bounded.

`frames/handeye_test.go`: existing tests carry over under the rename with no
other edits — that is what proves the extraction was behavior-preserving.

`models/posetracker`:

- Each wrong-verb/mount pairing errors.
- View without target; view without snapshot.
- `Validate`: bad `camera_mount` string; `eye_in_hand` missing `arm`;
  `eye_in_hand` missing `camera`; default is `eye_to_hand`.
- **Snapshot-freezes-flange regression test.** `posesource.Fake` pops a queued
  pose per `Capture`, so queue `[F1, F2]`, snapshot, view, assert the
  observation holds `F1`. A re-read implementation gets `F2` and fails.
- **Parent-frame regression test.** Solve persists `parent = <arm>` — the direct
  analogue of the `destination_frame` test that caught the shipped bug.

`store`: buffer add/read/len/clear, copy-on-read.

Frontend: `EyeInHandPanel` and the extracted shared components.

### Review

A **final holistic review is mandatory**, not optional. Both bugs that escaped
eye-to-hand were integration bugs — a mislabeled parent, a missing resolution
check — and both passed per-task reviews that were individually correct. Only
the cross-cutting pass found them.

## Out of scope

- Fiducial / AX=XB solving.
- Automatic point detection (manual pick only).
- Live camera streaming (frozen snapshots only).
- Changing the eye-to-hand workflow.

## Known gaps

- **No hardware validation.** Eye-to-hand's math has never been run against a
  real RGBD camera; eye-in-hand builds on it. Everything here tests against
  fakes. That is real verification of the math and plumbing, but it is not
  evidence the feature works on a bench.
- TCP calibration remains a documented, unenforced prerequisite — `P` is only as
  accurate as `teach_tcp_position` made it. This matters more here than for
  eye-to-hand, since flange poses feed the solve directly.
- `persist/appclient_test.go` fails `gofmt -l` on `main`. Pre-existing;
  `make lint` fails only on this.
