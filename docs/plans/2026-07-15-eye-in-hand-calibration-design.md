# Eye-in-hand hand-eye calibration — design

**Status:** Approved, not yet implemented.

**Scope:** wrist-mounted (eye-in-hand) camera, **RGBD**, **manual pixel
picking**, **point-correspondence (Kabsch/Umeyama) solve**. Extends the shipped
eye-to-hand feature (`4372c5b`, PR #4). Fiducial/AX=XB solving remains out of
scope.

This design also **fixes a bug in shipped eye-to-hand** that it would otherwise
inherit and make far more reachable — see
[Shipped bug](#shipped-bug-the-degeneracy-guard-inspects-only-one-of-the-two-point-clouds).

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

The camera moves *in the world*, but it is bolted to the flange, so the camera
frame is fixed *relative to the flange* — and the flange is the only frame the
solve references. Every `c_i` lands in one consistent camera frame; every `q_i`
lands in one consistent flange frame. Kabsch applies.

### Verified properties

Confirmed numerically against the real solver, twice (once by the author, once
independently during spec review):

| Property | Result |
|---|---|
| Recovers a known `X` from diverse flange poses | Yes, residual ~1e-13 |
| **Pure-translation vantages (no wrist rotation) recover `X`** | **Yes**, incl. rotation |
| Flexible protocol (2 targets × 2 vantages; 4 targets × 1 vantage) | Yes, both recover `X` |
| Collinear vantages | Rejected by the existing guard |
| **Coincident camera points** | **NOT rejected — see below** |
| **Reference-side degeneracy (forgot to jog, + sensor noise)** | **NOT rejected, and reports an excellent residual — see below** |

The pure-translation result is counter-intuitive and worth stating loudly:
classic AX=XB *cannot* recover rotation without rotational motion. It does not
apply here, because deprojection yields full 3D points rather than bearings —
each view contributes 3 constraints, making this plain point-set registration.

**Do not add a rotational-diversity check.** It would reject valid data. See
[Conditioning](#conditioning-a-doc-note-not-a-check) for the real, weaker
concern.

## Shipped bug: the degeneracy guard inspects only one of the two point clouds

`frames/handeye.go:66-74` builds and checks the centered **camera** matrix only.
Two distinct failures follow. Both are live in shipped eye-to-hand today.

### Failure 1 — coincident camera points

The guard is a **relative** test:

```go
if len(csv) < 2 || csv[1] < collinearSingularEps*csv[0] {
```

For exactly-coincident camera points the centered matrix is all zeros, so
`csv = [0,0,0]` and the predicate is `0 < 1e-6*0` → `0 < 0` → **false**. It never
fires. Measured against the shipped solver, 4 coincident points: `err = nil`,
`residual_rms = 0` (a *perfect* fit), translation off by **232.6 mm**, rotation
returned as identity against a 51.54° truth.

### Failure 2 — reference-side degeneracy (the serious one)

If the **reference** points are all identical — in eye-in-hand, the operator
never jogged, so `q_i = F⁻¹ ∘ P` is the same every time — then
`w_i − wMean = 0` exactly, so `H = 0` exactly, so `R = I` **always** and
`t = wMean − camMean`. Meanwhile the camera points scatter by ordinary click and
depth noise, so the camera side looks perfectly healthy and **no camera-side
guard of any kind can see this**. Measured:

| camera noise σ | residual reported | actual translation error | rotation |
|---|---|---|---|
| 0.0 mm | 0.0000 mm | 232.6 mm | identity (truth 51.54°) |
| 0.1 mm | 0.1153 mm | 232.5 mm | identity |
| 1.0 mm | **0.9765 mm** | 232.6 mm | identity |

This is **worse than Failure 1**. A `residual_rms` of 0 is at least suspicious;
`0.98 mm` reads as an excellent sub-millimetre calibration. The operator has no
signal at all. The mechanism is deterministic, not luck.

**A camera-side `minSpreadEps` does not fix this** — an earlier draft of this
design proposed exactly that, and it would have shipped the failure untouched
while looking rigorous.

### The trap this sets for testing

`posesource.FakeCamera.Snapshot` returns a **fixed** snapshot, so every
fake-based test yields byte-identical camera points → `csv = [0,0,0]` → a
camera-side guard fires → the test goes green while the hardware failure is
completely untouched. Any test of this fix **must inject camera noise**, or it
verifies nothing. See the note on fakes in [Testing](#testing).

### Why an absolute spread floor is not enough

The obvious fix — an absolute `minSpreadEps` floor on each cloud's spread —
**does not close Failure 2**, and an earlier draft of this design claimed it did.

Exactly-zero reference spread requires the flange pose to be *bit-identical*
across vantages. That happens with `posesource.Fake` popping the same pose. It
does not happen on hardware: `motion.GetPose` runs FK over reported joints, and
encoder quantization alone (~1e-5 rad over ~500 mm ≈ 5 µm) puts a genuinely
un-jogged arm around 5e-3 mm — *above* any floor conservative enough to be safe.
So a floor catches the fake and misses the machine. That is the same trap this
document flags for `FakeCamera`, reproduced on the reference side.

Measured — operator jogs the flange by δ between vantages, σ = 1 mm camera
noise, n = 8:

| jog δ | ref spread | cam spread | **ratio** | residual | **translation error** |
|---|---|---|---|---|---|
| 0 mm | 1.4e-14 | 1.43 | 1e14 | 1.77 | **851 mm** |
| 0.01 mm | 8.7e-3 | 1.42 | 164 | 1.76 | **547 mm** |
| 0.1 mm | 8.7e-2 | 1.38 | 15.9 | 1.66 | **557 mm** |
| 1 mm | 8.7e-1 | 1.33 | 1.54 | 1.59 | **467 mm** |
| 10 mm | 8.66 | 8.33 | 0.96 | 1.70 | 22.7 mm |
| 70 mm | 60.6 | 60.2 | 0.99 | 1.70 | 3.1 mm |

**The residual is flat at ~1.6–1.8 mm across every row**, including rows that are
half a metre wrong. It carries no information about this failure. Note also that
this is *not* the same category as pure-translation conditioning (2.42° worst
case, graceful); this is a cliff.

### Fix, in `ComputeRigidTransform`

Guard **both** clouds, with two complementary checks. The centered reference
matrix is free — the existing loop at `handeye.go:49-64` already computes
`w := p.World.Sub(worldMean)` and discards it.

Define each cloud's spread as `csv[0]/√n`.

1. **Absolute floor** — reject either cloud below `minSpreadEps`. Catches the
   both-degenerate case (fake camera + no jog), where the ratio below is `0/0`.
2. **Spectrum agreement** (the real guard for Failure 2) — reject when
   `max(camSpread, refSpread) / min(camSpread, refSpread) > maxSpreadRatio`.

Check 2 is **dimensionless and scale-free**, which is why it works where a floor
cannot. Rigid transforms preserve the centered spectrum, so for genuinely rigid
data the two spreads *must* agree; the ratio degrades smoothly with the failure
instead of only touching its endpoint. It needs no tuning against physical units,
and it separates the catastrophic rows (ratio ≥ 1.54) from the usable ones
(ratio ≈ 0.96–0.99) cleanly.

`maxSpreadRatio` has a principled reading. For valid data with true spread `S`
and camera noise `σ`, the ratio runs about `√(1 + (σ/S)²)` — so it rises exactly
as the point spread approaches the noise floor, without anyone having to know
`σ`. A threshold of **1.25** rejects when spread falls below ~1.33σ. Start there
and let TDD confirm; guard the division so a zero denominator cannot produce
`NaN` (a `NaN` comparison is false, and the check would silently pass).

**1.25 does not break the shipped tests.** Measured against their actual data:
`TestComputeCameraToWorldRoundTrip` → 1.000000,
`TestComputeCameraToWorldCoplanarAccepted` → 1.000000, and
`TestComputeCameraToWorldNoisyResidual` (which injects 5 mm on one point, the
only existing test at any risk) → 1.025309. All pass with wide margin.

**On the units of check 1.** `csv[0]` is *not* mm of spread — it is
`√n × RMS spread`. A fixed threshold on raw `csv[0]` would demand *more* spread
at small `n` and *less* at large `n`, which is backwards, since small `n` is
exactly when spread matters most. `csv[0]/√n` is `n`-invariant and in true mm.
Measured, at a fixed physical spread of 1e-3 mm:

```
n=  3  csv[0]=8.16e-04   csv[0]/√n=4.71e-04
n=  4  csv[0]=1.00e-03   csv[0]/√n=5.00e-04
n= 50  csv[0]=3.54e-03   csv[0]/√n=5.00e-04
n=200  csv[0]=7.07e-03   csv[0]/√n=5.00e-04
```

At `n=4` the discrepancy is only 2×, which is why this is easy to miss.

`minSpreadEps` is in **mm**; `1e-3` is a safe starting point (confirmed not to
reject valid data across 1e-4→70 mm spreads). Because it introduces a
dimensional constant, the new `ComputeRigidTransform` doc comment must **pin the
unit contract** (inputs mm, residual mm, `minSpreadEps` mm). The old
`ComputeCameraToWorld` comment carried "residual (mm)"; the rename would
otherwise drop it.

3. **Collinearity**, as `csv[1] < collinearSingularEps*csv[0]`, on the camera
   cloud. Unchanged. Adding it on the reference cloud is near-redundant (rigid
   transforms preserve collinearity) and optional.

**Guard ordering is not load-bearing**, contrary to what a "check absolute
first" framing implies: for exactly-coincident input the relative test is
`0 < 0` → false and falls through regardless of order. Ordering only decides
*which message* the operator gets (`csv=[1e-12, 1e-20, 0]` would report
"collinear" when it is really coincident). Worth ordering deliberately, not
worth claiming as correctness.

The existing `len(csv) < 2` clause is dead (`len(csv) == min(3,n)` and `n ≥ 3`
is already enforced) and can go, but that is a tidy-up, not part of the fix.

This lands in the shared core, so eye-to-hand gets it for free — and it needs
regression tests on the **eye-to-hand** path, where the bug shipped, not only on
the new one.

## Frame conventions (load-bearing)

The RDK frame system gives each component **two** frames. For an arm named
`arm1`:

- `arm1_origin` — static, placed by the component's `frame` config. The arm
  **base**.
- `arm1` — the kinematic model, whose tip is the **end effector** (flange).

A child with `frame.parent: "arm1"` therefore attaches at the **flange**, not
the base. This is what makes `parent = <arm>` correct for a wrist camera, and it
matches the shipped TCP-teaching precedent (`tcp_component.frame.parent = arm`).

Verified: `referenceframe/frame_system.go` `createFramesFromPart` builds
`<name>_origin` (static) and `<name>` (model, child of `_origin`).

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

`Validate` rejects any other value, and requires both `arm` and `camera` to be
**non-empty** when the mount is `eye_in_hand`. The default preserves every
existing config unchanged.

`Validate` sees config strings only — it cannot detect an *unresolvable*
dependency. Dependency resolution stays warn-and-continue (the existing
pattern): if the arm does not resolve, `flangeInDest` is left **nil** and
`handeye_snapshot` errors on it with a clear message. Building `flangeInDest`
gated on the arm resolving (rather than on `cfg.Arm != ""`) is what makes the
nil-guard meaningful — a `MotionSource` holds only a name string and would
otherwise be non-nil even for an arm that doesn't exist, deferring the failure
to an opaque `GetPose` error.

### Tracker state

```go
cameraMount   string
flangeInDest  posesource.PoseSource // &MotionSource{Motion: mot, Component: cfg.Arm, DestFrame: dest}
                                    // nil if arm unresolved

targetMu      sync.Mutex
currentTarget *r3.Vector            // set by capture_handeye_target

// under the existing snapshotMu, alongside lastSnapshot:
lastFlange    spatialmath.Pose      // = flangeInDest.Capture().Pose()
```

**Lifecycle.** `currentTarget` is cleared on `solve_handeye` success and on
`clear_handeye_buffer`, so a stale target cannot silently poison the next
session. `teachTracker` embeds `resource.AlwaysRebuild`, so any config edit
drops `currentTarget`, `lastFlange`, and the buffer mid-calibration — accepted
(the operator restarts), but it must be documented in the README rather than
discovered.

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
rotation convention lives, one place the degeneracy guards live.

### The rename, and everything it touches

`HandEyePair{World, Camera}` → `PointPair{Reference, Camera}`;
`ComputeCameraToWorld` → `ComputeRigidTransform`. `Reference` is accurate in both
modes, where `World` would be a lie in one — the shipped name was already
imprecise (the value is a `destination_frame` point, not necessarily world).

This is mechanical but **not** confined to `frames/`. Every call site:

- `frames/handeye.go` — the type, the function, and a comment naming
  `TestComputeCameraToWorldRoundTrip`
- `frames/handeye_test.go` — six test *function names* embed
  `ComputeCameraToWorld`, and every literal is `HandEyePair{World: ...}`
- `store/store.go` — `handeyeBuffer []frames.HandEyePair`, `AddHandEyePair`,
  `HandEyeBuffer` (**public store API changes**)
- `store/store_test.go` — `HandEyePair{World: ...}` literals
- `models/posetracker/handeye.go`, `models/posetracker/docommand.go` (`p.World`)
- `models/posetracker/docommand_test.go` — `HandEyePair{World: ...}` literals
- `README.md` — names `ComputeCameraToWorld`

The existing `frames/handeye_test.go` assertions are what prove the extraction
was behaviour-preserving, but they **do** require edits (names and literals).
The proof is that no assertion's *expected value* changes.

**`frontend/src/lib/poseTracker.ts` is NOT on this list, deliberately.** Its
`HandEyePair` interface describes **the wire**, not the Go type, and
`HandEyePanel.svelte` reads `p.world.x`. The Go rename must not leak to the wire
(see [Wire shapes](#command-surface)); renaming the TS field would break the
shipped panel against a wire that still sends `world`. Leave it alone.

### Command surface

`handeye_snapshot`, `get_handeye_buffer`, and `clear_handeye_buffer` are shared.

| Verb | `eye_to_hand` | `eye_in_hand` |
|---|---|---|
| `capture_handeye_point` | OK | **error** — names the right verb |
| `capture_handeye_target` | **error** | OK — reads TCP pose, sets current target |
| `capture_handeye_view {u,v}` | **error** | OK — pairs click against current target |
| `solve_handeye` | camera→destFrame, parent = `destination_frame` | camera→flange, parent = `arm` |
| `get_handeye_mode` | returns config | returns config |

`get_handeye_mode` is **new and required**: the frontend has no other way to read
config (no existing verb exposes it), and both panels need it to know which one
should render. Returns:

```json
{"camera_mount": "eye_in_hand", "camera_configured": true, "arm_configured": true}
```

**Wire shapes** for the shared buffer verbs, which differ by mount:

- `eye_to_hand` — **unchanged**: `{"points": [{"world": {...}, "camera": {...}}]}`.
  Keeping the `world` key is deliberate: the shipped `HandEyePanel` and the
  README examples depend on it. The Go field rename must **not** leak to the wire.
- `eye_in_hand` — new:
  `{"points": [{"target": {x,y,z}, "flange": {x,y,z,o_x,o_y,o_z,theta}, "camera": {x,y,z}}]}`.
  Note `flange` is a **pose**, not a vector — it uses `poseToMap` (7 keys), while
  `target` and `camera` use `vecToMap` (3 keys). Dropping the orientation here
  would discard exactly the information the solve depends on.

`solve_handeye` response is unchanged in shape, but `parent` differs by mount:
`destination_frame` for eye-to-hand (as shipped, `handeye.go:133`), the
configured `arm` for eye-in-hand.

## Data flow

```
capture_handeye_target
  P = source.Capture()                 -> TCP pose in destination_frame
  store as current target (targetMu)

handeye_snapshot                       [operator has jogged to a vantage]
  snap = cameraSrc.Snapshot()          -> RGB + depth + intrinsics
  if mount == eye_in_hand:             <-- GATED; eye-to-hand has no arm
      F = flangeInDest.Capture().Pose()   -> flange in destination_frame
  cache snap (+F) under snapshotMu
  return {image, width, height}

capture_handeye_view {u, v}            [operator clicks the target in the frozen frame]
  c = Deproject(cached depth, u, v)    -> point in camera frame
  F = cached flange                    -> NOT a fresh read
  append EyeInHandObservation{Target: P, Flange: F, Camera: c}
  INVALIDATE cached snapshot           <-- one view per snapshot

  ... repeat snapshot+view per vantage, or set a new target ...

solve_handeye
  X, residual = ComputeCameraToFlange(observations)
  persist camera.frame {parent: arm, translation, orientation}
  clear buffer AND currentTarget, only on success
```

The flange read is **gated on mount**. Eye-to-hand does not require an arm
(`Validate` demands only `tcp_component`, and the README documents camera-only
operation), so an unconditional `flangeInDest.Capture()` in the shared snapshot
verb would break the shipped feature on every arm-less machine.

### The snapshot must freeze the flange pose, and be consumed

`handeye_snapshot` caches `F` **alongside** the image, under the same
`snapshotMu`. Eye-to-hand never needed this: it touches and clicks at the same
arm pose. Eye-in-hand jogs, snapshots, *then* clicks — so a flange pose read at
click time could describe a pose the pixels never came from. Same race the
frozen snapshot already solves for depth, same fix.

`capture_handeye_view` then **invalidates** the cache, so each snapshot yields at
most one observation. Without this an operator can click twice without
re-snapshotting, producing two byte-identical observations that are perfectly
self-consistent — which is exactly the coincident set that escapes the
degeneracy guard and reports `residual_rms = 0`. The invalidation and the
`minSpreadEps` guard are belt and braces for the same failure; ship both.

### Observation protocol

The buffer accepts **any mix** of targets and vantages — one target from four
vantages, four targets from one vantage each, or anything between. Each
observation is an independent constraint regardless of which target it came
from, so this falls out of the math at no cost. (Verified: both extremes recover
`X` exactly, including 4 targets × 1 vantage with the flange never moving.)

Minimum 3 observations; 4+ for a meaningful residual (3 is exactly determined).

### Conditioning: a doc note, not a check

Pure-translation vantages recover `X` exactly in the noiseless case, but are
measurably worse-conditioned under noise. Measured over 200 trials, 1 mm camera
noise, matched ~70 mm vantage spread:

| Protocol | worst rotation error | worst translation error |
|---|---|---|
| Pure translation | 2.42° | 13.0 mm |
| Rotation-diverse | 0.77° | 5.4 mm |

Both report small residuals, so **residual does not reveal this** — it is a
point-spread effect, not rank deficiency. A hard check would reject valid data
and is not warranted. The README should *recommend* varying wrist orientation
across vantages, and should not claim residual alone certifies a good
calibration.

## Error handling

All of these preserve the buffer and refuse rather than guess:

- Wrong verb for the configured mount → error naming the correct verb.
- `capture_handeye_view` with no target → "call capture_handeye_target first".
- `capture_handeye_view` with no cached snapshot (fresh, or already consumed) →
  "call handeye_snapshot first".
- `handeye_snapshot` in eye-in-hand with `flangeInDest == nil` (arm unresolved)
  → error naming the arm dependency.
- `solve_handeye` with <3 observations, collinear camera points, coincident
  points on either cloud, or camera/reference spreads that disagree beyond
  `maxSpreadRatio` (the "never jogged" case) → the core's guards. The
  spread-disagreement error must tell the operator to **vary the arm pose
  between views**, since the residual will look fine and they have no other clue.
- Persist failure → error, buffer kept, nothing committed.

Resolution-mismatch and IR-as-color guards are inherited unchanged from
`posesource`.

## Frontend

A separate `EyeInHandPanel.svelte` alongside `HandEyePanel.svelte`, each
self-contained. Each panel calls `get_handeye_mode` and renders only when the
configured mount matches, so an operator never sees a panel that can only error.
`displayToNativePixel` is reused as-is.

**No refactor of the shipped `HandEyePanel`.** Some snapshot/overlay/buffer
markup will be duplicated between the two panels. That duplication is accepted
deliberately: extracting shared components would mean restructuring working,
shipped UI code for a feature whose only two escaped defects were both
integration bugs. Revisit if a third consumer appears.

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
- Mixed targets (2 targets × 2 vantages) → recovers `X`. Validates the flexible
  protocol.
- Collinear vantages → error. **Coincident camera points → error.**
- **Reference-side degeneracy → error, WITH CAMERA NOISE INJECTED.** Identical
  `q_i` (flange never moved, target unchanged) plus σ = 1 mm of camera noise.
  The noise is not incidental: without it the camera points coincide too, and
  the test is then satisfiable by a **camera-side-only** spread fix — so it would
  not force the two-sided guard that is the entire point. With noise, the camera
  side looks healthy and only the spectrum-agreement check can catch it.
- **Near-degenerate reference → error.** δ = 0.01 mm and δ = 1 mm jogs with
  σ = 1 mm noise (rows from the measured table). These are the rows an absolute
  floor misses and the ratio catches; they are the hardware-realistic form of the
  failure. Without these, the fix is only pinned at the fake-only endpoint.
- **δ = 10 mm and δ = 70 mm jogs → accepted**, confirming the guard does not
  reject valid data.
- <3 observations → error. Noisy (but non-degenerate) input → residual > 0 and
  bounded.

`frames/handeye_test.go`:

- Existing assertions carry over under the rename; no *expected value* changes.
- **Regression tests on the eye-to-hand path** for both failures: coincident
  camera points → error, and reference-side degeneracy with camera noise →
  error. This is where the bug shipped; it must be pinned there, not only on the
  new path.

`models/posetracker`:

- Each wrong-verb/mount pairing errors.
- View without target; view without snapshot; view twice without re-snapshotting
  (second call errors — the invalidation).
- `handeye_snapshot` in **eye_to_hand with no arm configured** succeeds. This is
  the regression test for the gating; it fails if the flange read is
  unconditional.
- `Validate`: bad `camera_mount` string; `eye_in_hand` missing `arm`;
  `eye_in_hand` missing `camera`; default is `eye_to_hand`.
- `get_handeye_mode` returns the configured mount.
- **Snapshot-freezes-flange regression test.** `posesource.Fake` pops a queued
  pose per `Capture`, so queue `[F1, F2]` on `flangeInDest`, snapshot, view,
  assert the observation holds `F1`. A re-read implementation gets `F2` and
  fails. **Use a separate `Fake` for `source` and for `flangeInDest`** — one
  fake backing both shares a queue and the assertion stops meaning what it says.
- **Parent-frame regression test.** Solve persists `parent = <arm>` — the direct
  analogue of the `destination_frame` test that caught the shipped bug.

`store`: two buffers (`[]PointPair` for eye-to-hand, `[]EyeInHandObservation`
for eye-in-hand — different types, so they cannot be one). Add/read/len/clear
and copy-on-read for each.

Frontend: `EyeInHandPanel`, and mode-gated rendering for both panels.

**Note on the fakes — read before trusting a green run.** Both fakes hide
exactly the bug classes this feature keeps shipping:

- `posesource.Fake.Capture` hardcodes `referenceframe.World` as the frame name
  regardless of `DestFrame`, so no test using it can exercise frame-name
  correctness — the class of `4afb61e`.
- `posesource.FakeCamera.Snapshot` returns a **fixed** snapshot, so every
  observation deprojects to a byte-identical camera point. That is a degenerate
  configuration, and it means a fake-driven end-to-end test can go green while
  the reference-side guard is entirely absent.

A passing fake-based test is evidence the plumbing is wired, not that the
geometry is right.

### Review

A **final holistic review is mandatory**, not optional. Both bugs that escaped
eye-to-hand were integration bugs — a mislabeled parent, a missing resolution
check — and both passed per-task reviews that were individually correct. Only
the cross-cutting pass found them. The coincident-points bug documented above
was in turn missed by that review and caught only at spec review.

## Documentation

Not optional, and not covered by "out of scope":

- README: the `camera_mount` attribute; the `capture_handeye_target`,
  `capture_handeye_view`, and `get_handeye_mode` verbs; the eye-in-hand
  walkthrough; the `parent = <arm>` persistence note; the rotational-diversity
  recommendation; the AlwaysRebuild caveat.
- `models/posetracker/docommand.go` holds the canonical command doc comment —
  both new verbs must be added.
- README currently names `ComputeCameraToWorld`; update for the rename.

## Out of scope

- Fiducial / AX=XB solving.
- Automatic point detection (manual pick only).
- Live camera streaming (frozen snapshots only).
- Changing the eye-to-hand workflow (the `minSpreadEps` fix changes only which
  inputs are *rejected*, never a returned value).
- Extracting shared frontend components.

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
