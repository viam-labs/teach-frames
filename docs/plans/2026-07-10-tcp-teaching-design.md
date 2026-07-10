# TCP Teaching — Design

**Date:** 2026-07-10
**Status:** Approved design, pending implementation plan

## Purpose

Extend the `teach-frames` module so an operator can teach an arbitrary tool's
TCP (tool center point) by touching a reference point from multiple arm
orientations — the calibration a Universal Robots teach pendant performs for a
new tool. This complements the existing workcell-frame teaching: where that
feature records *tracked* frames in space, TCP teaching produces a one-shot
*calibration* that is written to the tool component's own `frame` config.

Two capabilities, prioritized in this order:

1. **TCP position** — 4-point pivot method: touch the same fixed world point
   from ≥4 arm orientations and least-squares solve for the tool tip offset.
2. **TCP orientation** — defaults to the flange orientation (identity relative
   to the flange); optionally set explicitly by passing a known tool rotation
   (from CAD/datasheet) in the teach command.

Orientation is optional: if the tool axis already matches the arm flange
orientation, position-only is a complete calibration.

> **Note on orientation method (revised during planning).** An earlier draft
> proposed a "2-point axis extension" — touching two more points with the same
> tip to fix the tool axes. That is geometrically impossible: the tip is a
> single point on a rigid tool, so once the pivot solves the tip offset `t`, the
> tip's world position at any capture is fully determined by `t` and the flange
> pose (`R_i·t + p_i`) and carries **zero** additional orientation information.
> Teaching orientation requires either an external reference or a second tool
> feature. This phase therefore ships orientation as **identity-default plus
> explicit input**. An **alignment-capture** method (physically align the tool
> to a known reference direction and capture the flange orientation once,
> `R_tool_in_flange = R_flangeᵀ · R_reference`) is deferred to the app phase,
> where physical alignment gets real visual feedback.

## Scope

**In scope:** the two capabilities above, driven via `DoCommand`, persisted to
the tool component's `frame` config.

**Deferred (later phases):** additional workcell-geometry methods
(plane/line/bore-center), the visual teach-pendant application, and
freedrive / force-torque features. These are explicitly out of scope here.

## Key decisions

These were settled during brainstorming:

- **Single control surface.** TCP teaching lives in the existing `pose-tracker`
  component rather than a dedicated resource. The module's real interface is
  already `DoCommand`, so a second resource would buy little cleanliness while
  costing the future app a second connection and a split vocabulary. The arm
  dependency added here is also what later unlocks jogging and live pose readout
  for the app, making `pose-tracker` the one "teach controller" the app drives.
- **TCP results are not tracked frames.** A taught TCP never enters the
  `frames` set or `Poses()` output. It is a calibration written to the tool
  component's config.
- **Distinct `arm` dependency.** A new `arm` attribute, separate from
  `tcp_component`, so the component that *moves* (arm) is decoupled from the
  component whose tip is *taught* (tool/gripper). The arm is the source of the
  flange pose the pivot math requires.
- **Orientation via identity-default plus explicit input.** The axis-point
  method was found geometrically impossible during planning (see the note under
  Purpose); orientation defaults to identity and can be set explicitly.
- **Two-stage teach.** Position and orientation are separate commands so a tip
  can be taught and persisted independently of orientation.

## Config changes

New **optional** `arm` attribute on the `pose-tracker` component, declared as a
real arm resource dependency so the RDK wires it up before construction.

When `arm` is absent, TCP teaching commands return a clear
"arm dependency not configured" error — the same graceful-degradation pattern as
the existing persistence-disabled path. Existing workcell-frame teaching is
unaffected.

## Command vocabulary

TCP capture uses a **different data source** than the existing `capture_point`:
where `capture_point` records the world TCP pose via the motion service, TCP
teaching records the **arm flange pose** via `arm.EndPosition` (expressed in the
arm base frame). It therefore uses its own capture command and a **dedicated TCP
buffer**, separate from the frame capture buffer.

All commands are sent to the `pose-tracker` component.

| Command | Payload | Effect | Response keys |
|---|---|---|---|
| `capture_tcp_point` | `{}` | Read `arm.EndPosition` and append to the TCP buffer. | `index`, `buffer_len`, `pose` |
| `teach_tcp_position` | `{}` | Least-squares pivot solve over buffered points → tip offset + RMS fit residual; persist the translation to `tcp_component.frame`; clear the buffer. | `offset`, `residual_rms`, `committed` |
| `teach_tcp_orientation` | `{"o_x","o_y","o_z","theta"}` | Persist an explicit tool orientation (orientation-vector degrees) to `tcp_component.frame`, leaving the taught translation intact. Omitting this command leaves orientation at identity. | `orientation`, `committed` |
| `get_tcp_buffer` | `{}` | Return all poses in the TCP buffer. | `points` |
| `clear_tcp_buffer` | `{}` | Empty the TCP buffer. | `cleared` |

`teach_tcp_position` **reports the RMS fit residual** — the "TCP error" figure a
real pendant shows, the trust signal that the touches consistently hit one point
— and **persists on success** (mirroring how `define_frame` computes-and-commits),
echoing the solved offset so the caller can see exactly what was written before
trusting it. `teach_tcp_orientation` takes an explicit rotation rather than
capturing points, because tool orientation cannot be derived from tip touches
(see the note under Purpose). Because it writes only the rotation, it must be
run *after* `teach_tcp_position` (or alongside a previously-taught tip), never on
an untaught tool.

## Computation

New solver in the `frames/` package alongside `compute.go`:

- `ComputePivotTCP(poses []spatialmath.Pose) (offset r3.Vector, residualRMS float64, err error)` —
  builds the linear system where each capture contributes
  `[R_i | -I]·[t; c] = -p_i` (unknowns: tool tip `t` in the flange frame and the
  common touched point `c` in the arm base frame), solves it in the
  least-squares sense, and returns `t` plus the RMS residual of the fit. Requires
  `≥4` captures and guards against degenerate input (too few points,
  near-parallel orientations that leave the system rank-deficient).

Orientation needs no solver: it is either identity (default) or an explicit
rotation supplied by the operator and persisted verbatim.

Tested to the same standard as `compute_test.go`: known-geometry fixtures with an
analytically known tip (e.g. synthesize flange poses that all place a chosen tip
at a fixed base point, assert the solver recovers it with ~0 residual), plus
degenerate-input guards.

## Persistence

Generalize the `persist` package from "patch my own `attributes.frames`" to also
"patch component **X**'s `frame` (translation + orientation) by name," using the
same `GetRobotPart` / `UpdateRobotPart` read-modify-write and the same
platform-API credentials. TCP teaching writes to `tcp_component.frame`; existing
frame persistence is unchanged.

## Open risks to resolve during planning

- **Frame-parent semantics.** The solved offset is in the *flange* frame, but a
  Viam component `frame` is expressed relative to its `parent`. We must confirm
  how the frame system composes a tool frame parented to an arm (attaches at the
  end effector vs. the base) and transform the offset accordingly before
  writing. This affects the correctness of what gets persisted.
- **`EndPosition` and existing tool offset.** If `tcp_component` already carries
  a (stale or zero) frame offset, we must determine whether `arm.EndPosition`
  already folds it in, so we solve the *fresh* flange pose and do not
  double-count an existing offset.

## Failure modes

- **No `arm` configured** → TCP commands error clearly; frame teaching works.
- **Persistence disabled** (missing platform-API creds) → teach commands compute
  and echo the result but report `committed: false` / error on write, matching
  the existing `define_frame` behavior.
- **Degenerate touches** (too few points, near-parallel orientations, collinear
  axis points) → solver returns a descriptive error; buffer is preserved so the
  operator can add more points rather than start over.

## Integration verification (pending, drafted 2026-07-10)

Implementation, `go vet`, and the full unit test suite are complete and green
(see the Task 9 build/test run). The steps below require a real arm + tool
component and live cloud credentials, which are not available in this
environment. They are recorded here as a **pending** checklist for a human
operator to run and check off; they directly resolve the two open risks above
("Frame-parent semantics" and "`EndPosition` and existing tool offset").

- [ ] **1. Baseline pivot teach.** Configure the tool as `tcp_component`, the
      arm as `arm`. Jog the tip to a fixed point from ≥4 varied orientations,
      `capture_tcp_point` each, then `teach_tcp_position`. Confirm
      `residual_rms` is small (e.g. < 1–2 mm for careful touches).
- [ ] **2. Frame-parent semantics (open risk, narrowed).** The persist layer
      now FORCES `frame.parent = arm` on every TCP write (`setComponentFrame`
      in `persist/appclient.go` treats a non-empty provided parent as
      authoritative and overrides any existing parent) — so a tool with a
      pre-existing non-arm parent (e.g. a stale "world" default) can no longer
      silently corrupt the result by keeping the wrong parent. The parent
      value itself is therefore no longer in question. What remains open and
      still requires hardware verification is purely the **flange-vs-base
      geometry question**: after teaching, inspect the tool component's
      `frame` in the app config and verify in the visualizer that the tool tip
      now renders at the correct physical location. If Viam's frame system
      attaches an arm-parented component at the arm *base* instead of the
      *flange*/end-effector, the solved offset needs transforming before
      persistence — record the correction and open a follow-up.
- [ ] **3. Existing tool offset (open risk).** Repeat teaching on a tool that
      already has a non-zero `frame`. Confirm `arm.EndPosition` returns the
      flange pose *without* folding in the tool offset (i.e. re-teaching
      converges to the same physical tip, not a doubled offset). If it
      double-counts, document that the tool `frame` must be cleared before
      re-teaching.
- [ ] **4. Orientation-only write.** `teach_tcp_orientation` with a known
      rotation; confirm it writes `frame.orientation` and leaves `translation`
      unchanged.

Results (pass/fail, measured residuals, any corrections needed) should be
recorded under this heading once the checklist is run.
