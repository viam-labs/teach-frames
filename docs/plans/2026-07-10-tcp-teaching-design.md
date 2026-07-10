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
2. **TCP orientation** — 2-point axis extension: after the tip is known, touch
   two more points that fix the tool's +Z (approach) axis and +X direction.

Orientation is optional: if the tool axis already matches the arm flange
orientation, position-only is a complete calibration.

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
- **Axis-point orientation.** Reuses the cross-product / orthonormalization
  path already in `frames/compute.go`, expressed in flange coordinates, rather
  than an alignment-capture method that would depend on the (deferred) app.
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
| `teach_tcp_orientation` | `{}` | Solve orientation from the 2 axis points (origin = tip, +Z = point 1, +X from point 2 in the +XZ half-plane); persist the rotation to `tcp_component.frame`. | `orientation`, `committed` |
| `get_tcp_buffer` | `{}` | Return all poses in the TCP buffer. | `points` |
| `clear_tcp_buffer` | `{}` | Empty the TCP buffer. | `cleared` |

Both teach commands **report the fit residual / solved values** and **persist on
success** (mirroring how `define_frame` computes-and-commits), echoing the solved
pose so the caller can see exactly what was written before trusting it. The RMS
residual on `teach_tcp_position` is the "TCP error" figure a real pendant shows —
the trust signal that the touches consistently hit one point.

## Computation

New solver in the `frames/` package alongside `compute.go`:

- `ComputePivotTCP(poses []spatialmath.Pose) (offset r3.Vector, residualRMS float64, err error)` —
  stacks the pivot constraints (each pair `(R_i − R_j)·t = p_j − p_i`), solves
  the least-squares system for the tool tip offset `t` in the flange frame, and
  returns the RMS residual of the fit. Guards against degenerate input (too few
  points, near-parallel orientations).
- An orientation helper that reuses the existing cross-product /
  orthonormalization path from `compute.go`, applied to the two axis-point
  displacement vectors in flange coordinates.

Tested to the same standard as `compute_test.go`: known-geometry fixtures with
an analytically known tip/orientation, plus degenerate / collinear guards.

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
