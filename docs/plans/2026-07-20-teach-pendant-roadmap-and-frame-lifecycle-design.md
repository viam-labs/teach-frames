# Teach pendant — remaining-work roadmap + frame-lifecycle (A+B) design

**Date:** 2026-07-20
**Branch:** `redesign/visualizer-teach-pendant`
**Status:** design approved; ready for an implementation plan (milestone 1 = A+B).

Builds directly on the shipped 3D teach-pendant vertical slice (draft PR #6) and
the handoff at `docs/plans/2026-07-20-teach-pendant-remaining-work-handoff.md`.
Read that handoff for the established architecture, conventions/gotchas, and the
reusable inventory — this doc does not repeat them.

---

## Part 1 — Sequenced roadmap (all remaining tracks)

Sequencing decision: **no external driver** (no demo/customer/deadline), so we
sequence by value/effort — complete the core teach loop first, then de-risk
incrementally toward the hardest design problem.

| Track | What | Value | Effort | Risk | Deps |
|---|---|---|---|---|---|
| **A** | Finish frame-define methods (`point`, `tcp_snapshot`) | High | Small | Low | Extends shipped 3-point wizard |
| **B** | Frame management UI (list / select / delete / clear) | High | Small-Med | Low-Med | Enables E's frame picker |
| **C** | TCP teaching (tool offset/orientation) | Medium | Medium | Low | Independent; live TCP triad exists |
| **D** | Hand-eye calibration (eye-to-hand + eye-in-hand) | Highest | Largest | High | Independent; reuses "stage solved pose as triad" |
| **E** | Jog enhancements (frame-relative jog, go-to-pose) | Medium | Medium | Med | Frame-relative **needs B** |
| **F** | Line (2-point), waypoint record/replay | Low | — | — | Design after above land |
| **G** | Non-origin arm-base correctness fix | Correctness | Medium | — | Cross-cutting; blocks general-cell claim |

**Agreed order: A+B → C → D → E → F/G.**

Hard dependency edges (everything else is independent, sequence by appetite):
- **B before E's frame-relative jog** (it needs a frame picker / scene selection).
- **G underlies every scene draw** — it's a correctness fix, not a feature, and
  becomes a blocker the moment a real target cell has a non-identity arm base.
  Pull it forward if/when general-cell support is needed; otherwise it rides
  along whenever we next touch scene-parenting.

**Milestone 1 = A + B** — the two halves of the frame lifecycle (create /
manage). Designed in detail below. C–G will each get their own design doc when
scheduled.

---

## Part 2 — Milestone 1 design: A + B

### Backend is fixed and already shipped

The three `define_frame` methods (`models/posetracker/docommand.go`, computed in
`frames/compute.go`) differ only in capture count and the committed orientation.
The frontend previews must match exactly:

| Method | Captures | Committed pose | Provisional preview |
|---|---|---|---|
| `3point` | 3 | origin = P0, +X→P1, P2 in +XY half-plane (`ComputeThreePoint`) | basis triad (shipped) |
| `point` | 1 | origin = P, **identity orientation** (`ComputePoint`) | **parent-aligned** triad at origin (identity quaternion) |
| `tcp_snapshot` | 1 | the captured pose **as-is** (`pose = buf[0]`) | triad at the captured pose's **full** position + orientation |

So A is genuinely small: both new methods are 1-capture; only the capture count,
prompt copy, and preview math change.

### A — Method-parameterized guided wizard

Decision: **one guided wizard with a method selector**, not three separate flows
(the pendant targets a mixed-skill audience; the guided step-machine is the point
of the redesign, and the three methods share almost all machinery).

**Store (`src/lib/wizard/frameDefine.svelte.ts`)** — refactor to a semantic phase
model so the scene plugin stops keying visuals off magic step numbers:

- Add `method: FrameMethod`; derive `requiredCaptures` (3 for `3point`, else 1).
- Replace numeric `step: 0..5` with `phase: 'setup' | 'capturing' | 'preview' |
  'committed'`. The capture phase ends when `captures.length === requiredCaptures`.
- `start(name, method)` sets both; `canCommit = captures.length === requiredCaptures`.
- The plugin and panel read `phase` + `method` + `captures.length` — no magic numbers.
- Re-green `frameDefine.test.ts` against the phase model (see Testing).

**Panel (`src/panels/FrameDefineWizard.svelte`)**:

- `setup` phase gains a **method selector** — 3 radios reusing the legacy
  `METHOD_LABELS` / `METHOD_HINTS` copy — alongside the name field.
- Capture prompts become method-aware:
  - `3point`: the existing 3 prompts (origin / +X / +XY-plane).
  - `point`: "Jog the tool to the frame's origin, then Capture."
  - `tcp_snapshot`: "Jog the tool to the exact pose to snapshot, then Capture."
- **Name-collision warning moves here** from the retired define-form: read
  `listFrames` in the wizard and warn in `setup` — "a frame named X exists —
  committing will replace it."

**Scene plugin (`src/scene/FrameDefinePlugin.svelte`)** — provisional previews
matched to the backend table above:

- `3point`: unchanged basis triad.
- `point`: origin marker + **identity-quaternion** triad (parent-aligned).
- `tcp_snapshot`: triad at the captured pose's full orientation, converted with
  the existing `poseTransform` (the same conversion `TcpTriad` uses).
- Markers/preview-line logic generalizes to `requiredCaptures`.

### B — Frame management panel

Decision: **port `FramePanel` as a dedicated "Manage frames" FloatingPanel +
DashboardToggle**, stripped to **list / delete / clear-all** (drop its
define-form — define now lives in the wizard; avoid two define UIs).

- Reuse the legacy panel's confirm-before-destroy logic and
  `listFrames`/`deleteFrame`/`clearFrames` wiring verbatim — they are correct and
  tested; the work is restyle-to-prime + re-home into FloatingPanel +
  DashboardToggle, following the shipped template.
- **Scene-click-to-select** (click a triad → highlight its row / delete) is a
  **deferred spike**, not a blocker: WSS entities are `removable: false` and the
  Visualizer's selection/Details hookability is unverified. Time-box a feasibility
  check during B; wire it only if cheap. Otherwise management stays panel-driven,
  revisited when E needs a scene frame-picker anyway.

### Panels & toolbar

Two toolbar entries via `DashboardToggle`: **Define frame** (existing wizard,
now multi-method) and **Manage frames** (new). Panel visibility stays local UI
state, independent of wizard progress (as shipped).

### Error handling (reuse shipped patterns)

- Capture guards carry over unchanged: `isPending` re-entrancy guard,
  `motion.busy` gate, and the `buffer_len` desync check (1-capture methods expect
  `buffer_len === 1`).
- Degeneracy errors (collinear points) exist only for `3point`;
  `point`/`tcp_snapshot` cannot be degenerate. Surface any backend error via
  `wizard.setError`.
- Manage panel: confirm-before-destroy + per-action error surfacing port as-is.

### Testing

- **Store units** (`frameDefine.test.ts`): phase transitions and `requiredCaptures`
  per method (1 vs 3 captures); `canCommit` gating; collision-warning derivation.
- **Preview mapping**: assert `point` → identity/parent-aligned triad and
  `tcp_snapshot` → captured-orientation triad (reuses already-tested
  `poseTransform`; no new non-symmetric fixture needed beyond existing).
- **Manage panel**: port preserves tested confirm logic; a light render test is
  optional.
- **Hardware pass**: the slice was hardware-verified for `3point`; give `point`
  and `tcp_snapshot` each one jog→capture→commit→live-render run.

### Out of scope for milestone 1

C (TCP teaching), D (hand-eye), E (jog enhancements), F, and G (non-origin
arm-base fix). Each gets its own design when scheduled.
