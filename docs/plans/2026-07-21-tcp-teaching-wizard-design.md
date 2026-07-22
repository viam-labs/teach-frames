# TCP teaching wizard (track C) — design

**Date:** 2026-07-21
**Branch:** `redesign/visualizer-teach-pendant`
**Status:** design approved; ready for an implementation plan.

Track C of the teach-pendant roadmap (A+B → **C** → D → E → F/G). Builds on the
shipped 3D pendant and the A+B wizard template. The **backend is already shipped**
(2026-07-10 TCP-teaching design/impl) — this is a frontend port + guided-wizard
reshape, no backend change.

## What TCP teaching is (and how it differs from frame-define)

TCP teaching produces a one-shot **calibration** written to the tool component's
own `frame` config — NOT a tracked workcell frame. The operator touches one fixed
physical point with the tool tip from ≥4 arm orientations; a least-squares pivot
solve recovers the tool-tip offset in the flange frame, plus an **RMS residual**
(the "TCP error" trust signal). Orientation cannot be derived from tip touches
(geometrically impossible — see the 2026-07-10 design); it defaults to the flange
orientation and can be set explicitly.

Key differences from the Define-Frame wizard:
- Result is persisted to the **tool's `frame` config**, not to `Poses()`.
- Capture reads `arm.EndPosition` (flange pose, arm base frame) into a **dedicated
  TCP buffer**, distinct from the frame capture buffer.
- `teach_tcp_position` **solves *and* persists atomically** (like `define_frame`),
  and there is **no frontend pivot solver** — so, unlike the frame-define
  provisional triad, we cannot preview before commit. Review is *post-commit*,
  with a Re-teach loop.

## Decisions (from brainstorming, 2026-07-21)

1. **Guided wizard**, matching the A+B template (shared rune store + FloatingPanel
   + DashboardToggle) — foolproof for the mixed audience, prominent RMS trust
   signal.
2. **Minimal scene coupling** — no new scene plugin. The tool's own frame triad
   (correct since the Bug-2 SDK fix) auto-moves to the taught TCP after commit
   (`useFrames` refetches `frameSystemConfig` on config-revision change), giving a
   free before/after.
3. **Optional explicit orientation** — the wizard's core is the pivot *position*
   teach; orientation is an optional final step reusing the shipped explicit
   `teach_tcp_orientation` (OV degrees), framed "most tools can leave this at the
   flange orientation." No backend change. (The friendlier alignment-capture
   method remains a future enhancement — it needs backend work.)

## Architecture

- **Shared rune store** `frontend/src/lib/wizard/tcpTeach.svelte.ts` (pure,
  unit-tested; created once in `App.svelte`, passed to the panel).
- **FloatingPanel** `frontend/src/panels/TcpTeachWizard.svelte` + a
  `DashboardToggle` with a distinct icon, prime chrome (Tailwind tokens matching
  `FrameDefineWizard`). Drives the shipped DoCommands via `poseTracker.ts`
  builders: `captureTcpPoint`, `teachTcpPosition`, `teachTcpOrientation`,
  `clearTcpBuffer`, `getTcpBuffer`.
- **No scene plugin.** The tool frame triad is rendered by motion-tools.
- Logic source to port: legacy `frontend/src/panels/TcpPanel.svelte` (still on the
  removed `tokens.css`).

## Flow (phase model)

`setup → capturing → taught → orientation? → done`

1. **setup** — instructions ("Touch the same fixed point with the tool tip from ≥4
   different arm orientations"). **Start** clears the TCP buffer and → `capturing`.
2. **capturing** — **Capture** each touch (`capture_tcp_point`); show
   `N / ≥4 captured` and the buffered points. **Teach position** enabled at ≥4.
3. **taught** — after `teach_tcp_position` (solves + persists): show the solved
   **offset** and, prominently, the **RMS residual** (e.g. "TCP error: 0.4 mm").
   **Re-teach** (clear buffer → `capturing`) or **continue**.
   - On commit the tool's `frame` config changes → `useFrames` revision-refetch →
     the tool triad moves to the taught TCP (the before/after; no Visualizer
     remount needed — this is not the WSS path).
4. **orientation** (optional) — "Set tool orientation? Most tools can leave this at
   the flange orientation." **Skip** → `done`, or reveal explicit OV inputs
   (`oX/oY/oZ/theta`) → **Save** (`teach_tcp_orientation`). Offered only *after* a
   successful position teach — matching the backend's ordering constraint.
5. **done** — "TCP calibrated." Offer to re-teach.

## Store shape (sketch)

```
phase: 'setup' | 'capturing' | 'taught' | 'orientation' | 'done'
captureCount: number            // from capture_tcp_point response buffer_len
position: { offset, residual_rms } | undefined
error: string | undefined
get canSolve() { return captureCount >= 4 }
start(); recordCapture(bufferLen); solved(result); reteach();
toOrientation(); finish(); setError(msg); reset()
```

## Error handling (reuse shipped patterns)

- Capture re-entrancy guard (`isPending`), `armState.hasArm` + `motion.busy` gates.
- Degenerate-solve errors (<4 points, near-parallel orientations) surface via
  `wizard.setError`; the **buffer is preserved** (backend keeps it on degenerate
  input) so the operator adds more touches rather than starting over.
- Persistence disabled (missing platform creds) → `committed:false` / error,
  surfaced (mirrors `define_frame`).

## Testing

- **Store units**: phase transitions; `canSolve` (≥4) gating; `recordCapture`
  count; `reteach` reset; orientation only reachable after `taught`.
- **Panel**: light render test optional.
- No new geometry (minimal scene coupling).
- **Hardware** (the 2026-07-10 design's pending checklist, now verifiable thanks
  to Bug-2): small `residual_rms` for careful touches; the taught TCP renders at
  the correct physical location. The Bug-2 fix empirically resolves the doc's open
  "frame-parent" risk — arm-parented components render at the flange/end (confirmed
  with cam-1).

## Out of scope

Alignment-capture orientation (needs a new backend command), and the deferred
workcell-geometry TCP methods. Track C ships the guided pivot-position teach plus
optional explicit orientation.
