# Visualizer-based teach pendant — design

**Date:** 2026-07-20
**Status:** Approved (brainstorm), ready for implementation plan
**Scope:** Frontend-only redesign. The Go backend (pose-tracker DoCommands,
`world_state_store`, frame math) is untouched.

## Goal

Rebuild the `teach-frames` frontend around the Visualizer from
`@viamrobotics/motion-tools`, so the teach pendant is a 3D view of the live
robot and workspace with floating panels layered over it — a UR-style pendant
where multi-step calibration procedures are *staged in the scene*.

This document covers a **vertical slice**: prove the architecture end-to-end on
the highest-value path (embed + live arm, Jog, and the 3-point Frame-Define
wizard), and establish a template the other procedures copy later.

## Decisions (from brainstorm)

- **Architecture: Hybrid.** `<Visualizer>` hosts the 3D scene and its native
  strengths (resource tree, live arm from the frame system, camera, and — via
  our existing `world_state_store` service — taught-frame triads). Scene-coupled
  interactions become plugins; procedural wizards become floating panels layered
  in. Chosen per-feature.
- **Primary win: Guided procedures.** The scene is a stage for step-by-step
  wizards; it shows where to go next and what's been captured. This serves the
  mixed audience in `PRODUCT.md` (an operator should follow the common path
  without a manual).
- **Prototype scope: Vertical slice.** Embed Visualizer + live arm, port Jog and
  ONE guided wizard (3-point frame define), persist a frame, see its triad.
  Defer the other methods/wizards until the pattern is proven.
- **Visual language: adopt the Visualizer's chrome.** Use `prime-core` and the
  Visualizer's portals (`FloatingPanel`, `DashboardPortal`) so the whole surface
  reads as one native Viam tool. This also dissolves the two-dark-palette
  reconciliation problem — we stop owning a separate theme (`tokens.css`).

## Key insight: the module already speaks the Visualizer's protocol

`meta.json` ships a `world_state_store` service
(`viam-labs:teach-frames:world-state-store`) whose job is "mirror the
pose-tracker's taught frames into the Viam visualizer as axes triads." That is
the exact RDK API the Visualizer consumes. So:

- The **live arm** renders natively from the machine's frame system.
- The **taught frames** render natively as triads via `world_state_store`.
- When `define_frame` persists a frame into config, `world_state_store` surfaces
  it and the Visualizer redraws the triad — a closed loop with **no new frontend
  rendering code** for persisted frames.

The redesign is therefore a *shell swap*, not a data-plumbing rebuild.

## Architecture

Three layers:

1. **Scene (native Visualizer).** Live arm, grid, camera, taught-frame triads,
   resource tree. We configure it; we don't rebuild it.
2. **Scene-coupled plugins.** Svelte components mounted inside `<Visualizer>`
   that draw procedure-specific 3D affordances (captured-point markers, axis/
   plane previews, the provisional frame triad, the live TCP triad).
3. **Wizard/control panels.** Step-by-step forms (Jog, 3-point wizard) docked via
   `FloatingPanel` / `DashboardPortal`, styled with `prime-core`.

**Data flow is unchanged.** All robot actions still go through the pose-tracker
`DoCommand` API (`jog_cartesian`, `capture_point`, `define_frame`, …). The Go
backend is untouched. This is why the redesign is frontend-only.

### Connection — what we need and (deliberately) don't

Confirmed against `@viamrobotics/motion-tools`:

- `<Visualizer>` does **not** dial. It consumes `@viamrobotics/svelte-sdk`
  context. Minimum for a live connection = wrap it in `ViamProvider` with a
  `dialConfigs: { [partID]: DialConf }` entry. We **already** build that
  `DialConf` in `lib/machine.ts` and already use `ViamProvider` today, keyed by
  `machine.id`. So: `<Visualizer partID={machine.id}>` inside our existing
  provider. No new auth.
- We **skip** the Visualizer's config-editing seam entirely (`usePartConfig`,
  `ViamAppProvider`, API-key creds, `localConfigProps`). Our `define_frame`
  DoCommand persists frames server-side and `world_state_store` surfaces them.
  We never write config through the Visualizer.
- We **stay off** the Visualizer's `interactionMode` union
  (`navigate|measure|select|gizmo` — a fixed type; a new mode would fork the
  package). We don't need it: capture is *physical* — jog the real TCP to the
  real reference point and hit Capture. The scene is a verification/legibility
  surface, not a click-input surface.

### Mount tree

`FloatingPanel` and the portals render into `PortalTarget`s that live *inside*
`<Visualizer>`. So our panels and our scene objects both mount in the `children`
snippet — the MeasureTool pattern (one plugin renders a 3D mesh *and* a
`DashboardPortal` button). `ViamProvider` sits above; DoCommand context flows
down.

```
<ViamProvider dialConfigs={{ [machine.id]: dialConf }}>   ← reused from machine.ts
  {#if !selectedResource}  <ResourcePicker/>              ← reused; picks the pose_tracker
  {:else}
    <Visualizer partID={machine.id}>
      {#snippet children()}
        <TcpTriad/>              ← live TCP, our get_arm_state poll
        <FrameDefinePlugin/>     ← markers / axis+plane previews / provisional triad (3D)
        <JogPanel/>              ← FloatingPanel + DashboardPortal (HTML via portal)
        <FrameDefineWizard/>     ← FloatingPanel; drives steps, calls DoCommands
      {/snippet}
    </Visualizer>
  {/if}
</ViamProvider>
```

## The 3-point Frame-Define wizard, staged in the scene

**Procedure (unchanged backend).** 3-point defines a frame from three captured
TCP poses: **P0 = origin**, **P1 = a point on +X**, **P2 = a point in the +XY
plane** (fixes +Y; Z = X×Y). Each capture is `capture_point` (server appends the
live TCP pose); the wizard reads back with `get_buffer`, resets with
`clear_buffer`, commits with `define_frame(name, '3point')`.

**Split.** One `FloatingPanel` ("Define Frame") drives the steps; one in-scene
plugin (`FrameDefinePlugin`) draws the spatial story. They share a small
`$state` wizard store (captured poses, step index, frame name, error).

| Step | Panel shows | Scene shows |
|------|-------------|-------------|
| **0 · Name & start** | Frame name field, method = 3-point, "Start". Live TCP pose readout (`get_arm_state` poll — backend truth). | Live **TCP triad** tracking the real arm. Grid + existing taught-frame triads (native `world_state_store`). |
| **1 · Capture origin (P0)** | "Jog the tool to the frame's origin, then Capture." Capture button (reuses jog + `capture_point`). | TCP triad live. **Ghost marker** pulses at current TCP to preview where P0 lands. |
| **2 · Capture +X (P1)** | "Jog along the intended +X, then Capture." | P0 a solid **numbered marker**. **Dashed line P0→live-TCP** previews the emerging X axis. |
| **3 · Capture +XY (P2)** | "Jog to a point in the +Y half-plane, then Capture." | P0, P1 solid + solid X segment. **Translucent plane/wedge** through P0–P1 previews where P2 sets +Y. |
| **4 · Preview & commit** | Computed pose readout; "Looks right?" **Commit** / **Re-capture**. Warns on degenerate input (collinear → axis undefined). | **Provisional frame triad** (`<AxesHelper>`) at the computed pose, next to the arm — the verification moment. |
| **5 · Committed** | "Frame *name* saved." → new frame / done. | Provisional triad becomes the **real** triad (now from `world_state_store`); markers fade out. |

**Why this is the guided-procedures win:** the scene tells the operator what a
good next capture looks like (ghost, dashed axis, plane), and the provisional
triad lets them *see* a bad frame (flipped axis, collinear capture) before it's
persisted. Honors "honest, live state" and "never imply a captured point that
didn't happen."

**Rendering, concretely (confirmed APIs):**

- Markers: plain Threlte `<T.Mesh raycast={() => null}>` (MeasureTool pattern) at
  each captured `{x,y,z}`.
- Axis/plane previews: `MeshLineGeometry` segment + a translucent `<T.Mesh>`
  plane, updated from the live TCP.
- Live TCP + provisional triads: the exported `<AxesHelper position rotation>`
  (from the `/lib` subpath).
- Degeneracy guard is **already** in the Go backend (`ComputeThreePoint`); the
  panel surfaces its error — no new math.
- Live TCP is read from our own `get_arm_state` DoCommand poll (backend truth,
  the same source the readout uses) — not the Visualizer's arm hooks, so panel
  and scene never disagree.

## File inventory

- **Reused as-is (backend-facing, zero change):** `lib/machine.ts`,
  `lib/clients.ts`, `lib/poseTracker.ts` (+ `poseTracker.test.ts`),
  `lib/armState.svelte.ts`, `lib/captureBuffer.svelte.ts`,
  `lib/resource.svelte.ts`, `lib/motion.svelte.ts`. The entire DoCommand surface
  survives — this is why it's frontend-only.
- **Reworked:** `App.svelte` → the provider/host shell above (loses its
  two-column layout and its `tokens.css` styling role). `JogPanel.svelte` → same
  controls/logic, re-homed into a `FloatingPanel` + a `DashboardPortal` toggle,
  restyled to prime-core.
- **New:** `lib/wizard/frameDefine.svelte.ts` (wizard `$state`: step, captured
  poses, name, error); `scene/TcpTriad.svelte`; `scene/FrameDefinePlugin.svelte`;
  `panels/FrameDefineWizard.svelte` (absorbs the relevant bits of today's
  `CapturePanel` + `FramePanel`).
- **Deferred (files stay on disk, not mounted):** `TcpPanel`, `HandEyePanel`,
  `EyeInHandPanel`, and the non-3-point paths of `CapturePanel` / `FramePanel`.
  Nothing deleted; the slice just doesn't wire them.

## Build / bundle

Current build is plain Vite + `vite-plugin-svelte`, Svelte 5, `base: './'`,
static `dist/index.html` — already runes-compatible, so motion-tools drops in.

Steps: add `@viamrobotics/motion-tools` + its peers; add `.npmrc`
`auto-install-peers=true` (docs' recommended path; keeps us on npm); import the
package CSS; no SSR concerns (we're a pure SPA, not SvelteKit).

**Top build risk (the reason the slice exists):** motion-tools ships web-worker
assets and pulls three.js/Threlte (large). Two things to prove in the slice:

1. Vite resolves the package's prebuilt workers under `base: './'` relative
   paths when served from the module.
2. Bundle size is acceptable over the Application delivery path.

If workers fight `base: './'`, the fallback is a Vite `worker` / `assetsInclude`
config tweak — a known-shape problem, not an unknown. `prime-core` CSS/theme and
tweakpane are styling that follows once the package loads.

## Verification

- **Pure logic — unit tests (existing pattern).** `poseTracker.ts` builders/
  parsers already have `poseTracker.test.ts`. The new wizard `$state` store is
  pure — step transitions, "3 captures ⇒ ready to commit," degeneracy-error
  surfacing — and gets the same vitest treatment.
- **Backend math — already covered.** `ComputeThreePoint` + degeneracy guard are
  Go-side and unit-tested; the frontend only displays results.
- **Integration — one manual `verify` pass against a machine.** The visual
  claims (TCP triad tracks the arm; markers land where captured; provisional
  triad matches the committed `world_state_store` triad; workers load; bundle
  serves) are checked by driving the real flow once — jog, capture ×3, commit,
  confirm the triad. This is also the go/no-go moment for the two build risks.

## Deferred boundary (not in this slice)

single-point & TCP-snapshot frame methods; eye-to-hand and eye-in-hand wizards;
TCP teaching; frame-relative jogging and the other roadmap items. Their `lib/`
DoCommand code already exists and is untouched, so porting each later is "add a
wizard panel + a scene plugin following the 3-point template." The slice
deliberately establishes that template once.

## Definition of done (prototype)

An operator can open the pendant, see the live arm and existing frames in 3D, jog
the TCP, capture three points with the scene guiding each, preview the
provisional frame, commit it, and watch it persist as a real triad — with the
wizard state machine unit-tested and one end-to-end `verify` pass recorded.
