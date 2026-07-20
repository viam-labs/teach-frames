# Teach pendant — remaining work handoff (for brainstorming)

**Date:** 2026-07-20
**Purpose:** enough context for a fresh session to brainstorm the work remaining
after the Visualizer-based 3D teach-pendant vertical slice shipped.

## Where we are

The 3D teach pendant vertical slice is **built, hardware-verified, and up as
draft PR #6** on branch `redesign/visualizer-teach-pendant`. It delivers the full
loop for the **3-point** frame method: jog → capture ×3 → commit → the frame
renders **live** and persists.

Read first:
- Design + slice outcome: `docs/plans/2026-07-20-visualizer-teach-pendant-design.md`
  (the "Slice outcome" and "Deferred boundary" sections especially).
- Implementation plan: `docs/plans/2026-07-20-visualizer-teach-pendant.md`.
- Backend contracts for the deferred features already exist — see the
  per-feature design/impl docs: `2026-07-10-tcp-teaching-*`,
  `2026-07-12-hand-eye-calibration*`, `2026-07-15-eye-in-hand-calibration*`.

## The established architecture (build on this, don't reinvent)

Hybrid: `<Visualizer partID localConfigProps componentNameToFragmentInfo>` hosts
the 3D scene; our contribution mounts in its `children` snippet. Everything drives
the **unchanged** pose-tracker `DoCommand` API.

**The porting template** (each deferred feature follows it):
1. A **shared rune store** in `src/lib/wizard/` for multi-step flows (see
   `frameDefine.svelte.ts` — pure, unit-tested; created once in `App.svelte`,
   passed as a prop to the panel + any scene plugin).
2. A **FloatingPanel** in `src/panels/` that drives the DoCommands and feeds the
   store, styled in the prime-core chrome (Tailwind tokens: `bg-white`,
   `border-medium/light`, `text-gray-9`, `text-subtle-1/2`, `text-danger-dark`,
   the dark-button recipe; `min-h-11`; `focus-visible:ring-2`; `role="alert"` /
   `aria-live` on status/error). Add a `DashboardToggle` (in `src/panels/`) so it
   opens/closes from the native toolbar.
3. An optional **scene plugin** in `src/scene/` reading the shared store +
   `armState`, drawing Threlte objects: markers, previews, provisional triads.

**Conventions & gotchas already discovered (reuse the answers):**
- Positions are **millimeters**; the scene is **meters** (`* 0.001`). `theta` is
  **degrees**; orientation is a Viam **OrientationVector** (NOT axis-angle) —
  convert with `OrientationVector` from `@viamrobotics/motion-tools/lib`
  (`ov.set(oX,oY,oZ, degToRad(theta)); ov.toQuaternion(q)`), mirroring the
  Visualizer's `poseToMatrix`. See `src/scene/poseTransform.ts`.
- Scene overlays: `<AxesHelper …/>` from `.../lib` with `depthTest={false}` (else
  occluded by the arm mesh) and a wider `width`; non-interactive markers use
  `raycast={() => null}`.
- Frontend geometry must match the Go backend convention (e.g.
  `frames/compute.go`); pin it with an exact-value unit test (the identity case
  guards nothing — use a non-symmetric fixture).
- **One `get_arm_state` poll owner** (`JogPanel`); scene components are pure
  `armState` readers. If you add another always-mounted panel, don't add a second
  poll — read `armState`.
- **Capture is physical**: the operator jogs real metal then hits Capture; source
  the captured pose from the DoCommand **response** (e.g. `CaptureResponse.pose`),
  not the polled `armState` (avoids drift + desync). Guard capture handlers on
  `isPending` (re-entrancy) and `motion.busy` (don't capture mid-jog).
- **Cookie-only connection**: never add `ViamAppProvider`/app keys. The only two
  Visualizer app-cloud seams (`usePartConfig`, `useFragmentInfo`) are already
  routed embedded.
- **Live frame render** works via the `world_state_store` poll-diff stream
  (`models/worldstatestore/`); the pose-tracker's in-memory store updates on
  `define_frame` before persist, so committed frames appear within ~1s.
- Toolchain: Tailwind v4 + vite-plugin-glsl + Vite 8; `npm run check` /
  `test` / `build` in `frontend/`; `go build/test ./...` for the module.

## Reusable inventory (most work is a PORT, not new build)

**DoCommand builders already in `src/lib/poseTracker.ts`** (backend is shipped &
untouched): jog, capture/buffer, `defineFrame` (3point|point|tcp_snapshot),
`listFrames`/`deleteFrame`/`clearFrames`, TCP teaching
(`captureTcpPoint`/`teachTcpPosition`/`teachTcpOrientation`), eye-to-hand
(`handeyeSnapshot`/`captureHandeyePoint`/`solveHandeye`/…), eye-in-hand
(`captureHandeyeTarget`/`captureHandeyeView`/`getHandeyeMode`). Plus
`displayToNativePixel` for image-click → pixel mapping.

**Legacy panels on disk, unmounted, still using the removed `tokens.css`** (they
have the working DoCommand logic — port them, don't rewrite):
`CapturePanel` (181), `FramePanel` (413), `TcpPanel` (305), `HandEyePanel` (186),
`EyeInHandPanel` (279), `StatusBar` (146). Each needs: restyle to prime, re-home
into FloatingPanel + DashboardToggle, and a decision on what to scene-couple.

## Remaining work to brainstorm

Grouped roughly by value/effort. Each notes what exists to reuse + the open
design questions.

### A. Finish the frame-define methods (small — extends the shipped wizard)
`single-point` and `tcp_snapshot` are the other two `define_frame` methods. The
3-point wizard/store/plugin already exist; these need 1 capture instead of 3.
**Open Qs:** one wizard with a method selector vs. separate flows? What's the
provisional-triad preview for a single point (origin only — no orientation from
one point; tcp_snapshot uses the captured pose's own orientation)?

### B. Frame management UI (small-medium — no UI exists yet in the new shell)
`FramePanel` (list/rename/delete/clear taught frames, with confirm steps) is
unmounted. The live scene shows frames as triads but there's no way to
select/delete one from the new UI. **Open Qs:** port `FramePanel` as a panel, or
make frames **selectable in the 3D scene** (click a triad → details/delete via the
Visualizer's Details portal)? The Visualizer has a selection/entity system;
world_state_store entities are marked `removable: false` — check whether
scene-driven delete is feasible or stays panel-driven.

### C. TCP teaching (medium — port + decide scene role)
`TcpPanel` + `teach_tcp_position`/`teach_tcp_orientation` exist. Teaches the tool
offset/orientation. **Open Qs:** how does the live TCP triad interact with an
un-taught vs taught TCP? Show a before/after preview of the tool frame in-scene?

### D. Hand-eye calibration wizards (largest — the interesting design problem)
Both eye-to-hand (`HandEyePanel`) and eye-in-hand (`EyeInHandPanel`) exist as
legacy panels with a **2D camera image + click-to-pixel** interaction
(`handeye_snapshot` returns an image; the operator clicks the target in the image;
`displayToNativePixel` maps to `(u,v)`; `capture_handeye_point`/`_view`). **Open
Qs (brainstorm-worthy):**
- The old flow is fundamentally 2D (click on a camera image). How does that live
  in a 3D-scene-centric pendant? A floating 2D image panel alongside the 3D view?
  Can the Visualizer show the camera stream in-scene / as a frustum overlay?
- Staging: show captured correspondences and the solved camera pose as a triad in
  the 3D scene (verification, like the frame provisional triad)?
- eye-in-hand vs eye-to-hand share a protocol shape but differ (`get_handeye_mode`
  tells which) — one adaptive wizard or two?
- Guided-procedure framing: these are the hardest procedures for the mixed
  audience — the biggest payoff for "make it foolproof."

### E. Jogging enhancements (medium)
Deferred roadmap items: **frame-relative jogging** (jog along a taught frame's
axes, not just world/tool); **editable move-to fields** (absolute go-to-pose /
go-to-joints with a confirm step). **Open Qs:** frame-relative jog needs a
frame-picker (tie to B's scene selection?); go-to-pose needs a safe confirm UX
(this commands real motion to an absolute target).

### F. Smaller roadmap items
2-point **Line** feature; **waypoint record/replay**. Lower priority; design when
the above land.

### G. Known limitations to design around (not features, but constraints)
- **Non-origin arm base:** our scene-root draws (TCP triad, markers, provisional
  triad) assume the arm base ≈ world origin. For cells with a non-identity arm
  base they'd diverge from the Visualizer's frame-system draws. Fix = parent our
  scene objects under the arm's base frame entity (ECS). Worth resolving before
  claiming general-cell support.
- **Repeated-commit reconfigure:** hardware-verified working, but the live-render
  path leans on the WSS stream surviving the `AlwaysRebuild` reconfigure. If it
  ever regresses for rapid repeated commits, the fallback is rendering taught
  frames ourselves from `list_frames`. (Documented in the design doc + roadmap
  memory.)
- **Cleanup debt:** legacy panels still import `tokens.css`; a bundle
  code-splitting / HDR-trim pass is deferred.

## Suggested first brainstorming question

Which track first? **A+B** (finish frame methods + frame management) is the
natural completion of the core teach loop and is low-risk. **D** (hand-eye) is the
highest-value + most interesting design problem (2D-image interaction inside a 3D
pendant). Recommend deciding A+B vs D as the opening move.

## Scene-select feasibility spike (2026-07-20)

**Question:** can a taught-frame triad be made clickable in the 3D scene so
selecting it drives a Delete action (via the Visualizer's Details panel) or at
least highlights the matching `ManageFramesPanel` row? Time-boxed
investigation only — no code changed.

### What I found, with citations into `@viamrobotics/motion-tools` 1.36.2
(paths relative to `frontend/node_modules/@viamrobotics/motion-tools/dist/`)

**Selection is a first-class, exported ECS concept, not gated by `removable`.**
`ecs/traits.js:196` defines `traits.Selected` (a plain marker trait). Clicking a
rendered *object* toggles it via the shared `onclick` handler in
`components/Entities/hooks/useEntityEvents.svelte.js` (used by `Mesh.svelte`,
`Boxes.svelte`, `Spheres.svelte`, `Capsules.svelte`, `GLTF.svelte`,
`Points.svelte`, `Line.svelte`, `Arrows/Arrows.svelte` — confirmed by grep, this
is the exhaustive list of renderers wired to it). `components/App.svelte:31`
(`const selected = useQuery(traits.Selected)`) and `:156-162`
(`{#each selected.current as entity, index}<Details {entity} {details} .../>`)
show selection driving the Details panel for **any** entity that carries
`traits.Selected`, regardless of how it got added.

**But taught-frame triads are not wired to that click handler at all.**
`hooks/useWorldState.svelte.js`'s `spawnEntity` calls
`drawTransform(world, transform, traits.WorldStateStoreAPI, { removable: false })`
(`draw.js:38`), which — for a geometry-less transform — adds `traits.ReferenceFrame`
+ (if the transform's metadata says so) `traits.ShowAxesHelper`. The actual
rendering of all axes-helper entities happens in one shared object:
`components/Entities/AxesHelpers.svelte` batches every `ShowAxesHelper` entity
into a single `BatchedAxesHelpers` (`three/BatchedAxesHelper.js`, a
`LineSegments2`) and mounts it once as `<T is={axesHelpers} />` with **no**
`onclick`/`onpointer*` props and no use of `useEntityEvents`/
`useInstancedEntityEvents`. So today, clicking directly on a rendered triad is a
no-op — there's no raycast handler attached to it, confirmed by absence (grep
for the events hooks lists only the 8 renderers above; `AxesHelpers.svelte` is
not among them).

**`removable: false` blocks only one unrelated built-in icon, not selection or
a custom action.** `draw.js:56` only pushes `traits.Removable` onto the entity
`if (removable)` — world-state-store transforms never get that trait at all.
In `components/overlay/Details.svelte`, the only place `removable` is read is
the trash-can icon guard (`{#if removable.current}` around a button that calls
`entity.destroy()` directly on the local ECS world — it does **not** call any
backend command, so it wouldn't have been useful to us anyway). Nothing else in
the selection/Details pipeline checks `removable`.

**A working custom-action extension point already exists: the `details` snippet
prop.** `Details.svelte` renders `{@render details?.({ entity })}` inside its
markup; that `details` prop is `App.svelte`'s own `Props.details: Snippet<[{entity: Entity}]>`,
and `Visualizer` **is** `App.svelte` (`dist/index.js:3` / `dist/index.d.ts:3`:
"MotionTools has been renamed to Visualizer... `export { default as Visualizer }
from './components/App.svelte'`"). Our `frontend/src/App.svelte` already
mounts `<Visualizer partID={...} {localConfigProps} {componentNameToFragmentInfo}>`
but doesn't pass a `details` snippet today — it's an unused, already-wired hook.

**By contrast, the package's own `DetailsPortal` export looks broken in this
version.** `components/overlay/Portals/DetailsPortal.svelte` wraps children in
`<Portal id="details">`, but grepping the whole `dist/` tree for
`PortalTarget id=` turns up only `"dom"`, `"details-header-icon"`,
`"details-extensions"`, `"controls"`, `"dashboard"` — there is no
`PortalTarget id="details"` anywhere, so `<Portal id="details">` has nothing to
mount into. Don't build on `DetailsPortal`; use the `details` snippet prop
instead.

**Entities carry stable identifiers we can match against.** `drawTransform`
(`draw.js:38`) sets `traits.Name(referenceFrame)` (the taught frame's name) and,
when the transform has a UUID, `traits.UUID(uuidStr)`. Both `traits.Name` and
`traits.UUID` are readable via the exported `useTrait`/`useQuery`, so a selected
entity can be matched back to one of our taught frames (by name or UUID)
without re-deriving any pose math ourselves.

**A free, already-functioning selection path exists today, just not a
scene-click one.** `components/overlay/left-pane/TreeContainer.svelte` mounts
an always-open (`isOpen`, `exitable={false}`), non-gated `FloatingPanel`
titled "World" built from `useTree.svelte.js`, which walks **every** entity
with `traits.Name` (no filter excluding `WorldStateStoreAPI`) into a tree.
Selecting a row there calls the exact same `entity.add(traits.Selected)` /
`entity.remove(traits.Selected)` pattern, opening the Details panel. So taught
frames are already selectable (and Details-panel-visible) via this native
"World" tree list, unconditionally mounted in our app right now — confirming
the whole selection→Details pipeline works end-to-end for world-state-store
entities, just not via "click the triad in 3D."

**Public API surface confirmed sufficient to build a custom in-scene picker.**
`dist/index.js` exports `relations`, `traits`, `provideWorld`/`useWorld`,
`useQuery`, `useTrait`, `FloatingPanel`, `DashboardPortal`, `DetailsPortal`
(broken, see above), alongside `Visualizer`. The `./lib` subpath (what our
`poseTransform.ts`/`FrameDefinePlugin.svelte` already import from) is a
deliberately narrow, hook-free surface (`AxesHelper`, `Snapshot`, `BatchedArrow`,
`CapsuleGeometry`, `OrientationVector`, a few pure functions) and does **not**
carry ECS hooks — but the *main* entry point does, and our own `children`
snippet mounts inside `Scene.svelte`, which is inside `App.svelte`'s
`provideWorld()` context, so a plugin in `frontend/src/scene/` can call
`useWorld()`/`useQuery()`/`useTrait()` directly.

### Risks for the one viable in-scope approach

Since the shared `BatchedAxesHelpers` object can't be given per-instance click
handlers without patching the vendor package, the only viable approach is a
**parallel invisible proxy mesh per matched taught-frame entity**, positioned
from that entity's own resolved `traits.WorldMatrix` (read via `useTrait`, not
recomputed from our own `poseTransform.ts` math) — this specifically avoids the
documented **non-origin arm-base caveat** (design doc "Slice outcome," item 2:
our own triads/markers draw in scene-root and only coincide with the
Visualizer's arm draw when the arm base ≈ world origin; reusing the entity's
already-resolved `WorldMatrix` sidesteps re-deriving that transform a second
time and getting it wrong for non-identity arm bases).

- **Raycast/click-vs-drag:** our existing scene markers use
  `raycast={() => null}` (`frontend/src/scene/FrameDefinePlugin.svelte:56,64`)
  specifically so they stay inert during camera-orbit drags. A clickable proxy
  is the opposite — it must reimplement the drag-distance guard from
  `useEntityEvents.svelte.js` (`down.distanceToSquared(event.pointer) >= 0.1`)
  or every orbit-drag that starts/ends over a triad will spuriously select it.
- **Disambiguating entities:** `traits.WorldStateStoreAPI` may in principle tag
  non-frame world-state-store entities too; matching should cross-reference our
  own `listFrames` DoCommand result (name/UUID), not just the tag + `ShowAxesHelper`.
- **Multi-select:** `App.svelte` renders one `<Details>` per selected entity
  (`{#each selected.current ...}`); our `details` snippet is passed the specific
  entity per instance so this is safe, but the snippet must itself check
  "is this entity one of our taught frames" before rendering a Delete button
  (arbitrary other entities can be selected too, e.g. via the "World" tree).
- **No code changes made** to verify click-to-select empirically on hardware —
  this is a static-analysis read of the shipped package source; I did not spin
  up the app to click-test the "World" tree selection path, though the code
  path is unambiguous from the source above.

### Verdict: feasible but costly

No hard block: the public API (`useWorld`, `useQuery`, `useTrait`, `traits`,
and the `Visualizer`'s `details` snippet prop) is sufficient, and
`removable: false` only suppresses one unrelated built-in "remove from scene"
icon — it does not gate selection or a custom action. But nothing needed for
"click the triad" exists yet: it requires a new scene plugin doing (1)
entity-to-taught-frame matching by name/UUID, (2) a proxy mesh per frame
positioned off `traits.WorldMatrix`, (3) hand-rolled click-vs-drag
disambiguation, and (4) a new `details` snippet wired into `App.svelte` that
renders a Delete button calling `deleteFrame` (`lib/poseTracker.ts`) when the
selected entity matches. That's comparable in size to standing up a new panel +
scene plugin pair, not a small add-on.

### Recommendation

Keep frame management panel-driven via `ManageFramesPanel.svelte` for now.
Revisit scene-select as part of **track E** (frame-relative jog), which will
need its own scene frame-picker (clicking a frame in 3D to pick "jog relative
to this frame") — that picker requires the *same* entity-matching +
clickable-proxy + click-vs-drag machinery. Building it once for track E and
then reusing the resulting selection signal to add a Delete button in the
`details` snippet (and/or highlight the matching `ManageFramesPanel` row) is
much better ROI than building the picker mechanism twice.
