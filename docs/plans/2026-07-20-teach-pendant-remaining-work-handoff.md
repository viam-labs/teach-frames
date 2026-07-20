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
