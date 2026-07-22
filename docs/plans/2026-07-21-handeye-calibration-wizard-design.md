# Hand-eye calibration wizard (track D) — design

**Date:** 2026-07-21
**Branch:** `redesign/visualizer-teach-pendant`
**Status:** design approved; ready for an implementation plan.

Track D of the teach-pendant roadmap (A+B → C → **D** → E → F/G) — the largest and
most interesting: a fundamentally **2D image-click** calibration inside a
3D-scene-centric pendant. The **backend is already shipped** (2026-07-12
eye-to-hand, 2026-07-15 eye-in-hand) — this is a frontend port + guided-wizard
reshape, no backend change. It is also the **first real-RGBD-hardware validation**
of the solve (the eye-to-hand solver was previously unit-tested against fakes only).

## The two modes (machine is configured as exactly one)

`get_handeye_mode` returns `{ camera_mount: 'eye_to_hand' | 'eye_in_hand',
camera_configured, arm_configured }`.

- **Eye-to-hand** — camera fixed in the world, arm moves; solves **camera → world**
  (persisted to the camera's `frame`, parent = destination_frame). Snapshot once,
  then jog the TCP to touch several physical points and click each in the image
  (`capture_handeye_point` pairs the world TCP pose with the camera-deprojected
  point). One snapshot serves many clicks (camera is static). Solve at ≥3 points.
- **Eye-in-hand** — camera on the wrist, moves with the arm; solves
  **camera → flange** (persisted to the camera's `frame`, parent = arm). Touch ONE
  target, then repeatedly jog to a new vantage → snapshot (freezes the frame + the
  flange pose) → click that SAME target (`capture_handeye_view`). Each snapshot is
  **consumed** per click. Solve at ≥3 vantages. Caveat: a low residual alone does
  NOT prove correctness — the wrist **orientation** must vary across vantages.

## Decisions (brainstorming, 2026-07-21)

1. **One mode-adaptive wizard** — a single "Camera Calibration" toggle + wizard
   that reads `get_handeye_mode` and shows the right guided flow, sharing the
   image-click / snapshot / solve substrate and branching on the mode-specific
   steps (eye-in-hand's target step + per-view snapshot lifecycle). No inert
   second panel.
2. **Minimal scene coupling** — the clickable 2D image (required; deprojection
   needs a real pixel) lives in a FloatingPanel. Rely on the camera's own frame
   triad auto-moving to the solved pose (free since the Bug-2 SDK fix) as the
   before/after; show the residual RMS prominently in the wizard. No scene plugin.

## Architecture

- **Shared rune store** `frontend/src/lib/wizard/handeyeCalib.svelte.ts` (pure,
  unit-tested; created once in `App.svelte`, passed to the panel). Holds the
  procedure phase + mode + capture count + eye-in-hand target flag + solve result.
- **FloatingPanel** `frontend/src/panels/HandeyeWizard.svelte` hosting the
  **clickable camera image** AND the step controls (resizable for click
  precision), + a `DashboardToggle` (distinct icon), prime chrome. Reads
  `get_handeye_mode`; the transient snapshot image + click dots live in component
  state (as in the legacy panels). Drives the shipped DoCommands via `poseTracker.ts`
  builders: `handeyeSnapshot`, `captureHandeyePoint`, `captureHandeyeTarget`,
  `captureHandeyeView`, `getHandeyeBuffer`, `clearHandeyeBuffer`, `solveHandeye`,
  `getHandeyeMode`, and the pure `displayToNativePixel` (handles `object-fit: fill`
  scaling; already unit-tested in `poseTracker.test.ts`).
- **No scene plugin.** The camera frame triad is rendered by motion-tools.
- Logic sources to port: `frontend/src/panels/HandEyePanel.svelte` (eye-to-hand)
  and `EyeInHandPanel.svelte` (eye-in-hand), both still on the removed `tokens.css`.

## Flow (mode-adaptive phase model: `setup → capturing → solved → done`)

1. **setup** — read `get_handeye_mode`; mode-aware intro. **Start**. Gate on
   `camera_configured` (both modes) and `arm_configured` (eye-in-hand); if the
   mode query fails, show "calibration unavailable: <reason>" (as the legacy
   `EyeInHandPanel` does — only ONE panel surfaces it to avoid a duplicate error).
2. **capturing** — the image + click loop, branched on `mode` + `hasTarget`:
   - **Eye-in-hand & no target:** **Touch target** (`capture_handeye_target`) →
     `hasTarget`. Then the vantage loop.
   - **Snapshot:** "Refresh view" (eye-to-hand) / "Snapshot" (eye-in-hand) →
     `handeye_snapshot` → render the base64 JPEG frame.
   - **Click the image** → `displayToNativePixel(offsetX, offsetY, clientW,
     clientH, snap.width, snap.height)` → `(u,v)` → `capture_handeye_point` /
     `capture_handeye_view`; dot overlay at the click.
     - *Eye-to-hand:* snapshot persists across clicks; jog the TCP between touched
       points.
     - *Eye-in-hand:* snapshot is **spent** on click (server takes-and-clears
       before deprojecting) → clear the image unconditionally in a `finally` so the
       UI matches "one snapshot, one attempt"; re-snapshot each vantage.
   - `N / ≥3 captured`; **Solve** enabled at ≥3.
3. **solved** — `solve_handeye` → **residual RMS** + committed (+ persisted
   `parent`). For eye-in-hand, show the **vary-orientation caveat** prominently.
   The camera triad moves to the solved pose. **Re-calibrate** (clear + back to
   capturing) or **Done**.

## Store shape (sketch)

```
phase: 'setup' | 'capturing' | 'solved' | 'done'
mode: CameraMount | undefined     // 'eye_to_hand' | 'eye_in_hand'
captureCount: number              // from capture_*/get_handeye_buffer buffer_len
hasTarget: boolean                // eye-in-hand only
solve: { residual_rms, parent } | undefined
error: string | undefined
get minCaptures() { return 3 }
get canSolve() { return captureCount >= 3 }
get needsTarget() { return mode === 'eye_in_hand' && !hasTarget }
start(mode); targetTouched(); recordCapture(bufferLen); solved(result);
clearCaptures() /* count=0, hasTarget=false */; setError(msg); reset()
```

## Error handling (reuse the legacy panels' hard-won logic)

- **Deps not configured** — `get_handeye_mode` reports `camera_configured` /
  `arm_configured`; snapshot errors matched to "Requires a configured RGBD
  camera / arm".
- **No-depth pixel click** (routine on depth cameras — specular/edges/out of
  range): the capture errors; in eye-in-hand the snapshot is already spent
  server-side, so clear the image in a `finally` and surface the error, letting
  the operator re-snapshot. One snapshot, one attempt.
- **Degenerate / persistence-disabled solve** — surfaced; the buffer is preserved
  so the operator can add more captures.

## Testing

- **Store units**: mode-adaptive phase transitions; `canSolve` (≥3); `needsTarget`
  gating for eye-in-hand; `recordCapture` count; `clearCaptures` resets target;
  `reset`.
- **`displayToNativePixel`**: already unit-tested (`poseTracker.test.ts`) — keep.
- **Panel**: light render test optional.
- **Hardware (first real RGBD validation of the solve)** for the configured mode:
  small residual RMS; the camera frame triad renders at the solved pose; for
  eye-in-hand, confirm orientation variation actually lowers error (the caveat).

## Out of scope

Rendering the camera stream/frustum in the 3D scene and 3D correspondence/residual
visualization (minimal-scene decision) — the click stays in the 2D image panel.
The friendlier future enhancements (e.g. auto-suggesting well-spread vantages)
are deferred.
