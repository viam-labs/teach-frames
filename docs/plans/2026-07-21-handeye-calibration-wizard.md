# Hand-eye Calibration Wizard Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** One mode-adaptive "Camera Calibration" wizard (eye-to-hand OR eye-in-hand, per `get_handeye_mode`): snapshot → click target(s) in the 2D camera image → solve+persist the camera transform (with RMS residual), driving the already-shipped DoCommands.

**Architecture:** The A+B wizard template: a pure rune store (`handeyeCalib.svelte.ts`) + a prime-chrome `FloatingPanel` (`HandeyeWizard.svelte`) that hosts the clickable camera image AND the mode-branched controls, mounted in `<Visualizer>`'s `children`. No scene plugin (the camera's own frame triad, correct since the Bug-2 fix, jumps to the solved pose). Ports the logic of `HandEyePanel.svelte` (eye-to-hand) + `EyeInHandPanel.svelte` (eye-in-hand) into one component.

**Tech Stack:** Svelte 5 runes, `@viamrobotics/motion-tools` (`FloatingPanel`/`DashboardPortal`), `@viamrobotics/svelte-sdk`, `@viamrobotics/prime-core` (`Icon`), Tailwind v4, Vitest.

**Design doc:** `docs/plans/2026-07-21-handeye-calibration-wizard-design.md`

**Commands (in `frontend/`):** `npx vitest run <file>`, `npm run test`, `npm run check`, `npm run build`.

**Backend contract (shipped, do not change) — `frontend/src/lib/poseTracker.ts`:**
- `getHandeyeMode()` → `HandEyeModeResponse { camera_mount: CameraMount, camera_configured, arm_configured }`; `CameraMount = 'eye_to_hand' | 'eye_in_hand'`.
- `handeyeSnapshot()` → `HandEyeSnapshotResponse { image (base64 JPEG), width, height }`.
- Eye-to-hand: `captureHandeyePoint(u,v)` → `CaptureHandEyeResponse { index, buffer_len, world, camera }`.
- Eye-in-hand: `captureHandeyeTarget()` → `CaptureHandEyeTargetResponse { target }`; `captureHandeyeView(u,v)` → `CaptureHandEyeViewResponse { index, buffer_len, target, flange, camera }`.
- `getHandeyeBuffer()` → `HandEyeBufferResponse` (eye-to-hand) / `EyeInHandBufferResponse` (eye-in-hand) — `{ points }`.
- `clearHandeyeBuffer()` → `{ cleared }` (also drops the eye-in-hand target).
- `solveHandeye()` → `SolveHandEyeResponse { committed, pose, residual_rms, parent, orientation }` — solves+persists+clears buffer on success; buffer preserved on failure.
- `displayToNativePixel(offsetX, offsetY, clientW, clientH, nativeW, nativeH)` → `{ u, v }` — pure, already unit-tested in `poseTracker.test.ts`.

**Conventions:** re-entrancy guards on capture/click (`isPending`), prime tokens matching `FrameDefineWizard.svelte`, `min-h-11` + `focus-visible:ring-2`, `role="alert"` errors, required 4-arg `createResourceQuery(..., () => ({}))`.

---

## Task 1: Hand-eye calibration wizard store (mode-adaptive)

**Files:**
- Create: `frontend/src/lib/wizard/handeyeCalib.svelte.ts`
- Create: `frontend/src/lib/wizard/handeyeCalib.test.ts`

**Step 1: Write the failing test** — `frontend/src/lib/wizard/handeyeCalib.test.ts`:

```ts
import { describe, it, expect, beforeEach } from 'vitest'
import { createHandeyeWizard } from './handeyeCalib.svelte'

const solveResult = { residual_rms: 0.8, parent: 'world' }

describe('handeyeCalib wizard', () => {
  let w: ReturnType<typeof createHandeyeWizard>
  beforeEach(() => { w = createHandeyeWizard() })

  it('starts in setup, no mode, 0 captures, cannot solve, no target need', () => {
    expect(w.phase).toBe('setup')
    expect(w.mode).toBeUndefined()
    expect(w.captureCount).toBe(0)
    expect(w.canSolve).toBe(false)
    expect(w.minCaptures).toBe(3)
    expect(w.needsTarget).toBe(false)
  })

  it('start(eye_to_hand) → capturing, no target step', () => {
    w.start('eye_to_hand')
    expect(w.phase).toBe('capturing')
    expect(w.mode).toBe('eye_to_hand')
    expect(w.needsTarget).toBe(false)
  })

  it('start(eye_in_hand) → capturing and needs a target first', () => {
    w.start('eye_in_hand')
    expect(w.phase).toBe('capturing')
    expect(w.needsTarget).toBe(true)
    w.targetTouched()
    expect(w.hasTarget).toBe(true)
    expect(w.needsTarget).toBe(false)
  })

  it('targetTouched only applies while capturing', () => {
    w.targetTouched() // in setup
    expect(w.hasTarget).toBe(false)
  })

  it('recordCapture tracks buffer_len and gates solve at >=3', () => {
    w.start('eye_to_hand')
    w.recordCapture(1); expect(w.canSolve).toBe(false)
    w.recordCapture(2); expect(w.canSolve).toBe(false)
    w.recordCapture(3); expect(w.captureCount).toBe(3); expect(w.canSolve).toBe(true)
  })

  it('ignores recordCapture outside capturing', () => {
    w.recordCapture(3)
    expect(w.captureCount).toBe(0)
  })

  it('solved() stores the result and enters solved', () => {
    w.start('eye_to_hand'); w.recordCapture(3)
    w.solved(solveResult)
    expect(w.phase).toBe('solved')
    expect(w.solve).toEqual(solveResult)
  })

  it('clearCaptures() resets count + target and returns to capturing', () => {
    w.start('eye_in_hand'); w.targetTouched(); w.recordCapture(3); w.solved(solveResult)
    w.clearCaptures()
    expect(w.phase).toBe('capturing')
    expect(w.captureCount).toBe(0)
    expect(w.hasTarget).toBe(false)
  })

  it('finish() reaches done (from solved)', () => {
    w.start('eye_to_hand'); w.recordCapture(3); w.solved(solveResult)
    w.finish()
    expect(w.phase).toBe('done')
    expect(w.done).toBe(true)
  })

  it('setError surfaces a message without losing count', () => {
    w.start('eye_to_hand'); w.recordCapture(2)
    w.setError('no depth at pixel')
    expect(w.error).toBe('no depth at pixel')
    expect(w.captureCount).toBe(2)
  })

  it('reset() returns to a fresh setup wizard', () => {
    w.start('eye_in_hand'); w.targetTouched(); w.recordCapture(3); w.solved(solveResult); w.reset()
    expect(w.phase).toBe('setup')
    expect(w.mode).toBeUndefined()
    expect(w.captureCount).toBe(0)
    expect(w.hasTarget).toBe(false)
    expect(w.solve).toBeUndefined()
  })
})
```

**Step 2: Run — expect FAIL** — `cd frontend && npx vitest run src/lib/wizard/handeyeCalib.test.ts`.

**Step 3: Write the store** — `frontend/src/lib/wizard/handeyeCalib.svelte.ts`:

```ts
// Mode-adaptive hand-eye calibration wizard. `get_handeye_mode` decides the flow:
//   eye_to_hand  — snapshot once, touch several world points with the TCP and
//                  click each in the image; solve camera→world.
//   eye_in_hand  — touch ONE target, then re-snapshot+click it from ≥3 vantages;
//                  solve camera→flange. Needs a target before any view.
// Pure logic; the panel wires the DoCommands and owns the transient snapshot
// image + click dots. solve_handeye commits atomically (persists to the camera's
// frame), so review is post-commit (the `solved` phase).
import type { CameraMount } from '../poseTracker'

export type HandeyePhase = 'setup' | 'capturing' | 'solved' | 'done'
export interface HandeyeSolve { residual_rms: number; parent: string }

const MIN_CAPTURES = 3

export function createHandeyeWizard() {
  let phase = $state<HandeyePhase>('setup')
  let mode = $state<CameraMount | undefined>(undefined)
  let captureCount = $state(0)
  let hasTarget = $state(false)
  let solve = $state<HandeyeSolve | undefined>(undefined)
  let error = $state<string | undefined>(undefined)

  return {
    get phase() { return phase },
    get mode() { return mode },
    get captureCount() { return captureCount },
    get hasTarget() { return hasTarget },
    get solve() { return solve },
    get error() { return error },
    get minCaptures() { return MIN_CAPTURES },
    get canSolve() { return captureCount >= MIN_CAPTURES },
    get needsTarget() { return mode === 'eye_in_hand' && !hasTarget },
    get done() { return phase === 'done' },

    start(m: CameraMount) {
      mode = m; captureCount = 0; hasTarget = false; solve = undefined; error = undefined
      phase = 'capturing'
    },
    targetTouched() { if (phase === 'capturing') { hasTarget = true; error = undefined } },
    recordCapture(bufferLen: number) { if (phase === 'capturing') { captureCount = bufferLen; error = undefined } },
    solved(result: HandeyeSolve) { solve = result; error = undefined; phase = 'solved' },
    // Re-calibrate: clear_handeye_buffer drops the eye-in-hand target too, so mirror that.
    clearCaptures() { captureCount = 0; hasTarget = false; error = undefined; phase = 'capturing' },
    finish() { phase = 'done' },
    setError(message: string) { error = message },
    reset() {
      phase = 'setup'; mode = undefined; captureCount = 0; hasTarget = false
      solve = undefined; error = undefined
    },
  }
}
```

**Step 4: Verify green** — `npx vitest run … handeyeCalib.test.ts` (PASS), `npm run check` (0 errors), `npm run test` (all green).

**Step 5: Commit**

```bash
git add frontend/src/lib/wizard/handeyeCalib.svelte.ts frontend/src/lib/wizard/handeyeCalib.test.ts
git commit -m "feat(wizard): mode-adaptive hand-eye calibration store"
```

---

## Task 2: Hand-eye wizard panel + mount

**Files:**
- Create: `frontend/src/panels/HandeyeWizard.svelte`
- Modify: `frontend/src/App.svelte`

**Read first:** `frontend/src/panels/HandEyePanel.svelte` and `EyeInHandPanel.svelte` — the exact DoCommand wiring, the `displayToNativePixel` image-click handler, the dot overlay, and the eye-in-hand snapshot-consumed-in-`finally` logic and its comment. Port those. Also `FrameDefineWizard.svelte` for the prime chrome (button/input recipes, DashboardToggle/FloatingPanel usage).

**Step 1: Create the panel** — `frontend/src/panels/HandeyeWizard.svelte`.

Script essentials:
- `let { wizard }: { wizard: ReturnType<typeof createHandeyeWizard> } = $props()`.
- `pt = poseTrackerClient(machineId, () => selectedResource.name ?? '')`.
- Queries: `modeQ = createResourceQuery(pt, 'doCommand', () => toCommandArgs(getHandeyeMode()), () => ({}))`; `bufferQ = createResourceQuery(pt, 'doCommand', () => toCommandArgs(getHandeyeBuffer()), () => ({}))`.
- Mutations: `snap`, `capturePt`, `captureTgt`, `captureView`, `clear`, `solveM` — all `createResourceMutation(pt, 'doCommand')`.
- Derived: `const modeResp = $derived(modeQ.data as HandEyeModeResponse | undefined)`; `const mode = $derived(modeResp?.camera_mount)`; `const cameraOk = $derived(modeResp?.camera_configured ?? false)`; `const armOk = $derived(modeResp?.arm_configured ?? false)`.
- Component state: `let open = $state(false)`; `let snapshot = $state<HandEyeSnapshotResponse | undefined>(undefined)`; `let dots = $state<{x:number;y:number}[]>([])`; `let imgEl = $state<HTMLImageElement | undefined>(undefined)`.
- Handlers (guard each mutation on `.isPending`; source counts from response `buffer_len`):
  - `handleStart()`: `if (!mode) return; wizard.start(mode); snapshot = undefined; dots = []`.
  - `handleTouchTarget()` (eye-in-hand): `captureTgt.mutateAsync(captureHandeyeTarget())` → `wizard.targetTouched(); snapshot = undefined`.
  - `handleSnapshot()`: `const res = await snap.mutateAsync(handeyeSnapshot())` → `snapshot = res`. (catch → `wizard.setError`.)
  - `handleImageClick(event)`: `if (!snapshot || !imgEl) return`; compute `{u,v} = displayToNativePixel(event.offsetX, event.offsetY, imgEl.clientWidth, imgEl.clientHeight, snapshot.width, snapshot.height)`. Then branch on `wizard.mode`:
    - eye_to_hand: `if (capturePt.isPending) return`; `const res = await capturePt.mutateAsync(captureHandeyePoint(u,v))`; `dots = [...dots, {x:event.offsetX, y:event.offsetY}]`; `wizard.recordCapture(res.buffer_len)`; `void bufferQ.refetch()`. **Do NOT clear the snapshot** (camera static, one snapshot serves many clicks). catch → setError.
    - eye_in_hand: `if (captureView.isPending) return`; try `const res = await captureView.mutateAsync(captureHandeyeView(u,v))`; `wizard.recordCapture(res.buffer_len)`; `void bufferQ.refetch()`; catch → setError; **`finally { snapshot = undefined }`** (server takes-and-clears the snapshot before deprojecting — one snapshot, one attempt; port the EyeInHandPanel comment).
  - `handleClear()`: `clear.mutateAsync(clearHandeyeBuffer())` → `wizard.clearCaptures(); dots = []; snapshot = undefined; void bufferQ.refetch()`.
  - `handleSolve()`: `const res = await solveM.mutateAsync(solveHandeye())`; `if (res?.committed) { wizard.solved({ residual_rms: res.residual_rms, parent: res.parent }); dots = []; snapshot = undefined; void bufferQ.refetch() } else wizard.setError('solve_handeye did not commit')`. catch → setError (degenerate/persistence; buffer preserved).
  - `handleRecalibrate()`: same as `handleClear` but the store call is `wizard.clearCaptures()` (returns to capturing from solved).
  - `handleDone()`: `wizard.finish()`.
  - `handleAgain()`: `wizard.reset(); snapshot = undefined; dots = []`.

Cast responses with `as unknown as <Type>` (as the sibling panels do). Do NOT import or bump `frameRevision` (the camera triad updates via motion-tools' `useFrames` refetch).

Markup — `DashboardPortal` + `DashboardToggle` (label "Camera calibration", a VALID prime-core icon distinct from `image-filter-center-focus`/`table`/`axis-arrow` — try `"camera-outline"`; if `npm run check` rejects it, pick another valid distinct one e.g. `"cctv"`/`"camera"`; REPORT the choice) → `FloatingPanel title="Camera Calibration" bind:isOpen={open} defaultPosition={{ x: 24, y: 460 }} defaultSize={{ width: 420, height: 460 }} resizable`. Body `div class="flex h-full flex-col gap-3 overflow-y-auto p-4"`, prime chrome from `FrameDefineWizard`, phases:

- `{#if modeQ.error}`: `<p class="text-danger-dark text-xs" role="alert">Camera calibration unavailable: {modeQ.error.message}</p>` (only THIS panel surfaces the mode error).
- `{:else if wizard.phase === 'setup'}`: mode-aware intro text (`text-gray-9 text-sm`) — eye-to-hand: "Touch several fixed points with the TCP and click each in the camera view; solves camera → world." / eye-in-hand: "Touch one target, then snapshot and click it from ≥3 wrist vantages; solves camera → flange." Gating: if `!cameraOk` → "Requires a configured RGBD camera."; if `mode === 'eye_in_hand' && !armOk` → "Requires a configured arm." Otherwise **Start** (primary, `disabled={!mode || !cameraOk || (mode === 'eye_in_hand' && !armOk)}`).
- `{:else if wizard.phase === 'capturing'}`:
  - If `wizard.needsTarget`: a "Touch target" step — instruction "Jog the TCP onto a physical point and touch it once; every view clicks this same point." + **Touch target** button (`disabled={captureTgt.isPending}`).
  - Else (target set, or eye-to-hand): snapshot control — **Refresh view** (eye-to-hand) / **Snapshot** (eye-in-hand) button (`disabled={snap.isPending}`); then, if `snapshot`, the clickable image in a `relative inline-block` container: `<img bind:this={imgEl} src={`data:image/jpeg;base64,${snapshot.image}`} alt="Camera view — click the target point" onclick={handleImageClick} class="block max-w-full h-auto cursor-crosshair rounded-md" style="object-fit: fill" />` plus the `dots` overlay spans (absolute, `pointer-events-none`), with the `<!-- svelte-ignore a11y_click_events_have_key_events, a11y_no_noninteractive_element_interactions -->` comment from the legacy panels. `object-fit: fill` is REQUIRED (displayToNativePixel assumes it).
  - Count line `{wizard.captureCount} / {wizard.minCaptures}+ captured`.
  - **Clear** (secondary, `disabled={clear.isPending || (wizard.captureCount === 0 && !wizard.hasTarget)}`) + **Solve calibration** (primary, `disabled={!wizard.canSolve || solveM.isPending}`).
- `{:else if wizard.phase === 'solved'}`: `Committed to {wizard.solve?.parent} · residual RMS {fmt(wizard.solve?.residual_rms ?? 0)} mm` (prominent). If `wizard.mode === 'eye_in_hand'`: the vary-orientation caveat (`text-subtle-1 text-xs`): "A low residual doesn't prove the calibration is good — vary the wrist orientation across vantages, not just position." Buttons: **Re-calibrate** (secondary → `handleRecalibrate`), **Done** (primary → `handleDone`).
- `{:else if wizard.phase === 'done'}`: "Camera calibrated." + **Calibrate again** (`handleAgain`).
- Always after: capture/solve/snapshot mutation errors and `{#if wizard.error}<p class="text-danger-dark text-xs" role="alert">{wizard.error}</p>{/if}`.

**Step 2: Mount in App.svelte**
- Import `createHandeyeWizard` and `HandeyeWizard`.
- At script top (OUTSIDE the `{#key frameRevision.value}` block, beside the other wizard stores): `const handeyeWizard = createHandeyeWizard()`.
- In the children snippet, after `<TcpTeachWizard wizard={tcpWizard} />`: `<HandeyeWizard wizard={handeyeWizard} />`.

**Step 3: Verify** — from `frontend/`: `npm run check` (0 errors — confirms the icon + all response types), `npm run test` (all green), `npm run build` (success).

**Step 4: Commit**

```bash
git add frontend/src/panels/HandeyeWizard.svelte frontend/src/App.svelte
git commit -m "feat(frontend): mode-adaptive hand-eye calibration wizard in the Visualizer chrome"
```

---

## Task 3: Hardware verification (user-run) — first real RGBD validation

For the machine's configured mode:
1. **Eye-to-hand:** Refresh view; jog the TCP to ≥3 well-spread points, click each; Solve → confirm **residual RMS small**; the camera frame triad renders at the solved world pose.
2. **Eye-in-hand:** Touch target; from ≥3 vantages with **varied wrist orientation**, snapshot + click the same point; Solve → small residual; camera triad at solved flange-relative pose. Confirm that NOT varying orientation keeps residual low yet mis-calibrates (the caveat).
3. No-depth pixel click → clear error + (eye-in-hand) snapshot cleared; re-snapshot works.
4. Persistence-disabled path (if reproducible) reports `committed:false`.

Record results under the design docs' verification headings. No code commit unless a bug surfaces (TDD the fix).

---

## Definition of done
- [ ] `npm run check` / `npm run test` / `npm run build` green.
- [ ] One adaptive wizard: reads `get_handeye_mode`, runs the correct eye-to-hand / eye-in-hand guided flow with the clickable 2D image; RMS residual shown; eye-in-hand caveat shown.
- [ ] Distinct toolbar toggle; prime chrome; capture/click re-entrancy guards; dependency gating from the mode query.
- [ ] Hardware: small residual for the configured mode; camera triad renders at the solved pose; roadmap memory updated (D done, E next).
