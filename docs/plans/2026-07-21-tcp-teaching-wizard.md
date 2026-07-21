# TCP Teaching Wizard Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** A guided TCP-calibration wizard: capture ≥4 pivot touches → solve+persist the tool-tip offset (with RMS residual) → optional explicit orientation — driving the already-shipped pose-tracker DoCommands.

**Architecture:** Mirrors the shipped A+B wizard template: a pure rune store (`tcpTeach.svelte.ts`) + a prime-chrome `FloatingPanel` (`TcpTeachWizard.svelte`) + `DashboardToggle`, mounted in `<Visualizer>`'s `children`. No scene plugin (the tool's own frame triad, correct since the Bug-2 SDK fix, gives the before/after; it auto-updates via `useFrames` on config revision). Ports the logic of the legacy `TcpPanel.svelte`.

**Tech Stack:** Svelte 5 runes, `@viamrobotics/motion-tools` (`FloatingPanel`/`DashboardPortal`), `@viamrobotics/svelte-sdk` (`createResourceMutation`/`createResourceQuery`), `@viamrobotics/prime-core` (`Icon`), Tailwind v4, Vitest.

**Design doc:** `docs/plans/2026-07-21-tcp-teaching-wizard-design.md`

**Commands (run in `frontend/`):** `npx vitest run <file>`, `npm run test`, `npm run check`, `npm run build`.

**Backend contract (shipped, do not change) — `frontend/src/lib/poseTracker.ts`:**
- Builders: `captureTcpPoint()`, `getTcpBuffer()`, `clearTcpBuffer()`, `teachTcpPosition()`, `teachTcpOrientation(o_x,o_y,o_z,theta)`.
- `captureTcpPoint` → `CaptureResponse { index, buffer_len, pose }`.
- `getTcpBuffer` → `BufferResponse { points: PoseMap[] }`.
- `teachTcpPosition` → `TeachTcpPositionResponse { committed, offset:{x,y,z}, residual_rms }` — **solves AND persists atomically**, clears the TCP buffer on success.
- `teachTcpOrientation` → `TeachTcpOrientationResponse { committed, orientation:{o_x,o_y,o_z,theta} }` — must run AFTER a position teach.
- All are DoCommands on the pose-tracker; the TCP buffer is separate from the frame capture buffer.

**Conventions:** re-entrancy guard on capture (`isPending`), gate on `armState.hasArm` + `motion.busy`, prime tokens matching `FrameDefineWizard.svelte`, `min-h-11` + `focus-visible:ring-2` on controls, `role="alert"` on errors, the required 4-arg `createResourceQuery(..., () => ({}))` form.

---

## Task 1: TCP teach wizard store (phase model)

**Files:**
- Create: `frontend/src/lib/wizard/tcpTeach.svelte.ts`
- Create: `frontend/src/lib/wizard/tcpTeach.test.ts`

**Step 1: Write the failing test**

`frontend/src/lib/wizard/tcpTeach.test.ts`:

```ts
import { describe, it, expect, beforeEach } from 'vitest'
import { createTcpTeachWizard } from './tcpTeach.svelte'

const posResult = { offset: { x: 1, y: 2, z: 3 }, residual_rms: 0.4 }

describe('tcpTeach wizard', () => {
  let w: ReturnType<typeof createTcpTeachWizard>
  beforeEach(() => { w = createTcpTeachWizard() })

  it('starts idle in setup, 0 captures, cannot solve', () => {
    expect(w.phase).toBe('setup')
    expect(w.captureCount).toBe(0)
    expect(w.canSolve).toBe(false)
    expect(w.minCaptures).toBe(4)
  })

  it('start() enters capturing and resets state', () => {
    w.start()
    expect(w.phase).toBe('capturing')
    expect(w.captureCount).toBe(0)
    expect(w.position).toBeUndefined()
  })

  it('recordCapture tracks the server buffer_len and gates solving at >=4', () => {
    w.start()
    w.recordCapture(1); expect(w.captureCount).toBe(1); expect(w.canSolve).toBe(false)
    w.recordCapture(2); w.recordCapture(3); expect(w.canSolve).toBe(false)
    w.recordCapture(4); expect(w.captureCount).toBe(4); expect(w.canSolve).toBe(true)
  })

  it('ignores recordCapture outside the capturing phase', () => {
    w.recordCapture(3) // still in setup
    expect(w.captureCount).toBe(0)
  })

  it('solved() stores the result and enters taught', () => {
    w.start(); w.recordCapture(4)
    w.solved(posResult)
    expect(w.phase).toBe('taught')
    expect(w.position).toEqual(posResult)
  })

  it('reteach() clears back to capturing', () => {
    w.start(); w.recordCapture(4); w.solved(posResult)
    w.reteach()
    expect(w.phase).toBe('capturing')
    expect(w.captureCount).toBe(0)
    expect(w.position).toBeUndefined()
  })

  it('toOrientation() only advances from taught', () => {
    w.toOrientation() // from setup: no-op
    expect(w.phase).toBe('setup')
    w.start(); w.recordCapture(4); w.solved(posResult)
    w.toOrientation()
    expect(w.phase).toBe('orientation')
  })

  it('finish() reaches done from taught or orientation', () => {
    w.start(); w.recordCapture(4); w.solved(posResult)
    w.finish()
    expect(w.phase).toBe('done')
    expect(w.done).toBe(true)
  })

  it('setError surfaces a message without losing the capture count', () => {
    w.start(); w.recordCapture(4)
    w.setError('too few points')
    expect(w.error).toBe('too few points')
    expect(w.captureCount).toBe(4)
  })

  it('reset() returns to a fresh setup wizard', () => {
    w.start(); w.recordCapture(4); w.solved(posResult); w.reset()
    expect(w.phase).toBe('setup')
    expect(w.captureCount).toBe(0)
    expect(w.position).toBeUndefined()
    expect(w.error).toBeUndefined()
  })
})
```

**Step 2: Run it — expect FAIL** — `cd frontend && npx vitest run src/lib/wizard/tcpTeach.test.ts` (`createTcpTeachWizard` not defined).

**Step 3: Write the store**

`frontend/src/lib/wizard/tcpTeach.svelte.ts`:

```ts
// TCP-calibration wizard: capture ≥4 pivot touches (capture_tcp_point) →
// teach_tcp_position (solves the tip offset + RMS residual AND persists to the
// tool's frame config) → optional explicit orientation. Pure logic; the panel
// wires the DoCommands. Unlike the frame-define wizard there is no frontend
// solver and teach_tcp_position commits atomically, so review is post-commit
// (the `taught` phase) with a Re-teach loop.
export type TcpPhase = 'setup' | 'capturing' | 'taught' | 'orientation' | 'done'

export interface TcpPosition {
  offset: { x: number; y: number; z: number }
  residual_rms: number
}

const MIN_CAPTURES = 4

export function createTcpTeachWizard() {
  let phase = $state<TcpPhase>('setup')
  let captureCount = $state(0)
  let position = $state<TcpPosition | undefined>(undefined)
  let error = $state<string | undefined>(undefined)

  return {
    get phase() { return phase },
    get captureCount() { return captureCount },
    get position() { return position },
    get error() { return error },
    get minCaptures() { return MIN_CAPTURES },
    get canSolve() { return captureCount >= MIN_CAPTURES },
    get done() { return phase === 'done' },

    start() { captureCount = 0; position = undefined; error = undefined; phase = 'capturing' },
    // buffer_len from the capture_tcp_point response is the authoritative count.
    recordCapture(bufferLen: number) {
      if (phase !== 'capturing') return
      captureCount = bufferLen
      error = undefined
    },
    solved(result: TcpPosition) { position = result; error = undefined; phase = 'taught' },
    reteach() { captureCount = 0; position = undefined; error = undefined; phase = 'capturing' },
    toOrientation() { if (phase === 'taught') phase = 'orientation' },
    finish() { phase = 'done' },
    setError(message: string) { error = message },
    reset() { phase = 'setup'; captureCount = 0; position = undefined; error = undefined },
  }
}
```

**Step 4: Run — expect PASS** — `npx vitest run src/lib/wizard/tcpTeach.test.ts`, then `npm run check` (0 errors) and `npm run test` (all green).

**Step 5: Commit**

```bash
git add frontend/src/lib/wizard/tcpTeach.svelte.ts frontend/src/lib/wizard/tcpTeach.test.ts
git commit -m "feat(wizard): TCP-teach store (pivot capture → solve → optional orientation)"
```

---

## Task 2: TCP teach wizard panel + mount

**Files:**
- Create: `frontend/src/panels/TcpTeachWizard.svelte`
- Modify: `frontend/src/App.svelte`

Read `frontend/src/panels/FrameDefineWizard.svelte` first — copy its structure exactly: `DashboardPortal` + `DashboardToggle` (distinct icon), `FloatingPanel`, prime token classes, the capture re-entrancy guard, `armState`/`motion` gating, and the `role="alert"` error line. Read `frontend/src/panels/TcpPanel.svelte` for the DoCommand wiring to port (it uses the same builders).

**Step 1: Create the panel**

`frontend/src/panels/TcpTeachWizard.svelte`. Script:

```svelte
<script lang="ts">
  // Guided TCP calibration, mounted in <Visualizer>'s children (see App.svelte).
  // Store lives in `wizard` (created in App.svelte) so panel state survives the
  // Bug-1 frameRevision remount. Drives the shipped pose-tracker TCP DoCommands.
  import { createResourceMutation, createResourceQuery } from '@viamrobotics/svelte-sdk'
  import { DashboardPortal, FloatingPanel } from '@viamrobotics/motion-tools'
  import { Icon } from '@viamrobotics/prime-core'
  import DashboardToggle from './DashboardToggle.svelte'
  import { poseTrackerClient } from '../lib/clients'
  import { selectedResource } from '../lib/resource.svelte'
  import { useMachineId } from '../lib/machine'
  import { armState } from '../lib/armState.svelte'
  import { motion } from '../lib/motion.svelte'
  import {
    captureTcpPoint, clearTcpBuffer, getTcpBuffer, teachTcpPosition, teachTcpOrientation,
    toCommandArgs,
    type CaptureResponse, type BufferResponse,
    type TeachTcpPositionResponse, type TeachTcpOrientationResponse,
  } from '../lib/poseTracker'
  import type { createTcpTeachWizard } from '../lib/wizard/tcpTeach.svelte'

  let { wizard }: { wizard: ReturnType<typeof createTcpTeachWizard> } = $props()

  const machineId = useMachineId()
  const pt = poseTrackerClient(machineId, () => selectedResource.name ?? '')
  const capture = createResourceMutation(pt, 'doCommand')
  const clear = createResourceMutation(pt, 'doCommand')
  const teachPos = createResourceMutation(pt, 'doCommand')
  const teachOri = createResourceMutation(pt, 'doCommand')
  // 4th options arg required (see FrameDefineWizard/ManageFramesPanel).
  const bufferQ = createResourceQuery(pt, 'doCommand', () => toCommandArgs(getTcpBuffer()), () => ({}))

  let open = $state(false)
  // Explicit orientation inputs (orientation-vector degrees). Default = flange axis.
  let oX = $state(0), oY = $state(0), oZ = $state(1), theta = $state(0)

  const points = $derived((bufferQ.data as BufferResponse | undefined)?.points ?? [])
  const busy = $derived(!armState.hasArm || motion.busy > 0)

  function errorMessage(err: unknown): string {
    return err instanceof Error ? err.message : String(err)
  }
  async function clearServerBuffer() {
    try { await clear.mutateAsync(toCommandArgs(clearTcpBuffer())) } catch { /* best-effort */ }
  }

  async function handleStart() { await clearServerBuffer(); wizard.start(); void bufferQ.refetch() }

  async function handleCapture() {
    if (capture.isPending) return
    try {
      const res = (await capture.mutateAsync(toCommandArgs(captureTcpPoint()))) as unknown as CaptureResponse
      wizard.recordCapture(res.buffer_len)
      void bufferQ.refetch()
    } catch (err) { wizard.setError(errorMessage(err)) }
  }

  async function handleTeachPosition() {
    try {
      const res = (await teachPos.mutateAsync(toCommandArgs(teachTcpPosition()))) as unknown as TeachTcpPositionResponse
      if (res?.committed) { wizard.solved({ offset: res.offset, residual_rms: res.residual_rms }); void bufferQ.refetch() }
      else wizard.setError('teach_tcp_position did not commit (check platform credentials)')
    } catch (err) {
      // Surfaces the backend's degeneracy error (too few / near-parallel poses);
      // the backend preserves the buffer so the operator can add more touches.
      wizard.setError(errorMessage(err))
    }
  }

  async function handleReteach() { await clearServerBuffer(); wizard.reteach(); void bufferQ.refetch() }

  async function handleTeachOrientation() {
    try {
      const res = (await teachOri.mutateAsync(
        toCommandArgs(teachTcpOrientation(oX, oY, oZ, theta)),
      )) as unknown as TeachTcpOrientationResponse
      if (res?.committed) wizard.finish()
      else wizard.setError('teach_tcp_orientation did not commit')
    } catch (err) { wizard.setError(errorMessage(err)) }
  }

  function handleTeachAgain() { oX = 0; oY = 0; oZ = 1; theta = 0; wizard.reset() }
  function fmt(n: number): string { return n.toFixed(2) }
</script>
```

Template — mirror `FrameDefineWizard`'s prime chrome exactly (dark primary button recipe, secondary button recipe, `min-h-11`, `focus-visible:ring-2`, `text-danger-dark` alert). Structure:

- `DashboardPortal` → `DashboardToggle active={open} label="Teach TCP" onclick={() => (open = !open)}` with a **distinct** `<Icon name="…" />`. Pick a valid prime-core icon distinct from `image-filter-center-focus` (frame) and `table` (manage) — try `"target"` or `"crosshairs-gps"`; **verify it exists** (svelte-check errors on an invalid `IconName`, as the ManageFrames `format-list-bulleted`→`table` fix showed). Report your choice.
- `FloatingPanel title="Teach TCP" bind:isOpen={open} defaultPosition={{ x: 688, y: 88 }} defaultSize={{ width: 320, height: 380 }} resizable` (position clears Define-Frame @24 and Manage @356).
- Body `div class="flex h-full flex-col gap-3 overflow-y-auto p-4"`, then per phase:
  - `wizard.phase === 'setup'`: instructions text ("Touch the same fixed point with the tool tip from ≥4 different arm orientations, capturing each.") + **Start** (primary). Show "Requires a configured arm." when `!armState.hasArm`.
  - `wizard.phase === 'capturing'` (`aria-live="polite"`): prompt "Jog the tip to the point, then Capture."; `{wizard.captureCount} / {wizard.minCaptures}+ captured`; optional buffered-points list from `points`; **Capture** button (`disabled={capture.isPending || busy}`); **Teach position** (primary, `disabled={!wizard.canSolve || teachPos.isPending}`).
  - `wizard.phase === 'taught'`: "TCP taught." + PROMINENT `TCP error: {fmt(wizard.position.residual_rms)} mm` and `offset ({fmt x},{fmt y},{fmt z})`. Buttons: **Re-teach** (secondary), **Set orientation** (secondary → `wizard.toOrientation()`), **Finish** (primary → `wizard.finish()`).
  - `wizard.phase === 'orientation'`: helper "Most tools can leave this at the flange orientation." + four number inputs bound to `oX/oY/oZ/theta` (labels Oˣ Oʸ Oᶻ θ) + **Save orientation** (`disabled={teachOri.isPending}`) + **Skip** (secondary → `wizard.finish()`).
  - `wizard.phase === 'done'`: "TCP calibrated." + **Teach again** (`handleTeachAgain`).
  - Always: `{#if wizard.error}<p class="text-danger-dark text-xs" role="alert">{wizard.error}</p>{/if}`.

Do NOT bump `frameRevision` anywhere here — the taught TCP updates the tool triad via `useFrames`' own config-revision refetch (no Visualizer remount needed).

**Step 2: Mount in App.svelte**

- Import: `import { createTcpTeachWizard } from './lib/wizard/tcpTeach.svelte'` and `import TcpTeachWizard from './panels/TcpTeachWizard.svelte'`.
- At script top, beside `const wizard = createFrameDefineWizard()` (OUTSIDE the `{#key frameRevision.value}` block so it survives the Bug-1 remount): `const tcpWizard = createTcpTeachWizard()`.
- In the `{#snippet children()}`, after `<FrameDefineWizard {wizard} />`: `<TcpTeachWizard wizard={tcpWizard} />`.

**Step 3: Verify** — from `frontend/`: `npm run check` (0 errors — confirms the icon name), `npm run test` (all green), `npm run build` (success).

**Step 4: Commit**

```bash
git add frontend/src/panels/TcpTeachWizard.svelte frontend/src/App.svelte
git commit -m "feat(frontend): guided TCP-teach wizard in the Visualizer chrome"
```

---

## Task 3: Hardware verification (user-run)

The 2026-07-10 design's pending checklist, now verifiable thanks to the Bug-2 fix:
1. Jog the tool tip to a fixed point from ≥4 varied orientations, Capture each, Teach position → confirm **`residual_rms` is small** (< 1–2 mm for careful touches).
2. After teaching, the **tool's frame triad moves to the taught tip** (before/after) and renders at the correct physical location — confirms the flange-vs-base frame-parent question empirically.
3. Re-teach on a tool that already has a non-zero `frame` → confirm no double-counted offset (converges to the same physical tip). If it double-counts, document that the tool `frame` must be cleared before re-teaching.
4. `teach_tcp_orientation` with a known rotation → writes `frame.orientation`, leaves translation intact.

Record results under the design doc's "Integration verification" heading. No code commit unless a bug surfaces (then TDD the fix).

---

## Definition of done
- [ ] `npm run check` / `npm run test` / `npm run build` green.
- [ ] Guided wizard: setup → capture ≥4 → teach position (RMS shown) → optional orientation → done, with a Re-teach loop.
- [ ] Distinct toolbar toggle; panel in prime chrome; arm/motion gating + capture re-entrancy guard.
- [ ] Hardware: small residual; taught TCP renders at the correct location; roadmap memory updated (C done, D next).
