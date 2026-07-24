# Frame Lifecycle (A+B) Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Finish the frame-define methods (`point`, `tcp_snapshot`) by extending the shipped guided wizard, and add a "Manage frames" panel (list / delete / clear-all).

**Architecture:** One guided wizard, method-parameterized: the shared rune store (`frameDefine.svelte.ts`) moves from magic step numbers to a semantic `setup | capturing | preview | committed` phase model with `requiredCaptures` (3 for `3point`, else 1). The `FrameDefinePlugin` scene overlay and `FrameDefineWizard` panel read that store; provisional triads are computed by a new pure helper matched exactly to the Go backend (`point` → identity orientation, `tcp_snapshot` → captured pose as-is). Frame management is a ported `FloatingPanel` reusing the legacy `FramePanel`'s tested list/delete/clear logic, restyled to prime chrome.

**Tech Stack:** Svelte 5 runes, `@viamrobotics/motion-tools` `<Visualizer>` + `FloatingPanel`/`DashboardPortal`, `@viamrobotics/svelte-sdk` resource queries/mutations, Threlte, Tailwind v4, Vitest.

**Design doc:** `docs/plans/2026-07-20-teach-pendant-roadmap-and-frame-lifecycle-design.md`

**Commands (run in `frontend/`):**
- Single test file: `npx vitest run src/path/to/file.test.ts`
- All tests: `npm run test`
- Typecheck: `npm run check`
- Build: `npm run build`

**Conventions (from the handoff — do not relitigate):** positions are **mm**, scene is **m** (`* 0.001`); orientation is a Viam **OrientationVector** (use `poseToSceneTransform`, NOT axis-angle); overlay `AxesHelper` needs `depthTest={false}`; non-interactive meshes use `raycast={() => null}`; there is **one** `get_arm_state` poll owner (`JogPanel`) — scene code reads `armState`, never adds a poll; source captured poses from the DoCommand **response**, not `armState`.

**Backend contract (fixed, already shipped — previews MUST match):**

| Method | Captures | Committed pose | Provisional preview |
|---|---|---|---|
| `3point` | 3 | `ComputeThreePoint` basis | basis triad (shipped) |
| `point` | 1 | origin, **identity orientation** (`ComputePoint`) | parent-aligned triad (identity quaternion) at origin |
| `tcp_snapshot` | 1 | captured pose **as-is** (`pose = buf[0]`) | triad at captured pose's full position + orientation |

---

## Task 1: Method-parameterized wizard store (phase model)

Behavior-preserving for `3point` (existing hardware-verified flow is unchanged); adds `method` + `requiredCaptures` and the `point`/`tcp_snapshot` capture counts. Consumers are updated in the SAME task so `npm run check` stays green.

**Files:**
- Modify: `frontend/src/lib/wizard/frameDefine.svelte.ts`
- Modify: `frontend/src/lib/wizard/frameDefine.test.ts`
- Modify (consumer, signature only): `frontend/src/panels/FrameDefineWizard.svelte`
- Modify (consumer, signature only): `frontend/src/scene/FrameDefinePlugin.svelte`

**Step 1: Rewrite the store test for the phase model + methods**

Replace the whole body of `frontend/src/lib/wizard/frameDefine.test.ts` with:

```ts
import { describe, it, expect, beforeEach } from 'vitest'
import { createFrameDefineWizard } from './frameDefine.svelte'
import type { PoseMap } from '../poseTracker'

const pose = (x: number, y: number, z: number): PoseMap =>
  ({ x, y, z, o_x: 0, o_y: 0, o_z: 1, theta: 0 })

describe('frameDefine wizard', () => {
  let w: ReturnType<typeof createFrameDefineWizard>
  beforeEach(() => { w = createFrameDefineWizard() })

  it('starts idle in the setup phase with no captures', () => {
    expect(w.phase).toBe('setup')
    expect(w.method).toBe('3point')
    expect(w.captures).toEqual([])
    expect(w.canCommit).toBe(false)
  })

  it('start() requires a non-blank name and enters the capturing phase', () => {
    expect(w.start('', '3point')).toBe(false)
    expect(w.phase).toBe('setup')
    expect(w.start('  table  ', '3point')).toBe(true)
    expect(w.name).toBe('table')   // trimmed
    expect(w.method).toBe('3point')
    expect(w.phase).toBe('capturing')
  })

  it('3point requires 3 captures before preview', () => {
    w.start('f', '3point')
    expect(w.requiredCaptures).toBe(3)
    w.recordCapture(pose(0, 0, 0)); expect(w.phase).toBe('capturing'); expect(w.captureIndex).toBe(1)
    w.recordCapture(pose(1, 0, 0)); expect(w.phase).toBe('capturing'); expect(w.captureIndex).toBe(2)
    expect(w.canCommit).toBe(false)
    w.recordCapture(pose(0, 1, 0)); expect(w.phase).toBe('preview'); expect(w.canCommit).toBe(true)
  })

  it('point needs exactly 1 capture and goes straight to preview', () => {
    w.start('f', 'point')
    expect(w.requiredCaptures).toBe(1)
    w.recordCapture(pose(2, 3, 4))
    expect(w.phase).toBe('preview')
    expect(w.canCommit).toBe(true)
    expect(w.captures.length).toBe(1)
  })

  it('tcp_snapshot needs exactly 1 capture and goes straight to preview', () => {
    w.start('f', 'tcp_snapshot')
    expect(w.requiredCaptures).toBe(1)
    w.recordCapture(pose(2, 3, 4))
    expect(w.phase).toBe('preview')
    expect(w.canCommit).toBe(true)
  })

  it('ignores captures once out of the capturing phase', () => {
    w.start('f', 'point')
    w.recordCapture(pose(0, 0, 0))   // -> preview
    w.recordCapture(pose(9, 9, 9))   // dropped
    expect(w.captures.length).toBe(1)
  })

  it('recapture() clears captures and returns to capturing', () => {
    w.start('f', '3point')
    w.recordCapture(pose(0, 0, 0)); w.recordCapture(pose(1, 0, 0)); w.recordCapture(pose(0, 1, 0))
    w.recapture()
    expect(w.captures).toEqual([]); expect(w.phase).toBe('capturing'); expect(w.error).toBeUndefined()
  })

  it('setError surfaces a backend failure without losing captures', () => {
    w.start('f', '3point'); w.recordCapture(pose(0, 0, 0))
    w.setError('collinear points: X axis undefined')
    expect(w.error).toBe('collinear points: X axis undefined')
    expect(w.captures.length).toBe(1)
  })

  it('committed() advances to the committed phase', () => {
    w.start('f', 'point')
    w.recordCapture(pose(0, 0, 0))
    w.committed()
    expect(w.phase).toBe('committed'); expect(w.done).toBe(true)
  })

  it('reset() returns to a fresh setup wizard (method back to 3point)', () => {
    w.start('f', 'tcp_snapshot'); w.recordCapture(pose(0, 0, 0)); w.reset()
    expect(w.phase).toBe('setup'); expect(w.method).toBe('3point'); expect(w.name).toBe(''); expect(w.captures).toEqual([])
  })
})
```

**Step 2: Run the test to verify it fails**

Run: `npx vitest run src/lib/wizard/frameDefine.test.ts`
Expected: FAIL (`w.phase` is undefined, `start` takes one arg, etc.)

**Step 3: Rewrite the store**

Replace the whole body of `frontend/src/lib/wizard/frameDefine.svelte.ts` with:

```ts
import type { FrameMethod, PoseMap } from '../poseTracker'

// Phases: setup (choose method + name) · capturing (jog + Capture ×N) ·
// preview (review the provisional triad, then Commit) · committed.
// Pure logic; the panel wires DoCommands and the scene plugin reads
// `phase`/`method`/`captures` to draw the matching spatial story.
export type WizardPhase = 'setup' | 'capturing' | 'preview' | 'committed'

export function createFrameDefineWizard() {
  let phase = $state<WizardPhase>('setup')
  let method = $state<FrameMethod>('3point')
  let name = $state('')
  let captures = $state<PoseMap[]>([])
  let error = $state<string | undefined>(undefined)

  const required = () => (method === '3point' ? 3 : 1)

  return {
    get phase() { return phase },
    get method() { return method },
    get name() { return name },
    get captures() { return captures },
    get error() { return error },
    get requiredCaptures() { return required() },
    // 0-based index of the point being captured next (also = count captured so far).
    get captureIndex() { return captures.length },
    get canCommit() { return captures.length === required() },
    get done() { return phase === 'committed' },

    start(rawName: string, m: FrameMethod): boolean {
      const trimmed = rawName.trim()
      if (!trimmed) return false
      name = trimmed; method = m; captures = []; error = undefined; phase = 'capturing'
      return true
    },
    recordCapture(pose: PoseMap) {
      if (phase !== 'capturing') return
      captures = [...captures, pose]
      error = undefined
      if (captures.length >= required()) phase = 'preview'
    },
    recapture() { captures = []; error = undefined; phase = 'capturing' },
    setError(message: string) { error = message },
    committed() { phase = 'committed' },
    reset() { phase = 'setup'; method = '3point'; name = ''; captures = []; error = undefined },
  }
}
```

**Step 4: Update the panel's store references (behavior-preserving)**

In `frontend/src/panels/FrameDefineWizard.svelte`, make the minimal changes so it still compiles and behaves as today (method selector + method-aware prompts come in Task 4). Full method-aware rewrite is Task 4 — here just keep `3point` working under the phase API:

- `handleStart`: change `wizard.start(nameInput)` → `wizard.start(nameInput, '3point')`.
- Template guards, replace step numbers with phases:
  - `{#if wizard.step === 0}` → `{#if wizard.phase === 'setup'}`
  - `{:else if wizard.step >= 1 && wizard.step <= 3}` → `{:else if wizard.phase === 'capturing'}`
  - `{:else if wizard.step === 4}` → `{:else if wizard.phase === 'preview'}`
  - `{:else if wizard.step === 5}` → `{:else if wizard.phase === 'committed'}`
- Capture prompt line currently `CAPTURE_PROMPTS[wizard.step]` → `CAPTURE_PROMPTS[wizard.captureIndex + 1]` (keep the existing 1/2/3-keyed map for now; Task 4 replaces the map).
- Progress line `{wizard.captures.length} / 3 captured` → `{wizard.captures.length} / {wizard.requiredCaptures} captured`.

**Step 5: Update the plugin's store references (behavior-preserving)**

In `frontend/src/scene/FrameDefinePlugin.svelte`, replace step-number reads with phases (method-aware previews come in Task 3):

- markers guard `{#if wizard.step >= 1 && wizard.step <= 4}` → `{#if wizard.phase === 'capturing' || wizard.phase === 'preview'}`
- `provisionalBasis` derived: `wizard.step === 4 && wizard.captures.length === 3` → `wizard.phase === 'preview' && wizard.captures.length === 3`
- `previewLine` derived: `(wizard.step === 2 || wizard.step === 3)` → `(wizard.phase === 'capturing' && wizard.captures.length >= 1)`

**Step 6: Run typecheck + tests**

Run: `npx vitest run src/lib/wizard/frameDefine.test.ts` → Expected: PASS
Run: `npm run check` → Expected: 0 errors
Run: `npm run test` → Expected: all green

**Step 7: Commit**

```bash
git add frontend/src/lib/wizard/frameDefine.svelte.ts frontend/src/lib/wizard/frameDefine.test.ts frontend/src/panels/FrameDefineWizard.svelte frontend/src/scene/FrameDefinePlugin.svelte
git commit -m "refactor(wizard): method-parameterized frame-define store (phase model)"
```

---

## Task 2: Provisional-triad helper (pure, per-method)

A pure function that maps `(method, captures)` → scene triad, matched to the backend contract. Extracted so the mapping is unit-testable (the plugin is not).

**Files:**
- Create: `frontend/src/scene/provisionalTriad.ts`
- Create: `frontend/src/scene/provisionalTriad.test.ts`

**Step 1: Write the failing test**

`frontend/src/scene/provisionalTriad.test.ts`:

```ts
import { describe, it, expect } from 'vitest'
import { provisionalTriad } from './provisionalTriad'
import type { PoseMap } from '../lib/poseTracker'

const pose = (p: Partial<PoseMap>): PoseMap =>
  ({ x: 0, y: 0, z: 0, o_x: 0, o_y: 0, o_z: 1, theta: 0, ...p })

describe('provisionalTriad', () => {
  it('returns null before enough captures', () => {
    expect(provisionalTriad('3point', [])).toBeNull()
    expect(provisionalTriad('point', [])).toBeNull()
    expect(provisionalTriad('3point', [pose({}), pose({})])).toBeNull()
  })

  it('point: origin scaled to meters, identity orientation', () => {
    const t = provisionalTriad('point', [pose({ x: 100, y: 200, z: 300 })])!
    expect(t.position[0]).toBeCloseTo(0.1)
    expect(t.position[1]).toBeCloseTo(0.2)
    expect(t.position[2]).toBeCloseTo(0.3)
    expect(t.quaternion).toEqual([0, 0, 0, 1])
  })

  it('tcp_snapshot: uses the captured pose position AND orientation', () => {
    // o_z=1, theta=180 -> a 180deg rotation about Z (w component ~ 0).
    const t = provisionalTriad('tcp_snapshot', [pose({ x: 10, y: 0, z: 0, o_z: 1, theta: 180 })])!
    expect(t.position[0]).toBeCloseTo(0.01)
    expect(Math.abs(t.quaternion[3])).toBeLessThan(1e-6) // w ~ 0 for a 180deg turn
  })

  it('3point: origin at P0, non-identity basis (X toward P1)', () => {
    const t = provisionalTriad('3point', [
      pose({ x: 0, y: 0, z: 0 }),
      pose({ x: 1000, y: 0, z: 0 }), // +X
      pose({ x: 0, y: 1000, z: 0 }), // +XY plane
    ])!
    expect(t.position).toEqual([0, 0, 0])
    // basis should not be identity (a real rotation is present)
    expect(t.quaternion).not.toEqual([0, 0, 0, 1])
  })
})
```

**Step 2: Run test to verify it fails**

Run: `npx vitest run src/scene/provisionalTriad.test.ts`
Expected: FAIL (`provisionalTriad` not defined)

**Step 3: Write the helper**

`frontend/src/scene/provisionalTriad.ts`:

```ts
// Pure map from (method, captures) to the provisional frame triad drawn in the
// scene during the wizard's preview phase. Matched EXACTLY to the Go backend
// (frames/compute.go, models/posetracker/docommand.go):
//   3point       -> ComputeThreePoint basis (origin P0, +X->P1, P2 in +XY plane)
//   point        -> ComputePoint: origin only, IDENTITY orientation
//   tcp_snapshot -> the captured pose as-is (position + orientation)
import type { FrameMethod, PoseMap } from '../lib/poseTracker'
import { threePointBasis, basisToQuaternion } from './frameGeometry'
import { poseToSceneTransform, type SceneTransform } from './poseTransform'

const MM_TO_M = 0.001

export function provisionalTriad(method: FrameMethod, captures: PoseMap[]): SceneTransform | null {
  if (method === '3point') {
    if (captures.length !== 3) return null
    const basis = threePointBasis(captures[0], captures[1], captures[2])
    if (!basis) return null
    return {
      position: [basis.origin[0] * MM_TO_M, basis.origin[1] * MM_TO_M, basis.origin[2] * MM_TO_M],
      quaternion: basisToQuaternion(basis),
    }
  }

  if (captures.length !== 1) return null
  const c = captures[0]

  if (method === 'point') {
    // ComputePoint: identity orientation at the captured origin.
    return { position: [c.x * MM_TO_M, c.y * MM_TO_M, c.z * MM_TO_M], quaternion: [0, 0, 0, 1] }
  }

  // tcp_snapshot: the captured TCP pose, verbatim.
  return poseToSceneTransform(c)
}
```

> Note: confirm `frameGeometry.ts` exports `threePointBasis`, `basisToQuaternion`, and a `Basis` with an `origin: [number,number,number]` field, and that `poseTransform.ts` exports `SceneTransform`. All three exist (verified). If `threePointBasis` accepts `PoseMap` positionally it already reads `.x/.y/.z`; pass the captures directly.

**Step 4: Run test to verify it passes**

Run: `npx vitest run src/scene/provisionalTriad.test.ts`
Expected: PASS

**Step 5: Commit**

```bash
git add frontend/src/scene/provisionalTriad.ts frontend/src/scene/provisionalTriad.test.ts
git commit -m "feat(scene): pure provisional-triad helper matched to backend methods"
```

---

## Task 3: Method-aware scene previews in the plugin

Route the plugin's provisional triad through the Task 2 helper and keep the live preview line 3point-only.

**Files:**
- Modify: `frontend/src/scene/FrameDefinePlugin.svelte`

**Step 1: Replace the provisional-triad derivation**

Delete the `provisionalBasis` + `provisionalTriad` `$derived` blocks and their `threePointBasis`/`basisToQuaternion` import. Import and use the helper:

```svelte
import { provisionalTriad } from './provisionalTriad'
```

```svelte
const triad = $derived(
  wizard.phase === 'preview' ? provisionalTriad(wizard.method, wizard.captures) : null,
)
```

And the render block:

```svelte
{#if triad}
  <AxesHelper
    position={triad.position}
    quaternion={triad.quaternion}
    length={0.12}
    width={3}
    depthTest={false}
  />
{/if}
```

**Step 2: Scope the live preview line to 3point**

The origin→live-TCP guide line is only meaningful when a subsequent axis point is still being placed (3point, after P0). Update `previewLine`:

```svelte
const previewLine = $derived(
  wizard.method === '3point' &&
  wizard.phase === 'capturing' &&
  wizard.captures.length >= 1 &&
  armState.pose
    ? [
        new Vector3(...toScenePosition(wizard.captures[0])),
        new Vector3(...toScenePosition(armState.pose)),
      ]
    : null,
)
```

**Step 3: Typecheck + build**

Run: `npm run check` → Expected: 0 errors
Run: `npm run build` → Expected: success (Threlte/GLSL compile)

**Step 4: Commit**

```bash
git add frontend/src/scene/FrameDefinePlugin.svelte
git commit -m "feat(scene): method-aware provisional frame previews"
```

---

## Task 4: Method selector + method-aware prompts + collision warning

The wizard panel becomes fully multi-method: pick a method in setup, method-aware capture prompts, and a name-collision warning (ported from the retired define-form).

**Files:**
- Modify: `frontend/src/panels/FrameDefineWizard.svelte`

**Step 1: Add method-selection state + copy**

In `<script>`, add imports and state:

```svelte
import { createResourceQuery } from '@viamrobotics/svelte-sdk'
import {
  // ...existing imports...
  listFrames,
  type FrameMethod,
  type FramesResponse,
} from '../lib/poseTracker'
```

```svelte
let methodInput = $state<FrameMethod>('3point')

const METHOD_LABELS: Record<FrameMethod, string> = {
  '3point': '3-point',
  point: 'Single point',
  tcp_snapshot: 'TCP snapshot',
}
const METHOD_HINTS: Record<FrameMethod, string> = {
  '3point': 'origin, +X, and a point in the +XY plane',
  point: 'one point — origin only, no rotation',
  tcp_snapshot: 'one point — uses the tool pose as-is',
}
const METHODS: FrameMethod[] = ['3point', 'point', 'tcp_snapshot']

const CAPTURE_PROMPTS: Record<FrameMethod, string[]> = {
  '3point': [
    "Jog the tool to the frame's ORIGIN, then Capture.",
    'Jog along the intended +X axis, then Capture.',
    'Jog to a point in the +Y half-plane, then Capture.',
  ],
  point: ["Jog the tool to the frame's origin, then Capture."],
  tcp_snapshot: ['Jog the tool to the exact pose to snapshot, then Capture.'],
}

// Name-collision warning (moved here from the retired define-form): defining
// with an existing name overwrites it — surface that before Commit.
const framesQuery = createResourceQuery(
  pt, 'doCommand', () => toCommandArgs(listFrames()), () => ({}),
)
const existingNames = $derived(
  Object.keys((framesQuery.data as FramesResponse | undefined)?.frames ?? {}),
)
const willReplace = $derived(existingNames.includes(nameInput.trim()))
```

Delete the old `CAPTURE_PROMPTS: Record<number, string>` map.

**Step 2: Pass the method into start, refetch frames after commit**

```svelte
async function handleStart() {
  await clearServerBuffer()
  wizard.start(nameInput, methodInput)
}
```

In `handleCommit`, after `wizard.committed()` on success, add `void framesQuery.refetch()` so the collision list stays fresh for the next "Define another".

**Step 3: Render the method selector + warning in the setup phase**

Inside the `{#if wizard.phase === 'setup'}` form, replace the static `<p>Method: 3-point</p>` with a radio group and a collision warning:

```svelte
<fieldset class="flex flex-col gap-1.5">
  <legend class="text-subtle-1 text-sm">Method</legend>
  {#each METHODS as m (m)}
    <label class="text-gray-9 flex items-center gap-2 text-sm">
      <input type="radio" name="frame-method" value={m} bind:group={methodInput} />
      <span>{METHOD_LABELS[m]}</span>
      <span class="text-subtle-2 text-xs">— {METHOD_HINTS[m]}</span>
    </label>
  {/each}
</fieldset>

{#if willReplace}
  <p class="text-subtle-1 text-xs" role="status">
    A frame named “{nameInput.trim()}” exists — committing will replace it.
  </p>
{/if}
```

**Step 4: Method-aware capture prompt**

In the `{:else if wizard.phase === 'capturing'}` block, the prompt line becomes:

```svelte
<p class="text-gray-9 text-sm">{CAPTURE_PROMPTS[wizard.method][wizard.captureIndex]}</p>
<p class="text-subtle-1 text-xs">{wizard.captures.length} / {wizard.requiredCaptures} captured</p>
```

(The step-4 change to the prompt from Task 1 is now superseded by the method-keyed map.)

**Step 5: Typecheck + build**

Run: `npm run check` → Expected: 0 errors
Run: `npm run build` → Expected: success

Manual smoke (optional, `npm run dev`): with a connected machine, run `point` and `tcp_snapshot` through setup → capture → preview and confirm a triad renders in preview (identity-aligned for `point`, tool-aligned for `tcp_snapshot`).

**Step 6: Commit**

```bash
git add frontend/src/panels/FrameDefineWizard.svelte
git commit -m "feat(frontend): multi-method frame-define wizard + collision warning"
```

---

## Task 5: Manage frames panel (port + prime + toggle + mount)

Port the legacy `FramePanel` list/delete/clear logic into a prime-chrome `FloatingPanel` with a toolbar toggle. Drop the define-form (the wizard owns defining now).

**Files:**
- Create: `frontend/src/panels/ManageFramesPanel.svelte`
- Modify: `frontend/src/App.svelte`

**Step 1: Create the panel**

`frontend/src/panels/ManageFramesPanel.svelte` — reuse the legacy `FramePanel`'s query/mutation wiring and confirm-before-destroy state machine (`frontend/src/panels/FramePanel.svelte` lines 1–106 for logic; drop everything about `define`, `method`, `name`, `willReplace`, `lastResult`). Structure:

```svelte
<script lang="ts">
  import { createResourceMutation, createResourceQuery } from '@viamrobotics/svelte-sdk'
  import { DashboardPortal, FloatingPanel } from '@viamrobotics/motion-tools'
  import { Icon } from '@viamrobotics/prime-core'
  import DashboardToggle from './DashboardToggle.svelte'
  import { poseTrackerClient } from '../lib/clients'
  import { selectedResource } from '../lib/resource.svelte'
  import { useMachineId } from '../lib/machine'
  import {
    clearFrames, deleteFrame, listFrames, toCommandArgs, type FramesResponse,
  } from '../lib/poseTracker'

  const machineId = useMachineId()
  const pt = poseTrackerClient(machineId, () => selectedResource.name ?? '')
  const frames = createResourceQuery(pt, 'doCommand', () => toCommandArgs(listFrames()), () => ({}))
  const del = createResourceMutation(pt, 'doCommand')
  const clearAll = createResourceMutation(pt, 'doCommand')

  let open = $state(false)

  const frameEntries = $derived(
    Object.entries((frames.data as FramesResponse | undefined)?.frames ?? {}),
  )

  type PendingConfirm = { kind: 'delete' | 'clearAll'; name?: string } | null
  let pendingConfirm = $state<PendingConfirm>(null)
  const confirmBusy = $derived(del.isPending || clearAll.isPending)

  async function runConfirm() {
    const c = pendingConfirm
    if (!c) return
    try {
      if (c.kind === 'delete') {
        if (!c.name) return
        await del.mutateAsync(toCommandArgs(deleteFrame(c.name)))
      } else {
        await clearAll.mutateAsync(toCommandArgs(clearFrames()))
      }
      await frames.refetch()
      pendingConfirm = null
    } catch {
      // Surfaced via del.error / clearAll.error below.
    }
  }

  function focusOnMount(node: HTMLElement) { node.focus() }
  function fmt(n: number): string { return n.toFixed(2) }
</script>

<DashboardPortal>
  <DashboardToggle active={open} label="Manage frames" onclick={() => (open = !open)}>
    {#snippet children()}
      <Icon name="format-list-bulleted" />
    {/snippet}
  </DashboardToggle>
</DashboardPortal>

<FloatingPanel
  title="Manage Frames"
  bind:isOpen={open}
  defaultPosition={{ x: 356, y: 88 }}
  defaultSize={{ width: 340, height: 360 }}
  resizable
>
  <div class="flex h-full flex-col gap-3 overflow-y-auto p-4">
    <div class="flex items-center justify-between">
      <h3 class="text-gray-9 text-sm font-medium">Committed frames</h3>
      <button
        type="button"
        onclick={() => (pendingConfirm = { kind: 'clearAll' })}
        disabled={confirmBusy || frameEntries.length === 0}
        class="border-medium text-gray-9 hover:border-gray-6 min-h-11 rounded-md border bg-white px-3 text-sm focus:outline-none focus-visible:ring-2 focus-visible:ring-gray-9/30 disabled:cursor-not-allowed disabled:text-disabled-dark"
      >
        Clear all
      </button>
    </div>

    {#if pendingConfirm}
      <div class="border-danger-dark bg-danger-light/40 flex flex-col gap-2 rounded-md border p-3" role="alert">
        <p class="text-gray-9 text-sm">
          {#if pendingConfirm.kind === 'delete'}
            Delete frame “{pendingConfirm.name}”? This can’t be undone.
          {:else}
            Clear all {frameEntries.length} frames? This can’t be undone.
          {/if}
        </p>
        <div class="flex gap-2">
          <button
            type="button"
            onclick={runConfirm}
            disabled={confirmBusy}
            class="border-danger-dark bg-danger-dark min-h-11 rounded-md border px-4 text-sm font-medium text-white focus:outline-none focus-visible:ring-2 disabled:cursor-not-allowed disabled:opacity-60"
          >
            {#if confirmBusy}{pendingConfirm.kind === 'delete' ? 'Deleting…' : 'Clearing…'}{:else}{pendingConfirm.kind === 'delete' ? 'Delete' : 'Clear all'}{/if}
          </button>
          <button
            type="button"
            onclick={() => (pendingConfirm = null)}
            disabled={confirmBusy}
            use:focusOnMount
            class="border-medium text-gray-9 hover:border-gray-6 min-h-11 rounded-md border bg-white px-4 text-sm focus:outline-none focus-visible:ring-2 focus-visible:ring-gray-9/30"
          >
            Cancel
          </button>
        </div>
      </div>
    {/if}

    {#if frames.error}<p class="text-danger-dark text-xs" role="alert">{frames.error.message}</p>{/if}
    {#if del.error}<p class="text-danger-dark text-xs" role="alert">{del.error.message}</p>{/if}
    {#if clearAll.error}<p class="text-danger-dark text-xs" role="alert">{clearAll.error.message}</p>{/if}

    {#if frameEntries.length === 0}
      <p class="text-subtle-1 text-sm">No frames defined yet.</p>
    {:else}
      <table class="w-full text-sm tabular-nums">
        <thead>
          <tr class="text-subtle-1 text-left text-xs">
            <th class="py-1">Name</th><th class="py-1 text-right">X</th><th class="py-1 text-right">Y</th><th class="py-1 text-right">Z</th><th><span class="sr-only">Actions</span></th>
          </tr>
        </thead>
        <tbody>
          {#each frameEntries as [frameName, pose] (frameName)}
            <tr class="border-light border-t">
              <td class="text-gray-9 py-1.5">{frameName}</td>
              <td class="text-gray-9 py-1.5 text-right">{fmt(pose.x)}</td>
              <td class="text-gray-9 py-1.5 text-right">{fmt(pose.y)}</td>
              <td class="text-gray-9 py-1.5 text-right">{fmt(pose.z)}</td>
              <td class="py-1.5 text-right">
                <button
                  type="button"
                  onclick={() => (pendingConfirm = { kind: 'delete', name: frameName })}
                  disabled={confirmBusy}
                  aria-label="Delete {frameName}"
                  class="text-danger-dark hover:underline"
                >Delete</button>
              </td>
            </tr>
          {/each}
        </tbody>
      </table>
    {/if}
  </div>
</FloatingPanel>
```

> Verify the Tailwind color tokens against a shipped prime panel (`FrameDefineWizard`, `JogPanel`): use whatever danger token those use (e.g. `text-danger-dark`, `border-danger-dark`, `bg-danger-light`). If a token doesn't exist, match the closest one already in use — don't introduce new CSS.

**Step 2: Mount in App.svelte**

In `frontend/src/App.svelte`, import and mount inside the `{#snippet children()}`:

```svelte
import ManageFramesPanel from './panels/ManageFramesPanel.svelte'
```

```svelte
{#snippet children()}
  <TcpTriad />
  <FrameDefinePlugin {wizard} />
  <FrameDefineWizard {wizard} />
  <ManageFramesPanel />
  <JogPanel />
{/snippet}
```

**Step 3: Typecheck + build**

Run: `npm run check` → Expected: 0 errors
Run: `npm run build` → Expected: success

**Step 4: Commit**

```bash
git add frontend/src/panels/ManageFramesPanel.svelte frontend/src/App.svelte
git commit -m "feat(frontend): Manage Frames panel (list/delete/clear) in the Visualizer chrome"
```

---

## Task 6: Scene-select feasibility spike (time-boxed, ≤30 min — document only)

Not a feature commit. Determine whether clicking a taught-frame triad in the 3D scene can drive selection/delete, given WSS entities are `removable: false`.

**Steps:**
1. Read how the Visualizer exposes selection + the Details portal (search `@viamrobotics/motion-tools` for a selection store/entity API and any `removable`/`Details` usage).
2. Check whether a world_state_store transform entity can be made selectable and whether a custom action (Delete) can be attached to the Details portal for it.
3. Write findings (feasible & cheap / feasible but costly / blocked) into `docs/plans/2026-07-20-teach-pendant-remaining-work-handoff.md` under a new "Scene-select spike" note, and update the roadmap memory.
4. **Do NOT implement** unless the spike concludes "cheap" — then add a follow-up task. Otherwise management stays panel-driven (Task 5), revisited in track E.

**Commit:** docs only.

```bash
git add docs/plans/2026-07-20-teach-pendant-remaining-work-handoff.md
git commit -m "docs: scene-select feasibility spike findings"
```

---

## Task 7: Hardware verification + PR update

The slice was hardware-verified for `3point`; verify the two new methods end-to-end against a real machine.

**Steps:**
1. `npm run build`, deploy the module to the test machine (per the existing dev/reload workflow).
2. **`point`**: jog → Capture ×1 → preview shows a parent-aligned triad at the origin → Commit → the frame renders live and persists (reload confirms).
3. **`tcp_snapshot`**: jog to a tilted tool pose → Capture ×1 → preview triad matches the tool orientation → Commit → renders live + persists.
4. **Manage panel**: the committed frames list them; delete one and clear-all both persist (reload confirms).
5. **Collision**: re-define an existing name → the warning shows in setup → commit replaces (no duplicate).
6. Note any divergence between the provisional preview and the committed WSS triad (a mismatch means the preview math disagrees with the backend — file it).
7. Update draft PR #6 description with the A+B additions; update roadmap memory to mark A+B done and C next.

**No code commit** unless verification surfaces a bug (then TDD the fix as its own task).

---

## Definition of done (milestone 1)

- [ ] `npm run check`, `npm run test`, `npm run build` all green.
- [ ] All three frame methods drive the guided wizard end-to-end with correct provisional previews.
- [ ] Manage Frames panel lists/deletes/clears taught frames with confirms, in prime chrome, toggled from the toolbar.
- [ ] Name-collision warning shows in the wizard setup phase.
- [ ] `point` + `tcp_snapshot` hardware-verified (capture → preview → commit → live render → persist).
- [ ] Scene-select spike documented; roadmap memory updated (A+B done, C next).
