# Visualizer Teach Pendant — Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Rebuild the `teach-frames` frontend as a 3D teach pendant — embed the
`@viamrobotics/motion-tools` `<Visualizer>`, and stage a 3-point Frame-Define
wizard in the live scene, as a vertical slice that proves the pattern.

**Architecture:** Hybrid. `<Visualizer partID>` hosts the scene (live arm from
the frame system; taught-frame triads via our existing `world_state_store`).
Our contribution mounts inside its `children` snippet: scene-coupled plugins
(3D markers, previews, triads) and floating panels (Jog, wizard) that drive the
existing pose-tracker `DoCommand` API. Backend is untouched — frontend-only.

**Tech Stack:** Svelte 5 (runes) · Vite · `@viamrobotics/motion-tools` ·
`@viamrobotics/svelte-sdk` · `@viamrobotics/sdk` · Threlte/three.js (via the
package) · `prime-core` (via the package) · vitest.

**Design doc:** `docs/plans/2026-07-20-visualizer-teach-pendant-design.md` — read
it first for the full rationale and the confirmed motion-tools API facts.

---

## Conventions for the executor

- Work in `frontend/`. Dev server: `cd frontend && npm run dev`. Type-check:
  `npm run check`. Unit tests: `npm run test`. Build: `npm run build`.
- **TDD applies to pure logic only.** The wizard state machine (Task 3) is pure
  and is written test-first. The 3D scene objects, panels, and the embed itself
  are integration surfaces that WebGL/live-machine make dishonest to unit-test —
  those tasks are verified with the **`verify` skill** driving the real flow,
  exactly as the design's Section 4 specifies. Do not fabricate shallow tests
  for 3D components to satisfy a TDD reflex.
- **Package-internal specifics** (exact `AxesHelper` import subpath, `prime-core`
  component names, worker handling) are resolved empirically in Task 1 once the
  package is installed. Where this plan marks `⚠ confirm against installed
  package`, check the real export before hardcoding.
- Commit after each task with the message shown. Frequent, small commits.
- Existing reused files — do not modify: `lib/machine.ts`, `lib/clients.ts`,
  `lib/poseTracker.ts`, `lib/armState.svelte.ts`, `lib/captureBuffer.svelte.ts`,
  `lib/resource.svelte.ts`, `lib/motion.svelte.ts`, `panels/ResourcePicker.svelte`.

---

## Task 1: Install motion-tools and embed a minimal live Visualizer

Retires the top build risk (workers under `base: './'`, bundle size, peer deps)
before any UI is built. End state: picking a pose-tracker resource shows the
live arm in a 3D scene.

**Files:**
- Modify: `frontend/package.json` (deps)
- Create: `frontend/.npmrc`
- Modify: `frontend/src/App.svelte` (provider + Visualizer host)
- Modify: `frontend/src/main.ts` (import package CSS)
- Modify: `frontend/vite.config.ts` (only if workers need it — see Step 5)

**Step 1: Add the dependency and peer auto-install**

Create `frontend/.npmrc`:

```
auto-install-peers=true
```

Run:

```bash
cd frontend && npm install @viamrobotics/motion-tools
```

Expected: installs plus a tree of peers (Threlte, three, prime-core, zag,
tweakpane, svelte-tweakpane-ui, …). If npm reports missing peers it did not
auto-add, install each by name until `npm ls` is clean of `UNMET PEER`.

**Step 2: Confirm the package's exports you will use**

Run:

```bash
node -e "console.log(Object.keys(require('@viamrobotics/motion-tools')))" 2>/dev/null || true
cat frontend/node_modules/@viamrobotics/motion-tools/package.json | grep -A40 '"exports"'
```

Note the entry that provides `Visualizer` (root: `@viamrobotics/motion-tools`)
and the `/lib` subpath that provides `AxesHelper`, `FloatingPanel`,
`DashboardPortal`, `useQuery`, `useTrait`. Record any CSS file the package ships
(e.g. a `dist/*.css`) — you import it in Step 4. `⚠ confirm against installed
package`.

**Step 3: Replace App.svelte with the provider/host shell**

`frontend/src/App.svelte` — keep the machine-identity + error handling and the
`ResourcePicker` gate; replace the two-column layout with the Visualizer host.
The `dialConfigs` key and `partID` are both `machine.id` (this is what already
works with svelte-sdk's resource clients today).

```svelte
<script lang="ts">
  import { ViamProvider } from '@viamrobotics/svelte-sdk'
  import { Visualizer } from '@viamrobotics/motion-tools'
  import { currentMachine, provideMachineId, type MachineIdentity } from './lib/machine'
  import { selectedResource } from './lib/resource.svelte'
  import ResourcePicker from './panels/ResourcePicker.svelte'

  let machine: MachineIdentity | undefined
  let error: string | undefined

  try {
    machine = currentMachine()
    provideMachineId(machine.id)
  } catch (err) {
    error = err instanceof Error ? err.message : String(err)
  }
</script>

{#if error}
  <main class="error-screen"><p class="error">Unable to connect to machine: {error}</p></main>
{:else if machine}
  <ViamProvider dialConfigs={{ [machine.id]: machine.dialConf }}>
    {#if selectedResource.name}
      <div class="viz-host">
        <Visualizer partID={machine.id}>
          {#snippet children()}
            <!-- scene plugins + panels mount here in later tasks -->
          {/snippet}
        </Visualizer>
      </div>
    {:else}
      <ResourcePicker />
    {/if}
  </ViamProvider>
{/if}

<style>
  :global(html, body) { margin: 0; height: 100%; }
  :global(*) { box-sizing: border-box; }
  /* Visualizer stretches to fill an explicitly-sized parent. */
  .viz-host { position: fixed; inset: 0; }
  .error-screen { padding: 1rem; }
  .error { color: #ff8080; padding: 1rem; }
</style>
```

Note: `tokens.css` is no longer the app theme (the Visualizer brings its own).
Leave the file for now; it is removed when the last panel using it is re-homed
(Task 6).

**Step 4: Import the package CSS**

In `frontend/src/main.ts`, add the package stylesheet import found in Step 2
(above the `App` import). `⚠ confirm against installed package` — exact path.
Keep `import './styles/tokens.css'` until Task 6.

**Step 5: Run the dev server and verify the live arm renders**

Use the **`verify` skill**. Drive it:

```bash
cd frontend && npm run dev
```

Open the app against a machine with a configured pose-tracker + arm (the `.env`
machine, or `viam module local-app-testing`). Pick the resource. Confirm:
- The 3D scene loads (grid + camera controls).
- The **live arm renders** and follows the real arm.
- Any already-taught frames appear as triads (native `world_state_store`).
- The browser console has no worker-load 404s.

If workers 404 under the relative `base: './'`, add to `vite.config.ts`:

```ts
worker: { format: 'es' },
// and if a specific asset type is emitted unhashed/unresolved:
// assetsInclude: ['**/*.worker.js'],
```

Re-verify. Record the production bundle size for the go/no-go:

```bash
npm run build && du -sh dist && ls -lh dist/assets | sort -k5 -h | tail -5
```

**Step 6: Commit**

```bash
git add frontend/package.json frontend/package-lock.json frontend/.npmrc \
        frontend/src/App.svelte frontend/src/main.ts frontend/vite.config.ts
git commit -m "feat(frontend): embed motion-tools Visualizer with live arm"
```

---

## Task 2: TcpTriad scene object (proves the in-scene plugin pattern)

Smallest possible scene-coupled plugin: a live coordinate triad at the arm's
TCP, driven by our own `get_arm_state` poll (backend truth). Proves that a
component in the `children` snippet can render 3D and read live pose.

**Files:**
- Create: `frontend/src/scene/TcpTriad.svelte`
- Modify: `frontend/src/App.svelte` (mount it in `children`)

**Step 1: Write the component**

The poll mirrors `JogPanel`'s existing pattern (`createResourceQuery` on
`doCommand` with `getArmState()`), writing into the shared `armState` store.
Render the exported `<AxesHelper>` at the pose. `⚠ confirm` the `AxesHelper`
import subpath and its `position`/`rotation`/quaternion prop names from Step 2
of Task 1; convert the pose orientation (o_x,o_y,o_z,theta axis-angle, degrees)
to a three.js quaternion.

```svelte
<script lang="ts">
  import { createResourceQuery, usePolling } from '@viamrobotics/svelte-sdk'
  import { Quaternion, Vector3, MathUtils } from 'three'
  import { AxesHelper } from '@viamrobotics/motion-tools/lib' // ⚠ confirm subpath
  import { poseTrackerClient } from '../lib/clients'
  import { selectedResource } from '../lib/resource.svelte'
  import { useMachineId } from '../lib/machine'
  import { armState } from '../lib/armState.svelte'
  import { getArmState, parseArmState, toCommandArgs } from '../lib/poseTracker'

  const machineId = useMachineId()
  const pt = poseTrackerClient(machineId, () => selectedResource.name ?? '')
  const q = createResourceQuery(pt, 'doCommand', () => toCommandArgs(getArmState()), () => ({}))
  usePolling(() => q.queryKey, () => (armState.hasArm === false && q.error ? false : 500))

  $effect(() => {
    if (q.data) {
      const s = parseArmState(q.data as Record<string, unknown>)
      armState.pose = s.pose; armState.joints = s.joints; armState.hasArm = true
    } else if (q.error) { armState.hasArm = false }
  })

  // Viam poses are millimeters; the scene is meters.
  const position = $derived<[number, number, number]>(
    armState.pose ? [armState.pose.x / 1000, armState.pose.y / 1000, armState.pose.z / 1000] : [0, 0, 0],
  )
  const quaternion = $derived.by(() => {
    const p = armState.pose
    if (!p) return new Quaternion()
    const axis = new Vector3(p.o_x, p.o_y, p.o_z)
    if (axis.lengthSq() === 0) return new Quaternion()
    return new Quaternion().setFromAxisAngle(axis.normalize(), MathUtils.degToRad(p.theta))
  })
</script>

{#if armState.pose}
  <AxesHelper {position} {quaternion} length={0.1} />
{/if}
```

**Step 2: Mount it in App.svelte**

Inside the `children` snippet:

```svelte
{#snippet children()}
  <TcpTriad />
{/snippet}
```

with `import TcpTriad from './scene/TcpTriad.svelte'`.

**Step 3: Type-check**

Run: `cd frontend && npm run check` — Expected: no errors.

**Step 4: Verify (verify skill)**

Dev server up, resource picked. Confirm a triad sits at the tool tip and follows
the arm as you jog it (jog via Viam app controls or wait for Task 6). Axis
directions match the flange orientation. If the triad is 1000× off or rotated,
fix the mm→m scale / quaternion here before proceeding.

**Step 5: Commit**

```bash
git add frontend/src/scene/TcpTriad.svelte frontend/src/App.svelte
git commit -m "feat(frontend): live TCP triad in the Visualizer scene"
```

---

## Task 3: Wizard state machine (pure, TDD)

The 3-point wizard's logic as a pure, framework-free-ish rune store: step
transitions, capture accumulation, ready-to-commit, and error surfacing. This is
the one fully test-first task.

**Files:**
- Create: `frontend/src/lib/wizard/frameDefine.svelte.ts`
- Test: `frontend/src/lib/wizard/frameDefine.test.ts`

**Step 1: Write the failing tests**

`frontend/src/lib/wizard/frameDefine.test.ts`:

```ts
import { describe, it, expect, beforeEach } from 'vitest'
import { createFrameDefineWizard } from './frameDefine.svelte'
import type { PoseMap } from '../poseTracker'

const pose = (x: number, y: number, z: number): PoseMap =>
  ({ x, y, z, o_x: 0, o_y: 0, o_z: 1, theta: 0 })

describe('frameDefine wizard', () => {
  let w: ReturnType<typeof createFrameDefineWizard>
  beforeEach(() => { w = createFrameDefineWizard() })

  it('starts idle at step 0 with no captures', () => {
    expect(w.step).toBe(0)
    expect(w.captures).toEqual([])
    expect(w.canCommit).toBe(false)
  })

  it('start() requires a non-blank name and advances to capture P0', () => {
    expect(w.start('')).toBe(false)
    expect(w.step).toBe(0)
    expect(w.start('  table  ')).toBe(true)
    expect(w.name).toBe('table')   // trimmed
    expect(w.step).toBe(1)
  })

  it('records captures and advances one step each', () => {
    w.start('f')
    w.recordCapture(pose(0, 0, 0)); expect(w.step).toBe(2); expect(w.captures.length).toBe(1)
    w.recordCapture(pose(1, 0, 0)); expect(w.step).toBe(3); expect(w.captures.length).toBe(2)
    w.recordCapture(pose(0, 1, 0)); expect(w.step).toBe(4); expect(w.captures.length).toBe(3)
  })

  it('is committable only with exactly 3 captures', () => {
    w.start('f')
    w.recordCapture(pose(0, 0, 0)); w.recordCapture(pose(1, 0, 0))
    expect(w.canCommit).toBe(false)
    w.recordCapture(pose(0, 1, 0))
    expect(w.canCommit).toBe(true)
  })

  it('recapture() clears captures and returns to capture P0', () => {
    w.start('f')
    w.recordCapture(pose(0, 0, 0)); w.recordCapture(pose(1, 0, 0)); w.recordCapture(pose(0, 1, 0))
    w.recapture()
    expect(w.captures).toEqual([]); expect(w.step).toBe(1); expect(w.error).toBeUndefined()
  })

  it('setError surfaces a backend failure without losing captures', () => {
    w.start('f'); w.recordCapture(pose(0, 0, 0))
    w.setError('collinear points: X axis undefined')
    expect(w.error).toBe('collinear points: X axis undefined')
    expect(w.captures.length).toBe(1)
  })

  it('committed() advances to the done step', () => {
    w.start('f')
    w.recordCapture(pose(0, 0, 0)); w.recordCapture(pose(1, 0, 0)); w.recordCapture(pose(0, 1, 0))
    w.committed()
    expect(w.step).toBe(5); expect(w.done).toBe(true)
  })

  it('reset() returns to a fresh idle wizard', () => {
    w.start('f'); w.recordCapture(pose(0, 0, 0)); w.reset()
    expect(w.step).toBe(0); expect(w.name).toBe(''); expect(w.captures).toEqual([])
  })
})
```

**Step 2: Run to verify it fails**

Run: `cd frontend && npm run test -- frameDefine`
Expected: FAIL — `createFrameDefineWizard` not found.

**Step 3: Implement the store**

`frontend/src/lib/wizard/frameDefine.svelte.ts`:

```ts
import type { PoseMap } from '../poseTracker'

// Steps: 0 name&start · 1 capture P0 · 2 capture P1 · 3 capture P2 ·
// 4 preview&commit · 5 committed. Pure logic; the panel wires DoCommands and
// the scene plugin reads `captures`/`step` to draw the spatial story.
export type WizardStep = 0 | 1 | 2 | 3 | 4 | 5

export function createFrameDefineWizard() {
  let step = $state<WizardStep>(0)
  let name = $state('')
  let captures = $state<PoseMap[]>([])
  let error = $state<string | undefined>(undefined)

  return {
    get step() { return step },
    get name() { return name },
    get captures() { return captures },
    get error() { return error },
    get canCommit() { return captures.length === 3 },
    get done() { return step === 5 },

    start(rawName: string): boolean {
      const trimmed = rawName.trim()
      if (!trimmed) return false
      name = trimmed; captures = []; error = undefined; step = 1
      return true
    },
    recordCapture(pose: PoseMap) {
      if (step < 1 || step > 3) return
      captures = [...captures, pose]
      error = undefined
      step = (captures.length === 3 ? 4 : step + 1) as WizardStep
    },
    recapture() { captures = []; error = undefined; step = 1 },
    setError(message: string) { error = message },
    committed() { step = 5 },
    reset() { step = 0; name = ''; captures = []; error = undefined },
  }
}
```

**Step 4: Run to verify it passes**

Run: `cd frontend && npm run test -- frameDefine` — Expected: PASS (all cases).

**Step 5: Commit**

```bash
git add frontend/src/lib/wizard/frameDefine.svelte.ts frontend/src/lib/wizard/frameDefine.test.ts
git commit -m "feat(frontend): 3-point frame-define wizard state machine (TDD)"
```

---

## Task 4: FrameDefineWizard panel (drives the procedure)

A `FloatingPanel` that walks the steps, calling the existing DoCommands and
feeding the wizard store. Reuses `CapturePanel`'s capture flow
(`capture_point` → `get_buffer`) and `FramePanel`'s define flow
(`define_frame(name, '3point')` → `bumpCaptureBuffer()`).

**Files:**
- Create: `frontend/src/panels/FrameDefineWizard.svelte`
- Modify: `frontend/src/App.svelte` (mount in `children`, share one wizard instance)

**Step 1: Share one wizard instance**

The panel (Task 4) and the plugin (Task 5) must read the SAME wizard store.
Create it in `App.svelte` and pass it down as a prop to both.

In `App.svelte`:

```svelte
import { createFrameDefineWizard } from './lib/wizard/frameDefine.svelte'
import FrameDefineWizard from './panels/FrameDefineWizard.svelte'
// ...
const wizard = createFrameDefineWizard()
// ...
{#snippet children()}
  <TcpTriad />
  <FrameDefineWizard {wizard} />
{/snippet}
```

**Step 2: Write the panel**

Prime-core component names (`Button`, panel body) are `⚠ confirm against
installed package`; fall back to plain `<button>` if a name is uncertain — the
logic is what matters. The panel drives DoCommands and updates `wizard`.

```svelte
<script lang="ts">
  import { createResourceMutation } from '@viamrobotics/svelte-sdk'
  import { FloatingPanel } from '@viamrobotics/motion-tools' // ⚠ confirm export
  import { poseTrackerClient } from '../lib/clients'
  import { selectedResource } from '../lib/resource.svelte'
  import { useMachineId } from '../lib/machine'
  import { armState } from '../lib/armState.svelte'
  import { bumpCaptureBuffer } from '../lib/captureBuffer.svelte'
  import {
    capturePoint, clearBuffer, defineFrame, toCommandArgs,
    type DefineFrameResponse,
  } from '../lib/poseTracker'
  import type { createFrameDefineWizard } from '../lib/wizard/frameDefine.svelte'

  let { wizard }: { wizard: ReturnType<typeof createFrameDefineWizard> } = $props()

  const machineId = useMachineId()
  const pt = poseTrackerClient(machineId, () => selectedResource.name ?? '')
  const capture = createResourceMutation(pt, 'doCommand')
  const clear = createResourceMutation(pt, 'doCommand')
  const define = createResourceMutation(pt, 'doCommand')

  let nameInput = $state('')

  const PROMPTS = [
    'Name the frame, then Start.',
    'Jog the tool to the frame ORIGIN, then Capture.',
    'Jog along the intended +X axis, then Capture.',
    'Jog to a point in the +Y half-plane, then Capture.',
    'Review the frame, then Commit or Re-capture.',
    'Frame saved.',
  ]

  async function handleStart() {
    // Clear any stale server buffer so our 3 captures are the only points.
    try { await clear.mutateAsync(toCommandArgs(clearBuffer())) } catch {}
    wizard.start(nameInput)
  }

  async function handleCapture() {
    if (!armState.pose) return
    try {
      await capture.mutateAsync(toCommandArgs(capturePoint()))
      // Record the live TCP pose the operator just captured (backend truth is
      // the same pose the readout shows).
      wizard.recordCapture({ ...armState.pose })
    } catch (e) {
      wizard.setError(e instanceof Error ? e.message : String(e))
    }
  }

  async function handleCommit() {
    try {
      const res = (await define.mutateAsync(
        toCommandArgs(defineFrame(wizard.name, '3point')),
      )) as unknown as DefineFrameResponse
      if (res?.committed) { bumpCaptureBuffer(); wizard.committed() }
      else wizard.setError('define_frame did not commit')
    } catch (e) {
      wizard.setError(e instanceof Error ? e.message : String(e))
    }
  }

  async function handleRecapture() {
    try { await clear.mutateAsync(toCommandArgs(clearBuffer())) } catch {}
    wizard.recapture()
  }
</script>

<FloatingPanel title="Define Frame" defaultPosition={{ x: 24, y: 88 }} isOpen>
  <div class="wizard">
    <p class="prompt">{PROMPTS[wizard.step]}</p>

    {#if wizard.step === 0}
      <input bind:value={nameInput} placeholder="frame name" />
      <button onclick={handleStart} disabled={!nameInput.trim()}>Start</button>
    {:else if wizard.step >= 1 && wizard.step <= 3}
      <p class="count">{wizard.captures.length} / 3 captured</p>
      <button onclick={handleCapture} disabled={capture.isPending || !armState.hasArm}>
        {capture.isPending ? 'Capturing…' : 'Capture'}
      </button>
    {:else if wizard.step === 4}
      <button onclick={handleCommit} disabled={define.isPending || !wizard.canCommit}>Commit</button>
      <button onclick={handleRecapture} class="secondary">Re-capture</button>
    {:else if wizard.step === 5}
      <button onclick={() => { nameInput = ''; wizard.reset() }}>Define another</button>
    {/if}

    {#if wizard.error}<p class="error">{wizard.error}</p>{/if}
  </div>
</FloatingPanel>

<style>
  .wizard { display: flex; flex-direction: column; gap: 0.75rem; padding: 0.5rem; }
  .prompt { margin: 0; font-weight: 600; }
  .count { margin: 0; opacity: 0.7; }
  .error { color: #c62828; margin: 0; }
  button { min-height: 44px; }
</style>
```

**Step 3: Type-check**

Run: `cd frontend && npm run check` — Expected: no errors. `⚠` If `FloatingPanel`
props differ from the design's table (title/defaultPosition/defaultSize/isOpen),
adjust to the installed signature.

**Step 4: Verify (verify skill) — capture flow only**

Dev server, resource picked. Without the scene plugin yet: enter a name → Start;
jog the arm (Viam controls, or after Task 6); Capture three times; watch the
`n / 3` counter; Commit. Confirm a new frame triad appears in the scene (proving
`define_frame` → `world_state_store` → redraw). Trigger a degenerate case
(3 near-collinear points) and confirm the backend error surfaces in the panel.

**Step 5: Commit**

```bash
git add frontend/src/panels/FrameDefineWizard.svelte frontend/src/App.svelte
git commit -m "feat(frontend): 3-point frame-define wizard panel"
```

---

## Task 5: FrameDefinePlugin (the spatial story)

Draws the guided-procedure affordances from the shared wizard store: captured
markers, the P0→TCP dashed axis preview, the +XY plane preview, and the
provisional frame triad at step 4.

**Files:**
- Create: `frontend/src/scene/FrameDefinePlugin.svelte`
- Create: `frontend/src/scene/frameGeometry.ts` (pure helpers, TDD-able)
- Test: `frontend/src/scene/frameGeometry.test.ts`
- Modify: `frontend/src/App.svelte` (mount in `children`, pass `wizard`)

**Step 1 (TDD): pure geometry helper for the provisional triad**

The one bit of real math worth testing: turn 3 captured points into the
provisional frame's origin + basis (mirrors the backend `ComputeThreePoint` so
the preview matches what will commit). Write the test first.

`frontend/src/scene/frameGeometry.test.ts`:

```ts
import { describe, it, expect } from 'vitest'
import { threePointBasis } from './frameGeometry'

describe('threePointBasis', () => {
  it('origin is P0; +X points toward P1; +Z is X×(toward P2)', () => {
    const b = threePointBasis(
      { x: 0, y: 0, z: 0 }, { x: 2, y: 0, z: 0 }, { x: 0, y: 3, z: 0 },
    )
    expect(b.origin).toEqual([0, 0, 0])
    expect(b.x[0]).toBeCloseTo(1); expect(b.x[1]).toBeCloseTo(0); expect(b.x[2]).toBeCloseTo(0)
    expect(b.z[2]).toBeCloseTo(1)   // right-handed, +Z up
    expect(b.y[1]).toBeCloseTo(1)
  })

  it('returns null for collinear points (degenerate)', () => {
    expect(threePointBasis({ x: 0, y: 0, z: 0 }, { x: 1, y: 0, z: 0 }, { x: 2, y: 0, z: 0 })).toBeNull()
  })
})
```

`frontend/src/scene/frameGeometry.ts`:

```ts
import { Vector3, Matrix4, Quaternion } from 'three'

type P = { x: number; y: number; z: number }

export interface Basis {
  origin: [number, number, number]
  x: [number, number, number]
  y: [number, number, number]
  z: [number, number, number]
}

// Mirrors the Go backend's ComputeThreePoint: origin=P0, +X toward P1,
// +Z = X × (P0→P2), +Y = Z × X. Returns null when the points are collinear
// (cross product ~0), matching the backend's degeneracy guard.
export function threePointBasis(p0: P, p1: P, p2: P): Basis | null {
  const o = new Vector3(p0.x, p0.y, p0.z)
  const x = new Vector3(p1.x, p1.y, p1.z).sub(o)
  const toP2 = new Vector3(p2.x, p2.y, p2.z).sub(o)
  if (x.lengthSq() === 0 || toP2.lengthSq() === 0) return null
  x.normalize()
  const z = new Vector3().crossVectors(x, toP2)
  if (z.lengthSq() < 1e-9) return null
  z.normalize()
  const y = new Vector3().crossVectors(z, x).normalize()
  return { origin: [o.x, o.y, o.z], x: [x.x, x.y, x.z], y: [y.x, y.y, y.z], z: [z.x, z.y, z.z] }
}

// Quaternion (for <AxesHelper>) from a basis, in scene meters at the origin.
export function basisToQuaternion(b: Basis): Quaternion {
  const m = new Matrix4().makeBasis(
    new Vector3(...b.x), new Vector3(...b.y), new Vector3(...b.z),
  )
  return new Quaternion().setFromRotationMatrix(m)
}
```

Run: `cd frontend && npm run test -- frameGeometry` — first FAIL (no module),
then PASS after creating the file.

**Step 2: Write the plugin**

Markers as non-interactive meshes (MeasureTool pattern: `raycast={() => null}`);
dashed axis + plane as Threlte primitives updated from `armState.pose`;
provisional triad via `<AxesHelper>` from the basis. All positions convert
mm→m (`/1000`). `⚠ confirm` Threlte `<T.*>` / `MeshLineGeometry` /`AxesHelper`
imports against the installed package.

```svelte
<script lang="ts">
  import { T } from '@threlte/core'
  import { AxesHelper } from '@viamrobotics/motion-tools/lib' // ⚠ confirm
  import { armState } from '../lib/armState.svelte'
  import { threePointBasis, basisToQuaternion } from './frameGeometry'
  import type { createFrameDefineWizard } from '../lib/wizard/frameDefine.svelte'

  let { wizard }: { wizard: ReturnType<typeof createFrameDefineWizard> } = $props()

  const m = (p: { x: number; y: number; z: number }): [number, number, number] =>
    [p.x / 1000, p.y / 1000, p.z / 1000]

  const basis = $derived.by(() => {
    if (wizard.captures.length !== 3) return null
    return threePointBasis(...(wizard.captures as [any, any, any]))
  })
</script>

<!-- Captured-point markers -->
{#each wizard.captures as c, i (i)}
  <T.Mesh position={m(c)} raycast={() => null}>
    <T.SphereGeometry args={[0.006]} />
    <T.MeshBasicMaterial color="#3560ee" />
  </T.Mesh>
{/each}

<!-- Provisional frame triad at step 4 -->
{#if basis}
  <AxesHelper
    position={[basis.origin[0] / 1000, basis.origin[1] / 1000, basis.origin[2] / 1000]}
    quaternion={basisToQuaternion(basis)}
    length={0.12}
  />
{/if}

<!-- NOTE: dashed P0→TCP axis preview and the +XY plane preview are added here,
     updated from armState.pose during steps 2 and 3. Use MeshLineGeometry for
     the segment (MeasureTool reference) and a translucent T.Mesh + PlaneGeometry
     for the plane. Keep both raycast={() => null}. -->
```

**Step 3: Mount in App.svelte**

```svelte
{#snippet children()}
  <TcpTriad />
  <FrameDefinePlugin {wizard} />
  <FrameDefineWizard {wizard} />
{/snippet}
```

**Step 4: Type-check + verify (verify skill)**

`npm run check` clean. Then drive the full staged flow: markers appear as each
point is captured; at 3 captures the provisional triad shows at P0 with axes
matching the intended X/Y; Commit replaces it with the real `world_state_store`
triad. Confirm collinear captures show NO provisional triad (basis null) — the
scene refuses to imply a frame that can't be computed.

**Step 5: Commit**

```bash
git add frontend/src/scene/FrameDefinePlugin.svelte frontend/src/scene/frameGeometry.ts \
        frontend/src/scene/frameGeometry.test.ts frontend/src/App.svelte
git commit -m "feat(frontend): stage the 3-point wizard in the scene"
```

---

## Task 6: Re-home JogPanel into the Visualizer chrome

Move the existing jog controls into a `FloatingPanel` with a `DashboardPortal`
toggle, restyled off `tokens.css`. Preserve ALL existing jog logic
(`jogHold` press-and-hold action, `startHold`/`stopHold`, the arm-state poll,
`motion` stop-sequence integration) verbatim — only the wrapper and styles
change.

**Files:**
- Modify: `frontend/src/panels/JogPanel.svelte`
- Modify: `frontend/src/App.svelte` (mount in `children`)
- Delete: `frontend/src/styles/tokens.css` and its import in `main.ts`
  (only once no file references `var(--…)` — grep first)

**Step 1: Wrap JogPanel**

Keep the entire `<script>` (jog action, poll, holds) unchanged. Replace the outer
`<section class="jog-panel">` with a `FloatingPanel title="Jog"`. Add a
`DashboardPortal` toggle button that opens/closes the panel (`isOpen` bindable),
mirroring MeasureTool's dashboard button. `⚠ confirm` `DashboardPortal` +
prime-core `Button` import names.

```svelte
<script lang="ts">
  import { FloatingPanel, DashboardPortal } from '@viamrobotics/motion-tools' // ⚠ confirm
  // ...all existing imports and logic unchanged...
  let open = $state(true)
</script>

<DashboardPortal>
  <button onclick={() => (open = !open)} aria-pressed={open}>Jog</button>
</DashboardPortal>

<FloatingPanel title="Jog" bind:isOpen={open} defaultPosition={{ x: 24, y: 24 }}>
  <!-- existing jog body: mode toggle, step groups, axis grids, readout table -->
</FloatingPanel>
```

**Step 2: Restyle off tokens**

Replace `var(--…)` references in JogPanel's `<style>` with literal values or
prime-core equivalents so the panel no longer depends on `tokens.css`. Keep the
44px touch targets (`PRODUCT.md` accessibility requirement) and the `.holding`
pressed-key affordance.

**Step 3: Mount + drop tokens.css**

Add `<JogPanel />` to the `children` snippet. Then:

```bash
cd frontend && grep -rl "var(--" src && echo "still referenced — do NOT delete tokens yet" || \
  ( git rm src/styles/tokens.css && sed -i '' "/tokens.css/d" src/main.ts )
```

Only remove `tokens.css` if the grep finds no remaining references (deferred,
unmounted panels still import it — if so, leave `tokens.css` in place; it costs
nothing while unused).

**Step 4: Type-check + verify (verify skill)**

`npm run check` clean. Confirm the Jog panel floats over the scene, the dashboard
toggle shows/hides it, press-and-hold jogging still works (arm moves, readout is
live, the held key fills), and STOP integration is intact.

**Step 5: Commit**

```bash
git add -A frontend/src
git commit -m "feat(frontend): re-home jog controls into the Visualizer chrome"
```

---

## Task 7: End-to-end verification and slice wrap-up

**Files:** none (verification + docs)

**Step 1: Full verify pass (verify skill)**

Against a real machine (RGBD not needed for this slice; arm + pose-tracker
required), drive the entire prototype definition-of-done:

1. Open pendant → pick pose-tracker resource.
2. Live arm + existing frame triads render; TCP triad tracks the tool.
3. Toggle Jog panel; jog the TCP to a physical origin.
4. Start the Define Frame wizard, name it; Capture P0/P1/P2 with the scene
   guiding each (markers, axis, plane).
5. Provisional triad matches intent at step 4; Commit.
6. The frame persists and re-appears as a `world_state_store` triad.
7. Reload — the frame is still there (persisted to config).

Record the result (pass, or the specific failure) in the commit message.

**Step 2: Confirm the unit suite is green**

Run: `cd frontend && npm run test` — Expected: PASS (wizard + geometry suites).
Run: `npm run check` — Expected: no type errors.
Run: `npm run build` — Expected: succeeds; note final bundle size.

**Step 3: Note the go/no-go and deferred boundary**

Append a short "Slice outcome" note to the design doc: bundle size, whether
workers needed a Vite tweak, and confirmation the pattern (wizard store +
panel + scene plugin) is ready to be copied for single-point / TCP-snapshot /
hand-eye / eye-in-hand.

**Step 4: Commit**

```bash
git add docs/plans/2026-07-20-visualizer-teach-pendant-design.md
git commit -m "docs: record vertical-slice outcome and deferred boundary"
```

---

## Deferred (explicitly NOT in this slice)

single-point & tcp_snapshot frame methods; eye-to-hand and eye-in-hand wizards;
TCP teaching; frame-relative jogging; waypoint record/replay. Their `lib/`
DoCommand code already exists and is untouched — each is later added as "a wizard
store + a panel + a scene plugin" following the template this slice establishes.
