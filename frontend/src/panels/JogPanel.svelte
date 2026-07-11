<script lang="ts">
  import type { Action } from 'svelte/action'
  import { createResourceMutation, createResourceQuery, usePolling } from '@viamrobotics/svelte-sdk'
  import { poseTrackerClient } from '../lib/clients'
  import { selectedResource } from '../lib/resource.svelte'
  import { useMachineId } from '../lib/machine'
  import { armState } from '../lib/armState.svelte'
  import { motion, withMove } from '../lib/motion.svelte'
  import {
    getArmState,
    jogCartesian,
    jogJoint,
    parseArmState,
    toCommandArgs,
    type CartesianAxis,
    type PoseMap,
  } from '../lib/poseTracker'

  const machineId = useMachineId()
  const pt = poseTrackerClient(machineId, () => selectedResource.name ?? '')
  const jog = createResourceMutation(pt, 'doCommand')

  // Live arm-state poll. Lives here (not in the status bar) so the pose/joint
  // readout sits with the jog controls. 4th arg (options) is required — see the
  // note in the other query panels. Paused while a move is in flight and once we
  // know no arm is configured (that query would only ever error).
  const armStateQuery = createResourceQuery(
    pt,
    'doCommand',
    () => toCommandArgs(getArmState()),
    () => ({}),
  )

  const NO_ARM_PHRASE = 'arm dependency not configured'
  const noArmConfigured = $derived(
    armStateQuery.error !== null && armStateQuery.error.message.includes(NO_ARM_PHRASE),
  )
  const unexpectedError = $derived(
    armStateQuery.error !== null && !noArmConfigured ? armStateQuery.error.message : undefined,
  )

  usePolling(
    () => armStateQuery.queryKey,
    () => (noArmConfigured || motion.busy > 0 ? false : 500),
  )

  // Publish the live arm state into the shared store so TcpPanel (and jogging
  // here) can react to whether an arm is present.
  $effect(() => {
    if (armStateQuery.data) {
      const parsed = parseArmState(armStateQuery.data as Record<string, unknown>)
      armState.pose = parsed.pose
      armState.joints = parsed.joints
      armState.hasArm = true
    } else if (armStateQuery.error) {
      armState.hasArm = false
    }
  })

  function fmt(n: number | undefined): string {
    return n === undefined ? '—' : n.toFixed(2)
  }

  type Mode = 'cartesian' | 'joint'
  let mode = $state<Mode>('cartesian')

  const TRANSLATION_STEPS = [0.1, 1, 10] as const
  const ROTATION_STEPS = [1, 5] as const
  const JOINT_STEPS = [1, 5] as const

  let transStep = $state<number>(TRANSLATION_STEPS[1])
  let rotStep = $state<number>(ROTATION_STEPS[0])
  let jointStep = $state<number>(JOINT_STEPS[0])

  const TRANSLATION_AXES: CartesianAxis[] = ['x', 'y', 'z']
  const ROTATION_AXES: CartesianAxis[] = ['roll', 'pitch', 'yaw']

  // Jog buttons are gated only on arm presence. Overlap is prevented by the
  // `holding` guard + single-pointer capture, NOT by disabling on motion.busy —
  // disabling the held button mid-hold would abort the press-and-hold loop.
  const disabled = $derived(!armState.hasArm)

  // Press-and-hold: while a jog button is held, send one jog, await it, and
  // repeat. Awaiting each move paces the loop to the arm (no command stacking).
  // A single `holding` flag guarantees only one loop runs at a time.
  let holding = false

  async function startHold(build: () => Record<string, unknown>) {
    if (holding || !armState.hasArm) {
      return
    }
    holding = true
    // Capture the stop generation; if Stop is pressed mid-hold it bumps and we bail.
    const seq = motion.stopSeq
    await withMove(async () => {
      try {
        while (holding && armState.hasArm && motion.stopSeq === seq) {
          const resp = (await jog.mutateAsync(toCommandArgs(build()))) as Record<string, unknown>
          // Drive the readout live from the jog response (polling is paused
          // while a move is in flight).
          if (resp && typeof resp === 'object') {
            if (resp.pose) {
              armState.pose = resp.pose as PoseMap
            }
            if (Array.isArray(resp.joints)) {
              armState.joints = resp.joints as number[]
            }
          }
        }
      } catch {
        // Surfaced via jog.error in the template; stop the loop.
      }
    })
    holding = false
  }

  function stopHold() {
    holding = false
  }

  // Action: press-and-hold (or hold Enter/Space) to jog repeatedly; release to
  // stop. Pointer capture keeps the release reliable even if the pointer drifts
  // off the button, and a window blur is a safety backstop.
  const jogHold: Action<HTMLButtonElement, () => Record<string, unknown>> = (node, build) => {
    let current = build

    // Reflect the held state on the button itself — the operator should see the
    // key engage while the arm jogs, not just watch numbers move in the readout.
    // Driven by the hold loop's lifetime (via .finally) so it also clears when
    // STOP ends the loop mid-press, not only on pointer/key release.
    function onPointerDown(event: PointerEvent) {
      if (node.disabled) {
        return
      }
      node.setPointerCapture(event.pointerId)
      node.classList.add('holding')
      void startHold(current).finally(() => node.classList.remove('holding'))
    }
    function onKeyDown(event: KeyboardEvent) {
      if (event.key !== 'Enter' && event.key !== ' ') {
        return
      }
      event.preventDefault() // suppress the synthetic click so we don't double-send
      node.classList.add('holding')
      void startHold(current).finally(() => node.classList.remove('holding'))
    }
    function onKeyUp(event: KeyboardEvent) {
      if (event.key === 'Enter' || event.key === ' ') {
        stopHold()
      }
    }

    node.addEventListener('pointerdown', onPointerDown)
    node.addEventListener('pointerup', stopHold)
    node.addEventListener('pointercancel', stopHold)
    node.addEventListener('lostpointercapture', stopHold)
    node.addEventListener('keydown', onKeyDown)
    node.addEventListener('keyup', onKeyUp)
    window.addEventListener('blur', stopHold)

    return {
      update(next: () => Record<string, unknown>) {
        current = next
      },
      destroy() {
        node.removeEventListener('pointerdown', onPointerDown)
        node.removeEventListener('pointerup', stopHold)
        node.removeEventListener('pointercancel', stopHold)
        node.removeEventListener('lostpointercapture', stopHold)
        node.removeEventListener('keydown', onKeyDown)
        node.removeEventListener('keyup', onKeyUp)
        window.removeEventListener('blur', stopHold)
        stopHold()
      },
    }
  }
</script>

<section class="jog-panel">
  <header class="panel-header">
    <div class="panel-titles">
      <h2>Jog</h2>
      <p class="panel-subtitle">Move the arm to position it for capture.</p>
    </div>
    <div class="mode-toggle" role="group" aria-label="Jog mode">
      <button type="button" class:active={mode === 'cartesian'} onclick={() => (mode = 'cartesian')}>
        Cartesian
      </button>
      <button type="button" class:active={mode === 'joint'} onclick={() => (mode = 'joint')}>
        Joint
      </button>
    </div>
  </header>

  <div class="jog-body">
    <div class="controls">
      {#if mode === 'cartesian'}
        <div class="step-group">
          <span class="step-label">Translation step (mm)</span>
          <div class="step-options" role="group" aria-label="Translation step size">
            {#each TRANSLATION_STEPS as step (step)}
              <button type="button" class:active={transStep === step} onclick={() => (transStep = step)}>
                {step}
              </button>
            {/each}
          </div>
        </div>

        <div class="axis-grid">
          {#each TRANSLATION_AXES as axis (axis)}
            <div class="axis-row">
              <span class="axis-name">{axis.toUpperCase()}</span>
              <button type="button" {disabled} use:jogHold={() => jogCartesian(axis, -transStep)} aria-label="{axis} minus">−</button>
              <button type="button" {disabled} use:jogHold={() => jogCartesian(axis, transStep)} aria-label="{axis} plus">+</button>
            </div>
          {/each}
        </div>

        <div class="step-group">
          <span class="step-label">Rotation step (deg)</span>
          <div class="step-options" role="group" aria-label="Rotation step size">
            {#each ROTATION_STEPS as step (step)}
              <button type="button" class:active={rotStep === step} onclick={() => (rotStep = step)}>
                {step}
              </button>
            {/each}
          </div>
        </div>

        <div class="axis-grid">
          {#each ROTATION_AXES as axis (axis)}
            <div class="axis-row">
              <span class="axis-name">{axis[0]?.toUpperCase()}{axis.slice(1)}</span>
              <button type="button" {disabled} use:jogHold={() => jogCartesian(axis, -rotStep)} aria-label="{axis} minus">−</button>
              <button type="button" {disabled} use:jogHold={() => jogCartesian(axis, rotStep)} aria-label="{axis} plus">+</button>
            </div>
          {/each}
        </div>
      {:else}
        <div class="step-group">
          <span class="step-label">Joint step (deg)</span>
          <div class="step-options" role="group" aria-label="Joint step size">
            {#each JOINT_STEPS as step (step)}
              <button type="button" class:active={jointStep === step} onclick={() => (jointStep = step)}>
                {step}
              </button>
            {/each}
          </div>
        </div>

        <div class="axis-grid">
          {#each armState.joints as _joint, i (i)}
            <div class="axis-row">
              <span class="axis-name">J{i}</span>
              <button type="button" {disabled} use:jogHold={() => jogJoint(i, -jointStep)} aria-label="joint {i} minus">−</button>
              <button type="button" {disabled} use:jogHold={() => jogJoint(i, jointStep)} aria-label="joint {i} plus">+</button>
            </div>
          {/each}
          {#if armState.joints.length === 0}
            <p class="panel-empty">No joint data available.</p>
          {/if}
        </div>
      {/if}
    </div>

    <div class="readout-col">
      {#if armState.hasArm && armState.pose}
        {#if mode === 'cartesian'}
          <table class="readout" aria-label="Pose values">
            <thead>
              <tr><th>Pose</th><th>Value</th></tr>
            </thead>
            <tbody>
              <tr><th scope="row">X</th><td>{fmt(armState.pose.x)}<span class="unit">mm</span></td></tr>
              <tr><th scope="row">Y</th><td>{fmt(armState.pose.y)}<span class="unit">mm</span></td></tr>
              <tr><th scope="row">Z</th><td>{fmt(armState.pose.z)}<span class="unit">mm</span></td></tr>
              <tr><th scope="row">OX</th><td>{fmt(armState.pose.o_x)}</td></tr>
              <tr><th scope="row">OY</th><td>{fmt(armState.pose.o_y)}</td></tr>
              <tr><th scope="row">OZ</th><td>{fmt(armState.pose.o_z)}</td></tr>
              <tr><th scope="row">&#952;</th><td>{fmt(armState.pose.theta)}<span class="unit">&deg;</span></td></tr>
            </tbody>
          </table>
        {:else}
          <table class="readout" aria-label="Joint positions">
            <thead>
              <tr><th>Joint</th><th>Position (&deg;)</th></tr>
            </thead>
            <tbody>
              {#each armState.joints as joint, i (i)}
                <tr><th scope="row">{i}</th><td>{fmt(joint)}<span class="unit">&deg;</span></td></tr>
              {:else}
                <tr><td colspan="2" class="panel-empty">No joint data available.</td></tr>
              {/each}
            </tbody>
          </table>
        {/if}
      {:else if unexpectedError}
        <p class="error">{unexpectedError}</p>
      {:else if noArmConfigured}
        <p class="panel-warning">No arm configured — jogging is disabled.</p>
      {:else}
        <p class="panel-empty">Loading arm state…</p>
      {/if}
    </div>
  </div>

  {#if jog.error}
    <p class="error">{jog.error.message}</p>
  {/if}
</section>

<style>
  .jog-panel {
    display: flex;
    flex-direction: column;
    gap: 1rem;
    padding: 1rem;
  }

  .jog-body {
    display: grid;
    grid-template-columns: minmax(0, 1fr) minmax(0, 1fr);
    gap: 1.25rem;
    align-items: start;
  }

  .controls {
    display: flex;
    flex-direction: column;
    gap: 1rem;
    min-width: 0;
  }

  .readout-col {
    min-width: 0;
  }

  @media (max-width: 560px) {
    .jog-body {
      grid-template-columns: 1fr;
    }
  }

  .mode-toggle {
    display: flex;
    gap: 0.5rem;
  }

  .mode-toggle button,
  .step-options button {
    min-height: 44px;
    padding: 0.4rem 1rem;
    border-radius: var(--radius-md);
    border: 1px solid var(--border-control);
    background: var(--surface-control);
    color: inherit;
    cursor: pointer;
  }

  .mode-toggle button.active,
  .step-options button.active {
    background: var(--accent);
    border-color: var(--accent);
    color: var(--ink-on-accent);
  }

  .step-group {
    display: flex;
    flex-direction: column;
    gap: 0.4rem;
  }

  .step-label {
    font-size: 0.85rem;
    color: var(--ink-muted);
  }

  .step-options {
    display: flex;
    gap: 0.5rem;
    flex-wrap: wrap;
  }

  .axis-grid {
    display: flex;
    flex-direction: column;
    gap: 0.5rem;
  }

  .axis-row {
    display: flex;
    align-items: center;
    gap: 0.75rem;
  }

  .axis-name {
    width: 4rem;
    font-weight: 600;
  }

  .axis-row button {
    min-width: 44px;
    min-height: 44px;
    font-size: 1.3rem;
    border-radius: var(--radius-lg);
    border: 1px solid var(--border-control);
    background: var(--surface-control);
    color: inherit;
    cursor: pointer;
  }

  /* Held / pressed: the key fills with accent and sinks slightly, so an
     engaged jog is unmistakable at a glance. `.holding` tracks the jog loop;
     `:active` covers the instant of press before the loop reports back. */
  .axis-row button:global(.holding),
  .axis-row button:active:not(:disabled) {
    background: var(--accent);
    border-color: var(--accent);
    color: var(--ink-on-accent);
    transform: translateY(1px);
  }

  .axis-row button:disabled {
    opacity: 0.4;
    cursor: not-allowed;
  }

  .readout {
    width: 100%;
    border-collapse: collapse;
    background: var(--surface-base);
    border: 1px solid var(--border-panel);
    border-radius: var(--radius-lg);
    overflow: hidden;
    font-variant-numeric: tabular-nums;
    font-size: 0.9rem;
  }

  .readout thead th {
    text-align: left;
    font-size: 0.72rem;
    font-weight: 600;
    letter-spacing: 0.04em;
    text-transform: uppercase;
    color: var(--ink-muted);
    padding: 0.4rem 0.75rem;
    border-bottom: 1px solid var(--border-panel);
    background: var(--surface-header);
  }

  .readout thead th:last-child {
    text-align: right;
  }

  .readout tbody th {
    text-align: left;
    font-weight: 600;
    color: var(--ink);
    width: 5rem;
  }

  .readout tbody td {
    text-align: right;
    color: var(--ink-secondary);
  }

  .readout tbody th,
  .readout tbody td {
    padding: 0.3rem 0.75rem;
    border-bottom: 1px solid var(--border-subtle);
  }

  .readout tbody tr:last-child th,
  .readout tbody tr:last-child td {
    border-bottom: none;
  }

  .readout .unit {
    margin-left: 0.35rem;
    color: var(--ink-faint);
    font-size: 0.8em;
  }

  .error {
    color: var(--error-text);
    font-size: 0.85rem;
  }
</style>
