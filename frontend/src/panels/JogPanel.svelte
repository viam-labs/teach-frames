<script lang="ts">
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

  const disabled = $derived(!armState.hasArm || jog.isPending || motion.busy > 0)

  function cartesian(axis: CartesianAxis, step: number) {
    void withMove(async () => {
      try {
        await jog.mutateAsync(toCommandArgs(jogCartesian(axis, step)))
      } catch {
        // Surfaced via jog.error in the template.
      }
    })
  }

  function joint(index: number, step: number) {
    void withMove(async () => {
      try {
        await jog.mutateAsync(toCommandArgs(jogJoint(index, step)))
      } catch {
        // Surfaced via jog.error in the template.
      }
    })
  }
</script>

<section class="jog-panel">
  <header>
    <h2>Jog</h2>
    <div class="mode-toggle" role="group" aria-label="Jog mode">
      <button type="button" class:active={mode === 'cartesian'} onclick={() => (mode = 'cartesian')}>
        Cartesian
      </button>
      <button type="button" class:active={mode === 'joint'} onclick={() => (mode = 'joint')}>
        Joint
      </button>
    </div>
  </header>

  {#if armState.hasArm && armState.pose}
    <div class="readout" aria-label="Arm state">
      <div class="readout-line">
        <span>X {fmt(armState.pose.x)}</span>
        <span>Y {fmt(armState.pose.y)}</span>
        <span>Z {fmt(armState.pose.z)}</span>
        <span>O&#8339; {fmt(armState.pose.o_x)}</span>
        <span>O&#8340; {fmt(armState.pose.o_y)}</span>
        <span>O&#8342; {fmt(armState.pose.o_z)}</span>
        <span>&#952; {fmt(armState.pose.theta)}</span>
      </div>
      {#if armState.joints.length > 0}
        <div class="readout-line joints">
          {#each armState.joints as joint, i (i)}
            <span>J{i} {fmt(joint)}&deg;</span>
          {/each}
        </div>
      {/if}
    </div>
  {:else if unexpectedError}
    <p class="error">{unexpectedError}</p>
  {:else if noArmConfigured}
    <p class="panel-warning">No arm configured — jogging is disabled.</p>
  {:else}
    <p class="panel-empty">Loading arm state…</p>
  {/if}

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
          <button type="button" {disabled} onclick={() => cartesian(axis, -transStep)}>−</button>
          <button type="button" {disabled} onclick={() => cartesian(axis, transStep)}>+</button>
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
          <button type="button" {disabled} onclick={() => cartesian(axis, -rotStep)}>−</button>
          <button type="button" {disabled} onclick={() => cartesian(axis, rotStep)}>+</button>
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
          <button type="button" {disabled} onclick={() => joint(i, -jointStep)}>−</button>
          <button type="button" {disabled} onclick={() => joint(i, jointStep)}>+</button>
        </div>
      {/each}
      {#if armState.joints.length === 0}
        <p class="panel-empty">No joint data available.</p>
      {/if}
    </div>
  {/if}

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

  header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    flex-wrap: wrap;
    gap: 0.75rem;
  }

  h2 {
    margin: 0;
    font-size: 1.15rem;
  }

  .mode-toggle {
    display: flex;
    gap: 0.5rem;
  }

  .mode-toggle button,
  .step-options button {
    min-height: 44px;
    padding: 0.4rem 1rem;
    border-radius: 0.4rem;
    border: 1px solid var(--control-border, #444);
    background: var(--control-bg, #2a2e37);
    color: inherit;
    cursor: pointer;
  }

  .mode-toggle button.active,
  .step-options button.active {
    background: var(--control-active-bg, #3d6bff);
    border-color: var(--control-active-bg, #3d6bff);
    color: #fff;
  }

  .step-group {
    display: flex;
    flex-direction: column;
    gap: 0.4rem;
  }

  .step-label {
    font-size: 0.85rem;
    opacity: 0.8;
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
    width: 3.5rem;
    font-weight: 600;
  }

  .axis-row button {
    min-width: 44px;
    min-height: 44px;
    font-size: 1.3rem;
    border-radius: 0.5rem;
    border: 1px solid var(--control-border, #444);
    background: var(--control-bg, #2a2e37);
    color: inherit;
    cursor: pointer;
  }

  .axis-row button:disabled {
    opacity: 0.4;
    cursor: not-allowed;
  }

  .readout {
    display: flex;
    flex-direction: column;
    gap: 0.25rem;
    padding: 0.6rem 0.75rem;
    background: #12141a;
    border: 1px solid var(--control-border, #333);
    border-radius: 0.5rem;
    font-variant-numeric: tabular-nums;
    font-size: 0.9rem;
  }

  .readout-line {
    display: flex;
    gap: 0.9rem;
    flex-wrap: wrap;
  }

  .readout-line.joints {
    color: #c8ccd2;
  }

  .error {
    color: #ff8080;
    font-size: 0.85rem;
  }
</style>
