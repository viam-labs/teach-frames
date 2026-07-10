<script lang="ts">
  import { createResourceMutation } from '@viamrobotics/svelte-sdk'
  import { poseTrackerClient } from '../lib/clients'
  import { useMachineId } from '../lib/machine'
  import { armState } from '../lib/armState.svelte'
  import { motion, withMove } from '../lib/motion.svelte'
  import { jogCartesian, jogJoint, toCommandArgs, type CartesianAxis } from '../lib/poseTracker'

  const machineId = useMachineId()
  const pt = poseTrackerClient(machineId)
  const jog = createResourceMutation(pt, 'doCommand')

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

  {#if !armState.hasArm}
    <p class="notice">No arm configured — jogging is disabled.</p>
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
        <p class="notice">No joint data available.</p>
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

  .notice {
    color: #d9a441;
    font-size: 0.9rem;
  }

  .error {
    color: #ff8080;
    font-size: 0.85rem;
  }
</style>
