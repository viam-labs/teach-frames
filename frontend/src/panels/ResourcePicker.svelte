<script lang="ts">
  import { useResourceNames } from '@viamrobotics/svelte-sdk'
  import { useMachineId } from '../lib/machine'
  import { selectedResource } from '../lib/resource.svelte'

  const machineId = useMachineId()

  // Discover every resource on the machine that implements the PoseTracker API
  // (rdk:component:pose_tracker → subtype "pose_tracker").
  const resources = useResourceNames(machineId, 'pose_tracker')

  // Default the selection to the first discovered resource, once they load.
  // Guarded so it sets exactly once and never clobbers an operator's choice.
  let choice = $state<string | null>(null)
  $effect(() => {
    if (choice === null && resources.current.length > 0) {
      choice = resources.current[0].name
    }
  })

  function start() {
    if (choice) {
      selectedResource.name = choice
    }
  }
</script>

<main class="onboarding">
  <div class="card">
    <h1>Teach Pendant</h1>
    <p class="lead">Choose the pose-tracker resource to teach with.</p>

    {#if resources.current.length === 0}
      <p class="muted">Discovering pose-tracker resources…</p>
      <p class="hint">
        If none appear, add a <code>pose_tracker</code> component (for example
        <code>viam-labs:teach-frames:pose-tracker</code>) to this machine and reload.
      </p>
    {:else}
      <label for="resource">Pose-tracker resource</label>
      <select id="resource" bind:value={choice}>
        {#each resources.current as resource (resource.name)}
          <option value={resource.name}>{resource.name}</option>
        {/each}
      </select>
      <button type="button" onclick={start} disabled={!choice}>Start teaching</button>
    {/if}
  </div>
</main>

<style>
  .onboarding {
    min-height: 100vh;
    display: flex;
    align-items: center;
    justify-content: center;
    padding: 1.5rem;
  }

  .card {
    width: 100%;
    max-width: 26rem;
    display: flex;
    flex-direction: column;
    gap: 0.75rem;
    padding: 1.75rem;
    background: var(--surface-panel);
    border: 1px solid var(--border-panel);
    border-radius: var(--radius-xl);
  }

  h1 {
    margin: 0;
    font-size: 1.4rem;
  }

  .lead {
    margin: 0;
    color: var(--ink-secondary);
  }

  label {
    font-size: 0.9rem;
    color: var(--ink-secondary);
    margin-top: 0.25rem;
  }

  select {
    min-height: 44px;
    padding: 0.5rem 0.6rem;
    border-radius: var(--radius-md);
    border: 1px solid var(--border-control);
    background: var(--surface-control);
    color: inherit;
    font-size: 1rem;
  }

  button {
    min-height: 44px;
    margin-top: 0.5rem;
    padding: 0.6rem 1.1rem;
    border-radius: var(--radius-md);
    border: none;
    background: var(--accent);
    color: var(--ink-on-accent);
    font-size: 1rem;
    font-weight: 600;
    cursor: pointer;
  }

  button:disabled {
    opacity: 0.5;
    cursor: not-allowed;
  }

  .muted {
    margin: 0;
    color: var(--ink-muted);
    font-style: italic;
  }

  .hint {
    margin: 0;
    font-size: 0.85rem;
    color: var(--ink-muted);
  }

  code {
    background: var(--surface-base);
    padding: 0.1rem 0.3rem;
    border-radius: var(--radius-sm);
    font-size: 0.85em;
  }
</style>
