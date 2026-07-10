<script lang="ts">
  import { createResourceMutation, createResourceQuery } from '@viamrobotics/svelte-sdk'
  import { poseTrackerClient } from '../lib/clients'
  import { useMachineId } from '../lib/machine'
  import { capturePoint, clearBuffer, getBuffer, toCommandArgs, type BufferResponse } from '../lib/poseTracker'

  const machineId = useMachineId()
  const pt = poseTrackerClient(machineId)

  const buffer = createResourceQuery(pt, 'doCommand', () => toCommandArgs(getBuffer()))
  const capture = createResourceMutation(pt, 'doCommand')
  const clear = createResourceMutation(pt, 'doCommand')

  const points = $derived((buffer.data as BufferResponse | undefined)?.points ?? [])

  async function handleCapture() {
    await capture.mutateAsync(toCommandArgs(capturePoint()))
    await buffer.refetch()
  }

  async function handleClear() {
    await clear.mutateAsync(toCommandArgs(clearBuffer()))
    await buffer.refetch()
  }

  function fmt(n: number): string {
    return n.toFixed(2)
  }
</script>

<section class="capture-panel">
  <header>
    <h2>Capture</h2>
    <div class="actions">
      <button type="button" onclick={handleCapture} disabled={capture.isPending}>
        {capture.isPending ? 'Capturing…' : 'Capture point'}
      </button>
      <button
        type="button"
        class="secondary"
        onclick={handleClear}
        disabled={clear.isPending || points.length === 0}
      >
        Clear
      </button>
    </div>
  </header>

  {#if capture.error}
    <p class="error">{capture.error.message}</p>
  {/if}
  {#if clear.error}
    <p class="error">{clear.error.message}</p>
  {/if}
  {#if buffer.error}
    <p class="error">{buffer.error.message}</p>
  {/if}

  {#if points.length === 0}
    <p class="notice">No points captured yet.</p>
  {:else}
    <table>
      <thead>
        <tr>
          <th>#</th>
          <th>X</th>
          <th>Y</th>
          <th>Z</th>
          <th>O&#8339;</th>
          <th>O&#8340;</th>
          <th>O&#8342;</th>
          <th>&#952;</th>
        </tr>
      </thead>
      <tbody>
        {#each points as point, i (i)}
          <tr>
            <td>{i}</td>
            <td>{fmt(point.x)}</td>
            <td>{fmt(point.y)}</td>
            <td>{fmt(point.z)}</td>
            <td>{fmt(point.o_x)}</td>
            <td>{fmt(point.o_y)}</td>
            <td>{fmt(point.o_z)}</td>
            <td>{fmt(point.theta)}</td>
          </tr>
        {/each}
      </tbody>
    </table>
  {/if}
</section>

<style>
  .capture-panel {
    display: flex;
    flex-direction: column;
    gap: 0.75rem;
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

  .actions {
    display: flex;
    gap: 0.5rem;
  }

  button {
    min-height: 44px;
    padding: 0.5rem 1.1rem;
    border-radius: 0.4rem;
    border: 1px solid var(--control-border, #444);
    background: var(--control-active-bg, #3d6bff);
    color: #fff;
    cursor: pointer;
  }

  button.secondary {
    background: var(--control-bg, #2a2e37);
    color: inherit;
  }

  button:disabled {
    opacity: 0.5;
    cursor: not-allowed;
  }

  table {
    border-collapse: collapse;
    width: 100%;
    font-variant-numeric: tabular-nums;
    font-size: 0.9rem;
  }

  th,
  td {
    padding: 0.35rem 0.6rem;
    text-align: right;
    border-bottom: 1px solid var(--control-border, #333);
  }

  th:first-child,
  td:first-child {
    text-align: left;
  }

  .notice {
    color: #9aa0a6;
    font-style: italic;
    font-size: 0.9rem;
  }

  .error {
    color: #ff8080;
    font-size: 0.85rem;
  }
</style>
