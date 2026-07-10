<script lang="ts">
  import { createResourceMutation, createResourceQuery } from '@viamrobotics/svelte-sdk'
  import { poseTrackerClient } from '../lib/clients'
  import { useMachineId } from '../lib/machine'
  import {
    clearFrames,
    defineFrame,
    deleteFrame,
    listFrames,
    toCommandArgs,
    type DefineFrameResponse,
    type FrameMethod,
    type FramesResponse,
  } from '../lib/poseTracker'

  const machineId = useMachineId()
  const pt = poseTrackerClient(machineId)

  const frames = createResourceQuery(pt, 'doCommand', () => toCommandArgs(listFrames()))
  const define = createResourceMutation(pt, 'doCommand')
  const del = createResourceMutation(pt, 'doCommand')
  const clearAll = createResourceMutation(pt, 'doCommand')

  const METHOD_HINTS: Record<FrameMethod, string> = {
    '3point': 'requires 3 captured points',
    point: 'requires 1 captured point',
    tcp_snapshot: 'requires 1 captured point (used as-is)',
  }
  const METHODS: FrameMethod[] = ['3point', 'point', 'tcp_snapshot']

  let name = $state('')
  let method = $state<FrameMethod>('3point')
  let lastResult: DefineFrameResponse | undefined = $state(undefined)

  const frameEntries = $derived(Object.entries((frames.data as FramesResponse | undefined)?.frames ?? {}))

  async function handleDefine() {
    lastResult = undefined
    const resp = (await define.mutateAsync(
      toCommandArgs(defineFrame(name, method)),
    )) as unknown as DefineFrameResponse
    lastResult = resp
    if (resp.committed) {
      name = ''
    }
    await frames.refetch()
  }

  async function handleDelete(frameName: string) {
    await del.mutateAsync(toCommandArgs(deleteFrame(frameName)))
    await frames.refetch()
  }

  async function handleClearAll() {
    await clearAll.mutateAsync(toCommandArgs(clearFrames()))
    await frames.refetch()
  }

  function fmt(n: number): string {
    return n.toFixed(2)
  }
</script>

<section class="frame-panel">
  <header>
    <h2>Frames</h2>
  </header>

  <div class="define-form">
    <label>
      <span>Name</span>
      <input type="text" bind:value={name} placeholder="frame name" />
    </label>

    <fieldset>
      <legend>Method</legend>
      {#each METHODS as m (m)}
        <label class="method-option">
          <input type="radio" name="method" value={m} bind:group={method} />
          <span>{m}</span>
          <span class="hint">({METHOD_HINTS[m]})</span>
        </label>
      {/each}
    </fieldset>

    <button type="button" onclick={handleDefine} disabled={define.isPending || name.trim() === ''}>
      {define.isPending ? 'Defining…' : 'Define'}
    </button>

    {#if define.error}
      <p class="error">{define.error.message}</p>
    {/if}
    {#if lastResult}
      {#if lastResult.committed}
        <p class="success">
          "{lastResult.name}" {lastResult.replaced ? 'replaced' : 'committed'}.
        </p>
      {:else}
        <p class="notice">
          "{lastResult.name}" computed but not persisted — check platform credentials.
        </p>
      {/if}
    {/if}
  </div>

  <div class="frame-list">
    <div class="list-header">
      <h3>Committed frames</h3>
      <button
        type="button"
        class="secondary"
        onclick={handleClearAll}
        disabled={clearAll.isPending || frameEntries.length === 0}
      >
        Clear all
      </button>
    </div>

    {#if frames.error}
      <p class="error">{frames.error.message}</p>
    {/if}
    {#if del.error}
      <p class="error">{del.error.message}</p>
    {/if}
    {#if clearAll.error}
      <p class="error">{clearAll.error.message}</p>
    {/if}

    {#if frameEntries.length === 0}
      <p class="notice">No frames defined yet.</p>
    {:else}
      <table>
        <thead>
          <tr>
            <th>Name</th>
            <th>X</th>
            <th>Y</th>
            <th>Z</th>
            <th></th>
          </tr>
        </thead>
        <tbody>
          {#each frameEntries as [frameName, pose] (frameName)}
            <tr>
              <td>{frameName}</td>
              <td>{fmt(pose.x)}</td>
              <td>{fmt(pose.y)}</td>
              <td>{fmt(pose.z)}</td>
              <td>
                <button type="button" class="secondary" onclick={() => handleDelete(frameName)} disabled={del.isPending}>
                  Delete
                </button>
              </td>
            </tr>
          {/each}
        </tbody>
      </table>
    {/if}
  </div>
</section>

<style>
  .frame-panel {
    display: flex;
    flex-direction: column;
    gap: 1rem;
    padding: 1rem;
  }

  h2 {
    margin: 0;
    font-size: 1.15rem;
  }

  h3 {
    margin: 0;
    font-size: 1rem;
  }

  .define-form {
    display: flex;
    flex-direction: column;
    gap: 0.6rem;
  }

  label {
    display: flex;
    flex-direction: column;
    gap: 0.25rem;
    font-size: 0.9rem;
  }

  input[type='text'] {
    min-height: 44px;
    padding: 0 0.6rem;
    border-radius: 0.4rem;
    border: 1px solid var(--control-border, #444);
    background: var(--control-bg, #2a2e37);
    color: inherit;
  }

  fieldset {
    border: 1px solid var(--control-border, #444);
    border-radius: 0.4rem;
    display: flex;
    flex-direction: column;
    gap: 0.35rem;
    padding: 0.6rem 0.75rem;
  }

  legend {
    font-size: 0.8rem;
    opacity: 0.8;
    padding: 0 0.3rem;
  }

  .method-option {
    display: flex;
    flex-direction: row;
    align-items: center;
    gap: 0.4rem;
    font-size: 0.9rem;
  }

  .hint {
    opacity: 0.7;
    font-size: 0.8rem;
  }

  button {
    min-height: 44px;
    padding: 0.5rem 1.1rem;
    border-radius: 0.4rem;
    border: 1px solid var(--control-border, #444);
    background: var(--control-active-bg, #3d6bff);
    color: #fff;
    cursor: pointer;
    align-self: flex-start;
  }

  button.secondary {
    background: var(--control-bg, #2a2e37);
    color: inherit;
  }

  button:disabled {
    opacity: 0.5;
    cursor: not-allowed;
  }

  .list-header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    gap: 0.75rem;
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

  .success {
    color: #3ecf6a;
    font-size: 0.9rem;
  }

  .error {
    color: #ff8080;
    font-size: 0.85rem;
  }
</style>
