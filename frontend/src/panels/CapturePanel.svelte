<script lang="ts">
  import { untrack } from 'svelte'
  import { createResourceMutation, createResourceQuery } from '@viamrobotics/svelte-sdk'
  import { poseTrackerClient } from '../lib/clients'
  import { selectedResource } from '../lib/resource.svelte'
  import { useMachineId } from '../lib/machine'
  import { captureBufferSignal } from '../lib/captureBuffer.svelte'
  import { capturePoint, clearBuffer, getBuffer, toCommandArgs, type BufferResponse } from '../lib/poseTracker'

  const machineId = useMachineId()
  const pt = poseTrackerClient(machineId, () => selectedResource.name ?? '')

  // 4th arg (options) required — a lone trailing arg is treated as options and
  // would drop our args, sending an empty command. See StatusBar for detail.
  const buffer = createResourceQuery(pt, 'doCommand', () => toCommandArgs(getBuffer()), () => ({}))
  const capture = createResourceMutation(pt, 'doCommand')
  const clear = createResourceMutation(pt, 'doCommand')

  const points = $derived((buffer.data as BufferResponse | undefined)?.points ?? [])

  // Other panels (e.g. FramePanel after a successful define) can clear the
  // server-side buffer out from under us. Refetch whenever that signal bumps
  // so we don't keep displaying points the server already discarded.
  //
  // This effect MUST depend ONLY on captureBufferSignal.version. The refetch()
  // call is wrapped in untrack() because reading `buffer` here would otherwise
  // subscribe this effect to the TanStack query result; refetch() then mutates
  // that result (svelte-query reassigns every result field on each observer
  // notification), which would re-fire this effect and spin forever, hanging
  // the page. version starts at 0 and only ever increments, so we skip the
  // initial mount run — the buffer query already fetches on mount on its own.
  $effect(() => {
    const version = captureBufferSignal.version
    if (version === 0) return
    untrack(() => {
      void buffer.refetch()
    })
  })

  async function handleCapture() {
    try {
      await capture.mutateAsync(toCommandArgs(capturePoint()))
      await buffer.refetch()
    } catch {
      // Surfaced via capture.error in the template.
    }
  }

  async function handleClear() {
    try {
      await clear.mutateAsync(toCommandArgs(clearBuffer()))
      await buffer.refetch()
    } catch {
      // Surfaced via clear.error in the template.
    }
  }

  function fmt(n: number): string {
    return n.toFixed(2)
  }
</script>

<section class="capture-panel">
  <header class="panel-header">
    <div class="panel-titles">
      <h2>Capture Points</h2>
      <p class="panel-subtitle">Reference points used to define a frame.</p>
    </div>
    <div class="panel-actions">
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
    <p class="panel-empty">No points captured yet.</p>
  {:else}
    <div class="table-scroll">
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
    </div>
  {/if}
</section>

<style>
  .capture-panel {
    display: flex;
    flex-direction: column;
    gap: 1rem;
    padding: 1rem;
  }

  button {
    min-height: 44px;
    padding: 0.5rem 1.1rem;
    border-radius: var(--radius-md);
    border: 1px solid var(--border-control);
    background: var(--accent);
    color: var(--ink-on-accent);
    cursor: pointer;
  }

  button.secondary {
    background: var(--surface-control);
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
    border-bottom: 1px solid var(--border-panel);
  }

  th:first-child,
  td:first-child {
    text-align: left;
  }

  .error {
    color: var(--error-text);
    font-size: 0.85rem;
  }
</style>
