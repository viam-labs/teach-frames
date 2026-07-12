<script lang="ts">
  import { createResourceMutation, createResourceQuery } from '@viamrobotics/svelte-sdk'
  import { poseTrackerClient } from '../lib/clients'
  import { selectedResource } from '../lib/resource.svelte'
  import { useMachineId } from '../lib/machine'
  import { bumpCaptureBuffer } from '../lib/captureBuffer.svelte'
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
  const pt = poseTrackerClient(machineId, () => selectedResource.name ?? '')

  // 4th arg (options) required — a lone trailing arg is treated as options and
  // would drop our args, sending an empty command. See StatusBar for detail.
  const frames = createResourceQuery(pt, 'doCommand', () => toCommandArgs(listFrames()), () => ({}))
  const define = createResourceMutation(pt, 'doCommand')
  const del = createResourceMutation(pt, 'doCommand')
  const clearAll = createResourceMutation(pt, 'doCommand')

  const METHOD_HINTS: Record<FrameMethod, string> = {
    '3point': 'requires 3 captured points',
    point: 'requires 1 captured point',
    tcp_snapshot: 'requires 1 captured point (used as-is)',
  }
  // Human-readable labels for the raw method tokens the API expects. The token
  // stays the radio value; only the visible text changes.
  const METHOD_LABELS: Record<FrameMethod, string> = {
    '3point': '3-point',
    point: 'Single point',
    tcp_snapshot: 'TCP snapshot',
  }
  const METHODS: FrameMethod[] = ['3point', 'point', 'tcp_snapshot']

  let name = $state('')
  let method = $state<FrameMethod>('3point')
  let lastResult: DefineFrameResponse | undefined = $state(undefined)

  const frameEntries = $derived(Object.entries((frames.data as FramesResponse | undefined)?.frames ?? {}))

  const trimmedName = $derived(name.trim())
  // Defining with an existing name overwrites it. Surface that BEFORE the user
  // commits, instead of only reporting "replaced" after the fact.
  const willReplace = $derived(frameEntries.some(([n]) => n === trimmedName))

  // Delete and Clear-all persist to config and can't be undone, so they arm an
  // inline confirmation instead of firing on the first click.
  type PendingConfirm = { kind: 'delete' | 'clearAll'; name?: string } | null
  let pendingConfirm = $state<PendingConfirm>(null)
  const confirmBusy = $derived(del.isPending || clearAll.isPending)

  async function handleDefine() {
    lastResult = undefined
    try {
      const resp = (await define.mutateAsync(
        toCommandArgs(defineFrame(name, method)),
      )) as unknown as DefineFrameResponse
      lastResult = resp
      if (resp.committed) {
        name = ''
        // define_frame clears the server-side capture buffer on success;
        // let CapturePanel know so it doesn't keep showing stale points.
        bumpCaptureBuffer()
      }
      await frames.refetch()
    } catch {
      // Surfaced via define.error in the template; swallow here so the
      // rejection doesn't bubble up as an unhandled promise rejection.
    }
  }

  // Runs the armed destructive action. Leaves the confirmation open on failure
  // (the error shows below it) so the user can retry or cancel.
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
      // Surfaced via del.error / clearAll.error in the template.
    }
  }

  // Land keyboard focus on the safe (Cancel) button when the confirmation opens.
  function focusOnMount(node: HTMLElement) {
    node.focus()
  }

  function fmt(n: number): string {
    return n.toFixed(2)
  }
</script>

<section class="frame-panel">
  <header class="panel-header">
    <div class="panel-titles">
      <h2>Frames</h2>
      <p class="panel-subtitle">Compute and store workcell coordinate frames.</p>
    </div>
  </header>

  <div class="define-form">
    <label>
      <span>Name</span>
      <input
        type="text"
        bind:value={name}
        placeholder="frame name"
        oninput={() => (lastResult = undefined)}
      />
    </label>

    <fieldset>
      <legend>Method</legend>
      {#each METHODS as m (m)}
        <label class="method-option">
          <input type="radio" name="method" value={m} bind:group={method} />
          <span>{METHOD_LABELS[m]}</span>
          <span class="hint">({METHOD_HINTS[m]})</span>
        </label>
      {/each}
    </fieldset>

    <button type="button" onclick={handleDefine} disabled={define.isPending || trimmedName === ''}>
      {#if define.isPending}
        {willReplace ? 'Replacing…' : 'Defining…'}
      {:else if willReplace}
        Replace "{trimmedName}"
      {:else}
        Define
      {/if}
    </button>

    {#if willReplace && !define.isPending}
      <p class="panel-warning">A frame named "{trimmedName}" already exists — defining will replace it.</p>
    {/if}

    {#if define.error}
      <p class="error">{define.error.message}</p>
    {/if}
    {#if lastResult}
      {#if lastResult.committed}
        <p class="success">
          "{lastResult.name}" {lastResult.replaced ? 'replaced' : 'committed'}.
        </p>
      {:else}
        <p class="panel-warning">
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
        onclick={() => (pendingConfirm = { kind: 'clearAll' })}
        disabled={confirmBusy || frameEntries.length === 0}
      >
        Clear all
      </button>
    </div>

    {#if pendingConfirm}
      <div class="confirm-bar" role="alert">
        <p>
          {#if pendingConfirm.kind === 'delete'}
            Delete frame "{pendingConfirm.name}"? This can't be undone.
          {:else}
            Clear all {frameEntries.length} frames? This can't be undone.
          {/if}
        </p>
        <div class="confirm-actions">
          <button type="button" class="danger" onclick={runConfirm} disabled={confirmBusy}>
            {#if confirmBusy}
              {pendingConfirm.kind === 'delete' ? 'Deleting…' : 'Clearing…'}
            {:else}
              {pendingConfirm.kind === 'delete' ? 'Delete' : 'Clear all'}
            {/if}
          </button>
          <button
            type="button"
            class="secondary"
            onclick={() => (pendingConfirm = null)}
            disabled={confirmBusy}
            use:focusOnMount
          >
            Cancel
          </button>
        </div>
      </div>
    {/if}

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
      <p class="panel-empty">No frames defined yet.</p>
    {:else}
      <div class="table-scroll">
        <table>
          <thead>
            <tr>
              <th>Name</th>
              <th>X</th>
              <th>Y</th>
              <th>Z</th>
              <th><span class="sr-only">Actions</span></th>
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
                  <button type="button" class="secondary" onclick={() => (pendingConfirm = { kind: 'delete', name: frameName })} disabled={confirmBusy} aria-label="Delete {frameName}">
                    Delete
                  </button>
                </td>
              </tr>
            {/each}
          </tbody>
        </table>
      </div>
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
    border-radius: var(--radius-md);
    border: 1px solid var(--border-control);
    background: var(--surface-control);
    color: inherit;
  }

  fieldset {
    border: 1px solid var(--border-control);
    border-radius: var(--radius-md);
    display: flex;
    flex-direction: column;
    gap: 0.35rem;
    padding: 0.6rem 0.75rem;
  }

  legend {
    font-size: 0.8rem;
    color: var(--ink-muted);
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
    color: var(--ink-muted);
    font-size: 0.8rem;
  }

  button {
    min-height: 44px;
    padding: 0.5rem 1.1rem;
    border-radius: var(--radius-md);
    border: 1px solid var(--border-control);
    background: var(--accent);
    color: var(--ink-on-accent);
    cursor: pointer;
    align-self: flex-start;
  }

  button.secondary {
    background: var(--surface-control);
    color: inherit;
  }

  /* Destructive commit — the confirm button inside the confirmation bar. */
  button.danger {
    background: var(--danger);
    border-color: var(--danger);
    color: var(--ink-on-accent);
  }

  button:disabled {
    opacity: 0.5;
    cursor: not-allowed;
  }

  /* Inline confirmation for destructive, non-undoable actions — roomier and
     clearer than cramming Confirm/Cancel into a table cell, no modal needed. */
  .confirm-bar {
    display: flex;
    align-items: center;
    justify-content: space-between;
    gap: 0.5rem 1rem;
    flex-wrap: wrap;
    padding: 0.6rem 0.75rem;
    border: 1px solid var(--danger);
    border-radius: var(--radius-md);
    background: color-mix(in oklab, var(--danger) 12%, var(--surface-panel));
  }

  .confirm-bar p {
    margin: 0;
    font-size: 0.9rem;
  }

  .confirm-actions {
    display: flex;
    gap: 0.5rem;
  }

  /* Divider between the define zone and the committed-frames zone, so the
     panel reads as "define, then review" rather than two evenly-gapped peers. */
  .frame-list {
    padding-top: 0.75rem;
    border-top: 1px solid var(--border-panel);
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
    border-bottom: 1px solid var(--border-panel);
  }

  th:first-child,
  td:first-child {
    text-align: left;
  }

  .success {
    color: var(--success);
    font-size: 0.9rem;
  }

  .error {
    color: var(--error-text);
    font-size: 0.85rem;
  }
</style>
