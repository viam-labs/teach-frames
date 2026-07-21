<script lang="ts">
  // Floating panel for the OTHER half of the frame lifecycle: listing,
  // deleting, and clearing already-committed frames. Defining a frame lives
  // in FrameDefineWizard.svelte — this panel intentionally has no define UI,
  // ported (list/delete/clear + confirm-before-destroy) from the legacy
  // FramePanel.svelte, which also serves as the query/mutation wiring
  // reference; see that file for the "why" behind the 4-arg query form and
  // the confirm state machine.
  import { createResourceMutation, createResourceQuery } from '@viamrobotics/svelte-sdk'
  import { DashboardPortal, FloatingPanel } from '@viamrobotics/motion-tools'
  import { Icon } from '@viamrobotics/prime-core'
  import DashboardToggle from './DashboardToggle.svelte'
  import { poseTrackerClient } from '../lib/clients'
  import { selectedResource } from '../lib/resource.svelte'
  import { useMachineId } from '../lib/machine'
  import { bumpFrameRevision } from '../lib/frameRevision.svelte'
  import {
    clearFrames,
    deleteFrame,
    listFrames,
    toCommandArgs,
    type FramesResponse,
  } from '../lib/poseTracker'

  const machineId = useMachineId()
  const pt = poseTrackerClient(machineId, () => selectedResource.name ?? '')
  // 4th arg (options) required — a lone trailing arg is treated as options and
  // would drop our command args. See FramePanel.svelte.
  const frames = createResourceQuery(pt, 'doCommand', () => toCommandArgs(listFrames()), () => ({}))
  const del = createResourceMutation(pt, 'doCommand')
  const clearAll = createResourceMutation(pt, 'doCommand')

  // Closed by default so it doesn't clutter the chrome alongside the wizard
  // on load — the operator opens it when they want to review/prune frames.
  let open = $state(false)

  const frameEntries = $derived(
    Object.entries((frames.data as FramesResponse | undefined)?.frames ?? {}),
  )

  // Delete and Clear-all persist to config and can't be undone, so they arm an
  // inline confirmation instead of firing on the first click.
  type PendingConfirm = { kind: 'delete' | 'clearAll'; name?: string } | null
  let pendingConfirm = $state<PendingConfirm>(null)
  const confirmBusy = $derived(del.isPending || clearAll.isPending)

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
      // Force the Visualizer to remount and re-snapshot world_state_store —
      // see the {#key frameRevision.value} comment in App.svelte. Covers both
      // 'delete' and 'clearAll', which share this success path.
      bumpFrameRevision()
    } catch {
      // Surfaced via del.error / clearAll.error below.
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

<DashboardPortal>
  <DashboardToggle active={open} label="Manage frames" onclick={() => (open = !open)}>
    {#snippet children()}
      <Icon name="table" />
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
        class="border-medium text-gray-9 hover:border-gray-6 min-h-11 rounded-md border bg-white px-3 text-sm font-medium focus:outline-none focus-visible:ring-2 focus-visible:ring-gray-9/30 disabled:cursor-not-allowed disabled:text-disabled-dark"
      >
        Clear all
      </button>
    </div>

    {#if pendingConfirm}
      <div class="border-danger-dark bg-danger-light flex flex-col gap-2 rounded-md border p-3" role="alert">
        <p class="text-gray-9 text-sm">
          {#if pendingConfirm.kind === 'delete'}
            Delete frame "{pendingConfirm.name}"? This can't be undone.
          {:else}
            Clear all {frameEntries.length} frames? This can't be undone.
          {/if}
        </p>
        <div class="flex gap-2">
          <button
            type="button"
            onclick={runConfirm}
            disabled={confirmBusy}
            class="border-danger-dark bg-danger-dark min-h-11 rounded-md border px-4 text-sm font-medium text-white focus:outline-none focus-visible:ring-2 focus-visible:ring-danger-dark/40 disabled:cursor-not-allowed disabled:opacity-60"
          >
            {#if confirmBusy}
              {pendingConfirm.kind === 'delete' ? 'Deleting…' : 'Clearing…'}
            {:else}
              {pendingConfirm.kind === 'delete' ? 'Delete' : 'Clear all'}
            {/if}
          </button>
          <button
            type="button"
            onclick={() => (pendingConfirm = null)}
            disabled={confirmBusy}
            use:focusOnMount
            class="border-medium text-gray-9 hover:border-gray-6 min-h-11 rounded-md border bg-white px-4 text-sm font-medium focus:outline-none focus-visible:ring-2 focus-visible:ring-gray-9/30"
          >
            Cancel
          </button>
        </div>
      </div>
    {/if}

    {#if frames.error}
      <p class="text-danger-dark text-xs" role="alert">{frames.error.message}</p>
    {/if}
    {#if del.error}
      <p class="text-danger-dark text-xs" role="alert">{del.error.message}</p>
    {/if}
    {#if clearAll.error}
      <p class="text-danger-dark text-xs" role="alert">{clearAll.error.message}</p>
    {/if}

    {#if frameEntries.length === 0}
      <p class="text-subtle-1 text-sm">No frames defined yet.</p>
    {:else}
      <table class="w-full text-sm tabular-nums">
        <thead>
          <tr class="text-subtle-1 text-left text-xs">
            <th class="py-1">Name</th>
            <th class="py-1 text-right">X</th>
            <th class="py-1 text-right">Y</th>
            <th class="py-1 text-right">Z</th>
            <th><span class="sr-only">Actions</span></th>
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
                  class="text-danger-dark min-h-11 rounded-md px-2 text-sm hover:underline focus:outline-none focus-visible:ring-2 focus-visible:ring-gray-9/30 disabled:cursor-not-allowed disabled:text-disabled-dark"
                >
                  Delete
                </button>
              </td>
            </tr>
          {/each}
        </tbody>
      </table>
    {/if}
  </div>
</FloatingPanel>
