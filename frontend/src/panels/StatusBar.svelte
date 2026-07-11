<script lang="ts">
  import { MachineConnectionEvent } from '@viamrobotics/sdk'
  import { createResourceMutation, useConnectionStatus } from '@viamrobotics/svelte-sdk'
  import { poseTrackerClient } from '../lib/clients'
  import { useMachineId } from '../lib/machine'
  import { selectedResource } from '../lib/resource.svelte'
  import { requestStop } from '../lib/motion.svelte'
  import { stopArm, toCommandArgs } from '../lib/poseTracker'

  const machineId = useMachineId()
  const pt = poseTrackerClient(machineId, () => selectedResource.name ?? '')

  // Stop is its own mutation so it stays callable — and immediately in-flight —
  // even while a jog move is pending. It is never gated by motion.busy.
  const stop = createResourceMutation(pt, 'doCommand')

  const connectionStatus = useConnectionStatus(machineId)

  const connectionLabel = $derived.by(() => {
    switch (connectionStatus.current) {
      case MachineConnectionEvent.CONNECTED:
        return 'Connected'
      case MachineConnectionEvent.CONNECTING:
      case MachineConnectionEvent.DIALING:
        return 'Connecting'
      case MachineConnectionEvent.DISCONNECTING:
      case MachineConnectionEvent.DISCONNECTED:
        return 'Disconnected'
      default:
        return 'Unknown'
    }
  })

  const connectionClass = $derived(
    connectionStatus.current === MachineConnectionEvent.CONNECTED ? 'connected' : 'not-connected',
  )

  function handleStop() {
    // Cancel any in-progress press-and-hold jog loop first, then command the
    // arm to stop. `.mutate` (not `.mutateAsync`) so a rejected command
    // surfaces via `stop.error` without an unhandled promise rejection.
    requestStop()
    stop.mutate(toCommandArgs(stopArm()))
  }
</script>

<div class="status-bar">
  <div class="connection">
    <span class="dot {connectionClass}"></span>
    <span>{connectionLabel}</span>
  </div>

  <div class="resource" title="Selected pose-tracker resource">
    {selectedResource.name}
  </div>

  <div class="stop-group">
    <button type="button" class="stop" onclick={handleStop} disabled={stop.isPending} aria-label="Stop arm">
      {stop.isPending ? 'Stopping…' : 'STOP'}
    </button>
    {#if stop.error}
      <span class="error">{stop.error.message}</span>
    {/if}
  </div>
</div>

<style>
  .status-bar {
    display: flex;
    align-items: center;
    gap: 1.25rem;
    padding: 0.75rem 1rem;
    background: var(--panel-bg, #1c1f26);
    color: var(--panel-fg, #f2f2f2);
    border-bottom: 1px solid var(--panel-border, #333);
    flex-wrap: wrap;
  }

  .connection {
    display: flex;
    align-items: center;
    gap: 0.4rem;
    font-weight: 600;
    min-width: 7.5rem;
  }

  .dot {
    width: 0.65rem;
    height: 0.65rem;
    border-radius: 50%;
    background: #999;
    flex-shrink: 0;
  }

  .dot.connected {
    background: #3ecf6a;
  }

  .dot.not-connected {
    background: #d94f4f;
  }

  .resource {
    flex: 1;
    min-width: 0;
    color: #c8ccd2;
    font-variant-numeric: tabular-nums;
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
  }

  .stop-group {
    display: flex;
    align-items: center;
    gap: 0.6rem;
  }

  .stop {
    min-width: 6.5rem;
    min-height: 44px;
    padding: 0.5rem 1.5rem;
    font-size: 1.1rem;
    font-weight: 800;
    letter-spacing: 0.04em;
    color: #fff;
    background: #c62828;
    border: none;
    border-radius: 0.5rem;
    cursor: pointer;
  }

  .stop:hover:not(:disabled) {
    background: #e53935;
  }

  .stop:disabled {
    opacity: 0.7;
    cursor: wait;
  }

  .error {
    color: #ff8080;
    font-size: 0.85rem;
  }
</style>
