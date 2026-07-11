<script lang="ts">
  import { MachineConnectionEvent } from '@viamrobotics/sdk'
  import {
    createResourceMutation,
    createResourceQuery,
    useConnectionStatus,
    usePolling,
  } from '@viamrobotics/svelte-sdk'
  import { poseTrackerClient } from '../lib/clients'
  import { useMachineId } from '../lib/machine'
  import { selectedResource } from '../lib/resource.svelte'
  import { motion } from '../lib/motion.svelte'
  import { armState } from '../lib/armState.svelte'
  import { getArmState, parseArmState, stopArm, toCommandArgs } from '../lib/poseTracker'

  const machineId = useMachineId()
  const pt = poseTrackerClient(machineId, () => selectedResource.name ?? '')

  // Live arm-state poll: 500ms, paused while a jog/move is in flight so the
  // poll doesn't race the in-flight command's own state mutation. Also paused
  // once we know there's no arm configured — the initial query still runs (so
  // we can detect that condition), but polling every 500ms an endpoint that
  // will only ever error is pointless churn.
  const armStateQuery = createResourceQuery(pt, 'doCommand', () => toCommandArgs(getArmState()))
  usePolling(
    () => armStateQuery.queryKey,
    () => (noArmConfigured || motion.busy > 0 ? false : 500),
  )

  // Stop is intentionally its own mutation (not `withMove`) — it must be
  // callable, and immediately in-flight, even while a jog move is pending.
  const stop = createResourceMutation(pt, 'doCommand')

  const connectionStatus = useConnectionStatus(machineId)

  const NO_ARM_PHRASE = 'arm dependency not configured'

  // Distinguish "no arm configured" (expected, quiet) from a genuine error
  // (transport failure, bug, etc — surfaced to the operator).
  const noArmConfigured = $derived(
    armStateQuery.error !== null && armStateQuery.error.message.includes(NO_ARM_PHRASE),
  )
  const unexpectedError = $derived(
    armStateQuery.error !== null && !noArmConfigured ? armStateQuery.error.message : undefined,
  )

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
    // Fire-and-forget from the click handler's perspective; the mutation's
    // own pending/error state drives the button label and any error text.
    // `.mutate` (not `.mutateAsync`) so a rejected command surfaces via
    // `stop.error` without producing an unhandled promise rejection.
    stop.mutate(toCommandArgs(stopArm()))
  }

  function fmt(n: number | undefined): string {
    return n === undefined ? '—' : n.toFixed(2)
  }
</script>

<div class="status-bar">
  <div class="connection">
    <span class="dot {connectionClass}"></span>
    <span>{connectionLabel}</span>
  </div>

  <div class="readout">
    {#if !armState.hasArm}
      {#if noArmConfigured}
        <span class="muted">No arm configured</span>
      {:else if armStateQuery.isLoading}
        <span class="muted">Loading arm state…</span>
      {:else}
        <span class="muted">Arm state unavailable</span>
      {/if}
      {#if unexpectedError}
        <span class="error">{unexpectedError}</span>
      {/if}
    {:else if armState.pose}
      <div class="pose">
        <span>X {fmt(armState.pose.x)}</span>
        <span>Y {fmt(armState.pose.y)}</span>
        <span>Z {fmt(armState.pose.z)}</span>
        <span>O&#8339; {fmt(armState.pose.o_x)}</span>
        <span>O&#8340; {fmt(armState.pose.o_y)}</span>
        <span>O&#8342; {fmt(armState.pose.o_z)}</span>
        <span>&#952; {fmt(armState.pose.theta)}</span>
      </div>
      <div class="joints">
        {#each armState.joints as joint, i (i)}
          <span>J{i} {fmt(joint)}&deg;</span>
        {/each}
      </div>
    {/if}
  </div>

  <button
    type="button"
    class="stop"
    onclick={handleStop}
    disabled={stop.isPending}
    aria-label="Stop arm"
  >
    {stop.isPending ? 'Stopping…' : 'STOP'}
  </button>
  {#if stop.error}
    <span class="error">{stop.error.message}</span>
  {/if}
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

  .readout {
    display: flex;
    flex-direction: column;
    gap: 0.15rem;
    flex: 1;
    min-width: 0;
    font-variant-numeric: tabular-nums;
    font-size: 0.9rem;
  }

  .pose,
  .joints {
    display: flex;
    gap: 0.9rem;
    flex-wrap: wrap;
  }

  .muted {
    color: #9aa0a6;
    font-style: italic;
  }

  .error {
    color: #ff8080;
    font-size: 0.85rem;
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
</style>
