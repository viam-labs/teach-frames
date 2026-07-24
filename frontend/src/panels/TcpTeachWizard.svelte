<script lang="ts">
  // Guided TCP calibration, mounted in <Visualizer>'s children (see App.svelte).
  // Store lives in `wizard` (created in App.svelte) so panel state survives the
  // Bug-1 frameRevision remount. Drives the shipped pose-tracker TCP DoCommands.
  import { createResourceMutation, createResourceQuery } from '@viamrobotics/svelte-sdk'
  import { DashboardPortal, FloatingPanel } from '@viamrobotics/motion-tools'
  import { Icon } from '@viamrobotics/prime-core'
  import DashboardToggle from './DashboardToggle.svelte'
  import { poseTrackerClient } from '../lib/clients'
  import { selectedResource } from '../lib/resource.svelte'
  import { useMachineId } from '../lib/machine'
  import { armState } from '../lib/armState.svelte'
  import { motion } from '../lib/motion.svelte'
  import {
    captureTcpPoint, clearTcpBuffer, getTcpBuffer, teachTcpPosition, teachTcpOrientation,
    toCommandArgs,
    type CaptureResponse, type BufferResponse,
    type TeachTcpPositionResponse, type TeachTcpOrientationResponse,
  } from '../lib/poseTracker'
  import type { createTcpTeachWizard } from '../lib/wizard/tcpTeach.svelte'

  let { wizard }: { wizard: ReturnType<typeof createTcpTeachWizard> } = $props()

  const machineId = useMachineId()
  const pt = poseTrackerClient(machineId, () => selectedResource.name ?? '')
  const capture = createResourceMutation(pt, 'doCommand')
  const clear = createResourceMutation(pt, 'doCommand')
  const teachPos = createResourceMutation(pt, 'doCommand')
  const teachOri = createResourceMutation(pt, 'doCommand')
  // 4th options arg required (see FrameDefineWizard/ManageFramesPanel).
  const bufferQ = createResourceQuery(pt, 'doCommand', () => toCommandArgs(getTcpBuffer()), () => ({}))

  let open = $state(false)
  // Explicit orientation inputs (orientation-vector degrees). Default = flange axis.
  let oX = $state(0), oY = $state(0), oZ = $state(1), theta = $state(0)

  const points = $derived((bufferQ.data as BufferResponse | undefined)?.points ?? [])
  const busy = $derived(!armState.hasArm || motion.busy > 0)

  function errorMessage(err: unknown): string {
    return err instanceof Error ? err.message : String(err)
  }
  async function clearServerBuffer() {
    try { await clear.mutateAsync(toCommandArgs(clearTcpBuffer())) } catch { /* best-effort */ }
  }

  async function handleStart() { await clearServerBuffer(); wizard.start(); void bufferQ.refetch() }

  async function handleCapture() {
    if (capture.isPending) return
    try {
      const res = (await capture.mutateAsync(toCommandArgs(captureTcpPoint()))) as unknown as CaptureResponse
      wizard.recordCapture(res.buffer_len)
      void bufferQ.refetch()
    } catch (err) { wizard.setError(errorMessage(err)) }
  }

  async function handleTeachPosition() {
    if (teachPos.isPending) return
    try {
      const res = (await teachPos.mutateAsync(toCommandArgs(teachTcpPosition()))) as unknown as TeachTcpPositionResponse
      if (res?.committed) { wizard.solved({ offset: res.offset, residual_rms: res.residual_rms }); void bufferQ.refetch() }
      else wizard.setError('teach_tcp_position did not commit (check platform credentials)')
    } catch (err) {
      // Surfaces the backend degeneracy error (too few / near-parallel poses);
      // the backend preserves the buffer so the operator can add more touches.
      wizard.setError(errorMessage(err))
    }
  }

  async function handleReteach() {
    if (clear.isPending) return
    await clearServerBuffer()
    wizard.reteach()
    void bufferQ.refetch()
  }

  async function handleTeachOrientation() {
    if (teachOri.isPending) return
    try {
      const res = (await teachOri.mutateAsync(
        toCommandArgs(teachTcpOrientation(oX, oY, oZ, theta)),
      )) as unknown as TeachTcpOrientationResponse
      if (res?.committed) wizard.finish()
      else wizard.setError('teach_tcp_orientation did not commit')
    } catch (err) { wizard.setError(errorMessage(err)) }
  }

  function handleTeachAgain() { oX = 0; oY = 0; oZ = 1; theta = 0; wizard.reset() }
  function fmt(n: number): string { return n.toFixed(2) }
</script>

<DashboardPortal>
  <DashboardToggle active={open} label="Teach TCP" onclick={() => (open = !open)}>
    {#snippet children()}
      <Icon name="axis-arrow" />
    {/snippet}
  </DashboardToggle>
</DashboardPortal>

<FloatingPanel
  title="Teach TCP"
  bind:isOpen={open}
  defaultPosition={{ x: 688, y: 88 }}
  defaultSize={{ width: 320, height: 380 }}
  resizable
>
  <div class="flex h-full flex-col gap-3 overflow-y-auto p-4">
    {#if wizard.phase === 'setup'}
      <p class="text-gray-9 text-sm">
        Touch the same fixed point with the tool tip from ≥4 different arm orientations, capturing each.
      </p>
      {#if !armState.hasArm}
        <p class="text-subtle-1 text-xs">Requires a configured arm.</p>
      {/if}
      <button
        type="button"
        onclick={handleStart}
        disabled={busy}
        class="border-gray-9 bg-gray-9 min-h-11 rounded-md border px-4 text-sm font-medium text-white hover:border-black hover:bg-black active:bg-black focus:outline-none focus-visible:ring-2 focus-visible:ring-gray-9/40 focus-visible:ring-offset-2 disabled:cursor-not-allowed disabled:!border-disabled-light disabled:!bg-disabled-light disabled:text-disabled-dark"
      >
        Start
      </button>
    {:else if wizard.phase === 'capturing'}
      <div aria-live="polite">
        <p class="text-gray-9 text-sm">Jog the tool tip to the point, then Capture.</p>
        <p class="text-subtle-1 text-xs">{wizard.captureCount} / {wizard.minCaptures}+ captured</p>
      </div>
      {#if points.length > 0}
        <div class="max-h-24 overflow-y-auto">
          <table class="w-full text-xs">
            <thead>
              <tr class="text-subtle-2">
                <th class="text-left font-normal">#</th>
                <th class="text-right font-normal">X</th>
                <th class="text-right font-normal">Y</th>
                <th class="text-right font-normal">Z</th>
              </tr>
            </thead>
            <tbody>
              {#each points as point, i (i)}
                <tr class="text-gray-9">
                  <td class="text-left">{i}</td>
                  <td class="text-right">{fmt(point.x)}</td>
                  <td class="text-right">{fmt(point.y)}</td>
                  <td class="text-right">{fmt(point.z)}</td>
                </tr>
              {/each}
            </tbody>
          </table>
        </div>
      {/if}
      <div class="flex gap-2">
        <button
          type="button"
          onclick={handleCapture}
          disabled={capture.isPending || busy}
          class="border-gray-9 bg-gray-9 min-h-11 rounded-md border px-4 text-sm font-medium text-white hover:border-black hover:bg-black active:bg-black focus:outline-none focus-visible:ring-2 focus-visible:ring-gray-9/40 focus-visible:ring-offset-2 disabled:cursor-not-allowed disabled:!border-disabled-light disabled:!bg-disabled-light disabled:text-disabled-dark"
        >
          {capture.isPending ? 'Capturing…' : 'Capture'}
        </button>
        <button
          type="button"
          onclick={handleTeachPosition}
          disabled={!wizard.canSolve || teachPos.isPending}
          class="border-gray-9 bg-gray-9 min-h-11 rounded-md border px-4 text-sm font-medium text-white hover:border-black hover:bg-black active:bg-black focus:outline-none focus-visible:ring-2 focus-visible:ring-gray-9/40 focus-visible:ring-offset-2 disabled:cursor-not-allowed disabled:!border-disabled-light disabled:!bg-disabled-light disabled:text-disabled-dark"
        >
          {teachPos.isPending ? 'Solving…' : 'Teach position'}
        </button>
      </div>
    {:else if wizard.phase === 'taught'}
      <p class="text-gray-9 text-sm">TCP taught.</p>
      <p class="text-gray-9 text-sm font-medium">
        TCP error: {fmt(wizard.position?.residual_rms ?? 0)} mm
      </p>
      <p class="text-subtle-1 text-xs">
        offset ({fmt(wizard.position?.offset.x ?? 0)}, {fmt(wizard.position?.offset.y ?? 0)}, {fmt(
          wizard.position?.offset.z ?? 0,
        )}) mm
      </p>
      <div class="flex gap-2">
        <button
          type="button"
          onclick={handleReteach}
          class="border-medium text-gray-9 hover:border-gray-6 min-h-11 rounded-md border bg-white px-4 text-sm font-medium focus:outline-none focus-visible:ring-2 focus-visible:ring-gray-9/30"
        >
          Re-teach
        </button>
        <button
          type="button"
          onclick={() => wizard.toOrientation()}
          class="border-medium text-gray-9 hover:border-gray-6 min-h-11 rounded-md border bg-white px-4 text-sm font-medium focus:outline-none focus-visible:ring-2 focus-visible:ring-gray-9/30"
        >
          Set orientation
        </button>
        <button
          type="button"
          onclick={() => wizard.finish()}
          class="border-gray-9 bg-gray-9 min-h-11 rounded-md border px-4 text-sm font-medium text-white hover:border-black hover:bg-black active:bg-black focus:outline-none focus-visible:ring-2 focus-visible:ring-gray-9/40 focus-visible:ring-offset-2"
        >
          Finish
        </button>
      </div>
    {:else if wizard.phase === 'orientation'}
      <p class="text-subtle-1 text-xs">Most tools can leave this at the flange orientation.</p>
      <div class="flex flex-wrap gap-2">
        <label class="flex flex-col gap-1 text-sm">
          <span class="text-subtle-1">O&#8339;</span>
          <input
            type="number"
            step="any"
            bind:value={oX}
            aria-label="Orientation vector X"
            class="border-medium hover:border-gray-6 focus:border-gray-9 text-gray-9 min-h-11 w-20 rounded-md border bg-white px-3 text-sm focus:outline-none focus-visible:ring-2 focus-visible:ring-gray-9/30"
          />
        </label>
        <label class="flex flex-col gap-1 text-sm">
          <span class="text-subtle-1">O&#8340;</span>
          <input
            type="number"
            step="any"
            bind:value={oY}
            aria-label="Orientation vector Y"
            class="border-medium hover:border-gray-6 focus:border-gray-9 text-gray-9 min-h-11 w-20 rounded-md border bg-white px-3 text-sm focus:outline-none focus-visible:ring-2 focus-visible:ring-gray-9/30"
          />
        </label>
        <label class="flex flex-col gap-1 text-sm">
          <span class="text-subtle-1">O&#8342;</span>
          <input
            type="number"
            step="any"
            bind:value={oZ}
            aria-label="Orientation vector Z"
            class="border-medium hover:border-gray-6 focus:border-gray-9 text-gray-9 min-h-11 w-20 rounded-md border bg-white px-3 text-sm focus:outline-none focus-visible:ring-2 focus-visible:ring-gray-9/30"
          />
        </label>
        <label class="flex flex-col gap-1 text-sm">
          <span class="text-subtle-1">&#952;</span>
          <input
            type="number"
            step="any"
            bind:value={theta}
            aria-label="Orientation angle theta (degrees)"
            class="border-medium hover:border-gray-6 focus:border-gray-9 text-gray-9 min-h-11 w-20 rounded-md border bg-white px-3 text-sm focus:outline-none focus-visible:ring-2 focus-visible:ring-gray-9/30"
          />
        </label>
      </div>
      <div class="flex gap-2">
        <button
          type="button"
          onclick={handleTeachOrientation}
          disabled={teachOri.isPending}
          class="border-gray-9 bg-gray-9 min-h-11 rounded-md border px-4 text-sm font-medium text-white hover:border-black hover:bg-black active:bg-black focus:outline-none focus-visible:ring-2 focus-visible:ring-gray-9/40 focus-visible:ring-offset-2 disabled:cursor-not-allowed disabled:!border-disabled-light disabled:!bg-disabled-light disabled:text-disabled-dark"
        >
          {teachOri.isPending ? 'Saving…' : 'Save orientation'}
        </button>
        <button
          type="button"
          onclick={() => wizard.finish()}
          class="border-medium text-gray-9 hover:border-gray-6 min-h-11 rounded-md border bg-white px-4 text-sm font-medium focus:outline-none focus-visible:ring-2 focus-visible:ring-gray-9/30"
        >
          Skip
        </button>
      </div>
    {:else if wizard.phase === 'done'}
      <p class="text-gray-9 text-sm">TCP calibrated.</p>
      <button
        type="button"
        onclick={handleTeachAgain}
        class="border-gray-9 bg-gray-9 min-h-11 rounded-md border px-4 text-sm font-medium text-white hover:border-black hover:bg-black active:bg-black focus:outline-none focus-visible:ring-2 focus-visible:ring-gray-9/40 focus-visible:ring-offset-2"
      >
        Teach again
      </button>
    {/if}

    {#if wizard.error}
      <p class="text-danger-dark text-xs" role="alert">{wizard.error}</p>
    {/if}
  </div>
</FloatingPanel>
