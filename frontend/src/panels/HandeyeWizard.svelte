<script lang="ts">
  // Mode-adaptive hand-eye calibration wizard, mounted in <Visualizer>'s
  // children (see App.svelte). One adaptive panel covering both eye-to-hand and
  // eye-in-hand calibration, driven by `get_handeye_mode`, matching the prime
  // chrome of FrameDefineWizard /
  // TcpTeachWizard. Store lives in `wizard` (created in App.svelte) so panel
  // state survives the Bug-1 frameRevision remount — this panel intentionally
  // does NOT import/bump frameRevision: the camera triad updates via
  // motion-tools' own useFrames config-revision refetch.
  import { createResourceMutation, createResourceQuery } from '@viamrobotics/svelte-sdk'
  import { DashboardPortal, FloatingPanel } from '@viamrobotics/motion-tools'
  import { Icon } from '@viamrobotics/prime-core'
  import DashboardToggle from './DashboardToggle.svelte'
  import { poseTrackerClient } from '../lib/clients'
  import { selectedResource } from '../lib/resource.svelte'
  import { useMachineId } from '../lib/machine'
  import {
    getHandeyeMode, handeyeSnapshot, captureHandeyePoint, captureHandeyeTarget, captureHandeyeView,
    clearHandeyeBuffer, solveHandeye, displayToNativePixel, toCommandArgs,
    type HandEyeModeResponse, type HandEyeSnapshotResponse, type CaptureHandEyeResponse,
    type CaptureHandEyeViewResponse, type SolveHandEyeResponse,
  } from '../lib/poseTracker'
  import type { createHandeyeWizard } from '../lib/wizard/handeyeCalib.svelte'

  let { wizard }: { wizard: ReturnType<typeof createHandeyeWizard> } = $props()

  const machineId = useMachineId()
  const pt = poseTrackerClient(machineId, () => selectedResource.name ?? '')

  // 4th options arg required (see FrameDefineWizard/ManageFramesPanel).
  const modeQ = createResourceQuery(pt, 'doCommand', () => toCommandArgs(getHandeyeMode()), () => ({}))
  const snap = createResourceMutation(pt, 'doCommand')
  const capturePt = createResourceMutation(pt, 'doCommand')
  const captureTgt = createResourceMutation(pt, 'doCommand')
  const captureView = createResourceMutation(pt, 'doCommand')
  const clear = createResourceMutation(pt, 'doCommand')
  const solveM = createResourceMutation(pt, 'doCommand')

  const modeResp = $derived(modeQ.data as HandEyeModeResponse | undefined)
  const mode = $derived(modeResp?.camera_mount)
  const cameraOk = $derived(modeResp?.camera_configured ?? false)
  const armOk = $derived(modeResp?.arm_configured ?? false)

  let open = $state(false)
  let snapshot = $state<HandEyeSnapshotResponse | undefined>(undefined)
  let dots = $state<{ x: number; y: number }[]>([])
  let imgEl = $state<HTMLImageElement | undefined>(undefined)

  function errorMessage(err: unknown): string {
    return err instanceof Error ? err.message : String(err)
  }

  function handleStart() {
    if (!mode) return
    wizard.start(mode)
    snapshot = undefined
    dots = []
  }

  async function handleTouchTarget() {
    try {
      await captureTgt.mutateAsync(toCommandArgs(captureHandeyeTarget()))
      wizard.targetTouched()
      // A fresh target invalidates any frame still frozen from the last one.
      snapshot = undefined
    } catch (err) {
      wizard.setError(errorMessage(err))
    }
  }

  async function handleSnapshot() {
    if (snap.isPending) return
    try {
      const res = (await snap.mutateAsync(toCommandArgs(handeyeSnapshot()))) as unknown as HandEyeSnapshotResponse
      snapshot = res
    } catch (err) {
      wizard.setError(errorMessage(err))
    }
  }

  async function handleImageClick(event: MouseEvent) {
    if (!snapshot || !imgEl) return
    const { u, v } = displayToNativePixel(
      event.offsetX, event.offsetY, imgEl.clientWidth, imgEl.clientHeight, snapshot.width, snapshot.height,
    )
    if (wizard.mode === 'eye_to_hand') {
      if (capturePt.isPending) return
      try {
        const res = (await capturePt.mutateAsync(toCommandArgs(captureHandeyePoint(u, v)))) as unknown as CaptureHandEyeResponse
        // Normalized fractions (not raw display px) so dots stay put when the
        // resizable panel changes the <img>'s rendered size.
        dots = [...dots, { x: event.offsetX / imgEl.clientWidth, y: event.offsetY / imgEl.clientHeight }]
        wizard.recordCapture(res.buffer_len)
      } catch (err) {
        wizard.setError(errorMessage(err))
      }
      // Camera is static in eye-to-hand: the snapshot is NOT cleared here, so
      // one snapshot serves many clicks.
    } else if (wizard.mode === 'eye_in_hand') {
      if (captureView.isPending) return
      try {
        const res = (await captureView.mutateAsync(toCommandArgs(captureHandeyeView(u, v)))) as unknown as CaptureHandEyeViewResponse
        wizard.recordCapture(res.buffer_len)
      } catch (err) {
        wizard.setError(errorMessage(err))
      } finally {
        // The server takes-and-clears its cached snapshot BEFORE deprojecting
        // (handeye.go), so the frame is spent whether or not the click landed
        // on valid depth. Clicking a no-depth pixel is routine on a depth
        // camera (specular surfaces, edges, out of range), and leaving the
        // frame up would invite the obvious retry on a better pixel — which
        // the server answers with a contradictory "no snapshot cached". Clear
        // unconditionally so the UI matches the server: one snapshot, one
        // attempt.
        snapshot = undefined
      }
    }
  }

  async function handleClear() {
    try {
      await clear.mutateAsync(toCommandArgs(clearHandeyeBuffer()))
      wizard.clearCaptures()
      dots = []
      snapshot = undefined
    } catch (err) {
      wizard.setError(errorMessage(err))
    }
  }

  async function handleSolve() {
    if (solveM.isPending) return
    try {
      const res = (await solveM.mutateAsync(toCommandArgs(solveHandeye()))) as unknown as SolveHandEyeResponse
      if (res?.committed) {
        wizard.solved({ residual_rms: res.residual_rms, parent: res.parent })
        dots = []
        snapshot = undefined
      } else {
        wizard.setError('solve_handeye did not commit (check platform credentials)')
      }
    } catch (err) {
      wizard.setError(errorMessage(err))
    }
  }

  function handleDone() {
    wizard.finish()
  }

  function handleAgain() {
    wizard.reset()
    snapshot = undefined
    dots = []
  }

  function fmt(n: number): string {
    return n.toFixed(2)
  }
</script>

<DashboardPortal>
  <DashboardToggle active={open} label="Camera calibration" onclick={() => (open = !open)}>
    {#snippet children()}
      <Icon name="camera-outline" />
    {/snippet}
  </DashboardToggle>
</DashboardPortal>

<FloatingPanel
  title="Camera Calibration"
  bind:isOpen={open}
  defaultPosition={{ x: 24, y: 460 }}
  defaultSize={{ width: 420, height: 460 }}
  resizable
>
  <div class="flex h-full flex-col gap-3 overflow-y-auto p-4">
    {#if modeQ.error}
      <p class="text-danger-dark text-xs" role="alert">
        Camera calibration unavailable: {modeQ.error.message}
      </p>
    {:else if wizard.phase === 'setup'}
      <p class="text-gray-9 text-sm">
        {#if mode === 'eye_in_hand'}
          Touch one target, then snapshot and click it from &ge;3 wrist vantages. Solves camera&nbsp;&rarr;&nbsp;flange.
        {:else}
          Touch several fixed points with the TCP and click each in the camera view. Solves camera&nbsp;&rarr;&nbsp;world.
        {/if}
      </p>
      {#if !cameraOk}
        <p class="text-subtle-1 text-xs">Requires a configured RGBD camera.</p>
      {/if}
      {#if mode === 'eye_in_hand' && !armOk}
        <p class="text-subtle-1 text-xs">Requires a configured arm.</p>
      {/if}
      <button
        type="button"
        onclick={handleStart}
        disabled={!mode || !cameraOk || (mode === 'eye_in_hand' && !armOk)}
        class="border-gray-9 bg-gray-9 min-h-11 rounded-md border px-4 text-sm font-medium text-white hover:border-black hover:bg-black active:bg-black focus:outline-none focus-visible:ring-2 focus-visible:ring-gray-9/40 focus-visible:ring-offset-2 disabled:cursor-not-allowed disabled:!border-disabled-light disabled:!bg-disabled-light disabled:text-disabled-dark"
      >
        Start
      </button>
    {:else if wizard.phase === 'capturing'}
      <div aria-live="polite">
        {#if wizard.needsTarget}
          <p class="text-gray-9 text-sm">
            Jog the TCP onto a physical point and touch it once — every view clicks this same point.
          </p>
          <button
            type="button"
            onclick={handleTouchTarget}
            disabled={captureTgt.isPending}
            class="border-gray-9 bg-gray-9 min-h-11 rounded-md border px-4 text-sm font-medium text-white hover:border-black hover:bg-black active:bg-black focus:outline-none focus-visible:ring-2 focus-visible:ring-gray-9/40 focus-visible:ring-offset-2 disabled:cursor-not-allowed disabled:!border-disabled-light disabled:!bg-disabled-light disabled:text-disabled-dark"
          >
            {captureTgt.isPending ? 'Touching…' : 'Touch target'}
          </button>
        {:else}
          <button
            type="button"
            onclick={handleSnapshot}
            disabled={snap.isPending}
            class="border-gray-9 bg-gray-9 min-h-11 rounded-md border px-4 text-sm font-medium text-white hover:border-black hover:bg-black active:bg-black focus:outline-none focus-visible:ring-2 focus-visible:ring-gray-9/40 focus-visible:ring-offset-2 disabled:cursor-not-allowed disabled:!border-disabled-light disabled:!bg-disabled-light disabled:text-disabled-dark"
          >
            {snap.isPending ? 'Snapshotting…' : mode === 'eye_in_hand' ? 'Snapshot' : 'Refresh view'}
          </button>
          {#if snapshot}
            <div class="relative inline-block max-w-full">
              <!-- svelte-ignore a11y_click_events_have_key_events, a11y_no_noninteractive_element_interactions -->
              <img
                bind:this={imgEl}
                src={`data:image/jpeg;base64,${snapshot.image}`}
                alt="Camera view — click the target point"
                onclick={handleImageClick}
                class="block h-auto max-w-full cursor-crosshair rounded-md"
                style="object-fit: fill"
              />
              {#each dots as dot, i (i)}
                <span
                  class="pointer-events-none absolute h-2.5 w-2.5 -translate-x-1/2 -translate-y-1/2 rounded-full bg-gray-9 ring-2 ring-white"
                  style={`left:${dot.x * 100}%; top:${dot.y * 100}%`}
                ></span>
              {/each}
            </div>
          {/if}
        {/if}
        <p class="text-subtle-1 text-xs">{wizard.captureCount} / {wizard.minCaptures}+ captured</p>
      </div>
      <div class="flex gap-2">
        <button
          type="button"
          onclick={handleClear}
          disabled={clear.isPending || (wizard.captureCount === 0 && !wizard.hasTarget)}
          class="border-medium text-gray-9 hover:border-gray-6 min-h-11 rounded-md border bg-white px-4 text-sm font-medium focus:outline-none focus-visible:ring-2 focus-visible:ring-gray-9/30 disabled:cursor-not-allowed disabled:text-disabled-dark"
        >
          Clear
        </button>
        <button
          type="button"
          onclick={handleSolve}
          disabled={!wizard.canSolve || solveM.isPending}
          class="border-gray-9 bg-gray-9 min-h-11 rounded-md border px-4 text-sm font-medium text-white hover:border-black hover:bg-black active:bg-black focus:outline-none focus-visible:ring-2 focus-visible:ring-gray-9/40 focus-visible:ring-offset-2 disabled:cursor-not-allowed disabled:!border-disabled-light disabled:!bg-disabled-light disabled:text-disabled-dark"
        >
          {solveM.isPending ? 'Solving…' : 'Solve calibration'}
        </button>
      </div>
    {:else if wizard.phase === 'solved'}
      <p class="text-gray-9 text-sm font-medium">
        Committed to {wizard.solve?.parent} · residual RMS {fmt(wizard.solve?.residual_rms ?? 0)} mm
      </p>
      {#if wizard.mode === 'eye_in_hand'}
        <p class="text-subtle-1 text-xs">
          A low residual doesn't prove the calibration is good — vary the wrist orientation across
          vantages, not just position.
        </p>
      {/if}
      <div class="flex gap-2">
        <button
          type="button"
          onclick={handleClear}
          class="border-medium text-gray-9 hover:border-gray-6 min-h-11 rounded-md border bg-white px-4 text-sm font-medium focus:outline-none focus-visible:ring-2 focus-visible:ring-gray-9/30"
        >
          Re-calibrate
        </button>
        <button
          type="button"
          onclick={handleDone}
          class="border-gray-9 bg-gray-9 min-h-11 rounded-md border px-4 text-sm font-medium text-white hover:border-black hover:bg-black active:bg-black focus:outline-none focus-visible:ring-2 focus-visible:ring-gray-9/40 focus-visible:ring-offset-2"
        >
          Done
        </button>
      </div>
    {:else if wizard.phase === 'done'}
      <p class="text-gray-9 text-sm">Camera calibrated.</p>
      <button
        type="button"
        onclick={handleAgain}
        class="border-gray-9 bg-gray-9 min-h-11 rounded-md border px-4 text-sm font-medium text-white hover:border-black hover:bg-black active:bg-black focus:outline-none focus-visible:ring-2 focus-visible:ring-gray-9/40 focus-visible:ring-offset-2"
      >
        Calibrate again
      </button>
    {/if}

    {#if wizard.error}
      <p class="text-danger-dark text-xs" role="alert">{wizard.error}</p>
    {/if}
  </div>
</FloatingPanel>
