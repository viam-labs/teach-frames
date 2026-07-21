<script lang="ts">
  // Floating panel that walks an operator through defining a frame by one of
  // three methods (3-point: P0 origin, P1 +X, P2 +XY-plane; single-point:
  // origin only; TCP-snapshot: uses the current tool pose as-is), driving the
  // real pose-tracker DoCommands. Mounted inside <Visualizer>'s `children`
  // snippet (see App.svelte), alongside <TcpTriad />, which is what makes
  // FloatingPanel's overlay portal available.
  //
  // Step machine itself lives in `wizard` (created once in App.svelte and
  // passed down as a prop) so the scene plugin can read/write the SAME
  // instance and draw the matching spatial story.
  import { createResourceMutation, createResourceQuery } from '@viamrobotics/svelte-sdk'
  import { DashboardPortal, FloatingPanel } from '@viamrobotics/motion-tools'
  import { Icon } from '@viamrobotics/prime-core'
  import DashboardToggle from './DashboardToggle.svelte'
  import { poseTrackerClient } from '../lib/clients'
  import { selectedResource } from '../lib/resource.svelte'
  import { useMachineId } from '../lib/machine'
  import { armState } from '../lib/armState.svelte'
  import { motion } from '../lib/motion.svelte'
  import { bumpCaptureBuffer } from '../lib/captureBuffer.svelte'
  import { bumpFrameRevision } from '../lib/frameRevision.svelte'
  import {
    capturePoint,
    clearBuffer,
    defineFrame,
    listFrames,
    toCommandArgs,
    type CaptureResponse,
    type DefineFrameResponse,
    type FrameMethod,
    type FramesResponse,
  } from '../lib/poseTracker'
  import type { createFrameDefineWizard } from '../lib/wizard/frameDefine.svelte'

  let { wizard }: { wizard: ReturnType<typeof createFrameDefineWizard> } = $props()

  const machineId = useMachineId()
  const pt = poseTrackerClient(machineId, () => selectedResource.name ?? '')
  const capture = createResourceMutation(pt, 'doCommand')
  const clear = createResourceMutation(pt, 'doCommand')
  const define = createResourceMutation(pt, 'doCommand')

  // Panel visibility is local UI state, separate from the wizard's step
  // machine — closing/reopening the panel shouldn't reset progress.
  let open = $state(true)
  let nameInput = $state('')
  let methodInput = $state<FrameMethod>('3point')

  const METHOD_LABELS: Record<FrameMethod, string> = {
    '3point': '3-point',
    point: 'Single point',
    tcp_snapshot: 'TCP snapshot',
  }
  // Wording here is intentionally teach-flow-specific (describes what each
  // method's point(s) mean during capture) and deliberately differs from the
  // count-based hints in the sibling FramePanel.svelte — don't "reconcile"
  // them, that would degrade the guided copy here.
  const METHOD_HINTS: Record<FrameMethod, string> = {
    '3point': 'origin, +X, and a point in the +XY plane',
    point: 'one point — origin only, no rotation',
    tcp_snapshot: 'one point — uses the tool pose as-is',
  }
  const METHODS: FrameMethod[] = ['3point', 'point', 'tcp_snapshot']

  // Defining with an existing name overwrites it — surface that before Commit.
  // 4th arg (options) is required — a lone trailing arg is treated as options
  // and would drop our command args (see FramePanel.svelte for the same
  // pattern with more detail).
  const framesQuery = createResourceQuery(
    pt, 'doCommand', () => toCommandArgs(listFrames()), () => ({}),
  )
  const existingNames = $derived(
    Object.keys((framesQuery.data as FramesResponse | undefined)?.frames ?? {}),
  )
  const willReplace = $derived(existingNames.includes(nameInput.trim()))

  const CAPTURE_PROMPTS: Record<FrameMethod, string[]> = {
    '3point': [
      "Jog the tool to the frame's ORIGIN, then Capture.",
      'Jog along the intended +X axis, then Capture.',
      'Jog to a point in the +Y half-plane, then Capture.',
    ],
    point: ["Jog the tool to the frame's origin, then Capture."],
    tcp_snapshot: ['Jog the tool to the exact pose to snapshot, then Capture.'],
  }

  function errorMessage(err: unknown): string {
    return err instanceof Error ? err.message : String(err)
  }

  // Clears any stale server-side capture buffer so exactly our captures
  // define the frame. Best-effort: a failure here shouldn't block starting —
  // it's surfaced (if it matters) once a real capture/define call fails.
  async function clearServerBuffer() {
    try {
      await clear.mutateAsync(toCommandArgs(clearBuffer()))
    } catch {
      // Ignored — see comment above.
    }
  }

  async function handleStart() {
    await clearServerBuffer()
    wizard.start(nameInput, methodInput)
  }

  async function handleCapture() {
    // Re-entrancy guard: the Capture button's `disabled={capture.isPending}`
    // paints asynchronously, so a fast double-click can fire this twice
    // synchronously before the DOM updates. A second concurrent
    // capture_point call would advance the backend buffer without a
    // matching wizard.recordCapture (the phase !== 'capturing' guard drops
    // it), desyncing the buffer the define runs against from the wizard's
    // count.
    if (capture.isPending) return
    try {
      // Source the captured pose from the capture_point RESPONSE, not the
      // armState poll — it's the exact point the backend appended to its
      // buffer, whereas the poll can be up to 500ms stale and could reflect
      // a different pose than what was just captured.
      const res = (await capture.mutateAsync(toCommandArgs(capturePoint()))) as unknown as CaptureResponse
      // Defend against a desynced backend buffer (e.g. a swallowed
      // clear_buffer failure left stale points behind): the response's
      // buffer_len is the backend's authoritative count. If it doesn't match
      // what the wizard is about to have, define_frame would run against a
      // buffer that doesn't match the reviewed markers — bail with an error
      // instead of silently recording a mismatched capture.
      if (res.buffer_len !== wizard.captures.length + 1) {
        wizard.setError(
          `Capture buffer out of sync (server has ${res.buffer_len} points, expected ${wizard.captures.length + 1}). Re-capture to reset.`,
        )
        return
      }
      wizard.recordCapture(res.pose)
    } catch (err) {
      wizard.setError(errorMessage(err))
    }
  }

  async function handleCommit() {
    try {
      const res = (await define.mutateAsync(
        toCommandArgs(defineFrame(wizard.name, wizard.method)),
      )) as unknown as DefineFrameResponse
      if (res?.committed) {
        // define_frame clears the server-side capture buffer on success;
        // let CapturePanel (if mounted) know so it doesn't show stale points.
        bumpCaptureBuffer()
        wizard.committed()
        void framesQuery.refetch()
        // Force the Visualizer to remount and re-snapshot world_state_store —
        // see the {#key frameRevision.value} comment in App.svelte.
        bumpFrameRevision()
      } else {
        wizard.setError('define_frame did not commit')
      }
    } catch (err) {
      // Surfaces the backend's degeneracy error for collinear points.
      wizard.setError(errorMessage(err))
    }
  }

  async function handleRecapture() {
    await clearServerBuffer()
    wizard.recapture()
  }

  function handleDefineAnother() {
    nameInput = ''
    wizard.reset()
  }
</script>

<DashboardPortal>
  <DashboardToggle active={open} label="Define frame" onclick={() => (open = !open)}>
    {#snippet children()}
      <Icon name="image-filter-center-focus" />
    {/snippet}
  </DashboardToggle>
</DashboardPortal>

<FloatingPanel
  title="Define Frame"
  bind:isOpen={open}
  defaultPosition={{ x: 24, y: 88 }}
  defaultSize={{ width: 320, height: 360 }}
  resizable
>
  <div class="flex h-full flex-col gap-3 overflow-y-auto p-4">
    {#if wizard.phase === 'setup'}
      <form
        class="flex flex-col gap-3"
        onsubmit={(event) => {
          event.preventDefault()
          void handleStart()
        }}
      >
        <label class="flex flex-col gap-1 text-sm" for="frame-name">
          <span class="text-subtle-1">Frame name</span>
          <input
            id="frame-name"
            type="text"
            bind:value={nameInput}
            placeholder="e.g. table"
            class="border-medium hover:border-gray-6 focus:border-gray-9 text-gray-9 min-h-11 w-full rounded-md border bg-white px-3 text-sm focus:outline-none focus-visible:ring-2 focus-visible:ring-gray-9/30"
          />
        </label>
        <fieldset class="flex flex-col gap-1.5">
          <legend class="text-subtle-1 text-sm">Method</legend>
          {#each METHODS as m (m)}
            <label class="text-gray-9 flex items-center gap-2 text-sm">
              <input type="radio" name="frame-method" value={m} bind:group={methodInput} />
              <span>{METHOD_LABELS[m]}</span>
              <span class="text-subtle-2 text-xs">— {METHOD_HINTS[m]}</span>
            </label>
          {/each}
        </fieldset>

        {#if willReplace}
          <p class="text-subtle-1 text-xs" role="status">
            A frame named "{nameInput.trim()}" exists — committing will replace it.
          </p>
        {/if}
        <button
          type="submit"
          disabled={nameInput.trim() === ''}
          class="border-gray-9 bg-gray-9 min-h-11 rounded-md border px-4 text-sm font-medium text-white hover:border-black hover:bg-black active:bg-black focus:outline-none focus-visible:ring-2 focus-visible:ring-gray-9/40 focus-visible:ring-offset-2 disabled:cursor-not-allowed disabled:!border-disabled-light disabled:!bg-disabled-light disabled:text-disabled-dark"
        >
          Start
        </button>
      </form>
    {:else if wizard.phase === 'capturing'}
      <div aria-live="polite">
        <p class="text-gray-9 text-sm">{CAPTURE_PROMPTS[wizard.method][wizard.captureIndex]}</p>
        <p class="text-subtle-1 text-xs">{wizard.captures.length} / {wizard.requiredCaptures} captured</p>
      </div>
      <button
        type="button"
        onclick={handleCapture}
        disabled={capture.isPending || !armState.hasArm || motion.busy > 0}
        class="border-gray-9 bg-gray-9 min-h-11 rounded-md border px-4 text-sm font-medium text-white hover:border-black hover:bg-black active:bg-black focus:outline-none focus-visible:ring-2 focus-visible:ring-gray-9/40 focus-visible:ring-offset-2 disabled:cursor-not-allowed disabled:!border-disabled-light disabled:!bg-disabled-light disabled:text-disabled-dark"
      >
        {capture.isPending ? 'Capturing…' : 'Capture'}
      </button>
    {:else if wizard.phase === 'preview'}
      <p class="text-gray-9 text-sm">Review the frame, then commit.</p>
      <div class="flex gap-2">
        <button
          type="button"
          onclick={handleCommit}
          disabled={!wizard.canCommit || define.isPending}
          class="border-gray-9 bg-gray-9 min-h-11 rounded-md border px-4 text-sm font-medium text-white hover:border-black hover:bg-black active:bg-black focus:outline-none focus-visible:ring-2 focus-visible:ring-gray-9/40 focus-visible:ring-offset-2 disabled:cursor-not-allowed disabled:!border-disabled-light disabled:!bg-disabled-light disabled:text-disabled-dark"
        >
          {define.isPending ? 'Committing…' : 'Commit'}
        </button>
        <button
          type="button"
          onclick={handleRecapture}
          disabled={define.isPending}
          class="border-medium text-gray-9 hover:border-gray-6 min-h-11 rounded-md border bg-white px-4 text-sm font-medium focus:outline-none focus-visible:ring-2 focus-visible:ring-gray-9/30 disabled:cursor-not-allowed disabled:text-disabled-dark"
        >
          Re-capture
        </button>
      </div>
    {:else if wizard.phase === 'committed'}
      <p class="text-gray-9 text-sm">Frame <em>{wizard.name}</em> saved.</p>
      <button
        type="button"
        onclick={handleDefineAnother}
        class="border-gray-9 bg-gray-9 min-h-11 rounded-md border px-4 text-sm font-medium text-white hover:border-black hover:bg-black active:bg-black focus:outline-none focus-visible:ring-2 focus-visible:ring-gray-9/40 focus-visible:ring-offset-2"
      >
        Define another
      </button>
    {/if}

    {#if wizard.error}
      <p class="text-danger-dark text-xs" role="alert">{wizard.error}</p>
    {/if}
  </div>
</FloatingPanel>
