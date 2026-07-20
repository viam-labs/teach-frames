<script lang="ts">
  // Floating panel that walks an operator through the 3-point frame-define
  // procedure (P0 origin, P1 +X, P2 +XY-plane), driving the real pose-tracker
  // DoCommands. Mounted inside <Visualizer>'s `children` snippet (see
  // App.svelte), alongside <TcpTriad />, which is what makes FloatingPanel's
  // overlay portal available.
  //
  // Step machine itself lives in `wizard` (created once in App.svelte and
  // passed down as a prop) so the scene plugin (Task 5) can read/write the
  // SAME instance and draw the matching spatial story.
  import { createResourceMutation } from '@viamrobotics/svelte-sdk'
  import { FloatingPanel } from '@viamrobotics/motion-tools'
  import { poseTrackerClient } from '../lib/clients'
  import { selectedResource } from '../lib/resource.svelte'
  import { useMachineId } from '../lib/machine'
  import { armState } from '../lib/armState.svelte'
  import { bumpCaptureBuffer } from '../lib/captureBuffer.svelte'
  import {
    capturePoint,
    clearBuffer,
    defineFrame,
    toCommandArgs,
    type DefineFrameResponse,
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

  const CAPTURE_PROMPTS: Record<number, string> = {
    1: "Jog the tool to the frame's ORIGIN, then Capture.",
    2: 'Jog along the intended +X axis, then Capture.',
    3: 'Jog to a point in the +Y half-plane, then Capture.',
  }

  function errorMessage(err: unknown): string {
    return err instanceof Error ? err.message : String(err)
  }

  // Clears any stale server-side capture buffer so exactly our 3 captures
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
    wizard.start(nameInput)
  }

  async function handleCapture() {
    if (!armState.pose) return
    try {
      await capture.mutateAsync(toCommandArgs(capturePoint()))
      // Snapshot the live TCP pose the operator just captured — the same
      // pose the backend just appended to its buffer. Re-read (rather than
      // reuse the pre-await value) so we reflect the freshest poll, and
      // re-guard for null since TS can't narrow a reactive property across
      // an await.
      const pose = armState.pose
      if (!pose) return
      wizard.recordCapture({ ...pose })
    } catch (err) {
      wizard.setError(errorMessage(err))
    }
  }

  async function handleCommit() {
    try {
      const res = (await define.mutateAsync(
        toCommandArgs(defineFrame(wizard.name, '3point')),
      )) as unknown as DefineFrameResponse
      if (res?.committed) {
        // define_frame clears the server-side capture buffer on success;
        // let CapturePanel (if mounted) know so it doesn't show stale points.
        bumpCaptureBuffer()
        wizard.committed()
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

<FloatingPanel
  title="Define Frame"
  bind:isOpen={open}
  defaultPosition={{ x: 24, y: 88 }}
  defaultSize={{ width: 320, height: 360 }}
  resizable
>
  <div class="flex h-full flex-col gap-3 overflow-y-auto p-4">
    {#if wizard.step === 0}
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
        <p class="text-subtle-2 text-xs">Method: 3-point</p>
        <button
          type="submit"
          disabled={nameInput.trim() === ''}
          class="border-gray-9 bg-gray-9 min-h-11 rounded-md border px-4 text-sm font-medium text-white hover:border-black hover:bg-black active:bg-black focus:outline-none focus-visible:ring-2 focus-visible:ring-gray-9/40 focus-visible:ring-offset-2 disabled:cursor-not-allowed disabled:!border-disabled-light disabled:!bg-disabled-light disabled:text-disabled-dark"
        >
          Start
        </button>
      </form>
    {:else if wizard.step >= 1 && wizard.step <= 3}
      <p class="text-gray-9 text-sm">{CAPTURE_PROMPTS[wizard.step]}</p>
      <p class="text-subtle-1 text-xs">{wizard.captures.length} / 3 captured</p>
      <button
        type="button"
        onclick={handleCapture}
        disabled={capture.isPending || !armState.hasArm}
        class="border-gray-9 bg-gray-9 min-h-11 rounded-md border px-4 text-sm font-medium text-white hover:border-black hover:bg-black active:bg-black focus:outline-none focus-visible:ring-2 focus-visible:ring-gray-9/40 focus-visible:ring-offset-2 disabled:cursor-not-allowed disabled:!border-disabled-light disabled:!bg-disabled-light disabled:text-disabled-dark"
      >
        {capture.isPending ? 'Capturing…' : 'Capture'}
      </button>
    {:else if wizard.step === 4}
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
    {:else if wizard.step === 5}
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
      <p class="text-danger-dark text-xs">{wizard.error}</p>
    {/if}
  </div>
</FloatingPanel>
