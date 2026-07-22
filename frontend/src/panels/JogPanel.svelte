<script lang="ts">
  import type { Action } from 'svelte/action'
  import { createResourceMutation, createResourceQuery, usePolling } from '@viamrobotics/svelte-sdk'
  import { DashboardPortal, FloatingPanel } from '@viamrobotics/motion-tools'
  import { Icon } from '@viamrobotics/prime-core'
  import DashboardToggle from './DashboardToggle.svelte'
  import { poseTrackerClient } from '../lib/clients'
  import { selectedResource } from '../lib/resource.svelte'
  import { useMachineId } from '../lib/machine'
  import { armState } from '../lib/armState.svelte'
  import { motion, withMove, requestStop } from '../lib/motion.svelte'
  import {
    getArmState,
    jogCartesian,
    jogJoint,
    listFrames,
    moveToJoints,
    moveToPose,
    parseArmState,
    stopArm,
    toCommandArgs,
    type CartesianAxis,
    type FramesResponse,
    type PoseMap,
  } from '../lib/poseTracker'

  const machineId = useMachineId()
  const pt = poseTrackerClient(machineId, () => selectedResource.name ?? '')
  const jog = createResourceMutation(pt, 'doCommand')

  // Stop is its own mutation so it stays callable — and immediately in-flight —
  // even while a jog/move is pending. It is never gated by motion.busy. Mirrors
  // StatusBar.svelte's handleStop; rendered into the dashboard toolbar below
  // since the global shell (StatusBar isn't mounted here) has no stop control
  // of its own — this IS that control, always visible regardless of whether
  // the Jog panel itself is open.
  const stop = createResourceMutation(pt, 'doCommand')

  function handleStop() {
    requestStop()
    stop.mutate(toCommandArgs(stopArm()))
  }

  // Live arm-state poll. Lives here (not in the status bar) so the pose/joint
  // readout sits with the jog controls. 4th arg (options) is required — see the
  // note in the other query panels. Paused while a move is in flight and once we
  // know no arm is configured (that query would only ever error).
  const armStateQuery = createResourceQuery(
    pt,
    'doCommand',
    () => toCommandArgs(getArmState()),
    () => ({}),
  )

  const NO_ARM_PHRASE = 'arm dependency not configured'
  const noArmConfigured = $derived(
    armStateQuery.error !== null && armStateQuery.error.message.includes(NO_ARM_PHRASE),
  )
  const unexpectedError = $derived(
    armStateQuery.error !== null && !noArmConfigured ? armStateQuery.error.message : undefined,
  )

  usePolling(
    () => armStateQuery.queryKey,
    () => (noArmConfigured || motion.busy > 0 ? false : 500),
  )

  // Publish the live arm state into the shared store so TcpPanel (and jogging
  // here) can react to whether an arm is present.
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

  function fmt(n: number | undefined): string {
    return n === undefined ? '—' : n.toFixed(2)
  }

  type Mode = 'cartesian' | 'joint' | 'moveto'
  let mode = $state<Mode>('cartesian')

  // Taught frames, for the Cartesian jog Reference picker (World/Tool/<taught
  // frame>) — lets the operator jog relative to a previously-defined frame,
  // not just the robot's world/tool basis.
  const framesQ = createResourceQuery(pt, 'doCommand', () => toCommandArgs(listFrames()), () => ({}))
  const taughtFrames = $derived(Object.keys((framesQ.data as FramesResponse | undefined)?.frames ?? {}))

  let reference = $state('world')

  // Guard against a reference that pointed at a taught frame which has since
  // been deleted (e.g. via ManageFramesPanel) — fall back to World rather
  // than silently jogging in a frame that no longer exists.
  $effect(() => {
    if (reference !== 'world' && reference !== 'tool' && !taughtFrames.includes(reference)) {
      reference = 'world'
    }
  })

  // "Move to" — absolute go-to-pose / go-to-joints with a two-step confirm.
  // The target fields are a FROZEN editable draft: they are only ever
  // populated by loadCurrent(), never overwritten by the live armState poll,
  // so the operator's in-progress edits can't be clobbered mid-edit.
  type MoveKind = 'pose' | 'joints'
  let moveKind = $state<MoveKind>('pose')
  let moveConfirm = $state(false)
  let targetPose = $state<PoseMap>({ x: 0, y: 0, z: 0, o_x: 0, o_y: 0, o_z: 1, theta: 0 })
  let targetJoints = $state<number[]>([])
  const moveM = createResourceMutation(pt, 'doCommand')

  function loadCurrent() {
    if (moveKind === 'pose' && armState.pose) {
      targetPose = { ...armState.pose }
    } else if (moveKind === 'joints') {
      targetJoints = [...armState.joints]
    }
    moveConfirm = false
  }

  // Dropping back to the edit step whenever the reviewed target could have
  // changed (kind switch or any field edit) — so the confirm bar can never
  // reflect a target other than the exact one the operator just reviewed.
  function resetConfirm() {
    moveConfirm = false
  }

  const poseValid = $derived(Object.values(targetPose).every((n) => Number.isFinite(n)))
  const jointsValid = $derived(
    targetJoints.length === armState.joints.length && targetJoints.every((n) => Number.isFinite(n)),
  )
  const canReview = $derived(moveKind === 'pose' ? poseValid : jointsValid)

  async function handleExecute() {
    // Defense in depth: mirrors the Execute button's `disabled` guard so a
    // stale/incorrect target (e.g. an empty joints array) can never reach
    // the arm even if this is invoked some other way.
    if (moveM.isPending || motion.busy > 0 || !canReview) {
      return
    }
    try {
      await withMove(async () => {
        const args = moveKind === 'pose' ? moveToPose(targetPose) : moveToJoints(targetJoints)
        const resp = (await moveM.mutateAsync(toCommandArgs(args))) as Record<string, unknown>
        if (resp && typeof resp === 'object') {
          if (resp.pose) {
            armState.pose = resp.pose as PoseMap
          }
          if (Array.isArray(resp.joints)) {
            armState.joints = resp.joints as number[]
          }
        }
      })
    } catch {
      // Surfaced via moveM.error in the template.
    }
    moveConfirm = false
  }

  const TRANSLATION_STEPS = [0.1, 1, 10] as const
  const ROTATION_STEPS = [1, 5] as const
  const JOINT_STEPS = [1, 5] as const

  let transStep = $state<number>(TRANSLATION_STEPS[1])
  let rotStep = $state<number>(ROTATION_STEPS[0])
  let jointStep = $state<number>(JOINT_STEPS[0])

  const TRANSLATION_AXES: CartesianAxis[] = ['x', 'y', 'z']
  const ROTATION_AXES: CartesianAxis[] = ['roll', 'pitch', 'yaw']

  // Jog buttons are gated only on arm presence. Overlap is prevented by the
  // `holding` guard + single-pointer capture, NOT by disabling on motion.busy —
  // disabling the held button mid-hold would abort the press-and-hold loop.
  const disabled = $derived(!armState.hasArm)

  // Press-and-hold: while a jog button is held, send one jog, await it, and
  // repeat. Awaiting each move paces the loop to the arm (no command stacking).
  // A single `holding` flag guarantees only one loop runs at a time.
  let holding = false

  async function startHold(build: () => Record<string, unknown>) {
    if (holding || !armState.hasArm) {
      return
    }
    holding = true
    // Capture the stop generation; if Stop is pressed mid-hold it bumps and we bail.
    const seq = motion.stopSeq
    await withMove(async () => {
      try {
        while (holding && armState.hasArm && motion.stopSeq === seq) {
          const resp = (await jog.mutateAsync(toCommandArgs(build()))) as Record<string, unknown>
          // Drive the readout live from the jog response (polling is paused
          // while a move is in flight).
          if (resp && typeof resp === 'object') {
            if (resp.pose) {
              armState.pose = resp.pose as PoseMap
            }
            if (Array.isArray(resp.joints)) {
              armState.joints = resp.joints as number[]
            }
          }
        }
      } catch {
        // Surfaced via jog.error in the template; stop the loop.
      }
    })
    holding = false
  }

  function stopHold() {
    holding = false
  }

  // Action: press-and-hold (or hold Enter/Space) to jog repeatedly; release to
  // stop. Pointer capture keeps the release reliable even if the pointer drifts
  // off the button, and a window blur is a safety backstop.
  const jogHold: Action<HTMLButtonElement, () => Record<string, unknown>> = (node, build) => {
    let current = build

    // Reflect the held state on the button itself — the operator should see the
    // key engage while the arm jogs, not just watch numbers move in the readout.
    // Driven by the hold loop's lifetime (via .finally) so it also clears when
    // STOP ends the loop mid-press, not only on pointer/key release.
    function onPointerDown(event: PointerEvent) {
      if (node.disabled) {
        return
      }
      node.setPointerCapture(event.pointerId)
      node.classList.add('holding')
      void startHold(current).finally(() => node.classList.remove('holding'))
    }
    function onKeyDown(event: KeyboardEvent) {
      if (event.key !== 'Enter' && event.key !== ' ') {
        return
      }
      event.preventDefault() // suppress the synthetic click so we don't double-send
      node.classList.add('holding')
      void startHold(current).finally(() => node.classList.remove('holding'))
    }
    function onKeyUp(event: KeyboardEvent) {
      if (event.key === 'Enter' || event.key === ' ') {
        stopHold()
      }
    }

    node.addEventListener('pointerdown', onPointerDown)
    node.addEventListener('pointerup', stopHold)
    node.addEventListener('pointercancel', stopHold)
    node.addEventListener('lostpointercapture', stopHold)
    node.addEventListener('keydown', onKeyDown)
    node.addEventListener('keyup', onKeyUp)
    window.addEventListener('blur', stopHold)

    return {
      update(next: () => Record<string, unknown>) {
        current = next
      },
      destroy() {
        node.removeEventListener('pointerdown', onPointerDown)
        node.removeEventListener('pointerup', stopHold)
        node.removeEventListener('pointercancel', stopHold)
        node.removeEventListener('lostpointercapture', stopHold)
        node.removeEventListener('keydown', onKeyDown)
        node.removeEventListener('keyup', onKeyUp)
        window.removeEventListener('blur', stopHold)
        stopHold()
      },
    }
  }

  // Panel visibility is local UI state, toggled from the DashboardPortal
  // button below — mirrors FrameDefineWizard's `open` pattern. The poll and
  // hold logic above run regardless of this flag (this <script> executes on
  // component mount, not on panel-open), so armState keeps flowing to
  // TcpTriad even while the panel is collapsed.
  let open = $state(true)

  // Prime-chrome button recipes (Tailwind classes only — kept as plain
  // strings so the toggle/step markup below stays a simple ternary, matching
  // the dark-button recipe used throughout FrameDefineWizard/ResourcePicker).
  const toggleBase =
    'min-h-11 min-w-11 rounded-md border px-3 text-sm font-medium focus:outline-none focus-visible:ring-2 focus-visible:ring-gray-9/40 focus-visible:ring-offset-1'
  const toggleActive = 'border-gray-9 bg-gray-9 text-white hover:bg-black'
  const toggleInactive = 'border-medium bg-white text-gray-9 hover:border-gray-6'
  const jogKeyClass =
    'jog-key min-h-11 min-w-11 rounded-lg border border-medium bg-white text-lg font-semibold text-gray-9 hover:border-gray-6 focus:outline-none focus-visible:ring-2 focus-visible:ring-gray-9/40 disabled:cursor-not-allowed disabled:opacity-40'
  const secondaryButtonClass =
    'border-medium text-gray-9 hover:border-gray-6 min-h-11 rounded-md border bg-white px-3 text-sm font-medium focus:outline-none focus-visible:ring-2 focus-visible:ring-gray-9/30 disabled:cursor-not-allowed disabled:text-disabled-dark'
  const primaryButtonClass =
    'min-h-11 rounded-md border border-gray-9 bg-gray-9 px-4 text-sm font-medium text-white hover:bg-black focus:outline-none focus-visible:ring-2 focus-visible:ring-gray-9/40 disabled:cursor-not-allowed disabled:opacity-40'
  const dangerButtonClass =
    'border-danger-dark bg-danger-dark min-h-11 rounded-md border px-4 text-sm font-medium text-white focus:outline-none focus-visible:ring-2 focus-visible:ring-danger-dark/40 disabled:cursor-not-allowed disabled:opacity-60'
  const inputClass =
    'border-medium min-h-11 w-full rounded-md border px-2 text-sm tabular-nums focus:outline-none focus-visible:ring-2 focus-visible:ring-gray-9/40'

  // Lands keyboard focus on the safe (Cancel) button when the Execute
  // confirmation opens — mirrors ManageFramesPanel's focusOnMount.
  function focusOnMount(node: HTMLElement) {
    node.focus()
  }
</script>

<DashboardPortal>
  <DashboardToggle active={open} label="Jog" onclick={() => (open = !open)}>
    {#snippet children()}
      <Icon name="robot-outline" />
    {/snippet}
  </DashboardToggle>
</DashboardPortal>

<!-- Global STOP, rendered into the always-visible dashboard toolbar (not
     inside the FloatingPanel body) so it stays reachable even when the Jog
     panel is collapsed — the shell has no other stop control (StatusBar
     isn't mounted here). Never gated on motion.busy/armState.hasArm; only
     disabled while the stop command itself is in flight. -->
<DashboardPortal>
  <button
    type="button"
    class="bg-danger-dark min-h-9 rounded-md border border-danger-dark px-3 text-xs font-bold tracking-wide text-white uppercase focus:outline-none focus-visible:ring-2 focus-visible:ring-danger-dark/40 disabled:cursor-not-allowed disabled:opacity-60"
    onclick={handleStop}
    disabled={stop.isPending}
    aria-label="Stop arm"
  >
    {stop.isPending ? 'Stopping…' : 'STOP'}
  </button>
</DashboardPortal>

<FloatingPanel
  title="Jog"
  bind:isOpen={open}
  defaultPosition={{ x: 360, y: 24 }}
  defaultSize={{ width: 460, height: 520 }}
  resizable
>
  <div class="flex h-full flex-col gap-3 overflow-y-auto p-4">
    {#if stop.error}
      <p class="text-danger-dark text-xs" role="alert">{stop.error.message}</p>
    {/if}

    <div class="flex items-start justify-between gap-2">
      <p class="text-subtle-1 text-xs">Move the arm to position it for capture.</p>
      <div class="flex shrink-0 gap-2" role="group" aria-label="Jog mode">
        <button type="button" class={`${toggleBase} ${mode === 'cartesian' ? toggleActive : toggleInactive}`} onclick={() => (mode = 'cartesian')}>
          Cartesian
        </button>
        <button type="button" class={`${toggleBase} ${mode === 'joint' ? toggleActive : toggleInactive}`} onclick={() => (mode = 'joint')}>
          Joint
        </button>
        <button type="button" class={`${toggleBase} ${mode === 'moveto' ? toggleActive : toggleInactive}`} onclick={() => (mode = 'moveto')}>
          Move to
        </button>
      </div>
    </div>

    <div class="grid grid-cols-1 items-start gap-5 sm:grid-cols-2">
      <div class="flex min-w-0 flex-col gap-4">
        {#if mode === 'cartesian'}
          <div class="flex flex-col gap-1.5">
            <span class="text-subtle-2 text-xs">Reference</span>
            <div class="flex flex-wrap gap-2" role="group" aria-label="Jog reference frame">
              <button type="button" class={`${toggleBase} ${reference === 'world' ? toggleActive : toggleInactive}`} onclick={() => (reference = 'world')}>
                World
              </button>
              <button type="button" class={`${toggleBase} ${reference === 'tool' ? toggleActive : toggleInactive}`} onclick={() => (reference = 'tool')}>
                Tool
              </button>
              {#each taughtFrames as f (f)}
                <button type="button" class={`${toggleBase} ${reference === f ? toggleActive : toggleInactive}`} onclick={() => (reference = f)}>
                  {f}
                </button>
              {/each}
            </div>
          </div>

          <div class="flex flex-col gap-1.5">
            <span class="text-subtle-2 text-xs">Translation step (mm)</span>
            <div class="flex flex-wrap gap-2" role="group" aria-label="Translation step size">
              {#each TRANSLATION_STEPS as step (step)}
                <button type="button" class={`${toggleBase} ${transStep === step ? toggleActive : toggleInactive}`} onclick={() => (transStep = step)}>
                  {step}
                </button>
              {/each}
            </div>
          </div>

          <div class="flex flex-col gap-2">
            {#each TRANSLATION_AXES as axis (axis)}
              <div class="flex items-center gap-3">
                <span class="text-gray-9 w-16 text-sm font-semibold">{axis.toUpperCase()}</span>
                <button type="button" {disabled} class={jogKeyClass} use:jogHold={() => jogCartesian(axis, -transStep, reference)} aria-label="{axis} minus">−</button>
                <button type="button" {disabled} class={jogKeyClass} use:jogHold={() => jogCartesian(axis, transStep, reference)} aria-label="{axis} plus">+</button>
              </div>
            {/each}
          </div>

          <div class="flex flex-col gap-1.5">
            <span class="text-subtle-2 text-xs">Rotation step (deg)</span>
            <div class="flex flex-wrap gap-2" role="group" aria-label="Rotation step size">
              {#each ROTATION_STEPS as step (step)}
                <button type="button" class={`${toggleBase} ${rotStep === step ? toggleActive : toggleInactive}`} onclick={() => (rotStep = step)}>
                  {step}
                </button>
              {/each}
            </div>
          </div>

          <div class="flex flex-col gap-2">
            {#each ROTATION_AXES as axis (axis)}
              <div class="flex items-center gap-3">
                <span class="text-gray-9 w-16 text-sm font-semibold">{axis[0]?.toUpperCase()}{axis.slice(1)}</span>
                <button type="button" {disabled} class={jogKeyClass} use:jogHold={() => jogCartesian(axis, -rotStep, reference)} aria-label="{axis} minus">−</button>
                <button type="button" {disabled} class={jogKeyClass} use:jogHold={() => jogCartesian(axis, rotStep, reference)} aria-label="{axis} plus">+</button>
              </div>
            {/each}
          </div>
        {:else if mode === 'joint'}
          <div class="flex flex-col gap-1.5">
            <span class="text-subtle-2 text-xs">Joint step (deg)</span>
            <div class="flex flex-wrap gap-2" role="group" aria-label="Joint step size">
              {#each JOINT_STEPS as step (step)}
                <button type="button" class={`${toggleBase} ${jointStep === step ? toggleActive : toggleInactive}`} onclick={() => (jointStep = step)}>
                  {step}
                </button>
              {/each}
            </div>
          </div>

          <div class="flex flex-col gap-2">
            {#each armState.joints as _joint, i (i)}
              <div class="flex items-center gap-3">
                <span class="text-gray-9 w-16 text-sm font-semibold">J{i}</span>
                <button type="button" {disabled} class={jogKeyClass} use:jogHold={() => jogJoint(i, -jointStep)} aria-label="joint {i} minus">−</button>
                <button type="button" {disabled} class={jogKeyClass} use:jogHold={() => jogJoint(i, jointStep)} aria-label="joint {i} plus">+</button>
              </div>
            {/each}
            {#if armState.joints.length === 0}
              <p class="text-subtle-2 text-sm italic">No joint data available.</p>
            {/if}
          </div>
        {:else if !armState.hasArm}
          <p class="text-subtle-2 text-sm italic">Requires a configured arm.</p>
        {:else}
          <div class="flex flex-col gap-3">
            <div class="flex flex-wrap gap-2" role="group" aria-label="Move-to kind">
              <button type="button" class={`${toggleBase} ${moveKind === 'pose' ? toggleActive : toggleInactive}`} onclick={() => { moveKind = 'pose'; resetConfirm() }}>
                Pose
              </button>
              <button type="button" class={`${toggleBase} ${moveKind === 'joints' ? toggleActive : toggleInactive}`} onclick={() => { moveKind = 'joints'; resetConfirm() }}>
                Joints
              </button>
            </div>

            <button type="button" class={secondaryButtonClass} onclick={loadCurrent}>Load current</button>

            {#if moveKind === 'pose'}
              <div class="grid grid-cols-2 gap-2">
                <label class="flex flex-col gap-1 text-xs">
                  <span class="text-subtle-2">X (mm)</span>
                  <input type="number" step="any" class={inputClass} bind:value={targetPose.x} oninput={resetConfirm} />
                </label>
                <label class="flex flex-col gap-1 text-xs">
                  <span class="text-subtle-2">Y (mm)</span>
                  <input type="number" step="any" class={inputClass} bind:value={targetPose.y} oninput={resetConfirm} />
                </label>
                <label class="flex flex-col gap-1 text-xs">
                  <span class="text-subtle-2">Z (mm)</span>
                  <input type="number" step="any" class={inputClass} bind:value={targetPose.z} oninput={resetConfirm} />
                </label>
                <label class="flex flex-col gap-1 text-xs">
                  <span class="text-subtle-2">OX</span>
                  <input type="number" step="any" class={inputClass} bind:value={targetPose.o_x} oninput={resetConfirm} />
                </label>
                <label class="flex flex-col gap-1 text-xs">
                  <span class="text-subtle-2">OY</span>
                  <input type="number" step="any" class={inputClass} bind:value={targetPose.o_y} oninput={resetConfirm} />
                </label>
                <label class="flex flex-col gap-1 text-xs">
                  <span class="text-subtle-2">OZ</span>
                  <input type="number" step="any" class={inputClass} bind:value={targetPose.o_z} oninput={resetConfirm} />
                </label>
                <label class="flex flex-col gap-1 text-xs">
                  <span class="text-subtle-2">&#952; (deg)</span>
                  <input type="number" step="any" class={inputClass} bind:value={targetPose.theta} oninput={resetConfirm} />
                </label>
              </div>
            {:else if targetJoints.length === 0}
              <p class="text-subtle-2 text-sm italic">Load current to populate joint targets.</p>
            {:else}
              <div class="grid grid-cols-2 gap-2">
                {#each targetJoints as _joint, i (i)}
                  <label class="flex flex-col gap-1 text-xs">
                    <span class="text-subtle-2">J{i}</span>
                    <input type="number" step="any" class={inputClass} bind:value={targetJoints[i]} oninput={resetConfirm} />
                  </label>
                {/each}
              </div>
            {/if}

            {#if !moveConfirm}
              <button type="button" class={primaryButtonClass} disabled={!canReview} onclick={() => (moveConfirm = true)}>
                Review target
              </button>
            {:else}
              <div class="border-danger-dark bg-danger-light flex flex-col gap-2 rounded-md border p-3" role="alert">
                {#if moveKind === 'pose'}
                  <p class="text-gray-9 text-sm tabular-nums">
                    X {fmt(targetPose.x)}, Y {fmt(targetPose.y)}, Z {fmt(targetPose.z)}, OX {fmt(targetPose.o_x)}, OY {fmt(targetPose.o_y)}, OZ {fmt(targetPose.o_z)}, &#952; {fmt(targetPose.theta)}
                  </p>
                {:else}
                  <p class="text-gray-9 text-sm tabular-nums">
                    {targetJoints.map((j, i) => `J${i}=${fmt(j)}`).join(', ')}
                  </p>
                {/if}
                <p class="text-gray-9 text-sm">The arm will move to this absolute target.</p>
                <div class="flex gap-2">
                  <button
                    type="button"
                    class={dangerButtonClass}
                    disabled={moveM.isPending || motion.busy > 0 || !canReview}
                    onclick={handleExecute}
                  >
                    {moveM.isPending ? 'Moving…' : 'Execute'}
                  </button>
                  <button type="button" class={secondaryButtonClass} use:focusOnMount onclick={() => (moveConfirm = false)}>
                    Cancel
                  </button>
                </div>
              </div>
            {/if}

            {#if moveM.error}
              <p class="text-danger-dark text-xs" role="alert">{moveM.error.message}</p>
            {/if}
          </div>
        {/if}
      </div>

      <div class="min-w-0">
        {#if armState.hasArm && armState.pose}
          {#if mode === 'cartesian'}
            <table class="border-medium w-full table-fixed border-collapse overflow-hidden rounded-md border text-sm tabular-nums" aria-label="Pose values">
              <thead>
                <tr>
                  <th class="border-medium bg-light text-subtle-2 border-b px-3 py-1.5 text-left text-[11px] font-semibold tracking-wide uppercase">Pose</th>
                  <th class="border-medium bg-light text-subtle-2 border-b px-3 py-1.5 text-right text-[11px] font-semibold tracking-wide uppercase">Value</th>
                </tr>
              </thead>
              <tbody class="[&>tr:last-child>*]:border-b-0">
                <tr><th scope="row" class="text-gray-9 border-light w-16 border-b px-3 py-1 text-left font-semibold">X</th><td class="text-subtle-1 border-light border-b px-3 py-1 text-right">{fmt(armState.pose.x)}<span class="text-subtle-2 ml-1 text-xs">mm</span></td></tr>
                <tr><th scope="row" class="text-gray-9 border-light w-16 border-b px-3 py-1 text-left font-semibold">Y</th><td class="text-subtle-1 border-light border-b px-3 py-1 text-right">{fmt(armState.pose.y)}<span class="text-subtle-2 ml-1 text-xs">mm</span></td></tr>
                <tr><th scope="row" class="text-gray-9 border-light w-16 border-b px-3 py-1 text-left font-semibold">Z</th><td class="text-subtle-1 border-light border-b px-3 py-1 text-right">{fmt(armState.pose.z)}<span class="text-subtle-2 ml-1 text-xs">mm</span></td></tr>
                <tr><th scope="row" class="text-gray-9 border-light w-16 border-b px-3 py-1 text-left font-semibold">OX</th><td class="text-subtle-1 border-light border-b px-3 py-1 text-right">{fmt(armState.pose.o_x)}</td></tr>
                <tr><th scope="row" class="text-gray-9 border-light w-16 border-b px-3 py-1 text-left font-semibold">OY</th><td class="text-subtle-1 border-light border-b px-3 py-1 text-right">{fmt(armState.pose.o_y)}</td></tr>
                <tr><th scope="row" class="text-gray-9 border-light w-16 border-b px-3 py-1 text-left font-semibold">OZ</th><td class="text-subtle-1 border-light border-b px-3 py-1 text-right">{fmt(armState.pose.o_z)}</td></tr>
                <tr><th scope="row" class="text-gray-9 border-light w-16 border-b px-3 py-1 text-left font-semibold">&#952;</th><td class="text-subtle-1 border-light border-b px-3 py-1 text-right">{fmt(armState.pose.theta)}<span class="text-subtle-2 ml-1 text-xs">&deg;</span></td></tr>
              </tbody>
            </table>
          {:else}
            <table class="border-medium w-full table-fixed border-collapse overflow-hidden rounded-md border text-sm tabular-nums" aria-label="Joint positions">
              <thead>
                <tr>
                  <th class="border-medium bg-light text-subtle-2 border-b px-3 py-1.5 text-left text-[11px] font-semibold tracking-wide uppercase">Joint</th>
                  <th class="border-medium bg-light text-subtle-2 border-b px-3 py-1.5 text-right text-[11px] font-semibold tracking-wide uppercase">Position (&deg;)</th>
                </tr>
              </thead>
              <tbody class="[&>tr:last-child>*]:border-b-0">
                {#each armState.joints as joint, i (i)}
                  <tr><th scope="row" class="text-gray-9 border-light w-16 border-b px-3 py-1 text-left font-semibold">{i}</th><td class="text-subtle-1 border-light border-b px-3 py-1 text-right">{fmt(joint)}<span class="text-subtle-2 ml-1 text-xs">&deg;</span></td></tr>
                {:else}
                  <tr><td colspan="2" class="text-subtle-2 px-3 py-2 text-sm italic">No joint data available.</td></tr>
                {/each}
              </tbody>
            </table>
          {/if}
        {:else if unexpectedError}
          <p class="text-danger-dark text-xs" role="alert">{unexpectedError}</p>
        {:else if noArmConfigured}
          <p class="text-warning-dark text-sm">No arm configured — jogging is disabled.</p>
        {:else}
          <p class="text-subtle-2 text-sm italic">Loading arm state…</p>
        {/if}
      </div>
    </div>

    {#if jog.error}
      <p class="text-danger-dark text-xs" role="alert">{jog.error.message}</p>
    {/if}
  </div>
</FloatingPanel>

<style>
  /* Held / pressed jog key: filled solid so an engaged jog is unmistakable
     at a glance. `.holding` is toggled by the jogHold action's classList
     (driven by the hold loop's lifetime, not just pointer/key events);
     `:active` covers the instant of press before the loop reports back.
     Tailwind can't target a JS-toggled class, so this stays scoped CSS —
     colors mirror prime's gray-9/black (the same dark-button recipe used
     for the active toggle/step buttons above) rather than the removed
     --accent/--ink-on-accent tokens. */
  :global(.jog-key.holding),
  :global(.jog-key:active:not(:disabled)) {
    background-color: #282829; /* gray-9 */
    border-color: #282829;
    color: #fff;
    transform: translateY(1px);
  }
</style>
