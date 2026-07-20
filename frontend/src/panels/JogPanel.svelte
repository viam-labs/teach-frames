<script lang="ts">
  import type { Action } from 'svelte/action'
  import { createResourceMutation, createResourceQuery, usePolling } from '@viamrobotics/svelte-sdk'
  import { DashboardPortal, FloatingPanel } from '@viamrobotics/motion-tools'
  import { poseTrackerClient } from '../lib/clients'
  import { selectedResource } from '../lib/resource.svelte'
  import { useMachineId } from '../lib/machine'
  import { armState } from '../lib/armState.svelte'
  import { motion, withMove } from '../lib/motion.svelte'
  import {
    getArmState,
    jogCartesian,
    jogJoint,
    parseArmState,
    toCommandArgs,
    type CartesianAxis,
    type PoseMap,
  } from '../lib/poseTracker'

  const machineId = useMachineId()
  const pt = poseTrackerClient(machineId, () => selectedResource.name ?? '')
  const jog = createResourceMutation(pt, 'doCommand')

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

  type Mode = 'cartesian' | 'joint'
  let mode = $state<Mode>('cartesian')

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
</script>

<DashboardPortal>
  <button
    type="button"
    onclick={() => (open = !open)}
    aria-pressed={open}
    class={`${toggleBase} ${open ? toggleActive : toggleInactive}`}
  >
    Jog
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
    <div class="flex items-start justify-between gap-2">
      <p class="text-subtle-1 text-xs">Move the arm to position it for capture.</p>
      <div class="flex shrink-0 gap-2" role="group" aria-label="Jog mode">
        <button type="button" class={`${toggleBase} ${mode === 'cartesian' ? toggleActive : toggleInactive}`} onclick={() => (mode = 'cartesian')}>
          Cartesian
        </button>
        <button type="button" class={`${toggleBase} ${mode === 'joint' ? toggleActive : toggleInactive}`} onclick={() => (mode = 'joint')}>
          Joint
        </button>
      </div>
    </div>

    <div class="grid grid-cols-1 items-start gap-5 sm:grid-cols-2">
      <div class="flex min-w-0 flex-col gap-4">
        {#if mode === 'cartesian'}
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
                <button type="button" {disabled} class={jogKeyClass} use:jogHold={() => jogCartesian(axis, -transStep)} aria-label="{axis} minus">−</button>
                <button type="button" {disabled} class={jogKeyClass} use:jogHold={() => jogCartesian(axis, transStep)} aria-label="{axis} plus">+</button>
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
                <button type="button" {disabled} class={jogKeyClass} use:jogHold={() => jogCartesian(axis, -rotStep)} aria-label="{axis} minus">−</button>
                <button type="button" {disabled} class={jogKeyClass} use:jogHold={() => jogCartesian(axis, rotStep)} aria-label="{axis} plus">+</button>
              </div>
            {/each}
          </div>
        {:else}
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
