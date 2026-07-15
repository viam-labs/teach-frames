<script lang="ts">
  import { createResourceMutation, createResourceQuery } from '@viamrobotics/svelte-sdk'
  import { poseTrackerClient } from '../lib/clients'
  import { selectedResource } from '../lib/resource.svelte'
  import { useMachineId } from '../lib/machine'
  import {
    handeyeSnapshot, captureHandeyeTarget, captureHandeyeView, getHandeyeBuffer, getHandeyeMode,
    clearHandeyeBuffer, solveHandeye, displayToNativePixel, toCommandArgs,
    type HandEyeSnapshotResponse, type EyeInHandBufferResponse, type SolveHandEyeResponse,
    type CaptureHandEyeTargetResponse, type HandEyeModeResponse, type PoseMap,
  } from '../lib/poseTracker'

  const machineId = useMachineId()
  const pt = poseTrackerClient(machineId, () => selectedResource.name ?? '')

  const modeQuery = createResourceQuery(pt, 'doCommand', () => toCommandArgs(getHandeyeMode()), () => ({}))
  const buffer = createResourceQuery(pt, 'doCommand', () => toCommandArgs(getHandeyeBuffer()), () => ({}))
  const target = createResourceMutation(pt, 'doCommand')
  const snap = createResourceMutation(pt, 'doCommand')
  const view = createResourceMutation(pt, 'doCommand')
  const clear = createResourceMutation(pt, 'doCommand')
  const solve = createResourceMutation(pt, 'doCommand')

  let currentTarget = $state<{ x: number; y: number; z: number } | undefined>(undefined)
  let snapshot = $state<HandEyeSnapshotResponse | undefined>(undefined)
  let solveResult = $state<SolveHandEyeResponse | undefined>(undefined)
  let noCamera = $state(false)
  let noArm = $state(false)
  let imgEl = $state<HTMLImageElement | undefined>(undefined)

  const mode = $derived((modeQuery.data as HandEyeModeResponse | undefined)?.camera_mount)
  const points = $derived((buffer.data as EyeInHandBufferResponse | undefined)?.points ?? [])

  async function handleTouchTarget() {
    try {
      const resp = (await target.mutateAsync(
        toCommandArgs(captureHandeyeTarget()),
      )) as unknown as CaptureHandEyeTargetResponse
      currentTarget = resp.target
      // A fresh target invalidates any frame still frozen from the last one.
      snapshot = undefined
    } catch {
      // Surfaced via target.error.
    }
  }

  async function handleSnapshot() {
    // Reset both BEFORE the attempt, not just on success: these only suppress
    // the raw error display below, so a sticky one would hide every later,
    // unrelated snapshot error behind a stale "requires a camera" warning.
    // Each attempt re-derives them from its own outcome.
    noCamera = false
    noArm = false
    try {
      const resp = (await snap.mutateAsync(toCommandArgs(handeyeSnapshot()))) as unknown as HandEyeSnapshotResponse
      snapshot = resp
    } catch (err) {
      if (err instanceof Error && /camera dependency not configured/.test(err.message)) noCamera = true
      if (err instanceof Error && /arm dependency not configured/.test(err.message)) noArm = true
    }
  }

  async function handleImageClick(event: MouseEvent) {
    if (!snapshot || !imgEl) return
    const { u, v } = displayToNativePixel(
      event.offsetX, event.offsetY, imgEl.clientWidth, imgEl.clientHeight, snapshot.width, snapshot.height,
    )
    try {
      await view.mutateAsync(toCommandArgs(captureHandeyeView(u, v)))
      await buffer.refetch()
    } catch {
      // Surfaced via view.error.
    } finally {
      // The server takes-and-clears its cached snapshot BEFORE deprojecting
      // (handeye.go), so the frame is spent whether or not the click landed on
      // valid depth. Clicking a no-depth pixel is routine on a depth camera
      // (specular surfaces, edges, out of range), and leaving the frame up
      // would invite the obvious retry on a better pixel -- which the server
      // answers with a contradictory "no snapshot cached". Clear unconditionally
      // so the UI matches the server: one snapshot, one attempt.
      //
      // The only server path that returns before that clear is "no target set",
      // which is unreachable here (Snapshot is disabled until a target exists),
      // and harmless if it ever were: recovering means touching a target, and
      // handleTouchTarget drops the frame anyway.
      snapshot = undefined
    }
  }

  async function handleClear() {
    try {
      await clear.mutateAsync(toCommandArgs(clearHandeyeBuffer()))
      // clear_handeye_buffer drops the server-side target too, so mirror that here.
      currentTarget = undefined
      snapshot = undefined
      solveResult = undefined
      await buffer.refetch()
    } catch {
      // Surfaced via clear.error.
    }
  }

  async function handleSolve() {
    solveResult = undefined
    try {
      const resp = (await solve.mutateAsync(toCommandArgs(solveHandeye()))) as unknown as SolveHandEyeResponse
      solveResult = resp
      // solve_handeye clears the buffer and target on success.
      currentTarget = undefined
      snapshot = undefined
      await buffer.refetch()
    } catch {
      // Surfaced via solve.error.
    }
  }

  function fmt(n: number): string {
    return n.toFixed(2)
  }

  function fmtFlange(f: PoseMap): string {
    return `${fmt(f.x)}, ${fmt(f.y)}, ${fmt(f.z)} · ${fmt(f.o_x)}, ${fmt(f.o_y)}, ${fmt(f.o_z)}, ${fmt(f.theta)}°`
  }
</script>

{#if mode === 'eye_in_hand'}
  <section class="handeye-panel">
    <header class="panel-header">
      <div class="panel-titles">
        <h2>Camera Calibration</h2>
        <p class="panel-subtitle">
          Eye-in-hand: touch ONE physical point with the TCP, then jog the arm to a new vantage,
          snapshot, and click that SAME point again. Repeat from several vantages, then solve
          camera&nbsp;&rarr;&nbsp;flange.
        </p>
      </div>
    </header>

    <div class="step">
      <h3>1. Touch target</h3>
      <p class="panel-subtitle">Jog the TCP onto a physical point and touch it once. Every view below must click this same point.</p>
      <div class="panel-actions">
        <button type="button" onclick={handleTouchTarget} disabled={target.isPending}>
          {target.isPending ? 'Touching…' : currentTarget ? 'Re-touch target' : 'Touch target'}
        </button>
      </div>
      {#if target.error}
        <p class="error">{target.error.message}</p>
      {/if}
      {#if currentTarget}
        <p class="panel-empty">Target: {fmt(currentTarget.x)}, {fmt(currentTarget.y)}, {fmt(currentTarget.z)}</p>
      {/if}
    </div>

    <div class="step">
      <h3>2. New vantage &rarr; snapshot &rarr; click the SAME point</h3>
      <p class="panel-subtitle">
        Jog the arm so the wrist camera sees the target from a different angle, then snapshot and
        click on the target in the frozen frame — not a new point.
      </p>
      <div class="panel-actions">
        <button type="button" onclick={handleSnapshot} disabled={snap.isPending || !currentTarget}>
          {snap.isPending ? 'Snapshotting…' : 'Snapshot'}
        </button>
      </div>
      {#if !currentTarget}
        <p class="panel-empty">Touch a target first.</p>
      {/if}
      {#if noCamera}
        <p class="panel-warning">Requires a configured RGBD camera.</p>
      {/if}
      {#if noArm}
        <p class="panel-warning">Requires a configured arm.</p>
      {/if}
      {#if snap.error && !noCamera && !noArm}
        <p class="error">{snap.error.message}</p>
      {/if}
      {#if view.error}
        <p class="error">{view.error.message}</p>
      {/if}

      {#if snapshot}
        <div class="view">
          <!-- svelte-ignore a11y_click_events_have_key_events, a11y_no_noninteractive_element_interactions -->
          <img
            bind:this={imgEl}
            src={`data:image/jpeg;base64,${snapshot.image}`}
            alt="Camera view from this vantage — click the SAME target point"
            onclick={handleImageClick}
          />
        </div>
      {:else if currentTarget && !noCamera && !noArm}
        <p class="panel-empty">Snapshot this vantage, then click the target point in the image.</p>
      {/if}
    </div>

    {#if buffer.error}
      <p class="error">{buffer.error.message}</p>
    {/if}

    <div class="handeye-actions">
      <button type="button" class="secondary" onclick={handleClear} disabled={clear.isPending || (points.length === 0 && !currentTarget)}>
        Clear buffer
      </button>
      <button type="button" onclick={handleSolve} disabled={solve.isPending || points.length < 3}>
        {solve.isPending ? 'Solving…' : 'Solve calibration'}
      </button>
    </div>
    {#if clear.error}
      <p class="error">{clear.error.message}</p>
    {/if}

    {#if solve.error}
      <p class="error">{solve.error.message}</p>
    {/if}
    {#if solveResult}
      <p class="success">Committed to {solveResult.parent} · residual RMS {fmt(solveResult.residual_rms)} mm</p>
      <p class="panel-warning">
        A low residual does not by itself prove the calibration is good — it stays small even when
        the arm was never jogged between vantages. Vary the wrist orientation across vantages, not
        just position.
      </p>
    {/if}

    {#if points.length > 0}
      <div class="table-scroll">
        <table>
          <thead>
            <tr><th>#</th><th>Target XYZ</th><th>Flange (pos · orient)</th><th>Camera XYZ</th></tr>
          </thead>
          <tbody>
            {#each points as p, i (i)}
              <tr>
                <td>{i}</td>
                <td>{fmt(p.target.x)}, {fmt(p.target.y)}, {fmt(p.target.z)}</td>
                <td>{fmtFlange(p.flange)}</td>
                <td>{fmt(p.camera.x)}, {fmt(p.camera.y)}, {fmt(p.camera.z)}</td>
              </tr>
            {/each}
          </tbody>
        </table>
      </div>
    {/if}
  </section>
{:else if modeQuery.error}
  <!-- Without this branch a failed get_handeye_mode leaves `mode` undefined, so
       both calibration panels gate themselves off and the operator gets no UI
       and no reason why. Reported here only: HandEyePanel runs the same query,
       so surfacing it in both would print the identical error twice. -->
  <section class="handeye-panel">
    <header class="panel-header">
      <div class="panel-titles">
        <h2>Camera Calibration</h2>
      </div>
    </header>
    <p class="error">Could not determine the camera mount, so calibration is unavailable: {modeQuery.error.message}</p>
  </section>
{/if}

<style>
  .handeye-panel { display: flex; flex-direction: column; gap: 1rem; padding: 1rem; }
  .step { display: flex; flex-direction: column; gap: 0.4rem; padding-top: 0.5rem; border-top: 1px solid var(--border-panel); }
  .step h3 { margin: 0; font-size: 0.95rem; }
  .view { position: relative; display: inline-block; max-width: 100%; }
  .view img { display: block; max-width: 100%; height: auto; object-fit: fill; cursor: crosshair; border-radius: var(--radius-md); }
  .handeye-actions { display: flex; gap: 0.5rem; flex-wrap: wrap; }
  button { min-height: 44px; padding: 0.5rem 1.1rem; border-radius: var(--radius-md); border: 1px solid var(--border-control); background: var(--accent); color: var(--ink-on-accent); cursor: pointer; }
  button.secondary { background: var(--surface-control); color: inherit; }
  button:disabled { opacity: 0.5; cursor: not-allowed; }
  table { border-collapse: collapse; width: 100%; font-variant-numeric: tabular-nums; font-size: 0.9rem; }
  th, td { padding: 0.35rem 0.6rem; text-align: right; border-bottom: 1px solid var(--border-panel); }
  th:first-child, td:first-child { text-align: left; }
  .success { color: var(--success); font-size: 0.9rem; }
  .error { color: var(--error-text); font-size: 0.85rem; }
</style>
