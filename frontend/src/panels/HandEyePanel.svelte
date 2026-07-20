<script lang="ts">
  import { createResourceMutation, createResourceQuery } from '@viamrobotics/svelte-sdk'
  import { poseTrackerClient } from '../lib/clients'
  import { selectedResource } from '../lib/resource.svelte'
  import { useMachineId } from '../lib/machine'
  import {
    handeyeSnapshot, captureHandeyePoint, getHandeyeBuffer, clearHandeyeBuffer,
    solveHandeye, getHandeyeMode, displayToNativePixel, toCommandArgs,
    type HandEyeSnapshotResponse, type HandEyeBufferResponse, type SolveHandEyeResponse,
    type HandEyeModeResponse,
  } from '../lib/poseTracker'

  const machineId = useMachineId()
  const pt = poseTrackerClient(machineId, () => selectedResource.name ?? '')

  const modeQuery = createResourceQuery(pt, 'doCommand', () => toCommandArgs(getHandeyeMode()), () => ({}))
  const buffer = createResourceQuery(pt, 'doCommand', () => toCommandArgs(getHandeyeBuffer()), () => ({}))
  const snap = createResourceMutation(pt, 'doCommand')
  const capture = createResourceMutation(pt, 'doCommand')
  const clear = createResourceMutation(pt, 'doCommand')
  const solve = createResourceMutation(pt, 'doCommand')

  let snapshot = $state<HandEyeSnapshotResponse | undefined>(undefined)
  let dots = $state<{ x: number; y: number }[]>([])
  let solveResult = $state<SolveHandEyeResponse | undefined>(undefined)
  let noCamera = $state(false)
  let imgEl = $state<HTMLImageElement | undefined>(undefined)

  const points = $derived((buffer.data as HandEyeBufferResponse | undefined)?.points ?? [])
  const mode = $derived((modeQuery.data as HandEyeModeResponse | undefined)?.camera_mount)

  async function handleRefresh() {
    solveResult = undefined
    try {
      const resp = (await snap.mutateAsync(toCommandArgs(handeyeSnapshot()))) as unknown as HandEyeSnapshotResponse
      snapshot = resp
      noCamera = false
    } catch (err) {
      if (err instanceof Error && /camera dependency not configured/.test(err.message)) noCamera = true
    }
  }

  async function handleImageClick(event: MouseEvent) {
    if (!snapshot || !imgEl) return
    const { u, v } = displayToNativePixel(
      event.offsetX, event.offsetY, imgEl.clientWidth, imgEl.clientHeight, snapshot.width, snapshot.height,
    )
    try {
      await capture.mutateAsync(toCommandArgs(captureHandeyePoint(u, v)))
      dots = [...dots, { x: event.offsetX, y: event.offsetY }]
      await buffer.refetch()
    } catch {
      // Surfaced via capture.error.
    }
  }

  async function handleClear() {
    try {
      await clear.mutateAsync(toCommandArgs(clearHandeyeBuffer()))
      dots = []
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
      dots = []
      await buffer.refetch()
    } catch {
      // Surfaced via solve.error.
    }
  }

  function fmt(n: number): string {
    return n.toFixed(2)
  }
</script>

{#if mode === 'eye_to_hand'}
<section class="panel">
<section class="handeye-panel">
  <header class="panel-header">
    <div class="panel-titles">
      <h2>Camera Calibration</h2>
      <p class="panel-subtitle">Eye-to-hand: touch points with the TCP, click them in the camera view, solve camera&nbsp;&rarr;&nbsp;world.</p>
    </div>
    <div class="panel-actions">
      <button type="button" onclick={handleRefresh} disabled={snap.isPending}>
        {snap.isPending ? 'Refreshing…' : 'Refresh view'}
      </button>
    </div>
  </header>

  {#if noCamera}
    <p class="panel-warning">Requires a configured RGBD camera.</p>
  {/if}

  {#if snap.error && !noCamera}
    <p class="error">{snap.error.message}</p>
  {/if}
  {#if capture.error}
    <p class="error">{capture.error.message}</p>
  {/if}
  {#if clear.error}
    <p class="error">{clear.error.message}</p>
  {/if}
  {#if buffer.error}
    <p class="error">{buffer.error.message}</p>
  {/if}

  {#if snapshot}
    <div class="view">
      <!-- svelte-ignore a11y_click_events_have_key_events, a11y_no_noninteractive_element_interactions -->
      <img
        bind:this={imgEl}
        src={`data:image/jpeg;base64,${snapshot.image}`}
        alt="Camera view — click a touched point"
        onclick={handleImageClick}
      />
      {#each dots as dot, i (i)}
        <span class="dot" style={`left:${dot.x}px; top:${dot.y}px`}></span>
      {/each}
    </div>
  {:else if !noCamera}
    <p class="panel-empty">Refresh the view, jog the TCP to touch a point, then click it in the image.</p>
  {/if}

  <div class="handeye-actions">
    <button type="button" class="secondary" onclick={handleClear} disabled={clear.isPending || points.length === 0}>Clear points</button>
    <button type="button" onclick={handleSolve} disabled={solve.isPending || points.length < 3}>
      {solve.isPending ? 'Solving…' : 'Solve calibration'}
    </button>
  </div>

  {#if solve.error}
    <p class="error">{solve.error.message}</p>
  {/if}
  {#if solveResult}
    <p class="success">Committed · residual RMS {fmt(solveResult.residual_rms)} mm</p>
  {/if}

  {#if points.length > 0}
    <div class="table-scroll">
      <table>
        <thead>
          <tr><th>#</th><th>World XYZ</th><th>Camera XYZ</th></tr>
        </thead>
        <tbody>
          {#each points as p, i (i)}
            <tr>
              <td>{i}</td>
              <td>{fmt(p.world.x)}, {fmt(p.world.y)}, {fmt(p.world.z)}</td>
              <td>{fmt(p.camera.x)}, {fmt(p.camera.y)}, {fmt(p.camera.z)}</td>
            </tr>
          {/each}
        </tbody>
      </table>
    </div>
  {/if}
</section>
</section>
{/if}

<style>
  .handeye-panel { display: flex; flex-direction: column; gap: 1rem; padding: 1rem; }
  .view { position: relative; display: inline-block; max-width: 100%; }
  .view img { display: block; max-width: 100%; height: auto; object-fit: fill; cursor: crosshair; border-radius: var(--radius-md); }
  .dot {
    position: absolute; width: 10px; height: 10px; margin: -5px 0 0 -5px;
    border-radius: 50%; background: var(--accent); border: 2px solid var(--ink-on-accent); pointer-events: none;
  }
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
