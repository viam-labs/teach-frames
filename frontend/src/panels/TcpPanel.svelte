<script lang="ts">
  import { createResourceMutation, createResourceQuery } from '@viamrobotics/svelte-sdk'
  import { poseTrackerClient } from '../lib/clients'
  import { selectedResource } from '../lib/resource.svelte'
  import { useMachineId } from '../lib/machine'
  import { armState } from '../lib/armState.svelte'
  import {
    captureTcpPoint,
    clearTcpBuffer,
    getTcpBuffer,
    teachTcpOrientation,
    teachTcpPosition,
    toCommandArgs,
    type BufferResponse,
    type TeachTcpOrientationResponse,
    type TeachTcpPositionResponse,
  } from '../lib/poseTracker'

  const machineId = useMachineId()
  const pt = poseTrackerClient(machineId, () => selectedResource.name ?? '')

  // 4th arg (options) required — a lone trailing arg is treated as options and
  // would drop our args, sending an empty command. See StatusBar for detail.
  const buffer = createResourceQuery(pt, 'doCommand', () => toCommandArgs(getTcpBuffer()), () => ({}))
  const capture = createResourceMutation(pt, 'doCommand')
  const clear = createResourceMutation(pt, 'doCommand')
  const teachPosition = createResourceMutation(pt, 'doCommand')
  const teachOrientation = createResourceMutation(pt, 'doCommand')

  const points = $derived((buffer.data as BufferResponse | undefined)?.points ?? [])

  let positionResult: TeachTcpPositionResponse | undefined = $state(undefined)
  let orientationResult: TeachTcpOrientationResponse | undefined = $state(undefined)

  let oX = $state(0)
  let oY = $state(0)
  let oZ = $state(1)
  let theta = $state(0)

  const disabled = $derived(!armState.hasArm)

  async function handleCapture() {
    try {
      await capture.mutateAsync(toCommandArgs(captureTcpPoint()))
      await buffer.refetch()
    } catch {
      // Surfaced via capture.error in the template.
    }
  }

  async function handleClear() {
    try {
      await clear.mutateAsync(toCommandArgs(clearTcpBuffer()))
      await buffer.refetch()
    } catch {
      // Surfaced via clear.error in the template.
    }
  }

  async function handleTeachPosition() {
    positionResult = undefined
    try {
      const resp = (await teachPosition.mutateAsync(
        toCommandArgs(teachTcpPosition()),
      )) as unknown as TeachTcpPositionResponse
      positionResult = resp
      await buffer.refetch()
    } catch {
      // Surfaced via teachPosition.error in the template.
    }
  }

  async function handleTeachOrientation() {
    orientationResult = undefined
    try {
      const resp = (await teachOrientation.mutateAsync(
        toCommandArgs(teachTcpOrientation(oX, oY, oZ, theta)),
      )) as unknown as TeachTcpOrientationResponse
      orientationResult = resp
    } catch {
      // Surfaced via teachOrientation.error in the template.
    }
  }

  function fmt(n: number): string {
    return n.toFixed(3)
  }
</script>

<section class="tcp-panel" class:disabled>
  <header>
    <h2>TCP</h2>
  </header>

  {#if disabled}
    <p class="notice">Requires a configured arm.</p>
  {/if}

  <div class="capture-row">
    <button type="button" onclick={handleCapture} disabled={disabled || capture.isPending}>
      {capture.isPending ? 'Capturing…' : 'Capture TCP point'}
    </button>
    <button
      type="button"
      class="secondary"
      onclick={handleClear}
      disabled={disabled || clear.isPending || points.length === 0}
    >
      Clear
    </button>
  </div>

  {#if capture.error}
    <p class="error">{capture.error.message}</p>
  {/if}
  {#if clear.error}
    <p class="error">{clear.error.message}</p>
  {/if}
  {#if buffer.error}
    <p class="error">{buffer.error.message}</p>
  {/if}

  {#if points.length === 0}
    <p class="notice">No TCP points captured yet.</p>
  {:else}
    <table>
      <thead>
        <tr>
          <th>#</th>
          <th>X</th>
          <th>Y</th>
          <th>Z</th>
        </tr>
      </thead>
      <tbody>
        {#each points as point, i (i)}
          <tr>
            <td>{i}</td>
            <td>{fmt(point.x)}</td>
            <td>{fmt(point.y)}</td>
            <td>{fmt(point.z)}</td>
          </tr>
        {/each}
      </tbody>
    </table>
  {/if}

  <div class="teach-block">
    <h3>Teach position (pivot)</h3>
    <button type="button" onclick={handleTeachPosition} disabled={disabled || teachPosition.isPending}>
      {teachPosition.isPending ? 'Solving…' : 'Teach TCP position'}
    </button>
    {#if teachPosition.error}
      <p class="error">{teachPosition.error.message}</p>
    {/if}
    {#if positionResult}
      <p class="success">
        offset ({fmt(positionResult.offset.x)}, {fmt(positionResult.offset.y)}, {fmt(positionResult.offset.z)}) ·
        residual RMS {fmt(positionResult.residual_rms)}
      </p>
    {/if}
  </div>

  <div class="teach-block">
    <h3>Teach orientation</h3>
    <div class="orientation-inputs">
      <label>
        <span>O&#8339;</span>
        <input type="number" step="any" bind:value={oX} disabled={disabled} />
      </label>
      <label>
        <span>O&#8340;</span>
        <input type="number" step="any" bind:value={oY} disabled={disabled} />
      </label>
      <label>
        <span>O&#8342;</span>
        <input type="number" step="any" bind:value={oZ} disabled={disabled} />
      </label>
      <label>
        <span>&#952;</span>
        <input type="number" step="any" bind:value={theta} disabled={disabled} />
      </label>
    </div>
    <button
      type="button"
      onclick={handleTeachOrientation}
      disabled={disabled || teachOrientation.isPending}
    >
      {teachOrientation.isPending ? 'Saving…' : 'Teach TCP orientation'}
    </button>
    {#if teachOrientation.error}
      <p class="error">{teachOrientation.error.message}</p>
    {/if}
    {#if orientationResult}
      <p class="success">
        orientation saved: ({fmt(orientationResult.orientation.o_x)}, {fmt(orientationResult.orientation.o_y)}, {fmt(
          orientationResult.orientation.o_z,
        )}) &#952; {fmt(orientationResult.orientation.theta)}
      </p>
    {/if}
  </div>
</section>

<style>
  .tcp-panel {
    display: flex;
    flex-direction: column;
    gap: 0.9rem;
    padding: 1rem;
  }

  .tcp-panel.disabled {
    opacity: 0.85;
  }

  h2 {
    margin: 0;
    font-size: 1.15rem;
  }

  h3 {
    margin: 0 0 0.4rem;
    font-size: 1rem;
  }

  .capture-row {
    display: flex;
    gap: 0.5rem;
  }

  button {
    min-height: 44px;
    padding: 0.5rem 1.1rem;
    border-radius: 0.4rem;
    border: 1px solid var(--control-border, #444);
    background: var(--control-active-bg, #3d6bff);
    color: #fff;
    cursor: pointer;
  }

  button.secondary {
    background: var(--control-bg, #2a2e37);
    color: inherit;
  }

  button:disabled {
    opacity: 0.5;
    cursor: not-allowed;
  }

  table {
    border-collapse: collapse;
    width: 100%;
    font-variant-numeric: tabular-nums;
    font-size: 0.9rem;
  }

  th,
  td {
    padding: 0.35rem 0.6rem;
    text-align: right;
    border-bottom: 1px solid var(--control-border, #333);
  }

  th:first-child,
  td:first-child {
    text-align: left;
  }

  .teach-block {
    display: flex;
    flex-direction: column;
    gap: 0.5rem;
    padding-top: 0.5rem;
    border-top: 1px solid var(--control-border, #333);
  }

  .orientation-inputs {
    display: flex;
    gap: 0.6rem;
    flex-wrap: wrap;
  }

  .orientation-inputs label {
    display: flex;
    flex-direction: column;
    gap: 0.2rem;
    font-size: 0.85rem;
  }

  .orientation-inputs input {
    width: 5.5rem;
    min-height: 44px;
    padding: 0 0.5rem;
    border-radius: 0.4rem;
    border: 1px solid var(--control-border, #444);
    background: var(--control-bg, #2a2e37);
    color: inherit;
  }

  .notice {
    color: #d9a441;
    font-size: 0.9rem;
  }

  .success {
    color: #3ecf6a;
    font-size: 0.9rem;
  }

  .error {
    color: #ff8080;
    font-size: 0.85rem;
  }
</style>
