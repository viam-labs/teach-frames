<script lang="ts">
  import { ViamProvider } from '@viamrobotics/svelte-sdk'
  import { currentMachine, provideMachineId, type MachineIdentity } from './lib/machine'
  import { selectedResource } from './lib/resource.svelte'
  import ResourcePicker from './panels/ResourcePicker.svelte'
  import StatusBar from './panels/StatusBar.svelte'
  import JogPanel from './panels/JogPanel.svelte'
  import CapturePanel from './panels/CapturePanel.svelte'
  import FramePanel from './panels/FramePanel.svelte'
  import TcpPanel from './panels/TcpPanel.svelte'

  let machine: MachineIdentity | undefined
  let error: string | undefined

  try {
    machine = currentMachine()
    provideMachineId(machine.id)
  } catch (err) {
    error = err instanceof Error ? err.message : String(err)
  }
</script>

{#if error}
  <main class="error-screen">
    <p class="error">Unable to connect to machine: {error}</p>
  </main>
{:else if machine}
  <ViamProvider dialConfigs={{ [machine.id]: machine.dialConf }}>
    {#if selectedResource.name}
      <div class="app-shell">
        <StatusBar />
        <main class="layout">
          <section class="jog-region panel">
            <JogPanel />
          </section>
          <div class="side-region">
            <section class="panel">
              <CapturePanel />
            </section>
            <section class="panel">
              <FramePanel />
            </section>
            <section class="panel">
              <TcpPanel />
            </section>
          </div>
        </main>
      </div>
    {:else}
      <ResourcePicker />
    {/if}
  </ViamProvider>
{/if}

<style>
  :global(html, body) {
    margin: 0;
    height: 100%;
    background: #12141a;
    color: #f2f2f2;
    font-family:
      system-ui,
      -apple-system,
      'Segoe UI',
      sans-serif;
  }

  :global(*) {
    box-sizing: border-box;
  }

  /* Shared panel chrome — keeps every panel's header, titles, and status
     messages visually consistent. Individual panels supply their own controls. */
  :global(.panel-header) {
    display: flex;
    align-items: flex-start;
    justify-content: space-between;
    gap: 0.75rem 1rem;
    flex-wrap: wrap;
    margin-bottom: 0.25rem;
  }

  :global(.panel-titles) {
    display: flex;
    flex-direction: column;
    gap: 0.15rem;
    min-width: 0;
  }

  :global(.panel-header h2) {
    margin: 0;
    font-size: 1.15rem;
  }

  :global(.panel-subtitle) {
    margin: 0;
    font-size: 0.82rem;
    color: #9aa0a6;
  }

  :global(.panel-actions) {
    display: flex;
    gap: 0.5rem;
    flex-wrap: wrap;
  }

  /* Empty states ("No X yet") — muted, quiet. */
  :global(.panel-empty) {
    margin: 0;
    color: #9aa0a6;
    font-style: italic;
    font-size: 0.9rem;
  }

  /* Warnings (disabled/unavailable, e.g. no arm configured) — amber. */
  :global(.panel-warning) {
    margin: 0;
    color: #d9a441;
    font-size: 0.9rem;
  }

  .error-screen {
    padding: 1rem;
  }

  .error {
    color: #ff8080;
    padding: 1rem;
  }

  .app-shell {
    display: flex;
    flex-direction: column;
    min-height: 100vh;
  }

  .layout {
    flex: 1;
    display: grid;
    grid-template-columns: minmax(0, 1.4fr) minmax(0, 1fr);
    gap: 1rem;
    padding: 1rem;
    align-items: start;
  }

  .side-region {
    display: flex;
    flex-direction: column;
    gap: 1rem;
    min-width: 0;
  }

  .panel {
    background: var(--panel-bg, #1c1f26);
    border: 1px solid var(--panel-border, #333);
    border-radius: 0.75rem;
    min-width: 0;
  }

  .jog-region {
    min-width: 0;
  }

  @media (max-width: 900px) {
    .layout {
      grid-template-columns: 1fr;
    }
  }
</style>
