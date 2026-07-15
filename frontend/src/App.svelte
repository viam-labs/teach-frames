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
  import HandEyePanel from './panels/HandEyePanel.svelte'
  import EyeInHandPanel from './panels/EyeInHandPanel.svelte'

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
        <h1 class="sr-only">Teach pendant</h1>
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
            <section class="panel">
              <HandEyePanel />
            </section>
            <section class="panel">
              <EyeInHandPanel />
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
    background: var(--surface-base);
    color: var(--ink);
    font-family:
      system-ui,
      -apple-system,
      'Segoe UI',
      sans-serif;
  }

  :global(*) {
    box-sizing: border-box;
  }

  /* Motion layer — one place so every control gets the same responsive feel.
     State-conveying only: hover, current selection, press, enable/disable.
     Transitions cover color/border/shadow/opacity/transform; no layout props. */
  :global(button),
  :global(select),
  :global(input),
  :global(.dot) {
    transition:
      background-color var(--transition-fast) var(--ease-out),
      border-color var(--transition-fast) var(--ease-out),
      color var(--transition-fast) var(--ease-out),
      box-shadow var(--transition-fast) var(--ease-out),
      opacity var(--transition-fast) var(--ease-out),
      filter var(--transition-fast) var(--ease-out),
      transform var(--transition-fast) var(--ease-out);
  }

  /* Uniform hover: brighten whatever the control's own color is, so accent and
     neutral controls both acknowledge the pointer without per-control rules. */
  :global(button:hover:not(:disabled)),
  :global(select:hover),
  :global(input:hover:not(:disabled)) {
    /* Gentle lift; kept at 1.08 so it can't pull white-on-accent text below AA. */
    filter: brightness(1.08);
  }

  /* Tactile press — a small nudge on any button. */
  :global(button:active:not(:disabled)) {
    transform: translateY(1px);
  }

  /* Keyboard focus: a clear, high-contrast ring on every focusable control.
     :focus-visible keeps it keyboard-only, so pointer users don't see rings on
     click. WCAG 2.4.7. */
  :global(:focus-visible) {
    outline: 2px solid var(--focus-ring);
    outline-offset: 2px;
  }

  /* Accessibility: honor Reduce Motion. Feedback still changes state, just
     instantly — no animated transitions. */
  @media (prefers-reduced-motion: reduce) {
    :global(*) {
      animation-duration: 0.01ms !important;
      animation-iteration-count: 1 !important;
      transition-duration: 0.01ms !important;
      scroll-behavior: auto !important;
    }
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
    color: var(--ink-muted);
  }

  :global(.panel-actions) {
    display: flex;
    gap: 0.5rem;
    flex-wrap: wrap;
  }

  /* Empty states ("No X yet") — muted, quiet. */
  :global(.panel-empty) {
    margin: 0;
    color: var(--ink-muted);
    font-style: italic;
    font-size: 0.9rem;
  }

  /* Warnings (disabled/unavailable, e.g. no arm configured) — amber. */
  :global(.panel-warning) {
    margin: 0;
    color: var(--warning);
    font-size: 0.9rem;
  }

  /* Data tables can carry more columns than a stacked, full-width panel has
     room for on a phone. Scroll them inside their own box so the page body
     never scrolls sideways. */
  :global(.table-scroll) {
    overflow-x: auto;
  }

  /* Visually hidden, still announced by screen readers. For labels a sighted
     user gets from context but assistive tech needs spelled out. */
  :global(.sr-only) {
    position: absolute;
    width: 1px;
    height: 1px;
    padding: 0;
    margin: -1px;
    overflow: hidden;
    clip: rect(0 0 0 0);
    clip-path: inset(50%);
    white-space: nowrap;
    border: 0;
  }

  .error-screen {
    padding: 1rem;
  }

  .error {
    color: var(--error-text);
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
    background: var(--surface-panel);
    border: 1px solid var(--border-panel);
    border-radius: var(--radius-xl);
    min-width: 0;
  }

  .jog-region {
    min-width: 0;
  }

  /* The Jog panel is the primary surface — its title outranks the secondary
     panels' 1.15rem headings so it reads as primary even when the grid
     collapses to one column and the width cue disappears. */
  .jog-region :global(.panel-header h2) {
    font-size: 1.4rem;
  }

  @media (max-width: 900px) {
    .layout {
      grid-template-columns: 1fr;
    }
  }
</style>
