<script lang="ts">
  import { ViamProvider } from '@viamrobotics/svelte-sdk'
  import { Visualizer } from '@viamrobotics/motion-tools'
  import { currentMachine, provideMachineId, type MachineIdentity } from './lib/machine'
  import { selectedResource } from './lib/resource.svelte'
  import ResourcePicker from './panels/ResourcePicker.svelte'

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
  <main class="error-screen"><p class="error">Unable to connect to machine: {error}</p></main>
{:else if machine}
  <ViamProvider dialConfigs={{ [machine.id]: machine.dialConf }}>
    {#if selectedResource.name}
      <div class="viz-host">
        <Visualizer partID={machine.id}>
          {#snippet children()}
            <!-- scene plugins + panels mount here in later tasks -->
          {/snippet}
        </Visualizer>
      </div>
    {:else}
      <ResourcePicker />
    {/if}
  </ViamProvider>
{/if}

<style>
  :global(html, body) { margin: 0; height: 100%; }
  :global(*) { box-sizing: border-box; }
  .viz-host { position: fixed; inset: 0; }
  .error-screen { padding: 1rem; }
  .error { color: #ff8080; padding: 1rem; }
</style>
