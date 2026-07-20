<script lang="ts">
  import { ViamProvider } from '@viamrobotics/svelte-sdk'
  import { Visualizer } from '@viamrobotics/motion-tools'
  import { Struct } from '@viamrobotics/sdk'
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

  // The Visualizer's part-config layer has two backends. Without localConfigProps
  // it takes the STANDALONE path, which calls createAppQuery('getRobotPart') and
  // needs a ViamAppProvider/app-key credentials we deliberately don't supply
  // (this Application authenticates with only the machine cookie). That path
  // crashes: "can't access property 'current', viamClient is undefined".
  // Passing localConfigProps routes it to the EMBEDDED (host-owned) backend
  // instead. We hand it a no-op empty config: the live scene doesn't need it
  // (the arm and taught-frame triads come from the robot's frameSystemConfig +
  // world_state_store; part config only tints frame colors and drives build-mode
  // editing, which we don't use), and frame persistence stays on our own
  // pose-tracker define_frame DoCommand.
  const localConfigProps = {
    current: Struct.fromJson({ components: [] }),
    isDirty: false,
    setLocalPartConfig: () => {},
  }
</script>

{#if error}
  <main class="error-screen"><p class="error">Unable to connect to machine: {error}</p></main>
{:else if machine}
  <ViamProvider dialConfigs={{ [machine.id]: machine.dialConf }}>
    {#if selectedResource.name}
      <div class="viz-host">
        <Visualizer partID={machine.id} {localConfigProps}>
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
