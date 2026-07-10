<script lang="ts">
  import { ViamProvider } from '@viamrobotics/svelte-sdk'
  import { currentMachine, provideMachineId, type MachineIdentity } from './lib/machine'

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
  <main>
    <p class="error">Unable to connect to machine: {error}</p>
  </main>
{:else if machine}
  <ViamProvider dialConfigs={{ [machine.id]: machine.dialConf }}>
    <main>
      <h1>Teach Pendant</h1>
    </main>
  </ViamProvider>
{/if}

<style>
  .error {
    color: #b00020;
    padding: 1rem;
  }
</style>
