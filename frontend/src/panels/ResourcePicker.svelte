<script lang="ts">
  import { useResourceNames } from '@viamrobotics/svelte-sdk'
  import { useMachineId } from '../lib/machine'
  import { selectedResource } from '../lib/resource.svelte'

  const machineId = useMachineId()

  // Discover every resource on the machine that implements the PoseTracker API
  // (rdk:component:pose_tracker → subtype "pose_tracker").
  const resources = useResourceNames(machineId, 'pose_tracker')

  // Default the selection to the first discovered resource, once they load.
  // Guarded so it sets exactly once and never clobbers an operator's choice.
  let choice = $state<string | null>(null)
  $effect(() => {
    if (choice === null && resources.current.length > 0) {
      choice = resources.current[0].name
    }
  })

  function start() {
    if (choice) {
      selectedResource.name = choice
    }
  }
</script>

<main class="bg-light flex min-h-screen items-center justify-center p-6">
  <div class="border-light flex w-full max-w-md flex-col gap-3 rounded-lg border bg-white p-7 shadow-sm">
    <h1 class="text-gray-9 text-xl font-semibold">Teach Pendant</h1>
    <p class="text-subtle-1 text-sm">Choose the pose-tracker resource to teach with.</p>

    {#if resources.current.length === 0}
      <p class="text-subtle-2 text-sm italic">Discovering pose-tracker resources…</p>
      <p class="text-subtle-2 text-xs">
        If none appear, add a
        <code class="bg-light font-roboto-mono rounded px-1 py-0.5 text-[0.85em]">pose_tracker</code>
        component (for example
        <code class="bg-light font-roboto-mono rounded px-1 py-0.5 text-[0.85em]"
          >viam-labs:teach-frames:pose-tracker</code
        >) to this machine and reload.
      </p>
    {:else}
      <label for="resource" class="text-subtle-1 mt-1 text-sm">Pose-tracker resource</label>
      <select
        id="resource"
        bind:value={choice}
        class="border-medium hover:border-gray-6 focus:border-gray-9 text-gray-9 min-h-11 w-full rounded-md border bg-white px-3 text-sm focus:outline-none focus-visible:ring-2 focus-visible:ring-gray-9/30"
      >
        {#each resources.current as resource (resource.name)}
          <option value={resource.name}>{resource.name}</option>
        {/each}
      </select>
      <button
        type="button"
        onclick={start}
        disabled={!choice}
        class="border-gray-9 bg-gray-9 mt-2 min-h-11 rounded-md border px-4 text-sm font-medium text-white hover:border-black hover:bg-black active:bg-black focus:outline-none focus-visible:ring-2 focus-visible:ring-gray-9/40 focus-visible:ring-offset-2 disabled:cursor-not-allowed disabled:!border-disabled-light disabled:!bg-disabled-light disabled:text-disabled-dark"
      >
        Start teaching
      </button>
    {/if}
  </div>
</main>
