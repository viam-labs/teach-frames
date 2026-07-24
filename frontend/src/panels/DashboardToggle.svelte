<script lang="ts">
  // Shared toggle button for the Visualizer's native dashboard toolbar
  // (top-center, rendered via <DashboardPortal>). Replicates the classes of
  // the (non-exported) dashboard Button.svelte from @viamrobotics/motion-tools
  // so our panel toggles sit visually flush with the native transform/snap/
  // space buttons instead of looking like a foreign, oversized control.
  //
  // The native button ties its footprint to a default-size icon + p-1.5
  // padding and no explicit width/height — that recipe is kept verbatim below
  // so our toggles are pixel-consistent with the native buttons in the same
  // toolbar row (an earlier min-h-11/min-w-11 hit-target widening made them
  // visibly larger than their neighbours, which read as foreign).
  import type { Snippet } from 'svelte'

  let {
    active,
    label,
    onclick,
    children,
  }: {
    active: boolean
    label: string
    onclick: () => void
    children?: Snippet
  } = $props()
</script>

<label
  class={[
    'relative block rounded-md border active:z-4 active:border-[#666] active:bg-[#666] active:text-white',
    active ? 'z-4 border-[#666] bg-[#666] text-white' : 'border-gray-5 text-gray-8 bg-white',
  ]}
>
  <button
    type="button"
    class="p-1.5"
    aria-pressed={active}
    aria-label={label}
    title={label}
    {onclick}
  >
    {@render children?.()}
  </button>
</label>
