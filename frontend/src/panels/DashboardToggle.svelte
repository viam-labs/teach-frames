<script lang="ts">
  // Shared toggle button for the Visualizer's native dashboard toolbar
  // (top-center, rendered via <DashboardPortal>). Replicates the classes of
  // the (non-exported) dashboard Button.svelte from @viamrobotics/motion-tools
  // so our panel toggles sit visually flush with the native transform/snap/
  // space buttons instead of looking like a foreign, oversized control.
  //
  // The native button ties its 32px visual footprint to a 16px icon + p-1.5
  // padding — that recipe is kept verbatim below. `min-h-11`/`min-w-11` widen
  // the actual hit target for touch without growing the visible chrome
  // (the label wraps the button so the extra hit area stays inside a single
  // rounded surface, matching the native active/pressed treatment).
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
    class="flex min-h-11 min-w-11 items-center justify-center p-1.5"
    aria-pressed={active}
    aria-label={label}
    title={label}
    {onclick}
  >
    {@render children?.()}
  </button>
</label>
