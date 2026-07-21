// A monotonically increasing counter bumped whenever THIS frontend commits/
// deletes/clears a taught frame. App.svelte keys <Visualizer> on it so the
// scene remounts and re-snapshots world_state_store — a stopgap for the
// motion-tools WSS-stale-across-reconfigure bug (see
// docs/plans/2026-07-20-motion-tools-wss-reconfigure-upstream-patch.md).
let revision = $state(0)

export const frameRevision = {
  get value() {
    return revision
  },
}

export function bumpFrameRevision() {
  revision += 1
}
