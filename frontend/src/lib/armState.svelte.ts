import type { PoseMap } from './poseTracker'

// Shared live arm-state snapshot. TcpTriad currently owns the get_arm_state
// poll and writes into this store (JogPanel will take over that ownership
// once it's mounted, per the note in TcpTriad.svelte); any other panel
// reads it to decide whether to render its controls enabled, without each
// panel running its own duplicate query.
export const armState = $state<{
  pose: PoseMap | null
  joints: number[]
  hasArm: boolean
}>({ pose: null, joints: [], hasArm: false })
