import type { PoseMap } from './poseTracker'

// Shared live arm-state snapshot. StatusBar owns the get_arm_state poll and
// writes into this store; any other panel (JogPanel, TcpPanel) reads it to
// decide whether to render its controls enabled, without each panel running
// its own duplicate query.
export const armState = $state<{
  pose: PoseMap | null
  joints: number[]
  hasArm: boolean
}>({ pose: null, joints: [], hasArm: false })
