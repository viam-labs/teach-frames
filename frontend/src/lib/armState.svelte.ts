import type { PoseMap } from './poseTracker'

// Shared live arm-state snapshot. JogPanel owns the get_arm_state poll and
// writes into this store; TcpTriad and FrameDefinePlugin (and any other
// panel) are pure readers, deciding whether to render controls enabled or
// where to draw scene geometry, without each running its own duplicate
// query.
export const armState = $state<{
  pose: PoseMap | null
  joints: number[]
  hasArm: boolean
}>({ pose: null, joints: [], hasArm: false })
