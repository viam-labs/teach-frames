import { createResourceClient } from '@viamrobotics/svelte-sdk'
import { PoseTrackerClient } from '@viamrobotics/sdk'

export function poseTrackerClient(partID: () => string, name = 'pose-tracker') {
  return createResourceClient(PoseTrackerClient, partID, () => name)
}
