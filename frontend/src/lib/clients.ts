import { createResourceClient } from '@viamrobotics/svelte-sdk'
import { PoseTrackerClient } from '@viamrobotics/sdk'

// The resource name is not hardcoded: it is chosen during onboarding (see
// ResourcePicker) and passed in as a reactive accessor so the client tracks the
// selected pose-tracker resource.
export function poseTrackerClient(partID: () => string, name: () => string) {
  return createResourceClient(PoseTrackerClient, partID, name)
}
