<script lang="ts">
  // Live TCP (tool-center-point) triad: a coordinate-axes gizmo rendered at
  // the arm's current end-effector pose inside the Visualizer scene. This is
  // the first scene-coupled plugin — it proves the pattern of a Svelte
  // component mounted inside <Visualizer>'s `children` snippet that renders
  // 3D and reads live robot state.
  //
  // JogPanel is NOT mounted yet (see App.svelte), so there's no other poll to
  // fight with here. This component owns its own get_arm_state poll — mirrors
  // the pattern in JogPanel.svelte exactly (same query + polling shape), just
  // without the jog controls.
  import { AxesHelper } from '@viamrobotics/motion-tools/lib'
  import { createResourceQuery, usePolling } from '@viamrobotics/svelte-sdk'
  import { poseTrackerClient } from '../lib/clients'
  import { selectedResource } from '../lib/resource.svelte'
  import { useMachineId } from '../lib/machine'
  import { armState } from '../lib/armState.svelte'
  import { getArmState, parseArmState, toCommandArgs } from '../lib/poseTracker'
  import { poseToSceneTransform } from './poseTransform'

  const machineId = useMachineId()
  const pt = poseTrackerClient(machineId, () => selectedResource.name ?? '')

  const armStateQuery = createResourceQuery(
    pt,
    'doCommand',
    () => toCommandArgs(getArmState()),
    () => ({}),
  )

  // Mirrors JogPanel's guard exactly: pause only on the specific "no arm
  // configured" error, not on any error. A transient failure on the very
  // first poll (before any success has populated data) must NOT wedge
  // polling off forever — that would leave the triad permanently gone even
  // though the arm is fine, since usePolling only reruns when queryKey
  // changes, not on a timer of its own.
  const NO_ARM_PHRASE = 'arm dependency not configured'
  const noArmConfigured = $derived(
    armStateQuery.error !== null && armStateQuery.error.message.includes(NO_ARM_PHRASE),
  )

  usePolling(
    () => armStateQuery.queryKey,
    () => (noArmConfigured ? false : 500),
  )

  $effect(() => {
    if (armStateQuery.data) {
      const parsed = parseArmState(armStateQuery.data as Record<string, unknown>)
      armState.pose = parsed.pose
      armState.joints = parsed.joints
      armState.hasArm = true
    } else if (armStateQuery.error) {
      armState.hasArm = false
    }
  })

  const transform = $derived(armState.pose ? poseToSceneTransform(armState.pose) : null)
</script>

{#if transform}
  <AxesHelper position={transform.position} quaternion={transform.quaternion} length={0.1} />
{/if}
