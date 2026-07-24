<script lang="ts">
  // Live TCP (tool-center-point) triad: a coordinate-axes gizmo rendered at
  // the arm's current end-effector pose inside the Visualizer scene.
  //
  // Pure reader: JogPanel now owns the get_arm_state poll (it needs the live
  // pose/joints for its readout and drives it from jog responses too), so
  // this component just reads the shared armState store — no query, no
  // polling. JogPanel and TcpTriad are always mounted as siblings (see
  // App.svelte), so armState stays populated regardless of which panel is
  // open.
  import { AxesHelper } from '@viamrobotics/motion-tools/lib'
  import { armState } from '../lib/armState.svelte'
  import { poseToSceneTransform } from './poseTransform'

  const transform = $derived(armState.pose ? poseToSceneTransform(armState.pose) : null)
</script>

{#if transform}
  <!-- depthTest={false} draws the triad ON TOP of the arm/gripper mesh it sits
       inside — otherwise (the AxesHelper default depthTest=true) the tool-tip
       triad is occluded by the gripper and only a faint sliver shows. A wider
       linewidth than the 0.1 default makes it read as a clear TCP indicator. -->
  <AxesHelper
    position={transform.position}
    quaternion={transform.quaternion}
    length={0.12}
    width={3}
    depthTest={false}
  />
{/if}
