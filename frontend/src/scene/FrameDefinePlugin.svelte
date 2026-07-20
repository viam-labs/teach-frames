<script lang="ts">
  // In-scene "spatial story" for the frame-define wizard: numbered markers at
  // each captured point, a 3point-only live guide line from P0 toward the
  // current TCP while capturing, and a method-appropriate provisional frame
  // triad during the preview phase (via provisionalTriad). Reads the SAME
  // wizard store the panel (FrameDefineWizard) drives (see App.svelte) and
  // the SAME live armState.pose TcpTriad polls — this component owns no
  // poll of its own.
  import { T } from '@threlte/core'
  import { MeshLineGeometry, MeshLineMaterial } from '@threlte/extras'
  import { AxesHelper } from '@viamrobotics/motion-tools/lib'
  import { Vector3 } from 'three'
  import { armState } from '../lib/armState.svelte'
  import type { createFrameDefineWizard } from '../lib/wizard/frameDefine.svelte'
  import type { PoseMap } from '../lib/poseTracker'
  import { provisionalTriad } from './provisionalTriad'

  let { wizard }: { wizard: ReturnType<typeof createFrameDefineWizard> } = $props()

  const MM_TO_M = 0.001

  // mm PoseMap/point -> meters scene tuple.
  function toScenePosition(p: { x: number; y: number; z: number }): [number, number, number] {
    return [p.x * MM_TO_M, p.y * MM_TO_M, p.z * MM_TO_M]
  }

  const markers = $derived(wizard.captures.map((c: PoseMap) => toScenePosition(c)))

  // Only the preview phase shows the provisional triad — once committed,
  // the real world_state_store triad takes over, and this must stop
  // drawing or the scene would show two triads at once.
  const triad = $derived(
    wizard.phase === 'preview' ? provisionalTriad(wizard.method, wizard.captures) : null,
  )

  // Live preview: while capturing 3point and at least P0 is down, draw a
  // thin guide line from the origin to the live TCP — it previews whichever
  // axis point is currently being taught. The other methods only ever
  // capture a single point, so a guide line toward the live TCP wouldn't
  // preview anything meaningful.
  const previewLine = $derived(
    wizard.method === '3point' &&
    wizard.phase === 'capturing' &&
    wizard.captures.length >= 1 &&
    armState.pose
      ? [
          new Vector3(...toScenePosition(wizard.captures[0])),
          new Vector3(...toScenePosition(armState.pose)),
        ]
      : null,
  )
</script>

{#if wizard.phase === 'capturing' || wizard.phase === 'preview'}
  {#each markers as position, i (i)}
    <T.Mesh {position} raycast={() => null} renderOrder={1}>
      <T.SphereGeometry args={[0.006]} />
      <T.MeshBasicMaterial color="#3560ee" depthTest={false} />
    </T.Mesh>
  {/each}
{/if}

{#if previewLine}
  <T.Mesh raycast={() => null} renderOrder={1}>
    <MeshLineGeometry points={previewLine} />
    <MeshLineMaterial width={1.5} depthTest={false} color="#3560ee" opacity={0.6} attenuate={false} transparent />
  </T.Mesh>
{/if}

{#if triad}
  <AxesHelper
    position={triad.position}
    quaternion={triad.quaternion}
    length={0.12}
    width={3}
    depthTest={false}
  />
{/if}
