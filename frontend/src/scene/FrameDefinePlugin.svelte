<script lang="ts">
  // In-scene "spatial story" for the 3-point frame-define wizard: numbered
  // markers at each captured point, a live axis preview from P0 toward the
  // current TCP while capturing, and a provisional frame triad once all 3
  // points are in. Reads the SAME wizard store the panel (FrameDefineWizard)
  // drives (see App.svelte) and the SAME live armState.pose TcpTriad polls —
  // this component owns no poll of its own.
  import { T } from '@threlte/core'
  import { MeshLineGeometry, MeshLineMaterial } from '@threlte/extras'
  import { AxesHelper } from '@viamrobotics/motion-tools/lib'
  import { Vector3 } from 'three'
  import { armState } from '../lib/armState.svelte'
  import type { createFrameDefineWizard } from '../lib/wizard/frameDefine.svelte'
  import type { PoseMap } from '../lib/poseTracker'
  import { threePointBasis, basisToQuaternion } from './frameGeometry'

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
  const provisionalBasis = $derived(
    wizard.phase === 'preview' && wizard.captures.length === 3
      ? threePointBasis(wizard.captures[0], wizard.captures[1], wizard.captures[2])
      : null,
  )

  const provisionalTriad = $derived(
    provisionalBasis
      ? {
          position: [
            provisionalBasis.origin[0] * MM_TO_M,
            provisionalBasis.origin[1] * MM_TO_M,
            provisionalBasis.origin[2] * MM_TO_M,
          ] as [number, number, number],
          quaternion: basisToQuaternion(provisionalBasis),
        }
      : null,
  )

  // Live preview: while capturing and at least P0 is down, draw a thin
  // guide line from the origin to the live TCP — it previews whichever
  // axis point is currently being taught.
  const previewLine = $derived(
    wizard.phase === 'capturing' && wizard.captures.length >= 1 && armState.pose
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

{#if provisionalTriad}
  <AxesHelper
    position={provisionalTriad.position}
    quaternion={provisionalTriad.quaternion}
    length={0.12}
    width={3}
    depthTest={false}
  />
{/if}
