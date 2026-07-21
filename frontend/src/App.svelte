<script lang="ts">
  import { ViamProvider } from '@viamrobotics/svelte-sdk'
  import { Visualizer } from '@viamrobotics/motion-tools'
  import { Struct } from '@viamrobotics/sdk'
  import { currentMachine, provideMachineId, type MachineIdentity } from './lib/machine'
  import { selectedResource } from './lib/resource.svelte'
  import { createFrameDefineWizard } from './lib/wizard/frameDefine.svelte'
  import { createTcpTeachWizard } from './lib/wizard/tcpTeach.svelte'
  import { frameRevision } from './lib/frameRevision.svelte'
  import ResourcePicker from './panels/ResourcePicker.svelte'
  import FrameDefineWizard from './panels/FrameDefineWizard.svelte'
  import TcpTeachWizard from './panels/TcpTeachWizard.svelte'
  import ManageFramesPanel from './panels/ManageFramesPanel.svelte'
  import JogPanel from './panels/JogPanel.svelte'
  import TcpTriad from './scene/TcpTriad.svelte'
  import FrameDefinePlugin from './scene/FrameDefinePlugin.svelte'

  let machine: MachineIdentity | undefined
  let error: string | undefined

  // Single shared instance: the panel (this file's <FrameDefineWizard>) and
  // the scene plugin (Task 5) must read/write the SAME wizard store so the
  // panel's steps and the scene's spatial story never disagree.
  const wizard = createFrameDefineWizard()
  // Same reasoning as `wizard` above: created outside the {#key frameRevision}
  // block so TCP-teach progress survives the Bug-1 scene remount.
  const tcpWizard = createTcpTeachWizard()

  try {
    machine = currentMachine()
    provideMachineId(machine.id)
  } catch (err) {
    error = err instanceof Error ? err.message : String(err)
  }

  // The Visualizer's part-config layer has two backends. Without localConfigProps
  // it takes the STANDALONE path, which calls createAppQuery('getRobotPart') and
  // needs a ViamAppProvider/app-key credentials we deliberately don't supply
  // (this Application authenticates with only the machine cookie). That path
  // crashes: "can't access property 'current', viamClient is undefined".
  // Passing localConfigProps routes it to the EMBEDDED (host-owned) backend
  // instead. We hand it a no-op empty config: the live scene doesn't need it
  // (the arm and taught-frame triads come from the robot's frameSystemConfig +
  // world_state_store; part config only tints frame colors and drives build-mode
  // editing, which we don't use), and frame persistence stays on our own
  // pose-tracker define_frame DoCommand.
  const localConfigProps = {
    current: Struct.fromJson({ components: [] }),
    isDirty: false,
    setLocalPartConfig: () => {},
  }

  // Same standalone-vs-embedded split, second (and last) seam: useFragmentInfo.
  // Clicking a component in the scene makes the Details panel ask whether that
  // component is fragment-defined, which on the STANDALONE path fires
  // createAppQuery('getFragment') and hits the same missing-viamClient crash
  // (and locks the UI in a re-render loop). Passing componentNameToFragmentInfo
  // — even an empty map — routes it to the EMBEDDED path. Empty is correct here:
  // we don't drive fragment-variable editing, and no component needs to report
  // as fragment-defined. These two props (with localConfigProps) are the only
  // createAppQuery/createAppMutation call sites in the Visualizer, so this
  // fully closes the app-cloud dependency for our cookie-only Application.
  const componentNameToFragmentInfo = {}
</script>

{#if error}
  <main class="error-screen"><p class="error">Unable to connect to machine: {error}</p></main>
{:else if machine}
  <ViamProvider dialConfigs={{ [machine.id]: machine.dialConf }}>
    {#if selectedResource.name}
      <div class="viz-host">
        <!--
          Keyed on frameRevision so the Visualizer scene remounts (and
          useWorldState re-initializes/re-snapshots) after OUR OWN taught-frame
          commits/deletes/clears. Stopgap for a motion-tools bug where the
          world_state_store live stream goes permanently stale after the first
          AlwaysRebuild reconfigure — see
          docs/plans/2026-07-20-motion-tools-wss-reconfigure-upstream-patch.md.
          Scoped to just <Visualizer> (not ViamProvider) so the robot
          connection + query cache survive the remount; only the 3D scene
          re-initializes.
        -->
        {#key frameRevision.value}
          <Visualizer partID={machine.id} {localConfigProps} {componentNameToFragmentInfo}>
            {#snippet children()}
              <TcpTriad />
              <FrameDefinePlugin {wizard} />
              <FrameDefineWizard {wizard} />
              <TcpTeachWizard wizard={tcpWizard} />
              <ManageFramesPanel />
              <JogPanel />
            {/snippet}
          </Visualizer>
        {/key}
      </div>
    {:else}
      <ResourcePicker />
    {/if}
  </ViamProvider>
{/if}

<style>
  :global(html, body) { margin: 0; height: 100%; }
  :global(*) { box-sizing: border-box; }
  .viz-host { position: fixed; inset: 0; }
  .error-screen { padding: 1rem; }
  .error { color: #ff8080; padding: 1rem; }
</style>
