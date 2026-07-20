import type { PoseMap } from '../poseTracker'

// Steps: 0 name&start · 1 capture P0 · 2 capture P1 · 3 capture P2 ·
// 4 preview&commit · 5 committed. Pure logic; the panel wires DoCommands and
// the scene plugin reads `captures`/`step` to draw the spatial story.
export type WizardStep = 0 | 1 | 2 | 3 | 4 | 5

export function createFrameDefineWizard() {
  let step = $state<WizardStep>(0)
  let name = $state('')
  let captures = $state<PoseMap[]>([])
  let error = $state<string | undefined>(undefined)

  return {
    get step() { return step },
    get name() { return name },
    get captures() { return captures },
    get error() { return error },
    get canCommit() { return captures.length === 3 },
    get done() { return step === 5 },

    start(rawName: string): boolean {
      const trimmed = rawName.trim()
      if (!trimmed) return false
      name = trimmed; captures = []; error = undefined; step = 1
      return true
    },
    recordCapture(pose: PoseMap) {
      if (step < 1 || step > 3) return
      captures = [...captures, pose]
      error = undefined
      step = (captures.length === 3 ? 4 : step + 1) as WizardStep
    },
    recapture() { captures = []; error = undefined; step = 1 },
    setError(message: string) { error = message },
    committed() { step = 5 },
    reset() { step = 0; name = ''; captures = []; error = undefined },
  }
}
