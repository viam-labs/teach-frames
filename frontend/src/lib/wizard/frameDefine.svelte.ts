import type { FrameMethod, PoseMap } from '../poseTracker'

// Phases: setup (choose method + name) · capturing (jog + Capture ×N) ·
// preview (review the provisional triad, then Commit) · committed.
// Pure logic; the panel wires DoCommands and the scene plugin reads
// `phase`/`method`/`captures` to draw the matching spatial story.
export type WizardPhase = 'setup' | 'capturing' | 'preview' | 'committed'

export function createFrameDefineWizard() {
  let phase = $state<WizardPhase>('setup')
  let method = $state<FrameMethod>('3point')
  let name = $state('')
  let captures = $state<PoseMap[]>([])
  let error = $state<string | undefined>(undefined)

  const required = () => (method === '3point' ? 3 : 1)

  return {
    get phase() { return phase },
    get method() { return method },
    get name() { return name },
    get captures() { return captures },
    get error() { return error },
    get requiredCaptures() { return required() },
    // 0-based index of the point being captured next (also = count captured so far).
    get captureIndex() { return captures.length },
    get canCommit() { return captures.length === required() },
    get done() { return phase === 'committed' },

    start(rawName: string, m: FrameMethod): boolean {
      const trimmed = rawName.trim()
      if (!trimmed) return false
      name = trimmed; method = m; captures = []; error = undefined; phase = 'capturing'
      return true
    },
    recordCapture(pose: PoseMap) {
      if (phase !== 'capturing') return
      captures = [...captures, pose]
      error = undefined
      if (captures.length >= required()) phase = 'preview'
    },
    recapture() { captures = []; error = undefined; phase = 'capturing' },
    setError(message: string) { error = message },
    committed() { phase = 'committed' },
    reset() { phase = 'setup'; method = '3point'; name = ''; captures = []; error = undefined },
  }
}
