// Mode-adaptive hand-eye calibration wizard. `get_handeye_mode` decides the flow:
//   eye_to_hand  — snapshot once, touch several world points with the TCP and
//                  click each in the image; solve camera→world.
//   eye_in_hand  — touch ONE target, then re-snapshot+click it from ≥3 vantages;
//                  solve camera→flange. Needs a target before any view.
// Pure logic; the panel wires the DoCommands and owns the transient snapshot
// image + click dots. solve_handeye commits atomically (persists to the camera's
// frame), so review is post-commit (the `solved` phase).
import type { CameraMount } from '../poseTracker'

export type HandeyePhase = 'setup' | 'capturing' | 'solved' | 'done'
export interface HandeyeSolve { residual_rms: number; parent: string }

const MIN_CAPTURES = 3

export function createHandeyeWizard() {
  let phase = $state<HandeyePhase>('setup')
  let mode = $state<CameraMount | undefined>(undefined)
  let captureCount = $state(0)
  let hasTarget = $state(false)
  let solve = $state<HandeyeSolve | undefined>(undefined)
  let error = $state<string | undefined>(undefined)

  return {
    get phase() { return phase },
    get mode() { return mode },
    get captureCount() { return captureCount },
    get hasTarget() { return hasTarget },
    get solve() { return solve },
    get error() { return error },
    get minCaptures() { return MIN_CAPTURES },
    get canSolve() { return captureCount >= MIN_CAPTURES },
    get needsTarget() { return mode === 'eye_in_hand' && !hasTarget },
    get done() { return phase === 'done' },

    start(m: CameraMount) {
      mode = m; captureCount = 0; hasTarget = false; solve = undefined; error = undefined
      phase = 'capturing'
    },
    targetTouched() { if (phase === 'capturing') { hasTarget = true; error = undefined } },
    recordCapture(bufferLen: number) { if (phase === 'capturing') { captureCount = bufferLen; error = undefined } },
    solved(result: HandeyeSolve) { solve = result; error = undefined; phase = 'solved' },
    // Re-calibrate: clear_handeye_buffer drops the eye-in-hand target too, so mirror that.
    clearCaptures() { captureCount = 0; hasTarget = false; error = undefined; phase = 'capturing' },
    finish() { phase = 'done' },
    setError(message: string) { error = message },
    reset() {
      phase = 'setup'; mode = undefined; captureCount = 0; hasTarget = false
      solve = undefined; error = undefined
    },
  }
}
