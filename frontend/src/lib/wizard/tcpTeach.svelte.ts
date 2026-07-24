// TCP-calibration wizard: capture ≥4 pivot touches (capture_tcp_point) →
// teach_tcp_position (solves the tip offset + RMS residual AND persists to the
// tool's frame config) → optional explicit orientation. Pure logic; the panel
// wires the DoCommands. Unlike the frame-define wizard there is no frontend
// solver and teach_tcp_position commits atomically, so review is post-commit
// (the `taught` phase) with a Re-teach loop.
export type TcpPhase = 'setup' | 'capturing' | 'taught' | 'orientation' | 'done'

export interface TcpPosition {
  offset: { x: number; y: number; z: number }
  residual_rms: number
}

const MIN_CAPTURES = 4

export function createTcpTeachWizard() {
  let phase = $state<TcpPhase>('setup')
  let captureCount = $state(0)
  let position = $state<TcpPosition | undefined>(undefined)
  let error = $state<string | undefined>(undefined)

  return {
    get phase() { return phase },
    get captureCount() { return captureCount },
    get position() { return position },
    get error() { return error },
    get minCaptures() { return MIN_CAPTURES },
    get canSolve() { return captureCount >= MIN_CAPTURES },
    get done() { return phase === 'done' },

    start() { captureCount = 0; position = undefined; error = undefined; phase = 'capturing' },
    // buffer_len from the capture_tcp_point response is the authoritative count.
    recordCapture(bufferLen: number) {
      if (phase !== 'capturing') return
      captureCount = bufferLen
      error = undefined
    },
    solved(result: TcpPosition) { position = result; error = undefined; phase = 'taught' },
    reteach() { captureCount = 0; position = undefined; error = undefined; phase = 'capturing' },
    toOrientation() { if (phase === 'taught') phase = 'orientation' },
    finish() { phase = 'done' },
    setError(message: string) { error = message },
    reset() { phase = 'setup'; captureCount = 0; position = undefined; error = undefined },
  }
}
