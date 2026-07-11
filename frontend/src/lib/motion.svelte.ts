// Shared motion state.
// - `busy`: a move is in flight; the live arm-state poll pauses while busy>0 so
//   it doesn't race an in-flight move.
// - `stopSeq`: bumped whenever Stop is pressed. A press-and-hold jog loop
//   captures the current value and bails the moment it changes, so Stop
//   immediately halts a continuous jog even while the button is still held.
export const motion = $state({ busy: 0, stopSeq: 0 })

export async function withMove<T>(fn: () => Promise<T>): Promise<T> {
  motion.busy++
  try {
    return await fn()
  } finally {
    motion.busy--
  }
}

// Signal that all motion should stop (e.g. the Stop button was pressed).
export function requestStop(): void {
  motion.stopSeq++
}
