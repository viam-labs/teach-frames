// Shared "a move is in flight" flag. Live polling (StatusBar's get_arm_state
// query) pauses while busy>0, and jog/move buttons disable themselves, so a
// slow in-flight move doesn't race a poll tick or let the operator queue up
// contradictory motion commands.
export const motion = $state({ busy: 0 })

export async function withMove<T>(fn: () => Promise<T>): Promise<T> {
  motion.busy++
  try {
    return await fn()
  } finally {
    motion.busy--
  }
}
