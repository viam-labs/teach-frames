import { describe, it, expect, beforeEach } from 'vitest'
import { createTcpTeachWizard } from './tcpTeach.svelte'

const posResult = { offset: { x: 1, y: 2, z: 3 }, residual_rms: 0.4 }

describe('tcpTeach wizard', () => {
  let w: ReturnType<typeof createTcpTeachWizard>
  beforeEach(() => { w = createTcpTeachWizard() })

  it('starts idle in setup, 0 captures, cannot solve', () => {
    expect(w.phase).toBe('setup')
    expect(w.captureCount).toBe(0)
    expect(w.canSolve).toBe(false)
    expect(w.minCaptures).toBe(4)
  })

  it('start() enters capturing and resets state', () => {
    w.start()
    expect(w.phase).toBe('capturing')
    expect(w.captureCount).toBe(0)
    expect(w.position).toBeUndefined()
  })

  it('recordCapture tracks the server buffer_len and gates solving at >=4', () => {
    w.start()
    w.recordCapture(1); expect(w.captureCount).toBe(1); expect(w.canSolve).toBe(false)
    w.recordCapture(2); w.recordCapture(3); expect(w.canSolve).toBe(false)
    w.recordCapture(4); expect(w.captureCount).toBe(4); expect(w.canSolve).toBe(true)
  })

  it('ignores recordCapture outside the capturing phase', () => {
    w.recordCapture(3) // still in setup
    expect(w.captureCount).toBe(0)
  })

  it('solved() stores the result and enters taught', () => {
    w.start(); w.recordCapture(4)
    w.solved(posResult)
    expect(w.phase).toBe('taught')
    expect(w.position).toEqual(posResult)
  })

  it('reteach() clears back to capturing', () => {
    w.start(); w.recordCapture(4); w.solved(posResult)
    w.reteach()
    expect(w.phase).toBe('capturing')
    expect(w.captureCount).toBe(0)
    expect(w.position).toBeUndefined()
  })

  it('toOrientation() only advances from taught', () => {
    w.toOrientation() // from setup: no-op
    expect(w.phase).toBe('setup')
    w.start(); w.recordCapture(4); w.solved(posResult)
    w.toOrientation()
    expect(w.phase).toBe('orientation')
  })

  it('finish() reaches done from taught or orientation', () => {
    w.start(); w.recordCapture(4); w.solved(posResult)
    w.finish()
    expect(w.phase).toBe('done')
    expect(w.done).toBe(true)

    // also reachable from the orientation phase
    const w2 = createTcpTeachWizard()
    w2.start(); w2.recordCapture(4); w2.solved(posResult); w2.toOrientation()
    w2.finish()
    expect(w2.phase).toBe('done')
  })

  it('setError surfaces a message without losing the capture count', () => {
    w.start(); w.recordCapture(4)
    w.setError('too few points')
    expect(w.error).toBe('too few points')
    expect(w.captureCount).toBe(4)
  })

  it('reset() returns to a fresh setup wizard', () => {
    w.start(); w.recordCapture(4); w.solved(posResult); w.reset()
    expect(w.phase).toBe('setup')
    expect(w.captureCount).toBe(0)
    expect(w.position).toBeUndefined()
    expect(w.error).toBeUndefined()
  })
})
