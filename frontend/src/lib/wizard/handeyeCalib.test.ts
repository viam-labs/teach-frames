import { describe, it, expect, beforeEach } from 'vitest'
import { createHandeyeWizard } from './handeyeCalib.svelte'

const solveResult = { residual_rms: 0.8, parent: 'world' }

describe('handeyeCalib wizard', () => {
  let w: ReturnType<typeof createHandeyeWizard>
  beforeEach(() => { w = createHandeyeWizard() })

  it('starts in setup, no mode, 0 captures, cannot solve, no target need', () => {
    expect(w.phase).toBe('setup')
    expect(w.mode).toBeUndefined()
    expect(w.captureCount).toBe(0)
    expect(w.canSolve).toBe(false)
    expect(w.minCaptures).toBe(3)
    expect(w.needsTarget).toBe(false)
  })

  it('start(eye_to_hand) → capturing, no target step', () => {
    w.start('eye_to_hand')
    expect(w.phase).toBe('capturing')
    expect(w.mode).toBe('eye_to_hand')
    expect(w.needsTarget).toBe(false)
  })

  it('start(eye_in_hand) → capturing and needs a target first', () => {
    w.start('eye_in_hand')
    expect(w.phase).toBe('capturing')
    expect(w.needsTarget).toBe(true)
    w.targetTouched()
    expect(w.hasTarget).toBe(true)
    expect(w.needsTarget).toBe(false)
  })

  it('targetTouched only applies while capturing', () => {
    w.targetTouched() // in setup
    expect(w.hasTarget).toBe(false)
  })

  it('recordCapture tracks buffer_len and gates solve at >=3', () => {
    w.start('eye_to_hand')
    w.recordCapture(1); expect(w.canSolve).toBe(false)
    w.recordCapture(2); expect(w.canSolve).toBe(false)
    w.recordCapture(3); expect(w.captureCount).toBe(3); expect(w.canSolve).toBe(true)
  })

  it('ignores recordCapture outside capturing', () => {
    w.recordCapture(3)
    expect(w.captureCount).toBe(0)
  })

  it('solved() stores the result and enters solved', () => {
    w.start('eye_to_hand'); w.recordCapture(3)
    w.solved(solveResult)
    expect(w.phase).toBe('solved')
    expect(w.solve).toEqual(solveResult)
  })

  it('clearCaptures() resets count + target and returns to capturing', () => {
    w.start('eye_in_hand'); w.targetTouched(); w.recordCapture(3); w.solved(solveResult)
    w.clearCaptures()
    expect(w.phase).toBe('capturing')
    expect(w.captureCount).toBe(0)
    expect(w.hasTarget).toBe(false)
  })

  it('finish() reaches done (from solved)', () => {
    w.start('eye_to_hand'); w.recordCapture(3); w.solved(solveResult)
    w.finish()
    expect(w.phase).toBe('done')
    expect(w.done).toBe(true)
  })

  it('setError surfaces a message without losing count', () => {
    w.start('eye_to_hand'); w.recordCapture(2)
    w.setError('no depth at pixel')
    expect(w.error).toBe('no depth at pixel')
    expect(w.captureCount).toBe(2)
  })

  it('reset() returns to a fresh setup wizard', () => {
    w.start('eye_in_hand'); w.targetTouched(); w.recordCapture(3); w.solved(solveResult); w.reset()
    expect(w.phase).toBe('setup')
    expect(w.mode).toBeUndefined()
    expect(w.captureCount).toBe(0)
    expect(w.hasTarget).toBe(false)
    expect(w.solve).toBeUndefined()
  })
})
