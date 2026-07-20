import { describe, it, expect, beforeEach } from 'vitest'
import { createFrameDefineWizard } from './frameDefine.svelte'
import type { PoseMap } from '../poseTracker'

const pose = (x: number, y: number, z: number): PoseMap =>
  ({ x, y, z, o_x: 0, o_y: 0, o_z: 1, theta: 0 })

describe('frameDefine wizard', () => {
  let w: ReturnType<typeof createFrameDefineWizard>
  beforeEach(() => { w = createFrameDefineWizard() })

  it('starts idle at step 0 with no captures', () => {
    expect(w.step).toBe(0)
    expect(w.captures).toEqual([])
    expect(w.canCommit).toBe(false)
  })

  it('start() requires a non-blank name and advances to capture P0', () => {
    expect(w.start('')).toBe(false)
    expect(w.step).toBe(0)
    expect(w.start('  table  ')).toBe(true)
    expect(w.name).toBe('table')   // trimmed
    expect(w.step).toBe(1)
  })

  it('records captures and advances one step each', () => {
    w.start('f')
    w.recordCapture(pose(0, 0, 0)); expect(w.step).toBe(2); expect(w.captures.length).toBe(1)
    w.recordCapture(pose(1, 0, 0)); expect(w.step).toBe(3); expect(w.captures.length).toBe(2)
    w.recordCapture(pose(0, 1, 0)); expect(w.step).toBe(4); expect(w.captures.length).toBe(3)
  })

  it('is committable only with exactly 3 captures', () => {
    w.start('f')
    w.recordCapture(pose(0, 0, 0)); w.recordCapture(pose(1, 0, 0))
    expect(w.canCommit).toBe(false)
    w.recordCapture(pose(0, 1, 0))
    expect(w.canCommit).toBe(true)
  })

  it('recapture() clears captures and returns to capture P0', () => {
    w.start('f')
    w.recordCapture(pose(0, 0, 0)); w.recordCapture(pose(1, 0, 0)); w.recordCapture(pose(0, 1, 0))
    w.recapture()
    expect(w.captures).toEqual([]); expect(w.step).toBe(1); expect(w.error).toBeUndefined()
  })

  it('setError surfaces a backend failure without losing captures', () => {
    w.start('f'); w.recordCapture(pose(0, 0, 0))
    w.setError('collinear points: X axis undefined')
    expect(w.error).toBe('collinear points: X axis undefined')
    expect(w.captures.length).toBe(1)
  })

  it('committed() advances to the done step', () => {
    w.start('f')
    w.recordCapture(pose(0, 0, 0)); w.recordCapture(pose(1, 0, 0)); w.recordCapture(pose(0, 1, 0))
    w.committed()
    expect(w.step).toBe(5); expect(w.done).toBe(true)
  })

  it('reset() returns to a fresh idle wizard', () => {
    w.start('f'); w.recordCapture(pose(0, 0, 0)); w.reset()
    expect(w.step).toBe(0); expect(w.name).toBe(''); expect(w.captures).toEqual([])
  })
})
