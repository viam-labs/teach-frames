import { describe, it, expect, beforeEach } from 'vitest'
import { createFrameDefineWizard } from './frameDefine.svelte'
import type { PoseMap } from '../poseTracker'

const pose = (x: number, y: number, z: number): PoseMap =>
  ({ x, y, z, o_x: 0, o_y: 0, o_z: 1, theta: 0 })

describe('frameDefine wizard', () => {
  let w: ReturnType<typeof createFrameDefineWizard>
  beforeEach(() => { w = createFrameDefineWizard() })

  it('starts idle in the setup phase with no captures', () => {
    expect(w.phase).toBe('setup')
    expect(w.method).toBe('3point')
    expect(w.captures).toEqual([])
    expect(w.canCommit).toBe(false)
  })

  it('start() requires a non-blank name and enters the capturing phase', () => {
    expect(w.start('', '3point')).toBe(false)
    expect(w.phase).toBe('setup')
    expect(w.start('  table  ', '3point')).toBe(true)
    expect(w.name).toBe('table')   // trimmed
    expect(w.method).toBe('3point')
    expect(w.phase).toBe('capturing')
  })

  it('3point requires 3 captures before preview', () => {
    w.start('f', '3point')
    expect(w.requiredCaptures).toBe(3)
    w.recordCapture(pose(0, 0, 0)); expect(w.phase).toBe('capturing'); expect(w.captureIndex).toBe(1)
    w.recordCapture(pose(1, 0, 0)); expect(w.phase).toBe('capturing'); expect(w.captureIndex).toBe(2)
    expect(w.canCommit).toBe(false)
    w.recordCapture(pose(0, 1, 0)); expect(w.phase).toBe('preview'); expect(w.canCommit).toBe(true)
  })

  it('point needs exactly 1 capture and goes straight to preview', () => {
    w.start('f', 'point')
    expect(w.requiredCaptures).toBe(1)
    w.recordCapture(pose(2, 3, 4))
    expect(w.phase).toBe('preview')
    expect(w.canCommit).toBe(true)
    expect(w.captures.length).toBe(1)
  })

  it('tcp_snapshot needs exactly 1 capture and goes straight to preview', () => {
    w.start('f', 'tcp_snapshot')
    expect(w.requiredCaptures).toBe(1)
    w.recordCapture(pose(2, 3, 4))
    expect(w.phase).toBe('preview')
    expect(w.canCommit).toBe(true)
  })

  it('ignores captures once out of the capturing phase', () => {
    w.start('f', 'point')
    w.recordCapture(pose(0, 0, 0))   // -> preview
    w.recordCapture(pose(9, 9, 9))   // dropped
    expect(w.captures.length).toBe(1)
  })

  it('recapture() clears captures and returns to capturing', () => {
    w.start('f', '3point')
    w.recordCapture(pose(0, 0, 0)); w.recordCapture(pose(1, 0, 0)); w.recordCapture(pose(0, 1, 0))
    w.recapture()
    expect(w.captures).toEqual([]); expect(w.phase).toBe('capturing'); expect(w.error).toBeUndefined()
  })

  it('setError surfaces a backend failure without losing captures', () => {
    w.start('f', '3point'); w.recordCapture(pose(0, 0, 0))
    w.setError('collinear points: X axis undefined')
    expect(w.error).toBe('collinear points: X axis undefined')
    expect(w.captures.length).toBe(1)
  })

  it('committed() advances to the committed phase', () => {
    w.start('f', 'point')
    w.recordCapture(pose(0, 0, 0))
    w.committed()
    expect(w.phase).toBe('committed'); expect(w.done).toBe(true)
  })

  it('reset() returns to a fresh setup wizard (method back to 3point)', () => {
    w.start('f', 'tcp_snapshot'); w.recordCapture(pose(0, 0, 0)); w.reset()
    expect(w.phase).toBe('setup'); expect(w.method).toBe('3point'); expect(w.name).toBe(''); expect(w.captures).toEqual([])
  })
})
