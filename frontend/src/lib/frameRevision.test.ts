import { describe, it, expect } from 'vitest'
import { frameRevision, bumpFrameRevision } from './frameRevision.svelte'

describe('frameRevision', () => {
  it('starts at 0', () => {
    expect(frameRevision.value).toBe(0)
  })

  it('bumpFrameRevision() increments by 1', () => {
    const before = frameRevision.value
    bumpFrameRevision()
    expect(frameRevision.value).toBe(before + 1)
  })

  it('accumulates across multiple bumps', () => {
    const before = frameRevision.value
    bumpFrameRevision()
    bumpFrameRevision()
    bumpFrameRevision()
    expect(frameRevision.value).toBe(before + 3)
  })
})
