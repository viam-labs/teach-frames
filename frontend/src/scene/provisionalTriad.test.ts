import { describe, it, expect } from 'vitest'
import { provisionalTriad } from './provisionalTriad'
import type { PoseMap } from '../lib/poseTracker'

const pose = (p: Partial<PoseMap>): PoseMap =>
  ({ x: 0, y: 0, z: 0, o_x: 0, o_y: 0, o_z: 1, theta: 0, ...p })

describe('provisionalTriad', () => {
  it('returns null before enough captures', () => {
    expect(provisionalTriad('3point', [])).toBeNull()
    expect(provisionalTriad('point', [])).toBeNull()
    expect(provisionalTriad('3point', [pose({}), pose({})])).toBeNull()
  })

  it('point: origin scaled to meters, identity orientation', () => {
    // Non-identity orientation on the fixture proves the `point` path
    // IGNORES pose orientation rather than incidentally passing because the
    // default fixture orientation happened to already be identity.
    const t = provisionalTriad('point', [pose({ x: 100, y: 200, z: 300, o_z: 1, theta: 180 })])!
    expect(t.position[0]).toBeCloseTo(0.1)
    expect(t.position[1]).toBeCloseTo(0.2)
    expect(t.position[2]).toBeCloseTo(0.3)
    expect(t.quaternion).toEqual([0, 0, 0, 1])
  })

  it('tcp_snapshot: uses the captured pose position AND orientation', () => {
    // o_z=1, theta=180 -> a 180deg rotation about Z (w component ~ 0).
    const t = provisionalTriad('tcp_snapshot', [pose({ x: 10, y: 0, z: 0, o_z: 1, theta: 180 })])!
    expect(t.position[0]).toBeCloseTo(0.01)
    expect(t.position[1]).toBeCloseTo(0)
    expect(t.position[2]).toBeCloseTo(0)
    expect(Math.abs(t.quaternion[3])).toBeLessThan(1e-6) // w ~ 0 for a 180deg turn
  })

  it('3point: origin at P0, non-identity basis (X toward P1)', () => {
    // NOTE: P2 must NOT lie in the world Z=0 plane when P0->P1 is exactly the
    // world +X axis — otherwise ComputeThreePoint's basis (Z = X x toP2, then
    // Y = Z x X) always lands back on the canonical world axes, making this
    // assertion untestable. P2 here is off-plane so the basis is genuinely
    // rotated relative to world identity.
    const t = provisionalTriad('3point', [
      pose({ x: 0, y: 0, z: 0 }),
      pose({ x: 1000, y: 0, z: 0 }), // +X
      pose({ x: 0, y: 500, z: 1000 }), // out of the XY plane -> non-trivial basis
    ])!
    expect(t.position).toEqual([0, 0, 0])
    expect(t.quaternion).not.toEqual([0, 0, 0, 1])
  })
})
