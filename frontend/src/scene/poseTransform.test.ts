import { describe, it, expect } from 'vitest'
import { poseToSceneTransform } from './poseTransform'
import type { PoseMap } from '../lib/poseTracker'

describe('poseToSceneTransform', () => {
  it('scales position from millimeters to meters', () => {
    const pose: PoseMap = { x: 1000, y: 2000, z: -500, o_x: 0, o_y: 0, o_z: 1, theta: 0 }
    const { position } = poseToSceneTransform(pose)
    expect(position).toEqual([1, 2, -0.5])
  })

  it('maps identity orientation (o_x:0, o_y:0, o_z:1, theta:0) to an identity-ish quaternion', () => {
    const pose: PoseMap = { x: 0, y: 0, z: 0, o_x: 0, o_y: 0, o_z: 1, theta: 0 }
    const { quaternion } = poseToSceneTransform(pose)
    expect(quaternion[0]).toBeCloseTo(0)
    expect(quaternion[1]).toBeCloseTo(0)
    expect(quaternion[2]).toBeCloseTo(0)
    expect(quaternion[3]).toBeCloseTo(1)
  })

  it('returns a normalized, finite quaternion for a non-trivial orientation', () => {
    const pose: PoseMap = { x: 10, y: 20, z: 30, o_x: 1, o_y: 0, o_z: 0, theta: 90 }
    const { quaternion } = poseToSceneTransform(pose)

    for (const component of quaternion) {
      expect(Number.isFinite(component)).toBe(true)
    }

    const length = Math.sqrt(quaternion.reduce((sum, c) => sum + c * c, 0))
    expect(length).toBeCloseTo(1)
  })

  // Pins the exact OrientationVector convention. o_x:1,o_y:0,o_z:0,theta:90
  // is chosen specifically because it diverges from the WRONG
  // `Quaternion.setFromAxisAngle([o_x,o_y,o_z], degToRad(theta))` convention
  // ([~0.7071, 0, 0, ~0.7071]) — the identity-orientation and
  // normalized/finite cases above don't discriminate between the two, so
  // without this case a regression to axis-angle would pass the suite.
  // Golden value captured empirically from the verified-correct
  // OrientationVector implementation (see poseTransform.ts), not
  // hand-derived.
  it('matches the OrientationVector convention exactly (not axis-angle)', () => {
    const pose: PoseMap = { x: 0, y: 0, z: 0, o_x: 1, o_y: 0, o_z: 0, theta: 90 }
    const { quaternion } = poseToSceneTransform(pose)
    expect(quaternion[0]).toBeCloseTo(0.5)
    expect(quaternion[1]).toBeCloseTo(0.5)
    expect(quaternion[2]).toBeCloseTo(0.5)
    expect(quaternion[3]).toBeCloseTo(0.5)
  })
})
