import { describe, it, expect } from 'vitest'
import { threePointBasis, basisToQuaternion } from './frameGeometry'

describe('threePointBasis', () => {
  it('computes origin=P0, +X toward P1, +Z = X × (P0→P2), +Y = Z × X', () => {
    const basis = threePointBasis({ x: 0, y: 0, z: 0 }, { x: 2, y: 0, z: 0 }, { x: 0, y: 3, z: 0 })
    expect(basis).not.toBeNull()
    expect(basis!.origin).toEqual([0, 0, 0])

    expect(basis!.x[0]).toBeCloseTo(1)
    expect(basis!.x[1]).toBeCloseTo(0)
    expect(basis!.x[2]).toBeCloseTo(0)

    expect(basis!.y[0]).toBeCloseTo(0)
    expect(basis!.y[1]).toBeCloseTo(1)
    expect(basis!.y[2]).toBeCloseTo(0)

    expect(basis!.z[0]).toBeCloseTo(0)
    expect(basis!.z[1]).toBeCloseTo(0)
    expect(basis!.z[2]).toBeCloseTo(1)
  })

  it('returns null for collinear points', () => {
    const basis = threePointBasis({ x: 0, y: 0, z: 0 }, { x: 1, y: 0, z: 0 }, { x: 2, y: 0, z: 0 })
    expect(basis).toBeNull()
  })

  it('returns null when P0 and P1 coincide', () => {
    const basis = threePointBasis({ x: 0, y: 0, z: 0 }, { x: 0, y: 0, z: 0 }, { x: 0, y: 1, z: 0 })
    expect(basis).toBeNull()
  })
})

describe('basisToQuaternion', () => {
  it('maps the identity basis to the identity quaternion', () => {
    const q = basisToQuaternion({
      origin: [0, 0, 0],
      x: [1, 0, 0],
      y: [0, 1, 0],
      z: [0, 0, 1],
    })
    expect(q[0]).toBeCloseTo(0)
    expect(q[1]).toBeCloseTo(0)
    expect(q[2]).toBeCloseTo(0)
    expect(q[3]).toBeCloseTo(1)
  })
})
