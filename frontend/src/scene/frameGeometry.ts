import { Vector3, Matrix4, Quaternion } from 'three'

type P = { x: number; y: number; z: number }

export interface Basis {
  origin: [number, number, number] // millimeters (same units as input)
  x: [number, number, number]
  y: [number, number, number]
  z: [number, number, number]
}

// Mirrors the Go backend frames.ComputeThreePoint: origin=P0, +X toward P1,
// +Z = X × (P0→P2), +Y = Z × X. Returns null when the points are collinear
// (either cross product ~0), matching the backend's degeneracy guard — so the
// scene shows NO provisional triad for a frame that can't be computed.
export function threePointBasis(p0: P, p1: P, p2: P): Basis | null {
  const o = new Vector3(p0.x, p0.y, p0.z)
  const x = new Vector3(p1.x, p1.y, p1.z).sub(o)
  const toP2 = new Vector3(p2.x, p2.y, p2.z).sub(o)
  if (x.lengthSq() === 0 || toP2.lengthSq() === 0) return null
  x.normalize()
  const z = new Vector3().crossVectors(x, toP2)
  if (z.lengthSq() < 1e-9) return null
  z.normalize()
  const y = new Vector3().crossVectors(z, x).normalize()
  return { origin: [o.x, o.y, o.z], x: [x.x, x.y, x.z], y: [y.x, y.y, y.z], z: [z.x, z.y, z.z] }
}

// Quaternion [x,y,z,w] whose columns are the basis vectors (local +X/+Y/+Z map
// to x/y/z in the parent) — this is what an AxesHelper needs so its drawn axes
// point along the basis. three.js makeBasis sets the matrix COLUMNS.
export function basisToQuaternion(b: Basis): [number, number, number, number] {
  const m = new Matrix4().makeBasis(
    new Vector3(...b.x), new Vector3(...b.y), new Vector3(...b.z),
  )
  return new Quaternion().setFromRotationMatrix(m).toArray() as [number, number, number, number]
}
