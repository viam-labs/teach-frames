// Pure map from (method, captures) to the provisional frame triad drawn in the
// scene during the wizard's preview phase. The COMPUTED TRANSFORM is matched
// EXACTLY to the Go backend (frames/compute.go, models/posetracker/docommand.go):
//   3point       -> ComputeThreePoint basis (origin P0, +X->P1, P2 in +XY plane)
//   point        -> ComputePoint: origin only, IDENTITY orientation
//   tcp_snapshot -> the captured pose as-is (position + orientation)
// The count guards below require EXACTLY the method's required capture count
// (3 / 1 / 1) and return null otherwise; the backend itself tolerates `>=`
// and ignores extras, but the wizard store caps captures at
// `requiredCaptures` before transitioning to preview, so `>N` is unreachable
// here — and for a preview, an unexpected count should render nothing.
import type { FrameMethod, PoseMap } from '../lib/poseTracker'
import { threePointBasis, basisToQuaternion } from './frameGeometry'
import { poseToSceneTransform, type SceneTransform } from './poseTransform'

const MM_TO_M = 0.001

export function provisionalTriad(method: FrameMethod, captures: PoseMap[]): SceneTransform | null {
  if (method === '3point') {
    if (captures.length !== 3) return null
    const basis = threePointBasis(captures[0], captures[1], captures[2])
    if (!basis) return null
    return {
      position: [basis.origin[0] * MM_TO_M, basis.origin[1] * MM_TO_M, basis.origin[2] * MM_TO_M],
      quaternion: basisToQuaternion(basis),
    }
  }

  if (captures.length !== 1) return null
  const c = captures[0]

  if (method === 'point') {
    // ComputePoint: identity orientation at the captured origin.
    return { position: [c.x * MM_TO_M, c.y * MM_TO_M, c.z * MM_TO_M], quaternion: [0, 0, 0, 1] }
  }

  // tcp_snapshot: the captured TCP pose, verbatim.
  return poseToSceneTransform(c)
}
