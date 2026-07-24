// Pure, framework-free conversion from a pose-tracker PoseMap (millimeters +
// Viam OrientationVector, theta in degrees) to a Threlte-ready scene
// transform (meters + [x,y,z,w] quaternion tuple). Kept dependency-light
// (only three.js + motion-tools' OrientationVector, no Svelte) so it can be
// unit tested directly.
//
// This mirrors the Visualizer's own poseToMatrix
// (../../visualization/src/lib/transform.ts:134-139) — position scaled
// mm -> m, orientation converted via Viam's OrientationVector (NOT
// axis-angle; `Quaternion.setFromAxisAngle` is the wrong convention here).
import { OrientationVector } from '@viamrobotics/motion-tools/lib'
import { Quaternion, MathUtils } from 'three'
import type { PoseMap } from '../lib/poseTracker'

export interface SceneTransform {
  position: [number, number, number]
  quaternion: [number, number, number, number]
}

export function poseToSceneTransform(pose: PoseMap): SceneTransform {
  const position: [number, number, number] = [pose.x * 0.001, pose.y * 0.001, pose.z * 0.001]

  const ov = new OrientationVector()
  ov.set(pose.o_x, pose.o_y, pose.o_z, MathUtils.degToRad(pose.theta))
  const q = new Quaternion()
  ov.toQuaternion(q)

  return { position, quaternion: q.toArray() as [number, number, number, number] }
}
