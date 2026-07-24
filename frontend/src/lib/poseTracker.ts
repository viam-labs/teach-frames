// Framework-free DoCommand payload builders + response parsers for the
// pose-tracker resource. Kept free of any Svelte/svelte-sdk imports so they
// can be unit tested in isolation (see Task 18).
//
// IMPORTANT: `PoseTrackerClient.doCommand` (from @viamrobotics/sdk) is typed
// as `doCommand(command: Struct, callOptions?: CallOptions): Promise<JsonValue>`
// — it takes a protobuf `Struct` (re-exported from `@bufbuild/protobuf`), NOT
// a plain object. The builders below intentionally keep returning plain
// objects (that's the surface the unit tests assert against); conversion to
// `Struct` happens only at the call boundary via `toCommand()`, e.g.:
//
//   mutation.mutate([toCommand(jogCartesian('x', 10))])
//
import { Struct, type JsonValue } from '@viamrobotics/sdk'

export type CartesianAxis = 'x' | 'y' | 'z' | 'roll' | 'pitch' | 'yaw'

export const jogCartesian = (axis: CartesianAxis, step: number, frame?: string) =>
  frame === undefined ? { jog_cartesian: { axis, step } } : { jog_cartesian: { axis, step, frame } }
export const jogJoint = (joint: number, step: number) => ({ jog_joint: { joint, step } })
export const stopArm = () => ({ stop_arm: {} })
export const getArmState = () => ({ get_arm_state: {} })
export const capturePoint = () => ({ capture_point: {} })
export const getBuffer = () => ({ get_buffer: {} })
export const clearBuffer = () => ({ clear_buffer: {} })

// The 3point method needs 3 captured points, point needs 1, and tcp_snapshot
// needs 1 (it takes the raw captured pose as-is, without deriving axes).
export type FrameMethod = 'point' | '3point' | 'tcp_snapshot'

export const defineFrame = (name: string, method: FrameMethod) => ({ define_frame: { name, method } })
export const listFrames = () => ({ list_frames: {} })
export const deleteFrame = (name: string) => ({ delete_frame: { name } })
export const clearFrames = () => ({ clear_frames: {} })
export const captureTcpPoint = () => ({ capture_tcp_point: {} })
export const getTcpBuffer = () => ({ get_tcp_buffer: {} })
export const clearTcpBuffer = () => ({ clear_tcp_buffer: {} })
export const teachTcpPosition = () => ({ teach_tcp_position: {} })
export const teachTcpOrientation = (o_x: number, o_y: number, o_z: number, theta: number) => ({
  teach_tcp_orientation: { o_x, o_y, o_z, theta },
})
export const moveToJoints = (positions: number[]) => ({ move_to_joints: { positions } })
export const moveToPose = (pose: PoseMap) => ({ move_to_pose: { pose } })

// Converts a plain-object DoCommand payload (as produced by the builders
// above) into the `Struct` that `PoseTrackerClient.doCommand` requires.
export function toCommand(payload: Record<string, unknown>): Struct {
  return Struct.fromJson(payload as JsonValue)
}

// `createResourceMutation`/`createResourceQuery` expect the exact argument
// tuple of `doCommand` (`[command: Struct, callOptions?: CallOptions]`) —
// a bare `[toCommand(payload)]` array literal infers as `Struct[]`, which
// TypeScript won't accept in that position. This helper pins the tuple type
// so call sites can just do `mutateAsync(toCommandArgs(jogCartesian(...)))`.
export function toCommandArgs(payload: Record<string, unknown>): [Struct] {
  return [toCommand(payload)]
}

export interface PoseMap {
  x: number
  y: number
  z: number
  o_x: number
  o_y: number
  o_z: number
  theta: number
}

export interface ArmState {
  pose: PoseMap
  joints: number[]
}

export function parseArmState(resp: Record<string, unknown>): ArmState {
  return { pose: resp.pose as PoseMap, joints: (resp.joints as number[]) ?? [] }
}

// Response shapes for the remaining DoCommand verbs (see
// models/posetracker/docommand.go for the authoritative Go-side shapes).

export interface CaptureResponse {
  index: number
  buffer_len: number
  pose: PoseMap
}

export interface BufferResponse {
  points: PoseMap[]
}

export interface ClearResponse {
  cleared: number
}

export interface DefineFrameResponse {
  name: string
  committed: boolean
  replaced: boolean
  pose: PoseMap
}

export interface FramesResponse {
  frames: Record<string, PoseMap>
}

export interface DeleteFrameResponse {
  deleted: boolean
}

export interface ClearFramesResponse {
  deleted: number
}

export interface TeachTcpPositionResponse {
  committed: boolean
  offset: { x: number; y: number; z: number }
  residual_rms: number
}

export interface TeachTcpOrientationResponse {
  committed: boolean
  orientation: { o_x: number; o_y: number; o_z: number; theta: number }
}

export interface MoveToJointsResponse {
  joints: number[]
  moved: boolean
}

export interface MoveToPoseResponse {
  pose: PoseMap
  moved: boolean
}

// Hand-eye calibration DoCommands (see models/posetracker/handeye.go for the
// authoritative Go-side shapes).

export const handeyeSnapshot = () => ({ handeye_snapshot: {} })
export const captureHandeyePoint = (u: number, v: number) => ({ capture_handeye_point: { u, v } })
export const getHandeyeBuffer = () => ({ get_handeye_buffer: {} })
export const clearHandeyeBuffer = () => ({ clear_handeye_buffer: {} })
export const solveHandeye = () => ({ solve_handeye: {} })

// Maps a click at (displayX, displayY) inside the rendered camera image of size
// (clientW, clientH) to the native source pixel (naturalW, naturalH). The image
// is styled `object-fit: fill` (see HandeyeWizard.svelte), so this is a straight
// ratio with no letterbox offset — if that ever changes to `object-fit: contain`,
// this helper must add the letterbox offsets to match.
// Result is clamped to valid pixel indices [0, natural-1].
export function displayToNativePixel(
  displayX: number, displayY: number,
  clientW: number, clientH: number,
  naturalW: number, naturalH: number,
): { u: number; v: number } {
  const u = Math.round((displayX / clientW) * naturalW)
  const v = Math.round((displayY / clientH) * naturalH)
  return {
    u: Math.max(0, Math.min(naturalW - 1, u)),
    v: Math.max(0, Math.min(naturalH - 1, v)),
  }
}

export interface HandEyePair {
  world: { x: number; y: number; z: number }
  camera: { x: number; y: number; z: number }
}

export interface HandEyeSnapshotResponse {
  image: string
  width: number
  height: number
}

export interface CaptureHandEyeResponse {
  index: number
  buffer_len: number
  world: { x: number; y: number; z: number }
  camera: { x: number; y: number; z: number }
}

export interface HandEyeBufferResponse {
  points: HandEyePair[]
}

export interface SolveHandEyeResponse {
  committed: boolean
  pose: PoseMap
  residual_rms: number
  parent: string
  orientation: { o_x: number; o_y: number; o_z: number; theta: number }
}

// Eye-in-hand hand-eye calibration DoCommands. Unlike eye-to-hand, the
// operator touches ONE target then re-views it from multiple flange poses
// ("jog, snapshot, click the SAME point"), so this is a separate protocol,
// not just a mount flag on the eye-to-hand one.

export const captureHandeyeTarget = () => ({ capture_handeye_target: {} })
export const captureHandeyeView = (u: number, v: number) => ({ capture_handeye_view: { u, v } })
export const getHandeyeMode = () => ({ get_handeye_mode: {} })

export type CameraMount = 'eye_to_hand' | 'eye_in_hand'

export interface HandEyeModeResponse {
  camera_mount: CameraMount
  camera_configured: boolean
  arm_configured: boolean
}

export interface CaptureHandEyeTargetResponse {
  target: { x: number; y: number; z: number }
}

// `flange` is a full pose, not a vector: the orientation is what the solve
// depends on.
export interface EyeInHandObservation {
  target: { x: number; y: number; z: number }
  flange: PoseMap
  camera: { x: number; y: number; z: number }
}

export interface CaptureHandEyeViewResponse extends EyeInHandObservation {
  index: number
  buffer_len: number
}

export interface EyeInHandBufferResponse {
  points: EyeInHandObservation[]
}
