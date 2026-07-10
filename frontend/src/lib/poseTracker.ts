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

export const jogCartesian = (axis: CartesianAxis, step: number) => ({ jog_cartesian: { axis, step } })
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
