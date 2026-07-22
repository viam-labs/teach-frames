import { describe, it, expect, test } from 'vitest'
import {
  jogCartesian,
  jogJoint,
  stopArm,
  getArmState,
  capturePoint,
  getBuffer,
  clearBuffer,
  clearTcpBuffer,
  defineFrame,
  listFrames,
  deleteFrame,
  clearFrames,
  captureTcpPoint,
  getTcpBuffer,
  teachTcpPosition,
  teachTcpOrientation,
  parseArmState,
  toCommand,
  toCommandArgs,
  handeyeSnapshot,
  captureHandeyePoint,
  getHandeyeBuffer,
  clearHandeyeBuffer,
  solveHandeye,
  displayToNativePixel,
  captureHandeyeTarget,
  captureHandeyeView,
  getHandeyeMode,
  moveToJoints,
  moveToPose,
} from './poseTracker'

describe('DoCommand payload builders', () => {
  it('jogCartesian emits the axis/step verb map', () => {
    expect(jogCartesian('x', -1)).toEqual({ jog_cartesian: { axis: 'x', step: -1 } })
  })

  it('jogCartesian keeps negative step sign for angular axes', () => {
    expect(jogCartesian('yaw', -5)).toEqual({ jog_cartesian: { axis: 'yaw', step: -5 } })
  })

  it('jogCartesian omits frame when not given (back-compat)', () => {
    expect(jogCartesian('x', 5)).toEqual({ jog_cartesian: { axis: 'x', step: 5 } })
  })

  it('jogCartesian includes frame when given', () => {
    expect(jogCartesian('x', 5, 'table')).toEqual({ jog_cartesian: { axis: 'x', step: 5, frame: 'table' } })
  })

  it('jogJoint emits the joint/step verb map', () => {
    expect(jogJoint(2, 5)).toEqual({ jog_joint: { joint: 2, step: 5 } })
  })

  it('stopArm emits an empty verb map', () => {
    expect(stopArm()).toEqual({ stop_arm: {} })
  })

  it('getArmState emits an empty verb map', () => {
    expect(getArmState()).toEqual({ get_arm_state: {} })
  })

  it('capturePoint emits an empty verb map', () => {
    expect(capturePoint()).toEqual({ capture_point: {} })
  })

  it('getBuffer emits an empty verb map', () => {
    expect(getBuffer()).toEqual({ get_buffer: {} })
  })

  it('clearBuffer emits an empty verb map', () => {
    expect(clearBuffer()).toEqual({ clear_buffer: {} })
  })

  it('clearTcpBuffer emits an empty verb map', () => {
    expect(clearTcpBuffer()).toEqual({ clear_tcp_buffer: {} })
  })

  it('defineFrame emits the name/method verb map', () => {
    expect(defineFrame('fixture_a', '3point')).toEqual({
      define_frame: { name: 'fixture_a', method: '3point' },
    })
  })

  it('listFrames emits an empty verb map', () => {
    expect(listFrames()).toEqual({ list_frames: {} })
  })

  it('deleteFrame emits the name verb map', () => {
    expect(deleteFrame('f')).toEqual({ delete_frame: { name: 'f' } })
  })

  it('clearFrames emits an empty verb map', () => {
    expect(clearFrames()).toEqual({ clear_frames: {} })
  })

  it('captureTcpPoint emits an empty verb map', () => {
    expect(captureTcpPoint()).toEqual({ capture_tcp_point: {} })
  })

  it('getTcpBuffer emits an empty verb map', () => {
    expect(getTcpBuffer()).toEqual({ get_tcp_buffer: {} })
  })

  it('teachTcpPosition emits an empty verb map', () => {
    expect(teachTcpPosition()).toEqual({ teach_tcp_position: {} })
  })

  it('teachTcpOrientation emits the o_x/o_y/o_z/theta verb map', () => {
    expect(teachTcpOrientation(0, 0, 1, 90)).toEqual({
      teach_tcp_orientation: { o_x: 0, o_y: 0, o_z: 1, theta: 90 },
    })
  })

  it('moveToJoints builds the command', () => {
    expect(moveToJoints([90, 0, 45])).toEqual({ move_to_joints: { positions: [90, 0, 45] } })
  })

  it('moveToPose builds the command', () => {
    const p = { x: 1, y: 2, z: 3, o_x: 0, o_y: 0, o_z: 1, theta: 0 }
    expect(moveToPose(p)).toEqual({ move_to_pose: { pose: p } })
  })
})

describe('Hand-eye DoCommand payload builders', () => {
  it('handeyeSnapshot emits an empty verb map', () => {
    expect(handeyeSnapshot()).toEqual({ handeye_snapshot: {} })
  })

  it('captureHandeyePoint emits the u/v verb map', () => {
    expect(captureHandeyePoint(12, 34)).toEqual({ capture_handeye_point: { u: 12, v: 34 } })
  })

  it('getHandeyeBuffer emits an empty verb map', () => {
    expect(getHandeyeBuffer()).toEqual({ get_handeye_buffer: {} })
  })

  it('clearHandeyeBuffer emits an empty verb map', () => {
    expect(clearHandeyeBuffer()).toEqual({ clear_handeye_buffer: {} })
  })

  it('solveHandeye emits an empty verb map', () => {
    expect(solveHandeye()).toEqual({ solve_handeye: {} })
  })
})

describe('eye-in-hand commands', () => {
  it('builds capture_handeye_target', () => {
    expect(captureHandeyeTarget()).toEqual({ capture_handeye_target: {} })
  })
  it('builds capture_handeye_view with pixel coords', () => {
    expect(captureHandeyeView(412, 233)).toEqual({ capture_handeye_view: { u: 412, v: 233 } })
  })
  it('builds get_handeye_mode', () => {
    expect(getHandeyeMode()).toEqual({ get_handeye_mode: {} })
  })
})

describe('displayToNativePixel', () => {
  test('maps a click in the rendered image box to native source pixels', () => {
    // A 640x480 image rendered into a 320x240 box: a click at (160,120) → (320,240).
    expect(displayToNativePixel(160, 120, 320, 240, 640, 480)).toEqual({ u: 320, v: 240 })
  })

  test('clamps to the upper bound', () => {
    expect(displayToNativePixel(400, 300, 320, 240, 640, 480)).toEqual({ u: 639, v: 479 })
  })

  test('clamps to the lower bound', () => {
    expect(displayToNativePixel(-5, -5, 320, 240, 640, 480)).toEqual({ u: 0, v: 0 })
  })
})

describe('parseArmState', () => {
  it('parses a full arm state response', () => {
    const pose = { x: 1, y: 2, z: 3, o_x: 0, o_y: 0, o_z: 1, theta: 0 }
    expect(parseArmState({ pose, joints: [0, 90] })).toEqual({ pose, joints: [0, 90] })
  })

  it('defaults joints to an empty array when missing', () => {
    const pose = { x: 1, y: 2, z: 3, o_x: 0, o_y: 0, o_z: 1, theta: 0 }
    const result = parseArmState({ pose })
    expect(result.joints).toEqual([])
    expect(Array.isArray(result.joints)).toBe(true)
    expect(result.pose).toEqual(pose)
  })
})

describe('toCommand / toCommandArgs', () => {
  it('toCommand converts a payload to a Struct that round-trips back to JSON', () => {
    const payload = capturePoint()
    const struct = toCommand(payload)
    expect(struct).toBeDefined()
    expect(struct.toJson()).toEqual(payload)
  })

  it('toCommand round-trips a payload with nested fields', () => {
    const payload = jogCartesian('x', -1)
    const struct = toCommand(payload)
    expect(struct.toJson()).toEqual(payload)
  })

  it('toCommandArgs returns a 1-tuple containing the Struct', () => {
    const args = toCommandArgs(capturePoint())
    expect(args).toHaveLength(1)
    expect(args[0]).toBeDefined()
  })
})
