# Teach Pendant UI — Design

Date: 2026-07-10
Status: Approved (brainstorm)

## Summary

Ship a browser-based **teach pendant UI** as a Viam Application bundled in the
`teach-frames` module. It is a full pendant: the operator jogs the arm and runs
the complete teaching workflow (capture points, define frames, TCP teaching)
from a single web surface. v1 is numeric-only (pose/joint readouts, tables) —
no in-app 3D, no camera. Jog logic lives in the Go module as new DoCommands so
the UI stays thin.

## Decisions (from brainstorm)

| Question | Decision |
|---|---|
| Scope | Full pendant — jogging **and** teach workflow in one UI. |
| Visualization (v1) | Numeric only (pose/joint readouts, buffer + frame tables). Camera / 3D deferred. |
| Jog control | New jog DoCommands on the pose-tracker; UI stays thin. |
| Frontend stack | Svelte 5 + Vite + TS, using `@viamrobotics/svelte-sdk`. |
| Jog panel must-haves | Discrete step sizes, Cartesian + joint modes, always-visible Stop. Speed limit deferred. |
| Jog target arm | Reuse the existing optional `arm` dependency as the jog target. |
| Cartesian frame convention | World-frame translation (X/Y/Z along base axes) + tool-frame rotation (roll/pitch/yaw about the TCP). |
| Packaging / connection | Bundled Viam Application, `type: single_machine`. One connection path in prod and dev — `viam module local-app-testing` proxies the same machine-credential injection. |

## Architecture & repo layout

```
teach-frames/
  frontend/                 # new: Svelte 5 + Vite + TS app
    src/
      lib/
        poseTracker.ts      # DoCommand payload builders + typed response parsers
      panels/
        JogPanel.svelte     # step-size selector, cartesian/joint modes, arrows, Stop
        CapturePanel.svelte # capture_point / buffer list / clear
        FramePanel.svelte   # define_frame (method picker), list, delete, clear
        TcpPanel.svelte     # capture_tcp_point, teach_tcp_position/orientation
        StatusBar.svelte    # live TCP pose + joints + connection state + Stop
      App.svelte            # ViamProvider wrapper + layout
      main.ts
    index.html
    vite.config.ts
    package.json
  ... existing Go module (new DoCommands + Makefile targets)
```

- **Build integration:** `make frontend` runs `npm ci && npm run build` in
  `frontend/`, emitting `frontend/dist/`. `make module` depends on `frontend` so
  the tarball carries the built SPA. `meta.json` gains an `applications` entry
  (`type: single_machine`, `entrypoint: frontend/dist/index.html`).
- **No new Viam resource** — the app talks to the existing `pose-tracker`
  component (DoCommands) and the arm (via the new `stop_arm` / `get_arm_state`
  DoCommands), no direct arm wiring required in the UI.

## Go changes — new jog DoCommands

All on the **pose-tracker** component, reusing the existing optional `arm`
dependency as the jog target. Each returns the same "arm dependency not
configured" error style as the TCP commands when `arm` is unset.

| Command key | Payload | Effect | Response |
|---|---|---|---|
| `jog_cartesian` | `{"axis": "x"\|"y"\|"z"\|"roll"\|"pitch"\|"yaw", "step": <mm-or-deg>}` | Read `arm.EndPosition`, apply signed delta, `MoveToPosition`. | `pose`, `moved` |
| `jog_joint` | `{"joint": <index>, "step": <deg>}` | Read `JointPositions`, add delta to joint index, `MoveToJointPositions`. | `joints` |
| `stop_arm` | `{}` | `arm.Stop()`. | `stopped` |
| `get_arm_state` | `{}` | Current `EndPosition` + `JointPositions` in one call for the poll loop. | `pose`, `joints` |

- **Sign convention:** `step` carries the sign; UI arrows send `+step` / `−step`.
- **Step vs. velocity:** discrete steps only in v1 (no speed param).

### Orientation math (`jog_cartesian`)

Poses carry orientation as an orientation vector (`OX, OY, OZ, Theta`);
`MoveToPosition` takes a `spatialmath.Pose`. **Do not add degrees to RPY and
rebuild** — Euler addition does not compose rotations correctly. Instead:

1. Read current pose (`Point()` + `Orientation()`).
2. Build the rotational delta as an `*spatialmath.EulerAngles` (single axis).
3. Compose with quaternions via `spatialmath.Compose(...)`, then hand the
   resulting `Pose` to `MoveToPosition`. OV ↔ quaternion conversion happens
   internally — we never touch `OX/OY/OZ/Theta` manually.

Composition order sets the frame:
- **Tool-frame rotation:** `Compose(currentPose, deltaPose)` (delta applied
  after, in the tool frame). Used for roll/pitch/yaw.
- **World-frame translation:** add the delta to the world point (or
  `Compose(deltaPose, currentPose)` for a world rotation — not used in v1).

v1 convention: **world-frame translation + tool-frame rotation.**

## Frontend (grounded on `@viamrobotics/svelte-sdk@1.2.3`)

Deps: `@viamrobotics/svelte-sdk`, peer `@viamrobotics/sdk` (≥0.51), Svelte 5.
SDK is built on `@tanstack/svelte-query` + `runed`; all state is
query/mutation-driven — no hand-rolled stores.

**Connection**
- Wrap the app in `ViamProvider` with `dialConfigs: Record<partID, DialConf>`
  built from the machine identity the Viam Application host injects (same path
  in production and under `viam module local-app-testing`).
- `useConnectionStatus` / `useMachineStatus` drive the connection indicator;
  `useResourceNames` discovers the arm resource.
- **Open detail for the plan:** confirm the exact helper for reading the hosted
  machine's part ID + credentials into `DialConf` against the Viam Application
  docs. `ViamAppProvider(serviceHost, credentials)` is the app/data-plane
  client; `ViamProvider` is the machine-plane one we need.

**Resource clients**
- `createResourceClient(PoseTrackerClient, () => partID, () => 'pose-tracker')`.
- `createResourceClient(ArmClient, () => partID, () => armName)`.

**DoCommands**
- Each action is `createResourceMutation(poseTracker, 'doCommand')` →
  TanStack mutation (`mutateAsync`, `isPending`, `error`). `isPending` disables
  buttons; `error` renders inline.

**Live readouts**
- `createResourceQuery(poseTracker, 'doCommand', () => [{ get_arm_state: {} }])`
  plus `usePolling(query.queryKey, () => isMoving ? false : 500)` — polling
  pauses while any jog/move mutation is in flight.

**Panels**
- **StatusBar** — live TCP pose (x/y/z + OV), joint positions, connection
  indicator, always-visible **Stop** (`stop_arm`, enabled even while a jog is
  pending).
- **JogPanel** — mode toggle (Cartesian ↔ Joint); step-size presets
  (0.1 / 1 / 10 mm, 1° / 5°); Cartesian 6-axis pad (X± Y± Z± roll± pitch±
  yaw±); Joint mode ± per joint (count from `get_arm_state`). Each button = one
  `jog_*` DoCommand with signed step.
- **CapturePanel** — capture button, live buffer list (index + pose), clear.
- **FramePanel** — name field + method picker (`3point` / `point` /
  `tcp_snapshot`) with buffer-requirement hints; committed-frames list with
  per-row delete; clear-all.
- **TcpPanel** — TCP capture + separate TCP buffer view; `teach_tcp_position`
  (shows residual RMS) and `teach_tcp_orientation` (OV inputs); disabled with an
  explanatory note when no `arm` is configured.

**Layout:** single responsive page — StatusBar pinned top, JogPanel the dominant
region, capture→frame→TCP panels alongside. Tablet-friendly hit targets.
`CameraStream`/`CameraImage` exist in the SDK for the deferred camera view.

## Error handling & safety

- Discrete steps only — a dropped connection cannot leave the arm moving.
- **Stop** always visible and enabled even while a jog mutation is pending;
  jog buttons disable while any move `isPending` to prevent overlapping moves.
- Go-side jog handlers validate `axis`/`joint`/`step` and return descriptive
  errors mirroring the TCP-command style. `MoveToPosition` /
  `MoveToJointPositions` failures (unreachable, singularity, collision)
  propagate as the DoCommand error and render inline — the arm's own limits
  remain the safety authority.
- `useConnectionStatus` drives a banner and disables mutations when
  disconnected; per-action `error` keeps one failure from taking down the page.
- `define_frame` / TCP teach already return `committed: false` on persist
  failure — panels surface "computed but not persisted" distinctly from a
  compute error.

## Testing

- **Go:** unit-test jog handlers with a fake arm — compose-order correctness
  (world-translation / tool-rotation), joint-index bounds, sign handling,
  arm-not-configured error. Follows existing table-test style.
- **Frontend:** Vitest + @testing-library/svelte for payload builders and panel
  logic against a mocked mutation hook — assert correct DoCommand payloads and
  pending/error rendering. Light; no full e2e in v1.
- **Manual:** `viam module local-app-testing` against a real or fake-arm machine
  to validate the end-to-end jog → capture → define loop.

## Deferred (post-v1)

- In-app 3D scene (arm + frame triads + captured points).
- Camera stream panel (`CameraStream`).
- Jog speed / velocity limiting.
- Tool-frame translation jog mode.
