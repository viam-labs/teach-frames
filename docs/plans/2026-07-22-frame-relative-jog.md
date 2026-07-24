# Frame-relative Jogging Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Jog the TCP along and about the axes of a chosen reference frame (World / Tool / a taught frame) via an optional `frame` param on `jog_cartesian` and a Reference picker in the Jog panel.

**Architecture:** Backend resolves a basis rotation `R` from the `frame` param (identity for world, current orientation for tool, the stored frame's orientation for a taught frame) and applies the jog step *in that basis* before the existing `arm.MoveToPosition`. Frontend adds a Reference selector to the Cartesian jog. Default `world` reproduces today's translation exactly.

**Tech Stack:** Go (RDK `spatialmath`/`r3`), Svelte 5 runes, Vitest, `go test`.

**Design doc:** `docs/plans/2026-07-22-frame-relative-jog-design.md`

**Commands:** backend `go build ./... && go test ./models/posetracker/...`; frontend (in `frontend/`) `npm run check`/`test`/`build`.

**Backend precedent — `models/posetracker/docommand.go` `jogCartesian` (~L530):** reads `arm.EndPosition` (arm-base), branches on axis (x/y/z add `step` to the point; roll/pitch/yaw `spatialmath.Compose(cur, NewPoseFromOrientation(delta))` — tool-frame), `arm.MoveToPosition(next)`. Store lookup: `pt.store.GetFrame(name)` → `(*referenceframe.PoseInFrame, bool)`; `.Pose().Orientation()` is the frame's orientation. Tests: `TestJogCartesianTranslation`, `TestJogCartesianRotationToolFrame`, `TestJogCartesianAllAxes` (fake `inject.Arm`, seed `EndPositionFunc`, record `MoveToPositionFunc`).

---

## Task 1: Backend — `frame` param on `jog_cartesian` (TDD, Go)

**Files:**
- Modify: `models/posetracker/docommand.go`
- Modify: `models/posetracker/docommand_test.go`

**The basis rotation `R`** (a `spatialmath.Orientation`): resolve from the optional `frame` arg:
- absent or `"world"` → identity (`spatialmath.NewZeroOrientation()`).
- `"tool"` → `cur.Orientation()` (the live `EndPosition` orientation).
- otherwise → `pt.store.GetFrame(frame)`; if found, `pif.Pose().Orientation()`; if not found → error `fmt.Errorf("unknown jog frame %q", frame)`.

**Apply the step in `R`** (this is the crux — pin it with tests):
- **Translation** (x/y/z): let `axisVec := r3.Vector{X|Y|Z: step}` (the one component set). Rotate it by `R`:
  `worldVec := spatialmath.Compose(spatialmath.NewPoseFromOrientation(R), spatialmath.NewPoseFromPoint(axisVec)).Point()`.
  `next := spatialmath.NewPose(cur.Point().Add(worldVec), cur.Orientation())`.
  (For `R = identity`, `worldVec == axisVec` — identical to today's translation.)
- **Rotation** (roll/pitch/yaw): build the canonical `delta` as today (`&spatialmath.EulerAngles{Roll|Pitch|Yaw: rad}`). Express it in world about the `R` basis and left-apply, preserving the point:
  ```go
  Rpose := spatialmath.NewPoseFromOrientation(R)
  deltaPose := spatialmath.NewPoseFromOrientation(delta)
  // W = R ∘ delta ∘ R⁻¹  (the delta expressed about the R-frame axis, in world)
  wPose := spatialmath.Compose(spatialmath.Compose(Rpose, deltaPose), spatialmath.PoseInverse(Rpose))
  nextOrient := spatialmath.Compose(wPose, spatialmath.NewPoseFromOrientation(cur.Orientation())).Orientation()
  next := spatialmath.NewPose(cur.Point(), nextOrient)
  ```
  (For `R = cur.Orientation()` i.e. `"tool"`, this reduces to `cur ∘ delta` — identical to today's tool rotation. For `R = identity` i.e. `"world"`, it's `delta ∘ cur` — rotation about the base axes, the intended new default.)

**Step 1: Write failing tests** in `docommand_test.go`:
- `TestJogCartesianWorldFrameMatchesDefault`: send `jog_cartesian {axis:"x", step:5, frame:"world"}` and assert the recorded `MoveToPosition` pose EQUALS the no-frame case (regression guard — world == today).
- `TestJogCartesianToolFrameRotation`: `frame:"tool"`, a rotation axis, non-identity `EndPosition` orientation → assert the recorded pose equals the existing tool-frame result (same as `TestJogCartesianRotationToolFrame`).
- `TestJogCartesianTaughtFrameTranslation`: seed the store with a frame whose orientation is a known non-identity (e.g. 90° about Z, so frame-X = world-Y); `frame:"<name>"`, `axis:"x", step:10` → assert the recorded point moved along world-Y by 10 (i.e. the step rotated by the frame basis). Use a tolerance.
- `TestJogCartesianTaughtFrameRotation`: same seeded frame, a rotation axis → assert the pose rotated about the frame axis (non-symmetric fixture; assert the resulting orientation with a tolerance/known value).
- `TestJogCartesianUnknownFrame`: `frame:"nope"` → error; `MoveToPosition` NOT called.
- Keep the existing `jog_cartesian` tests passing unchanged.

Run: `go test ./models/posetracker/... -run 'JogCartesian'` → new ones FAIL.

**Step 2: Implement.** Parse the optional `frame` string from args (`args["frame"]`, absent → `""`/world; a non-string → error). Add a `basisFor(frame string, cur spatialmath.Pose) (spatialmath.Orientation, error)` helper. Rework the axis switch to use the translation/rotation forms above with `R`. Keep the arm-nil guard, arg parsing for axis/step, and the response shape (`{"pose": poseToMap(next), "moved": true}`). Update the `jog_cartesian` doc-comment line to mention the optional `frame` (world|tool|<taught frame>, default world; note the origin-base caveat briefly). Confirm imports (`spatialmath`, `r3`, `referenceframe`) are present.

**Step 3: Verify** — `go test ./models/posetracker/...` (all green, incl. new + unchanged existing), `go build ./...`, `go vet ./...`, `gofmt -l` clean.

**Step 4: Commit**
```bash
git add models/posetracker/docommand.go models/posetracker/docommand_test.go
git commit -m "feat(posetracker): frame-relative jog_cartesian (world|tool|taught-frame basis)"
```

---

## Task 2: Frontend — builder `frame` arg + Reference picker

**Files:**
- Modify: `frontend/src/lib/poseTracker.ts`
- Modify: `frontend/src/lib/poseTracker.test.ts`
- Modify: `frontend/src/panels/JogPanel.svelte`

**Step 1: Builder (TDD).** In `poseTracker.test.ts` add:
```ts
it('jogCartesian omits frame when not given (back-compat)', () => {
  expect(jogCartesian('x', 5)).toEqual({ jog_cartesian: { axis: 'x', step: 5 } })
})
it('jogCartesian includes frame when given', () => {
  expect(jogCartesian('x', 5, 'table')).toEqual({ jog_cartesian: { axis: 'x', step: 5, frame: 'table' } })
})
```
In `poseTracker.ts`, change `jogCartesian` to accept an optional `frame`:
```ts
export const jogCartesian = (axis: CartesianAxis, step: number, frame?: string) =>
  frame === undefined ? { jog_cartesian: { axis, step } } : { jog_cartesian: { axis, step, frame } }
```
Run the test → PASS. (Existing call sites pass no `frame` → unchanged command.)

**Step 2: Reference picker in `JogPanel.svelte`** (Cartesian mode only):
- Add a `list_frames` query: `const framesQ = createResourceQuery(pt, 'doCommand', () => toCommandArgs(listFrames()), () => ({}))` (import `listFrames`, `type FramesResponse`); `const taughtFrames = $derived(Object.keys((framesQ.data as FramesResponse | undefined)?.frames ?? {}))`.
- Add `let reference = $state('world')` and a References segmented control (reuse the mode-toggle recipe): buttons `World`, `Tool`, then one per `taughtFrames`. Render it inside the `{#if mode === 'cartesian'}` block, above the translation axes, labelled "Reference" (role="group").
- If `reference` is a taught-frame name that disappears from `taughtFrames` (deleted), reset to `'world'`: a small `$effect(() => { if (reference !== 'world' && reference !== 'tool' && !taughtFrames.includes(reference)) reference = 'world' })`.
- Pass the reference into the jog-hold builders: `use:jogHold={() => jogCartesian(axis, transStep, reference)}` and `use:jogHold={() => jogCartesian(axis, -transStep, reference)}` for translation, and the same for the roll/pitch/yaw rotation buttons with `rotStep`. (The `jogHold` action already re-reads the builder via its `update`, so a reference change mid-session is picked up.)
- Leave Joint mode, Move-to mode, the readout, STOP, and the hold loop untouched.

**Step 3: Verify** — `npm run check` (0 errors), `npm run test` (all green), `npm run build` (success).

**Step 4: Commit**
```bash
git add frontend/src/lib/poseTracker.ts frontend/src/lib/poseTracker.test.ts frontend/src/panels/JogPanel.svelte
git commit -m "feat(frontend): frame-relative jog reference picker (World/Tool/taught frame)"
```

---

## Task 3: Hardware verification (user-run)

1. **World / Tool:** confirm x/y/z and roll/pitch/yaw jog as before with World selected; Tool gives tool-relative translate + rotate.
2. **Taught frame:** define a frame at a deliberate angle; select it; jog +X → the TCP moves along that frame's X axis (not world X); rotation is about the frame's axes.
3. **Deleted frame:** delete the selected frame → the picker falls back to World, no error.
Record results; no code commit unless a bug surfaces (TDD the fix).

---

## Definition of done
- [ ] `go build/test/vet ./...` and `npm run check`/`test`/`build` green.
- [ ] `jog_cartesian` `frame` param: world == today (regression-tested), tool matches today's rotation, a taught frame rotates the step into its basis, unknown frame errors without moving.
- [ ] Reference picker (World/Tool/taught frames) in the Cartesian jog; deleted-frame fallback.
- [ ] Hardware: frame-relative jog moves along/about the frame axes; World/Tool unchanged. Roadmap memory updated (E2 done → E complete; F/G remain).
