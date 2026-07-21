# Upstream patch context: motion-tools world_state_store goes stale across an AlwaysRebuild reconfigure

**Date:** 2026-07-20
**Target project:** `@viamrobotics/motion-tools` (the `visualization` / motion-tools package that ships `<Visualizer>`), with a contributing factor in `@viamrobotics/svelte-sdk`.
**Status:** context capture for a *separate* upstream PR. In `teach-frames` we ship a client-side workaround (render taught frames ourselves from `list_frames`, decoupled from the WSS stream — see the Bug-1 fix). This doc is what a person needs to fix it properly upstream.

## Symptom (as seen in a downstream app)

An app embeds `<Visualizer>` and relies on a `world_state_store` service to render live entities (in our case, taught coordinate-frame triads). After the **first** config-persisting reconfigure that happens *after page load*, the Visualizer stops receiving **all** live transform deltas (ADDED / REMOVED / UPDATED) for the rest of the session. Concretely: a deleted entity's triad never disappears, and newly added entities never appear, until a full page reload. This reproduces whether the config change comes from a DoCommand the app issued or from an unrelated direct edit of the machine config.

## Root cause

The frontend never learns that a `world_state_store` **resource instance** was rebuilt, because every layer of the client plumbing is keyed by **resource name + partID only**, with no notion of a server-side resource "generation"/instance identity. An RDK `AlwaysRebuild` resource (which the teach-frames mirror is, and which many resources are) is replaced by a *new instance under the same resource name* on reconfigure. Nothing on the client changes identity, so nothing re-subscribes.

Evidence chain (paths under `frontend/node_modules/`):

1. **`@viamrobotics/motion-tools/dist/hooks/useWorldState.svelte.js`**
   - `provideWorldStates` calls `createWorldState(client)` from an `$effect` keyed on `clients`, which is `$derived` from `useResourceNames(...).current`.
   - The live stream is opened in `createWorldState` by `consumeChanges` (subscribed at ~lines 277-285), an `$effect` keyed on `client.current`.
   - `consumeChanges` (~lines 257-276) drains `activeClient.streamTransformChanges(...)` via `for await`. When the stream ends it simply completes (~lines 262-269) — **no error, no reconnect**.
   - REMOVED/ADDED/UPDATED are themselves handled correctly when they *arrive*: `applyEvents` → `destroyEntity(uuid)` for REMOVED (~lines 129-138, 207-219). The bug is strictly upstream of event handling — the events never arrive after the first rebuild.
   - The initial-snapshot reconciliation (`ListUUIDs` + per-uuid `GetTransform`, ~lines 201-250) is gated by an `initialized` flag that is **never reset** (`if (initialized) return;`, ~lines 197, 238-239). So even a re-run would not re-diff the rendered set against a fresh snapshot.

2. **`@viamrobotics/svelte-sdk/dist/hooks/resource-names.svelte.js`**
   - It *does* poll `getMachineStatus` and debounce-refetch resource names on config-revision change (~lines 61-69) — the reconfigure *is* detectable here.
   - But `.current` is memoized by value-equality (`areResourceNamesEqual`) and returns the **same stable reference** when the name list is unchanged (~lines 74-80) — which it always is for a same-named rebuild. So `provideWorldStates`'s `$effect` never re-fires.

3. **`@viamrobotics/svelte-sdk/dist/hooks/create-resource-client.svelte.js`**
   - The client handle is `$derived.by` on `robotClient.current` / `connectionStatus.current` / resource name only (~lines 6-17) — none flip on an in-place, same-named resource rebuild.

4. **Backend confirmation (our resource, but the pattern is general):** `models/worldstatestore/model.go` — the mirror is `resource.AlwaysRebuild`; `Close()` cancels the poll goroutine and thus terminates the old gRPC stream on reconfigure (~lines 162-170). That stream-close is exactly what the client's `for await` sees as a clean completion with nothing to reconnect to.

### Compounding issue (must be fixed together, not separately)

Even if the client *did* re-subscribe to the new instance, `StreamTransformChanges` seeds its diff baseline (`prev`) from `pt.Poses()` on the **new** resource and deliberately does **not** emit that seed (comment: "the client already has the initial set via ListUUIDs/GetTransform"). So a bare reconnect would emit nothing for a deletion that happened during the rebuild. A correct fix must **re-subscribe AND re-snapshot** (reconcile the rendered entity set against a fresh `ListUUIDs`/`GetTransform`), destroying entities absent from the fresh snapshot.

## Precedent already in the codebase

`@viamrobotics/motion-tools/dist/hooks/useFrames.svelte.js` (~lines 45, 95-123) already refetches the *static* frame-system config when `machineStatus.current?.config?.revision` changes. `useWorldState` has no analogous revision-gated re-subscribe/re-snapshot. The fix should mirror that pattern.

## Proposed upstream fix

In `useWorldState` (and/or `provideWorldStates`), add a reactive dependency on the machine config revision (`machineStatus.data.config.revision`, same signal `useFrames` uses) that, on change:
1. Tears down and re-opens the `streamTransformChanges` subscription against the current `client.current` (which points at the rebuilt resource), and
2. Resets the `initialized` gate and re-runs the `ListUUIDs` + `GetTransform` snapshot, reconciling the ECS entity set against the fresh snapshot — i.e. destroy any rendered entity whose UUID is absent from the fresh set, add any new ones, update changed ones.

Both steps are required (see "Compounding issue"). A narrower alternative — giving `svelte-sdk`'s resource client/name layer a server-side resource "generation" signal so same-named rebuilds change identity — would fix this class of bug more generally but is a larger change to `svelte-sdk`.

### Test notes for the PR
- Repro without hardware: mount `<Visualizer>` against a fake `world_state_store` whose backing resource is swapped (new instance, same name) while a stream is open; assert the client re-subscribes and reconciles.
- Regression guard: after a simulated reconfigure, an entity removed on the server disappears client-side within one poll/reconcile cycle; an added entity appears; an updated one moves — all without a page reload.

## Cross-references
- Downstream root-cause investigation and the client-side workaround we shipped: `teach-frames` Bug 1 (see the remaining-work handoff and the roadmap memory).
- The teach-frames mirror that exposed it: `models/worldstatestore/{model,diff,mapping}.go`.
