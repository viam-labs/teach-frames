// Signal that the server-side capture buffer changed from OUTSIDE CapturePanel
// (e.g. define_frame clears it). CapturePanel refetches its buffer query when this bumps.
export const captureBufferSignal = $state({ version: 0 })

export function bumpCaptureBuffer() {
  captureBufferSignal.version++
}
