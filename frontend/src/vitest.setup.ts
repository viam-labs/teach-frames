// jsdom doesn't implement the Worker API. @viamrobotics/motion-tools'
// `./lib` barrel eagerly constructs a Worker at *import time* (its PCD
// point-cloud loader), as a side effect of exporting unrelated,
// worker-free utilities (e.g. OrientationVector) from the same entry point.
// That side effect only matters for point-cloud parsing, which nothing in
// this test suite exercises — stub it out so the barrel can load under
// jsdom without pulling in a real worker runtime.
if (typeof globalThis.Worker === 'undefined') {
  class NoopWorker {
    addEventListener() {}
    removeEventListener() {}
    postMessage() {}
    terminate() {}
  }
  // @ts-expect-error -- minimal stand-in, not a spec-complete Worker
  globalThis.Worker = NoopWorker
}
