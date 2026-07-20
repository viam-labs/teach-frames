import { defineConfig } from 'vitest/config'
import { svelte } from '@sveltejs/vite-plugin-svelte'
import tailwindcss from '@tailwindcss/vite'
import glsl from 'vite-plugin-glsl'

export default defineConfig(async () => ({
  base: './',
  // vite-plugin-glsl's factory is async; Vite's build/dev pipeline tolerates
  // an unawaited Promise<Plugin> in the array, but Vitest's collection phase
  // does not — it silently drops later transforms, making every test file
  // report "no test suite found". Await it explicitly.
  plugins: [await glsl(), tailwindcss(), svelte()],
  // @viamrobotics/motion-tools ships a .hdr environment map asset that Vite
  // doesn't recognize as a static asset type by default.
  assetsInclude: ['**/*.hdr'],
  optimizeDeps: {
    esbuildOptions: { target: 'esnext' },
    // @viamrobotics/motion-tools ships unbundled source that imports .glsl and
    // .hdr assets. Dep pre-bundling (dev) runs BEFORE the glsl()/assetsInclude
    // plugins, so those imports fail there ("stream did not contain valid
    // UTF-8", glsl parse errors). Excluding the package skips pre-bundling and
    // routes its source through the normal plugin pipeline, where glsl() and
    // assetsInclude handle those assets. Production `build` was unaffected
    // because it always runs the full pipeline. The reference app never hit
    // this: there motion-tools is local source, not a pre-bundled dependency.
    exclude: ['@viamrobotics/motion-tools'],
  },
  build: { outDir: 'dist', emptyOutDir: true, target: 'esnext' },
  test: { environment: 'jsdom', globals: true },
}))
