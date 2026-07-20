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
    // @viamrobotics/motion-tools ships unbundled source (110 .svelte files plus
    // .glsl shaders and a .hdr env map). vite-plugin-svelte already teaches the
    // dep optimizer (Rolldown, in Vite 8) to handle .svelte during pre-bundle;
    // only .glsl/.hdr block it ("stream did not contain valid UTF-8", glsl parse
    // errors). Declaring module types for those two lets motion-tools pre-bundle
    // fully — which is essential, because EXCLUDING it instead severs its many
    // transitive CommonJS deps (classnames, three-perf's nested tweakpane@3, …)
    // from the CJS->ESM interop that only pre-bundling provides, breaking
    // `import x from` in the browser. .hdr -> dataurl (the code imports it as a
    // URL string), .glsl -> text (imported as a raw shader string; these shaders
    // use no #include, so glsl()'s extra processing isn't needed at pre-bundle
    // time). Uses rolldownOptions.moduleTypes (the native Vite 8 option), not the
    // deprecated esbuildOptions.loader shim.
    rolldownOptions: {
      moduleTypes: { '.hdr': 'dataurl', '.glsl': 'text' },
    },
  },
  build: { outDir: 'dist', emptyOutDir: true, target: 'esnext' },
  test: {
    environment: 'jsdom',
    globals: true,
    setupFiles: ['./src/vitest.setup.ts'],
    // @viamrobotics/motion-tools' `./lib` entry re-exports a .svelte
    // component (AxesHelper) alongside plain classes/functions (see the
    // optimizeDeps comment above). Vitest's default dep externalization loads
    // node_modules packages via Node's native ESM loader, which chokes on the
    // unbundled .svelte source ("Unknown file extension '.svelte'") the
    // moment anything from `./lib` is imported — even just OrientationVector.
    // Inlining routes it through Vite's own transform pipeline (svelte()
    // plugin included) instead, matching how the dev/build pipeline handles
    // it. `@threlte/core` (motion-tools' own dependency, also shipped as
    // unbundled .svelte source) hits the same problem transitively, so it
    // needs inlining too.
    server: { deps: { inline: ['@viamrobotics/motion-tools', '@threlte/core'] } },
  },
}))
