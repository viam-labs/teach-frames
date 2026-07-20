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
  },
  build: { outDir: 'dist', emptyOutDir: true, target: 'esnext' },
  test: { environment: 'jsdom', globals: true },
}))
