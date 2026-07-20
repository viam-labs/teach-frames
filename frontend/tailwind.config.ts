import type { Config } from 'tailwindcss'

import { plugins } from '@viamrobotics/prime-core/plugins'
import { theme } from '@viamrobotics/prime-core/theme'

/** @type {import('tailwindcss').Config} */
export default {
  darkMode: 'class',
  content: [
    './index.html',
    './src/**/*.{html,js,svelte,ts}',
    './node_modules/@viamrobotics/prime-core/**/*.{ts,svelte}',
    './node_modules/@viamrobotics/motion-tools/dist/**/*.{js,svelte,ts}',
  ],
  theme,
  plugins,
} satisfies Config
