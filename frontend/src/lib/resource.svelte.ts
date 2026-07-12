// The pose-tracker resource the operator selected during onboarding
// (ResourcePicker). Null until chosen; every panel's client reads it so we never
// assume a resource name. Reset on reload (re-run onboarding).
export const selectedResource = $state<{ name: string | null }>({ name: null })
