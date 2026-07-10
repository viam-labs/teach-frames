import Cookies from 'js-cookie'
import { getContext, setContext } from 'svelte'
import type { DialConf, Credentials } from '@viamrobotics/sdk'

export interface MachineIdentity {
  id: string
  dialConf: DialConf
}

interface MachineCookie {
  hostname: string
  credentials: Credentials
  machineId: string
}

// The Viam browser SDK connects over WebRTC through the app.viam.com
// signaling server. This is the standard address for Viam Applications.
const SIGNALING_ADDRESS = 'https://app.viam.com:443'

// Viam Applications (single_machine) inject a browser cookie keyed by the
// machine id, which is the third URL path segment: /machine/{id}/...  `viam
// module local-app-testing` injects the same cookie in dev.
export function currentMachine(): MachineIdentity {
  const id = window.location.pathname.split('/')[2]
  if (!id) {
    throw new Error('no machine id in URL path (expected /machine/{id}/...)')
  }
  const raw = Cookies.get(id)
  if (!raw) {
    throw new Error(`no Viam credentials cookie for machine ${id}`)
  }
  const cookie = JSON.parse(raw) as MachineCookie
  return {
    id,
    dialConf: {
      host: cookie.hostname,
      credentials: cookie.credentials,
      signalingAddress: SIGNALING_ADDRESS,
    },
  }
}

// Svelte context plumbing so any panel deep in the tree can grab the machine
// id (as a reactive accessor, matching the `() => string` shape expected by
// createResourceClient) without threading it through every component's props.
const MACHINE_ID_KEY = Symbol('machine-id')

export function provideMachineId(id: string): void {
  setContext(MACHINE_ID_KEY, () => id)
}

export function useMachineId(): () => string {
  const accessor = getContext<() => string>(MACHINE_ID_KEY)
  if (!accessor) {
    throw new Error('useMachineId() called outside of a component tree with provideMachineId()')
  }
  return accessor
}
