import { createWorkerState, handleMessage } from './handle'
import type { FromWorker, ToWorker } from './protocol'

export { handleMessage, createWorkerState }

/**
 * Narrow view of the worker global. The tsconfig lib has both DOM and WebWorker, where `self`
 * resolves to the DOM Window; a local structural type avoids that conflict.
 */
interface WorkerScope {
  onmessage: ((e: { data: ToWorker }) => void) | null
  postMessage(msg: FromWorker, transfer: Transferable[]): void
}

const scope = globalThis as unknown as WorkerScope
const state = createWorkerState()

scope.onmessage = (e) => {
  const res = handleMessage(e.data, state)
  if (res) scope.postMessage(res, [res.gray])
}
