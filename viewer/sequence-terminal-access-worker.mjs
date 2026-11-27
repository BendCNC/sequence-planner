/** The closing stroke receives a worker within the ordinary fold budget. */
import {foldPoolSize,leaseTerminalStage} from './sequence-fold-pool.mjs';
export const terminalWorkerCount=()=>foldPoolSize()>=2?1:0;
export const startTerminalAccess=leaseTerminalStage;
export {foldPoolUsage} from './sequence-fold-pool.mjs';
