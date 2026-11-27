import {consistentSupportCentroid} from './sequence-operator-support.mjs';
﻿/** Synchronous worker pool for independent fold samples. Results are consumed
 * in sample order by the caller, so first-failure semantics are unchanged. */
import {Worker,MessageChannel,receiveMessageOnPort,isMainThread} from 'node:worker_threads';
import {availableParallelism,freemem} from 'node:os';

// Pool size scales with the machine: one core stays free for the main thread
// and the OS, the remainder is shared among concurrent planner processes
// (UNFOLD_PLANNER_PROCESSES, default 1), and each worker's private model and
// catalog copy is bounded by free memory. Fewer than 2 workers means the
// sequential path (1-2 core machines). UNFOLD_FOLD_WORKERS overrides all.
const WORKER_MEMORY_BYTES=160*1024*1024;
export function defaultFoldWorkerCount({cpus=availableParallelism(),processes=Number(process.env.UNFOLD_PLANNER_PROCESSES??1),free=freemem()}={}){
  const share=Math.floor((cpus-1)/Math.max(1,Math.floor(processes)||1));
  const byMemory=Math.floor(free*.5/WORKER_MEMORY_BYTES);
  // On four cores a third model copy competes with the solver and the Qt interface.
  const n=Math.max(0,Math.min(share,byMemory,cpus===4?2:Infinity));
  return n<2?0:n;
}
const requested=Number(process.env.UNFOLD_FOLD_WORKERS??NaN);
const workerCount=Number.isFinite(requested)?Math.max(0,Math.floor(requested)):defaultFoldWorkerCount();
export const foldWorkerCapacity=workerCount;
let pool=null,nextJobId=1,workersCreated=0,peakAssigned=0;
const ids=new WeakMap();let nextId=1;
const idOf=o=>{if(!ids.has(o))ids.set(o,nextId++);return ids.get(o);};

// A worker's scalar must belong to the same material-pose vector. A bad
// stage is discarded and the caller performs the ordinary geometry checks.
const coherentStage=checked=>checked?.feasible&&checked.stage?.foldSamples?.length&&
  checked.stage.foldSamples.every(sample=>consistentSupportCentroid(sample.operatorSupport));

function getPool(){
  if(!isMainThread||workerCount<2)return null;
  if(pool)return pool;
  // [finished workers, next position, cancel, first blocked position, progress].
  // The last three slots are used only by coordinated world validation.
  const shared=new Int32Array(new SharedArrayBuffer(20));
  pool={workers:[],leases:new Set(),done:shared,busy:false};
  return pool;
}
function addWorker(p){
  const {port1,port2}=new MessageChannel();
  const worker=new Worker(new URL('./sequence-fold-worker.mjs',import.meta.url),{workerData:{port:port2,done:p.done},transferList:[port2]});
  worker.unref();port1.unref();
  const entry={worker,port:port1,sent:new Set(),failure:null};
  worker.on('error',error=>{entry.failure=error;});
  worker.on('exit',code=>{if(code!==0)entry.failure??=new Error(`Fold worker exited with code ${code}`);});
  p.workers.push(entry);workersCreated++;peakAssigned=Math.max(peakAssigned,p.workers.length+p.leases.size);return entry;
}
function availableWorkers(p,count){
  const needed=Math.min(count,workerCount-p.leases.size);
  while(p.workers.length<needed)addWorker(p);
  return p.workers.slice(0,needed);
}
/** Read capacity without spawning a worker. */
export function foldPoolSize(){const p=getPool();return p?workerCount-p.leases.size:0;}
export function foldPoolUsage(){
  const p=getPool();
  return{workerLimit:p?workerCount:0,availableLanes:foldPoolSize(),allocatedWorkers:p?p.workers.length+p.leases.size:0,
    terminalLeases:p?.leases.size??0,workersCreated,peakAssignedWorkers:peakAssigned};
}

/** The last stroke uses a lane from this process's existing CPU/memory budget.
 * The leased job has a private reply and cancellation flag. It never writes
 * the other workers' sample counters or starts a nested pool. */
export function leaseTerminalStage({model,catalog,stage,options},deadline){
  const p=getPool();if(!p||p.busy||p.leases.size||foldPoolSize()<2)return null;
  if(!p.workers.length)addWorker(p);
  const w=p.workers.pop(),modelId=idOf(model),catalogId=idOf(catalog),jobId=nextJobId++;
  const cancel=new Int32Array(new SharedArrayBuffer(4));
  let active=true,resolve;
  const result=new Promise(r=>{resolve=r;});
  const finish=(value,reuse)=>{
    if(!active)return;active=false;p.leases.delete(lease);
    w.port.off('message',receive);w.worker.off('exit',exit);w.worker.off('error',exit);
    if(reuse&&pool===p&&!w.failure)p.workers.push(w);
    else{Atomics.store(cancel,0,1);w.port.close();w.worker.terminate().catch(()=>{});}
    resolve(value);
  };
  const receive=reply=>finish(reply?.kind==='terminal-stage'&&reply.jobId===jobId&&coherentStage(reply.checked)?reply.checked:null,reply?.jobId===jobId);
  const exit=()=>finish(null,false);
  const lease={
    async take(guard){
      let timer;
      try{return await Promise.race([result,new Promise((_,reject)=>{timer=setInterval(()=>{try{guard();}catch(error){reject(error);}},50);})]);}
      finally{clearInterval(timer);}
    },
    close(){finish(null,false);}
  };
  p.leases.add(lease);w.port.once('message',receive);w.worker.once('exit',exit);w.worker.once('error',exit);
  try{
    const msg={kind:'terminal-stage',jobId,modelId,catalogId,stage,options,deadline,cancel};
    if(!w.sent.has(modelId))msg.model=model;if(!w.sent.has('c'+catalogId))msg.catalog=catalog;
    w.port.postMessage(msg);w.sent.add(modelId);w.sent.add('c'+catalogId);
  }catch{lease.close();return null;}
  return lease;
}

/** Check unchanged ordinary entry/exit for independent stages of a known fold
 * chain. Use the already-reserved fold workers; the caller handles unsuccessful
 * or missing results with the full validator, in original stage order. */
export async function ordinaryStageAccess(model,catalog,entries,options,guard){
  const p=getPool();
  if(!p||p.busy||entries.length<3)return new Map();
  const used=availableWorkers(p,entries.length);
  const modelId=idOf(model),catalogId=idOf(catalog),jobId=nextJobId++,results=new Map();
  let finished=false;
  p.busy=true;
  try{
    Atomics.store(p.done,0,0);Atomics.store(p.done,2,0);
    for(const [lane,w] of used.entries()){
      const assigned=entries.filter((_,index)=>index%used.length===lane);
      const msg={kind:'normal-stage-access',modelId,catalogId,jobId,options,entries:assigned};
      if(!w.sent.has(modelId)){msg.model=model;w.sent.add(modelId);}
      if(!w.sent.has('c'+catalogId)){msg.catalog=catalog;w.sent.add('c'+catalogId);}
      w.port.postMessage(msg);
    }
    while(Atomics.load(p.done,0)<used.length){
      guard();
      if(used.some(w=>w.failure))return new Map();
      await new Promise(resolve=>setTimeout(resolve,10));
    }
    for(const [lane,w] of used.entries()){
      const reply=receiveMessageOnPort(w.port)?.message;
      if(reply?.jobId!==jobId||reply.kind!=='normal-stage-access'||!Array.isArray(reply.results))return new Map();
      for(const {index,checked} of reply.results){
        if(!coherentStage(checked)||results.has(index)||!entries.some((e,k)=>e.index===index&&k%used.length===lane))return new Map();
        results.set(index,checked);
      }
    }
    finished=true;return results;
  }finally{
    if(!finished)resetFoldPool();
    p.busy=false;
  }
}

/** Lease the existing fold workers to verify independent proposed strokes. */
export function startFoldPipeline(model,catalog,options){
  const p=getPool();
  if(!p||p.busy||foldPoolSize()<2)return null;
  const modelId=idOf(model),catalogId=idOf(catalog),pending=[],free=p.workers.slice(),tasks=[],owned=new Set(free);
  let closed=false,failed=false,posted=0;
  p.busy=true;
  Atomics.store(p.done,0,0);Atomics.store(p.done,2,0);
  function dispatch(){
    while(pending.length&&!closed&&!failed){
      if(!free.length){
        const returned=p.workers.find(worker=>!owned.has(worker));
        if(!returned&&p.workers.length>=foldPoolSize())break;
        const added=returned??addWorker(p);owned.add(added);added.worker.on('exit',exited);free.push(added);
      }
      const w=free.pop(),task=pending.shift(),jobId=nextJobId++;
      const msg={kind:'verify-fold-and-access',modelId,catalogId,jobId,options,fold:task.input};
      if(!w.sent.has(modelId))msg.model=model;
      if(!w.sent.has('c'+catalogId))msg.catalog=catalog;
      w.port.once('message',reply=>{
        if(closed)return;
        task.result=reply?.kind===msg.kind&&reply.jobId===jobId?reply.checked:null;
        task.done=true;failed||=!coherentStage(task.result);
        free.push(w);dispatch();
      });
      try{
        w.port.postMessage(msg);posted++;w.sent.add(modelId);w.sent.add('c'+catalogId);
      }catch{failed=true;}
    }
  }
  const exited=()=>{failed=true;};
  for(const w of p.workers)w.worker.on('exit',exited);
  return{
    submit(input){const task={input,done:false};tasks.push(task);pending.push(task);dispatch();return task;},
    async wait(guard){
      while(!failed&&(tasks.some(t=>!t.done)||Atomics.load(p.done,0)<posted)){
        guard();if(p.workers.some(w=>w.failure))return false;
        await new Promise(resolve=>setTimeout(resolve,4));
      }
      return !failed;
    },
    close(){
      if(closed)return;closed=true;
      for(const w of p.workers)w.worker.off('exit',exited);
      // Wait includes the post-message acknowledgement; a previous reply
      // cannot increment the shared counter after the next lease begins.
      if(failed||p.workers.some(w=>w.failure)||tasks.some(t=>!t.done)||Atomics.load(p.done,0)!==posted)resetFoldPool();
      p.busy=false;
    }
  };
}

/** Release all request inputs while retaining worker code between independent plans. */
export function releaseFoldInputs(){
  if(!pool)return;
  if(pool.busy||pool.leases.size){resetFoldPool();return;}
  for(const worker of pool.workers){
    worker.sent.clear();
    worker.port.postMessage({kind:'release-inputs'});
  }
}

/** Discard every port before a replacement pool can receive another job. */
export function resetFoldPool(){
  const previous=pool;pool=null;if(!previous)return;
  Atomics.store(previous.done,2,1);
  for(const lease of [...previous.leases])lease.close();
  for(const w of previous.workers){w.port.close();w.worker.terminate().catch(()=>{});}
}

// Independent immutable jobs share the existing lane budget. A missing reply
// selects the local validator for that job; partial replies are never accepted.
export function checkIndependentJobs(model,catalog,jobs,local,runMain){
  const p=getPool();
  const results=new Array(jobs.length);
  if(!p||p.busy||p.leases.size||jobs.length<2){
    const main=runMain();
    return {main,results:jobs.map(local)};
  }
  const workers=availableWorkers(p,Math.min(jobs.length,2));
  const jobId=nextJobId++,modelId=idOf(model),catalogId=idOf(catalog);
  const deadline=Date.now()+30000;
  let main,finished=false;
  try{
    p.busy=true;Atomics.store(p.done,0,0);
    for(const [i,w] of workers.entries()){
      const msg={kind:'independent-check',jobId,modelId,catalogId,input:jobs[i]};
      if(!w.sent.has(modelId))msg.model=model;
      if(!w.sent.has('c'+catalogId))msg.catalog=catalog;
      w.port.postMessage(msg);w.sent.add(modelId);w.sent.add('c'+catalogId);
    }
    main=runMain();
    for(let i=workers.length;i<jobs.length;i++)results[i]=local(jobs[i]);
    while(Date.now()<deadline){
      const count=Atomics.load(p.done,0);
      if(count===workers.length){finished=true;break;}
      Atomics.wait(p.done,0,count,Math.min(100,deadline-Date.now()));
    }
    if(finished)for(const [i,w] of workers.entries()){
      const reply=receiveMessageOnPort(w.port)?.message;
      if(reply?.kind==='independent-check'&&reply.jobId===jobId&&!reply.error)results[i]=reply.result;
    }
  }finally{
    if(!finished)resetFoldPool();
    p.busy=false;
  }
  for(let i=0;i<workers.length;i++)results[i]??=local(jobs[i]);
  return {main,results};
}

/** Run msg on one worker while local() runs on this thread; returns
 * [localResult, workerResult]. The worker is always drained, even on throw. */
export function runAside({model,catalog,msg},local){
  const p=getPool(),w=availableWorkers(p,1)[0],modelId=idOf(model),catalogId=idOf(catalog);
  if(p.busy)throw new Error('Fold worker pool is already in use');
  const out={...msg,modelId,catalogId};
  if(!w.sent.has(modelId)){out.model=model;w.sent.add(modelId);}
  if(!w.sent.has('c'+catalogId)){out.catalog=catalog;w.sent.add('c'+catalogId);}
  Atomics.store(p.done,0,0);w.port.postMessage(out);p.busy=true;
  let reply;
  try{return[local(),(()=>{for(let v;(v=Atomics.load(p.done,0))<1;)Atomics.wait(p.done,0,v);
    reply=receiveMessageOnPort(w.port).message;if(reply.error)throw new Error(reply.error);return reply.result;})()];}
  finally{try{if(!reply){for(let v;(v=Atomics.load(p.done,0))<1;)Atomics.wait(p.done,0,v);receiveMessageOnPort(w.port);}}finally{p.busy=false;}}
}

/** Evaluate sample indices in parallel; returns results indexed by sample.
 * Workers (and, when given, the calling thread via local(i)) claim indices
 * from a shared counter, so fast cores take more samples than slow ones.
 * before() runs on the calling thread while the workers start. Each sample
 * is a pure function of its index, so which thread computes it is irrelevant;
 * the caller consumes results in sample order. */
export function evaluateFoldSamples({model,catalog,baseConfig,bendId,from,to,count,clearanceMm,indices},{before,local}={}){
  const p=getPool();const modelId=idOf(model),catalogId=idOf(catalog);
  if(p.busy)throw new Error('Fold worker pool is already in use');
  Atomics.store(p.done,0,0);Atomics.store(p.done,1,0);
  const used=availableWorkers(p,indices.length);
  for(const w of used){
    const msg={kind:'fold-samples',modelId,catalogId,baseConfig,bendId,from,to,count,clearanceMm,indices};
    if(!w.sent.has(modelId)){msg.model=model;w.sent.add(modelId);}
    if(!w.sent.has('c'+catalogId)){msg.catalog=catalog;w.sent.add('c'+catalogId);}
    w.port.postMessage(msg);}
  p.busy=true;const results=new Map();
  const replies=[];
  try{
    before?.();
    if(local)for(let k;(k=Atomics.add(p.done,1,1))<indices.length;)results.set(indices[k],local(indices[k]));
  }catch(error){Atomics.store(p.done,1,indices.length);throw error;}
  finally{
    // Always drain workers and their replies, so an exception on this thread
    // never leaks results into the next batch.
    try{for(let v;(v=Atomics.load(p.done,0))<used.length;)Atomics.wait(p.done,0,v);
      for(const w of used)for(let m;(m=receiveMessageOnPort(w.port));)replies.push(m.message);
    }finally{p.busy=false;}
  }
  for(const reply of replies){
    if(reply.error)throw new Error(reply.error);
    for(const r of reply.results){
      // Omitting a contradictory sample invokes the caller's local exact evaluator.
      if(!r.support?.passes||consistentSupportCentroid(r.support))results.set(r.i,r);
    }}
  return results;
}

/** Original world-space checks, using the same pool as fold samples.
 * Each worker builds one world per job. Samples are claimed in increasing
 * order, so after a failure only indices GREATER than it may be skipped.
 * Replies account for every claimed index. Missing/error replies never clear.
 * null means the serial path is required (not an infrastructure failure).
 */
export function evaluateCoordinatedWorldSamples({model,catalog,bendId,config,amounts,obstacles=[],options={},edges,count},
 {guard=()=>{},workerTimeoutMs=30000,cancelDrainTimeoutMs=1000}={}){
  const p=getPool();if(!p||p.busy||foldPoolSize()<2)return null;
  if(!Number.isInteger(count)||count<1||count>0x7ffffffe||!Array.isArray(edges)||!edges.length)throw new RangeError('Invalid coordinated sample job');
  if(!Number.isFinite(workerTimeoutMs)||workerTimeoutMs<=0||!Number.isFinite(cancelDrainTimeoutMs)||cancelDrainTimeoutMs<=0)throw new RangeError('Invalid coordinated worker timeout');
  const modelId=idOf(model),catalogId=idOf(catalog),jobId=nextJobId++;
  // Callbacks, AbortSignal and planner state stay on the main thread.
  const physicalOptions=Object.fromEntries(['clearanceMm','toleranceMm','maxCoordinatedRamGapMm','frontMarginMm']
    .filter(key=>options[key]!==undefined).map(key=>[key,options[key]]));
  const payload={bendId,config,amounts,obstacles,options:physicalOptions};
  const used=availableWorkers(p,count),posted=[],replies=new Map();
  for(const [i,value]of [0,0,0,count,0].entries())Atomics.store(p.done,i,value);
  p.busy=true;
  let error=null,infrastructureError=false,cancelAt=Infinity,lastProgress=performance.now(),progress=0;
  const fail=(e,infrastructure)=>{if(!error){error=e;infrastructureError=infrastructure;cancelAt=performance.now();Atomics.store(p.done,2,1);}};
  const collect=()=>{
    for(const w of posted)for(let received;(received=receiveMessageOnPort(w.port));){
      const reply=received.message;
      if(reply?.kind!=='coordinated-world-samples'||reply.jobId!==jobId||replies.has(w)){
        fail(new Error('Invalid or stale coordinated worker reply'),true);continue;
      }
      replies.set(w,reply);
      if(reply.error)fail(new Error(`Coordinated world worker failed: ${reply.error}`),true);
    }
  };
  try{
    try{
      guard();
      for(const w of used){
        guard();if(w.failure)throw w.failure;
        const msg={kind:'coordinated-world-samples',jobId,modelId,catalogId,payload,edges,count};
        if(!w.sent.has(modelId)){msg.model=model;w.sent.add(modelId);}
        if(!w.sent.has('c'+catalogId)){msg.catalog=catalog;w.sent.add('c'+catalogId);}
        w.port.postMessage(msg);posted.push(w);
      }
    }catch(e){fail(e,true);}
    while(Atomics.load(p.done,0)<posted.length||replies.size<posted.length){
      collect();
      if(!error){
        try{guard();}catch(e){fail(e,false);}
        const completed=Atomics.load(p.done,4);
        if(completed!==progress){progress=completed;lastProgress=performance.now();}
        if(posted.some(w=>w.failure))fail(new Error('Coordinated world worker exited unexpectedly'),true);
        if(performance.now()-lastProgress>workerTimeoutMs)fail(new Error('Coordinated world worker made no progress before timeout'),true);
      }
      if(error&&performance.now()-cancelAt>=cancelDrainTimeoutMs)break;
      if(Atomics.load(p.done,0)>=posted.length&&replies.size>=posted.length)break;
      Atomics.wait(p.done,0,Atomics.load(p.done,0),25);
    }
    collect();
    if(error)throw error;
    if(posted.length!==used.length||replies.size!==used.length||Atomics.load(p.done,0)!==used.length)throw new Error('Incomplete coordinated world worker batch');
    let checked=0,skipped=0,firstFailure=null;
    for(const reply of replies.values()){
      if(!Number.isInteger(reply.checked)||reply.checked<0||!Number.isInteger(reply.skipped)||reply.skipped<0)throw new Error('Missing coordinated sample accounting');
      checked+=reply.checked;skipped+=reply.skipped;
      if(reply.firstFailure){
        const failure=reply.firstFailure;
        if(!Number.isInteger(failure.index)||failure.index<0||failure.index>=count||failure.result?.clear!==false)throw new Error('Invalid coordinated collision witness');
        if(!firstFailure||failure.index<firstFailure.index)firstFailure=failure;
      }
    }
    const claimed=Math.min(count,Atomics.load(p.done,1)),first=Atomics.load(p.done,3);
    if(checked+skipped!==claimed||checked!==Atomics.load(p.done,4)||
      (firstFailure?firstFailure.index!==first||checked<first+1:checked!==count||skipped!==0||first!==count))throw new Error('Incomplete coordinated sample coverage');
    return{firstFailure};
  }catch(e){
    // A cooperative guard abort can reuse a fully drained pool. Infrastructure
    // errors, protocol errors and undrained workers require a fresh pool.
    if(!error){error=e;infrastructureError=true;Atomics.store(p.done,2,1);}
    throw e;
  }finally{
    if(infrastructureError||Atomics.load(p.done,0)!==posted.length||replies.size!==posted.length)resetFoldPool();
    p.busy=false;
  }
}
