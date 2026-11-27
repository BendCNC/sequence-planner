/** Geometry worker owned by the local press service. No device or machine I/O. */
import {mkdtemp,writeFile,rm,readFile} from 'node:fs/promises';
import {join} from 'node:path';
import {tmpdir} from 'node:os';
import {fileURLToPath} from 'node:url';
import {runPlanFile} from './plan-file.mjs';
import {releaseFoldInputs} from '../viewer/sequence-fold-pool.mjs';

const quiet={write(){}};
async function prepareRuntime(){
  // An unrelated distributed example exercises the mesh kernels and Node's
  // compiler before the first user request. Discard the model and its outputs.
  const model=fileURLToPath(new URL('../viewer/examples/bcbend-branched-oblique.json',import.meta.url));
  const dir=await mkdtemp(join(tmpdir(),'bend-runtime-'));
  try{
    const options=join(dir,'options.json');
    await writeFile(options,JSON.stringify({bedLengthMm:838.2,backgauge:{separationMm:100,centerYmm:0},
      additionalRetractMm:6.35,maxStates:8,maxTransitions:12,timeBudgetMs:3500,allowPartialBends:false}));
    for(let i=0;i<3;i++){
      await runPlanFile(['--model',model,'--options',options,'--output',join(dir,'result.json')],{stdout:quiet,stderr:quiet});
      releaseFoldInputs();
    }
  }finally{await rm(dir,{recursive:true,force:true});}
}
async function availableMemory(){
  const info=await readFile('/proc/meminfo','utf8').catch(()=>'');
  return Number(/^MemAvailable:\s+(\d+)/m.exec(info)?.[1]??0)/1024;
}
if(!process.send)throw Error('Start this worker using the local press runtime');
await prepareRuntime();
globalThis.gc?.();
process.send({ready:true});
process.on('message',async ({model,options,file})=>{
  const stdout={write:line=>process.send({stdout:line})};
  const stderr={write:line=>process.send({stderr:line})};
  const code=await runPlanFile(['--model',model,'--options',options,'--output',file],{stdout,stderr});
  // The completed file is closed. Reclaim this request before accepting the next one.
  process.send({code});
  releaseFoldInputs();
  globalThis.gc?.();
  if(process.memoryUsage.rss()>550*1024*1024||await availableMemory()<450)process.disconnect();
  else process.send({ready:true});
});
process.on('disconnect',()=>process.exit(0));

