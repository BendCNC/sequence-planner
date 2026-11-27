// Differential gate for checkScene's optional broad phase: runs every sampled pose
// with collisionBroadphase 'off' and 'bvh' and reports any verdict difference.
// Usage: node tools/bvh_differential.mjs [--poses N] [--models a.json,b.json] [--seed S]
import {readFileSync} from 'node:fs';
import {fileURLToPath} from 'node:url';
import {performance} from 'node:perf_hooks';
import {sceneWithAmounts,checkScene} from '../viewer/sequence-scene.mjs';
import {generateSequenceSetups} from '../viewer/sequence-engine.mjs';

const root=new URL('../viewer/',import.meta.url);
const read=p=>JSON.parse(readFileSync(new URL(p,root),'utf8'));
export const DEFAULT_MODELS=['examples/batteryplate-step.json','examples/bcbend-simple-hole.json','examples/bcbend-branched-oblique.json',
  'examples/mixed-bends-rotated.json','examples/interrupted-band-rotated.json','examples/dp-bracket-component2.json','examples/sm1-step.json'];

function rng(seed){let s=seed>>>0||1;return()=>{s^=s<<13;s>>>=0;s^=s>>17;s^=s<<5;s>>>=0;return s/4294967296;};}
const hitKey=h=>`${h.bodyIds?.slice().sort().join('|')}`;

export function runDifferential({models=DEFAULT_MODELS,posesPerModel=40,seed=7,clearances=[.15,0,2]}={}){
  const catalog=read('tooling.json'),random=rng(seed);
  const out={poses:0,scenePairsChecked:0,disagreements:[],broadphaseSkipped:0,msOff:0,msBvh:0,models:[]};
  for(const file of models){
    const model=read(file);let setups;
    try{setups=generateSequenceSetups({model,catalog,options:{bedLengthMm:838.2}});}catch(e){out.models.push({file,error:e.message});continue;}
    const bends=model.bends.filter(b=>setups.byBend.get(b.id)?.length);let poses=0;
    for(let n=0;n<posesPerModel&&bends.length;n++){
      const bend=bends[Math.floor(random()*bends.length)],configs=setups.byBend.get(bend.id);
      const config={...configs[Math.floor(random()*configs.length)],bendId:bend.id};
      const pick=random(),amounts=Object.fromEntries(model.bends.map(b=>[b.id,pick<.3?0:pick<.6?(random()<.5?0:1):random()]));
      amounts[bend.id]=random();
      let scene;try{scene=sceneWithAmounts(model,catalog,config,amounts);}catch{continue;}
      for(const clearanceMm of clearances){
        const opts={clearanceMm,firstOnly:false};
        // Alternate which path runs first so warm caches do not bias the timing.
        const run=mode=>{const t=performance.now(),r=checkScene(model,scene,{...opts,collisionBroadphase:mode});out[mode==='off'?'msOff':'msBvh']+=performance.now()-t;return r;};
        let off,bvh;if(out.poses%2){bvh=run('bvh');off=run('off');}else{off=run('off');bvh=run('bvh');}
        out.poses++;poses++;out.broadphaseSkipped+=bvh.broadphaseSkipped;
        const nb=scene.bodies.length;out.scenePairsChecked+=nb*(nb-1)/2;
        const a=off.hits.map(hitKey).sort().join(','),b=bvh.hits.map(hitKey).sort().join(',');
        if(off.status!==bvh.status||a!==b)out.disagreements.push({file,bendId:bend.id,clearanceMm,amounts,off:[off.status,a],bvh:[bvh.status,b]});
      }
    }
    out.models.push({file,poses});
  }
  return out;
}

if(process.argv[1]&&fileURLToPath(import.meta.url)===process.argv[1]){
  const arg=k=>{const i=process.argv.indexOf(k);return i>0?process.argv[i+1]:undefined;};
  const r=runDifferential({posesPerModel:Number(arg('--poses')??40),seed:Number(arg('--seed')??7),models:arg('--models')?.split(',')??DEFAULT_MODELS});
  console.log(JSON.stringify({...r,disagreements:r.disagreements.length,disagreementSamples:r.disagreements.slice(0,5),
    speedup:+(r.msOff/r.msBvh).toFixed(3)},null,1));
  process.exitCode=r.disagreements.length?1:0;
}
