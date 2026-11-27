/** Screen complete short-first programs at fixed stations before spending time
 * checking access/retraction on prefixes that cannot finish a closing flange. */
import {materialIntervals} from './machine.mjs';
import {isPart,sceneWithAmounts,checkScene} from './sequence-scene.mjs';
import {evaluateOperatorSupport} from './sequence-operator-support.mjs';
import {toolSetupKey} from './tool-setup-search.mjs';

const span=b=>{const a=materialIntervals(b).intervals;return a.at(-1)[1]-a[0][0];};
const physicalKey=c=>JSON.stringify([toolSetupKey(c),c.segments.map(s=>[s.kind,s.lengthMm,s.centerY,s.offsetX??0,s.offsetZ??0,!!s.mirror]).sort()]);
function activeLength(c){
  const punches=c.segments.filter(s=>s.kind==='punch'),at=punches.find(s=>Math.abs(s.centerY-c.stationYmm)<=s.lengthMm/2+1e-7);
  return punches.filter(s=>s.bankId===at?.bankId).reduce((n,s)=>n+s.lengthMm,0);
}
export function* probeFixedClosurePrograms({model,catalog,setups,options:o,support,initial,guard,finalKey,stats}){
  const order=[...model.bends].sort((a,b)=>span(a)-span(b)),groups=new Map();
  for(const [id,configs] of setups.byBend)for(const c of configs){
    if(c.layout!=='COMPACT'&&toolSetupKey(c)!==finalKey)continue;
    const key=physicalKey(c);if(!groups.has(key))groups.set(key,new Map());
    const group=groups.get(key);if(!group.has(id))group.set(id,[]);group.get(id).push(c);
  }
  const rank=g=>{const c=g.values().next().value[0];return c.setupFamily!=='TWO_BANK'?0:c.layout==='COMPACT'?1:2;};
  const complete=[...groups.values()].filter(g=>g.size===order.length).sort((a,b)=>rank(a)-rank(b));
  const full=b=>({bend:b,target:1}),programs=[order.map(full)];
  for(const closing of order.filter(b=>Math.abs(b.signedAngle)>Math.PI/2+1e-7)){
    const target=o.partialAngleDegrees/(Math.abs(closing.signedAngle)*180/Math.PI);if(!(target>0&&target<1))continue;
    const others=order.filter(b=>b!==closing);
    for(let pos=others.length-1;pos>=Math.max(0,others.length-3);pos--){
      programs.push([...others.slice(0,pos).map(full),{bend:closing,target},...others.slice(pos).map(full),full(closing)]);
    }
  }
  // A failed screen only discards this trial. The general search remains
  // available. No pose screen may mark an operation or transfer verified.
  let probes=0,proposals=0;const memo=new Map();
  for(const group of complete.slice(0,4))for(const program of programs){
    const amounts={...initial},route=[];
    for(const {bend,target} of program){
      const id=String(bend.id);let chosen=null;
      for(const cfg of [...group.get(id)].sort((a,b)=>activeLength(a)-activeLength(b))) for(const flip of [false,true]){
        if(chosen)break;
        const config={...cfg,bendId:bend.id,flip,ramGapMm:0,contactGapMm:o.contactGapMm};
        const key=JSON.stringify([physicalKey(cfg),cfg.stationYmm,flip,id,amounts,target]);let clear=memo.get(key);
        if(clear===undefined){
          if(++probes>240)return;stats.configurations++;guard();clear=true;
          for(const q of [1,0,.5]){
            stats.poses++;
            const scene=sceneWithAmounts(model,catalog,config,{...amounts,[id]:amounts[id]+(target-amounts[id])*q});
            if(!evaluateOperatorSupport(support.get(id),scene).passes||
              checkScene(model,scene,{clearanceMm:o.clearanceMm,onlyIds:new Set(scene.bodies.filter(b=>!isPart(b)).map(b=>b.id))}).status!=='SAMPLE_CLEAR'){clear=false;break;}
          }
          memo.set(key,clear);
        }
        if(clear)chosen=config;
      }
      if(!chosen)break;route.push({bendId:bend.id,target,config:chosen});amounts[id]=target;
    }
    if(route.length===program.length){stats.programs++;yield route;if(++proposals>=6)return;}
  }
}
