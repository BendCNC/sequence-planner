/** Bounded geometric access search. No model identifiers or saved trajectories.
 * The projection proposes paths; only independent original 3-D checks approve. */
import {createAccessWorld,accessZero,accessInterpolate,validateCoordinatedAccess} from './sequence-access-world.mjs';
import {createAccessProjection} from './sequence-access-projection.mjs';
import {bodyBounds} from './sequence-scene.mjs';
export {createAccessWorld,validateCoordinatedAccess};

const frame=p=>({ramGapMm:p.ramGapMm,pose:{x:p.x,y:0,z:p.z,tiltDeg:p.tiltDeg}});
export function searchCoordinatedAccess(payload,guard=()=>{}){
 guard();
 const started=performance.now(),options=payload.options??{},budget=options.coordinatedAccessBudgetMs??30000,world=createAccessWorld(payload);
 const diagnostics={method:'GEOMETRIC_CO_LIFT_AND_RIGID_ACCESS_SEARCH',seed:options.accessRandomSeed??1404,nodes:0,iterations:0,projectionEvaluations:0,validationAttempts:0,seedTests:[]};
 const stop=()=>{guard();if(performance.now()-started>=budget)throw new Error('COORDINATED_ACCESS_BUDGET');};
 const finish=(status,extra={})=>({status,...extra,diagnostics:{...diagnostics,elapsedMs:performance.now()-started,world:world.stats},
  limits:{clearanceMm:options.clearanceMm??.15,fixedStationY:true,operatorSupportRulePreserved:true,physicalMachineTravelVerified:false,continuousCollisionDetection:false}});
 const zero=accessZero(),initial=world.check(zero);if(!initial.clear)return finish('BLOCKED',{reason:'WORKING_POSE_INTERFERENCE',blocked:initial});
 const projection=createAccessProjection({...world,clearanceMm:options.clearanceMm??.15}),cache=new Map();
 function clear(p){const key=world.key(p);if(cache.has(key))return cache.get(key);diagnostics.projectionEvaluations++;const c=projection.clear(p);cache.set(key,c);return c;}
 const checkPose=p=>({clear:clear(p)}),edge=(a,b,step=.65)=>world.edge(a,b,step,{guard:stop,checkPose});
 let rng=diagnostics.seed>>>0;const random=()=>{rng^=rng<<13;rng^=rng>>>17;rng^=rng<<5;return(rng>>>0)/4294967296;};
 const height=world.bounds.max[2]-world.bounds.min[2],span=world.bounds.max[0]-world.bounds.min[0],stepHeight=Math.ceil(Math.max(height,span)/10)*10;
 const limit=options.maxCoordinatedRamGapMm??Math.max(stepHeight*2,100),lifts=[...new Set([stepHeight,Math.ceil(stepHeight*.75),Math.ceil(stepHeight*1.5)].filter(v=>v>0&&v<=limit))];
 let solution=null;
 function accept(poses){diagnostics.validationAttempts++;
  // Shortcut in the same configuration space, then recheck all interpolated
  // world meshes. Projection clearance alone never returns SAMPLE_CLEAR.
  for(let i=0;i<poses.length-2;i++)for(let j=poses.length-1;j>i+1;j--){stop();if(edge(poses[i],poses[j],.4).clear){poses.splice(i+1,j-i-1);break;}}
  const motion={pivot:world.pivot,frames:poses.map(frame)},validation=validateCoordinatedAccess(payload,motion,guard);
  if(validation.status==='SAMPLE_CLEAR'){solution={motion,validation,wholePartOutside:true};return true;}diagnostics.lastValidationFailure=validation;return false;
 }
 try{
  // First retain the inexpensive straight families, including lift with the
  // punch. Lift scales derive from the current part's geometric envelope.
  for(const lift of lifts){stop();const start={...zero,z:lift,ramGapMm:lift};const check=world.edge(zero,start,Math.min(1,options.ramStepMm??1),{guard:stop});
   diagnostics.seedTests.push({lift,status:check.clear?'CLEAR':'BLOCKED',blocked:check.blocked,reason:check.result?.reason});if(!check.clear)continue;
   const end=world.outsidePose(start);if(edge(start,end).clear&&accept([zero,start,end]))break;
   // Geometric roll primitives use lower front corners of the actual mounted
   // punches as rotation centres. These are proposals, not collision exceptions.
   // A brief inward/downward translation can disengage a return flange before
   // rolling it round a gooseneck. Both signs and a range of offsets are tested.
   const punchBoxes=world.rawObstacles.filter(b=>b.kind==='punch').map(bodyBounds);
   for(const fraction of [.1,.075,.125,.05,0,-.05]){
    if(solution)break;const shifted={...start,x:span*fraction,z:lift-span*Math.abs(fraction)};
    if(!edge(start,shifted,.4).clear)continue;
    for(const box of punchBoxes)for(const dx of [0,-2*modelThickness(payload),2*modelThickness(payload)])for(const dz of [0,-2*modelThickness(payload),2*modelThickness(payload)])for(const sign of [1,-1]){
     if(solution)break;stop();const centre=[box.min[0]+dx,box.min[2]+lift+dz],poses=[zero,start,shifted];let previous=shifted;
     for(let degrees=2;degrees<=80;degrees+=2){const angle=degrees*sign*Math.PI/180,c=Math.cos(angle),s=Math.sin(angle),u=world.pivot[0]+shifted.x-centre[0],v=world.pivot[2]+shifted.z-centre[1];
      const p={...shifted,x:centre[0]+c*u+s*v-world.pivot[0],z:centre[1]-s*u+c*v-world.pivot[2],tiltDeg:degrees*sign};
      if(!edge(previous,p,.4).clear)break;poses.push(p);previous=p;
      const exterior=world.outsidePose(p);if(edge(p,exterior,.65).clear&&accept([...poses,exterior]))break;
     }
    }
   }
   if(solution)break;
   const nodes=[{pose:start,parent:-1}],cells=new Set(['0|0|0']),weight=Math.max(1,world.radius*Math.PI/180),scales=[.1,.25,.5,1,2,5,10,20];
   const distance=(a,b)=>(a.x-b.x)**2+(a.z-b.z)**2+((a.tiltDeg-b.tiltDeg)*weight)**2;
   const path=i=>{const p=[];while(i>=0){p.push(nodes[i].pose);i=nodes[i].parent;}return[zero,...p.reverse()];};
   // One lift receives a bounded share, leaving other geometric openings a try.
   const deadline=Math.min(started+budget,performance.now()+budget/Math.max(1,lifts.length));
   while(performance.now()<deadline&&nodes.length<(options.maxAccessSearchNodes??50000)){
    stop();diagnostics.iterations++;let target;
    if(nodes.length<200||random()<.72){const a=nodes[Math.floor(random()*nodes.length)].pose,size=scales[Math.floor(random()*scales.length)];target={...a,x:a.x+(random()*2-1)*size,z:a.z+(random()*2-1)*size,tiltDeg:a.tiltDeg+(random()*2-1)*size/weight};}
    else target={...start,x:-2*world.radius+random()*2.5*world.radius,z:random()*(lift+world.radius),tiltDeg:(random()*2-1)*140};
    let parent=0,best=Infinity;for(let i=0;i<nodes.length;i++){const d=distance(nodes[i].pose,target);if(d<best){best=d;parent=i;}}
    const a=nodes[parent].pose,p=accessInterpolate(a,target,Math.min(1,8/Math.sqrt(best||1)));
    if(p.z<-(options.maxAccessLoweringMm??height)||p.z>lift+world.radius||p.x>world.radius||Math.abs(p.tiltDeg)>170)continue;
    const cell=[p.x,p.z-lift,p.tiltDeg*weight].map(v=>Math.round(v/.15)).join('|');if(cells.has(cell)||!clear(p))continue;
    if(!edge(a,p).clear)continue;cells.add(cell);const i=nodes.length;nodes.push({pose:p,parent});diagnostics.nodes++;
    const outside=world.outsidePose(p);if(edge(p,outside).clear&&accept([...path(i),outside]))break;
   }
   if(solution)break;
  }
 }catch(error){if(error.message!=='COORDINATED_ACCESS_BUDGET')throw error;diagnostics.budgetExhausted=true;}
 return solution?finish('SAMPLE_CLEAR',solution):finish('BLOCKED',{reason:diagnostics.budgetExhausted?'COORDINATED_ACCESS_BUDGET':'NO_COORDINATED_ROUTE_IN_SEARCH'});
}
function modelThickness(payload){return Math.max(.1,payload.model.thicknessMm??1);}

/** Complete access for an already validated fold. Existing complete endpoints
 * are retained verbatim. Backgauge and exterior transfers remain caller duties. */
export function completeCoordinatedStage({model,catalog,stage,options={},obstacles=[]},guard=()=>{}){
 if(stage.foldSamplesClear!==true||!stage.foldSamples?.length||stage.foldSamples.some(s=>s.status!=='SAMPLE_CLEAR'))return{feasible:false,reason:'FOLD_NOT_VALIDATED'};
 const result=structuredClone(stage),coordinated={...result.coordinatedMotion},diagnostics=[];let changed=false;
 for(const [name,amounts]of [['entry',stage.from],['exit',stage.to]]){
  const existing=result.access?.[name];if(existing?.path?.status==='SAMPLE_CLEAR'&&existing?.ram?.status==='SAMPLE_CLEAR')continue;
  const found=searchCoordinatedAccess({model,catalog,bendId:stage.bendId,config:stage.config,amounts,options,obstacles},guard);diagnostics.push({endpoint:name,...found.diagnostics});
  if(found.status!=='SAMPLE_CLEAR')return{feasible:false,reason:found.reason,endpoint:name,search:found,diagnostics};
  changed=true;
  const {motion,validation}=found,frames=motion.frames,keyframes=frames.map((f,i)=>({phase:i?'COORDINATED_ACCESS':'AT_STATION',pose:f.pose})),last=frames.at(-1),zero=frames[0];
  result.access??={};result.access[name]={endpoint:name==='entry'?'start':'end',amounts:{...amounts},config:{...stage.config,ramGapMm:last.ramGapMm},
   path:{status:'SAMPLE_CLEAR',pivot:motion.pivot,keyframes,insertionKeyframes:keyframes.slice().reverse(),evaluations:validation.samples,
    wholePartOutsideObstacleFront:true,limits:{pointStepBoundMm:validation.maxRelativePointMotionBoundMm},coordinateNote:'Variable ram is represented by stage.coordinatedMotion.'},
   ram:{status:'SAMPLE_CLEAR',method:'COORDINATED_RAM_AND_PART',stationaryPart:false,samples:validation.samples},openingMethod:'COORDINATED_RAM_AND_PART'};
  if(name==='entry'){coordinated.entry={...motion,frames:frames.slice().reverse()};coordinated.approach={pivot:motion.pivot,frames:[zero,zero]};result.gaugeRamGapMm=0;}
  else{coordinated.retract={pivot:motion.pivot,frames:frames.slice(0,2)};coordinated.exit={pivot:motion.pivot,frames:frames.length>2?frames.slice(1):[last,last]};}
 }
 if(!changed)return{feasible:true,stage:result,diagnostics,requiresBackgaugeValidation:!result.backgaugeVerified};
 result.coordinatedMotion=coordinated;result.status='SAMPLE_CLEAR';result.accessVerified=true;result.ramVerified=true;
 result.limits={...result.limits,accessVerified:true,ramVerified:true,
  accessFamily:'independently sampled coordinated ram and rigid part motion; fixed station Y',continuousCollisionDetection:false};
 const startWorld=createAccessWorld({model,catalog,bendId:stage.bendId,config:stage.config,amounts:stage.from,options}),endWorld=createAccessWorld({model,catalog,bendId:stage.bendId,config:stage.config,amounts:stage.to,options});
 const closedTipAtStartMm=startWorld.scene.placement.punchTipZ,closedTipAtEndMm=endWorld.scene.placement.punchTipZ,gapFromEndMm=result.access.exit.config.ramGapMm;
 result.retraction={...result.retraction,method:'COORDINATED_RAM_AND_PART',stationaryPart:false,additionalRetractMm:null,fixedPartShadowPolicyApplicable:false,
  closedTipAtStartMm,closedTipAtEndMm,gapFromEndMm,recommendedTipZMm:closedTipAtEndMm+gapFromEndMm,
  maximumEntryRamGapMm:Math.max(result.access.entry.config.ramGapMm,...(coordinated.entry?.frames??[]).map(f=>f.ramGapMm)),
  numericalClearanceMm:options.clearanceMm??.15,physicalOpeningLimitMm:options.maxCoordinatedRamGapMm??null,
  note:'Explicit jointly checked trajectory; fixed-part shadow allowance is not a clearance proof for coordinated motion.'};
 result.openTipZMm=result.retraction.recommendedTipZMm;result.openGapMm=gapFromEndMm;
 result.completeGeometricRouteSampleClear=false;result.backgaugeVerified=false;
 return{feasible:true,stage:result,diagnostics,requiresBackgaugeValidation:true};
}
