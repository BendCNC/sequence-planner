import {shadow,certifiedSeparation} from './lite-separation.mjs';
import {meshSeparationQuery} from './mesh-separation.mjs';
import {movesWithRam} from './tool-geometry.mjs';
/** Validate one operation and its front-only entry/exit, with adaptive ram retreat. */
import {sceneWithAmounts,isPart,checkScene,translatedBody,bodyBounds} from './sequence-scene.mjs';
import {buildOperatorSupport,evaluateOperatorSupport} from './sequence-operator-support.mjs';
import {checkFrontAccess} from './sequence-front-access.mjs';
import {checkRecordedOperatorRoute} from './sequence-route-check.mjs';
import {preparePunchShadow,preparePartForPunch,createRequiredTipEvaluator,sampleRigidRoute} from './sequence-retract-geometry.mjs';
import {meshCollider,checkPair} from './collision.mjs';
import {mountedPunch,placementOriginFlat} from './machine.mjs';
import {assessCoverage} from './coverage.mjs';
import {coverageMetrics} from './tool-setup-search.mjs';
import {assessToolSupport} from './tool-support-policy.mjs';
import {strokeTransitionError} from './sequence-stroke.mjs';
const nodeFoldPool = globalThis.process?.versions?.node
  ? await import('./sequence-fold-pool.mjs')
  : null;

// Optional phase timing (UNFOLD_PROFILE_TRANSITIONS=1): totals printed at exit.
const profile=globalThis.process?.env?.UNFOLD_PROFILE_TRANSITIONS?{}:null;
const mark=(name,t0)=>{if(profile){const p=profile[name]??={ms:0,n:0};p.ms+=performance.now()-t0;p.n++;}};
if(profile)process.on('exit',()=>console.error('[transition-profile] '+JSON.stringify(profile)));

function compactRoute(route){return{...route,limits:{...route.limits,maxWorldZ:null},samples:route.samples.map(s=>({
  pose:s.pose,phase:s.phase,status:s.result.status,operatorSupport:s.operatorSupport,
  reason:s.result.reason??null,collision:s.result.collision??null}))};}

function requiredTip(scene,config,path,catalog,tool,options,guard){
  const parts=scene.bodies.filter(isPart),route=sampleRigidRoute({keyframes:path.keyframes,partBodies:parts,pivot:path.pivot,pointStepBoundMm:options.accessPointStepMm});
  const points=new WeakMap();
  const prepared=config.segments.filter(s=>s.kind==='punch').map(segment=>{
    const mounted=mountedPunch(catalog,tool,segment);
    const shadow=preparePunchShadow(mounted.tool,{...segment,offsetZ:(segment.offsetZ??0)+mounted.tipOffsetZ},{clearanceMm:options.clearanceMm});
    return createRequiredTipEvaluator(preparePartForPunch(parts,shadow,points),shadow,path.pivot);});
  let maximum=-Infinity,witness=null;
  for(const [sampleIndex,sample]of route.poses.entries()){
    if(sampleIndex%16===0)guard();
    for(const evaluate of prepared){const current=evaluate(sample.pose,maximum);
      if(current.requiredTipZMm!==null&&current.requiredTipZMm>maximum){maximum=current.requiredTipZMm;witness={...current.witness,sampleIndex,pose:sample.pose};}}
  }
  return{requiredTipZNumericalClearanceMm:Number.isFinite(maximum)?maximum:null,witness,
    sampleCount:route.poses.length,maxObservedPointDisplacementBoundMm:route.maxObservedPointDisplacementBoundMm};
}

function checkRam(scene,tip,endpoint,options,guard){
  const current=scene.placement.punchTipZ+scene.cycle.gap,closed=scene.placement.punchTipZ,delta=tip-closed;
  const fixed=scene.bodies.filter(b=>!movesWithRam(b)).map(body=>({body,bounds:bodyBounds(body)}));
  const sweep=meshSeparationQuery();
  const margin=options.clearanceMm+1e-7;
  function separated(lo,hi,otherLo,otherHi){
    const conservativeMargin=margin+32*Number.EPSILON*Math.max(1,Math.abs(lo),Math.abs(hi),Math.abs(otherLo),Math.abs(otherHi));
    return lo-otherHi>conservativeMargin||otherLo-hi>conservativeMargin;
  }
  const punches=scene.bodies.filter(movesWithRam).map(body=>{
    const bounds=bodyBounds(body);
    // The sweep only translates in Z. Boxes farther apart than the query
    // margin in X or Y cannot intersect, contain one another or violate it.
    const close=fixed.filter(other=>[0,1].every(k=>
      !separated(bounds.min[k],bounds.max[k],other.bounds.min[k],other.bounds.max[k]))&&
      sweep(body,other.body,0,0,closed-current,margin,0,0,delta)===null);
    return{body,bounds,close,projection:close.length?shadow(body):null};
  });
  const fixedProjection=new Map();
  const count=Math.max(1,Math.ceil(delta/options.ramStepMm)),samples=[];
  for(let i=0;i<=count;i++){
    guard();const q=endpoint==='start'?1-i/count:i/count,openTipZMm=closed+delta*q;let collision=null;
    const shift=openTipZMm-current;
    outer:for(const {body,bounds,close,projection} of punches){
      let moving;
      for(const other of close){
        if(separated(bounds.min[2]+shift,bounds.max[2]+shift,other.bounds.min[2],other.bounds.max[2]))continue;
        if(!fixedProjection.has(other))fixedProjection.set(other,shadow(other.body));
        if(certifiedSeparation(projection,fixedProjection.get(other),0,shift,options.clearanceMm))continue;
        moving??=meshCollider(translatedBody(body,0,0,shift));
        other.collider??=meshCollider(other.body);
        const hit=checkPair(moving,other.collider,{clearanceMm:options.clearanceMm,queryMode:'clearance'});
        if(hit.status==='INTERFERENCE'){collision=hit;break outer;}}}
    samples.push({openTipZMm,punchTipZMm:openTipZMm,ramGapMm:openTipZMm-closed,status:collision?'INTERFERENCE':'SAMPLE_CLEAR',collision});
    if(collision)return{status:'BLOCKED',reason:'RAM_SWEEP_INTERFERENCE',samples,maxStepMm:delta/count,continuousCollisionDetection:false};
  }
  return{status:'SAMPLE_CLEAR',samples,maxStepMm:delta/count,continuousCollisionDetection:false};
}

// A return flange can sit inside a gooseneck. Raising the punch above the
// entire part may catch that flange. Try rolling the part out toward the
// operator at a lower opening, around the same bend axis (no lateral travel).
function tryRollingAccess({model,catalog,bend,baseConfig,endpoint,options,guard,blockedSweep}){
  const {scene,amounts,path}=endpoint,closed=scene.placement.punchTipZ;
  const halfAngle=Math.abs(amounts[bend.id]*bend.signedAngle)*90/Math.PI;
  if(halfAngle<1)return null;
  const hitGap=blockedSweep.samples.at(-1).ramGapMm,extra=options.additionalRetractMm;
  if(hitGap<=extra)return null;
  const parts=scene.bodies.filter(isPart),rear=Math.max(0,...parts.map(b=>bodyBounds(b).max[0]));
  const radius=Math.max(...parts.map(b=>{
    const bounds=bodyBounds(b);
    return Math.hypot(Math.max(...[bounds.min[0],bounds.max[0]].map(Math.abs)),
      Math.max(...[bounds.min[2],bounds.max[2]].map(z=>Math.abs(z-path.pivot[2]))));
  }));
  const front=Math.min(...scene.bodies.filter(b=>!isPart(b)).map(b=>bodyBounds(b).min[0]));
  const outside=Math.min(-rear,front-radius-options.frontMarginMm-1);
  const frame=(phase,x,z,tiltDeg)=>({phase,pose:{x,y:0,z,tiltDeg}});
  const lift=2*model.thicknessMm,preLift=Math.min(model.thicknessMm,options.accessPointStepMm);
  const fixedIds=new Set(scene.bodies.filter(b=>!isPart(b)&&!movesWithRam(b)).map(b=>b.id));
  const blockedPrefixGaps=new Set();
  let blockedPrefixAtEveryGap=false;
  for(const angle of [halfAngle,-halfAngle])for(const reach of [.8,1]){
    if(blockedPrefixAtEveryGap)return null;
    const frames=[frame('AT_STATION',0,0,0),frame('LIFT_AT_STATION',0,preLift,0),
      frame('ROLL_TOWARD_OPERATOR',-rear*reach,lift,angle),frame('WITHDRAW_FRONT',outside,lift,angle)];
    let blockedAtEveryGap=false;
    const check=gap=>{
      guard();
      // All four rolls share the same stationary pose and initial lift.
      if(blockedAtEveryGap||blockedPrefixGaps.has(gap))return{status:'BLOCKED'};
      const open=sceneWithAmounts(model,catalog,{...baseConfig,ramGapMm:gap},amounts);
      const route=checkRecordedOperatorRoute({model,bendId:bend.id,scene:open,keyframes:frames,pivot:path.pivot,
        clearanceMm:options.clearanceMm,frontMarginMm:options.frontMarginMm});
      const blocked=route.blockedSample;
      if(blocked){
        // Changing ram height cannot move the part along a given roll, the
        // lower tooling, or the operator-side test. A punch/ram hit applies
        // only at the sampled gap and never rules out another opening.
        const fixed=blocked.result.reason==='OPERATOR_MAIN_BRANCH_FRONT_CONSTRAINT'||
          fixedIds.has(blocked.result.collision?.bodyIds?.[1]);
        blockedAtEveryGap||=fixed;
        if(blocked.phase==='AT_STATION'||blocked.phase==='LIFT_AT_STATION'){
          blockedPrefixGaps.add(gap);
          blockedPrefixAtEveryGap||=fixed;
        }
      }
      return route;
    };
    // Search only below the witnessed obstruction. Feasibility can be an
    // interval: both the opening and the opening plus allowance must pass.
    const step=Math.max(options.ramStepMm,(hitGap-extra)/12);
    for(let gap=0;!blockedAtEveryGap&&gap<hitGap;gap+=step){
      const route=check(gap);
      if(route.status!=='SAMPLE_CLEAR')continue;
      let low=Math.max(0,gap-step),required=gap;
      for(let i=0;i<4&&required-low>.1;i++){
        const mid=(low+required)/2;
        if(check(mid).status==='SAMPLE_CLEAR')required=mid;else low=mid;
      }
      const openGap=required+extra;
      if(openGap>=hitGap)break;
      const checked=extra?check(openGap):check(required);
      if(checked.status!=='SAMPLE_CLEAR')continue;
      const tip=closed+openGap,ram=checkRam(scene,tip,endpoint.endpoint,options,guard);
      if(ram.status!=='SAMPLE_CLEAR')break;
      return{endpoint:endpoint.endpoint,config:{...baseConfig,ramGapMm:openGap},amounts:{...amounts},
        path:compactRoute(checked),ram,openTipZMm:tip,requiredTipZNumericalClearanceMm:closed+required,
        openingMethod:'SAMPLED_FRONT_ROLL',sampleCount:checked.samples.length};
    }
  }
  return null;
}

const evidenceFor=(bendId,baseConfig,clearanceMm)=>(phase,amounts,collision,pose={})=>collision?.point?{
  bendId,phase,config:baseConfig,amounts,collision,clearanceMm,...pose}:undefined;
const WORKER_OPTION_KEYS=['plannerMode','clearanceMm','contactGapMm','frontMarginMm','maxAccessSamples','accessPointStepMm','additionalRetractMm','ramStepMm'];
const workerOptions=o=>Object.fromEntries(WORKER_OPTION_KEYS.map(k=>[k,o[k]]));

/** Phase 1 for one endpoint: front withdrawal route and the punch opening it needs. */
export function endpointRoute({catalog,baseConfig,scene,endpoint,amounts,options,guard=()=>{},bendId}){
  const evidence=evidenceFor(bendId,baseConfig,options.clearanceMm);
  const tool=catalog.tools.find(t=>String(t.id)===String(baseConfig.punchId));
  const parts=scene.bodies.filter(isPart),obstacles=scene.bodies.filter(b=>!isPart(b)&&!movesWithRam(b));
  if(!obstacles.length)return{failure:{feasible:false,reason:'NO_FIXED_TOOL_GEOMETRY'}};
  const fixedBounds=obstacles.map(bodyBounds),partBounds=parts.map(bodyBounds);
  const minZ=Math.min(...partBounds.map(b=>b.min[2])),top=Math.max(...fixedBounds.map(b=>b.max[2]));
  const completeLift=Math.max(0,top-minZ+options.contactGapMm),limit=Math.max(options.contactGapMm,completeLift+options.accessPointStepMm);
  const lifts=[...new Set([0,options.contactGapMm,2,5,10,15,20,30,40,50,60,80,100,150,200,completeLift,limit].filter(v=>v<=limit))].sort((a,b)=>a-b);
  const allObstacleBounds=scene.bodies.filter(b=>!isPart(b)).map(bodyBounds),frontX=Math.min(...allObstacleBounds.map(b=>b.min[0]))-options.frontMarginMm;
  const search=checkFrontAccess({partBodies:parts,obstacles,clearanceMm:options.clearanceMm,frontX,routeOnly:options.plannerMode!=='full',
    frontMarginMm:options.frontMarginMm,liftCandidatesMm:lifts,maxSamples:options.maxAccessSamples,
    stepMm:options.accessPointStepMm,minStepMm:Math.min(.25,options.accessPointStepMm)});
  guard();
  if(search.status!=='SAMPLE_CLEAR'){
    const blocked=search.attempts?.map(a=>a.blockedSample).find(s=>s?.result?.collision);
    return{failure:{feasible:false,reason:search.reason?.includes('BUDGET')?'ACCESS_SEARCH_BUDGET':'NO_FRONT_ROUTE_IN_TESTED_LIFTS',
      endpoint,search:{reason:search.reason,attempts:search.attempts,evaluations:search.evaluations},
      witness:evidence(endpoint==='start'?'entry':'exit',amounts,blocked?.result.collision,{partPose:blocked?.pose,fixedOnly:true})}};
  }
  const pivot=[0,scene.placement.stationYmm??0,scene.placement.neutralHeight];
  const path={...search,pivot,keyframes:search.keyframes.map(f=>({...f,pose:{...f.pose,y:0,tiltDeg:0}}))};
  const required=requiredTip(scene,baseConfig,path,catalog,tool,options,guard);
  return{data:{endpoint,amounts,path,required}};
}

/** Phase 2 for one endpoint: recorded route and ram sweep at the shared opening. */
export function endpointAccess({model,catalog,bend,baseConfig,e,recommendedTipZMm,options,guard=()=>{}}){
  const evidence=evidenceFor(bend.id,baseConfig,options.clearanceMm);
  const endpointConfig={...baseConfig,ramGapMm:recommendedTipZMm-e.scene.placement.punchTipZ};
  const scene=sceneWithAmounts(model,catalog,endpointConfig,e.amounts);
  const checked=checkRecordedOperatorRoute({model,bendId:bend.id,scene,keyframes:e.path.keyframes,pivot:e.path.pivot,maxWorldZ:Infinity,clearanceMm:options.clearanceMm,frontMarginMm:options.frontMarginMm});
  guard();if(checked.status!=='SAMPLE_CLEAR')return{failure:{feasible:false,reason:'ADAPTIVE_ACCESS_INTERFERENCE',endpoint:e.endpoint,blockedSample:checked.blockedSample,
    witness:evidence(e.endpoint==='start'?'entry':'exit',e.amounts,checked.blockedSample?.result.collision,
      {config:endpointConfig,partPose:checked.blockedSample?.pose,pivot:e.path.pivot})}};
  const ram=checkRam(scene,recommendedTipZMm,e.endpoint,options,guard);
  if(ram.status!=='SAMPLE_CLEAR'){
    const rolling=tryRollingAccess({model,catalog,bend,baseConfig,endpoint:e,options,guard,blockedSweep:ram});
    if(!rolling){
      const blocked=ram.samples.at(-1);
      return{failure:{feasible:false,reason:'RAM_SWEEP_INTERFERENCE',endpoint:e.endpoint,ram,
        witness:evidence(e.endpoint==='start'?'approach':'retract',e.amounts,blocked.collision,
          {config:{...baseConfig,ramGapMm:blocked.ramGapMm}})}};
    }
    return{access:rolling};
  }
  return{access:{endpoint:e.endpoint,config:endpointConfig,amounts:{...e.amounts},
    path:compactRoute(checked),ram,openTipZMm:recommendedTipZMm,...e.required}};
}

export function evaluateSequenceTransition(args,guard=()=>{}){
  const t0=performance.now();
  try{return evaluateSequenceTransitionImpl(args,guard);}finally{mark('total',t0);}
}

// Rejection at an endpoint is definitive. A clear screening result carries
// no stage, fold samples or authorization to skip the full transition.
export function screenTransitionEndpoints(args,guard){
  return evaluateSequenceTransitionImpl(args,guard,true);
}
function evaluateSequenceTransitionImpl({model,catalog,from,to,bendId,config,operationId,options,preparedSupport},guard,endpointsOnly=false){
  let t=performance.now();
  const bend=model.bends.find(b=>String(b.id)===String(bendId)),prepared=preparedSupport??buildOperatorSupport(model,bendId);
  const strokeError=strokeTransitionError(model,bend,from,to);
  if(strokeError)return{feasible:false,reason:'INVALID_STROKE_TRANSITION',error:strokeError};
  const baseConfig={...config,bendId:bend.id,amount:0,ramGapMm:0,contactGapMm:options.contactGapMm};
  const evidence=(phase,amounts,collision,pose={})=>collision?.point?{
    bendId:bend.id,phase,config:baseConfig,amounts,collision,clearanceMm:options.clearanceMm,...pose}:undefined;
  const initialScene=sceneWithAmounts(model,catalog,baseConfig,from);
  const coverage=assessCoverage({materialIntervals:initialScene.coverage.intervals,segments:baseConfig.segments,activeBankId:baseConfig.activeBankId,exact:initialScene.coverage.exact}),metrics=coverageMetrics(coverage);
  const toolSupport=assessToolSupport(coverage,options);
  if(options.allowPartialCoverage!==true&&!toolSupport.accepted)return{feasible:false,reason:'INSUFFICIENT_TOOL_SUPPORT',coverageMetrics:metrics,toolSupport};
  // Cheap reject-only endpoint checks precede the detailed fold and access
  // sampling. Passing these never skips any of the full checks below.
  for(const [fraction,scene]of [[0,initialScene],[1,sceneWithAmounts(model,catalog,baseConfig,to)]]){
    guard();const support=evaluateOperatorSupport(prepared,scene);
    if(!support.passes)return{feasible:false,reason:'MAIN_BRANCH_NOT_FRONT',fraction,operatorSupport:support};
    const check=checkScene(model,scene,{clearanceMm:options.clearanceMm,projectJointBounds:true,onlyIds:new Set(scene.bodies.filter(b=>!isPart(b)).map(b=>b.id))});
    if(check.status!=='SAMPLE_CLEAR')return{feasible:false,reason:'FOLD_INTERFERENCE',fraction,hits:check.hits,
      witness:evidence('fold',fraction===0?from:to,check.hits[0])};
  }
  if(endpointsOnly)return{prechecked:true};
  mark('precheck',t);t=performance.now();
  const delta=Math.abs((to[bend.id]-from[bend.id])*bend.signedAngle)*180/Math.PI;
  const count=Math.max(options.foldSamples-1,Math.ceil(delta/options.foldStepDegrees)),foldSamples=[];
  let firstScene,lastScene;
  // Interior samples are independent; evaluate them on worker threads and
  // consume results in order so the first failing sample is unchanged.
  const amountsAt=i=>Object.fromEntries(model.bends.map(b=>[b.id,from[b.id]+i/count*(to[b.id]-from[b.id])]));
  const sampleAt=i=>{
    const scene=sceneWithAmounts(model,catalog,baseConfig,amountsAt(i)),support=evaluateOperatorSupport(prepared,scene);
    if(!support.passes)return{scene,support};
    return{scene,support,check:checkScene(model,scene,{clearanceMm:options.clearanceMm,projectJointBounds:true}),punchTipZMm:scene.placement.punchTipZ};
  };
  // Every sample is a pure function of its index. With a pool, workers and
  // this thread claim interior samples from a shared counter while this
  // thread first computes both endpoints (whose scenes it keeps). Results are
  // then consumed in sample order, so the first failing sample is unchanged.
  const parallelEnds=new Map(),parallel=options.parallelWithinStage!==false&&count>2&&nodeFoldPool?.foldPoolSize()>1
    ?nodeFoldPool.evaluateFoldSamples({model,catalog,baseConfig,bendId:bend.id,from,to,count,clearanceMm:options.clearanceMm,
      indices:Array.from({length:count-1},(_,k)=>k+1)},{
        before:()=>{parallelEnds.set(0,sampleAt(0));parallelEnds.set(count,sampleAt(count));},
        local:i=>{guard();const {scene,...rest}=sampleAt(i);return rest;}}):null;
  if(parallel)for(const [i,r]of parallelEnds)parallel.set(i,r);
  if(parallel)mark('foldPool',t);
  for(let i=0;i<=count;i++){
    guard();const fraction=i/count,amounts=amountsAt(i);
    const {scene=null,support,check,punchTipZMm}=parallel?.get(i)??sampleAt(i);
    if(!support.passes)return{feasible:false,reason:'MAIN_BRANCH_NOT_FRONT',fraction,operatorSupport:support};
    if(check.status!=='SAMPLE_CLEAR')return{feasible:false,reason:'FOLD_INTERFERENCE',fraction,hits:check.hits,
      witness:evidence('fold',amounts,check.hits[0])};
    foldSamples.push({progress:fraction,amounts,status:'SAMPLE_CLEAR',operatorSupport:support,punchTipZMm,
      selfCollisionScope:check.selfCollisionScope,jointExceptions:check.jointExceptions,
      closedSolidContainmentAvailableForJointTargets:check.closedSolidContainmentAvailableForJointTargets});
    if(i===0)firstScene=scene;if(i===count)lastScene=scene;
  }
  mark('fold',t);
  return completeFoldAccess({model,catalog,bend,from,to,baseConfig,operationId,options,foldSamples,coverage,metrics,toolSupport,firstScene,lastScene},guard);
}

// Continue a fold already sampled by this search with the same model, exact
// setup, options and amounts. Both entry/exit routes and all ram checks follow.
export function finishAdvancedFoldAccess({model,catalog,fold,operationId,options},guard){
  const t=performance.now();
  try{
    const {bendId,from,to,config:baseConfig,foldSamples,coverage,coverageMetrics:metrics,toolSupport}=fold;
    const bend=model.bends.find(b=>b.id===bendId);
    return completeFoldAccess({model,catalog,bend,from,to,baseConfig,operationId,options,foldSamples,coverage,metrics,toolSupport},guard);
  }finally{mark('total',t);}
}

function completeFoldAccess({model,catalog,bend,from,to,baseConfig,operationId,options,foldSamples,coverage,metrics,toolSupport,firstScene,lastScene},guard){
  let t=performance.now();
  const count=foldSamples.length-1,endAmounts=foldSamples[count].amounts;
  firstScene??=sceneWithAmounts(model,catalog,baseConfig,foldSamples[0].amounts);
  lastScene??=sceneWithAmounts(model,catalog,baseConfig,endAmounts);
  // The start and end endpoints are independent until the shared opening is
  // known (phase 1), and again while that opening is checked (phase 2). With
  // a pool the end endpoint runs on a worker (rebuilding the identical scene
  // from the same amounts) while this thread does the start; results are
  // consumed start first, so the first reported failure is unchanged.
  const aside=options.parallelWithinStage!==false&&nodeFoldPool?.foldPoolSize()>1&&count>2;
  const job={bendId:bend.id,baseConfig,amounts:to,sceneAmounts:endAmounts,options:workerOptions(options)};
  const phase1=(endpoint,scene,amounts)=>endpointRoute({catalog,baseConfig,scene,endpoint,amounts,options,guard,bendId:bend.id});
  let endpointData=[];
  if(aside){
    const [start,end]=nodeFoldPool.runAside({model,catalog,msg:{kind:'endpoint-route',...job}},()=>phase1('start',firstScene,from));
    for(const [r,scene]of [[start,firstScene],[end,lastScene]]){if(r.failure)return r.failure;endpointData.push({...r.data,scene});}
  }else for(const [endpoint,scene,amounts]of [['start',firstScene,from],['end',lastScene,to]]){
    guard();const r=phase1(endpoint,scene,amounts);if(r.failure)return r.failure;endpointData.push({...r.data,scene});
  }
  mark('endpointRoute',t);t=performance.now();
  const closedTipAtStartMm=firstScene.placement.punchTipZ,closedTipAtEndMm=lastScene.placement.punchTipZ;
  const requiredTipZNumericalClearanceMm=Math.max(closedTipAtStartMm,closedTipAtEndMm,...endpointData.map(e=>e.required.requiredTipZNumericalClearanceMm??-Infinity));
  const recommendedTipZMm=requiredTipZNumericalClearanceMm+options.additionalRetractMm;
  const access={},phase2=e=>endpointAccess({model,catalog,bend,baseConfig,e,recommendedTipZMm,options,guard});
  const results=aside
    ?nodeFoldPool.runAside({model,catalog,msg:{kind:'endpoint-access',...job,recommendedTipZMm,
      e:{endpoint:'end',amounts:endpointData[1].amounts,path:endpointData[1].path,required:endpointData[1].required}}},()=>phase2(endpointData[0]))
    :null;
  for(const [k,e]of endpointData.entries()){
    guard();const r=results?results[k]:phase2(e);if(r.failure)return r.failure;
    access[e.endpoint==='start'?'entry':'exit']=r.access;
  }
  mark('endpointAccess',t);t=performance.now();
  const exitTip=access.exit.openTipZMm;
  const retraction={recommendedTipZMm:exitTip,requiredTipZNumericalClearanceMm:exitTip-options.additionalRetractMm,closedTipAtStartMm,closedTipAtEndMm,
    gapFromEndMm:exitTip-closedTipAtEndMm,approachStrokeToStartMm:access.entry.openTipZMm-closedTipAtStartMm,
    additionalRetractMm:options.additionalRetractMm,numericalClearanceMm:options.clearanceMm,physicalOpeningLimitMm:null};
  return{feasible:true,stage:{id:operationId,bendId:bend.id,from:{...from},to:{...to},config:baseConfig,flip:Boolean(baseConfig.flip),placementOrigin:placementOriginFlat(bend),
    status:'SAMPLE_CLEAR',foldSamplesClear:true,foldSamples,coverage,coverageMetrics:metrics,toolSupport,access,retraction,
    openTipZMm:exitTip,openGapMm:retraction.gapFromEndMm,
    limits:{continuousCollisionDetection:false,operatorGripVerified:false,fullWidthCoverageVerified:coverage.status==='COVERED',toolSupportPolicyVerified:toolSupport.accepted,formingVerified:false,
      selfCollisionScope:foldSamples[0].selfCollisionScope,
      closedSolidContainmentAvailableForJointTargets:foldSamples.every(s=>s.closedSolidContainmentAvailableForJointTargets),
      jointExceptionChecks:foldSamples.reduce((n,s)=>n+(s.jointExceptions?.length??0),0),
      maxJointExceptionMarginMm:Math.max(0,...foldSamples.flatMap(s=>(s.jointExceptions??[]).map(j=>j.exceptionMarginMm??0))),
      physicalMachineTravelVerified:false,accessFamily:[access.entry,access.exit].some(a=>a.openingMethod==='SAMPLED_FRONT_ROLL')?
        'front withdrawal with checked tilt around the bend axis; fixed station Y':'straight lift then front withdrawal; no tilt or lateral entry'}},coverage,metrics};
}
