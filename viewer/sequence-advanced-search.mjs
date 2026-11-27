/** Generic reverse search with geometry-derived partial bends and island banks.
 * No saved sequences, part identifiers, or weakened collision tolerances. */
import {withToolingRules} from './tooling-rules.mjs';
import {validateModel} from './model-validator.mjs';
import {bandPoint} from './solver.mjs';
import {linkJointStrokes} from './machine.mjs';
import {strokeSearchAllowed as strokeAllowed,strokeAmounts,strokeStateError} from './sequence-stroke.mjs';
import {buildOperatorSupport} from './sequence-operator-support.mjs';
import {finishAdvancedFoldAccess} from './sequence-transition.mjs';
import {evaluateAdvancedFold} from './sequence-advanced-fold.mjs';
import {findAdvancedEndpointSetups,streamAdvancedEndpointSetups,advancedConfigKey} from './sequence-advanced-setups.mjs';
import {sceneWithAmounts,isPart,checkScene} from './sequence-scene.mjs';
import {completeCoordinatedStage} from './sequence-access-search.mjs';
import {finishAdvancedSequencePlan} from './sequence-advanced-completion.mjs';
import {rankedTerminalRows} from './sequence-ranked-terminal.mjs';
import {diagnosePairedClosure} from './sequence-paired-closure.mjs';

export const advancedDefaults={timeBudgetMs:300000,maxAdvancedStates:160,maxAdvancedFoldChecks:4000,maxAdvancedEndpointSetups:16,
 maxAdvancedPartialBends:1,maxAdvancedChains:8,allowPartialBends:true,allowMirroredToolProfiles:true,
 clearanceMm:.15,contactGapMm:.3,additionalRetractMm:6.35,foldSamples:21,foldStepDegrees:5,accessPointStepMm:2,ramStepMm:2,
 maxAccessSamples:2500,frontMarginMm:10,maxVertices:50000,partialThresholdResolutionDegrees:.25,partialReserveFraction:.1};
class SearchBudget extends Error{constructor(reason){super(reason);this.reason=reason;}}
const normalized=model=>linkJointStrokes({...model,bends:model.bends.map(b=>b.band.flatPositions?b:{...b,band:{...b.band,flatPositions:b.band.parameters.flatMap((v,i,a)=>i%3?[]:bandPoint(b,v,a[i+1],a[i+2],0))}})});
export {normalized as normalizedAdvancedModel};
const pause=()=>new Promise(resolve=>setTimeout(resolve,0));

/** Find a lower endpoint in the connected sampled closing interval containing
 * target. This is only a heuristic bracket: every proposed whole stroke is
 * subsequently checked, so no global monotonicity is assumed. */
export function discoverPartialPredecessors({target=1,angleDegrees,check,guard=()=>{},resolutionDegrees=.25,reserveFraction=.1}){
 const probes=[],thresholds=[];let blocked=0,clear=null;
 for(const f of[.5,.75,.875,.9375,.96875]){
  guard();const q=target*f,result=check(q);probes.push({amount:q,feasible:result.feasible});
  if(result.feasible){clear=q;break;}blocked=q;
 }
 if(clear===null)return{amounts:[],probes};
 for(let i=0;i<12&&(clear-blocked)*angleDegrees>resolutionDegrees;i++){
  guard();const mid=(blocked+clear)/2,r=check(mid);probes.push({amount:mid,feasible:r.feasible});if(r.feasible)clear=mid;else blocked=mid;
 }
 const threshold=clear,remaining=(target-threshold)*angleDegrees,reserve=Math.max(1,remaining*reserveFraction),robust=Math.ceil(threshold*angleDegrees+reserve)/angleDegrees;
 thresholds.push(robust,clear,(clear+target)/2);
 return{amounts:[...new Set(thresholds.filter(q=>q>0&&q<target-1e-7))],probes,thresholdBracket:[blocked,clear],reserveDegrees:reserve};
}

export async function finishStageAccess(model,catalog,stage,operationId,options,guard){
 const normal=finishAdvancedFoldAccess({model,catalog,fold:stage,operationId,options},guard);
 if(normal.feasible||!['NO_FRONT_ROUTE_IN_TESTED_LIFTS','ACCESS_SEARCH_BUDGET','RAM_SWEEP_INTERFERENCE','ADAPTIVE_ACCESS_INTERFERENCE'].includes(normal.reason))return normal;
 if(options.completeStage)return options.completeStage(stage,{model,catalog,options,guard,normalFailure:normal});
 return completeCoordinatedStage({model,catalog,stage,options},guard);
}

export async function planAdvancedSequence(payload={},onProgress=()=>{}){
 return withToolingRules(payload,onProgress,runAdvancedSequence);
}
async function runAdvancedSequence({model:input,catalog,options={}},onProgress){
 const started=performance.now(),o={...advancedDefaults,...options},d={method:'REVERSE_PARTIAL_ISLAND_SEARCH',statesExpanded:0,foldChecks:0,endpointChecks:0,chainsFound:0,cacheHits:0,rejections:{},partialDiscoveries:[],endpointSetups:[],searchComplete:false,globalOptimumProven:false};
 const result=(status,extra={})=>({status,stages:[],allStagesSampleClear:false,completeGeometricRouteSampleClear:false,completeSequenceVerified:false,completeManufacturingVerified:false,...extra,diagnostics:{...d,...extra.diagnostics,elapsedMs:performance.now()-started}});
 let model;try{const validation=validateModel(input);if(validation.vertices>o.maxVertices)return result('SEARCH_BUDGET_EXHAUSTED',{diagnostics:{budgetReason:'mesh-vertex-budget'}});model=normalized(input);
  if(model.bends.length&&!Array.isArray(catalog?.tools))throw new Error('A tooling catalog is required');
  if(options.operationTemplates)throw new Error('Advanced automatic order search does not override explicit operation templates');
  if(options.fixedStations!==undefined&&!options.setups?.length&&!options.initialSegments)throw new Error('Advanced search requires explicit setups for fixedStations');
  for(const key of['timeBudgetMs','foldStepDegrees','accessPointStepMm','ramStepMm'])if(!(Number.isFinite(o[key])&&o[key]>0))throw new Error(`Invalid ${key}`);
  for(const key of['clearanceMm','contactGapMm','additionalRetractMm'])if(!(Number.isFinite(o[key])&&o[key]>=0))throw new Error(`Invalid ${key}`);
  if(!Number.isInteger(o.foldSamples)||o.foldSamples<2)throw new Error('Invalid foldSamples');
  const strokeError=strokeStateError(model,Object.fromEntries(model.bends.map(b=>[b.id,0])));if(strokeError)throw new Error(strokeError);
 }catch(error){return result('UNSUPPORTED_INPUT',{diagnostics:{error:error.message}});}
 if(!model.bends.length)return result('FLAT_NO_BENDS',{allStagesSampleClear:true});
 const guard=()=>{if(o.signal?.aborted)throw new SearchBudget('cancelled');if(performance.now()-started>=o.timeBudgetMs)throw new SearchBudget('time-budget');if(d.foldChecks>=o.maxAdvancedFoldChecks)throw new SearchBudget('fold-check-budget');};
 const emit=(phase,more={})=>onProgress({phase,...more,statesExpanded:d.statesExpanded,transitionsChecked:d.foldChecks,elapsedMs:performance.now()-started});
 const amountsKey=a=>model.bends.map(b=>Number(a[b.id]).toFixed(8)).join('|'),zero=Object.fromEntries(model.bends.map(b=>[b.id,0])),one=Object.fromEntries(model.bends.map(b=>[b.id,1]));
 const terminalRows=rankedTerminalRows(input,catalog,o);
 const rootRow=(amounts,bend)=>amounts===one?terminalRows?.get(bend.id):null;
 const supports=new Map(model.bends.map(b=>[String(b.id),buildOperatorSupport(model,b.id)])),endpointCache=new Map(),foldCache=new Map(),completionCache=new Map();let bestFoldChain=null,completed=null;
 const reject=r=>{const reason=r.reason??r.status??'UNKNOWN';d.rejections[reason]=(d.rejections[reason]??0)+1;if(r.witness&&!(d.witnesses??=[]).some(w=>w.bendId===r.witness.bendId&&w.collision.bodyIds.join('|')===r.witness.collision.bodyIds.join('|'))&&d.witnesses.length<24)d.witnesses.push(r.witness);};
 async function* generatedSetups(amounts,bend){
  const key=amountsKey(amounts)+'|'+bend.id;
  if(endpointCache.has(key)){d.cacheHits++;yield* endpointCache.get(key);return;}
  guard();emit('advanced-endpoint-setups',{bendId:bend.id,amounts});await pause();
  const observed={},stream=streamAdvancedEndpointSetups({model,catalog,bendId:bend.id,amounts,options:o,terminalRow:rootRow(amounts,bend)},guard,observed);
  let consumed=0,found;
  try{
   for(;;){
    const step=stream.next();if(step.done){found=step.value;break;}
    consumed++;yield step.value;
   }
   endpointCache.set(key,found.configs);
   yield* found.configs.slice(consumed);
  }finally{
   stream.return();
   const diagnostics=found?.diagnostics??observed.diagnostics;
   if(diagnostics){
    d.endpointChecks+=diagnostics.componentsTested+diagnostics.combinationsTested;
    d.endpointSetups.push({bendId:bend.id,amounts,retained:(found??observed).configs.length,...diagnostics,
     ...(!found?{stoppedAfterAcceptedPrefix:true}:{})});
   }
  }
 }
 async function setups(amounts,bend){
  const configs=[];for await(const config of generatedSetups(amounts,bend))configs.push(config);return configs;
 }
 // Tool continuity: the reverse search knows the stages that follow, so the
 // predecessor first tries their exact mounts (no change at all), then the
 // same punch/die pair, die, punch. Accepted mounts still pass every check.
 const mountKey=c=>JSON.stringify([c.punchId,c.dieId,c.stationYmm??0,c.activeBankId??null,c.segments.map(s=>[s.kind,s.lengthMm,s.centerY,s.mirror??false,s.offsetX??0,s.offsetZ??0,s.bankId??null,s.punchId??null])]);
 async function* continuitySetups(amounts,bend,suffix){
  if(o.advancedToolContinuity===false||!suffix.length){yield* generatedSetups(amounts,bend);return;}
  const mounts=[],seenMounts=new Set();for(const s of suffix){const c=s.config;if(!c?.segments)continue;const k=mountKey(c);if(!seenMounts.has(k)&&mounts.length<3){seenMounts.add(k);mounts.push(c);}}
  const key=amountsKey(amounts)+'|'+bend.id+'|reuse|'+[...seenMounts].join('#');let reused=endpointCache.get(key);
  if(!reused){guard();
   reused=mounts.length?findAdvancedEndpointSetups({model,catalog,bendId:bend.id,amounts,options:{...o,initialSegments:undefined,initialSetup:undefined,setups:mounts.map(({flip,bendId,...m})=>m)}},guard).configs:[];
   d.continuityReuse=(d.continuityReuse??0)+reused.length;endpointCache.set(key,reused);}
  const next=suffix[0]?.config,rank=c=>!next?4:mountKey(c)===mountKey(next)?0:c.punchId===next.punchId&&c.dieId===next.dieId?1:c.dieId===next.dieId?2:c.punchId===next.punchId?3:4;
  // Exact copies of the next mount sort first and both flips were checked
  // above. Only enumerate other stock if these candidates fail downstream.
  const seen=new Set();
  for(const c of reused)if(rank(c)===0){
   const k=advancedConfigKey(c);if(seen.has(k))continue;
   seen.add(k);yield c;
  }
  // Both flips of the exact next mount were already tested above. The
  // next ranked group uses the same punch and die. Its reused mounts
  // precede generated mounts, whose retained order comes from the stream.
  // For caller-supplied banks keep the complete original sort: custom
  // segment identifiers can differ between geometrically identical mounts.
  if(o.setups?.length||o.initialSegments||o.initialSetup){
   const remaining=[];
   for(const c of [...reused,...await setups(amounts,bend)]){
    const key=advancedConfigKey(c);if(seen.has(key))continue;seen.add(key);remaining.push(c);
   }
   remaining.sort((a,b)=>rank(a)-rank(b));yield* remaining;return;
  }
  const others=[];
  for(const c of reused){
   const k=advancedConfigKey(c);if(seen.has(k))continue;seen.add(k);
   if(rank(c)===1)yield c;else others.push(c);
  }
  for await(const c of generatedSetups(amounts,bend)){
   const k=advancedConfigKey(c);if(seen.has(k))continue;seen.add(k);
   if(rank(c)<=1)yield c;else others.push(c);
  }
  others.sort((a,b)=>rank(a)-rank(b));
  yield* others;
 }
 let proposedFold,pipeline=null,attemptedPipeline=false;
 const preliminary=new Map();
 class RetryFull extends Error{}

 function fold(amounts,bend,predecessor,config,sampleCache){
  const from=strokeAmounts(bend,{...amounts,[bend.id]:predecessor}),key=amountsKey(from)+'>'+amountsKey(amounts)+'|'+advancedConfigKey(config);
  if(foldCache.has(key)){d.cacheHits++;return foldCache.get(key);}guard();d.foldChecks++;
  if(pipeline){
   if(predecessor!==0)throw new RetryFull();
   const input={bendId:bend.id,from,to:amounts,config},quick=evaluateAdvancedFold({model,catalog,...input,options:{...o,foldSamples:2,foldStepDegrees:3600},preparedSupport:supports.get(String(bend.id))},guard);
   if(!quick.feasible)throw new RetryFull();
   preliminary.set(quick.stage,pipeline.submit(input));return quick;
  }
  const r=proposedFold?.amounts===amounts&&proposedFold.q===predecessor&&proposedFold.config===config?proposedFold.result:evaluateAdvancedFold({model,catalog,bendId:bend.id,from,to:amounts,config,options:o,preparedSupport:supports.get(String(bend.id)),sampleCache},guard);foldCache.set(key,r);if(!r.feasible)reject(r);return r;
 }
 const terminalWorker=o.speculateTerminalAccess&&!o.completeStage&&!o.foldOnlyDiagnostic&&globalThis.process?.versions?.node
   ?await import('./sequence-terminal-access-worker.mjs'):null;
 let terminal=null;
 function startClosingAccess(stage){
  if(terminal||!terminalWorker)return;
  const {signal,transitionCache,completeCandidate,...options}=o;
  const job=terminalWorker.startTerminalAccess({model,catalog,stage,options},Date.now()+o.timeBudgetMs-(performance.now()-started));
  if(job)terminal={stage,job};
 }
 async function complete(chain){
  const proposed=Boolean(pipeline);
  if(pipeline){
   if(!await pipeline.wait(guard))throw new RetryFull();
   pipeline.close();pipeline=null;
   d.parallelStageAccessAccepted=(d.parallelStageAccessAccepted??0)+preliminary.size;
   chain=chain.map(stage=>{
    const job=preliminary.get(stage);if(!job)return stage;
    const result=job.result;
    completionCache.set(amountsKey(stage.from)+'>'+amountsKey(stage.to)+'|'+advancedConfigKey(stage.config),result);
    return result.stage;
   });
  }
  bestFoldChain??=chain;d.chainsFound++;emit('advanced-fold-chain',{operations:chain.length,chain:chain.map(s=>({bendId:s.bendId,from:s.from[s.bendId],to:s.to[s.bendId]}))});await pause();
  if(o.foldOnlyDiagnostic===true)return result('FOLD_ONLY_CHAIN_FOUND',{stages:chain,allStagesSampleClear:false,foldOnlyChainFound:true});
  const accessKey=s=>amountsKey(s.from)+'>'+amountsKey(s.to)+'|'+advancedConfigKey(s.config);
  let ordinary;
  if(terminalWorker&&o.parallelChainAccess!==false){
   const {signal,transitionCache,completeCandidate,...opts}=o;
   const entries=chain.map((stage,index)=>({stage,index})).filter(({stage})=>stage!==terminal?.stage&&!completionCache.has(accessKey(stage)));
   try{
    const {ordinaryStageAccess}=await import('./sequence-fold-pool.mjs');
    ordinary=await ordinaryStageAccess(model,catalog,entries,opts,guard);
    if(ordinary.size)d.parallelStageAccessAccepted=(d.parallelStageAccessAccepted??0)+ordinary.size;
   }catch(error){if(error instanceof SearchBudget)throw error;}
  }
  const stages=[];for(const [i,source]of chain.entries()){
   guard();const key=accessKey(source);let checked=completionCache.get(key);
   if(!checked){emit('advanced-stage-access',{operation:i+1,operations:chain.length,bendId:source.bendId});await pause();
    if(terminal?.stage===source)checked=await terminal.job.take(guard);
    else checked=ordinary?.get(i);
    // A failed speculative attempt returns to the ordinary validator and
    // retains its original witnesses and fallback searches.
    if(!checked?.feasible)checked=await finishStageAccess(model,catalog,source,`advanced-${i+1}`,o,guard);
    completionCache.set(key,checked);
   }
   if(!checked?.feasible){reject(checked??{reason:'STAGE_COMPLETION_FAILED'});if(proposed)throw new RetryFull();return null;}stages.push({...checked.stage,id:`advanced-${i+1}`});
  }
  const candidate=result('CANDIDATE_SEQUENCE',{stages,allStagesSampleClear:true,allStagesOperatorSupportPass:true});emit('advanced-complete-validation',{operations:stages.length});await pause();
  const finish=o.completeCandidate??(c=>finishAdvancedSequencePlan(model,catalog,c,o,onProgress)),checked=await finish(candidate);
  if(checked?.status==='CANCELLED')throw new SearchBudget('cancelled');
  if(checked?.completeGeometricRouteSampleClear===true)return checked;
  reject({reason:checked?.diagnostics?.completionFailure??checked?.status??'COMPLETE_ROUTE_REJECTED'});d.lastCompleteCandidate=checked?.diagnostics;if(proposed)throw new RetryFull();return null;
 }
 async function reverseAfterPartial(stage,suffix,next){
  const search=()=>reverse(stage.from,[stage,...suffix],next);
  if(attemptedPipeline||suffix.length||terminal?.stage!==stage||o.completeCandidate)return search();
  attemptedPipeline=true;
  const {signal,transitionCache,completeCandidate,...opts}=o,pool=await import('./sequence-fold-pool.mjs');
  pipeline=pool.startFoldPipeline(model,catalog,opts);
  if(!pipeline)return search();
  const snapshot=structuredClone(d),searchCaches=[endpointCache,foldCache,completionCache],caches=searchCaches.map(cache=>new Map(cache)),oldBest=bestFoldChain;
  try{
   const answer=await search();if(answer)return answer;
  }catch(error){if(!(error instanceof RetryFull))throw error;}
  finally{pipeline?.close();pipeline=null;preliminary.clear();}
  // A failed proposal resumes at the exact verified predecessor. Restore the
  // logical budgets, rejection order and caches from before proposing it.
  for(const key of Object.keys(d))delete d[key];Object.assign(d,snapshot);bestFoldChain=oldBest;
  for(const [i,cache] of searchCaches.entries()){
   cache.clear();for(const [key,value] of caches[i])cache.set(key,value);
  }
  d.pipelinedFoldFallbacks=(d.pipelinedFoldFallbacks??0)+1;
  return search();
 }
 async function reverse(amounts,suffix,splitBends){
  guard();if(d.statesExpanded>=o.maxAdvancedStates)throw new SearchBudget('state-budget');d.statesExpanded++;
  if(model.bends.every(b=>amounts[b.id]<1e-7))return complete(suffix);
  // Full predecessor strokes first. Partial bends already introduced are kept
  // until other full strokes have had a chance, favouring early pre-bends.
  const active=model.bends.filter(b=>b.strokeLeadId==null&&amounts[b.id]>1e-7).sort((a,b)=>(amounts[a.id]<1?1:0)-(amounts[b.id]<1?1:0));
  const available=[],partialTried=new Set();
  async function tryPartial(bend,configs){
   if(pipeline)throw new RetryFull();
   if(o.allowPartialBends===false||splitBends.size>=o.maxAdvancedPartialBends||splitBends.has(String(bend.id))||amounts[bend.id]<1-1e-7||partialTried.has(String(bend.id)))return null;
   partialTried.add(String(bend.id));
   for await(const config of configs){
    const row=rootRow(amounts,bend),punches=config.segments.filter(s=>s.kind==='punch'),mirror=punches[0]?.mirror??false;
    const bound=row?.selfContact??(punches.length&&punches.every(s=>(s.mirror??false)===mirror&&!s.offsetX&&!s.offsetZ&&
     (s.punchId??config.punchId)===config.punchId)?row?.presentations?.find(w=>w.punchId===config.punchId&&w.flip===Boolean(config.flip)&&w.mirror===mirror):null);
    const sampleCache=new Map();
    let preview;
    // Predict only which already-qualified stroke to check first. A prediction
    // cannot approve a fold, change the discovery bracket or select a stage.
    if(bound&&!terminal&&!suffix.length&&terminalWorker?.terminalWorkerCount()){
     const endpointSamples=new Map();
     const proposal=discoverPartialPredecessors({target:amounts[bend.id],angleDegrees:Math.abs(bend.signedAngle)*180/Math.PI,
      check:q=>q<=bound.amount?{feasible:false}:evaluateAdvancedFold({model,catalog,bendId:bend.id,
       from:strokeAmounts(bend,{...amounts,[bend.id]:q}),to:amounts,config,options:{...o,foldSamples:2,foldStepDegrees:3600},
       preparedSupport:supports.get(String(bend.id)),sampleCache:endpointSamples},guard),
      guard,resolutionDegrees:o.partialThresholdResolutionDegrees,reserveFraction:o.partialReserveFraction});
     if(proposal.amounts.length){
      const q=proposal.amounts[0],guess=evaluateAdvancedFold({model,catalog,bendId:bend.id,from:strokeAmounts(bend,{...amounts,[bend.id]:q}),
       to:amounts,config,options:o,preparedSupport:supports.get(String(bend.id)),sampleCache},guard);
      if(guess.feasible){proposedFold={amounts,config,q,result:guess};startClosingAccess(guess.stage);preview={q,stage:guess.stage};}
     }
    }
    const discovery=discoverPartialPredecessors({target:amounts[bend.id],angleDegrees:Math.abs(bend.signedAngle)*180/Math.PI,check:q=>bound&&q<=bound.amount?{feasible:false}:fold(amounts,bend,q,config,sampleCache),guard,resolutionDegrees:o.partialThresholdResolutionDegrees,reserveFraction:o.partialReserveFraction});
    if(preview&&preview.q!==discovery.amounts[0]&&terminal?.stage===preview.stage){terminal.job.close();terminal=null;}
    if(!discovery.amounts.length)continue;d.partialDiscoveries.push({bendId:bend.id,atAmounts:amounts,config,...discovery});emit('advanced-partial-threshold',{bendId:bend.id,amounts:discovery.amounts,thresholdBracket:discovery.thresholdBracket});await pause();
    for(const q of discovery.amounts){const r=fold(amounts,bend,q,config,sampleCache);if(!r.feasible)continue;
     if(!suffix.length)startClosingAccess(r.stage);
     const next=new Set(splitBends);next.add(String(bend.id));const answer=await reverseAfterPartial(r.stage,suffix,next);if(answer)return answer;}
   }return null;
  }
  for(const bend of active){
   if(!strokeAllowed(model,bend,strokeAmounts(bend,{...amounts,[bend.id]:0}))){reject({reason:'JOINT_STROKE_CONSTRAINT'});continue;}
   // Rigid placement cannot fix self intersection. These reject-only probes
   // avoid spending an inventory search on a stroke already impossible in the
   // part's own geometry; the accepted stroke still receives all fold samples.
   let selfBlocked=false;
   const punch=catalog.tools.find(t=>t.kind==='punch'&&t.status==='ready'),die=catalog.tools.find(t=>t.kind==='die'&&t.status==='ready');
   if(punch&&die)for(const q of[0,.25,.5,.75]){
    guard();const probeConfig={bendId:bend.id,punchId:punch.id,dieId:die.id,bedLengthMm:o.bedLengthMm??catalog.machine?.bedLengthMm,segments:[{id:'probe-punch',kind:'punch',lengthMm:25.4,centerY:0},{id:'probe-die',kind:'die',lengthMm:25.4,centerY:0}],contactGapMm:o.contactGapMm};
    const scene=sceneWithAmounts(model,catalog,probeConfig,strokeAmounts(bend,{...amounts,[bend.id]:q*amounts[bend.id]}));
    if(checkScene(model,{...scene,bodies:scene.bodies.filter(isPart)},{clearanceMm:o.clearanceMm,projectJointBounds:true}).status!=='SAMPLE_CLEAR'){selfBlocked=true;break;}
   }
   if(selfBlocked){d.rejections.PART_SELF_INTERFERENCE=(d.rejections.PART_SELF_INTERFERENCE??0)+1;available.push([bend,null]);continue;}
   // Every full closing stroke crosses its certified witness before the
   // finished pose, for both presentations of every ready punch. A partial
   // that starts beyond that witness still uses the ordinary sampled solver.
   if(rootRow(amounts,bend)){
    (d.certifiedFullTerminalBlocks??=[]).push(rootRow(amounts,bend));available.push([bend,null]);
    if(o.advancedPreferFullOnlyFirst!==true){
     const answer=await tryPartial(bend,generatedSetups(amounts,bend));if(answer)return answer;
    }
    continue;
   }
   const configs=[];available.push([bend,configs]);
   for await(const config of continuitySetups(amounts,bend,suffix)){configs.push(config);const r=fold(amounts,bend,0,config);if(!r.feasible)continue;
    const answer=await reverse(r.stage.from,[r.stage,...suffix],splitBends);if(answer)return answer;if(d.chainsFound>=o.maxAdvancedChains)throw new SearchBudget('complete-candidate-budget');
   }
   // Once a usable final mount is known, test whether a pre-bend opens it
   // before exhaustively searching every other final tool family.
   if(o.advancedPreferFullOnlyFirst!==true){const partial=await tryPartial(bend,configs);if(partial)return partial;}
  }
  if(o.allowPartialBends===false||splitBends.size>=o.maxAdvancedPartialBends)return null;
  for(const [bend,existingConfigs]of available){
   if(splitBends.has(String(bend.id))||amounts[bend.id]<1-1e-7)continue;
   const configs=existingConfigs??await setups(amounts,bend),answer=await tryPartial(bend,configs);if(answer)return answer;
  }
  return null;
 }
 try{
  // Automatically generated advanced banks use one forming punch profile.
  if(!o.setups&&!o.initialSetup&&!o.initialSegments&&o.fixedStations==null){
   const conflict=diagnosePairedClosure({model,catalog,options:o},guard);
   if(conflict)return result(conflict.status,{reason:conflict.reason,diagnostics:{pairedClosure:conflict}});
  }
  completed=await reverse(one,[],new Set());if(terminalWorker)d.workerPool=terminalWorker.foldPoolUsage();if(completed)return{...completed,diagnostics:{...completed.diagnostics,advancedSearch:{...d,elapsedMs:performance.now()-started}}};d.searchComplete=true;}
 catch(error){if(!(error instanceof SearchBudget))throw error;d.budgetReason=error.reason;}
 finally{terminal?.job.close();}
 return result(d.budgetReason==='cancelled'?'CANCELLED':d.budgetReason?'SEARCH_BUDGET_EXHAUSTED':'NO_SEQUENCE_IN_ADVANCED_SEARCH',{foldOnlyChainFound:Boolean(bestFoldChain),foldOnlyCandidate:bestFoldChain??undefined});
}
