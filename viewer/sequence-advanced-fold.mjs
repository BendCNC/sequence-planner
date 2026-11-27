/** Fold-only authority for search. Passing this never approves access or a plan. */
import {sceneWithAmounts,isPart,checkScene} from './sequence-scene.mjs';
import {buildOperatorSupport,evaluateOperatorSupport} from './sequence-operator-support.mjs';
import {assessCoverage} from './coverage.mjs';
import {placementOriginFlat} from './machine.mjs';
import {coverageMetrics} from './tool-setup-search.mjs';
import {assessToolSupport} from './tool-support-policy.mjs';
import {strokeTransitionError} from './sequence-stroke.mjs';
const nodeFoldPool=globalThis.process?.versions?.node?await import('./sequence-fold-pool.mjs'):null;

const poseKey=(model,amounts)=>model.bends.map(b=>Object.is(amounts[b.id],-0)?'-0':amounts[b.id]).join('|');
export function evaluateAdvancedFold({model,catalog,from,to,bendId,config,options={},preparedSupport,operationId,sampleCache},guard=()=>{}){
 const bend=model.bends.find(b=>String(b.id)===String(bendId));
 if(!bend)throw new Error('Unknown active bend');
 const strokeError=strokeTransitionError(model,bend,from,to);
 if(strokeError)return{feasible:false,reason:'INVALID_STROKE_TRANSITION',error:strokeError};
 const prepared=preparedSupport??buildOperatorSupport(model,bend.id),baseConfig={...config,bendId:bend.id,amount:0,ramGapMm:0,contactGapMm:options.contactGapMm??.3};
 const initial=sceneWithAmounts(model,catalog,baseConfig,from),coverage=assessCoverage({materialIntervals:initial.coverage.intervals,segments:baseConfig.segments,activeBankId:baseConfig.activeBankId,exact:initial.coverage.exact}),toolSupport=assessToolSupport(coverage,options);
 if(!toolSupport.accepted)return{feasible:false,reason:'INSUFFICIENT_TOOL_SUPPORT',toolSupport};
 const failure=(check,amounts,fraction)=>({feasible:false,reason:'FOLD_INTERFERENCE',fraction,hits:check.hits,witness:check.hits[0]?.point?{bendId:bend.id,phase:'fold',config:baseConfig,amounts,collision:check.hits[0],clearanceMm:options.clearanceMm??.15}:undefined});
 // Neighbouring probes of the same partial stroke frequently ask about the
 // very same pose. Only exact states at this mount share sampled solid and
 // operator checks. The calling search discards the map before changing banks.
 const stateKey=amounts=>poseKey(model,amounts);
 const endpoints=new Map();
 for(const [fraction,amounts,scene]of [[0,from,initial],[1,to,sceneWithAmounts(model,catalog,baseConfig,to)]]){
  guard();const key=stateKey(amounts),known=sampleCache?.get(key);
  if(known?.check?.status==='SAMPLE_CLEAR'&&known.support.passes)continue;
  const support=evaluateOperatorSupport(prepared,scene);if(!support.passes)return{feasible:false,reason:'MAIN_BRANCH_NOT_FRONT',fraction,operatorSupport:support};
  const checked=checkScene(model,scene,{clearanceMm:options.clearanceMm??.15,onlyIds:new Set(scene.bodies.filter(b=>!isPart(b)).map(b=>b.id))});
  if(checked.status!=='SAMPLE_CLEAR')return failure(checked,amounts,fraction);
  endpoints.set(key,{scene,support});
 }
 const angle=Math.abs((to[bend.id]-from[bend.id])*bend.signedAngle)*180/Math.PI,count=Math.max((options.foldSamples??21)-1,Math.ceil(angle/(options.foldStepDegrees??5))),foldSamples=[];
 const samples=Array.from({length:count+1},(_,i)=>{
  const fraction=i/count,amounts=Object.fromEntries(model.bends.map(b=>[b.id,from[b.id]+fraction*(to[b.id]-from[b.id])]));
  const key=stateKey(amounts);return{fraction,amounts,key,known:sampleCache?.get(key)};
 });
 const evaluate=i=>{
  guard();const {amounts,key}=samples[i],endpoint=endpoints.get(key),scene=endpoint?.scene??sceneWithAmounts(model,catalog,baseConfig,amounts);
  const support=endpoint?.support??evaluateOperatorSupport(prepared,scene);
  return support.passes?{support,check:checkScene(model,scene,{clearanceMm:options.clearanceMm??.15,projectJointBounds:true}),punchTipZMm:scene.placement.punchTipZ}:{support};
 };
 const indices=[];
 for(let i=1;i<count;i++)if(!samples[i].known)indices.push(i);
 const parallel=indices.length&&count>2&&nodeFoldPool?.foldPoolSize()>0?nodeFoldPool.evaluateFoldSamples({model,catalog,baseConfig,bendId:bend.id,from,to,count,clearanceMm:options.clearanceMm??.15,indices},{local:evaluate}):null;
 for(let i=0;i<=count;i++){
  guard();const {fraction,amounts,key,known}=samples[i],sample=known??parallel?.get(i)??evaluate(i),{support,check}=sample;
  if(sampleCache&&!known)sampleCache.set(key,sample);
  if(!support.passes)return{feasible:false,reason:'MAIN_BRANCH_NOT_FRONT',fraction,operatorSupport:support};
  if(check.status!=='SAMPLE_CLEAR')return failure(check,amounts,fraction);
  foldSamples.push({progress:fraction,amounts,status:'SAMPLE_CLEAR',operatorSupport:support,punchTipZMm:sample.punchTipZMm,selfCollisionScope:check.selfCollisionScope,jointExceptions:check.jointExceptions,closedSolidContainmentAvailableForJointTargets:check.closedSolidContainmentAvailableForJointTargets});
 }
 return{feasible:true,stage:{id:operationId??`${bend.id}:${from[bend.id]}>${to[bend.id]}`,bendId:bend.id,from:{...from},to:{...to},config:baseConfig,flip:Boolean(config.flip),placementOrigin:placementOriginFlat(bend),status:'FOLD_SAMPLE_CLEAR',foldSamplesClear:true,foldSamples,coverage,coverageMetrics:coverageMetrics(coverage),toolSupport,
  limits:{continuousCollisionDetection:false,operatorGripVerified:false,formingVerified:false,accessVerified:false,physicalMachineTravelVerified:false}}};
}
