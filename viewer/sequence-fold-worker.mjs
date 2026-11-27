import {evaluateAdvancedFold} from './sequence-advanced-fold.mjs';
import {workerData} from 'node:worker_threads';
import {sceneWithAmounts,checkScene} from './sequence-scene.mjs';
import {buildOperatorSupport,evaluateOperatorSupport} from './sequence-operator-support.mjs';
import {endpointRoute,endpointAccess,finishAdvancedFoldAccess,evaluateSequenceTransition} from './sequence-transition.mjs';
import {createAccessWorld,coordinatedSampleAt} from './sequence-access-world.mjs';

const {port,done}=workerData,models=new Map(),catalogs=new Map(),supports=new Map();
port.on('message',async msg=>{
  if(msg.kind==='release-inputs'){
    models.clear();catalogs.clear();supports.clear();
    globalThis.gc?.();
    return;
  }
  let reply;
  try{
    if(msg.model)models.set(msg.modelId,msg.model);
    if(msg.catalog)catalogs.set(msg.catalogId,msg.catalog);
    const model=models.get(msg.modelId),catalog=catalogs.get(msg.catalogId),key=msg.modelId+'|'+msg.bendId;
    if(msg.kind==='independent-check'){
      const input=msg.input;
      let result;
      if(input.kind==='gauge'){
        const {checkStageGauge}=await import('./sequence-stage-gauge.mjs');
        result=checkStageGauge(model,catalog,input);
      }else if(input.kind==='ordinary'){
        const {bendId}=input.stage,sk=msg.modelId+'|'+bendId;
        if(!supports.has(sk))supports.set(sk,buildOperatorSupport(model,bendId));
        result=evaluateSequenceTransition({model,catalog,...input.stage,options:input.options,preparedSupport:supports.get(sk)});
      }else if(input.kind==='flat'||input.kind==='final'){
        const {checkFlatSelfContact,checkFinalSelfContact}=await import('./sequence-flat-self-check.mjs');
        result=(input.kind==='flat'?checkFlatSelfContact:checkFinalSelfContact)(model,{clearanceMm:input.clearanceMm});
      }else throw new Error('Unknown independent check');
      reply={kind:msg.kind,jobId:msg.jobId,result};
    }else if(msg.kind==='terminal-stage'){
      const {finishStageAccess}=await import('./sequence-advanced-search.mjs');
      const guard=()=>{if(Atomics.load(msg.cancel,0)||Date.now()>=msg.deadline)throw new Error('Terminal access stopped');};
      const checked=await finishStageAccess(model,catalog,msg.stage,msg.stage.id,msg.options,guard);
      reply={kind:msg.kind,jobId:msg.jobId,checked};
    }else if(msg.kind==='verify-fold-and-access'){
      const guard=()=>{if(Atomics.load(done,2))throw new Error('Fold pipeline cancelled');};
      const exact=evaluateAdvancedFold({model,catalog,...msg.fold,options:msg.options},guard);
      const checked=exact.feasible?finishAdvancedFoldAccess({model,catalog,fold:exact.stage,operationId:exact.stage.id,options:msg.options},guard):exact;
      reply={kind:msg.kind,jobId:msg.jobId,checked};
    }else if(msg.kind==='normal-stage-access'){
      const results=[],guard=()=>{if(Atomics.load(done,2))throw new Error('Stage batch cancelled');};
      for(const {index,stage} of msg.entries){
        guard();
        const checked=finishAdvancedFoldAccess({model,catalog,fold:stage,operationId:'advanced-'+(index+1),options:msg.options},guard);
        if(checked.feasible)results.push({index,checked});
      }
      reply={kind:msg.kind,jobId:msg.jobId,results};
    }else if(msg.kind==='coordinated-world-samples'){
      const world=createAccessWorld({...msg.payload,model,catalog});
      // Match the serial start check before any translated punch initializes
      // topology caches. Keep the exact initial pose, including tiny offsets
      // permitted by the working-pose contract; this is not an edge sample.
      if(!world.check(msg.edges[0].a,{world:true}).clear)throw new Error('Coordinated worker start differs from the validated working pose');
      let checked=0,skipped=0,firstFailure=null;
      while(!Atomics.load(done,2)){
        const index=Atomics.add(done,1,1);if(index>=msg.count)break;
        // No earlier index can be skipped: claims are monotonic, and the
        // threshold is the smallest failure already checked by any worker.
        if(index>Atomics.load(done,3)){skipped++;break;}
        const sample=coordinatedSampleAt(msg.edges,index),result=world.check(sample.pose,{world:true});
        checked++;Atomics.add(done,4,1);
        if(!result.clear){
          if(!firstFailure||index<firstFailure.index)firstFailure={index,result};
          for(let previous;(previous=Atomics.load(done,3))>index;)if(Atomics.compareExchange(done,3,previous,index)===previous)break;
        }
      }
      reply={kind:msg.kind,jobId:msg.jobId,checked,skipped,firstFailure};
    }else if(msg.kind==='endpoint-route'||msg.kind==='endpoint-access'){
      // End endpoint of a transition; the scene is rebuilt from the same amounts.
      const scene=sceneWithAmounts(model,catalog,msg.baseConfig,msg.sceneAmounts);
      const bend=model.bends.find(b=>String(b.id)===String(msg.bendId));
      const result=msg.kind==='endpoint-route'
        ?endpointRoute({catalog,baseConfig:msg.baseConfig,scene,endpoint:'end',amounts:msg.amounts,options:msg.options,bendId:bend.id})
        :endpointAccess({model,catalog,bend,baseConfig:msg.baseConfig,e:{...msg.e,scene},recommendedTipZMm:msg.recommendedTipZMm,options:msg.options});
      reply={result};
    }else if(msg.kind==='fold-samples'){
     if(!supports.has(key))supports.set(key,buildOperatorSupport(model,msg.bendId));
    const prepared=supports.get(key),{from,to,count}=msg,results=[];
    // Claim positions from the shared counter (done[1]) until exhausted.
    for(let k;(k=Atomics.add(done,1,1))<msg.indices.length;){const i=msg.indices[k];
      const fraction=i/count,amounts=Object.fromEntries(model.bends.map(b=>[b.id,from[b.id]+fraction*(to[b.id]-from[b.id])]));
      const scene=sceneWithAmounts(model,catalog,msg.baseConfig,amounts),support=evaluateOperatorSupport(prepared,scene);
      if(!support.passes){results.push({i,support});continue;}
      const check=checkScene(model,scene,{clearanceMm:msg.clearanceMm,projectJointBounds:true});
      results.push({i,support,punchTipZMm:scene.placement.punchTipZ,check:{status:check.status,hits:check.hits,
        selfCollisionScope:check.selfCollisionScope,jointExceptions:check.jointExceptions,
        closedSolidContainmentAvailableForJointTargets:check.closedSolidContainmentAvailableForJointTargets}});
    }
     reply={results};
    }else throw new Error(`Unknown fold worker job kind: ${String(msg.kind)}`);
  }catch(e){reply={kind:msg.kind,jobId:msg.jobId,error:String(e?.stack??e)};}
  // Signal completion even if posting a malformed result itself fails. The
  // coordinator's bounded receive loop rejects that missing reply.
  if(msg.kind==='terminal-stage'){port.postMessage(reply);return;}
  try{port.postMessage(reply);}finally{Atomics.add(done,0,1);Atomics.notify(done,0);}
});
