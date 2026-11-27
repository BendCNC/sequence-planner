/** Reject-only necessary condition: a completed part must fit its last tooling.
 * No order, pre-bend or ram retraction can remove a collision at the final
 * forming pose. A clear result is NOT a route or a manufacturing certificate.
 */
import {sceneWithAmounts,checkScene,isPart} from './sequence-scene.mjs';
import {buildOperatorSupport,evaluateOperatorSupport} from './sequence-operator-support.mjs';
import {strokeMaterialIntervals} from './machine.mjs';
import {strokeAllowed} from './sequence-stroke.mjs';

export async function checkFinalToolSetups({model,catalog,setups,options={},support,stopOnFirstClear=true},guard=()=>{}){
  const started=performance.now(),amounts=Object.fromEntries(model.bends.map(b=>[b.id,1]));
  const result={status:'NO_FINAL_TOOL_SETUP_IN_CANDIDATES',checkedPoses:0,collisionChecks:0,
    frontRejected:0,clearPoses:0,byBend:[],allCandidatesChecked:false,
    scope:'Necessary final forming pose only, for retained candidates. Piece/tool, table and tool/tool collisions. No self-collision, motion, forming or global impossibility certificate.'};
  for(const bend of model.bends){
    // A clear tool pose on a follower or on a lead whose carrier is already
    // folded cannot be the final stroke of a complete program.
    if(!strokeAllowed(model,bend,amounts))continue;
    const prepared=support?.get(String(bend.id))??buildOperatorSupport(model,bend.id),material=strokeMaterialIntervals(bend);
    const row={bendId:bend.id,materialSpanMm:material.intervals.at(-1)[1]-material.intervals[0][0],checkedPoses:0,
      frontRejected:0,clearPoses:0,blocked:[]};result.byBend.push(row);
    for(const candidate of setups.byBend.get(String(bend.id))??[])for(const flip of[false,true]){
      guard();if(result.checkedPoses%16===0){await new Promise(resolve=>setTimeout(resolve,0));guard();}
      const config={...candidate,bendId:bend.id,flip,ramGapMm:0,contactGapMm:options.contactGapMm??.3};
      const scene=sceneWithAmounts(model,catalog,config,amounts);result.checkedPoses++;row.checkedPoses++;
      if(!evaluateOperatorSupport(prepared,scene).passes){result.frontRejected++;row.frontRejected++;continue;}
      // Defer expensive self/seam checks to the full transition validator.
      const check=checkScene(model,scene,{clearanceMm:options.clearanceMm??.15,projectJointBounds:true,onlyIds:new Set(scene.bodies.filter(b=>!isPart(b)).map(b=>b.id))});
      result.collisionChecks++;
      if(check.status==='SAMPLE_CLEAR'){
        result.clearPoses++;row.clearPoses++;result.status='POSSIBLE_FINAL_TOOL_SETUP';
        result.clearSetup??={bendId:bend.id,config};
        if(stopOnFirstClear)return{...result,elapsedMs:performance.now()-started};
      }else{
        const hit=check.hits[0];
        row.blocked.push({config,collision:{bodyIds:hit.bodyIds,bodyKinds:hit.bodyKinds,category:hit.category,
          point:hit.point,closestPoints:hit.closestPoints,reason:hit.reason,clearanceMm:options.clearanceMm??.15,
          witnessedSurfaceDistanceMm:hit.witnessedSurfaceDistanceMm??hit.surfaceDistanceMm??null}});
      }
    }
  }
  return{...result,allCandidatesChecked:true,elapsedMs:performance.now()-started};
}
