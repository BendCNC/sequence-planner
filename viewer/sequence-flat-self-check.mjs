/** Invariant: a flat blank cannot interfere with itself. Any PIECE_SELF hit at
 * all amounts = 0 is a modelling/exception defect, never a manufacturing limit.
 * Checked once before the search so the failure is reported immediately with
 * its witness instead of being hidden as "no sequence". No part-specific rules.
 */
import {evaluateModel,identity} from './solver.mjs';
import {partBodies} from './machine.mjs';
import {checkScene} from './sequence-scene.mjs';

// Same clearance/tolerance as the search: joint exception margins derive from them.
export function checkFlatSelfContact(model,opts={}){return checkPoseSelfContact(model,0,opts);}
/** Same invariant at the fully bent pose: the designed part exists in CAD, so it
 * cannot interfere with itself either. Real closure blocks happen mid-stroke only. */
export function checkFinalSelfContact(model,opts={}){
  const r=checkPoseSelfContact(model,1,opts);return{...r,status:r.status==='FLAT_CLEAR'?'FINAL_CLEAR':'FINAL_SELF_CONTACT'};
}
function checkPoseSelfContact(model,amount,{clearanceMm=.15,toleranceMm=1e-7}={}){
  const amounts=Object.fromEntries(model.bends.map(b=>[b.id,amount]));
  const state=evaluateModel(model,0,{amounts});
  const frame=identity(),scene={state,placement:{frame},amounts,bodies:partBodies(model,state,frame)};
  const result=checkScene(model,scene,{clearanceMm,toleranceMm,firstOnly:false,projectJointBounds:true});
  return{status:result.hits.length?'FLAT_SELF_CONTACT':'FLAT_CLEAR',
    hits:result.hits.map(h=>({bodyIds:h.bodyIds,point:h.point,witnessedSurfaceDistanceMm:h.witnessedSurfaceDistanceMm,
      exception:h.jointException?.method??null}))};
}
