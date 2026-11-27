/** Reject-only certificate for ordinary programs with a fixed list of pre-bend angles.
 * The advanced solver chooses partial angles from geometry and is outside this proof.
 * Every monotone program has a last stroke. With all other bends complete, it
 * passes through both the greatest permitted starting amount and the closed pose.
 */
import {bandPoint,evaluateModel,identity} from './solver.mjs';
import {machinePlacement,partBodies,strokeMaterialIntervals} from './machine.mjs';
import {checkScene} from './sequence-scene.mjs';
import {buildOperatorSupport,evaluateOperatorSupport} from './sequence-operator-support.mjs';
import {closureProfiles,pairedClosureSurfaceIntervals} from './sequence-paired-closure.mjs';
import {partialBendTargets} from './sequence-search-variants.mjs';
import {toolSupportPolicy} from './tool-support-policy.mjs';

function mandatoryStock(material,flip,options){
  if(!material.exact||!material.intervals.length)return [];
  const spans=material.intervals.map(([lo,hi])=>flip?[-hi,-lo]:[lo,hi]).sort((a,b)=>a[0]-b[0]);
  const allowance=toolSupportPolicy(options).maxUnsupportedEndMm+1e-6;
  spans[0][0]+=allowance;
  spans.at(-1)[1]-=allowance;
  return spans.filter(([a,b])=>b>a);
}

function punchWitness(bodies,tipZ,punch,mirror,spans,options,guard,profiles){
  const gap=Math.max(.5,options.jointGapToleranceMm??0)+2*Math.max(1e-6,options.coverageEpsilonMm??0);
  for(const body of bodies){
    if(body.kind!=='panel')continue;
    const wall={...body,positions:body.positions.map((v,i)=>i%3===2?v-tipZ:v)};
    const wide=piece=>spans.some(([lo,hi])=>Math.min(hi,piece.interval[1])-Math.max(lo,piece.interval[0])>gap);
    const clipped=pairedClosureSurfaceIntervals(wall,punch,{mirror,stopAtPiece:wide,profiles},guard);
    if(!clipped)return null;
    for(const piece of clipped.pieces)for(const [lo,hi] of spans){
      const from=Math.max(lo,piece.interval[0]),to=Math.min(hi,piece.interval[1]);
      // A single real face triangle penetrates the filled profile throughout
      // this interval. Even the largest permitted joint cannot contain it.
      if(to-from>gap)return {kind:'PUNCH_IN_MANDATORY_STOCK',panel:body.id,wallTriangle:piece.wallTriangle,
        profileTriangle:piece.profileTriangle,stationIntervalMm:[from,to],strictInteriorMm:clipped.strictInteriorMm};
    }
  }
  return null;
}

export function proveRankedTerminalBlock(input,catalog,options,partialAngles,guard=()=>{}){
  if(options.experimentalAdvancedSearch!==false||options.allowPartialCoverage||!input.bends?.length||
    ['operationTemplates','setups','seedSetups','initialSegments','initialSetup','fixedStations','segments','stations'].some(k=>options[k]!=null)
    ||input.bends.some(b=>b.strokeLeadId!=null||b.strokeFollowerIds?.length||b.strokeRequiresFlatBendIds?.length||
      b.source?.strokeLeadId!=null||b.source?.strokeFollowerIds?.length||b.source?.strokeRequiresFlatBendIds?.length))return null;
  const model={...input,bends:input.bends.map(b=>b.band.flatPositions?b:{...b,band:{...b.band,
    flatPositions:b.band.parameters.flatMap((v,i,a)=>i%3?[]:bandPoint(b,v,a[i+1],a[i+2],0))}})};
  // Consider even ready punches outside a ranked group. Machine placement
  // changes identically in Z for the part and the punch when changing die.
  const punches=catalog.tools.filter(t=>t.kind==='punch'&&t.status==='ready');
  const die=catalog.tools.find(t=>t.kind==='die'&&t.status==='ready');
  if(!die||!punches.length||!Array.isArray(partialAngles)||!partialAngles.every(a=>Number.isFinite(a)&&a>0))return null;
  const allClosed=Object.fromEntries(model.bends.map(b=>[b.id,1])),bends=[],profiles=closureProfiles();
  for(const bend of model.bends){
    guard();
    // Include caller angles and every fallback variant, even if it is disabled.
    const starts=partialBendTargets(bend,0,true,{partialAnglesDegrees:partialAngles}).filter(q=>q<1);
    const highestStart=Math.max(0,...starts);
    const row={bendId:bend.id,highestStart,presentations:[]};
    const poses=[highestStart,1].map(q=>({q,state:evaluateModel(model,0,{amounts:{...allClosed,[bend.id]:q}})}));
    // This certificate accepts only rigid panel-panel contact. Running band
    // checks cannot establish that fact. The full trajectory validator still
    // checks panels, bands, tooling and joint reliefs for every accepted route.
    const first=poses[0].state,frame=identity(),bodies=partBodies(model,first,frame).filter(body=>body.kind==='panel');
    const self=checkScene(model,{state:first,placement:{frame},amounts:first.amounts,bodies},{clearanceMm:options.clearanceMm??.15});
    const hit=self.hits.find(h=>h.bodyKinds.every(k=>k==='panel')&&h.point?.every(Number.isFinite));
    if(hit){
      row.selfContact={amount:highestStart,bodyIds:hit.bodyIds,point:hit.point};
      bends.push(row);continue;
    }
    const support=buildOperatorSupport(model,bend.id),material=strokeMaterialIntervals(bend);
    for(const flip of [false,true]){
      const spans=mandatoryStock(material,flip,options);
      const byState=new Map();
      for(const punch of punches){
        const pending=new Set([false,true]);
        for(const {q,state} of [poses[1],poses[0]]){
          guard();
          const placement=machinePlacement(model,state,bend.id,die,{flip,stationYmm:0,contactGapMm:options.contactGapMm??.3},punch);
          let positioned=byState.get(state);
          if(!positioned||!positioned.frame.every((value,i)=>Object.is(value,placement.frame[i]))){
            const parts=partBodies(model,state,placement.frame);
            positioned={frame:placement.frame,parts,front:evaluateOperatorSupport(support,{bodies:parts})};
            byState.set(state,positioned);
          }
          const {front,parts}=positioned;
          for(const mirror of pending){
            const witness=front.centroidX>1e-6||front.frontFraction<.5-1e-6
              ?{kind:'MAIN_BRANCH_NOT_FRONT',centroidX:front.centroidX,frontFraction:front.frontFraction}
              :punchWitness(parts,placement.punchTipZ,punch,mirror,spans,options,guard,profiles);
            if(witness){row.presentations.push({punchId:punch.id,flip,mirror,amount:q,...witness});pending.delete(mirror);}
          }
          if(!pending.size)break;
        }
        if(pending.size)return null;
      }
    }
    bends.push(row);
  }
  return {status:'NO_LAST_STROKE_AT_RANKED_ANGLES',partialAnglesDegrees:partialAngles,bends,
    scope:'Uniform tip-aligned stock, both part and punch orientations, exact support and only the enumerated partial angles. Geometry-derived partials and coordinated search remain available.'};
}


// In-memory binding is deliberately lost across serialized/replacement models.
// Only the file adapter attaches it after checking the original inputs.
const verifiedRankedTerminal=Symbol('verifiedRankedTerminal');
const policyKey=o=>JSON.stringify([o.bedLengthMm??null,toolSupportPolicy(o).maxUnsupportedEndMm,o.clearanceMm??.15,
 o.contactGapMm??.3,o.jointGapToleranceMm??0,o.coverageEpsilonMm??0,Boolean(o.allowPartialCoverage)]);
export async function rankedTerminalFingerprint(model,catalog,options,proof){
 const {createHash}=await import('node:crypto');
 const hash=createHash('sha256');
 for(const value of ['uniform-terminal-proof-v1',model,catalog,policyKey(options),proof.status,proof.partialAnglesDegrees,proof.bends])
  hash.update(JSON.stringify(value)).update('\n');
 return hash.digest('hex');
}
export async function bindRankedTerminal(model,catalog,options,proof){
 delete options[verifiedRankedTerminal];
 if(proof?.status==='NO_LAST_STROKE_AT_RANKED_ANGLES'&&Array.isArray(proof.bends)&&Array.isArray(proof.partialAnglesDegrees)&&
    typeof proof.inputProofSha256==='string'&&proof.inputProofSha256===await rankedTerminalFingerprint(model,catalog,options,proof)){
  options[verifiedRankedTerminal]={model,catalog,policy:policyKey(options),proof};return true;
 }
 return false;
}
export function rankedTerminalRows(model,catalog,options){
 const bound=options[verifiedRankedTerminal];
 if(!bound||bound.model!==model||bound.catalog!==catalog||bound.policy!==policyKey(options)||options.allowPartialCoverage||
  ['operationTemplates','setups','seedSetups','initialSegments','initialSetup','fixedStations','segments','stations'].some(k=>options[k]!=null))return null;
 const punches=catalog.tools.filter(t=>t.kind==='punch'&&t.status==='ready');
 const ids=new Set(model.bends.map(b=>b.id)),rows=new Map(),validAmount=q=>Number.isFinite(q)&&q>0&&q<=1;
 const gap=Math.max(.5,options.jointGapToleranceMm??0)+2*Math.max(1e-6,options.coverageEpsilonMm??0);
 function obstructs(w){
  if(!w||!validAmount(w.amount))return false;
  if(w.kind==='MAIN_BRANCH_NOT_FRONT')return Number.isFinite(w.centroidX)&&Number.isFinite(w.frontFraction)&&
   (w.centroidX>1e-6||w.frontFraction<.5-1e-6);
  return w.kind==='PUNCH_IN_MANDATORY_STOCK'&&typeof w.panel==='string'&&Number.isInteger(w.wallTriangle)&&w.wallTriangle>=0&&
   Number.isInteger(w.profileTriangle)&&w.profileTriangle>=0&&w.strictInteriorMm>0&&Number.isFinite(w.strictInteriorMm)&&
   Array.isArray(w.stationIntervalMm)&&w.stationIntervalMm.length===2&&w.stationIntervalMm.every(Number.isFinite)&&
   w.stationIntervalMm[1]-w.stationIntervalMm[0]>gap;
 }
 for(const row of bound.proof.bends){
  if(!row||!ids.has(row.bendId))continue;
  const hit=row.selfContact;
  if(validAmount(hit?.amount)&&hit.bodyIds?.length===2&&hit.point?.length===3&&hit.point.every(Number.isFinite)){
   rows.set(row.bendId,{bendId:row.bendId,selfContact:hit,presentations:[]});continue;
  }
  if(!Array.isArray(row.presentations))continue;
  const presentations=row.presentations.filter(obstructs);
  if(punches.length&&punches.every(tool=>[false,true].every(flip=>[false,true].every(mirror=>presentations.some(w=>
   w.punchId===tool.id&&w.flip===flip&&w.mirror===mirror)))))
   rows.set(row.bendId,{bendId:row.bendId,presentations});
 }
 return rows.size?rows:null;
}
