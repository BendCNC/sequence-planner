/** Complete an automatically discovered route, including common exterior space,
 * gauge positioning, manual Y adjustments and playback continuity. */
import {finishSequencePlan} from './sequence-plan.mjs';
import {sceneWithAmounts,isPart,bodyBounds} from './sequence-scene.mjs';
import {accessMatrix,buildSequenceTimeline,sequenceSceneAt} from './sequence-playback.mjs';
import {planBackgauge} from './backgauge.mjs';
import {buildFingerBodies} from './finger-geometry.mjs';
import {strokeTransitionError,strokeStateError} from './sequence-stroke.mjs';

import {checkStageGauge} from './sequence-stage-gauge.mjs';
const foldPool=globalThis.process?.versions?.node?await import('./sequence-fold-pool.mjs'):null;

const sameAmounts=(model,a,b)=>model.bends.every(j=>Number.isFinite(a?.[j.id])&&Number.isFinite(b?.[j.id])&&Math.abs(a[j.id]-b[j.id])<1e-9);

/** Extending an already exterior endpoint strictly toward -X cannot encounter
 * its tools. The whole rigid part is already on the front side of their X box. */
export function extendAdvancedExterior(model,catalog,stages,options={}){
  const clearance=options.clearanceMm??.15,margin=Math.max(options.frontMarginMm??10,clearance+1);
  const globalFront=Math.min(...stages.flatMap(s=>sceneWithAmounts(model,catalog,s.config,s.from).bodies.filter(b=>!isPart(b)).map(b=>bodyBounds(b).min[0])));
  const extensions=[];
  for(const [index,stage]of stages.entries())for(const name of ['entry','exit']){
    const access=stage.access?.[name];
    if(access?.path?.status!=='SAMPLE_CLEAR'||access.ram?.status!=='SAMPLE_CLEAR')throw new Error('Unverified operation access');
    const scene=sceneWithAmounts(model,catalog,access.config,name==='entry'?stage.from:stage.to),last=access.path.keyframes.at(-1);
    const m=accessMatrix(last.pose,access.path.pivot);
    let maxX=-Infinity;
    for(const {positions} of scene.bodies.filter(isPart))for(let i=0;i<positions.length;i+=3)
      maxX=Math.max(maxX,m[0]*positions[i]+m[1]*positions[i+1]+m[2]*positions[i+2]+m[3]);
    const ownFront=Math.min(...scene.bodies.filter(b=>!isPart(b)).map(b=>bodyBounds(b).min[0]));
    if(!(maxX<ownFront-clearance))throw new Error('Access endpoint is not fully exterior');
    const dx=Math.min(0,globalFront-margin-maxX);
    if(dx>=-1e-8)continue;
    const frame={phase:'EXTERIOR_EXTENSION',pose:{...last.pose,x:(last.pose.x??0)+dx}};
    access.path.keyframes.push(frame);
    access.path.insertionKeyframes=[...access.path.keyframes].reverse();
    const motion=stage.coordinatedMotion?.[name];
    if(motion){
      const outer=name==='entry'?motion.frames[0]:motion.frames.at(-1);
      const added={ramGapMm:outer.ramGapMm,pose:{...frame.pose}};
      if(name==='entry')motion.frames.unshift(added);else motion.frames.push(added);
    }
    const proof={index,endpoint:name,translationXmm:dx,previousMaximumPartX:maxX,ownObstacleFrontX:ownFront,
      globalObstacleFrontX:globalFront,maximumPartX:maxX+dx,method:'WHOLE_PART_FRONT_HALF_SPACE_MONOTONE_TRANSLATION'};
    access.path.exteriorExtension=proof;extensions.push(proof);
  }
  return extensions;
}

/** A conservative X envelope holds throughout each linear translation and
 * Y-axis rotation, not just at keyframes. It is used only to park fingers. */
export function advancedParkEnvelope(model,catalog,stages){
  let maximumX=-Infinity;
  for(const stage of stages){
    for(const sample of stage.foldSamples??[]){
      const scene=sceneWithAmounts(model,catalog,stage.config,sample.amounts);
      for(const body of scene.bodies)maximumX=Math.max(maximumX,bodyBounds(body).max[0]);
    }
    for(const name of ['entry','exit']){
      const access=stage.access[name],scene=sceneWithAmounts(model,catalog,access.config,name==='entry'?stage.from:stage.to);
      for(const body of scene.bodies.filter(b=>!isPart(b)))maximumX=Math.max(maximumX,bodyBounds(body).max[0]);
      const pivot=access.path.pivot;
      let radius=-Infinity;
      for(const {positions} of scene.bodies.filter(isPart))for(let i=0;i<positions.length;i+=3)
        radius=Math.max(radius,Math.hypot(positions[i]-pivot[0],positions[i+2]-pivot[2]));
      const motionFrames=[...(stage.coordinatedMotion?.[name]?.frames??access.path.keyframes),
        ...(stage.coordinatedMotion?.[name==='entry'?'approach':'retract']?.frames??[])];
      const translation=Math.max(0,...motionFrames.map(f=>f.pose.x??0));
      maximumX=Math.max(maximumX,pivot[0]+radius+translation);
    }
  }
  return {maximumX,method:'FOLD_SAMPLES_AND_CONTINUOUS_RIGID_ACCESS_X_RADIUS_BOUND'};
}

export function certifyManualFingerAdjustment(transfer,first,last,definition,maximumToolX,clearance=.15){
  const samePark=['lowerFaceX','baseZ'].every(k=>Math.abs(first.park[k]-last.park[k])<1e-8);
  const sameCount=first.stationY.length===last.stationY.length;
  const endpointFingers=[first,last].map(p=>buildFingerBodies({definition,...p.park,stationY:p.stationY}));
  const minimumFingerX=Math.min(...endpointFingers.flat().map(b=>bodyBounds(b).min[0]));
  const mutualGap=sameCount&&first.stationY.length===2?Math.min(...endpointFingers.map(f=>bodyBounds(f[1]).min[1]-bodyBounds(f[0]).max[1])):null;
  const maximumPartX=transfer.proof.maximumPartX;
  const clear=samePark&&sameCount&&first.status==='SAMPLE_CLEAR'&&last.status==='SAMPLE_CLEAR'&&
    (mutualGap===null||mutualGap>clearance)&&minimumFingerX-maximumToolX>clearance&&minimumFingerX-maximumPartX>clearance;
  return {status:clear?'EXTERIOR_MANUAL_Y_ADJUSTMENT_GEOMETRY_CLEAR':'NOT_VERIFIED',
    stationYFrom:[...first.stationY],stationYTo:[...last.stationY],park:{...first.park},definitionId:definition.id,
    manualSetup:first.stationY.some((v,i)=>Math.abs(v-last.stationY[i])>1e-8),automaticYMotion:false,
    calibrated:false,physicalTravelLimitsVerified:false,
    proof:{method:'Fixed X/Z, affine Y separation between endpoint finger boxes; X separation from both tool sets and exterior part envelope.',
      samePark,sameCount,clearanceMm:clearance,minimumFingerX,maximumToolX,maximumPartX,
      toolSeparationMm:minimumFingerX-maximumToolX,partSeparationMm:minimumFingerX-maximumPartX,minimumMutualYGapMm:mutualGap}};
}

export function verifyAdvancedContinuity(model,catalog,plan){
  const timeline=buildSequenceTimeline(plan),failures=[];let maximumErrorMm=0;
  for(let i=1;i<timeline.segments.length;i++){
    const a=timeline.segments[i-1],b=timeline.segments[i];
    const left=sequenceSceneAt(model,catalog,plan,a.stageIndex,a.phase,1),right=sequenceSceneAt(model,catalog,plan,b.stageIndex,b.phase,0);
    const rightBodies=new Map(right.bodies.map(body=>[body.id,body]));
    if(left.bodies.length!==right.bodies.length)failures.push({boundary:i,reason:'BODY_COUNT'});
    for(const body of left.bodies){
      const other=rightBodies.get(body.id);
      if(!other||body.positions.length!==other.positions.length){failures.push({boundary:i,bodyId:body.id,reason:'TOPOLOGY'});continue;}
      let error=0;for(let k=0;k<body.positions.length;k++)error=Math.max(error,Math.abs(body.positions[k]-other.positions[k]));
      maximumErrorMm=Math.max(maximumErrorMm,error);
      if(error>1e-6)failures.push({boundary:i,bodyId:body.id,errorMm:error});
    }
  }
  return {status:failures.length?'DISCONTINUOUS':'CONTINUOUS',boundaries:timeline.segments.length-1,maximumErrorMm,failures};
}

export function finishAdvancedSequencePlan(model,catalog,input,options={},onProgress=()=>{}){
  const result=structuredClone(input),clearance=options.clearanceMm??.15;
  result.diagnostics??={};result.completeManufacturingVerified=false;
  const checkCancelled=()=>{if(options.signal?.aborted)throw Object.assign(new Error('Search cancelled'),{cancelled:true});};
  try{
    checkCancelled();
    if(!result.stages?.length||result.stages.some((s,i)=>!s.foldSamplesClear||!sameAmounts(model,s.from,i?result.stages[i-1].to:Object.fromEntries(model.bends.map(b=>[b.id,0])))))throw new Error('Incomplete fold chain');
    for(const stage of result.stages){
      const error=strokeTransitionError(model,model.bends.find(b=>String(b.id)===String(stage.bendId)),stage.from,stage.to);
      if(error)throw new Error(error);
      if(stage.foldSamples?.some(sample=>strokeStateError(model,sample.amounts)))throw new Error('Inconsistent joint-stroke fold samples');
    }
    result.exteriorExtensions=extendAdvancedExterior(model,catalog,result.stages,options);
    finishSequencePlan(model,catalog,result,options);
    if(!result.completeGeometricRouteSampleClear)return result;
    if(options.backgauge){
      if(!catalog.fingerDefinition)throw new Error('Missing finger geometry');
      onProgress({phase:'advanced-backgauge'});
      checkCancelled();
      const envelope=advancedParkEnvelope(model,catalog,result.stages),definition=catalog.fingerDefinition;
      const fingerMinX=bodyBounds({positions:definition.collisionMesh.positions}).min[0];
      const park={lowerFaceX:envelope.maximumX+25.4-Math.min(0,fingerMinX),baseZ:50};
      result.fingerParkingProof={...envelope,park,clearanceMm:clearance};
      const requests=result.stages.map(stage=>({kind:'gauge',from:stage.from,park,settings:options.backgauge,clearance,
        config:{...stage.access.entry.config,...(Number.isFinite(stage.gaugeRamGapMm)?{ramGapMm:stage.gaugeRamGapMm}:{})}}));
      const local=input=>{checkCancelled();return checkStageGauge(model,catalog,input);};
      const answers=options.plannerMode!=='full'&&requests.length>=4&&foldPool?.foldPoolSize()>1
        ?foldPool.checkIndependentJobs(model,catalog,requests,local,checkCancelled).results:requests.map(local);
      for(const [i,stage] of result.stages.entries()){
        checkCancelled();
        Object.assign(stage,answers[i]);
        if(stage.backgauge.status!=='SAMPLE_CLEAR')throw new Error('Backgauge positioning rejected: '+stage.bendId);
      }
      // A finger may be unused, but it cannot disappear between operations.
      // When automatic layouts mixed one and two fingers, keep both present
      // and test two placements of the unused finger with the full gauge check.
      if(new Set(result.stages.map(s=>s.backgauge.stationY.length)).size>1){
        if(options.backgauge.fixedStations)throw new Error('Fixed finger count changed');
        const face=definition.contactFeatures.lower;
        const separation=Math.max(51,options.backgauge.separationMm??100,face.maxY-face.minY+(options.backgauge.fingerGapMm??12.7));
        result.fingerLayoutCompletion=[];
        for(const [index,stage]of result.stages.entries())if(stage.backgauge.stationY.length===1){
          checkCancelled();const primaryY=stage.backgauge.stationY[0];
          const config={...stage.access.entry.config,...(Number.isFinite(stage.gaugeRamGapMm)?{ramGapMm:stage.gaugeRamGapMm}:{})};
          const scene=sceneWithAmounts(model,catalog,config,stage.from);let replacement=null;
          for(const sign of [1,-1]){
            const candidate=planBackgauge({scene,definition,settings:{...options.backgauge,fixedStations:true,separationMm:separation,centerYmm:primaryY+sign*separation/2},park,clearanceMm:clearance});
            if(candidate.status==='SAMPLE_CLEAR'){replacement=candidate;break;}
          }
          if(!replacement)throw new Error('No collision-free placement for the second physical finger');
          stage.backgauge=replacement;
          result.fingerLayoutCompletion.push({stageIndex:index,primaryY,stationY:replacement.stationY,method:'KEEP_BOTH_PHYSICAL_FINGERS_AND_RECHECK_GAUGE'});
        }
      }
      for(const [index,transfer]of result.transfers.entries()){
        const a=result.stages[index],b=result.stages[index+1];
        const toolMaximum=Math.max(...[a,b].flatMap(s=>sceneWithAmounts(model,catalog,s.config,s.from).bodies.filter(body=>!isPart(body)).map(body=>bodyBounds(body).max[0])));
        transfer.fingerAdjustment=certifyManualFingerAdjustment(transfer,a.backgauge,b.backgauge,definition,toolMaximum,clearance);
        if(transfer.fingerAdjustment.status!=='EXTERIOR_MANUAL_Y_ADJUSTMENT_GEOMETRY_CLEAR')throw new Error('Finger transfer adjustment rejected');
      }
      result.backgaugeSampleClear=true;result.backgaugeCalibrationVerified=false;
    }
    result.playbackContinuity=verifyAdvancedContinuity(model,catalog,result);
    checkCancelled();
    if(result.playbackContinuity.status!=='CONTINUOUS')throw new Error('Playback discontinuity');
    result.playbackIntegrationVerified=true;result.status='COMPLETE_GEOMETRIC_ROUTE_SAMPLE_CLEAR';
    result.toolChangeEvents=result.transfers.filter(t=>t.requiresManualSetupChange).length;
    result.diagnostics.pending=['Physical holders, travel limits and grip feasibility are not modeled.',
      'Manual tooling and finger setup changes require operator intervention.',
      'Forming force, springback and material behavior are not certified.'];
    return result;
  }catch(error){
    result.completeGeometricRouteSampleClear=false;result.status=error.cancelled?'CANCELLED':'ADVANCED_COMPLETE_ROUTE_REJECTED';
    result.reason=error.message;return result;
  }
}
