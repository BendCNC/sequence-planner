import {movesWithRam} from './tool-geometry.mjs';
/** Geometry-driven playback and exterior transfers; no part names, bend IDs or machine commands. */
import {evaluateModel,transform,transformPositions,multiply,rotation,identity} from './solver.mjs';
import {createEndpointSceneBuilder,partBodies} from './machine.mjs';
import {sceneWithAmounts,isPart,bodyBounds} from './sequence-scene.mjs';
import {poseTransferAt} from './sequence-transfer-motion.mjs';
import {withBackgauge} from './backgauge.mjs';
import {coordinatedMotionAt} from './sequence-coordinated-motion.mjs';

export const phaseLabels={entry:'Entrada frontal',gauge:'Posicionar backgauge',ungauge:'Retraer backgauge',approach:'Aproximación del punzón',fold:'Plegado',retract:'Retracción',exit:'Salida frontal',transfer:'Traslado exterior'};
const durations={entry:4,gauge:3,ungauge:2,approach:2,fold:5,retract:2,exit:4,transfer:5};
function transferSphere(bodies){
  const sum=[0,0,0];let count=0;
  for(const {positions} of bodies){
    count+=positions.length/3;
    for(let i=0;i<positions.length;i+=3)for(let k=0;k<3;k++)sum[k]+=positions[i+k];
  }
  const center=sum.map(v=>v/count);let radius=0;
  for(const {positions} of bodies)for(let i=0;i<positions.length;i+=3)
    radius=Math.max(radius,Math.hypot(positions[i]-center[0],positions[i+1]-center[1],positions[i+2]-center[2]));
  return {center,radius:radius+1e-6};
}
function transferBounds(bodies,matrix){
  const min=[Infinity,Infinity,Infinity],max=[-Infinity,-Infinity,-Infinity];
  for(const {positions} of bodies){
    const box=bodyBounds({positions:transformPositions(positions,matrix)});
    for(let k=0;k<3;k++){min[k]=Math.min(min[k],box.min[k]);max[k]=Math.max(max[k],box.max[k]);}
  }
  return {min,max};
}

export function accessMatrix(pose,pivot){
  const m=rotation([0,1,0],(pose.tiltDeg??0)*Math.PI/180),p=transform(m,pivot);
  m[3]=pivot[0]-p[0]+(pose.x??0);m[7]=pivot[1]-p[1]+(pose.y??0);m[11]=pivot[2]-p[2]+(pose.z??0);return m;
}
export function pathPoseAt(frames,q){
  if(!Array.isArray(frames)||!frames.length)throw new Error('Falta una trayectoria comprobada.');
  if(frames.length===1)return frames[0].pose;
  const lengths=frames.slice(1).map((f,i)=>Math.hypot(...['x','y','z'].map(k=>(f.pose[k]??0)-(frames[i].pose[k]??0)))+Math.abs((f.pose.tiltDeg??0)-(frames[i].pose.tiltDeg??0))*2);
  let distance=Math.max(0,Math.min(1,q))*lengths.reduce((a,b)=>a+b,0);
  for(let i=0;i<lengths.length;i++){
    if(distance<=lengths[i]||i===lengths.length-1){const a=frames[i].pose,b=frames[i+1].pose,t=lengths[i]?distance/lengths[i]:1;
      return Object.fromEntries(['x','y','z','tiltDeg'].map(k=>[k,(a[k]??0)+((b[k]??0)-(a[k]??0))*t]));}
    distance-=lengths[i];
  }
}
function quaternion(m){
  const trace=m[0]+m[5]+m[10];let x,y,z,w,s;
  if(trace>0){s=2*Math.sqrt(trace+1);w=s/4;x=(m[9]-m[6])/s;y=(m[2]-m[8])/s;z=(m[4]-m[1])/s;}
  else if(m[0]>m[5]&&m[0]>m[10]){s=2*Math.sqrt(1+m[0]-m[5]-m[10]);w=(m[9]-m[6])/s;x=s/4;y=(m[1]+m[4])/s;z=(m[2]+m[8])/s;}
  else if(m[5]>m[10]){s=2*Math.sqrt(1+m[5]-m[0]-m[10]);w=(m[2]-m[8])/s;x=(m[1]+m[4])/s;y=s/4;z=(m[6]+m[9])/s;}
  else{s=2*Math.sqrt(1+m[10]-m[0]-m[5]);w=(m[4]-m[1])/s;x=(m[2]+m[8])/s;y=(m[6]+m[9])/s;z=s/4;}
  const n=Math.hypot(x,y,z,w);return[x/n,y/n,z/n,w/n];
}
const frame=(matrix,center)=>({matrixRowMajor:matrix,vertexMeanWorld:transform(matrix,center),quaternionXYZW:quaternion(matrix)});
function sameSetupGeometry(a,b,deltaTip){
  if(a.length!==b.length)return false;
  return a.every(p=>{const q=b.find(x=>x.id===p.id);return q&&p.kind===q.kind&&p.toolId===q.toolId&&p.positions.length===q.positions.length&&p.triangles.length===q.triangles.length&&
    p.triangles.every((v,i)=>v===q.triangles[i])&&p.positions.every((v,i)=>Math.abs(q.positions[i]-v-(movesWithRam(p)&&i%3===2?deltaTip:0))<1e-7);});
}

/** The piece stays wholly in front of the union of both endpoint tool setups. */
export function buildExteriorTransfers(model,catalog,stages,{parkingClearanceMm=25}={}){
  const transfers=[];
  for(let i=0;i<stages.length-1;i++){
    const previous=stages[i],next=stages[i+1],a=previous.access.exit,b=next.access.entry;
    if(model.bends.some(bend=>Math.abs(previous.to[bend.id]-next.from[bend.id])>1e-10))throw new Error('El traslado no puede cambiar los pliegues de la pieza.');
    const canonical=partBodies(model,evaluateModel(model,0,{amounts:previous.to}),identity());
    const {center,radius}=transferSphere(canonical);
    const fromScene=sceneWithAmounts(model,catalog,a.config,previous.to),toScene=sceneWithAmounts(model,catalog,b.config,next.from);
    const fromObstacles=fromScene.bodies.filter(body=>!isPart(body)),toObstacles=toScene.bodies.filter(body=>!isPart(body));
    const front=Math.min(...[...fromObstacles,...toObstacles].map(b=>bodyBounds(b).min[0]));
    const am=accessMatrix(a.path.keyframes.at(-1).pose,a.path.pivot),bm=accessMatrix(b.path.insertionKeyframes[0].pose,b.path.pivot);
    const from=multiply(am,fromScene.placement.frame),to=multiply(bm,toScene.placement.frame);
    const fromCenter=transform(from,center),toCenter=transform(to,center);
    const fromBox=transferBounds(canonical,from),toBox=transferBounds(canonical,to);
    if(Math.max(fromBox.max[0],toBox.max[0])>=front-.15)throw new Error('El acceso no deja la pieza completamente delante de las herramientas.');
    const parkingX=Math.min(fromCenter[0],toCenter[0],front-radius-parkingClearanceMm);
    const parkFrom=[...from],parkTo=[...to];parkFrom[3]+=parkingX-fromCenter[0];parkTo[3]+=parkingX-toCenter[0];
    const fromTip=fromScene.placement.punchTipZ+a.config.ramGapMm,toTip=toScene.placement.punchTipZ+b.config.ramGapMm;
    const ramMotion={fromTipZMm:fromTip,toTipZMm:toTip};
    let ramTravel=toTip-fromTip;
    if([a,b].some(access=>access.config.segments.some(s=>s.punchId!=null))){
      ramMotion.fromPunchHeightMm=catalog.tools.find(t=>t.id===a.config.punchId).contact.mountPlaneZMm;
      ramMotion.toPunchHeightMm=catalog.tools.find(t=>t.id===b.config.punchId).contact.mountPlaneZMm;
      ramTravel+=ramMotion.toPunchHeightMm-ramMotion.fromPunchHeightMm;
    }
    const sameTools=sameSetupGeometry(fromObstacles,toObstacles,ramTravel);
    const box={min:[parkingX-radius,Math.min(fromCenter[1],toCenter[1])-radius,Math.min(fromCenter[2],toCenter[2])-radius],
      max:[parkingX+radius,Math.max(fromCenter[1],toCenter[1])+radius,Math.max(fromCenter[2],toCenter[2])+radius]};
    const envelope={min:box.min.map((v,k)=>Math.min(v,fromBox.min[k],toBox.min[k])),max:box.max.map((v,k)=>Math.max(v,fromBox.max[k],toBox.max[k]))};
    transfers.push({id:`transfer-${i+1}`,fromStageId:previous.id,toStageId:next.id,amounts:{...previous.to},
      status:'PART_EXTERIOR_ROUTE_ANALYTIC_CLEAR',canonical:{vertexMean:center,radiusMm:radius},
      keyframes:[frame(from,center),frame(parkFrom,center),frame(parkTo,center),frame(to,center)],
      ramMotion,requiresManualSetupChange:!sameTools,toolChangeMotionVerified:sameTools,
      proof:{obstacleFrontX:front,maximumPartX:Math.max(fromBox.max[0],toBox.max[0],parkingX+radius),
        parkingSeparationMm:front-(parkingX+radius),method:'Rigid triangle mesh enclosed by a sphere throughout exterior reorientation; both translation legs remain in front.'},
      requiredFreeFrontSpace:{conservativeEnvelope:envelope,requiredDepthMmForwardOfObstacleFront:front-envelope.min[0]}});
  }
  return transfers;
}

export function buildSequenceTimeline(plan){
  const segments=[];let durationSeconds=0;
  for(let stageIndex=0;stageIndex<plan.stages.length;stageIndex++){
    for(const phase of ['entry',...(plan.stages[stageIndex].backgauge?['gauge','ungauge']:[]),'approach','fold','retract','exit',...(stageIndex<plan.stages.length-1?['transfer']:[])]){
      if(phase==='transfer'&&!plan.transfers?.[stageIndex])throw new Error('Falta verificar un traslado exterior.');
      segments.push({stageIndex,phase,startSeconds:durationSeconds,durationSeconds:durations[phase]});durationSeconds+=durations[phase];
    }
  }
  return{segments,durationSeconds};
}
export function sequencePositionAt(timeline,normalized){
  if(!timeline.segments.length)return null;
  const time=Math.max(0,Math.min(1,normalized))*timeline.durationSeconds;
  const segment=timeline.segments.find(s=>time<s.startSeconds+s.durationSeconds)??timeline.segments.at(-1);
  return{...segment,q:Math.max(0,Math.min(1,(time-segment.startSeconds)/segment.durationSeconds))};
}

export function sequenceSceneAt(model,catalog,plan,stageIndex,phase,q){
  const stage=plan.stages[stageIndex];if(!stage)throw new Error('Operación de secuencia desconocida.');
  const from=['entry','gauge','ungauge','approach'].includes(phase),amounts=phase==='fold'?Object.fromEntries(model.bends.map(b=>[b.id,stage.from[b.id]+(stage.to[b.id]-stage.from[b.id])*q])):from?stage.from:stage.to;
  const access=stage.access[from?'entry':'exit'],openGap=access.config.ramGapMm;
  const coordinated=stage.coordinatedMotion?.[phase]?coordinatedMotionAt(stage.coordinatedMotion[phase],q):null;
  const gap=coordinated?coordinated.ramGapMm:['gauge','ungauge'].includes(phase)&&Number.isFinite(stage.gaugeRamGapMm)?stage.gaugeRamGapMm:
    phase==='fold'?0:phase==='approach'?openGap*(1-q):phase==='retract'?openGap*q:openGap;
  let result;
  if(phase!=='transfer')result=sceneWithAmounts(model,catalog,{...stage.config,ramGapMm:gap},amounts);
  if(phase!=='transfer'&&(coordinated||phase==='entry'||phase==='exit')){
    const frames=phase==='entry'?access.path.insertionKeyframes:access.path.keyframes,
      m=coordinated?accessMatrix(coordinated.pose,coordinated.pivot):accessMatrix(pathPoseAt(frames,q),access.path.pivot);
    result.bodies=result.bodies.map(body=>isPart(body)?move(body,m):body);
    result.placement={...result.placement,frame:multiply(m,result.placement.frame)};
  }
  if(phase==='transfer'){
    const transfer=plan.transfers[stageIndex],next=plan.stages[stageIndex+1],rq=Math.max(0,Math.min(1,q*3-1));
    const motion=transfer.ramMotion,fromHeight=motion.fromPunchHeightMm??0,toHeight=motion.toPunchHeightMm??0;
    const fromMount=motion.fromTipZMm+fromHeight,toMount=motion.toTipZMm+toHeight;
    const tip=fromMount+(toMount-fromMount)*rq-(q<.5?fromHeight:toHeight);
    const base=q<.5?stage:next,baseAmounts=q<.5?stage.to:next.from;
    const atGap=createEndpointSceneBuilder(model,catalog,base.config.bendId,baseAmounts);
    const closed=atGap({...base.config,ramGapMm:0}).placement.punchTipZ;
    result=atGap({...base.config,ramGapMm:tip-closed});
    const matrix=poseTransferAt(transfer,q),canonical=partBodies(model,evaluateModel(model,0,{amounts:transfer.amounts}),identity());
    result.bodies=[...canonical.map(body=>move(body,matrix)),...result.bodies.filter(body=>!isPart(body))];
    result.placement={...result.placement,frame:matrix};
  }
  let gauge=stage.backgauge;
  const adjustment=phase==='transfer'?plan.transfers[stageIndex]?.fingerAdjustment:null;
  if(adjustment?.status==='EXTERIOR_MANUAL_Y_ADJUSTMENT_GEOMETRY_CLEAR'){
    const t=Math.max(0,Math.min(1,q));
    gauge={...gauge,park:adjustment.park,stationY:adjustment.stationYFrom.map((v,i)=>v+(adjustment.stationYTo[i]-v)*t)};
  }
  if(gauge&&catalog.fingerDefinition)result=withBackgauge(result,catalog.fingerDefinition,gauge,phase,q);
  return{...result,cycle:{phase:phaseLabels[phase],gap,amount:amounts[stage.bendId]},amounts,stage,phase,q,openGap};
}
function move(body,matrix){
  return {...body,positions:transformPositions(body.positions,matrix)};
}
