/** Geometric press scene. X=feed, Y=tool station, Z=ram; millimetres. */
import {evaluateModel,transformedPanel,transformPositions,transform,rotation,bandPoint,identity,multiply} from './solver.mjs';
import {buildPrism,positionBounds as boundsOf} from './tool-geometry.mjs';
import {planBackgauge,parkingPose,withBackgauge} from './backgauge.mjs';

const dot=(a,b)=>a.reduce((s,x,i)=>s+x*b[i],0);
const direction=(m,v)=>[0,1,2].map(i=>m[i*4]*v[0]+m[i*4+1]*v[1]+m[i*4+2]*v[2]);
const normalized=v=>{const n=Math.hypot(...v);return v.map(x=>x/n);};
export const toolById=(catalog,id)=>catalog.tools.find(t=>String(t.id)===String(id));

// The active tip defines Z for forming. All mounted punches share the same
// clamping plane, including while another station makes the bend.
export function mountedPunch(catalog,active,segment){
  if(segment.punchId==null||String(segment.punchId)===String(active.id))return{tool:active,tipOffsetZ:0};
  const tool=toolById(catalog,segment.punchId);
  if(tool?.kind!=='punch'||tool.status!=='ready'||![active,tool].every(t=>Number.isFinite(t.contact?.mountPlaneZMm)))
    throw new Error('Se necesitan los perfiles y las alturas de montaje de los dos punzones.');
  return{tool,tipOffsetZ:active.contact.mountPlaneZMm-tool.contact.mountPlaneZMm};
}

export function bandAxisRange(bend){
  let min=Infinity,max=-Infinity;
  for(let i=0;i<bend.band.parameters.length;i+=3){min=Math.min(min,bend.band.parameters[i]);max=Math.max(max,bend.band.parameters[i]);}
  return {min,max,center:(min+max)/2,width:max-min};
}

/** Flat-pattern point (mm, model XY) that machinePlacement puts at machine
 *  Y = stationYmm: the active bend band's axial midpoint. Consumers (Qt LIVE)
 *  must centre the part here, not on a union of split-hinge spans. */
export function placementOriginFlat(bend){
  const range=bandAxisRange(bend),o=bend.origin,a=bend.axis;
  return{pointMm:[o[0]+a[0]*range.center,o[1]+a[1]*range.center],axis:[a[0],a[1]],
    axialMinMm:range.min,axialMaxMm:range.max,bendId:String(bend.id)};
}

/** Intersect the closed material band with f=.5 and z=neutralZ, retaining holes. */
/** Tooling coverage of a stroke: own material plus joint-stroke followers. */
export function strokeMaterialIntervals(bend){
  const own=materialIntervals(bend),followers=strokeFollowers.get(bend);
  if(!followers?.length)return own;
  // Joint stroke: coaxial spans on other parents are formed by the same stroke;
  // express their material in the lead's centred axial frame and merge.
  const range=bandAxisRange(bend),s=Math.sign(bend.signedAngle),all=[...own.intervals];let exact=own.exact;
  for(const f of followers){
    const fr=bandAxisRange(f),fs=Math.sign(f.signedAngle),dir=f.axis[0]*bend.axis[0]+f.axis[1]*bend.axis[1]+f.axis[2]*bend.axis[2];
    const shift=(f.origin[0]-bend.origin[0])*bend.axis[0]+(f.origin[1]-bend.origin[1])*bend.axis[1]+(f.origin[2]-bend.origin[2])*bend.axis[2];
    const map=x=>-s*(shift+dir*(fr.center-fs*x)-range.center);
    const sub=materialIntervals(f);exact&&=sub.exact;
    for(const[a,b]of sub.intervals){const p=map(a),q=map(b);all.push([Math.min(p,q),Math.max(p,q)]);}
  }
  all.sort((a,b)=>a[0]-b[0]);
  const intervals=[];for(const i of all){const last=intervals.at(-1);if(last&&i[0]<=last[1]+1e-6)last[1]=Math.max(last[1],i[1]);else intervals.push([...i]);}
  return{intervals,exact};
}
const strokeFollowers=new WeakMap();
/** Attach joint-stroke followers to their lead bend objects (idempotent). */
export function linkJointStrokes(model){
  for(const b of model.bends)if(b.strokeFollowerIds?.length)
    strokeFollowers.set(b,model.bends.filter(f=>b.strokeFollowerIds.map(String).includes(String(f.id))));
  return model;
}
export function materialIntervals(bend){
  // Mesh vertices lying exactly on the section line can give an odd crossing
  // count (corner slivers); retry slightly off the centre line before giving up.
  // A corner-trim step lying exactly on the section also splits it spuriously:
  // keep the exact section with the fewest intervals (first wins on ties).
  let result,best=null;
  for(const v of[.5,.5+1e-4,.5-1e-4,.5+1e-3,.5-1e-3]){
    result=materialIntervalsAt(bend,v);
    // An exact section contains at least one interval. The first such
    // single interval already wins every possible tie on later sections.
    if(result.exact&&result.intervals.length===1)return result;
    if(result.exact&&(!best||result.intervals.length<best.intervals.length))best=result;
  }
  return best??result;
}
function materialIntervalsAt(bend,section){
  const data=bend.band.parameters,ids=bend.band.triangles,hits=[],eps=1e-7;
  for(let i=0;i<ids.length;i+=3){
    const f0=data[ids[i]*3+1],f1=data[ids[i+1]*3+1],f2=data[ids[i+2]*3+1];
    if(Math.max(f0,f1,f2)<section-eps||Math.min(f0,f1,f2)>section+eps)continue;
    const vertices=ids.slice(i,i+3).map(k=>data.slice(k*3,k*3+3)),line=[];
    for(let j=0;j<3;j++){
      const a=vertices[j],b=vertices[(j+1)%3],da=a[1]-section,db=b[1]-section;
      if(Math.abs(da)<eps)line.push([a[0],a[2]]);
      if(da*db < -eps*eps){const t=da/(da-db);line.push([a[0]+t*(b[0]-a[0]),a[2]+t*(b[2]-a[2])]);}
    }
    const unique=line.filter((p,j)=>!line.slice(0,j).some(v=>Math.hypot(v[0]-p[0],v[1]-p[1])<eps));
    if(unique.length!==2)continue;
    const[a,b]=unique,da=a[1]-bend.neutralZ,db=b[1]-bend.neutralZ;
    if(Math.abs(da)<eps)hits.push(a[0]);
    if(Math.abs(db)<eps)hits.push(b[0]);
    if(da*db<0)hits.push(a[0]+da/(da-db)*(b[0]-a[0]));
  }
  hits.sort((a,b)=>a-b);
  const values=hits.filter((v,i)=>i===0||v-hits[i-1]>1e-3),range=bandAxisRange(bend),s=Math.sign(bend.signedAngle);
  if(values.length<2||values.length%2)return{intervals:[[-range.width/2,range.width/2]],exact:false};
  const intervals=[];
  for(let i=0;i<values.length;i+=2){const a=-s*(values[i]-range.center),b=-s*(values[i+1]-range.center);intervals.push([Math.min(a,b),Math.max(a,b)]);}
  intervals.sort((a,b)=>a[0]-b[0]);return{intervals,exact:true};
}

export function operationAmounts(model,bendId,amount,order=model.bends.map(b=>b.id)){
  if(order.length!==model.bends.length||new Set(order.map(String)).size!==order.length||order.some(id=>!model.bends.some(b=>String(b.id)===String(id))))throw new Error('La secuencia debe contener una vez cada pliegue.');
  const index=order.findIndex(id=>String(id)===String(bendId));
  if(index<0)throw new Error('Pliegue activo no encontrado.');
  const amounts={};order.forEach((id,i)=>{amounts[id]=i<index?1:i===index?Math.max(0,Math.min(1,amount)):0;});return amounts;
}

export function pressCycle(progress){
  const q=Math.max(0,Math.min(1,progress));
  if(q<.18)return{amount:0,gap:50*(1-q/.18),phase:'Aproximación'};
  if(q<.82)return{amount:(q-.18)/.64,gap:0,phase:'Plegado'};
  return{amount:1,gap:50*(q-.82)/.18,phase:'Retirada'};
}

// Only the convex nose connected to a registered apex may determine forming Z.
// Stop before the first edge matching the catalog opening angle: the shank,
// shoulders and hooks must remain ordinary obstacles, not lift the punch.
function punchNose(punch){
  const profile=Array.isArray(punch)?punch:punch?.profile,opening=Number(punch?.contact?.openingAngleDegrees);
  const fallback=reason=>({verified:false,reason,indices:[]});
  if(!Array.isArray(profile)||!profile.length)return fallback('NO_PROFILE');
  if(profile.some(p=>!Array.isArray(p)||p.length<2||!p.slice(0,2).every(Number.isFinite)))throw new RangeError('Punch profile coordinates must be finite.');
  if(!(opening>0&&opening<180))return fallback('NO_CATALOG_OPENING_ANGLE');
  const apex=profile.map((p,i)=>({p,i})).filter(({p})=>Math.abs(p[0])<=1e-7&&Math.abs(p[1])<=1e-7);
  if(apex.length!==1||profile.some(p=>p[1]<-1e-7))return fallback('AMBIGUOUS_REGISTERED_APEX');
  const target=Math.PI/2-opening*Math.PI/360,tolerance=2*Math.PI/180;
  const sides=[];
  for(const direction of[-1,1]){
    let index=apex[0].i,previousAngle=-Infinity,side=0;const indices=[index];let reached=false;
    for(let count=0;count<profile.length-1;count++){
      const next=(index+direction+profile.length)%profile.length,a=profile[index],b=profile[next],dx=b[0]-a[0],dz=b[1]-a[1];
      if(!side)side=Math.sign(dx);
      if(!side||side*dx<=1e-10||dz<-1e-10)return fallback('NO_CONVEX_NOSE_TO_FLANK');
      const angle=Math.atan2(dz,side*dx);
      if(angle+1e-7<previousAngle)return fallback('CONCAVITY_BEFORE_FLANK');
      if(Math.abs(angle-target)<=tolerance){reached=true;break;}
      if(angle>target+tolerance)return fallback('FLANK_ANGLE_NOT_IDENTIFIED');
      indices.push(next);index=next;previousAngle=angle;
    }
    if(!reached)return fallback('FLANK_NOT_FOUND');
    sides.push({side,indices});
  }
  if(sides[0].side===sides[1].side)return fallback('NO_OPPOSITE_NOSE_SIDES');
  return{verified:true,reason:null,indices:[...new Set(sides.flatMap(s=>s.indices))],openingAngleDegrees:opening,flankAngleToleranceDegrees:2};
}

export function machinePlacement(model,state,bendId,die,{contactGapMm=.3,flip=false,stationYmm=0}={},punch=null){
  if(!Number.isFinite(stationYmm))throw new RangeError('La posición de trabajo Y debe ser finita.');
  const bend=model.bends.find(b=>String(b.id)===String(bendId));
  if(!bend)throw new Error('Pliegue desconocido.');
  const q=state.amounts[bend.id],phi=bend.signedAngle*q,angle=Math.abs(phi),half=angle/2,sign=Math.sign(bend.signedAngle);
  const parent=state.panelMatrices[bend.parentPanelId],r=rotation(bend.axis,phi/2),range=bandAxisRange(bend);
  const center=transform(parent,bandPoint(bend,range.center,.5,bend.neutralZ,q));
  const t=normalized(direction(parent,direction(r,bend.transverse))),a=normalized(direction(parent,bend.axis)),n=normalized(direction(parent,direction(r,bend.normal)));
  const tmm=model.thicknessMm,k=bend.kFactor??model.kFactor??.44;
  const opening=Number(die?.contact?.geometryOpeningMm??die?.contact?.openingMm??die?.openingMm??26),x=opening/2;
  const radius=angle>1e-7?bend.neutralLength/angle+(1-k)*tmm:Infinity,span=radius*Math.sin(half);
  const riseAt=value=>{
    const x=Math.abs(value);
    if(angle<=1e-7)return 0;
    return x<=span?x*x/(radius+Math.sqrt(Math.max(0,radius*radius-x*x))):radius*(1-Math.cos(half))+(x-span)*Math.tan(half);
  };
  // Support against the complete DXF boundary, including rounded shoulders.
  // For a straight profile edge, z-rise(x) can peak at an arc tangent inside the edge.
  let support=-riseAt(x);
  if(die?.profile?.length){
    support=-Infinity;
    for(let i=0;i<die.profile.length;i++){
      const [xa,za]=die.profile[i],[xb,zb]=die.profile[(i+1)%die.profile.length];
      support=Math.max(support,za-riseAt(xa),zb-riseAt(xb));
      if(angle>1e-7&&Math.abs(xb-xa)>1e-10){
        const slope=(zb-za)/(xb-xa),critical=slope*radius/Math.sqrt(1+slope*slope);
        if(Math.abs(critical)<=span&&critical>=Math.min(xa,xb)&&critical<=Math.max(xa,xb))
          support=Math.max(support,za+(critical-xa)*slope-riseAt(critical));
      }
    }
  }
  const verticalGap=contactGapMm/Math.max(1e-8,Math.cos(half));
  const bottom=support+verticalGap,neutralHeight=bottom+(1-k)*tmm;
  // Register only the identified local nose against the active inner section.
  // Its circular arc joined to the flange tangents is convex, so rise(x)-z
  // attains its maximum on a polygon edge at an endpoint. This is independent
  // of remote obstacles; those remain subject to the ordinary collision check.
  const punchProfile=Array.isArray(punch)?punch:punch?.profile;
  let punchProfileLiftMm=0,criticalProfileVertex=null;
  const nose=punchNose(punch),hasPunchProfile=nose.verified;
  const innerRadius=angle>1e-7?bend.neutralLength/angle-k*tmm:Infinity;
  if(hasPunchProfile){
    if(innerRadius < -1e-7)throw new RangeError('The active inner bend radius must be nonnegative.');
    const innerR=Math.max(0,innerRadius),innerSpan=innerR*Math.sin(half);
    const innerRiseAt=value=>{
      const x=Math.abs(value);if(angle<=1e-7||x===0)return 0;
      return x<=innerSpan?x*x/(innerR+Math.sqrt(Math.max(0,innerR*innerR-x*x))):
        innerR*(1-Math.cos(half))+(x-innerSpan)*Math.tan(half);
    };
    for(const i of nose.indices){
      const point=punchProfile[i];
      if(!Array.isArray(point)||point.length<2||!point.slice(0,2).every(Number.isFinite))throw new RangeError('Punch profile coordinates must be finite.');
      const lift=innerRiseAt(point[0])-point[1];
      if(lift>punchProfileLiftMm){punchProfileLiftMm=lift;criticalProfileVertex={index:i,point:point.slice(0,2)};}
    }
  }
  const xRow=t,yRow=a.map(v=>-sign*v),zRow=n.map(v=>sign*v);
  let frame=[...xRow,-dot(xRow,center),...yRow,-dot(yRow,center),...zRow,neutralHeight-dot(zRow,center),0,0,0,1];
  // A 180-degree rotation about ram axis exchanges feed/station orientation, preserving the valley.
  if(flip)frame=multiply([-1,0,0,0,0,-1,0,0,0,0,1,0,0,0,0,1],frame);
  frame[7]+=stationYmm;
  return{frame,neutralHeight,punchTipZ:bottom+tmm+verticalGap+punchProfileLiftMm,openingMm:opening,range,angle,contactGapMm,verticalGapMm:verticalGap,supportZMm:support,stationYmm,
    punchProfileLiftMm,punchRegistration:{method:hasPunchProfile?'ACTIVE_INNER_SECTION_LOCAL_NOSE_ENVELOPE':'LEGACY_APEX_ONLY_UNREGISTERED_PROFILE',nose,
      innerRadiusMm:Number.isFinite(innerRadius)?Math.max(0,innerRadius):null,criticalProfileVertex,
      profileSourceHash:Array.isArray(punch)?null:punch?.sourceHash??punch?.source?.sha256??null,
      scope:'Local convex nose registration to the active circular section and infinite flange tangents; no physical tip radius, force or forming certification. Shank, shoulders and remote collisions do not affect this placement.'}};
}

export function partBodies(model,state,frame=identity()){
  const bodies=model.panels.map(p=>{
    const positions=transformedPanel(p,state.panelMatrices[p.id]);
    transformPositions(positions,frame,positions);
    return {id:`panel:${p.id}`,kind:'panel',panelId:p.id,positions,triangles:p.triangles};
  });
  for(const b of model.bends)bodies.push({id:`band:${b.id}`,kind:'band',bendId:b.id,positions:transformPositions(state.bandPositions[b.id],frame),triangles:b.band.triangles});
  return bodies;
}

export function defaultSegments(model,bendId){
  const b=model.bends.find(b=>String(b.id)===String(bendId)),width=bandAxisRange(b).width;
  let remaining=Math.ceil(width/25.4)*25.4;const lengths=[];
  for(const length of [101.6,50.8,25.4])while(remaining>=length-1e-5){lengths.push(length);remaining-=length;}
  if(!lengths.length)lengths.push(25.4);
  const total=lengths.reduce((s,x)=>s+x,0)+(lengths.length-1)*.4;
  return ['punch','die'].flatMap(kind=>{let edge=-total/2;return lengths.map((lengthMm,i)=>{const segment={id:`${kind}-${i+1}`,kind,lengthMm,centerY:edge+lengthMm/2,offsetX:0,offsetZ:0};edge+=lengthMm+.4;return segment;});});
}

/** Registered upper/lower stock and any seated risers, in machine coordinates. */
export function buildMountedBank(catalog,punch,die,segments,punchTipZ,gap){
  const bodies=[];
  for(const segment of segments){
    if(['finger-left','finger-right'].includes(segment.id))throw new Error('Identificador de segmento reservado para el backgauge.');
    const mounted=segment.kind==='punch'?mountedPunch(catalog,punch,segment):null;
    const definition=mounted?.tool??die;
    const offsetZ=(mounted?punchTipZ+gap+mounted.tipOffsetZ:0)+(segment.offsetZ??0);
    const mesh=buildPrism(definition,segment.lengthMm,{centerY:segment.centerY,offsetX:segment.offsetX??0,offsetZ,mirrorX:segment.mirror??false});
    bodies.push({...mesh,id:segment.id,kind:segment.kind,segmentId:segment.id});
  }
  const riser=die.contact?.catalogHeightIncludesRiser?catalog.tools.find(t=>t.kind==='riser'&&t.status==='ready'):null;
  const dieSeat=Number(die.contact?.seatingZMm??Math.min(...die.profile.map(p=>p[1])));
  let tableTop=dieSeat;
  if(riser){
    const offset=dieSeat-riser.contact.seatingZMm;
    tableTop=offset+riser.contact.mountPlaneZMm;
    for(const segment of segments.filter(s=>s.kind==='die')){
      const mesh=buildPrism(riser,segment.lengthMm,{centerY:segment.centerY,offsetX:segment.offsetX??0,offsetZ:offset+(segment.offsetZ??0)});
      bodies.push({...mesh,id:`riser:${segment.id}`,kind:'riser',parentSegmentId:segment.id});
    }
  }
  return {bodies,tableTop};
}

// A segment search holds the folded shape fixed while testing many banks.
// Tool geometry, support, ram and table still come from rawMachineScene for
// each configuration. Part vertices are shared only at an identical frame.
export function createEndpointSceneBuilder(model,catalog,bendId,amounts){
  const fixedAmounts={...amounts};
  for(const bend of model.bends)if(!Number.isFinite(fixedAmounts[bend.id])||fixedAmounts[bend.id]<0||fixedAmounts[bend.id]>1)
    throw new Error(`Invalid amount for bend ${bend.id}`);
  const fixed={state:null,parts:new Map(),bounds:new WeakMap()};
  return config=>{
    const scene=rawMachineScene(model,catalog,{...config,bendId,amount:fixedAmounts[bendId]},fixedAmounts,fixed);
    return {...scene,amounts:{...fixedAmounts}};
  };
}
function rawMachineScene(model,catalog,config,explicitAmounts,fixed){
  linkJointStrokes(model);
  const bendId=config.bendId??model.bends[0].id,cycle=config.amount!==undefined?{amount:config.amount,gap:config.ramGapMm??0,phase:'Posición'}:pressCycle(config.progress??0);
  const amounts=explicitAmounts??operationAmounts(model,bendId,cycle.amount,config.order);
  const state=fixed?(fixed.state??=evaluateModel(model,0,{amounts})):evaluateModel(model,0,{amounts});
  const punch=toolById(catalog,config.punchId),die=toolById(catalog,config.dieId);
  if(!punch||!die||punch.status==='missing'||die.status==='missing')throw new Error('La herramienta seleccionada no tiene geometría disponible.');
  const placement=machinePlacement(model,state,bendId,die,config,punch);
  let bodies;
  if(fixed){
    const key=placement.frame.map(v=>Object.is(v,-0)?'-0':v).join(',');
    let parts=fixed.parts.get(key);
    if(!parts){
      parts=partBodies(model,state,placement.frame);fixed.parts.set(key,parts);
      for(const part of parts)fixed.bounds.set(part,{id:part.id,...boundsOf(part.positions)});
    }
    bodies=parts.slice();
  }else bodies=partBodies(model,state,placement.frame);
  // Explicit segment positions are absolute machine Y coordinates. A default
  // bank follows the part's station; neither option moves the table.
  const segments=config.segments??defaultSegments(model,bendId).map(s=>({...s,centerY:s.centerY+placement.stationYmm}));
  const bank=buildMountedBank(catalog,punch,die,segments,placement.punchTipZ,cycle.gap);
  for(const body of bank.bodies)bodies.push(body);
  const tableTop=bank.tableTop;
  const table=catalog.tools.find(t=>t.kind==='table'&&t.status!=='missing');
  if(table){
    const mesh=buildPrism(table,config.bedLengthMm??838.2,{centerY:0,offsetX:0,offsetZ:tableTop});
    bodies.push({...mesh,id:'table',kind:'table'});
  }
  // Optional measured X/Z sections, each confined to its own physical Y span.
  // Upper profiles use the common punch mounting plane; lower ones use the table top.
  // No part of the frame is extrapolated outside the supplied profiles.
  const sections=catalog.machine?.frameSections;
  if(sections!==undefined){
    if(!Array.isArray(sections))throw new Error('machine.frameSections must be an array');
    const ids=new Set(bodies.map(b=>b.id));
    for(const section of sections){
      const id='frame:'+section?.id;
      if(!section?.id||ids.has(id)||!['ram','bed'].includes(section.kind)||
         !Number.isFinite(section.centerY)||!Number.isFinite(section.lengthMm)||section.lengthMm<=0)
        throw new Error('Invalid or duplicate registered machine frame section');
      const vertices=section.vertices2D??section.profile,validIndex=i=>Number.isInteger(i)&&i>=0&&i<vertices.length;
      if(!Array.isArray(vertices)||vertices.length<3||!vertices.every(v=>Array.isArray(v)&&v.length===2&&v.every(Number.isFinite))||
         !Array.isArray(section.triangles2D)||!section.triangles2D.length||section.triangles2D.length%3||!section.triangles2D.every(validIndex)||
         (section.loops!==undefined&&(!Array.isArray(section.loops)||!section.loops.length||
          !section.loops.every(loop=>Array.isArray(loop)&&loop.length>=3&&loop.every(validIndex)))))
        throw new Error('Invalid machine frame profile or triangulation');
      const moving=section.kind==='ram';
      if(moving&&!Number.isFinite(punch.contact?.mountPlaneZMm))
        throw new Error('The ram section requires a registered punch mounting plane');
      const datum=moving?placement.punchTipZ+cycle.gap+punch.contact.mountPlaneZMm:tableTop;
      const mesh=buildPrism(section,section.lengthMm,{centerY:section.centerY,offsetZ:datum});
      ids.add(id);
      bodies.push({...mesh,id,kind:section.kind});
    }
  }
  const active=model.bends.find(b=>String(b.id)===String(bendId));
  const coverage=strokeMaterialIntervals(active);
  if(config.flip)coverage.intervals=coverage.intervals.map(([a,b])=>[-b,-a]).sort((a,b)=>a[0]-b[0]);
  coverage.intervals=coverage.intervals.map(([a,b])=>[a+placement.stationYmm,b+placement.stationYmm]);
  const bounds=bodies.map(body=>{
    // Each tool here is the fresh, unmodified prism built above. Keep independent scene bounds.
    if(body.bounds)return {id:body.id,min:body.bounds.min.slice(),max:body.bounds.max.slice()};
    return fixed?.bounds.get(body)??{id:body.id,...boundsOf(body.positions)};
  });
  return{bodies,state,placement,cycle,coverage,segments,bounds};
}

const gaugeCache=new WeakMap();
export function buildMachineScene(model,catalog,config,explicitAmounts){
  if(!config.backgauge||config.amount!==undefined)return rawMachineScene(model,catalog,config,explicitAmounts);
  if(!catalog.fingerDefinition)throw new Error('Falta la geometría de los fingers.');
  const q=Math.max(0,Math.min(1,config.progress??0)),base={...config,backgauge:undefined};
  let cache=gaugeCache.get(model);if(!cache){cache=new Map();gaugeCache.set(model,cache);}
  const key=JSON.stringify([catalog.fingerDefinition.id,{...config,progress:undefined}]);
  let plan=cache.get(key);
  if(!plan){
    const start=rawMachineScene(model,catalog,{...base,progress:0});
    const envelope=Array.from({length:21},(_,i)=>rawMachineScene(model,catalog,{...base,progress:i/20}).bodies).flat();
    plan=planBackgauge({scene:start,definition:catalog.fingerDefinition,settings:config.backgauge,park:parkingPose(envelope)});
    if(cache.size>=24)cache.delete(cache.keys().next().value);cache.set(key,plan);
  }
  // Complete positioning and withdrawal with the punch open and the piece
  // still. The original press cycle follows; both fingers remain obstacles.
  const phase=q<.08?'gauge':q<.1?'gauge':q<.18?'ungauge':'parked';
  const gaugeQ=q<.08?q/.08:q<.1?1:q<.18?(q-.1)/.08:0;
  const scene=rawMachineScene(model,catalog,{...base,progress:Math.max(0,(q-.18)/.82)});
  if(q<.18)scene.cycle.phase=phase==='ungauge'?'Retraer backgauge':'Posicionar backgauge';
  return withBackgauge(scene,catalog.fingerDefinition,plan,phase,gaugeQ);
}

export const partKey=x=>x.kind==='band'?`band:${x.bendId}`:`panel:${x.panelId}`;
const reliefCache=new WeakMap();
/** Joint-stroke relief corner: the follower span (band + flange) and the lead
 * band meet the bend carrying the follower's parent (band + panel) at a cut. */
export function jointStrokeReliefPairs(model){
  let relief=reliefCache.get(model);if(relief)return relief;relief=new Set();
  for(const f of model.bends)for(const oid of model.bends.find(l=>String(l.id)===String(f.strokeLeadId))?.strokeRequiresFlatBendIds??[]){
    const o=model.bends.find(x=>String(x.id)===String(oid));if(!o)continue;
    for(const x of[`band:${f.id}`,`panel:${f.childPanelId}`,`band:${f.strokeLeadId}`])for(const y of[`band:${o.id}`,`panel:${o.childPanelId}`])relief.add(`${x}|${y}`).add(`${y}|${x}`);
  }
  reliefCache.set(model,relief);return relief;
}
export function collisionPairs(model,bodies){
  const pairs=[],adjacent=new Set(model.bends.flatMap(b=>[`${b.parentPanelId}|${b.childPanelId}`,`${b.childPanelId}|${b.parentPanelId}`]));
  // Joint-stroke relief corner: the follower span (band + flange) and the bend
  // carrying its parent (band + panel) meet at a cut, not across a gap.
  const relief=jointStrokeReliefPairs(model);
  for(let i=0;i<bodies.length;i++)for(let j=i+1;j<bodies.length;j++){
    const a=bodies[i],b=bodies[j],partA=['panel','band'].includes(a.kind),partB=['panel','band'].includes(b.kind);
    if(partA&&partB){
      if(relief.has(`${partKey(a)}|${partKey(b)}`))continue;
      if(a.kind==='panel'&&b.kind==='panel'){if(!adjacent.has(`${a.panelId}|${b.panelId}`))pairs.push([i,j]);}
      else if(a.kind==='band'&&b.kind==='band')pairs.push([i,j]);
      else {const band=a.kind==='band'?a:b,panel=a.kind==='panel'?a:b,joint=model.bends.find(x=>String(x.id)===String(band.bendId));
        if(joint&&String(panel.panelId)!==String(joint.parentPanelId)&&String(panel.panelId)!==String(joint.childPanelId))pairs.push([i,j]);}
      continue;
    }
    if(partA||partB){pairs.push([i,j]);continue;}
    if(a.kind==='ram'&&b.kind==='ram')continue; // sections of the same rigid upper frame
    if((a.kind==='bed'&&['table','bed'].includes(b.kind))||(b.kind==='bed'&&['table','bed'].includes(a.kind)))continue;
    if(a.kind==='table'&&b.kind==='die'||b.kind==='table'&&a.kind==='die')continue; // seated tooling mating pair
    if(a.kind==='table'&&b.kind==='riser'||b.kind==='table'&&a.kind==='riser')continue;
    if(a.parentSegmentId===b.id||b.parentSegmentId===a.id)continue;
    if(a.kind==='table'&&b.kind==='table')continue;
    pairs.push([i,j]);
  }
  return pairs;
}
