import {movesWithRam} from './tool-geometry.mjs';
/** Y intervals of the part's rearmost edge (within tolX of max X), sorted by Y. */
export function rearEdgeSpans(bodies,tolX=.3){
  const parts=bodies.filter(b=>b.kind==='panel'||b.kind==='band');
  let maxX=-Infinity;
  for(const b of parts)for(let i=0;i<b.positions.length;i+=3)maxX=Math.max(maxX,b.positions[i]);
  const raw=[];
  for(const b of parts)for(let i=0;i<b.triangles.length;i+=3){
    const t=b.triangles,p=b.positions;
    if(Math.max(p[t[i]*3],p[t[i+1]*3],p[t[i+2]*3])<maxX-tolX)continue;
    const v=trianglePoints(b,i);
    const poly=clip(v,0,maxX-tolX,true);if(!poly.length)continue;
    raw.push([Math.min(...poly.map(p=>p[1])),Math.max(...poly.map(p=>p[1]))]);
  }
  raw.sort((a,b)=>a[0]-b[0]);
  const spans=[];
  for(const [lo,hi]of raw){const last=spans.at(-1);
    if(last&&lo<=last.maxY+.5)last.maxY=Math.max(last.maxY,hi);else spans.push({minY:lo,maxY:hi});}
  return spans.filter(s=>s.maxY-s.minY>=1);
}

/** Two fixed-Y fingers, common geometric X/R, no machine commands. */
import {buildFingerBodies} from './finger-geometry.mjs';
import {meshBounds} from './tool-geometry.mjs';
import {meshCollider,validatePackedMesh,checkPair} from './collision.mjs';
import {meshSeparationQuery} from './mesh-separation.mjs';

const part=b=>b.kind==='panel'||b.kind==='band';
const clamp=q=>Math.max(0,Math.min(1,q));
const mix=(a,b,q)=>({lowerFaceX:a.lowerFaceX+(b.lowerFaceX-a.lowerFaceX)*q,baseZ:a.baseZ+(b.baseZ-a.baseZ)*q});
const boxGap=(a,b)=>Math.hypot(...[0,1,2].map(k=>Math.max(0,a.min[k]-b.max[k],b.min[k]-a.max[k])));
// The exact first Y clip can only retain a triangle whose Y interval
// intersects the closed finger window. Touching triangles still get clipped.
function outsideY(body,i,window){
  const t=body.triangles,p=body.positions,a=p[t[i]*3+1],b=p[t[i+1]*3+1],c=p[t[i+2]*3+1];
  return Math.max(a,b,c)<window.minY||Math.min(a,b,c)>window.maxY;
}
function trianglePoints(body,i){
  const t=body.triangles,p=body.positions,a=3*t[i],b=3*t[i+1],c=3*t[i+2];
  return [[p[a],p[a+1],p[a+2]],[p[b],p[b+1],p[b+2]],[p[c],p[c+1],p[c+2]]];
}
function clip(poly,axis,bound,greater){
  const out=[];
  for(let i=0;i<poly.length;i++){
    const a=poly[i],b=poly[(i+1)%poly.length],da=(a[axis]-bound)*(greater?1:-1),db=(b[axis]-bound)*(greater?1:-1);
    if(da>=0)out.push(a);
    if((da>=0)!==(db>=0)){const t=da/(da-db);out.push(a.map((v,j)=>v+t*(b[j]-v)));}
  }return out;
}

/** Rearmost triangle point inside the actual stop-face window, including holes. */
export function rearEdgeWitness(bodies,window){
  let best=null;
  for(const body of bodies.filter(part))for(let i=0;i<body.triangles.length;i+=3){
    if(outsideY(body,i,window))continue;
    const t=body.triangles,p=body.positions,a=3*t[i],b=3*t[i+1],c=3*t[i+2];
    const padding=1e-10*(1+Math.max(Math.abs(p[a]),Math.abs(p[b]),Math.abs(p[c]),Math.abs(p[a+2]),Math.abs(p[b+2]),Math.abs(p[c+2])));
    if(best && Math.max(p[a],p[b],p[c])+padding<best.point[0] ||
        Math.max(p[a+2],p[b+2],p[c+2])+padding<window.minZ || Math.min(p[a+2],p[b+2],p[c+2])-padding>window.maxZ)continue;
    let poly=trianglePoints(body,i);
    for(const [axis,lo,hi]of [[1,window.minY,window.maxY],[2,window.minZ,window.maxZ]]){
      poly=clip(poly,axis,lo,true);poly=clip(poly,axis,hi,false);
    }
    for(const point of poly)if(!best||point[0]>best.point[0])best={point,bodyId:body.id,triangleId:i/3};
  }return best;
}

/** Z extent of the rear face the fingers touch (points within tolX of the rearmost X). */
export function rearFaceZRange(bodies,window,tolX=.05){
  const edge=rearEdgeWitness(bodies,{...window,minZ:-Infinity,maxZ:Infinity});
  if(!edge)return null;
  let lo=Infinity,hi=-Infinity;
  for(const body of bodies.filter(part))for(let i=0;i<body.triangles.length;i+=3){
    if(outsideY(body,i,window))continue;
    const t=body.triangles,p=body.positions;
    const high=Math.max(p[3*t[i]],p[3*t[i+1]],p[3*t[i+2]]);
    if(high+1e-10*(1+Math.abs(high)+Math.abs(edge.point[0]))<edge.point[0]-tolX)continue;
    let poly=trianglePoints(body,i);
    poly=clip(poly,1,window.minY,true);poly=clip(poly,1,window.maxY,false);poly=clip(poly,0,edge.point[0]-tolX,true);
    for(const p of poly){lo=Math.min(lo,p[2]);hi=Math.max(hi,p[2]);}
  }
  return Number.isFinite(lo)?{minZ:lo,maxZ:hi,centerZ:(lo+hi)/2,edgeX:edge.point[0]}:null;
}

/** Rear-facing (+X normal) planar faces inside the Y window, grouped by X,
 * sorted by Z extent (largest first). */
export function rearFacingFaces(bodies,window,tolX=.1,sign=1){
  const groups=[];
  for(const body of bodies.filter(part))for(let i=0;i<body.triangles.length;i+=3){
    if(outsideY(body,i,window))continue;
    const mesh=body.positions,ids=body.triangles,j=3*ids[i],k=3*ids[i+1],l=3*ids[i+2];
    const ay=mesh[k+1]-mesh[j+1],az=mesh[k+2]-mesh[j+2],by=mesh[l+1]-mesh[j+1],bz=mesh[l+2]-mesh[j+2];
    const nx=ay*bz-az*by;if(sign*nx<=0)continue;
    const ax=mesh[k]-mesh[j],bx=mesh[l]-mesh[j],len=Math.hypot(nx,az*bx-ax*bz,ax*by-ay*bx);
    if(!(len>0)||sign*nx/len<.98)continue;
    const v=trianglePoints(body,i);
    let poly=clip(v,1,window.minY,true);poly=clip(poly,1,window.maxY,false);if(!poly.length)continue;
    const x=poly.reduce((s,p)=>s+p[0],0)/poly.length;
    let g=groups.find(g=>Math.abs(g.x-x)<tolX);if(!g)groups.push(g={x,minZ:Infinity,maxZ:-Infinity});
    for(const p of poly){g.minZ=Math.min(g.minZ,p[2]);g.maxZ=Math.max(g.maxZ,p[2]);}
  }
  return groups.map(g=>({minZ:g.minZ,maxZ:g.maxZ,centerZ:(g.minZ+g.maxZ)/2,edgeX:g.x,heightMm:g.maxZ-g.minZ}))
    .sort((p,q)=>q.heightMm-p.heightMm);
}

export const FINGER_DIE_GAP_MM=.5;

export function backgaugeSettings(settings={}){
  const separationMm=settings.separationMm??100,centerYmm=settings.centerYmm??0;
  if(!Number.isFinite(separationMm)||separationMm<51||!Number.isFinite(centerYmm))throw new RangeError('La separación de referencia de los fingers debe ser ≥ 51 mm y su centro debe ser finito.');
  return{separationMm,centerYmm,stationY:[centerYmm-separationMm/2,centerYmm+separationMm/2],
    calibrated:false,automaticYMotion:false,travelLimitsVerified:false};
}

export function parkingPose(bodies){
  // Put the entire finger beyond the sampled part/tool envelope, not a fixed
  // machine stroke. This is an uncalibrated geometric requirement.
  let maxX=0;
  for(const body of bodies){
    const vertices=body.positions;
    for(let i=0;i<vertices.length;i+=3)maxX=Math.max(maxX,vertices[i]);
  }
  return{lowerFaceX:maxX+25.4,baseZ:50};
}

/** Plan R positioning while retracted, then X approach/withdrawal with the piece quiet. */
/** Finger layout from the part width: one finger when it covers >= 80% of the
 * part, else two fingers with a fixed inner gap, centred on the part. */
export const FINGER_GAP_MM=12.7,SINGLE_FINGER_COVERAGE=.8;
export function autoFingerLayout(bodies,definition,gapMm=FINGER_GAP_MM){
  const face=definition.contactFeatures.lower,fingerWidth=face.maxY-face.minY;
  const parts=bodies.filter(part);if(!parts.length||!(fingerWidth>0))return null;
  let lo=Infinity,hi=-Infinity;
  for(const b of parts)for(let i=1;i<b.positions.length;i+=3){lo=Math.min(lo,b.positions[i]);hi=Math.max(hi,b.positions[i]);}
  const partWidth=hi-lo,centerYmm=(lo+hi)/2-(face.minY+face.maxY)/2;
  if(fingerWidth>=SINGLE_FINGER_COVERAGE*partWidth)
    return{layout:'SINGLE_FINGER',partWidthMm:partWidth,fingerWidthMm:fingerWidth,centerYmm,separationMm:0,stationY:[centerYmm]};
  const separationMm=fingerWidth+gapMm;
  return{layout:'TWO_FINGERS',partWidthMm:partWidth,fingerWidthMm:fingerWidth,centerYmm,separationMm,
    stationY:[centerYmm-separationMm/2,centerYmm+separationMm/2]};
}

/** Two fingers as far apart as the part allows (more stable): try the
 * widest spacing first, narrowing down to the minimum-gap layout; accept
 * the first spacing where both fingers touch and the route is clear. */
export function planBackgauge(args){
  const {scene,definition,settings={}}=args;
  const auto=settings.fixedStations?null:autoFingerLayout(scene.bodies,definition,settings.fingerGapMm??FINGER_GAP_MM);
  if(!args.widest||auto?.layout!=='TWO_FINGERS')return planBackgaugeAt(args);
  const min=auto.separationMm;
  // Narrow rear tabs (e.g. corner tabs) are missed by a coarse symmetric
  // sweep: put each finger directly on the outermost rearmost edge stretches.
  const face=definition.contactFeatures.lower,c=(face.minY+face.maxY)/2,w=(face.maxY-face.minY)/2;
  const spans=rearEdgeSpans(scene.bodies),centre=s=>s.maxY-s.minY<2*w?(s.minY+s.maxY)/2:null;
  const pairs=[];
  for(let i=0;i<spans.length;i++)for(let j=spans.length-1;j>i;j--){
    const yl=centre(spans[i])??spans[i].minY+w,yr=centre(spans[j])??spans[j].maxY-w;
    if(yr-yl>=min-1e-6){pairs.push([yl,yr]);continue;}
    // Narrow corner tabs closer than the finger gap: slide each finger
    // outward while it still overlaps its tab by a contact margin.
    const m=Math.min(3,(spans[i].maxY-spans[i].minY)/2,(spans[j].maxY-spans[j].minY)/2);
    const ol=spans[i].maxY-m-w,or=spans[j].minY+m+w;
    if(or-ol>=min-1e-6)pairs.push([ol,or]);
  }
  pairs.sort((a,b)=>(b[1]-b[0])-(a[1]-a[0]));
  for(const [yl,yr]of pairs.slice(0,6)){
    const r=planBackgaugeAt({...args,settings:{...settings,fixedStations:true,separationMm:yr-yl,centerYmm:(yl+yr)/2-c}});
    if(r.status==='SAMPLE_CLEAR'&&r.contactCount===2)
      return{...r,layout:'TWO_FINGERS',partWidthMm:auto.partWidthMm,fingerWidthMm:auto.fingerWidthMm,widestSpacing:true,rearEdgeSpans:spans};
  }
  const widest=auto.partWidthMm-auto.fingerWidthMm-1;
  const step=Math.max(5,(widest-min)/3);
  for(let sep=widest;sep>min+1e-6;sep-=step){
    const r=planBackgaugeAt({...args,settings:{...settings,fixedStations:true,separationMm:sep,centerYmm:auto.centerYmm}});
    if(r.status==='SAMPLE_CLEAR'&&r.contactCount===2)
      return{...r,layout:'TWO_FINGERS',partWidthMm:auto.partWidthMm,fingerWidthMm:auto.fingerWidthMm,widestSpacing:true};
  }
  return planBackgaugeAt(args);
}
function planBackgaugeAt({scene,definition,settings={},park=parkingPose(scene.bodies),stepMm=2,clearanceMm=.15}){
  const auto=settings.fixedStations?null:autoFingerLayout(scene.bodies,definition,settings.fingerGapMm??FINGER_GAP_MM);
  const setup=auto?{...backgaugeSettings({}),...auto}:backgaugeSettings(settings);
  const face=definition.contactFeatures.lower;
  // Centre the finger stop face on the face it touches (R may go negative).
  const faces=setup.stationY.map(y=>rearFaceZRange(scene.bodies,{minY:y+face.minY,maxY:y+face.maxY})).filter(Boolean);
  const rear=faces.length?faces.reduce((a,b)=>b.edgeX>a.edgeX?b:a):null;
  const nominal=rear?rear.centerZ:scene.placement.neutralHeight;
  // R = height of the finger stop-face bottom above the die top (in):
  // R = 0 is the finger sliding on the die.
  const dies=scene.bodies.filter(b=>b.kind==='die').map(b=>meshBounds(b.positions));
  const dieTopZ=dies.length?Math.max(...dies.map(d=>d.max[2])):0;
  const rIn=(r,mode)=>{if(!r.contact)return undefined;const v=mode==='zero'?0:(r.contact.baseZ+face.minZ-dieTopZ)/25.4;
    return Math.round((mode==='nonneg'?Math.max(0,v):v)*1e4)/1e4;};
  const tag=(r,clamp)=>({...r,touchedFaceZ:rear?{minZ:rear.minZ,maxZ:rear.maxZ}:null,
    rPositionIn:rIn(r,clamp)??r.rPositionIn});
  const run=z=>gaugeAtHeight({scene,definition,setup,park,stepMm,clearanceMm,referenceZ:z});
  // Rear edge inside the die footprint: the finger slides on the die top
  // (R = 0) and touches the sheet there, holding the bend axis.
  if(rear&&dies.length){
    const dieRearX=Math.max(...dies.map(d=>d.max[0])),dieTop=Math.max(...dies.map(d=>d.max[2]));
    if(rear.edgeX<=dieRearX+1e-6){
      const baseZ=dieTop+clearanceMm+.01-meshBounds(definition.collisionMesh.positions).min[2];
      const onDie=run(baseZ+(face.minZ+face.maxZ)/2);
      if(onDie.status==='SAMPLE_CLEAR')return tag({...onDie,referenceMethod:'ON_DIE',dieTopZ:dieTop,dieRearX},'zero');
    }
  }
  // Prefer the largest rear-facing face the finger can reach: centre on it
  // and accept only when the finger actually stops on that face.
  if(rear){
    const big=rearFacingFaces(scene.bodies,{minY:setup.stationY[0]+face.minY,maxY:setup.stationY.at(-1)+face.maxY});
    for(const f of big.slice(0,4)){
      if(f.heightMm<=rear.maxZ-rear.minZ+1e-6)break;
      if(dies.length&&f.edgeX<=Math.max(...dies.map(d=>d.max[0])))continue; // must be behind the die
      const r=run(f.centerZ);
      if(r.status==='SAMPLE_CLEAR'&&r.contact&&Math.abs(r.contact.lowerFaceX-f.edgeX)<clearanceMm+.5)
        return{...tag({...r,referenceMethod:'LARGEST_FACE'}),touchedFaceZ:{minZ:f.minZ,maxZ:f.maxZ}};
    }
  }
  const first=run(nominal);if(first.status==='SAMPLE_CLEAR')return tag({...first,referenceMethod:'FACE_CENTER'});
  // A preformed flange can present its rear edge above or below the bend line.
  // Use actual triangle points in the fixed fingers' Y windows; change only R
  // while parked. Every alternative repeats the complete two-finger X/R check.
  const heights=setup.stationY.map(y=>
    rearEdgeWitness(scene.bodies,{minY:y+face.minY,maxY:y+face.maxY,minZ:-Infinity,maxZ:Infinity})?.point[2])
    .filter(Number.isFinite).sort((a,b)=>Math.abs(a-nominal)-Math.abs(b-nominal));
  const tried=[nominal];
  for(const z of heights){if(tried.some(v=>Math.abs(v-z)<1e-6))continue;tried.push(z);
    const alternative=run(z);if(alternative.status==='SAMPLE_CLEAR')return tag({...alternative,referenceMethod:'REAR_EDGE_HEIGHT',referenceHeightsTriedMm:tried});}
  // A short rear flange can end above the die shoulder. Raise the bottom
  // of the same stop face over the shoulder while it still spans that edge.
  const blockingDie=scene.bodies.find(b=>b.kind==='die'&&first.collision?.bodyIds.includes(b.id));
  if(blockingDie&&first.contact){
    // Finger bottom kept at least FINGER_DIE_GAP_MM above the die top.
    const dieTop=meshBounds(blockingDie.positions).max[2];
    const baseZ=dieTop+Math.max(clearanceMm,FINGER_DIE_GAP_MM)+1e-4-meshBounds(definition.collisionMesh.positions).min[2];
    const z=baseZ+(face.minZ+face.maxZ)/2;
    tried.push(z);
    const raised=run(z);
    if(raised.status==='SAMPLE_CLEAR'&&Math.abs(raised.contact.lowerFaceX-first.contact.lowerFaceX)<1e-5&&
      raised.contacts.some((c,i)=>c.used&&first.contacts[i].used&&c.witness.bodyId===first.contacts[i].witness.bodyId))
      return tag({...raised,referenceMethod:'REAR_EDGE_ABOVE_DIE',dieTopZ:dieTop,referenceHeightsTriedMm:tried},'nonneg');
  }
  return tag({...first,referenceHeightsTriedMm:tried});
}
function gaugeAtHeight({scene,definition,setup,park,stepMm,clearanceMm,referenceZ}){
  if(!(stepMm>0)||!Number.isFinite(stepMm))throw new RangeError('Invalid backgauge sample step.');
  if(!Number.isFinite(clearanceMm)||clearanceMm<0)throw new RangeError('Invalid clearance/tolerance');
  const face=definition.contactFeatures.lower,gapMm=.3;
  const baseZ=referenceZ-(face.minZ+face.maxZ)/2;
  const witnesses=setup.stationY.map(y=>rearEdgeWitness(scene.bodies,{minY:y+face.minY,maxY:y+face.maxY,minZ:baseZ+face.minZ,maxZ:baseZ+face.maxZ}));
  const valid=witnesses.filter(Boolean),edgeX=valid.length?Math.max(...valid.map(w=>w.point[0])):null;
  const contact=edgeX===null?null:{lowerFaceX:edgeX+gapMm-face.planeX,baseZ};
  const contacts=witnesses.map((w,index)=>({fingerId:index?'finger-right':'finger-left',witness:w,
    used:Boolean(w&&Math.abs(w.point[0]-edgeX)<1e-5),nominalGapMm:w?edgeX+gapMm-w.point[0]:null}));
  const result={...setup,definitionId:definition.id,park,contact,contacts,referenceZ,referenceMethod:'BEND_HEIGHT',contactCount:contacts.filter(c=>c.used).length,
    status:contact?'PENDING':'NO_REFERENCE_EDGE',keyframes:[],samplesChecked:0,maxStepMm:stepMm,
    numericalStandOffMm:gapMm,continuousCollisionDetection:false,squarenessVerified:false,
    coordinateReference:'Nominal lower stop face X and finger base Z; not calibrated X/R commands.'};
  if(!contact)return result;
  if(park.lowerFaceX<contact.lowerFaceX+clearanceMm){result.status='INSUFFICIENT_RETRACTION';return result;}
  const atHeight={lowerFaceX:park.lowerFaceX,baseZ},frames=[park,atHeight,contact];
  const obstacles=scene.bodies.filter(b=>b.kind!=='finger').map(body=>({body,bounds:meshBounds(body.positions),collider:null}));
  let localFingerBounds;
  const prove=meshSeparationQuery();
  const meshes=setup.stationY.map((_,i)=>i===1?definition.rightCollisionMesh??definition.collisionMesh:definition.collisionMesh);
  function translatedBounds(box,x,y,z){
    return {min:[box.min[0]+x,box.min[1]+y,box.min[2]+z],max:[box.max[0]+x,box.max[1]+y,box.max[2]+z]};
  }
  for(let leg=1;leg<frames.length;leg++){
    const a=frames[leg-1],b=frames[leg],n=Math.max(1,Math.ceil(Math.hypot(b.lowerFaceX-a.lowerFaceX,b.baseZ-a.baseZ)/stepMm));
    if(result.samplesChecked+n+1>2000){result.status='GAUGE_SAMPLE_BUDGET';return result;}
    const nearby=meshes.map((mesh,f)=>obstacles.filter(o=>prove(mesh,o.body,a.lowerFaceX,setup.stationY[f],a.baseZ,
      clearanceMm+1e-7,b.lowerFaceX-a.lowerFaceX,0,b.baseZ-a.baseZ)===null));
    const mutualClear=meshes.length===1||prove(meshes[0],meshes[1],0,setup.stationY[0]-setup.stationY[1],0,clearanceMm+1e-7)!==null;
    if(mutualClear&&nearby.every(rows=>!rows.length)){
      if(!localFingerBounds){
        const initial=buildFingerBodies({definition,...a,stationY:setup.stationY});
        initial.forEach(validatePackedMesh);localFingerBounds=meshes.map(m=>meshBounds(m.positions));
      }
      result.samplesChecked+=n+1;continue;
    }
    for(let i=0;i<=n;i++){
      const pose=mix(a,b,i/n);result.samplesChecked++;
      let meshes;
      const actual=()=>meshes??=buildFingerBodies({definition,...pose,stationY:setup.stationY});
      let fingers;
      if(!localFingerBounds){
        fingers=actual();
        localFingerBounds=setup.stationY.map((_,index)=>meshBounds(
          (index===1?definition.rightCollisionMesh??definition.collisionMesh:definition.collisionMesh).positions));
      }else{
        fingers=setup.stationY.map((stationY,index)=>({bounds:translatedBounds(localFingerBounds[index],pose.lowerFaceX,stationY,pose.baseZ)}));
        if(fingers.some(f=>!f.bounds.min.every(Number.isFinite)||!f.bounds.max.every(Number.isFinite)))fingers=actual();
      }
      // Envelope separation proves clearance, including containment. Construct
      // a collider only for a finger with an overlapping obstacle or partner.
      // Both handed meshes must still pass collider validation on their first pose.
      const colliders=result.samplesChecked===1?fingers.map(validatePackedMesh):[];
      const fingerCollider=i=>colliders[i]??=meshCollider(actual()[i]);
      if(fingers.length===2){
        const left=fingers[0].bounds,right=fingers[1].bounds;
        const gapY=Math.max(0,left.min[1]-right.max[1],right.min[1]-left.max[1]);
        // Use the same squared threshold as checkPair. A single separated
        // axis suffices; for all other cases the full solid check stays active.
        if(!(gapY*gapY>(clearanceMm+1e-7)**2)){
          const mutual=checkPair(fingerCollider(0),fingerCollider(1),{clearanceMm,queryMode:'clearance'});
          if(mutual.status==='INTERFERENCE'){result.status='GAUGE_INTERFERENCE';result.collision=mutual;result.blockedPose=pose;return result;}
        }
      }
      for(let f=0;f<fingers.length;f++)for(const obstacle of nearby[f]){
        if(boxGap(fingers[f].bounds,obstacle.bounds)>clearanceMm)continue;
        obstacle.collider??=meshCollider(obstacle.body);
        const hit=checkPair(fingerCollider(f),obstacle.collider,{clearanceMm,queryMode:'clearance'});
        if(hit.status==='INTERFERENCE'){result.status='GAUGE_INTERFERENCE';result.collision=hit;result.blockedPose=pose;return result;}
      }
    }
  }
  // The fingers hold the part while the ram closes to the pinch point, so
  // sweep every punch body from the gauged opening down to gap 0 with the
  // fingers at contact.
  const gap=scene.cycle?.gap??0;
  if(gap>0){
    const fingers=buildFingerBodies({definition,...contact,stationY:setup.stationY}),colliders=fingers.map(validatePackedMesh);
    const punches=scene.bodies.filter(movesWithRam).map(body=>({body,box:meshBounds(body.positions)})),n=Math.max(1,Math.ceil(gap/stepMm));
    if(result.samplesChecked+n+1>2000){result.status='GAUGE_SAMPLE_BUDGET';return result;}
    for(const punch of punches)punch.fingers=fingers.map((finger,index)=>({finger,index})).filter(({finger})=>
      prove(punch.body,finger,0,0,0,clearanceMm+1e-7,0,0,-gap)===null);
    for(let i=0;i<=n;i++){
      const dz=gap*i/n;result.samplesChecked++;
      for(const {body,box,fingers:nearby} of punches){
        const bounds={min:[box.min[0],box.min[1],box.min[2]-dz],max:[box.max[0],box.max[1],box.max[2]-dz]};
        let moved;
        for(const {index:f} of nearby){
          if(boxGap(fingers[f].bounds,bounds)>clearanceMm)continue;
          if(!moved){
            const positions=Float64Array.from(body.positions,(v,k)=>k%3===2?v-dz:v);
            moved=meshCollider({...body,positions});
          }
          const hit=checkPair(colliders[f]??=meshCollider(fingers[f]),moved,{clearanceMm,queryMode:'clearance'});
          if(hit.status==='INTERFERENCE'){result.status='GAUGE_PUNCH_INTERFERENCE';result.collision=hit;
            result.blockedPose={...contact,ramGapMm:gap-dz};return result;}
        }
      }
    }
    result.pinchSweepMm=gap;
  }
  result.keyframes=frames;result.status='SAMPLE_CLEAR';return result;
}

export function gaugePoseAt(plan,phase,q){
  if(plan.status!=='SAMPLE_CLEAR'||!['gauge','ungauge'].includes(phase))return plan.park;
  const t=clamp(phase==='ungauge'?1-q:q),frames=plan.keyframes;
  // Half the animation for R, half for X. No longitudinal Y actuation.
  return t<.5?mix(frames[0],frames[1],t*2):mix(frames[1],frames[2],(t-.5)*2);
}

export function withBackgauge(scene,definition,plan,phase='parked',q=0){
  const pose=gaugePoseAt(plan,phase,q),bodies=buildFingerBodies({definition,...pose,stationY:plan.stationY});
  return{...scene,bodies:[...scene.bodies.filter(b=>b.kind!=='finger'),...bodies],backgauge:{...plan,pose,phase:plan.status==='SAMPLE_CLEAR'?phase:'parked'}};
}
