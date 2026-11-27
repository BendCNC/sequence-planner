import {movesWithRam} from './tool-geometry.mjs';
/** Original 3-D solid checks for coordinated fixed-Y access. No saved routes. */
import {sceneWithAmounts,isPart,bodyBounds,translatedBody} from './sequence-scene.mjs';
import {boxDistance2,meshCollider,checkPair,rigidColliderRefitter} from './collision.mjs';
import {buildOperatorSupport,evaluateOperatorSupport,prepareRigidFrontProof} from './sequence-operator-support.mjs';
import {transformFrontTiltPoint} from './sequence-rigid-pose.mjs';
const nodeThread=globalThis.process?.versions?.node?await import('node:worker_threads'):null;
const nodeFoldPool=nodeThread&&(nodeThread.isMainThread||nodeThread.workerData?.role==='terminal-access')?await import('./sequence-fold-pool.mjs'):null;

export const accessZero=()=>({x:0,y:0,z:0,tiltDeg:0,ramGapMm:0});
export const accessInterpolate=(a,b,q)=>Object.fromEntries(['x','y','z','tiltDeg','ramGapMm'].map(k=>[k,(a[k]??0)+((b[k]??0)-(a[k]??0))*q]));
// A rigid tilt maps box extrema to corners in X/Z and leaves Y unchanged.
// Use the same arithmetic as the vertex transform, including the ram translation.
function poseBounds(box,transform,ram=0){
 const a=transform([box.min[0],0,box.min[2]+ram]),b=transform([box.min[0],0,box.max[2]+ram]);
 const c=transform([box.max[0],0,box.min[2]+ram]),d=transform([box.max[0],0,box.max[2]+ram]);
 return {min:[Math.min(a[0],b[0],c[0],d[0]),box.min[1],Math.min(a[2],b[2],c[2],d[2])],
         max:[Math.max(a[0],b[0],c[0],d[0]),box.max[1],Math.max(a[2],b[2],c[2],d[2])]};
}
function axisSeparated(a,b,margin){
 for(let k=0;k<3;k++)if(a.min[k]-b.max[k]>margin||b.min[k]-a.max[k]>margin)return true;
 return false;
}
export function createAccessWorld({model,catalog,bendId,config,amounts,obstacles=[],options={}}){
 const clearanceMm=options.clearanceMm??.15,toleranceMm=options.toleranceMm??1e-7,margin=clearanceMm+toleranceMm;
 const ramLimit=options.maxCoordinatedRamGapMm??Infinity;
 if(Number.isNaN(ramLimit)||ramLimit<0)throw new RangeError('Invalid coordinated ram limit');
 const scene=sceneWithAmounts(model,catalog,{...config,bendId,ramGapMm:0},amounts);scene.bodies.push(...obstacles);
 const pivot=[0,scene.placement.stationYmm??0,scene.placement.neutralHeight],preparedSupport=buildOperatorSupport(model,bendId);
 const rawParts=scene.bodies.filter(isPart),rawObstacles=scene.bodies.filter(b=>!isPart(b)),parts=rawParts.map(meshCollider);
 const posedRefitters=new WeakMap();
 function posedCollider(body,transform,ram=0){
  let refit=posedRefitters.get(body);
  if(!refit){refit=rigidColliderRefitter(body);posedRefitters.set(body,refit);}
  return refit((x,y,z)=>transform([x,y,z+ram]));
 }
 const staticObstacles=rawObstacles.map(b=>movesWithRam(b)?null:meshCollider(b)),fixed=staticObstacles.filter(Boolean),punches=rawObstacles.filter(movesWithRam);
 const obstacleBounds=rawObstacles.map(bodyBounds);
 const points=parts.flatMap(p=>p.positions),radius=points.reduce((r,p)=>Math.max(r,Math.hypot(p[0]-pivot[0],p[2]-pivot[2])),0);
 const front=Math.min(...rawObstacles.map(b=>bodyBounds(b).min[0]))-(options.frontMarginMm??10),bounds={min:[0,1,2].map(k=>Math.min(...parts.map(p=>p.bounds.min[k]))),max:[0,1,2].map(k=>Math.max(...parts.map(p=>p.bounds.max[k])))};
 // Pair certificates live only in this rigid access world. A clear narrow-phase
 // query supplies a lower bound on separation. In the obstacle frame the maximum
 // part displacement is translation plus radius times the rotation chord. A
 // common ram/part translation cancels. At or near the margin we query both
 // original meshes as usual, retaining the first collision and its triangle IDs.
 const certificates=Array.from({length:2},()=>rawParts.map(()=>Array(rawObstacles.length)));
 const partRadii=parts.map(part=>part.positions.reduce((max,v)=>Math.max(max,Math.hypot(v[0]-pivot[0],v[2]-pivot[2])),0));
 const worldScale=Math.max(1,...parts.flatMap(part=>[...part.bounds.min,...part.bounds.max].map(Math.abs)),
                          ...obstacleBounds.flatMap(box=>[...box.min,...box.max].map(Math.abs)),...pivot.map(Math.abs));
 // The support skin reuses material vertices across triangles. Transform
 // each used vertex once per pose; a skin wholly in front satisfies both
 // the centroid and the majority-area conditions by positive area weights.
 const supportPoints=new Map(),supportBodies=preparedSupport.majorBranch.bodies.map(skin=>{
  const mesh=parts.find(part=>part.id===skin.bodyId),indices=[...new Set(skin.triangles.flatMap(t=>t.indices))];
  const posed=new Array(mesh.positions.length);supportPoints.set(skin.bodyId,posed);return{mesh,indices,posed};
 });
 const frontMajority=prepareRigidFrontProof(preparedSupport,new Map(parts.map(p=>[p.id,p])),pivot);
 function supportAt(transform,rounding){
  let fullyFront=Number.isFinite(preparedSupport.majorBranch.areaMm2)&&preparedSupport.majorBranch.areaMm2>0;
  for(const {mesh,indices,posed} of supportBodies)for(const i of indices){
   const p=transform(mesh.positions[i]);posed[i]=p;
   if(!(p[0]<-1e-7-rounding&&p.every(Number.isFinite)))fullyFront=false;
  }
  return fullyFront?{passes:true}:evaluateOperatorSupport(preparedSupport,scene,{posedPointsById:supportPoints});
 }
 // Additional fixed projection axes provide lower distance bounds for long
 // inclined flanges whose world X/Z boxes overlap the tooling. An interval
 // gap separates the complete meshes and also excludes containment.
 const axes=Array.from({length:16},(_,i)=>i).filter(i=>i!==0&&i!==8)
  .map(i=>[Math.cos(i*Math.PI/16),Math.sin(i*Math.PI/16)]);
 // The projection depends only on X and Z. Preserve the first occurrence and
 // both signs of zero so repeated points have exactly the original extrema.
 const negativeZero=Symbol();
 function projectionVertices(points){
  const seen=new Map(),vertices=[];
  for(const v of points){
   const x=v[0],z=v[2],key=Object.is(x,-0)?negativeZero:x;
   let zs=seen.get(key);if(!zs){zs=new Set();seen.set(key,zs);}
   const zKey=Object.is(z,-0)?negativeZero:z;
   if(!zs.has(zKey)){zs.add(zKey);vertices.push(x,z);}
  }
  return Float64Array.from(vertices);
 }
 function interval(points,axis){
  const [nx,nz]=axis;let min=Infinity,max=-Infinity;
  for(let i=0;i<points.length;i+=2){
   const value=points[i]*nx+points[i+1]*nz;
   if(value<min)min=value;if(value>max)max=value;
  }
  return [min,max];
 }
 const obstacleSpans=rawObstacles.map(body=>{
  const points=[];
  for(let i=0;i<body.positions.length;i+=3)points.push([body.positions[i],body.positions[i+1],body.positions[i+2]]);
  const xz=projectionVertices(points);return axes.map(axis=>interval(xz,axis));
 }),partAngles=parts.map(()=>new Map()),partXZ=parts.map(part=>projectionVertices(part.positions));
 function projectionGap(partIndex,obstacleIndex,angle,c,s,x,z,lift,rounding){
  const cache=partAngles[partIndex];let entry=cache.get(angle);
  if(!entry){
   const src=partXZ[partIndex],posed=new Float64Array(src.length);
   for(let i=0;i<src.length;i+=2){
    const dx=src[i]-pivot[0],dz=src[i+1]-pivot[2];
    posed[i]=pivot[0]+c*dx+s*dz;posed[i+1]=pivot[2]-s*dx+c*dz;
   }
   entry={posed,spans:[]};
   if(cache.size>=4)cache.delete(cache.keys().next().value);cache.set(angle,entry);
  }
  const other=obstacleSpans[obstacleIndex];
  for(let k=0;k<axes.length;k++){
   const [nx,nz]=axes[k],translated=x*nx+(z-lift)*nz;
   const span=entry.spans[k]??=interval(entry.posed,axes[k]);
   if(span[0]+translated-other[k][1]>margin+rounding||other[k][0]-(span[1]+translated)>margin+rounding)return true;
  }
  return false;
 }
 const stats={evaluations:0,edges:0,cacheHits:0},cache=new Map(),ramCache=new Map(),liftedPunchCache=new Map();
 // Original obstacle order and mesh identity are retained. Static BVHs belong
 // to this world; at most four ram positions retain moving punch BVHs.
 const initialized=new WeakSet();
 const validThreshold=Number.isFinite(clearanceMm)&&clearanceMm>=0&&Number.isFinite(toleranceMm)&&toleranceMm>=0;
 const marginSquared=margin*margin;
 const liftedPunches=ram=>{
  if(liftedPunchCache.has(ram))return liftedPunchCache.get(ram);
  const values=[];
  for(let index=0;index<rawObstacles.length;index++){
   const body=rawObstacles[index];if(!movesWithRam(body))continue;
   const original=obstacleBounds[index],bounds={
    min:[original.min[0]+0,original.min[1]+0,original.min[2]+ram],
    max:[original.max[0]+0,original.max[1]+0,original.max[2]+ram]
   };
   // Seed the real mesh at its original first pose (topology and BVH shape
   // can depend on this), and validate any translated coordinate overflow.
   let collider=null;
   const get=()=>collider??=meshCollider(translatedBody(body,0,0,ram));
   if(!initialized.has(body)||!Number.isFinite(bounds.min[2])||!Number.isFinite(bounds.max[2])){
    get();initialized.add(body);
   }
   values.push({bounds,get});
  }
  liftedPunchCache.set(ram,values);if(liftedPunchCache.size>4)liftedPunchCache.delete(liftedPunchCache.keys().next().value);
  return values;
 };
 const key=p=>['x','y','z','tiltDeg','ramGapMm'].map(k=>(p[k]??0).toPrecision(14)).join('|');
 const valid=p=>['x','y','z','tiltDeg','ramGapMm'].every(k=>Number.isFinite(p[k]??0))&&Math.abs(p.y??0)<1e-10&&p.ramGapMm>=0&&p.ramGapMm<=ramLimit;
 function check(p,{world=false}={}){if(!valid(p))return{clear:false,reason:'INVALID_COORDINATED_POSE'};const k=key(p);if(!world&&cache.has(k)){stats.cacheHits++;return cache.get(k);}stats.evaluations++;
  const angle=(p.tiltDeg??0)*Math.PI/180,c=Math.cos(angle),s=Math.sin(angle),x=p.x??0,z=p.z??0,ram=p.ramGapMm;
  const tf=v=>{const dx=v[0]-pivot[0],dz=v[2]-pivot[2];return[pivot[0]+c*dx+s*dz+x,v[1],pivot[2]-s*dx+c*dz+z];};
  const inverse=v=>{const dx=v[0]-pivot[0]-x,dz=v[2]-pivot[2]-z;return[pivot[0]+c*dx-s*dz,v[1],pivot[2]+s*dx+c*dz];};
  const rounding=512*Number.EPSILON*(worldScale+Math.abs(x)+Math.abs(z)+Math.abs(ram));
  const pairClear=(partIndex,obstacleIndex,lift)=>{
   const prior=certificates[Number(world)][partIndex][obstacleIndex];if(!prior)return false;
   const delta=Math.hypot(x-prior.x,(z-lift)-prior.relativeZ)+partRadii[partIndex]*Math.hypot(c-prior.c,s-prior.s);
   return delta+rounding<prior.available;
  };
  // The ordinary margin query can give an intentionally loose lower bound.
  // A wider separation query is a certificate for subsequent nearby poses.
  // If it finds a potential contact, rerun at the actual margin to obtain
  // precisely the ordinary verdict, body order and contact witness.
  const certifiedPair=(partIndex,obstacleIndex,a,b)=>{
   const previous=certificates[Number(world)][partIndex][obstacleIndex],pad=previous?.pad??0;
   let nextPad=0;
   if(pad>0){
    const proof=checkPair(a,b,{clearanceMm:clearanceMm+pad,toleranceMm,queryMode:'clearance'});
    if(proof.status==='CLEAR')return{hit:proof,pad:Math.min(8,pad*2)};
    const witness=proof.witnessedSurfaceDistanceMm;
    if(Number.isFinite(witness)&&witness>margin+.5)nextPad=Math.min(pad*.5,(witness-margin)*.7);
   }
   const hit=checkPair(a,b,{clearanceMm,toleranceMm,queryMode:'clearance'});
   if(!pad&&hit.status==='CLEAR'&&hit.distanceMm<margin+.65&&
      (hit.witnessedSurfaceDistanceMm===null||hit.witnessedSurfaceDistanceMm>margin+1))nextPad=2;
   return{hit,pad:nextPad>.25?nextPad:0};
  };
  const remember=(partIndex,obstacleIndex,lift,hit,pad)=>{
   certificates[Number(world)][partIndex][obstacleIndex]={x,relativeZ:z-lift,c,s,available:hit.distanceMm-margin-rounding,pad};
  };
  let result=ramCache.get(ram);
  if(!result){result={clear:true};outer:for(const moving of liftedPunches(ram)){for(const fixedBody of fixed){if(validThreshold&&boxDistance2(moving.bounds,fixedBody.bounds)>marginSquared)continue;const collision=checkPair(moving.get(),fixedBody,{clearanceMm,toleranceMm,queryMode:'clearance'});if(collision.status==='INTERFERENCE'){result={clear:false,reason:'TOOL_TOOL',collision};break outer;}}}ramCache.set(ram,result);}
  if(result.clear){
   if(world){let punchIndex=0;const movingPunches=liftedPunches(ram),obs=rawObstacles.map((b,i)=>movesWithRam(b)?movingPunches[punchIndex++]:staticObstacles[i]);
    outer:for(const [index,raw] of rawParts.entries()){
     const box=poseBounds(parts[index].bounds,tf);let moving;
     for(const [obstacleIndex,obstacle] of obs.entries()){
      const lift=movesWithRam(rawObstacles[obstacleIndex])?ram:0;
      if(axisSeparated(box,obstacle.bounds,margin)||pairClear(index,obstacleIndex,lift)||projectionGap(index,obstacleIndex,p.tiltDeg??0,c,s,x,z,lift,rounding))continue;
      moving??=posedCollider(raw,tf);
      const {hit:collision,pad}=certifiedPair(index,obstacleIndex,moving,obstacle.get?obstacle.get():obstacle);
      if(collision.status==='INTERFERENCE'){result={clear:false,reason:'PART_OBSTACLE',collision:structuredClone(collision)};break outer;}
      remember(index,obstacleIndex,lift,collision,pad);
     }
    }
   }else outer:for(const [index,raw] of rawObstacles.entries()){
    const lift=movesWithRam(raw)?ram:0,box=poseBounds(obstacleBounds[index],inverse,lift);let obstacle;
    for(const [partIndex,part] of parts.entries()){
     if(axisSeparated(part.bounds,box,margin)||pairClear(partIndex,index,lift)||projectionGap(partIndex,index,p.tiltDeg??0,c,s,x,z,lift,rounding))continue;
     obstacle??=posedCollider(raw,inverse,lift);
     const {hit,pad}=certifiedPair(partIndex,index,part,obstacle);
     if(hit.status==='INTERFERENCE'){
      result={clear:false,reason:'PART_OBSTACLE',collision:{...structuredClone(hit),point:hit.point?tf(hit.point):null,closestPoints:hit.closestPoints?.map(tf)??null}};break outer;
     }
     remember(partIndex,index,lift,hit,pad);
    }
   }
  }
  if(result.clear){const support=frontMajority(c,s,x,rounding)?{passes:true}:supportAt(tf,rounding);if(!support.passes)result={clear:false,reason:'OPERATOR_MAIN_BRANCH_FRONT_CONSTRAINT',support};}
  if(!world)cache.set(k,result);return result;
 }
 const bound=(a,b)=>Math.hypot(b.x-a.x,b.z-a.z)+radius*Math.abs(b.tiltDeg-a.tiltDeg)*Math.PI/180+Math.abs(b.ramGapMm-a.ramGapMm);
 function edge(a,b,spacing=.5,{world=false,guard=()=>{},checkPose=check}={}){stats.edges++;const n=Math.max(1,Math.ceil(bound(a,b)/spacing));let last=a;for(let i=1;i<=n;i++){guard();const p=accessInterpolate(a,b,i/n),r=checkPose(p,{world});if(!r.clear)return{clear:false,last,blocked:p,result:r,samples:i};last=p;}return{clear:true,last:b,samples:n};}
 const maxX=p=>{let m=-Infinity;for(const v of points)m=Math.max(m,transformFrontTiltPoint(v,p,pivot)[0]);return m;};
 const outside=p=>maxX(p)<front;
 const outsidePose=p=>({...p,x:p.x+Math.min(0,front-maxX(p)-1)});
 return{scene,pivot,preparedSupport,radius,front,bounds,parts,rawParts,rawObstacles,check,edge,bound,key,outside,outsidePose,stats};
}

/** Locate the exact serial edge sample without materializing a pose array. */
export function coordinatedSampleAt(edges,index){
 let lo=0,hi=edges.length-1;
 while(lo<=hi){const middle=(lo+hi)>>1,edge=edges[middle];
  if(index<edge.first){hi=middle-1;continue;}
  if(index>=edge.first+edge.count){lo=middle+1;continue;}
  const sample=index-edge.first+1;
  return{edge:edge.edge,sample,pose:accessInterpolate(edge.a,edge.b,sample/edge.count),
   last:sample===1?edge.a:accessInterpolate(edge.a,edge.b,(sample-1)/edge.count)};
 }
 throw new RangeError('Coordinated sample index is outside the motion');
}

export function validateCoordinatedAccess(payload,motion,guard=()=>{}){
 guard();
 const world=createAccessWorld(payload),requestedStep=payload.options?.accessValidationStepMm??.25,spacing=Math.min(.25,requestedStep),frames=motion?.frames;
 if(!Number.isFinite(requestedStep)||!(spacing>0)||!frames?.length||frames.length<2||!Array.isArray(motion.pivot)||motion.pivot.length!==3||motion.pivot.some((v,i)=>!Number.isFinite(v)||Math.abs(v-world.pivot[i])>1e-8))return{status:'BLOCKED',reason:'INVALID_MOTION'};
 if(frames.some(f=>!f?.pose||!Number.isFinite(f.ramGapMm)||f.ramGapMm<0||f.ramGapMm>(payload.options?.maxCoordinatedRamGapMm??Infinity)||['x','y','z','tiltDeg'].some(k=>!Number.isFinite(f.pose[k]??0))))return{status:'BLOCKED',reason:'INVALID_MOTION'};
 const poses=frames.map(f=>({...f.pose,ramGapMm:f.ramGapMm})),zero=accessZero();
 if(Object.keys(zero).some(k=>Math.abs(poses[0][k]??0)>1e-8))return{status:'BLOCKED',reason:'MOTION_NOT_AT_WORKING_POSE'};
 const start=world.check(poses[0],{world:true});if(!start.clear)return{status:'BLOCKED',reason:'START_BLOCKED',blocked:start};
 if(payload.options?.coordinatedAccessParallelValidation!==false&&nodeFoldPool?.foldPoolSize()>1){
  const edges=[];let count=0;
  for(let i=1;i<poses.length;i++){
   const n=Math.max(1,Math.ceil(world.bound(poses[i-1],poses[i])/spacing));
   edges.push({edge:i,first:count,count:n,a:poses[i-1],b:poses[i]});count+=n;
  }
  // Preserve the existing serial behavior for unusual numeric bounds; shared
  // counters may only represent finite positive signed-32-bit sample counts.
  if(Number.isSafeInteger(count)&&count>0&&count<=0x7ffffffe){
   const remote=nodeFoldPool.evaluateCoordinatedWorldSamples({...payload,edges,count},{guard});
   if(remote){
    if(remote.firstFailure){const sample=coordinatedSampleAt(edges,remote.firstFailure.index);
     return{status:'BLOCKED',reason:'WORLD_RECHECK_INTERFERENCE',edge:sample.edge,clear:false,last:sample.last,
      blocked:sample.pose,result:remote.firstFailure.result,samples:sample.sample};
    }
    const outside=world.outside(poses.at(-1));return{status:outside?'SAMPLE_CLEAR':'BLOCKED',reason:outside?null:'NOT_FULLY_OUTSIDE',samples:count+1,wholePartOutside:outside,maxRelativePointMotionBoundMm:spacing,clearanceMm:payload.options?.clearanceMm??.15,continuousCollisionDetection:false};
   }
  }
 }
 let samples=1;for(let i=1;i<poses.length;i++){const result=world.edge(poses[i-1],poses[i],spacing,{world:true,guard});samples+=result.samples;if(!result.clear)return{status:'BLOCKED',reason:'WORLD_RECHECK_INTERFERENCE',edge:i,...result};}
 const outside=world.outside(poses.at(-1));return{status:outside?'SAMPLE_CLEAR':'BLOCKED',reason:outside?null:'NOT_FULLY_OUTSIDE',samples,wholePartOutside:outside,maxRelativePointMotionBoundMm:spacing,clearanceMm:payload.options?.clearanceMm??.15,continuousCollisionDetection:false};
}
