/** Generic explicit-amount adapter: no saved part, bend ID or program is assumed. */
import {buildMachineScene,collisionPairs,jointStrokeReliefPairs,partKey} from './machine.mjs';
import {meshCollider,meshColliderForScene,checkPair,knownMeshClosure} from './collision.mjs';
import {jointFarPatches,bandCornerFarPatches,panelReliefFarPatches} from './sequence-joint-collision.mjs';
import {positionBounds} from './tool-geometry.mjs';
import {meshSeparationQuery} from './mesh-separation.mjs';
export const isPart=body=>body.kind==='panel'||body.kind==='band';
export function translatedBody(body,dx=0,dy=0,dz=0){
  const positions=new Float64Array(body.positions.length);
  for(let i=0;i<positions.length;i+=3){positions[i]=body.positions[i]+dx;positions[i+1]=body.positions[i+1]+dy;positions[i+2]=body.positions[i+2]+dz;}
  return{...body,positions};
}
export function sceneWithAmounts(model,catalog,config,amounts){
  for(const bend of model.bends)if(!Number.isFinite(amounts[bend.id])||amounts[bend.id]<0||amounts[bend.id]>1)throw new Error(`Invalid amount for bend ${bend.id}`);
  const stateAmounts={...amounts};
  const scene=buildMachineScene(model,catalog,{...config,amount:amounts[config.bendId]},stateAmounts);
  return {...scene,amounts:stateAmounts};
}
export function bodyBounds(body){return positionBounds(body.positions);}
export function intendedToolJoint(a,b){
  if(!['punch','die','riser'].includes(a.kind)||!['punch','die','riser'].includes(b.kind)||!a.placement||!b.placement)return false;
  // Lower stack: a die segment and the riser under its neighbour touch end to
  // end (the riser can be a fraction of a mm wider than the die).
  const lower=k=>k==='die'||k==='riser';
  if(a.kind!==b.kind&&!(lower(a.kind)&&lower(b.kind)))return false;
  if(a.kind===b.kind&&(a.toolId!==b.toolId||a.placement.offsetZ!==b.placement.offsetZ))return false;
  if(a.placement.offsetX!==b.placement.offsetX||a.placement.mirrorX!==b.placement.mirrorX)return false;
  return Math.abs(Math.abs(a.placement.centerY-b.placement.centerY)-(a.lengthMm+b.lengthMm)/2)<1e-7;
}
/** Default for checkScene's optional broad phase ('off' or 'bvh'). */
export const DEFAULT_COLLISION_BROADPHASE='off';
const preparedPatchShapes=new WeakSet();
const boxGap2=(min,max,box)=>{let d=0;for(let k=0;k<3;k++){const g=Math.max(0,min[k]-box.max[k],box.min[k]-max[k]);d+=g*g;}return d;};
/** Smallest squared distance from any triangle box of `body` to `box`. Lower bound on surface distance to anything inside `box`. */
function trianglesToBox2(body,box,cutoff2){
  const p=body.positions,t=body.triangles,min=[0,0,0],max=[0,0,0];let best=Infinity;
  const tri=t&&t.length?t:null,count=tri?tri.length/3:p.length/9;
  for(let f=0;f<count;f++){
    for(let k=0;k<3;k++){min[k]=Infinity;max[k]=-Infinity;}
    for(let v=0;v<3;v++){const i=(tri?tri[f*3+v]:f*3+v)*3;for(let k=0;k<3;k++){const x=p[i+k];if(x<min[k])min[k]=x;if(x>max[k])max[k]=x;}}
    const d=boxGap2(min,max,box);if(d<best){best=d;if(best<=cutoff2)return best;}
  }
  return best;
}
/**
 * Conservative proof that two bodies cannot be within `threshold`: every triangle
 * box of A is farther than threshold from B's box and vice versa. This also rules
 * out closed-solid containment (each body's vertices lie outside the other's box).
 * Returns a lower bound on the distance, or null when separation is not proven.
 */
export function provenSeparation(bodyA,boxA,bodyB,boxB,threshold){
  const cutoff2=threshold*threshold;
  const a=trianglesToBox2(bodyA,boxB,cutoff2);if(a<=cutoff2)return null;
  const b=trianglesToBox2(bodyB,boxA,cutoff2);if(b<=cutoff2)return null;
  return Math.sqrt(Math.max(a,b));
}
// A separating projection of both entire surfaces also separates their enclosed
// volumes. Overlapping intervals, even for a concave body, prove nothing.
function jointProjectionGap(patch,body,axes,tolerance){
  const a=patch.positions,b=body.positions;
  for(const axis of [axes[1],axes[2],axes[0]]){
    const [x,y,z]=axis;
    let loA=Infinity,hiA=-Infinity,magnitude=0;
    for(let i=0;i<a.length;i+=3){
      const px=x*a[i],py=y*a[i+1],pz=z*a[i+2],v=px+py+pz;
      loA=Math.min(loA,v);hiA=Math.max(hiA,v);magnitude=Math.max(magnitude,Math.abs(px)+Math.abs(py)+Math.abs(pz));
    }
    let loB=Infinity,hiB=-Infinity;
    for(let i=0;i<b.length;i+=3){
      const px=x*b[i],py=y*b[i+1],pz=z*b[i+2],v=px+py+pz;
      loB=Math.min(loB,v);hiB=Math.max(hiB,v);magnitude=Math.max(magnitude,Math.abs(px)+Math.abs(py)+Math.abs(pz));
      if(loA<=hiB&&loB<=hiA)break; // Further vertices can only widen this interval.
    }
    const length=Math.hypot(x,y,z);
    // Round inward by substantially more than a dot product's accumulated error,
    // including when large coordinate contributions nearly cancel.
    const gap=(Math.max(loA-hiB,loB-hiA)-1e-10*Math.max(1,magnitude))/length;
    if(Number.isFinite(gap)&&gap>tolerance)return gap;
  }
  return null;
}
export function checkScene(model,scene,{clearanceMm=.15,toleranceMm=1e-7,firstOnly=true,onlyIds=null,immutablePartColliders=null,projectJointBounds=false,collisionBroadphase=DEFAULT_COLLISION_BROADPHASE}={}){
  const {bodies}=scene,boxes=bodies.map(bodyBounds),cache=new Map(),hits=[],jointExceptions=[];let intendedJointContactCount=0;
  const separated=projectJointBounds?meshSeparationQuery():null;
  const closure=new Map();
  function collider(i){
    if(!cache.has(i)){
      const body=bodies[i];
      if(immutablePartColliders&&isPart(body)){
        if(!immutablePartColliders.has(body))immutablePartColliders.set(body,meshCollider(body));
        cache.set(i,immutablePartColliders.get(body));
      }else cache.set(i,meshColliderForScene(body));
    }
    return cache.get(i);
  }
  function closed(i){
    if(!closure.has(i))closure.set(i,cache.get(i)?.closed??(projectJointBounds?knownMeshClosure(bodies[i]):undefined)??collider(i).closed);
    return closure.get(i);
  }
  let minimum=Infinity,broadphaseSkipped=0;
  if(!['off','bvh'].includes(collisionBroadphase))throw new RangeError('collisionBroadphase must be off or bvh');
  const pairs=collisionPairs(model,bodies),present=new Set(pairs.map(([a,b])=>`${a}|${b}`));
  // Historical renderer pairs omit entire adjacent bodies. The sequence engine
  // must also inspect those pairs, with a local seam exception where necessary.
  const relief=jointStrokeReliefPairs(model);
  for(let a=0;a<bodies.length;a++)for(let b=a+1;b<bodies.length;b++)if(isPart(bodies[a])&&isPart(bodies[b])&&!present.has(`${a}|${b}`)&&
    !relief.has(`${partKey(bodies[a])}|${partKey(bodies[b])}`))pairs.push([a,b]);
  for(const [a,b]of pairs){
    if(onlyIds&&!onlyIds.has(bodies[a].id)&&!onlyIds.has(bodies[b].id))continue;
    if(intendedToolJoint(bodies[a],bodies[b])){intendedJointContactCount++;continue;}
    const gap=Math.hypot(...[0,1,2].map(k=>Math.max(0,boxes[a].min[k]-boxes[b].max[k],boxes[b].min[k]-boxes[a].max[k])));
    if(gap>clearanceMm+toleranceMm){minimum=Math.min(minimum,gap);continue;}
    const pa=bodies[a].kind==='panel'?a:bodies[b].kind==='panel'?b:null;
    const ba=bodies[a].kind==='band'?a:bodies[b].kind==='band'?b:null;
    const joint=pa!==null&&ba!==null?model.bends.find(j=>String(j.id)===String(bodies[ba].bendId)&&
      [String(j.parentPanelId),String(j.childPanelId)].includes(String(bodies[pa].panelId))):null;
    const selfPair=isPart(bodies[a])&&isPart(bodies[b]);
    if(projectJointBounds&&!selfPair){
      const d=separated(bodies[a],bodies[b],0,0,0,clearanceMm+toleranceMm);
      if(d!==null){minimum=Math.min(minimum,d);continue;}
    }
    // Optional broad phase: only skips pairs proven farther than the pair clearance.
    // Joint/corner pairs always go to the exact path (they touch by construction).
    if(collisionBroadphase==='bvh'&&!joint&&!(bodies[a].kind==='band'&&bodies[b].kind==='band')){
      const bound=provenSeparation(bodies[a],boxes[a],bodies[b],boxes[b],(selfPair?0:clearanceMm)+toleranceMm);
      if(bound!==null){minimum=Math.min(minimum,bound);broadphaseSkipped++;continue;}
    }
    let hit,jointDiagnostic;const checks=[];
    // A clipped patch has one indexed occurrence of every emitted vertex.
    // Its vertex bounds are therefore its BVH root bounds. A disjoint root
    // gives the exact same clearance-query lower bound without constructing
    // the tree, and excludes containment in either direction.
    const queuePatch=(patch,otherIndex,reusedShape=false,axes)=>{
      if(!patch)return;
      if(projectJointBounds&&axes){
        const gap=jointProjectionGap(patch,bodies[otherIndex],axes,toleranceMm);
        if(gap!==null){minimum=Math.min(minimum,gap);return;}
      }
      if(separated){
        const gap=separated(patch,bodies[otherIndex],0,0,0,toleranceMm);
        if(gap!==null){minimum=Math.min(minimum,gap);return;}
      }
      if(patch.openClippedPatch&&(!reusedShape||preparedPatchShapes.has(patch.triangles))){
        const box=bodyBounds(patch),limit=toleranceMm; // Both clipped bodies belong to the workpiece.
        const gap2=boxGap2(box.min,box.max,projectJointBounds?boxes[otherIndex]:collider(otherIndex).bvh.bounds);
        if(gap2>limit*limit){minimum=Math.min(minimum,Math.sqrt(gap2));return;}
      }
      checks.push([meshColliderForScene(patch),collider(otherIndex)]);
      if(reusedShape)preparedPatchShapes.add(patch.triangles);
    };
    if(joint){
      const clipped=jointFarPatches({model,scene,panelBody:bodies[pa],bandBody:bodies[ba],joint,clearanceMm,toleranceMm,indexedPatches:true,certifyBounds:projectJointBounds});
      if(clipped.certifiedFar)minimum=Math.min(minimum,toleranceMm);
      jointDiagnostic=clipped.diagnostics;
      jointExceptions.push({...clipped.diagnostics,originalMeshesClosed:[closed(pa),closed(ba)]});
      queuePatch(clipped.panelFar,ba,true,clipped.separationAxes);
      queuePatch(clipped.bandFar,pa,false,clipped.separationAxes);
    }else{
      const corner=bodies[a].kind==='band'&&bodies[b].kind==='band'?
        bandCornerFarPatches({model,scene,bodyA:bodies[a],bodyB:bodies[b],clearanceMm,toleranceMm,indexedPatches:true,certifyBounds:projectJointBounds}):
        bodies[a].kind==='panel'&&bodies[b].kind==='panel'&&closed(a)&&closed(b)?
          panelReliefFarPatches({model,scene,bodyA:bodies[a],bodyB:bodies[b],toleranceMm,indexedPatches:true}):null;
      if(corner){
        if(corner.certifiedFar)minimum=Math.min(minimum,toleranceMm);
        jointDiagnostic=corner.diagnostics;
        jointExceptions.push({...corner.diagnostics,originalMeshesClosed:[closed(a),closed(b)]});
        queuePatch(corner.aFar,b);
        queuePatch(corner.bFar,a);
      }else checks.push([collider(a),collider(b)]);
    }
    // Bodies of one part can sit closer than the tool clearance already in the
    // flat blank (corner reliefs, split-hinge slots); only real contact counts there.
    const pairClearance=selfPair?0:clearanceMm;
    for(const [one,two]of checks){
      const result=checkPair(one,two,{clearanceMm:pairClearance,toleranceMm,queryMode:'clearance'});
      minimum=Math.min(minimum,result.distanceMm);
      if(result.status==='INTERFERENCE'){hit=jointDiagnostic?{...result,jointException:jointDiagnostic}:result;break;}
    }
    if(!hit)continue;
    if(hit.status==='INTERFERENCE'){hits.push({...hit,bodyKinds:[bodies[a].kind,bodies[b].kind],
      category:isPart(bodies[a])&&isPart(bodies[b])?'PIECE_SELF':isPart(bodies[a])||isPart(bodies[b])?'PIECE_TOOL':'TOOL_TOOL'});if(firstOnly)break;}
  }
  return{status:hits.length?'INTERFERENCE':'SAMPLE_CLEAR',hits,intendedJointContactCount,jointExceptions,broadphaseSkipped,
    distanceMm:hits.length?0:minimum,distanceIsExact:false,distanceBound:'LOWER',
    selfCollisionScope:'All panel and band pairs; exact local clipping at panel/band tangencies and verified shared band corners. No whole-body omission.',
    closedSolidContainmentAvailableForJointTargets:jointExceptions.every(e=>e.originalMeshesClosed.every(Boolean))};
}
