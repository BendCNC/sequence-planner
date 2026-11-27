import {meshCollider,validatePackedMesh,checkPair,boxDistance2,cappedPairDistance} from './collision.mjs';
import {planSegmentMove} from './segment-planner.mjs';
import {meshSeparationQuery} from './mesh-separation.mjs';

const shifted=(p,dx,dz)=>[p[0]+dx,p[1],p[2]+dz];
const shiftedBounds=(b,dx,dz)=>({min:shifted(b.min,dx,dz),max:shifted(b.max,dx,dz)});
const asCollider=body=>body?.bvh?body:meshCollider(body);

// Preserve topology/BVH partitioning under translation: no re-triangulation or rebuild.
function translatedCollider(source,dx,dz){
  if(dx===0&&dz===0)return source;
  const positions=source.positions.map(p=>shifted(p,dx,dz));
  const items=source.items.map(t=>({...t,p:source.triangles[t.index].map(i=>positions[i]),
    bounds:shiftedBounds(t.bounds,dx,dz),center:shifted(t.center,dx,dz)}));
  function node(n){return{...n,bounds:shiftedBounds(n.bounds,dx,dz),
    ...(n.items?{items:n.items.map(t=>items[t.index])}:{left:node(n.left),right:node(n.right)})};}
  return{...source,positions,items,bounds:shiftedBounds(source.bounds,dx,dz),bvh:node(source.bvh),
    components:source.components.map(c=>({...c,representative:shifted(c.representative,dx,dz)}))};
}
function classify(result,toleranceMm){
  if(result.reason==='CONTAINMENT')return'SOLID_CONTAINMENT';
  const witness=result.witnessedSurfaceDistanceMm??result.surfaceDistanceMm;
  return Number.isFinite(witness)&&witness<=toleranceMm?'SURFACE_CONTACT_OR_INTERSECTION':'CLEARANCE_MARGIN_WITNESS';
}
function worldWitness(result,pose,toleranceMm){
  return{...result,point:result.point?shifted(result.point,pose.x,pose.z):null,
    closestPoints:result.closestPoints?.map(p=>shifted(p,pose.x,pose.z))??null,
    classification:classify(result,toleranceMm),
    classificationScope:'A positive-distance witness proves a margin violation, but does not exclude another intersection in the same pair.'};
}
function orderedSamples(samples,lift){
  const position=s=>Math.abs(s.pose.x)<1e-8?s.pose.z:lift+Math.abs(s.pose.x);
  const seen=new Set();
  return samples.slice().sort((a,b)=>position(a)-position(b)).filter(s=>{
    const key=`${s.pose.x.toPrecision(14)}|${s.pose.z.toPrecision(14)}`;
    if(seen.has(key))return false;seen.add(key);return true;
  });
}

/**
 * Rigid access for one fixed formed state. Front is X<0; Y and orientation never change.
 * Try lifting +Z at the station, then withdrawing -X. Insertion reverses that SAME state.
 * Call separately for pre-fold insertion and post-fold extraction. Obstacles must include
 * tools/fingers in their intended withdrawn state; no assembly contacts are silently ignored.
 * maxWorldZ/maxLiftMm are optional caller constraints, not assumed machine measurements.
 */
export function checkFrontAccess({partBodies,obstacles=[],clearanceMm=.15,stepMm=2,minStepMm=.25,routeOnly=false,
  frontX,frontMarginMm=10,liftCandidatesMm=[0,2,5,10,20,40],maxLiftMm=Infinity,maxWorldZ=Infinity,
  maxHorizontalTravelMm=Infinity,maxSamples=2500,toleranceMm=1e-7,poseAllowed}={}){
  if(!Array.isArray(partBodies)||partBodies.length===0||!Array.isArray(obstacles))throw new TypeError('partBodies and obstacles must be arrays; partBodies cannot be empty.');
  for(const [name,value]of Object.entries({clearanceMm,frontMarginMm,toleranceMm}))
    if(!Number.isFinite(value)||value<0)throw new RangeError(`Invalid ${name}`);
  if(frontX!==undefined&&!Number.isFinite(frontX))throw new RangeError('frontX must be finite');
  if(!Number.isFinite(stepMm)||!Number.isFinite(minStepMm)||!(stepMm>0)||!(minStepMm>0)||minStepMm>stepMm)throw new RangeError('Invalid sampling step');
  if(!Number.isInteger(maxSamples)||maxSamples<2)throw new RangeError('maxSamples must be at least two');
  for(const value of[maxLiftMm,maxWorldZ,maxHorizontalTravelMm])if(Number.isNaN(value)||value===-Infinity)throw new RangeError('Invalid supplied movement limit');
  if(maxLiftMm<0||maxHorizontalTravelMm<0)throw new RangeError('Movement limits cannot be negative');
  if(poseAllowed!==undefined&&typeof poseAllowed!=='function')throw new TypeError('poseAllowed must be a synchronous predicate');
  // Discovery needs only the coordinate extrema until a swept proof is
  // inconclusive. Its numerical fallback uses the original complete colliders.
  const boundsOf=body=>{
    if(body.bvh)return body.bounds;
    if(!routeOnly)return asCollider(body).bounds;
    const collider=validatePackedMesh(body);if(collider)return collider.bounds;
    const min=[Infinity,Infinity,Infinity],max=[-Infinity,-Infinity,-Infinity];
    for(let i=0;i<body.positions.length;i+=3)for(let k=0;k<3;k++){
      min[k]=Math.min(min[k],body.positions[i+k]);max[k]=Math.max(max[k],body.positions[i+k]);
    }
    return {min,max};
  };
  let parts=partBodies.map(body=>routeOnly?{bounds:boundsOf(body)}:asCollider(body));
  let fixed=obstacles.map(body=>routeOnly?{bounds:boundsOf(body)}:asCollider(body));
  const minPartX=Math.min(...parts.map(p=>p.bounds.min[0])),maxPartX=Math.max(...parts.map(p=>p.bounds.max[0]));
  const maxPartZ=Math.max(...parts.map(p=>p.bounds.max[2]));
  const obstacleFrontX=fixed.length?Math.min(...fixed.map(o=>o.bounds.min[0])):minPartX;
  const automaticPlane=obstacleFrontX-frontMarginMm;
  const targetFrontPlaneX=Math.min(frontX??automaticPlane,automaticPlane);
  const withdrawX=Math.min(0,targetFrontPlaneX-maxPartX-Math.max(1e-6,toleranceMm*2));
  const limits={method:'TRANSLATION_SAMPLING_WITH_CONSERVATIVE_DISTANCE_BOUNDS',continuousCollisionDetection:false,
    frontSide:'X<0',translationAxes:['X','Z'],fixedStationY:true,fixedOrientation:true,stepMm,minStepMm,
    maxSamples,clearanceMm,maxLiftMm:Number.isFinite(maxLiftMm)?maxLiftMm:null,
    maxWorldZ:Number.isFinite(maxWorldZ)?maxWorldZ:null,ceilingClearanceMm:clearanceMm,maxHorizontalTravelMm:Number.isFinite(maxHorizontalTravelMm)?maxHorizontalTravelMm:null,
    scope:'One fixed formed state and supplied static obstacle meshes. No forces, operator envelope, frame geometry beyond supplied obstacles, gauge motion or calibrated axis limits are inferred.'};
  const base={targetFrontPlaneX,withdrawX,obstacleFrontX,frontMarginMm,limits,
    startBounds:{minX:minPartX,maxX:maxPartX,maxZ:maxPartZ},obstacleCount:fixed.length,partBodyCount:parts.length};
  const fail=(reason,extra={})=>({...base,status:'BLOCKED',reason,keyframes:[],insertionKeyframes:[],samples:[],attempts:[],...extra});
  if(-withdrawX>maxHorizontalTravelMm)return fail('FRONT_TRAVEL_LIMIT');
  const lifts=[...new Set(liftCandidatesMm.map(Number))];
  if(lifts.some(d=>!Number.isFinite(d)||d<0))throw new RangeError('Lift candidates must be finite and nonnegative');
  const cache=new Map(),attempts=[];let evaluations=0;
  const separated=routeOnly?meshSeparationQuery():null;
  let originals;
  function prepareColliders(){
    if(originals)return;
    if(routeOnly){parts=partBodies.map(asCollider);fixed=obstacles.map(asCollider);}
    originals=new Map([...parts.map((p,i)=>[p,partBodies[i]]),...fixed.map((p,i)=>[p,obstacles[i]])]);
  }
  function evaluate(pose){
    prepareColliders();
    const key=`${pose.x.toPrecision(14)}|${pose.z.toPrecision(14)}`;
    if(cache.has(key))return cache.get(key);
    if(evaluations>=maxSamples)throw new Error('FRONT_SAMPLE_BUDGET_EXHAUSTED');
    evaluations++;
    if(pose.z>maxLiftMm+toleranceMm||maxPartZ+pose.z+clearanceMm>=maxWorldZ-toleranceMm){
      const result={status:'INTERFERENCE',distanceMm:0,reason:'SUPPLIED_LIFT_OR_CEILING_LIMIT'};cache.set(key,result);return result;
    }
    if(poseAllowed&&poseAllowed({...pose,y:0},{maxZ:maxPartZ+pose.z,minX:minPartX+pose.x,maxX:maxPartX+pose.x})!==true){
      const result={status:'INTERFERENCE',distanceMm:0,reason:'CALLER_POSE_CONSTRAINT'};cache.set(key,result);return result;
    }
    // T(part) vs obstacle equals part vs T^-1(obstacle). Tool meshes are usually
    // much smaller than the part. Keep the expensive part BVHs cached throughout.
    const dx=-pose.x,dz=-pose.z,unchanged=dx===0&&dz===0,threshold=clearanceMm+toleranceMm;
    const relative=fixed.map(o=>({source:o,collider:unchanged?o:null,
      bounds:unchanged?o.bounds:shiftedBounds(o.bounds,dx,dz),
      rootBounds:unchanged?o.bvh.bounds:shiftedBounds(o.bvh.bounds,dx,dz)}));
    let minDistance=Number.isFinite(maxWorldZ)?Math.max(0,maxWorldZ-(maxPartZ+pose.z)):Infinity;
    for(const p of parts)for(const entry of relative){
      // These are precisely the boxes checkPair would query after translation.
      // Separate vertex boxes also exclude containment, even with unreferenced vertices.
      const bound=boxDistance2(p.bvh.bounds,entry.rootBounds);
      if(bound>threshold*threshold&&p.bounds.min.some((min,k)=>min>entry.bounds.max[k]||entry.bounds.min[k]>p.bounds.max[k])){
        minDistance=Math.min(minDistance,Math.sqrt(bound));continue;
      }
      if(separated){
        const gap=separated(originals.get(p),originals.get(entry.source),pose.x,0,pose.z,threshold);
        if(gap!==null){minDistance=Math.min(minDistance,gap);continue;}
      }
      entry.collider??=translatedCollider(entry.source,dx,dz);
      const bounded=cappedPairDistance(p,entry.collider,clearanceMm,toleranceMm,minDistance);
      if(bounded!==null){minDistance=bounded;continue;}
      const result=checkPair(p,entry.collider,{clearanceMm,toleranceMm,queryMode:'clearance'});
      minDistance=Math.min(minDistance,result.distanceMm);
      if(result.status==='INTERFERENCE'){
        const answer={status:'INTERFERENCE',distanceMm:0,collision:worldWitness(result,pose,toleranceMm)};
        cache.set(key,answer);return answer;
      }
    }
    const answer={status:'CLEAR',distanceMm:minDistance,distanceIsExact:false,distanceBound:'LOWER'};
    cache.set(key,answer);return answer;
  }
  for(const lift of lifts){
    if(lift>maxLiftMm||maxPartZ+lift>maxWorldZ){attempts.push({liftMm:lift,status:'BLOCKED',reason:'SUPPLIED_LIFT_OR_CEILING_LIMIT'});continue;}
    if(separated&&!poseAllowed&&maxWorldZ===Infinity&&withdrawX< -1e-10){
      const keys=[{phase:'AT_STATION',pose:{x:0,z:0,y:0}}];
      if(lift>1e-10)keys.push({phase:'LIFT_AT_STATION',pose:{x:0,z:lift,y:0}});
      keys.push({phase:'WITHDRAW_FRONT',pose:{x:withdrawX,z:lift,y:0}});
      const clear=keys.slice(1).every(({pose},i)=>partBodies.every(part=>obstacles.every(obstacle=>
        separated(part,obstacle,keys[i].pose.x,0,keys[i].pose.z,clearanceMm+toleranceMm,
          pose.x-keys[i].pose.x,0,pose.z-keys[i].pose.z)!==null)));
      // The caller validates the full path again with moving tooling, support
      // and the selected opening. It only needs the discovery keyframes here.
      if(clear)return {...base,status:'SAMPLE_CLEAR',liftMm:lift,keyframes:keys,attempts,evaluations,
        limits:{...limits,sweptSeparation:true}};
    }
    let result;
    try{
      result=planSegmentMove({from:{x:0,z:0},to:{x:withdrawX,z:lift},evaluatePose:evaluate,retractAxis:'z',
        retractDistancesMm:[0],maxStepMm:stepMm,minStepMm,maxSamples:Math.max(2,maxSamples-evaluations),
        clearanceMm,distanceIsConservative:!poseAllowed});
    }catch(error){if(error.message!=='FRONT_SAMPLE_BUDGET_EXHAUSTED')throw error;
      return fail('FRONT_SAMPLE_BUDGET_EXHAUSTED',{attempts,evaluations});}
    const samples=orderedSamples(result.samples??[],lift);
    // The generic planner may discard route samples after all candidates fail;
    // its attempts retain the actual blocking witness and must not be lost here.
    const blockedSample=samples.find(s=>s.result.status==='INTERFERENCE')??result.attempts?.map(a=>a.blockedSample).find(Boolean);
    attempts.push({liftMm:lift,status:result.status,reason:result.reason??null,sampleCount:samples.length,
      blockedSample:blockedSample??null,maxObservedGapMm:result.limits?.maxObservedGapMm??null});
    if(result.status==='SAMPLE_CLEAR'){
      const keyframes=result.keyframes.map((k,i)=>({phase:i===0?'AT_STATION':Math.abs(k.pose.x)<1e-8?'LIFT_AT_STATION':'WITHDRAW_FRONT',pose:{...k.pose,y:0}}));
      const insertionKeyframes=keyframes.slice().reverse().map((k,i)=>({...k,phase:i===0?'OUTSIDE_FRONT':Math.abs(k.pose.x)<1e-8&&k.pose.z>0?'INSERT_FROM_FRONT':'LOWER_TO_STATION'}));
      return{...base,status:'SAMPLE_CLEAR',liftMm:lift,keyframes,insertionKeyframes,samples,attempts,evaluations,
        endMaxX:maxPartX+withdrawX,wholePartOutsideObstacleFront:maxPartX+withdrawX<targetFrontPlaneX,
        pathLengthMm:result.totalTravelMm,limits:{...limits,maxObservedGapMm:result.limits.maxObservedGapMm,
          boundedIntervals:result.limits.boundedIntervals},
        insertionScope:'Reverse path is valid for this same fixed shape with the same static obstacles; it is not insertion of a different pre-fold state.'};
    }
    if(result.reason==='SEARCH_BUDGET_EXHAUSTED')return fail('FRONT_SAMPLE_BUDGET_EXHAUSTED',{attempts,samples,evaluations});
    if(result.reason==='START_INTERFERENCE')return fail('START_INTERFERENCE',{attempts,samples,evaluations,collision:blockedSample?.result.collision??null});
  }
  return fail('NO_FRONT_ROUTE_IN_CANDIDATES',{attempts,evaluations,
    searchScope:'Only the listed lift-then-negative-X routes were tested; BLOCKED does not prove that every possible physical extraction is impossible.'});
}
