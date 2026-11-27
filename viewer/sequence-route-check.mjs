import {shadow,certifiedSeparation} from './lite-separation.mjs';
import {meshSeparationQuery} from './mesh-separation.mjs';
import {positionBounds} from './tool-geometry.mjs';
import {meshCollider,checkPair,validatePackedMesh} from './collision.mjs';
import {transformFrontTiltPoint} from './sequence-rigid-pose.mjs';
import {buildOperatorSupport,evaluateOperatorSupport} from './sequence-operator-support.mjs';
// With no rotation the last operation in each transformed X/Z coordinate is
// addition of the pose translation. Floating addition is monotone, so extrema
// can be selected before that final addition. Keep the original pivot math
// (including signed zero). Only referenced support vertices need world values.
function prepareTranslationSamples(meshes,prepared,pivot){
  const skins=new Map(prepared.majorBranch.bodies.map(b=>[b.bodyId,b]));
  const posedPointsById=new Map();
  const rows=meshes.map(mesh=>{
    const minimum=[Infinity,Infinity,Infinity],maximum=[-Infinity,-Infinity,-Infinity];
    const skin=skins.get(mesh.id),sources=[],posed=[],used=new Set();
    for(const face of skin?.triangles??[])for(const i of face.indices)used.add(i);
    for(const [i,point] of mesh.positions.entries()){
      const px=point[0]-pivot[0],pz=point[2]-pivot[2];
      const x=pivot[0]+px+0*pz,y=point[1],z=pivot[2]-0*px+pz;
      minimum[0]=Math.min(minimum[0],x);maximum[0]=Math.max(maximum[0],x);
      minimum[1]=Math.min(minimum[1],y);maximum[1]=Math.max(maximum[1],y);
      minimum[2]=Math.min(minimum[2],z);maximum[2]=Math.max(maximum[2],z);
      if(used.has(i)){
        const target=[x,y,z];posed[i]=target;sources.push({x,z,target});
      }
    }
    if(skin)posedPointsById.set(mesh.id,posed);
    return{minimum,maximum,sources};
  });
  return pose=>{
    const bounds=rows.map(({minimum,maximum,sources})=>{
      for(const {x,z,target} of sources){target[0]=x+pose.x;target[2]=z+pose.z;}
      return{min:[minimum[0]+pose.x,minimum[1],minimum[2]+pose.z],
        max:[maximum[0]+pose.x,maximum[1],maximum[2]+pose.z]};
    });
    return{bounds,posedPointsById};
  };
}
/** Recheck proposed rigid keyframes from scratch against current meshes and operator-side rule. */
export function checkRecordedOperatorRoute({model,bendId,scene,keyframes,pivot,maxWorldZ=Infinity,clearanceMm=.15,frontMarginMm=10}){
  if(!Array.isArray(keyframes)||keyframes.length<2)throw new Error('A route requires at least two keyframes');
  const first=keyframes[0].pose;
  if(Math.abs(first.x)+Math.abs(first.z)+Math.abs(first.tiltDeg)>1e-9)throw new Error('Route must start at its working pose');
  for(const f of keyframes)if([f.pose.x,f.pose.z,f.pose.tiltDeg,...pivot].some(v=>!Number.isFinite(v))||Math.abs(f.pose.y??0)>1e-9||f.pose.x>1e-9||f.pose.z< -1e-9)throw new Error('Invalid fixed-front pose');
  const part=scene.bodies.filter(b=>['panel','band'].includes(b.kind)),obstacles=scene.bodies.filter(b=>!['panel','band'].includes(b.kind));
  const fixedParts=part.map(body=>{
    const collider=validatePackedMesh(body);
    const xyz=body.positions,positions=[];
    for(let i=0;i<xyz.length;i+=3)positions.push([xyz[i],xyz[i+1],xyz[i+2]]);
    return {id:body.id,body,positions,collider};
  }),points=fixedParts.flatMap(p=>p.positions),prepared=buildOperatorSupport(model,bendId);
  const radius=Math.max(...points.map(p=>Math.hypot(p[0]-pivot[0],p[2]-pivot[2])));
  const obstacleBounds=obstacles.map(b=>positionBounds(b.positions));
  const obstacleFrontX=Math.min(...obstacleBounds.map(b=>b.min[0])),targetFrontPlaneX=obstacleFrontX-frontMarginMm;
  const samples=[];let maxPointStep=0;
  const translate=keyframes.every(f=>Object.is(f.pose.tiltDeg,0))
    ?prepareTranslationSamples(fixedParts,prepared,pivot):null;
  const pp=[],op=[],maxPivot=Math.max(Math.abs(pivot[0]),Math.abs(pivot[2]));
  const query=meshSeparationQuery();
  let clearLeg=new Set();
  function sample(pose,phase){
    const translated=translate?.(pose),posedPointsById=translated?.posedPointsById??new Map();
    const bounds=translated?.bounds??fixedParts.map((mesh,index)=>{
      const min=[Infinity,Infinity,Infinity],max=[-Infinity,-Infinity,-Infinity];
      const moved=mesh.positions.map(point=>{
        const v=transformFrontTiltPoint(point,pose,pivot);
        for(let k=0;k<3;k++){min[k]=Math.min(min[k],v[k]);max[k]=Math.max(max[k],v[k]);}
        return v;
      });
      posedPointsById.set(part[index].id,moved);
      return {min,max};
    });
    let maxX=-Infinity,maxZ=-Infinity;
    for(const {max} of bounds){maxX=Math.max(maxX,max[0]);maxZ=Math.max(maxZ,max[2]);}
    const support=evaluateOperatorSupport(prepared,scene,{posedPointsById});
    let result=maxZ+clearanceMm>=maxWorldZ-1e-7?{status:'INTERFERENCE',reason:'CONSERVATIVE_CEILING_LIMIT'}:
      !support.passes?{status:'INTERFERENCE',reason:'OPERATOR_MAIN_BRANCH_FRONT_CONSTRAINT'}:null;
    if(!result)outer:for(const [i,body] of obstacles.entries()){
      const box=obstacleBounds[i];
      const candidates=fixedParts.filter((mesh,index)=>{
        const other=bounds[index];
        if(clearLeg.has(index*obstacles.length+i))return false;
        // Keep numerical ties (also after inverse rotation) for the exact query.
        const overlaps=!other.min.some((v,k)=>{
          const scale=Math.max(1,Math.abs(v),Math.abs(other.max[k]),Math.abs(box.min[k]),Math.abs(box.max[k]),Math.abs(pivot[0]),Math.abs(pivot[2]),Math.abs(pose.x),Math.abs(pose.z));
          const margin=clearanceMm+1e-7+256*Number.EPSILON*scale;
          return v-box.max[k]>margin || box.min[k]-other.max[k]>margin;
        });
        if(!overlaps||!translate)return overlaps;
        pp[index]??=shadow(mesh);op[i]??=shadow(body);
        return !certifiedSeparation(pp[index],op[i],pose.x,pose.z,clearanceMm,maxPivot);
      });
      if(!candidates.length)continue;
      const positions=new Float64Array(body.positions.length);for(let i=0;i<positions.length;i+=3)positions.set(transformFrontTiltPoint(body.positions.slice(i,i+3),pose,pivot,true),i);
      const obstacle=meshCollider({...body,positions});
      for(const p of candidates){p.collider??=meshCollider(p.body);const hit=checkPair(p.collider,obstacle,{clearanceMm,queryMode:'clearance'});
        if(hit.status==='INTERFERENCE'){result={status:'INTERFERENCE',reason:'MESH_PAIR_WITNESS',collision:{...hit,
          point:hit.point?transformFrontTiltPoint(hit.point,pose,pivot):null,
          closestPoints:hit.closestPoints?.map(p=>transformFrontTiltPoint(p,pose,pivot))??null}};break outer;}
      }
    }
    const out={pose:{...pose,y:0},phase,result:result??{status:'CLEAR'},operatorSupport:support,maxX};samples.push(out);return out;
  }
  let blocked=sample(first,'AT_STATION');if(blocked.result.status==='CLEAR')blocked=null;
  for(let i=1;i<keyframes.length&&!blocked;i++){
    const a=keyframes[i-1].pose,b=keyframes[i].pose,translation=Math.hypot(b.x-a.x,b.z-a.z),degrees=Math.abs(b.tiltDeg-a.tiltDeg),bound=translation+radius*degrees*Math.PI/180;
    if(translate){
      clearLeg=new Set();
      for(let p=0;p<part.length;p++)for(let f=0;f<obstacles.length;f++)
        if(query(part[p],obstacles[f],a.x,0,a.z,clearanceMm+1e-7+1e-10*maxPivot,b.x-a.x,0,b.z-a.z)!==null)
          clearLeg.add(p*obstacles.length+f);
    }
    const count=Math.max(1,Math.ceil(bound/2),Math.ceil(degrees/2));maxPointStep=Math.max(maxPointStep,bound/count);
    for(let j=1;j<=count;j++){
      const q=j/count,pose={x:a.x+(b.x-a.x)*q,y:0,z:a.z+(b.z-a.z)*q,tiltDeg:a.tiltDeg+(b.tiltDeg-a.tiltDeg)*q};
      const s=sample(pose,keyframes[i].phase);if(s.result.status!=='CLEAR'){blocked=s;break;}
    }
  }
  const outside=!blocked&&samples.at(-1).maxX<targetFrontPlaneX;
  return{status:outside?'SAMPLE_CLEAR':'BLOCKED',reason:blocked?'RECORDED_ROUTE_INTERFERENCE':outside?null:'ROUTE_NOT_FULLY_OUTSIDE',
    pivot,keyframes:outside?keyframes:[],insertionKeyframes:outside?keyframes.slice().reverse().map(f=>({...f,phase:'REVERSE_'+f.phase})):[],
    samples,blockedSample:blocked,evaluations:samples.length,wholePartOutsideObstacleFront:outside,targetFrontPlaneX,endMaxX:samples.at(-1).maxX,
    limits:{method:'INDEPENDENT_RECORDED_RIGID_ROUTE_RECHECK',continuousCollisionDetection:false,fixedStationY:true,clearanceMm,
      maxWorldZ,maxObservedPointDisplacementBoundMm:maxPointStep,translationStepMm:2,rotationStepDeg:2,pointStepBoundMm:2}};
}
