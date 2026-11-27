/** Local exception for the artificial mating face of a finite sheet joint.
 * Clip triangle geometry, never discard a whole triangle merely touching a seam.
 * The returned patches are OPEN; each must be queried against the ORIGINAL full
 * opposite body, which retains closed-solid containment when its mesh is closed.
 */
import {multiply,transform,bandPoint} from './solver.mjs';

const caches=new WeakMap();
const cornerCaches=new WeakMap();
const dot=(a,b)=>a.reduce((s,x,i)=>s+x*b[i],0);
const sub=(a,b)=>a.map((x,i)=>x-b[i]);
const direction=(m,p)=>[0,1,2].map(i=>m[4*i]*p[0]+m[4*i+1]*p[1]+m[4*i+2]*p[2]);
const point=(positions,i)=>[positions[3*i],positions[3*i+1],positions[3*i+2]];
const interpolate=(a,b,t)=>[a[0]+(b[0]-a[0])*t,a[1]+(b[1]-a[1])*t,a[2]+(b[2]-a[2])*t];
const lerp=(a,b,t)=>({p:interpolate(a.p,b.p,t),q:interpolate(a.q,b.q,t)});
// The private collision solver can share exactly coincident positions of
// the flat clipped panel. Face order, winding and first occurrence stay fixed.
const negativeZero=Symbol();
const coordinateKey=x=>Object.is(x,-0)?negativeZero:x;
function compactFlatPatch(patch){
  if(!patch)return null;
  const coordinates=patch.positions,positions=[],seen=new Map();
  const remap=new Array(coordinates.length/3);
  for(let i=0;i<coordinates.length;i+=3){
    const x=coordinates[i],y=coordinates[i+1],z=coordinates[i+2];
    const kx=coordinateKey(x),ky=coordinateKey(y),kz=coordinateKey(z);
    let ys=seen.get(kx);if(!ys){ys=new Map();seen.set(kx,ys);}
    let zs=ys.get(ky);if(!zs){zs=new Map();ys.set(ky,zs);}
    let index=zs.get(kz);
    if(index===undefined){index=positions.length/3;zs.set(kz,index);positions.push(x,y,z);}
    remap[i/3]=index;
  }
  return {...patch,positions:Float64Array.from(positions),triangles:patch.triangles.map(i=>remap[i])};
}
function jointProjector(origin,basis){
  const [a,b,c]=basis;
  return p=>{
    const x=p[0]-origin[0],y=p[1]-origin[1],z=p[2]-origin[2];
    return [x*a[0]+y*a[1]+z*a[2],x*b[0]+y*b[1]+z*b[2],x*c[0]+y*c[1]+z*c[2]];
  };
}

// A point in a closed mesh also lies within every vertex projection bound.
// If the common part of the two expanded bounds lies strictly inside the
// exception on all three axes, no far patch can hit the opposite solid.
function confinedContact(bodyA,bodyB,origin,basis,min,max,tolerance){
  for(let k=0;k<3;k++){
    const [x,y,z]=basis[k],offset=x*origin[0]+y*origin[1]+z*origin[2];
    const length=Math.hypot(x,y,z);
    if(!(length>.99&&length<1.01))return false;
    let lo=-Infinity,hi=Infinity,scale=Math.abs(offset);
    for(const body of [bodyA,bodyB]){
      const coords=body.positions;
      let l=Infinity,h=-Infinity;
      for(let i=0;i<coords.length;i+=3){
        const a=x*coords[i],b=y*coords[i+1],c=z*coords[i+2],p=a+b+c-offset;
        l=Math.min(l,p);h=Math.max(h,p);scale=Math.max(scale,Math.abs(a)+Math.abs(b)+Math.abs(c));
      }
      lo=Math.max(lo,l);hi=Math.min(hi,h);
    }
    const pad=tolerance*length+1e-9*(1+scale);
    if(!(lo>min[k]+pad&&hi<max[k]-pad))return false;
  }
  return true;
}

// The panel patch is clipped in flat coordinates, then placed in the machine.
// Bound the round-trip error before using a world-space exception for both
// patches. Malformed/non-rigid frames continue through the clipping solver.
function flatPatchError(m,src,world,flatOrigin){
  if(src.length!==world.length)return Infinity;
  let scale=1,error=0,ortho=0;
  for(let i=0;i<3;i++)for(let j=0;j<3;j++){
    ortho=Math.max(ortho,Math.abs(m[i]*m[j]+m[4+i]*m[4+j]+m[8+i]*m[8+j]-(i===j?1:0)));
  }
  for(let i=0;i<src.length;i+=3){
    const x=src[i],y=src[i+1],z=src[i+2];
    scale=Math.max(scale,Math.abs(x),Math.abs(y),Math.abs(z));
    for(let k=0;k<3;k++){
      const j=4*k,p=m[j]*x+m[j+1]*y+m[j+2]*z+m[j+3];
      error=Math.max(error,Math.abs(p-world[i+k]));
      scale=Math.max(scale,Math.abs(p),Math.abs(m[j+3]),Math.abs(flatOrigin[k]));
    }
  }
  return 32*(error+ortho*scale)+1e-9*scale;
}

// The projection of a solid is covered by the projections of its boundary
// triangles (including concave solids and holes). Tangency relief boxes have
// a common thickness interval. Partition their footprint and look for any
// cell outside the exception that both complete solids could occupy.
function jointFootprint(positions,triangles,toLocal){
  const points=[],min=[Infinity,Infinity,Infinity],max=[-Infinity,-Infinity,-Infinity];
  for(let i=0;i<positions.length;i+=3){
    const q=toLocal([positions[i],positions[i+1],positions[i+2]]);points.push(q);
    for(let k=0;k<3;k++){min[k]=Math.min(min[k],q[k]);max[k]=Math.max(max[k],q[k]);}
  }
  return {points,triangles,min,max};
}
function footprintDepth(body,l,r,b,t,pad){
  let lo=Infinity,hi=-Infinity;
  const {points,triangles}=body;
  l-=pad;r+=pad;b-=pad;t+=pad;
  for(let i=0;i<triangles.length;i+=3){
    const a=points[triangles[i]],e=points[triangles[i+1]],f=points[triangles[i+2]];
    if(Math.max(a[0],e[0],f[0])<l||Math.min(a[0],e[0],f[0])>r||
       Math.max(a[1],e[1],f[1])<b||Math.min(a[1],e[1],f[1])>t)continue;
    let disjoint=false;
    for(const [one,two,opposite] of [[a,e,f],[e,f,a],[f,a,e]]){
      const dx=two[0]-one[0],dy=two[1]-one[1];
      const sign=dx*(opposite[1]-one[1])-dy*(opposite[0]-one[0]);
      const error=pad*(1+Math.abs(dx)+Math.abs(dy));
      if(Math.abs(sign)<=error)continue;
      const nx=sign>0?-dy:dy,ny=sign>0?dx:-dx;
      if(nx*((nx>=0?r:l)-one[0])+ny*((ny>=0?t:b)-one[1])< -error){disjoint=true;break;}
    }
    if(!disjoint){
      lo=Math.min(lo,a[2],e[2],f[2]);hi=Math.max(hi,a[2],e[2],f[2]);
    }
  }
  return [lo,hi];
}
function coveredFootprint(panel,band,boxes,pad){
  const scale=Math.max(1,...panel.min.map(Math.abs),...panel.max.map(Math.abs),...band.min.map(Math.abs),...band.max.map(Math.abs));
  if(!Number.isFinite(scale)||!Number.isFinite(pad))return false;
  pad+=1e-9*(1+scale);
  const z0=Math.max(panel.min[2],band.min[2])-pad,z1=Math.min(panel.max[2],band.max[2])+pad;
  if(z0>z1)return true;
  const usable=boxes.filter(([lo,hi])=>z0>lo[2]+pad&&z1<hi[2]-pad);
  if(!usable.length||usable.length>16)return false;
  const cuts=[0,1].map(k=>{
    const lo=Math.max(panel.min[k],band.min[k])-pad,hi=Math.min(panel.max[k],band.max[k])+pad;
    if(lo>hi)return [];
    return [...new Set([lo,hi,...usable.flatMap(([a,b])=>[a[k]+pad,b[k]-pad]).filter(x=>x>lo&&x<hi)])].sort((a,b)=>a-b);
  });
  if(cuts.some(a=>!a.length))return true;
  if(cuts.some(a=>a.length<2))return false;
  for(let i=1;i<cuts[0].length;i++)for(let j=1;j<cuts[1].length;j++){
    const l=cuts[0][i-1],r=cuts[0][i],b=cuts[1][j-1],t=cuts[1][j];
    if(usable.some(([lo,hi])=>l>=lo[0]+pad&&r<=hi[0]-pad&&b>=lo[1]+pad&&t<=hi[1]-pad))continue;
    const p=footprintDepth(panel,l,r,b,t,pad);
    if(p[0]>p[1])continue;
    const q=footprintDepth(band,l,r,b,t,pad);
    if(Math.max(p[0],q[0])<=Math.min(p[1],q[1])+2*pad)return false;
  }
  return true;
}

function half(poly,axis,bound,sign){
  const out=[];
  for(let i=0;i<poly.length;i++){
    const a=poly[i],b=poly[(i+1)%poly.length],da=sign*(a.q[axis]-bound),db=sign*(b.q[axis]-bound);
    if(da<=0)out.push(a);
    if((da<0&&db>0)||(da>0&&db<0))out.push(lerp(a,b,da/(da-db)));
  }
  return out;
}

/** Surface outside a closed orthogonal box, with original winding retained. */
export function clipSurfaceOutsideJointBox(body,toLocal,min,max,indexed=false){
  const positions=[],triangles=[];
  const planes=[...min.map((v,i)=>[i,v,-1]),...max.map((v,i)=>[i,v,1])];
  // The local projection is the same for every triangle sharing an input vertex.
  const projected=new Array(body.positions.length/3);
  function vertex(index){
    let v=projected[index];
    if(v===undefined){
      const p=point(body.positions,index),q=toLocal(p);
      const outside=(q[0]<min[0])|((q[1]<min[1])<<1)|((q[2]<min[2])<<2)|
        ((q[0]>max[0])<<3)|((q[1]>max[1])<<4)|((q[2]>max[2])<<5);
      v={p,q,outside};projected[index]=v;
    }
    return v;
  }
  function outputIndex(v){
    if(v.outputIndex===undefined){
      v.outputIndex=positions.length/3;
      positions.push(v.p[0],v.p[1],v.p[2]);
    }
    return v.outputIndex;
  }
  function emitTriangle(va,vb,vc){
    const a=va.p,b=vb.p,c=vc.p;
    const ux=b[0]-a[0],uy=b[1]-a[1],uz=b[2]-a[2],vx=c[0]-a[0],vy=c[1]-a[1],vz=c[2]-a[2];
    const nx=uy*vz-uz*vy,ny=uz*vx-ux*vz,nz=ux*vy-uy*vx;
    if(Math.abs(nx)<=1e-14&&Math.abs(ny)<=1e-14&&Math.abs(nz)<=1e-14&&Math.hypot(nx,ny,nz)<=1e-14)return;
    if(indexed){
      triangles.push(outputIndex(va),outputIndex(vb),outputIndex(vc));
    }else{
      const first=positions.length/3;
      positions.push(a[0],a[1],a[2],b[0],b[1],b[2],c[0],c[1],c[2]);triangles.push(first,first+1,first+2);
    }
  }
  function emit(poly){for(let i=1;i<poly.length-1;i++)emitTriangle(poly[0],poly[i],poly[i+1]);}
  for(let i=0;i<body.triangles.length;i+=3){
    const a=vertex(body.triangles[i]),b=vertex(body.triangles[i+1]),c=vertex(body.triangles[i+2]);
    const outside=a.outside|b.outside|c.outside;
    if(!outside)continue;
    // Earlier planes see the entire triangle inside. If every vertex is past
    // the first active plane, that plane emits the original triangle verbatim.
    const firstBit=outside&-outside;
    if(indexed&&(a.outside&b.outside&c.outside&firstBit)){emitTriangle(a,b,c);continue;}
    let pending=[a,b,c];
    // The first plane with an outside vertex is the first plane that would
    // emit or split geometry. Later planes still clip in the original order.
    const first=31-Math.clz32(firstBit);
    for(let plane=first;plane<planes.length;plane++){
      const [axis,bound,sign]=planes[plane];
      if(!pending.length)break;
      let allInside=true,allOutside=true;
      for(const v of pending){
        const d=sign*(v.q[axis]-bound);
        if(!(d<=0))allInside=false;
        if(!(d>=0))allOutside=false;
      }
      if(allInside)continue;
      if(allOutside){emit(pending);break;}
      // Both halves meet on the same crossing. Only the inside polygon
      // continues to the next plane. Keep the winding and boundary vertices.
      const inside=[],outside=[];
      for(let j=0;j<pending.length;j++){
        const a=pending[j],b=pending[(j+1)%pending.length],da=sign*(a.q[axis]-bound),db=sign*(b.q[axis]-bound);
        if(da<=0)inside.push(a);
        if(da>=0)outside.push(a);
        if((da<0&&db>0)||(da>0&&db<0)){
          const crossing=lerp(a,b,da/(da-db));inside.push(crossing);outside.push(crossing);
        }
      }
      emit(outside);
      pending=inside;
    }
  }
  return positions.length?{...body,id:body.id,positions:new Float64Array(positions),triangles,openClippedPatch:true}:null;
}

/** Qualify a contact-only split-hinge end. The local cap must exist and all
 * nearby material surfaces must stay on the inward side of that cap plane.
 * This is a current-pose geometric check, not an exception based on IDs alone.
 */
function reliefCapSide(body,toLocal,axial,min,max,epsilon){
  const local=Array.from({length:body.positions.length/3},(_,i)=>toLocal(point(body.positions,i)));
  let capSign=null,capArea=0,minAxial=Infinity,maxAxial=-Infinity;
  for(let i=0;i<body.triangles.length;i+=3){
    const ids=Array.from(body.triangles.slice(i,i+3)),q=ids.map(k=>local[k]);
    if([0,1,2].some(k=>Math.min(...q.map(p=>p[k]))>max[k]||Math.max(...q.map(p=>p[k]))<min[k]))continue;
    let poly=ids.map((k,j)=>({p:point(body.positions,k),q:q[j]}));
    for(let k=0;k<3&&poly.length;k++){poly=half(poly,k,min[k],-1);if(poly.length)poly=half(poly,k,max[k],1);}
    if(!poly.length)continue;
    for(const v of poly){minAxial=Math.min(minAxial,v.q[0]);maxAxial=Math.max(maxAxial,v.q[0]);}
    if(q.some(v=>Math.abs(v[0]-axial)>epsilon))continue;
    const u=sub(q[1],q[0]),v=sub(q[2],q[0]);
    const n=[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]],length=Math.hypot(...n);
    if(length<=1e-14||Math.abs(n[0])/length<1-1e-8)continue;
    let area=0;for(let k=1;k<poly.length-1;k++){
      const a=sub(poly[k].q,poly[0].q),b=sub(poly[k+1].q,poly[0].q);area+=Math.abs(a[1]*b[2]-a[2]*b[1])/2;
    }
    if(area<=epsilon*epsilon)continue;
    const sign=Math.sign(n[0]);if(capSign!==null&&capSign!==sign)return null;
    capSign=sign;capArea+=area;
  }
  if(capSign===null||capArea<=epsilon*epsilon)return null;
  // The normal points outward: material may touch its plane, never cross it.
  const wrongSide=capSign>0?maxAxial-axial:axial-minAxial;
  if(wrongSide>epsilon)return null;
  return{outwardAxisSign:capSign,capAreaMm2:capArea,maximumWrongSideMm:Math.max(0,wrongSide)};
}

/** Contact of opposite caps at an explicitly declared split-hinge relief.
 * Only the tiny numerical band around a proven separating plane is clipped.
 * Both remaining open patches must be checked against the ORIGINAL complete
 * opposite body, preserving remote intersections and closed-solid containment.
 */
export function panelReliefFarPatches({model,scene,bodyA,bodyB,toleranceMm=1e-7,indexedPatches=false}){
  if(bodyA.kind!=='panel'||bodyB.kind!=='panel'||!scene.state?.panelMatrices||!scene.placement?.frame)return null;
  const joint=model.bends.find(j=>j.splitHingeSpans>1&&
    [String(j.parentPanelId),String(j.childPanelId)].includes(String(bodyA.panelId))&&
    [String(j.parentPanelId),String(j.childPanelId)].includes(String(bodyB.panelId))&&String(bodyA.panelId)!==String(bodyB.panelId));
  if(!joint||!(joint.neutralLength>0)||!(model.thicknessMm>0))return null;
  const pid=String(joint.parentPanelId),contacts=joint.reliefContacts?.[pid],intervals=joint.source?.materialIntervalsAxialMm;
  if(!contacts?.length||!intervals?.length||!scene.state.panelMatrices[pid])return null;
  const frame=multiply(scene.placement.frame,scene.state.panelMatrices[pid]),origin=transform(frame,joint.origin),basis=[joint.axis,joint.transverse,joint.normal].map(v=>direction(frame,v));
  if(basis.some(v=>Math.abs(Math.hypot(...v)-1)>1e-7)||Math.abs(dot(basis[0],basis[1]))>1e-7||Math.abs(dot(basis[0],basis[2]))>1e-7||Math.abs(dot(basis[1],basis[2]))>1e-7)return null;
  const toLocal=p=>basis.map(v=>dot(sub(p,origin),v));
  const epsilon=Math.max(1e-10,toleranceMm*.1),margin=4*toleranceMm+2*epsilon;
  const probeDepth=Math.max(model.thicknessMm,joint.neutralLength,1),boxes=[];
  for(const range of contacts){
    if(!Array.isArray(range)||range.length!==2||!range.every(Number.isFinite)||Math.abs(range[1]-range[0])>epsilon)continue;
    const axial=(range[0]+range[1])/2;
    if(!intervals.some(interval=>Array.isArray(interval)&&interval.some(v=>Number.isFinite(v)&&Math.abs(v-axial)<=epsilon)))continue;
    // Stay strictly inside the bend strip; the unbent parent outside its
    // tangency plane legitimately spans both axial sides and is never exempt.
    const min=[axial-probeDepth,margin,-model.thicknessMm-margin],max=[axial+probeDepth,joint.neutralLength-margin,margin];
    const a=reliefCapSide(bodyA,toLocal,axial,min,max,epsilon),b=reliefCapSide(bodyB,toLocal,axial,min,max,epsilon);
    if(!a||!b||a.outwardAxisSign===b.outwardAxisSign)continue;
    boxes.push({axialMm:axial,min:[axial-margin,min[1],min[2]],max:[axial+margin,max[1],max[2]],sides:[a,b]});
  }
  if(!boxes.length)return null;
  const clip=body=>boxes.reduce((patch,b)=>patch?clipSurfaceOutsideJointBox(patch,toLocal,b.min,b.max,indexedPatches):null,body);
  return{aFar:clip(bodyA),bFar:clip(bodyB),diagnostics:{jointId:String(joint.id),panelIds:[String(bodyA.panelId),String(bodyB.panelId)],
    method:'VERIFIED_OPPOSITE_RELIEF_CAP_CONTACT',boxes,exceptionMarginMm:margin,qualificationToleranceMm:epsilon,
    fullBodyExclusion:false,clippedPatchesAreOpen:true,
    scope:'Declared split-hinge end planes only, with actual opposite caps and local separating halfspaces verified at this pose. No axial material crossing is exempt. Far surfaces remain checked against full original solids.'}};
}

/** Both far patches; ignore ONLY pair interactions wholly within this joint box. */
export function jointFarPatches({model,scene,panelBody,bandBody,joint,clearanceMm,toleranceMm=1e-7,indexedPatches=false,certifyBounds=false}){
  const child=String(panelBody.panelId)===String(joint.childPanelId),pid=String(panelBody.panelId);
  const margin=clearanceMm+2*toleranceMm+1e-6;
  let amin=Infinity,amax=-Infinity;
  for(let i=0;i<joint.band.parameters.length;i+=3){amin=Math.min(amin,joint.band.parameters[i]);amax=Math.max(amax,joint.band.parameters[i]);}
  const min=[amin-margin,-margin,-model.thicknessMm-margin],max=[amax+margin,margin,margin];
  // Split hinge: the band ends are tear faces flush with the panel material left
  // flat between spans. Exempt only the measured relief contacts (importer reliefContacts).
  // Joint-stroke followers have the same kind of measured end contact (relief notch).
  // A hinge ending inside material (cut by a hole on a diagonal edge) has the
  // same importer-verified end contacts; the importer only records them at span ends.
  const reliefBoxes=(joint.reliefContacts?.[pid]??[]).map(([a,b])=>[
    [Math.min(a,b)-margin,(child?-joint.neutralLength:0)-margin,-model.thicknessMm-margin],
    [Math.max(a,b)+margin,(child?0:joint.neutralLength)+margin,margin]]);
  const clipAll=(body,toLocal)=>{let out=clipSurfaceOutsideJointBox(body,toLocal,min,max,indexedPatches);
    for(const [lo,hi]of reliefBoxes)if(out)out=clipSurfaceOutsideJointBox(out,toLocal,lo,hi,indexedPatches);return out;};
  const flatOrigin=joint.origin.map((v,i)=>v+(child?joint.neutralLength:0)*joint.transverse[i]);
  const flatBasis=[joint.axis,joint.transverse,joint.normal];
  const flatLocal=jointProjector(flatOrigin,flatBasis);
  const sourcePanel=model.panels.find(p=>String(p.id)===pid);
  if(!sourcePanel||!scene.state?.panelMatrices?.[pid]||!scene.placement?.frame)throw new Error('Explicit state/frame required for local joint collision clipping.');
  let cache=caches.get(model);if(!cache){cache=new Map();caches.set(model,cache);}
  const key=JSON.stringify([joint.id,pid,margin,indexedPatches]);
  const frame=multiply(scene.placement.frame,scene.state.panelMatrices[pid]);
  const worldOrigin=transform(frame,flatOrigin),worldBasis=flatBasis.map(a=>direction(frame,a));
  const worldLocal=jointProjector(worldOrigin,worldBasis);
  let bandSeparated=certifyBounds&&confinedContact(panelBody,bandBody,worldOrigin,worldBasis,min,max,toleranceMm);
  const roundTrip=certifyBounds?flatPatchError(frame,sourcePanel.flatPositions,panelBody.positions,flatOrigin):Infinity;
  let panelSeparated=bandSeparated&&Number.isFinite(roundTrip)&&
    confinedContact(panelBody,bandBody,worldOrigin,worldBasis,min,max,toleranceMm+roundTrip);
  if(certifyBounds&&!panelSeparated&&reliefBoxes.length&&roundTrip<1e-4&&
      worldBasis.every((v,i)=>worldBasis.every((w,j)=>Math.abs(dot(v,w)-(i===j?1:0))<1e-7))&&
      sourcePanel.triangles===panelBody.triangles){
    const footprintKey=key+'|footprint',geometryKey=JSON.stringify([flatOrigin,flatBasis,min,max,reliefBoxes]);
    let part=cache.get(footprintKey);
    const unchanged=(a,b)=>a.length===b.length&&a.every((v,i)=>v===b[i]);
    if(!part||part.geometryKey!==geometryKey||!unchanged(sourcePanel.flatPositions,part.source)||!unchanged(sourcePanel.triangles,part.triangles)){
      part=jointFootprint(sourcePanel.flatPositions,Array.from(sourcePanel.triangles),flatLocal);
      part.source=Float64Array.from(sourcePanel.flatPositions);part.geometryKey=geometryKey;
      cache.set(footprintKey,part);
    }
    const band=jointFootprint(bandBody.positions,bandBody.triangles,worldLocal);
    const pad=4*(toleranceMm+roundTrip)+1e-8*(1+Math.max(...worldOrigin.map(Math.abs)));
    const proofKey=footprintKey+'|previous',previous=cache.get(proofKey);
    // Compare the actual projected vertices, including thickness. Bounded
    // movement of every vertex also bounds every projected triangle point.
    let same=previous?.part===part&&previous.band.points.length===band.points.length&&previous.pad>2*pad&&unchanged(previous.indices,band.triangles);
    if(same)for(let i=0;i<band.points.length&&same;i++)for(let k=0;k<3;k++){
      if(!(Math.abs(band.points[i][k]-previous.band.points[i][k])<previous.pad/2)){same=false;break;}
    }
    const allowance=2*pad+1e-5;
    if(same||coveredFootprint(part,band,[[min,max],...reliefBoxes],allowance)){
      if(!same)cache.set(proofKey,{part,band,indices:Array.from(band.triangles),pad:allowance});
      bandSeparated=panelSeparated=true;
    }
  }
  let panelFar=null;
  if(!panelSeparated){
    let panelFlat;
    if(cache.has(key))panelFlat=cache.get(key);
    else{
      panelFlat=clipAll({...panelBody,positions:sourcePanel.flatPositions,triangles:sourcePanel.triangles},flatLocal);
      if(indexedPatches)panelFlat=compactFlatPatch(panelFlat);
      cache.set(key,panelFlat);
    }
    if(panelFlat){
      const positions=new Float64Array(panelFlat.positions.length),m=frame,src=panelFlat.positions;
      for(let i=0;i<positions.length;i+=3){
        const x=src[i],y=src[i+1],z=src[i+2];
        positions[i]=m[0]*x+m[1]*y+m[2]*z+m[3];
        positions[i+1]=m[4]*x+m[5]*y+m[6]*z+m[7];
        positions[i+2]=m[8]*x+m[9]*y+m[10]*z+m[11];
      }
      panelFar={...panelFlat,positions};
    }
  }
  const bandFar=bandSeparated?null:clipAll(bandBody,worldLocal);
  return{panelFar,bandFar,certifiedFar:bandSeparated,separationAxes:worldBasis,diagnostics:{jointId:String(joint.id),panelId:pid,side:child?'child':'parent',
    method:'EXACT_TRIANGLE_CLIP_OUTSIDE_TANGENCY_BOX',exceptionMarginMm:margin,
    localBoxMin:min,localBoxMax:max,fullBodyExclusion:false,clippedPatchesAreOpen:true,
    scope:'Only collisions with both surfaces inside the stated tangency box are exempt. Far patches are tested against original complete opposite bodies; closed-solid containment depends on those original meshes.'}};
}

// Two perpendicular bend bands can meet at the thickness edge of a shared
// panel. Only explicit, geometrically verified tangency seams establish this
// relationship. Merely sharing a panel or touching somewhere is insufficient.
function cornerGroups(model,a,b){
  let cache=cornerCaches.get(model);if(!cache){cache=new Map();cornerCaches.set(model,cache);}
  const key=JSON.stringify([a.id,b.id]);if(cache.has(key))return cache.get(key);
  const groups=[],epsilon=1e-6,basis=[a.axis,a.transverse,a.normal];
  const common=[a.parentPanelId,a.childPanelId].map(String).filter(id=>[b.parentPanelId,b.childPanelId].map(String).includes(id));
  if(Math.abs(dot(a.normal,b.normal))<1-1e-6){cache.set(key,groups);return groups;}
  for(const seam of Array.isArray(model.seams)?model.seams:[]){
    if(!Array.isArray(seam))continue;
    const ar=seam.find(r=>['bend','band'].includes(r.kind)&&String(r.id)===String(a.id));
    const br=seam.find(r=>['bend','band'].includes(r.kind)&&String(r.id)===String(b.id));
    if(!ar||!br)continue;
    for(const pid of common){
      const pr=seam.find(r=>r.kind==='panel'&&String(r.id)===pid),panel=model.panels.find(p=>String(p.id)===pid);
      if(!pr||!panel)continue;
      const refs=[[ar,a.band.parameters],[br,b.band.parameters],[pr,panel.flatPositions]];
      if(refs.some(([r,data])=>!Number.isInteger(r.vertexIndex)||r.vertexIndex<0||3*r.vertexIndex+2>=data.length))continue;
      const ap=point(a.band.parameters,ar.vertexIndex),bp=point(b.band.parameters,br.vertexIndex),p=point(panel.flatPositions,pr.vertexIndex);
      const expectedA=String(a.parentPanelId)===pid?0:1,expectedB=String(b.parentPanelId)===pid?0:1;
      if(Math.abs(ap[1]-expectedA)>1e-8||Math.abs(bp[1]-expectedB)>1e-8)continue;
      if([bandPoint(a,...ap,0),bandPoint(b,...bp,0)].some(v=>Math.hypot(...sub(v,p))>epsilon))continue;
      const q=basis.map(v=>dot(p,v));
      let group=groups.find(g=>g.panelId===pid&&Math.hypot(q[0]-g.anchor[0],q[1]-g.anchor[1])<=epsilon);
      if(!group){group={panelId:pid,anchor:q,basis,points:[]};groups.push(group);}
      group.points.push({flat:p,q,a:ar.vertexIndex,b:br.vertexIndex});
    }
  }
  // Corners without relief: the flat pattern itself makes the two bend zones
  // share vertices on the common panel side. Exempt only those coincident flat
  // vertices; the pose check in bandCornerFarPatches drops them once they move apart.
  // Also when a seam welds one corner vertex: a trimmed band sliver can still
  // share a longer flat edge that the single welded vertex does not cover.
  // Spans of one CAD bend line hanging from different panels split one flat
  // strip: their bands abut at the cut. The column stays on the first span's parent.
  const collinear=!common.length&&Math.abs(dot(a.axis,b.axis))>1-1e-6&&
    Math.abs(dot(sub(b.origin,a.origin),a.transverse))<=1e-3&&Math.abs(dot(sub(b.origin,a.origin),a.normal))<=1e-3;
  if(common.length||collinear){
    const pid=common[0]??String(a.parentPanelId),count=data=>Math.floor((data?.length||0)/3),seamGroups=groups.length;
    const flatA=[...Array(count(a.band.parameters)).keys()].filter(i=>point(a.band.parameters,i)[2]===0).map(i=>[i,bandPoint(a,...point(a.band.parameters,i).slice(0,2),0)]);
    for(let j=0;j<count(b.band.parameters);j++){
      if(point(b.band.parameters,j)[2]!==0)continue;
      const p=bandPoint(b,...point(b.band.parameters,j).slice(0,2),0);
      for(const [i,v] of flatA){
        if(Math.hypot(...sub(v,p))>0.01)continue;
        const q=basis.map(w=>dot(p,w));
        let group=groups[seamGroups];
        if(!group){group={panelId:pid,anchor:q,basis,points:[],throughThicknessMm:Math.abs(Number(model.thicknessMm)||0)};groups.push(group);}
        group.points.push({flat:p,flatA:v,q,a:i,b:j});
      }
    }
  }
  cache.set(key,groups);return groups;
}

/** Remove only the small column around a declared shared thickness edge.
 * Both far patches are still tested against the ORIGINAL opposite mesh.
 */
export function bandCornerFarPatches({model,scene,bodyA,bodyB,clearanceMm,toleranceMm=1e-7,indexedPatches=false,certifyBounds=false}){
  const a=model.bends.find(j=>String(j.id)===String(bodyA.bendId)),b=model.bends.find(j=>String(j.id)===String(bodyB.bendId));
  if(!a||!b||!scene.placement?.frame||!scene.state?.panelMatrices)return null;
  const margin=clearanceMm+2*toleranceMm+1e-6,boxes=[];
  for(const group of cornerGroups(model,a,b)){
    const panelFrame=scene.state.panelMatrices[group.panelId];if(!panelFrame)continue;
    const frame=multiply(scene.placement.frame,panelFrame);
    // A stale or broken seam must not suppress a collision in the current pose.
    // Relief-less corners stay a fixed column on the common panel: the shared
    // flat edge only separates gradually, while everything outside is still checked.
    if(!group.throughThicknessMm&&group.points.some(p=>Math.hypot(...sub(point(bodyA.positions,p.a),transform(frame,p.flatA??p.flat)))>1e-6||
      Math.hypot(...sub(point(bodyB.positions,p.b),transform(frame,p.flat)))>1e-6))continue;
    const origin=group.points[0].flat,worldOrigin=transform(frame,origin),worldBasis=group.basis.map(v=>direction(frame,v));
    const local=group.points.map(p=>group.basis.map(v=>dot(sub(p.flat,origin),v)));
    const min=[0,1,2].map(k=>Math.min(...local.map(p=>p[k]))-margin),max=[0,1,2].map(k=>Math.max(...local.map(p=>p[k]))+margin);
    // Relief arcs of two bands can meet tangentially at a single flat vertex:
    // their surfaces stay sub-micron apart for a thickness-scale distance.
    if(group.throughThicknessMm){const t=group.throughThicknessMm;for(const k of [0,1,2]){min[k]-=t;max[k]+=t;}}
    boxes.push({panelId:group.panelId,min,max,worldOrigin,worldBasis,toLocal:p=>worldBasis.map(v=>dot(sub(p,worldOrigin),v))});
  }
  if(!boxes.length)return null;
  const clip=body=>boxes.reduce((patch,box)=>patch?clipSurfaceOutsideJointBox(patch,box.toLocal,box.min,box.max,indexedPatches):null,body);
  const separated=certifyBounds&&boxes.some(b=>confinedContact(bodyA,bodyB,b.worldOrigin,b.worldBasis,b.min,b.max,toleranceMm));
  return{aFar:separated?null:clip(bodyA),bFar:separated?null:clip(bodyB),certifiedFar:separated,diagnostics:{bendIds:[String(a.id),String(b.id)],
    method:'EXACT_TRIANGLE_CLIP_OUTSIDE_SHARED_CORNER',exceptionMarginMm:margin,
    boxes:boxes.map(({panelId,min,max})=>({panelId,min,max})),fullBodyExclusion:false,clippedPatchesAreOpen:true,
    scope:'Only the local column of verified shared tangency vertices is exempt; both far surfaces are checked against the original opposite mesh.'}};
}
