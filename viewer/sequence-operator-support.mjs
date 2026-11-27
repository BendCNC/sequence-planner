/** Material-side presentation constraint; not a hand, balance or gripping simulation. */
function scalarArea(a,b,c){
  const ux=b[0]-a[0],uy=b[1]-a[1],uz=b[2]-a[2];
  const vx=c[0]-a[0],vy=c[1]-a[1],vz=c[2]-a[2];
  return Math.hypot(uy*vz-uz*vy,uz*vx-ux*vz,ux*vy-uy*vx)/2;
}
// Typed transforms can round a component through their native .map operation.
function area(a,b,c){
  if(Array.isArray(a)&&Array.isArray(b)&&Array.isArray(c))return scalarArea(a,b,c);
  const u=b.map((v,i)=>v-a[i]),v=c.map((n,i)=>n-a[i]);
  return Math.hypot(u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0])/2;
}
const point=(positions,i)=>[positions[i*3],positions[i*3+1],positions[i*3+2]];
function skin(bodyId,positions,triangles,thickness){
  let top=-Infinity;for(let i=2;i<positions.length;i+=3)top=Math.max(top,positions[i]);
  const result=[];
  let i=0;
  for(;i+2<triangles.length;i+=3){
    const a=triangles[i],b=triangles[i+1],c=triangles[i+2],limit=thickness*.1;
    // Only the reference skin contributes. Avoid allocating faces for the walls or bottom.
    if(top-positions[3*a+2]>limit||top-positions[3*b+2]>limit||top-positions[3*c+2]>limit)continue;
    const weight=scalarArea(point(positions,a),point(positions,b),point(positions,c));
    if(weight>1e-9)result.push({indices:[a,b,c],areaMm2:weight});
  }
  if(i<triangles.length){
    // Public route checks can receive an unvalidated contour. Historically a
    // truncated wall face was ignored; a truncated top face raised an error.
    const tail=Array.from(triangles.slice(i),index=>point(positions,index));
    if(!tail.some(p=>top-p[2]>thickness*.1))area(...tail);
  }
  if(!result.length)throw new Error('No reference material skin for '+bodyId);
  return{bodyId,triangles:result,areaMm2:result.reduce((sum,t)=>sum+t.areaMm2,0)};
}
export function buildOperatorSupport(model,bendId){
  const bend=model.bends.find(b=>String(b.id)===String(bendId));if(!bend)throw new Error('Unknown bend');
  const graph=new Map(model.panels.map(p=>[String(p.id),[]]));
  for(const b of model.bends)if(b!==bend){graph.get(String(b.parentPanelId)).push(String(b.childPanelId));graph.get(String(b.childPanelId)).push(String(b.parentPanelId));}
  function branch(start){
    const ids=new Set([String(start)]),queue=[String(start)];for(const id of queue)for(const next of graph.get(id))if(!ids.has(next)){ids.add(next);queue.push(next);}
    const panels=model.panels.filter(p=>ids.has(String(p.id))),bands=model.bends.filter(b=>b!==bend&&ids.has(String(b.parentPanelId))&&ids.has(String(b.childPanelId)));
    const bodies=[...panels.map(p=>skin('panel:'+p.id,p.flatPositions,p.triangles,model.thicknessMm)),...bands.map(b=>skin('band:'+b.id,b.band.flatPositions,b.band.triangles,model.thicknessMm))];
    return{panelIds:panels.map(p=>String(p.id)),bendIds:bands.map(b=>String(b.id)),areaMm2:bodies.reduce((sum,b)=>sum+b.areaMm2,0),bodies};
  }
  const branches=[branch(bend.parentPanelId),branch(bend.childPanelId)];
  if(branches[0].panelIds.some(id=>branches[1].panelIds.includes(id)))throw new Error('Active bend does not separate the panel graph');
  const sorted=[...branches].sort((a,b)=>b.areaMm2-a.areaMm2);
  return{bendId:String(bendId),majorBranch:sorted[0],minorBranch:sorted[1],
    rule:'The larger branch has material centroid X<0 and strictly more than half its reference material skin area in X<0.',
    method:'One actual triangulated flat skin per panel and internal bend; holes preserved. Active bend excluded. Flat reference area weights are retained during deformation.',
    activeBandExcluded:true,gripVerified:false};
}
function frontPolygon(points){
  const out=[];
  for(let i=0;i<points.length;i++){
    const a=points[i],b=points[(i+1)%points.length],insideA=a[0]<=0,insideB=b[0]<=0;
    if(insideA)out.push(a);
    if(insideA!==insideB){const t=-a[0]/(b[0]-a[0]);out.push(a.map((v,k)=>v+t*(b[k]-v)));}
  }return out;
}
/** The scalar is a public alias of the vector calculated at the same pose. */
export function consistentSupportCentroid(support){
  return Number.isFinite(support?.centroid?.[0])&&support.centroidX===support.centroid[0];
}

export function evaluateOperatorSupport(prepared,scene,{transformPoint,posedPointsById}={}){
  const byId=new Map(scene.bodies.map(b=>[b.id,b]));
  let frontAreaMm2=0,totalAreaMm2=0,wx=0,wy=0,wz=0;
  for(const body of prepared.majorBranch.bodies){
    const mesh=byId.get(body.bodyId);if(!mesh)throw new Error('Missing body '+body.bodyId);
    const posed=posedPointsById?.get(mesh.id),positions=mesh.positions;
    for(const triangle of body.triangles){
      const indices=triangle.indices,weight=triangle.areaMm2;
      let ax,ay,az,bx,by,bz,cx,cy,cz,aPoint,bPoint,cPoint;
      if(posed||transformPoint!==undefined){
        if(posed){
          aPoint=posed[indices[0]];bPoint=posed[indices[1]];cPoint=posed[indices[2]];
        }else{
          aPoint=transformPoint(point(positions,indices[0]));bPoint=transformPoint(point(positions,indices[1]));
          cPoint=transformPoint(point(positions,indices[2]));
        }
        if((aPoint.length!==3||bPoint.length!==3||cPoint.length!==3)&&[aPoint,bPoint,cPoint].some(v=>v.some(n=>!Number.isFinite(n))))throw new Error('Nonfinite support geometry');
        ax=aPoint[0];ay=aPoint[1];az=aPoint[2];bx=bPoint[0];by=bPoint[1];bz=bPoint[2];
        cx=cPoint[0];cy=cPoint[1];cz=cPoint[2];
      }else{
        const a=3*indices[0],b=3*indices[1],c=3*indices[2];
        ax=positions[a];ay=positions[a+1];az=positions[a+2];
        bx=positions[b];by=positions[b+1];bz=positions[b+2];
        cx=positions[c];cy=positions[c+1];cz=positions[c+2];
      }
      if((!Number.isFinite(ax)||!Number.isFinite(ay)||!Number.isFinite(az)||
         !Number.isFinite(bx)||!Number.isFinite(by)||!Number.isFinite(bz)||
         !Number.isFinite(cx)||!Number.isFinite(cy)||!Number.isFinite(cz))&&
         (!aPoint||[aPoint,bPoint,cPoint].some(v=>v.some(n=>!Number.isFinite(n)))))throw new Error('Nonfinite support geometry');
      totalAreaMm2+=weight;
      wx+=weight*(ax+bx+cx)/3;wy+=weight*(ay+by+cy)/3;wz+=weight*(az+bz+cz)/3;
      if(ax<=0&&bx<=0&&cx<=0){frontAreaMm2+=weight;continue;}
      if(ax>=0&&bx>=0&&cx>=0)continue;
      const p=aPoint?[aPoint,bPoint,cPoint]:[[ax,ay,az],[bx,by,bz],[cx,cy,cz]],polygon=frontPolygon(p),full=area(...p);let clipped=0;
      for(let i=1;i<polygon.length-1;i++)clipped+=area(polygon[0],polygon[i],polygon[i+1]);
      if(full<1e-12)throw new Error('Collapsed material triangle');
      frontAreaMm2+=weight*Math.max(0,Math.min(1,clipped/full));
    }
  }
  const centroid=[wx/totalAreaMm2,wy/totalAreaMm2,wz/totalAreaMm2],frontFraction=frontAreaMm2/totalAreaMm2;
  return{passes:centroid[0]<-1e-7&&frontFraction>.5+1e-9,centroid,centroidX:centroid[0],frontFraction,backFraction:1-frontFraction,
    frontAreaMm2,totalAreaMm2,majorPanelIds:prepared.majorBranch.panelIds,gripVerified:false};
}

 // Rigid motion preserves both the material centroid and the flat reference
 // weights. Whole triangles on the operator side provide a lower bound on
 // front area. Ambiguous centroids, triangles or areas use the full clipping.
export function prepareRigidFrontProof(prepared,partsById,pivot){
 const rigidSupport=[],totalSupport=prepared.majorBranch.areaMm2;
 let weightedX=0,weightedZ=0,minAltitude=Infinity;
 for(const skin of prepared.majorBranch.bodies){
  const part=partsById.get(skin.bodyId);
  for(const face of skin.triangles){
   const [a,b,c]=face.indices.map(i=>part.positions[i]),w=face.areaMm2;
   weightedX+=w*(a[0]+b[0]+c[0])/3;weightedZ+=w*(a[2]+b[2]+c[2])/3;
   const ux=b[0]-a[0],uy=b[1]-a[1],uz=b[2]-a[2],vx=c[0]-a[0],vy=c[1]-a[1],vz=c[2]-a[2];
   const twiceArea=Math.hypot(uy*vz-uz*vy,uz*vx-ux*vz,ux*vy-uy*vx);
   minAltitude=Math.min(minAltitude,(twiceArea-2e-12)/(1+Math.hypot(ux,uy,uz)+Math.hypot(vx,vy,vz)));
   rigidSupport.push({w,dx:[a[0]-pivot[0],b[0]-pivot[0],c[0]-pivot[0]],dz:[a[2]-pivot[2],b[2]-pivot[2],c[2]-pivot[2]]});
  }
 }
 rigidSupport.sort((a,b)=>b.w-a.w);
 const cx=weightedX/totalSupport-pivot[0],cz=weightedZ/totalSupport-pivot[2],requiredArea=totalSupport*(.5+1e-9+64*Number.EPSILON*rigidSupport.length);
 return function frontMajority(c,s,x,rounding){
  if(!(minAltitude>rounding*8&&pivot[0]+c*cx+s*cz+x < -1e-7-rounding*(1+rigidSupport.length)))return false;
  let frontArea=0;
  for(const {w,dx,dz} of rigidSupport){
   if(pivot[0]+c*dx[0]+s*dz[0]+x>=-rounding||pivot[0]+c*dx[1]+s*dz[1]+x>=-rounding||pivot[0]+c*dx[2]+s*dz[2]+x>=-rounding)continue;
   frontArea+=w;if(frontArea>requiredArea)return true;
  }
  return false;
 };
}
