// Triangle-surface distance plus closed-solid containment. Coordinates are millimetres.
// This is a static mesh query, not continuous collision detection or a contact solver.
// These hot-path vectors always have three coordinates. Avoid creating a
// callback for every edge/triangle operation in the collision queries.
const add=(a,b)=>[a[0]+b[0],a[1]+b[1],a[2]+b[2]];
const sub=(a,b)=>[a[0]-b[0],a[1]-b[1],a[2]-b[2]];
const mul=(a,s)=>[a[0]*s,a[1]*s,a[2]*s];
const dot=(a,b)=>a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const norm2=a=>dot(a,a);
const clamp=x=>Math.max(0,Math.min(1,x));
const midpoint=(a,b)=>mul(add(a,b),.5);

function triples(values,label) {
  if(!values || !Number.isInteger(values.length)) throw new TypeError(`${label} must be an array`);
  if(values.length && (Array.isArray(values[0]) || ArrayBuffer.isView(values[0]))) {
    return Array.from(values,v=>{
      if(v.length!==3) throw new RangeError(`${label} entries must have three values`);
      return Array.from(v,Number);
    });
  }
  if(values.length%3) throw new RangeError(`${label} length must be divisible by three`);
  const out=[];
  for(let i=0;i<values.length;i+=3) out.push([Number(values[i]),Number(values[i+1]),Number(values[i+2])]);
  return out;
}
function bounds(points) {
  const min=[Infinity,Infinity,Infinity],max=[-Infinity,-Infinity,-Infinity];
  for(const p of points) for(let k=0;k<3;k++) { min[k]=Math.min(min[k],p[k]);max[k]=Math.max(max[k],p[k]); }
  return {min,max};
}
function mergeBounds(items) {
  const min=[Infinity,Infinity,Infinity],max=[-Infinity,-Infinity,-Infinity];
  for(const t of items) {
    const a=t.bounds.min,b=t.bounds.max;
    for(let k=0;k<3;k++) {
      if(a[k]<min[k]) min[k]=a[k]; if(a[k]>max[k]) max[k]=a[k];
      if(b[k]<min[k]) min[k]=b[k]; if(b[k]>max[k]) max[k]=b[k];
    }
  }
  return {min,max};
}
export function boxDistance2(a,b) {
  const dx=Math.max(0,a.min[0]-b.max[0],b.min[0]-a.max[0]);
  const dy=Math.max(0,a.min[1]-b.max[1],b.min[1]-a.max[1]);
  const dz=Math.max(0,a.min[2]-b.max[2],b.min[2]-a.max[2]);
  return dx*dx+dy*dy+dz*dz;
}
function makeBVH(items,leafSize) {
  const box=mergeBounds(items);
  if(items.length<=leafSize) return {bounds:box,items,count:items.length};
  const extent=box.max.map((x,i)=>x-box.min[i]);
  const axis=extent.indexOf(Math.max(...extent));
  const sorted=items.slice().sort((a,b)=>a.center[axis]-b.center[axis]);
  const half=Math.floor(sorted.length/2);
  return {bounds:box,count:items.length,left:makeBVH(sorted.slice(0,half),leafSize),right:makeBVH(sorted.slice(half),leafSize)};
}
// Partition (triangle indices per node) of a built tree, reused for rigidly
// moved copies of the same mesh: only the node bounds are recomputed.
function bvhShape(node) {
  return node.left?{left:bvhShape(node.left),right:bvhShape(node.right)}:{leaf:node.items.map(it=>it.index)};
}
function refitBVH(shape,items) {
  if(shape.leaf){const leaf=shape.leaf.map(i=>items[i]);return {bounds:mergeBounds(leaf),items:leaf,count:leaf.length};}
  const left=refitBVH(shape.left,items),right=refitBVH(shape.right,items);
  return {bounds:mergeBounds([left,right]),count:left.count+right.count,left,right};
}
const bvhShapeCache=new WeakMap();

function degenerateTriangle(a,b,c){
  const ux=b[0]-a[0],uy=b[1]-a[1],uz=b[2]-a[2];
  const vx=c[0]-a[0],vy=c[1]-a[1],vz=c[2]-a[2];
  const wx=c[0]-b[0],wy=c[1]-b[1],wz=c[2]-b[2];
  const nx=uy*vz-uz*vy,ny=uz*vx-ux*vz,nz=ux*vy-uy*vx;
  const edge2=Math.max(ux*ux+uy*uy+uz*uz,vx*vx+vy*vy+vz*vz,wx*wx+wy*wy+wz*wz);
  return nx*nx+ny*ny+nz*nz<=edge2*edge2*1e-24;
}

// Reuse only the partition established by the first eager build. Packed
// bounds are ordered by that tree's original leaf traversal, including ties.
function packedPartition(root){
  const nodes=[];
  function visit(node){
    if(!node.leaf){visit(node.left);visit(node.right);node.count=node.left.count+node.right.count;}else node.count=node.leaf.length;
    node.offset=6*nodes.length;
    nodes.push(node);
  }
  visit(root);
  return {root,nodes};
}

function deferredRefit(id,vertices,faces,box,topo,partition){
  const packed=new Float64Array(partition.nodes.length*6);
  let degenerateTriangles=0;
  for(const node of partition.nodes){
    const offset=node.offset;
    if(node.leaf){
      packed[offset]=packed[offset+1]=packed[offset+2]=Infinity;
      packed[offset+3]=packed[offset+4]=packed[offset+5]=-Infinity;
      for(const index of node.leaf){
        const f=faces[index],a=vertices[f[0]],b=vertices[f[1]],c=vertices[f[2]];
        if(degenerateTriangle(a,b,c))degenerateTriangles++;
        for(let k=0;k<3;k++){
          const lo=Math.min(a[k],b[k],c[k]),hi=Math.max(a[k],b[k],c[k]);
          const low=offset+k,high=low+3;
          if(lo<packed[low])packed[low]=lo;
          if(lo>packed[high])packed[high]=lo;
          if(hi>packed[high])packed[high]=hi;
        }
      }
    }else{
      const l=node.left.offset,r=node.right.offset;
      for(let k=0;k<3;k++){
        const lmin=packed[l+k],lmax=packed[l+k+3],rmin=packed[r+k],rmax=packed[r+k+3];
        packed[offset+k]=rmin<lmin?rmin:lmin;
        packed[offset+3+k]=rmax>lmax?rmax:lmax;
      }
    }
  }
  const materialized=new Array(faces.length);
  function item(index){
    let t=materialized[index];
    if(t)return t;
    const f=faces[index],p=[vertices[f[0]],vertices[f[1]],vertices[f[2]]];
    const min=[Math.min(p[0][0],p[1][0],p[2][0]),Math.min(p[0][1],p[1][1],p[2][1]),Math.min(p[0][2],p[1][2],p[2][2])];
    const max=[Math.max(p[0][0],p[1][0],p[2][0]),Math.max(p[0][1],p[1][1],p[2][1]),Math.max(p[0][2],p[1][2],p[2][2])];
    t={index,p,bounds:{min,max},center:mul(add(add(p[0],p[1]),p[2]),1/3)};
    materialized[index]=t;
    return t;
  }
  function branch(node){
    const i=node.offset,bounds={min:[packed[i],packed[i+1],packed[i+2]],max:[packed[i+3],packed[i+4],packed[i+5]]};
    if(node.leaf){
      let values;
      return {bounds,get items(){return values??=node.leaf.map(item);},count:node.leaf.length};
    }
    const count=node.count;
    let left,right;
    return {bounds,count,get left(){return left??=branch(node.left);},get right(){return right??=branch(node.right);}};
  }
  const components=topo.components.map(c=>({triangleIndices:c.triangleIndices,representative:vertices[faces[c.first][0]]}));
  const closed=topo.boundaryEdges===0&&topo.nonManifoldEdges===0&&degenerateTriangles===0;
  let all;
  return {id:String(id??'mesh'),positions:vertices,triangles:faces,bounds:box,get items(){
    if(!all){for(let i=0;i<faces.length;i++)item(i);all=materialized;}return all;
  },bvh:branch(partition.root),components,closed,diagnostics:{closed,boundaryEdges:topo.boundaryEdges,nonManifoldEdges:topo.nonManifoldEdges,degenerateTriangles,components:components.length,
    containmentAssumption:'Non-self-intersecting closed shells; holes require actual boundary triangles, not filled contour fans.'}};
}

// Eligibility for an immutable numeric broad-phase query. All other supported
// representations go through the ordinary collider and its conversions.
export function validatePackedMesh(body){
  const {positions:p,triangles:t,leafSize=8}=body;
  if(!p?.length||!t?.length||p.length%3||t.length%3||!Number.isInteger(leafSize)||leafSize<1||
      !p.every(Number.isFinite)||!t.every(i=>Number.isInteger(i)&&i>=0&&i<p.length/3))return meshCollider(body);
  return null;
}

/** Immutable copy of indexed triangles; supports shared or duplicated vertex indices. */
export function meshCollider(body) {return makeCollider(body,false);}
export function meshColliderForScene(body) {return makeCollider(body,true);}
function makeCollider({id,positions,triangles,leafSize=8},deferPartition) {
  const vertices=triples(positions,'positions'),faces=triples(triangles,'triangles');
  if(!vertices.length || !faces.length) throw new RangeError('Mesh must contain vertices and triangles');
  if(vertices.some(p=>p.some(x=>!Number.isFinite(x)))) throw new TypeError('Non-finite mesh position');
  if(faces.some(t=>t.some(i=>!Number.isInteger(i)||i<0||i>=vertices.length))) throw new RangeError('Invalid triangle index');
  if(!Number.isInteger(leafSize)||leafSize<1) throw new RangeError('leafSize must be a positive integer');
  const box=bounds(vertices),scale=Math.max(1,...box.max.map((x,i)=>x-box.min[i]));
  if(deferPartition&&faces.length>=1024&&faces.length<=0xffffffff/12&&typeof triangles==='object'&&triangles!==null){
    const topo=topologyCache.get(triangles),shape=bvhShapeCache.get(triangles);
    if(topo?.vertexCount===vertices.length&&shape?.leafSize===leafSize&&shape.count===faces.length){
      shape.partition??=packedPartition(shape.shape);
      return deferredRefit(id,vertices,faces,box,topo,shape.partition);
    }
  }
  let degenerateTriangles=0;
  const items=faces.map((indices,index)=>{
    const p=indices.map(i=>vertices[i]);
    // A curved bend changes the actual triangle area. Test the current
    // positions using that triangle's edge scale; a large distant panel or
    // a worker's first pose cannot turn a small real face into a hole.
    if(degenerateTriangle(p[0],p[1],p[2]))degenerateTriangles++;
    return {index,p,bounds:bounds(p),center:mul(add(add(p[0],p[1]),p[2]),1/3)};
  });
  // Topology depends only on which vertices coincide. Rigidly moved copies of
  // the same mesh (same triangles array) apply identical float operations to
  // coincident vertices, so the welded topology is reused instead of rebuilt.
  const cacheable=typeof triangles==='object'&&triangles!==null;
  let topo=cacheable?topologyCache.get(triangles):undefined;
  if(!topo||topo.vertexCount!==vertices.length) {
    topo=meshTopology(vertices,faces,items,scale);
    if(cacheable) topologyCache.set(triangles,topo);
  }
  const components=topo.components.map(c=>({triangleIndices:c.triangleIndices,representative:items[c.first].p[0]}));
  const closed=topo.boundaryEdges===0&&topo.nonManifoldEdges===0&&degenerateTriangles===0;
  let bvh;
  const shapeKey=cacheable&&bvhShapeCache.get(triangles);
  if(shapeKey&&shapeKey.leafSize===leafSize&&shapeKey.count===items.length) bvh=refitBVH(shapeKey.shape,items);
  else {bvh=makeBVH(items,leafSize);if(cacheable) bvhShapeCache.set(triangles,{leafSize,count:items.length,shape:bvhShape(bvh)});}
  return {id:String(id??'mesh'),positions:vertices,triangles:faces,bounds:box,items,bvh,components,
    closed,diagnostics:{closed,boundaryEdges:topo.boundaryEdges,nonManifoldEdges:topo.nonManifoldEdges,degenerateTriangles,components:components.length,
      containmentAssumption:'Non-self-intersecting closed shells; holes require actual boundary triangles, not filled contour fans.'}};
}

const topologyCache=new WeakMap();
// A separated seam still records whether the original meshes are closed.
// Reuse only the welded connectivity. Area is checked at the current pose.
export function knownMeshClosure(body){
  const {positions:p,triangles:t}=body,topo=topologyCache.get(t);
  if(!topo||topo.vertexCount!==p.length/3||typeof t[0]!=='number')return undefined;
  if(topo.boundaryEdges||topo.nonManifoldEdges)return false;
  for(let i=0;i<t.length;i+=3){
    const a=t[i]*3,b=t[i+1]*3,c=t[i+2]*3;
    const ux=p[b]-p[a],uy=p[b+1]-p[a+1],uz=p[b+2]-p[a+2];
    const vx=p[c]-p[a],vy=p[c+1]-p[a+1],vz=p[c+2]-p[a+2];
    const wx=p[c]-p[b],wy=p[c+1]-p[b+1],wz=p[c+2]-p[b+2];
    const nx=uy*vz-uz*vy,ny=uz*vx-ux*vz,nz=ux*vy-uy*vx;
    const edge=Math.max(ux*ux+uy*uy+uz*uz,vx*vx+vy*vy+vz*vz,wx*wx+wy*wy+wz*wz);
    if(nx*nx+ny*ny+nz*nz<=edge*edge*1e-24)return false;
  }
  return true;
}
function meshTopology(vertices,faces,items,scale) {
  // Weld duplicated vertices only at numeric precision, not at a manufacturing tolerance.
  const weld=scale*1e-10;
  const byX=new Map(),vertexIds=new Uint32Array(vertices.length);
  let stride=0;
  for(let i=0;i<vertices.length;i++){
    const p=vertices[i],x=Math.round(p[0]/weld),y=Math.round(p[1]/weld),z=Math.round(p[2]/weld);
    let byY=byX.get(x);if(!byY){byY=new Map();byX.set(x,byY);}
    let byZ=byY.get(y);if(!byZ){byZ=new Map();byY.set(y,byZ);}
    let id=byZ.get(z);
    if(id===undefined){id=stride++;byZ.set(z,id);}
    vertexIds[i]=id;
  }
  // Unordered pairs of welded IDs preserve edge grouping and insertion order.
  // Integers remove repeated coordinate formatting for the three edges per face.
  const numeric=stride*stride<=Number.MAX_SAFE_INTEGER;
  const edgeMap=new Map();
  faces.forEach((indices,index)=>{
    for(let e=0;e<3;e++) {
      const a=vertexIds[indices[e]],b=vertexIds[indices[(e+1)%3]],lo=Math.min(a,b),hi=Math.max(a,b);
      const key=numeric?lo*stride+hi:lo+'|'+hi;
      const incident=edgeMap.get(key);
      if(incident)incident.push(index);else edgeMap.set(key,[index]);
    }
  });
  const adjacency=items.map(()=>[]);
  let boundaryEdges=0,nonManifoldEdges=0;
  for(const inc of edgeMap.values()) {
    if(inc.length===1) boundaryEdges++;
    if(inc.length>2) nonManifoldEdges++;
    for(let i=1;i<inc.length;i++) {adjacency[inc[0]].push(inc[i]);adjacency[inc[i]].push(inc[0]);}
  }
  const seen=new Set(),components=[];
  for(const item of items) if(!seen.has(item.index)) {
    const pending=[item.index],members=[];seen.add(item.index);
    while(pending.length) {
      const index=pending.pop();members.push(index);
      for(const neighbor of adjacency[index]) if(!seen.has(neighbor)) {seen.add(neighbor);pending.push(neighbor);}
    }
    components.push({triangleIndices:members,first:item.index});
  }
  return {vertexCount:vertices.length,components,boundaryEdges,nonManifoldEdges};
}

function pointSegment(p,a,b) {
  const ab=sub(b,a),d=norm2(ab),t=d>0?clamp(dot(sub(p,a),ab)/d):0;
  const q=add(a,mul(ab,t));return {d2:norm2(sub(p,q)),a:p,b:q};
}
function best(...candidates) { return candidates.reduce((a,b)=>!a||b.d2<a.d2?b:a,null); }
// A meshCollider owns immutable vertex coordinates. All opponents of one
// triangle use the same directed edges and the same degeneracy threshold.
const featureKey=Symbol('triangleFeatures');
function triangleFeatures(item) {
  // Rigid translations preserve the BVH and copy the item, then replace its
  // points. A spread may carry features computed in the previous frame.
  if(item[featureKey]?.points===item.p)return item[featureKey];
  const points=item.p,[a,b,c]=points,edges=[sub(b,a),sub(c,b),sub(a,c)],ac=sub(c,a);
  const length2=[norm2(edges[0]),norm2(edges[1]),norm2(edges[2])],ac2=norm2(ac);
  const axis=cross(edges[0],ac),area2=norm2(axis),longest2=Math.max(...length2),longest=Math.sqrt(longest2);
  const projectionB=dot(edges[0],axis),projectionC=dot(ac,axis);
  const maxCoord=p=>Math.max(Math.abs(p[0]),Math.abs(p[1]),Math.abs(p[2]));
  const ma=maxCoord(a),mb=maxCoord(b),mc=maxCoord(c);
  return item[featureKey]={points,edges,ac,length2,ac2,degenerate:area2<=Number.EPSILON**2*Math.max(1,length2[0]*ac2),
    axis,axisLength:Math.sqrt(area2),minProjection:Math.min(0,projectionB,projectionC),maxProjection:Math.max(0,projectionB,projectionC),
    longest,quality:area2/(longest2*longest2),conditioning:longest*longest2/area2,
    edgeMax:[Math.max(ma,mb),Math.max(mb,mc),Math.max(mc,ma)],max:Math.max(ma,mb,mc)};
}
// Each triangle pair has one incumbent. Preserve comparison and tie order,
// materializing the two witness coordinates only when the distance improves.
function keepPoint(p,qx,qy,qz,result,reverse,known){
  const dx=p[0]-qx,dy=p[1]-qy,dz=p[2]-qz,d2=dx*dx+dy*dy+dz*dz;
  if(d2<result.d2){
    const q=known??[qx,qy,qz];
    result.d2=d2;result.a=reverse?q:p;result.b=reverse?p:q;
  }
}
function pointTriangle(p,tri,result,reverse) {
  const [a,b,c]=tri.points,ab=tri.edges[0],ac=tri.ac;
  if(tri.degenerate){
    const q=best(pointSegment(p,a,b),pointSegment(p,b,c),pointSegment(p,c,a)).b;
    keepPoint(p,q[0],q[1],q[2],result,reverse,q);return;
  }
  const ax=p[0]-a[0],ay=p[1]-a[1],az=p[2]-a[2];
  const d1=ab[0]*ax+ab[1]*ay+ab[2]*az,d2=ac[0]*ax+ac[1]*ay+ac[2]*az;
  if(d1<=0&&d2<=0){keepPoint(p,a[0],a[1],a[2],result,reverse,a);return;}
  const bx=p[0]-b[0],by=p[1]-b[1],bz=p[2]-b[2];
  const d3=ab[0]*bx+ab[1]*by+ab[2]*bz,d4=ac[0]*bx+ac[1]*by+ac[2]*bz;
  if(d3>=0&&d4<=d3){keepPoint(p,b[0],b[1],b[2],result,reverse,b);return;}
  const vc=d1*d4-d3*d2;
  if(vc<=0&&d1>=0&&d3<=0){
    const t=d1/(d1-d3);keepPoint(p,a[0]+ab[0]*t,a[1]+ab[1]*t,a[2]+ab[2]*t,result,reverse);return;
  }
  const cx=p[0]-c[0],cy=p[1]-c[1],cz=p[2]-c[2];
  const d5=ab[0]*cx+ab[1]*cy+ab[2]*cz,d6=ac[0]*cx+ac[1]*cy+ac[2]*cz;
  if(d6>=0&&d5<=d6){keepPoint(p,c[0],c[1],c[2],result,reverse,c);return;}
  const vb=d5*d2-d1*d6;
  if(vb<=0&&d2>=0&&d6<=0){
    const t=d2/(d2-d6);keepPoint(p,a[0]+ac[0]*t,a[1]+ac[1]*t,a[2]+ac[2]*t,result,reverse);return;
  }
  const va=d3*d6-d5*d4;
  if(va<=0&&(d4-d3)>=0&&(d5-d6)>=0){
    const t=(d4-d3)/((d4-d3)+(d5-d6)),bc=tri.edges[1];
    keepPoint(p,b[0]+bc[0]*t,b[1]+bc[1]*t,b[2]+bc[2]*t,result,reverse);return;
  }
  const denom=1/(va+vb+vc),v=vb*denom,w=vc*denom;
  keepPoint(p,a[0]+(ab[0]*v+ac[0]*w),a[1]+(ab[1]*v+ac[1]*w),a[2]+(ab[2]*v+ac[2]*w),result,reverse);
}
function closerEdgePair(one,i,two,j,bestPair) {
  const p1=one.points[i],p2=two.points[j],d1=one.edges[i],d2=two.edges[j];
  const rx=p1[0]-p2[0],ry=p1[1]-p2[1],rz=p1[2]-p2[2];
  const a=one.length2[i],e=two.length2[j],f=d2[0]*rx+d2[1]*ry+d2[2]*rz;
  let s=0,t=0;
  if(a<=1e-28&&e<=1e-28) {
    const d2=rx*rx+ry*ry+rz*rz;
    if(d2<bestPair.d2){bestPair.d2=d2;bestPair.a=p1;bestPair.b=p2;}return;
  }
  if(a<=1e-28) t=clamp(f/e);
  else {
    const c=d1[0]*rx+d1[1]*ry+d1[2]*rz;
    if(e<=1e-28) s=clamp(-c/a);
    else {
      const b=dot(d1,d2),denom=a*e-b*b;
      s=denom>1e-15*a*e?clamp((b*f-c*e)/denom):0;
      t=(b*s+f)/e;
      if(t<0) {t=0;s=clamp(-c/a);} else if(t>1) {t=1;s=clamp((b-c)/a);}
    }
  }
  // Test the same two closest points, materializing witnesses only when this
  // edge pair improves the incumbent. Preserve component operation order.
  const ux=p1[0]+d1[0]*s,uy=p1[1]+d1[1]*s,uz=p1[2]+d1[2]*s;
  const vx=p2[0]+d2[0]*t,vy=p2[1]+d2[1]*t,vz=p2[2]+d2[2]*t;
  const dx=ux-vx,dy=uy-vy,dz=uz-vz,distance2=dx*dx+dy*dy+dz*dz;
  if(distance2<bestPair.d2){bestPair.d2=distance2;bestPair.a=[ux,uy,uz];bestPair.b=[vx,vy,vz];}
}
function segmentTriangle(edge,i,face) {
  const a=edge.points[i],tri=face.points,direction=edge.edges[i],e1=face.edges[0],e2=face.ac;
  const h=cross(direction,e2),det=dot(e1,h);
  const scale=Math.sqrt(face.length2[0]*face.ac2*edge.length2[i]);
  if(Math.abs(det)<=1e-13*scale) return null;
  const inv=1/det,s=sub(a,tri[0]),u=inv*dot(s,h);
  if(u< -1e-12||u>1+1e-12) return null;
  const q=cross(s,e1),v=inv*dot(direction,q),t=inv*dot(e2,q);
  if(v< -1e-12||u+v>1+1e-12||t< -1e-12||t>1+1e-12) return null;
  const p=add(a,mul(direction,clamp(t))),facePoint=add(tri[0],add(mul(e1,u),mul(e2,v)));
  // Near-parallel determinants can lose the barycentric digits and claim an
  // intersection beyond a triangle's edge. Verify both parametrizations agree;
  // ill-conditioned cases fall back to point/face and edge/edge distances.
  const errorBound=256*Number.EPSILON*Math.max(1,edge.edgeMax[i],face.max);
  if(norm2(sub(p,facePoint))>errorBound*errorBound)return null;
  return {d2:0,a:p,b:p};
}
// Any separating unit axis provides a lower bound on Euclidean distance.
// Project all vertices, including the roundoff thickness of the first face.
function beyondFaceSlab(face,opponent,limit) {
  const radius=limit*face.axisLength,lo=face.minProjection-radius,hi=face.maxProjection+radius;
  const [x,y,z]=face.points[0],[nx,ny,nz]=face.axis;
  const projection=p=>(p[0]-x)*nx+(p[1]-y)*ny+(p[2]-z)*nz;
  const [a,b,c]=opponent.points,first=projection(a);
  return (first>hi&&projection(b)>hi&&projection(c)>hi)||(first<lo&&projection(b)<lo&&projection(c)<lo);
}
function triangleDistance(itemA,itemB,incumbent) {
  const a=triangleFeatures(itemA),b=triangleFeatures(itemB);
  if(Number.isFinite(incumbent)&&a.quality>1e-9&&b.quality>1e-9){
    const p=a.points[0],q=b.points[0];
    const range=Math.max(Math.abs(p[0]-q[0]),Math.abs(p[1]-q[1]),Math.abs(p[2]-q[2]))+a.longest+b.longest;
    // Allow for the tolerant intersection predicate and for roundoff in
    // the barycentric determinants (products of two edge/point dot terms).
    const margin=1e-7*Math.max(1,a.max,b.max)+4096*Number.EPSILON*range*range*Math.max(a.conditioning,b.conditioning);
    const limit=incumbent+margin;
    if(beyondFaceSlab(a,b,limit)||beyondFaceSlab(b,a,limit))return null;
  }
  // Segment-face intersections are needed when neither endpoint projects inside the other face.
  // Coplanar overlap is covered by point-face and edge-edge distances, including containment.
  for(let k=0;k<3;k++) {
    const hit=segmentTriangle(a,k,b)||segmentTriangle(b,k,a);
    if(hit) return hit;
  }
  const result={d2:Infinity,a:null,b:null};
  for(let k=0;k<3;k++) {
    pointTriangle(a.points[k],b,result,false);
    pointTriangle(b.points[k],a,result,true);
    for(let j=0;j<3;j++)closerEdgePair(a,k,b,j,result);
  }
  return result;
}
function closestSurfaces(a,b,tolerance) {
  let result={d2:Infinity,a:null,b:null},incumbent=Infinity,trianglePairs=0,bvhPairs=0;
  function visit(na,nb,bound) {
    bvhPairs++;
    if(bound>result.d2) return;
    if(na.items&&nb.items) {
      for(const ta of na.items) {
        if(boxDistance2(ta.bounds,nb.bounds)>result.d2)continue;
        for(const tb of nb.items) {
          if(boxDistance2(ta.bounds,tb.bounds)>result.d2) continue;
          trianglePairs++;const candidate=triangleDistance(ta,tb,incumbent);
          if(candidate&&candidate.d2<result.d2){result={...candidate,triangleIds:[ta.index,tb.index]};incumbent=Math.sqrt(candidate.d2);}
          if(result.d2<=tolerance*tolerance) return;
        }
      }
      return;
    }
    const splitB=!nb.items&&(na.items||nb.count>na.count);
    const al=splitB?na:na.left,bl=splitB?nb.left:nb,ar=splitB?na:na.right,br=splitB?nb.right:nb;
    const left=boxDistance2(al.bounds,bl.bounds),right=boxDistance2(ar.bounds,br.bounds);
    if(left<=right){visit(al,bl,left);if(result.d2>tolerance*tolerance)visit(ar,br,right);}
    else{visit(ar,br,right);if(result.d2>tolerance*tolerance)visit(al,bl,left);}
  }
  visit(a.bvh,b.bvh,boxDistance2(a.bvh.bounds,b.bvh.bounds));
  return {...result,trianglePairs,bvhPairs};
}
function clearanceSurfaces(a,b,threshold,ceiling=Infinity) {
  const cutoff=threshold*threshold;
  let lowerBound2=Infinity,witness=null,incumbent=Infinity,trianglePairs=0,bvhPairs=0,hit=false;
  function visit(na,nb,bound) {
    bvhPairs++;
    if(bound>cutoff){lowerBound2=Math.min(lowerBound2,bound);return false;}
    if(na.items&&nb.items) {
      for(const ta of na.items){
        // When every descendant is already outside the margin and beyond
        // the known minimum, it contributes neither a witness nor a tighter bound.
        const leafBound=boxDistance2(ta.bounds,nb.bounds);
        if(leafBound>cutoff&&leafBound>=lowerBound2)continue;
        for(const tb of nb.items){
          const triangleBound=boxDistance2(ta.bounds,tb.bounds);
          if(triangleBound>cutoff){lowerBound2=Math.min(lowerBound2,triangleBound);continue;}
          trianglePairs++;
          let best=incumbent;
          if(ceiling!==Infinity){
            const A=triangleFeatures(ta),B=triangleFeatures(tb);
            // Stay on the ordinary query for tiny, degenerate or non-CAD geometry.
            if(A.degenerate||B.degenerate||!(A.longest>=1e-5&&B.longest>=1e-5&&A.max<=1e6&&B.max<=1e6))return hit=true;
            best=Math.min(ceiling,Math.sqrt(lowerBound2));
          }
          const candidate=triangleDistance(ta,tb,best);
          // An earlier tested pair already bounds both witness and lowerBound2.
          if(!candidate)continue;
          lowerBound2=Math.min(lowerBound2,candidate.d2);
          if(ceiling===Infinity&&(!witness||candidate.d2<witness.d2)){witness={...candidate,triangleIds:[ta.index,tb.index]};incumbent=Math.sqrt(candidate.d2);}
          if(candidate.d2<=cutoff){hit=true;return true;}
        }
      }
      return false;
    }
    const splitB=!nb.items&&(na.items||nb.count>na.count);
    const al=splitB?na:na.left,bl=splitB?nb.left:nb,ar=splitB?na:na.right,br=splitB?nb.right:nb;
    const left=boxDistance2(al.bounds,bl.bounds),right=boxDistance2(ar.bounds,br.bounds);
    return left<=right?(visit(al,bl,left)||visit(ar,br,right)):(visit(ar,br,right)||visit(al,bl,left));
  }
  visit(a.bvh,b.bvh,boxDistance2(a.bvh.bounds,b.bvh.bounds));
  return {hit,lowerBound2,witness,trianglePairs,bvhPairs};
}
// Access search only consumes a clear pair's distance if it lowers the
// minimum across the scene. Return null for the ordinary full-witness query.
export function cappedPairDistance(a,b,clearanceMm,toleranceMm,ceiling){
  const threshold=clearanceMm+toleranceMm;
  if(!(Number.isFinite(ceiling)&&ceiling>threshold))return null;
  const query=clearanceSurfaces(a,b,threshold,ceiling);
  if(query.hit)return null;
  for(const c of a.components)if(insideClosed(c.representative,b,toleranceMm))return null;
  for(const c of b.components)if(insideClosed(c.representative,a,toleranceMm))return null;
  return Math.min(ceiling,Math.sqrt(query.lowerBound2));
}
function rayBox(origin,dir,box) {
  let near=0,far=Infinity;
  for(let k=0;k<3;k++) {
    if(Math.abs(dir[k])<1e-15) {if(origin[k]<box.min[k]||origin[k]>box.max[k]) return false;continue;}
    let a=(box.min[k]-origin[k])/dir[k],b=(box.max[k]-origin[k])/dir[k];
    if(a>b) [a,b]=[b,a];near=Math.max(near,a);far=Math.min(far,b);if(far<near) return false;
  }
  return true;
}
function rayTriangle(origin,direction,tri) {
  const e1=sub(tri[1],tri[0]),e2=sub(tri[2],tri[0]),h=cross(direction,e2),det=dot(e1,h);
  if(Math.abs(det)<=1e-13*Math.sqrt(norm2(e1)*norm2(e2))) return null;
  const inv=1/det,s=sub(origin,tri[0]),u=inv*dot(s,h);
  if(u< -1e-11||u>1+1e-11) return null;
  const q=cross(s,e1),v=inv*dot(direction,q),t=inv*dot(e2,q);
  return v>=-1e-11&&u+v<=1+1e-11&&t>0?t:null;
}
function insideClosed(point,mesh,tolerance) {
  if(!mesh.closed) return false;
  if(point.some((x,k)=>x<mesh.bounds.min[k]||x>mesh.bounds.max[k])) return false;
  // Majority voting avoids a single ray passing exactly through a shared edge/vertex.
  const directions=[[1,.3713906763541037,.6947465906068658],[.238197,1,.531371],[.712319,.193181,1]];
  let votes=0;
  for(const raw of directions) {
    const dir=mul(raw,1/Math.sqrt(norm2(raw))),hits=[];
    function visit(node) {
      if(!rayBox(point,dir,node.bounds)) return;
      if(node.items) {for(const tri of node.items) {const t=rayTriangle(point,dir,tri.p);if(t!==null) hits.push(t);}}
      else {visit(node.left);visit(node.right);}
    }
    visit(mesh.bvh);hits.sort((a,b)=>a-b);
    let count=0,last=-Infinity;
    for(const t of hits) if(t-last>Math.max(tolerance,Math.abs(t)*1e-11)) {count++;last=t;}
    if(count%2) votes++;
  }
  return votes>=2;
}

/**
 * CLEAR means separation greater than requested margin; touching is INTERFERENCE.
 * queryMode='clearance' avoids computing the full nearest distance. Its distanceMm
 * is a conservative lower bound (zero on any interference), NOT an exact clearance.
 */
export function checkPair(a,b,{clearanceMm=.2,toleranceMm=1e-7,queryMode='distance'}={}) {
  if(!a?.bvh||!b?.bvh) throw new TypeError('checkPair requires meshCollider results');
  if(!Number.isFinite(clearanceMm)||clearanceMm<0||!Number.isFinite(toleranceMm)||toleranceMm<0) throw new RangeError('Invalid clearance/tolerance');
  if(!['distance','clearance'].includes(queryMode))throw new RangeError('queryMode must be distance or clearance');
  if(queryMode==='clearance') {
    const query=clearanceSurfaces(a,b,clearanceMm+toleranceMm);
    let contained=null;
    if(!query.hit) {
      for(const c of a.components)if(insideClosed(c.representative,b,toleranceMm)){contained={body:a.id,inside:b.id,point:c.representative};break;}
      if(!contained)for(const c of b.components)if(insideClosed(c.representative,a,toleranceMm)){contained={body:b.id,inside:a.id,point:c.representative};break;}
    }
    const interference=query.hit||!!contained,w=query.witness;
    return {status:interference?'INTERFERENCE':'CLEAR',distanceMm:interference?0:Math.sqrt(query.lowerBound2),
      distanceIsExact:false,distanceBound:'LOWER',surfaceDistanceMm:null,
      witnessedSurfaceDistanceMm:w?Math.sqrt(w.d2):null,bodyIds:[a.id,b.id],
      reason:contained?'CONTAINMENT':query.hit?'WITHIN_CLEARANCE':'SEPARATED',
      point:contained?.point??(w?midpoint(w.a,w.b):null),closestPoints:w?[w.a,w.b]:null,triangleIds:w?.triangleIds??null,
      containment:contained,diagnostics:{trianglePairs:query.trianglePairs,bvhPairs:query.bvhPairs,queryMode,
        containmentSupported:[a.closed,b.closed],clearanceMm,toleranceMm,continuousCollisionDetection:false}};
  }
  const nearest=closestSurfaces(a,b,toleranceMm),surfaceDistanceMm=Math.sqrt(nearest.d2);
  let contained=null;
  if(surfaceDistanceMm>toleranceMm) {
    for(const c of a.components) if(insideClosed(c.representative,b,toleranceMm)) {contained={body:a.id,inside:b.id,point:c.representative};break;}
    if(!contained) for(const c of b.components) if(insideClosed(c.representative,a,toleranceMm)) {contained={body:b.id,inside:a.id,point:c.representative};break;}
  }
  const distanceMm=contained?0:surfaceDistanceMm,interference=distanceMm<=clearanceMm+toleranceMm;
  return {status:interference?'INTERFERENCE':'CLEAR',distanceMm,distanceIsExact:true,surfaceDistanceMm,bodyIds:[a.id,b.id],
    reason:contained?'CONTAINMENT':surfaceDistanceMm<=toleranceMm?'SURFACE_CONTACT_OR_INTERSECTION':interference?'CLEARANCE':'SEPARATED',
    point:contained?.point??midpoint(nearest.a,nearest.b),closestPoints:[nearest.a,nearest.b],triangleIds:nearest.triangleIds,
    containment:contained,diagnostics:{trianglePairs:nearest.trianglePairs,bvhPairs:nearest.bvhPairs,queryMode,
      containmentSupported:[a.closed,b.closed],clearanceMm,toleranceMm,continuousCollisionDetection:false}};
}

/** Return all tested pairs; ignorePairs accepts [[idA,idB],...] or a predicate. */
export function checkScene(colliders,{clearanceMm=.2,toleranceMm=1e-7,queryMode='distance',ignorePairs=[]}={}) {
  const ignored=typeof ignorePairs==='function'?ignorePairs:(a,b)=>ignorePairs.some(p=>(p[0]===a.id&&p[1]===b.id)||(p[0]===b.id&&p[1]===a.id));
  const pairs=[];
  for(let i=0;i<colliders.length;i++) for(let j=i+1;j<colliders.length;j++)
    if(!ignored(colliders[i],colliders[j])) pairs.push(checkPair(colliders[i],colliders[j],{clearanceMm,toleranceMm,queryMode}));
  return {status:pairs.some(p=>p.status==='INTERFERENCE')?'INTERFERENCE':'CLEAR',pairs,
    minDistanceMm:pairs.length?Math.min(...pairs.map(p=>p.distanceMm)):Infinity,distanceIsExact:queryMode==='distance',continuousCollisionDetection:false};
}
/** Private scratch collider for one immutable body within a serial access world.
 * Every witness returned from that world must be detached before the next pose. */
export function rigidColliderRefitter(body){
  let collider,shape,topology,refitBounds;
  const {triangles,leafSize=8}=body;
  const rebuild=transform=>{
    const values=body.positions,posed=new Float64Array(values.length);
    for(let i=0;i<values.length;i+=3)posed.set(transform(values[i],values[i+1],values[i+2]),i);
    collider=meshCollider({...body,positions:posed});
    shape=bvhShapeCache.get(triangles);topology=topologyCache.get(triangles);
    const nodes=[];
    function collect(node){
      if(!node.items){collect(node.left);collect(node.right);}
      nodes.push([node.bounds,node.items??[node.left,node.right]]);
    }
    collect(collider.bvh);
    refitBounds=()=>{for(const [box,children] of nodes){
      for(let k=0;k<3;k++){
        let lo=Infinity,hi=-Infinity;
        for(const child of children){
          const a=child.bounds.min[k],b=child.bounds.max[k];
          if(a<lo)lo=a;if(a>hi)hi=a;
          if(b<lo)lo=b;if(b>hi)hi=b;
        }
        box.min[k]=lo;box.max[k]=hi;
      }
    }};
    return collider;
  };
  return transform=>{
    // A foreign caller can build a different BVH for this triangles object.
    // Follow the ordinary constructor's current partition and welding policy.
    if(!collider||shape!==bvhShapeCache.get(triangles)||topology!==topologyCache.get(triangles)||
       shape?.leafSize!==leafSize)return rebuild(transform);
    const values=body.positions,{positions,bounds:box}=collider;
    for(let k=0;k<3;k++){box.min[k]=Infinity;box.max[k]=-Infinity;}
    for(let i=0;i<positions.length;i++){
      const j=i*3,p=transform(values[j],values[j+1],values[j+2]),stored=positions[i];
      for(let k=0;k<3;k++){
        const v=p[k];if(!Number.isFinite(v))throw new TypeError('Non-finite mesh position');
        stored[k]=v;box.min[k]=Math.min(box.min[k],v);box.max[k]=Math.max(box.max[k],v);
      }
    }
    let degenerate=0;
    for(const item of collider.items){
      const [a,b,c]=item.p;
      for(let k=0;k<3;k++){
        item.bounds.min[k]=Math.min(a[k],b[k],c[k]);item.bounds.max[k]=Math.max(a[k],b[k],c[k]);
        item.center[k]=((a[k]+b[k])+c[k])*(1/3);
      }
      // Vertices are updated in place, so identity alone cannot validate features.
      if(item[featureKey])item[featureKey]=undefined;
      if(degenerateTriangle(a,b,c))degenerate++;
    }
    collider.diagnostics.degenerateTriangles=degenerate;
    collider.closed=collider.diagnostics.closed=!topology.boundaryEdges&&!topology.nonManifoldEdges&&!degenerate;
    refitBounds();
    return collider;
  };
}
