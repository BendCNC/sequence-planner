import {movesWithRam} from './tool-geometry.mjs';
/** Search accelerator only. Axial projection can reject a valid 3-D pose; it
 * cannot approve a route. Every result is rechecked with original solid meshes. */
const cross=(a,b,c)=>(b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]);
const box=([a,b,c])=>({lo:[Math.min(a[0],b[0],c[0]),Math.min(a[1],b[1],c[1])],
                     hi:[Math.max(a[0],b[0],c[0]),Math.max(a[1],b[1],c[1])]});
const apart=(a,b,m)=>a.lo[0]>b.hi[0]+m||b.lo[0]>a.hi[0]+m||a.lo[1]>b.hi[1]+m||b.lo[1]>a.hi[1]+m;
function tree(items){const bounds={lo:[0,1].map(k=>Math.min(...items.map(t=>t.bounds.lo[k]))),hi:[0,1].map(k=>Math.max(...items.map(t=>t.bounds.hi[k])))};
 if(items.length<=8)return{bounds,items};const k=bounds.hi[0]-bounds.lo[0]>bounds.hi[1]-bounds.lo[1]?0:1;
 items.sort((a,b)=>a.bounds.lo[k]+a.bounds.hi[k]-b.bounds.lo[k]-b.bounds.hi[k]);const n=items.length>>1;return{bounds,left:tree(items.slice(0,n)),right:tree(items.slice(n))};}
function clip(poly,y,lower){const out=[];for(let i=0;i<poly.length;i++){const a=poly[i],b=poly[(i+1)%poly.length],ia=lower?a[1]>=y:a[1]<=y,ib=lower?b[1]>=y:b[1]<=y;if(ia)out.push(a);if(ia!==ib){const t=(y-a[1])/(b[1]-a[1]);out.push(a.map((v,k)=>v+t*(b[k]-v)));}}return out;}
function projected(bodies,lo=-Infinity,hi=Infinity){
 const seen=new Set(),items=[];
 for(const body of bodies){
  const coords=body.positions,index=body.triangles,vertices=new Array(coords.length/3);
  function vertex(i){
   const k=3*i;
   return vertices[i]??=[coords[k],coords[k+1],coords[k+2]];
  }
  for(let i=0;i<index.length;i+=3){
   const a=index[i],b=index[i+1],c=index[i+2];
   const ya=coords[3*a+1],yb=coords[3*b+1],yc=coords[3*c+1];
   if((ya<lo&&yb<lo&&yc<lo)||(ya>hi&&yb>hi&&yc>hi))continue;
   let poly=[vertex(a),vertex(b),vertex(c)];
   const crossesLow=Number.isFinite(lo)&&!(ya>=lo&&yb>=lo&&yc>=lo);
   if(crossesLow)poly=clip(poly,lo,true);
   if(poly.length&&Number.isFinite(hi)&&(crossesLow||!(ya<=hi&&yb<=hi&&yc<=hi)))poly=clip(poly,hi,false);
   for(let j=1;j<poly.length-1;j++){
    const tri=[poly[0],poly[j],poly[j+1]].map(v=>[v[0],v[2]]);
    if(Math.abs(cross(...tri))<1e-12)continue;
    const key=tri.map(v=>v.map(n=>Math.round(n*1e8)).join(',')).sort().join('|');
    if(seen.has(key))continue;
    seen.add(key);items.push({p:tri,bounds:box(tri)});
   }
  }
 }
 return items;
}
function axesFor(p){
 const axes=[];
 for(let i=0;i<3;i++){
  const a=p[i],b=p[(i+1)%3],x=b[1]-a[1],y=a[0]-b[0],norm=Math.hypot(x,y);
  if(norm<1e-12)continue;
  const nx=x/norm,ny=y/norm,d0=p[0][0]*nx+p[0][1]*ny,d1=p[1][0]*nx+p[1][1]*ny,d2=p[2][0]*nx+p[2][1]*ny;
  axes.push({nx,ny,min:Math.min(d0,d1,d2),max:Math.max(d0,d1,d2)});
 }
 return axes;
}
function satNear(a,b,ba,margin){
 for(const [axes,points]of [[a.axes,b],[ba,a.p]])for(const axis of axes){const d0=points[0][0]*axis.nx+points[0][1]*axis.ny,d1=points[1][0]*axis.nx+points[1][1]*axis.ny,d2=points[2][0]*axis.nx+points[2][1]*axis.ny;
  if(Math.min(d0,d1,d2)>axis.max+margin||Math.max(d0,d1,d2)<axis.min-margin)return false;}
 return true;
}

// Fraction of a triangle on the front (X <= 0) side of the station plane.
function frontFraction(a,b,c){
 if(a<=0&&b<=0&&c<=0)return 1;
 if(a>=0&&b>=0&&c>=0)return 0;
 let tmp;
 if(a>b){tmp=a;a=b;b=tmp;}
 if(b>c){tmp=b;b=c;c=tmp;}
 if(a>b){tmp=a;a=b;b=tmp;}
 if(c<=0)return 1;
 if(a<0&&b>=0)return a*a/((b-a)*(c-a));
 if(b<0)return 1-c*c/((c-a)*(c-b));
 return 0;
}

// Projected triangles often share all three vertices with neighbouring facets.
// Reuse the exact transformed coordinates within a pose (including signed zero).
function indexedVertices(triangles){
 const vertices=[],indices=new Map();
 const faces=triangles.map(triangle=>triangle.p.map(point=>{
  const key=point.map(v=>Object.is(v,-0)?'-0':v).join('|');
  let i=indices.get(key);if(i===undefined){i=vertices.length;indices.set(key,i);vertices.push(point);}return i;
 }));
 return{vertices,faces};
}
export function createAccessProjection({scene,pivot,preparedSupport,clearanceMm=.15}){
 const parts=scene.bodies.filter(b=>b.kind==='panel'||b.kind==='band'),obstacles=scene.bodies.filter(b=>!parts.includes(b)),pairs=[];
 // Several tool bodies may occupy the same axial span. They see the same
 // stationary projected part for this placement.
 const trees=new Map(),negativeZero=Symbol();
 const spanKey=value=>Object.is(value,-0)?negativeZero:value;
 function partTree(lo,hi){
  const lowKey=spanKey(lo),highKey=spanKey(hi);
  let upper=trees.get(lowKey);
  if(!upper){upper=new Map();trees.set(lowKey,upper);}
  if(upper.has(highKey))return upper.get(highKey);
  const faces=projected(parts,lo,hi);
  for(const f of faces)f.axes=axesFor(f.p);
  const part=faces.length?tree(faces):null;
  upper.set(highKey,part);
  return part;
 }
 for(const body of obstacles){let lo=Infinity,hi=-Infinity;for(let i=1;i<body.positions.length;i+=3){lo=Math.min(lo,body.positions[i]);hi=Math.max(hi,body.positions[i]);}
  const part=partTree(lo-clearanceMm-1e-7,hi+clearanceMm+1e-7),o=projected([body]);
  if(part&&o.length){
   const {vertices,faces}=indexedVertices(o),posed=vertices.map(()=>[0,0]);
   pairs.push({movesWithRam:movesWithRam(body),parts:part,vertices,posed,triangles:faces.map(face=>face.map(i=>posed[i]))});
  }}
 const byId=new Map(parts.map(b=>[b.id,b])),support=[];let area=0,cx=0,cz=0;
 for(const body of preparedSupport.majorBranch.bodies)for(const tri of body.triangles){const b=byId.get(body.bodyId),p=tri.indices.map(i=>[b.positions[i*3],b.positions[i*3+2]]),a=tri.areaMm2;support.push({p,a});area+=a;cx+=p.reduce((s,p)=>s+p[0],0)*a/3;cz+=p.reduce((s,p)=>s+p[1],0)*a/3;}
 cx/=area;cz/=area;
 const supportMesh=indexedVertices(support),supportX=new Float64Array(supportMesh.vertices.length);
 const margin=clearanceMm+1e-7;
 function clear(pose){const angle=pose.tiltDeg*Math.PI/180,c=Math.cos(angle),s=Math.sin(angle),px=pivot[0],pz=pivot[2],x=pose.x,z=pose.z,ram=pose.ramGapMm;
  const tx=v=>px+c*(v[0]-px)+s*(v[1]-pz)+x;
  if(tx([cx,cz])>=-1e-7)return false;
  for(const pair of pairs){
   const lift=pair.movesWithRam?ram:0;
   let minX=Infinity,minZ=Infinity,maxX=-Infinity,maxZ=-Infinity;
   for(let i=0;i<pair.vertices.length;i++){
    const v=pair.vertices[i],out=pair.posed[i],a=v[0]-px-x,b=v[1]+lift-pz-z;
    out[0]=px+c*a-s*b;out[1]=pz+s*a+c*b;
    minX=Math.min(minX,out[0]);maxX=Math.max(maxX,out[0]);minZ=Math.min(minZ,out[1]);maxZ=Math.max(maxZ,out[1]);
   }
   if(apart(pair.parts.bounds,{lo:[minX,minZ],hi:[maxX,maxZ]},margin))continue;
   for(const tri of pair.triangles){
    const bounds=box(tri);let axes;
    function hit(n){
     if(apart(n.bounds,bounds,margin))return false;
     if(!n.items)return hit(n.left)||hit(n.right);
     for(const t of n.items)if(!apart(t.bounds,bounds,margin)&&satNear(t,tri,axes??=axesFor(tri),margin))return true;
     return false;
    }
    if(hit(pair.parts))return false;
   }
  }
  let maximum=-Infinity;
  for(let i=0;i<supportX.length;i++){
   const v=tx(supportMesh.vertices[i]);supportX[i]=v;maximum=Math.max(maximum,v);
  }
  if(maximum<=0)return area>0&&Number.isFinite(area);
  let front=0;
  for(let i=0;i<support.length;i++){
   const face=supportMesh.faces[i];
   front+=support[i].a*frontFraction(supportX[face[0]],supportX[face[1]],supportX[face[2]]);
  }
  return front/area>.5+1e-9;
 }
 return{clear,pairCount:pairs.length,triangleCount:pairs.reduce((s,p)=>s+p.triangles.length,0),searchOnly:true};
}
