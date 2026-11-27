/** Tool profiles are [x,z] mm; positive y runs along the machine bed. No renderer dependency. */
export const movesWithRam = body => body.kind === 'punch' || body.kind === 'ram';

export function toolById(catalog,id) {
  const index=catalog.byId?.[id];
  return Number.isInteger(index)?catalog.tools[index]:catalog.tools.find(tool=>tool.id===id)??null;
}

function number(value,fallback,name) {
  const n=value===undefined?fallback:Number(value);
  if(!Number.isFinite(n))throw new TypeError(`${name} must be finite.`);
  return n;
}

export function profileArea(tool) {
  const points=tool.vertices2D??tool.profile;
  const loops=tool.loops??[points.map((_,i)=>i)];
  let area=0;
  for(const loop of loops)for(let i=0;i<loop.length;i++){
    const a=points[loop[i]],b=points[loop[(i+1)%loop.length]];
    area+=(a[0]*b[1]-b[0]*a[1])/2;
  }
  return area;
}

export function positionBounds(positions) {
  let xmin=Infinity,ymin=Infinity,zmin=Infinity,xmax=-Infinity,ymax=-Infinity,zmax=-Infinity;
  for(let i=0;i<positions.length;i+=3){
    const x=positions[i],y=positions[i+1],z=positions[i+2];
    xmin=Math.min(xmin,x);ymin=Math.min(ymin,y);zmin=Math.min(zmin,z);
    xmax=Math.max(xmax,x);ymax=Math.max(ymax,y);zmax=Math.max(zmax,z);
  }
  return {min:[xmin,ymin,zmin],max:[xmax,ymax,zmax]};
}
export function meshBounds(positions) {
  const {min,max}=positionBounds(positions);
  return {min,max,size:max.map((v,i)=>v-min[i])};
}

/**
 * Extrude an already oriented profile, preserving holes and original scale.
 * Options: centerY, offsetX, offsetZ (mm), rotationYRad, mirrorX.
 * rotationYRad is a physical right-handed rotation around +y. Face orientation
 * from the DXF is already included in each die's profile: do not apply it twice.
 */
export function buildPrism(tool,lengthMm,{centerY=0,offsetX=0,offsetZ=0,rotationYRad=0,mirrorX=false}={}) {
  if(!tool||tool.status!=='ready')throw new Error(`Tool geometry unavailable: ${tool?.id??'unknown'}`);
  const length=number(lengthMm,NaN,'lengthMm');
  if(length<=0)throw new RangeError('lengthMm must be greater than zero.');
  centerY=number(centerY,0,'centerY');offsetX=number(offsetX,0,'offsetX');offsetZ=number(offsetZ,0,'offsetZ');
  rotationYRad=number(rotationYRad,0,'rotationYRad');
  const vertices=tool.vertices2D??tool.profile,loops=tool.loops??[vertices.map((_,i)=>i)];
  if(vertices.length<3||!tool.triangles2D?.length)throw new Error('Profile triangulation is empty.');
  const count=vertices.length,positions=new Float64Array(count*6),triangles=[];
  const c=Math.cos(rotationYRad),s=Math.sin(rotationYRad),mirror=mirrorX?-1:1;
  for(let side=0;side<2;side++)for(let i=0;i<count;i++){
    const x=vertices[i][0]*mirror,z=vertices[i][1],at=(side*count+i)*3;
    positions[at]=c*x+s*z+offsetX;
    positions[at+1]=centerY+(side===0?-length/2:length/2);
    positions[at+2]=-s*x+c*z+offsetZ;
  }
  // CCW triangles in (x,z) face -y at the first cap.
  for(let i=0;i<tool.triangles2D.length;i+=3){
    const [a,b,c]=tool.triangles2D.slice(i,i+3);
    triangles.push(a,b,c,a+count,c+count,b+count);
  }
  for(const loop of loops)for(let i=0;i<loop.length;i++){
    const a=loop[i],b=loop[(i+1)%loop.length];
    triangles.push(a,a+count,b+count,a,b+count,b);
  }
  if(mirrorX)for(let i=0;i<triangles.length;i+=3)[triangles[i+1],triangles[i+2]]=[triangles[i+2],triangles[i+1]];
  return {positions,triangles,toolId:tool.id,lengthMm:length,bounds:meshBounds(positions),
    placement:{centerY,offsetX,offsetZ,rotationYRad,mirrorX:Boolean(mirrorX)}};
}

export function signedMeshVolume(mesh) {
  const bounds=meshBounds(mesh.positions),center=bounds.min.map((v,i)=>(v+bounds.max[i])/2);
  let volume=0;
  const point=index=>[0,1,2].map(i=>mesh.positions[index*3+i]-center[i]);
  for(let i=0;i<mesh.triangles.length;i+=3){
    const a=point(mesh.triangles[i]),b=point(mesh.triangles[i+1]),c=point(mesh.triangles[i+2]);
    volume+=(a[0]*(b[1]*c[2]-b[2]*c[1])+a[1]*(b[2]*c[0]-b[0]*c[2])+a[2]*(b[0]*c[1]-b[1]*c[0]))/6;
  }
  return volume;
}
