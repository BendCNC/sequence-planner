import encoded from './clearance-wasm.mjs';
import {positionBounds} from './tool-geometry.mjs';

let module;
function instance(){
  module??=new WebAssembly.Module(Uint8Array.from(atob(encoded),c=>c.charCodeAt(0)));
  return new WebAssembly.Instance(module).exports;
}

export function punchHeightKernel(polygons,envelope,pivot,c,s){
  try{
    const kernel=instance(),first=Math.ceil(kernel.__heap_base.value/8)*8;
    const limits=new Uint32Array(polygons.length);
    let count=0;
    for(let i=0;i<polygons.length;i++){count+=polygons[i].vertices.length;limits[i]=count;}
    const ranges=first+32*count,prof=ranges+16*Math.ceil(count/16),end=prof+48*envelope.length;
    const pages=Math.ceil((end+32-kernel.memory.buffer.byteLength)/65536);
    if(pages>0)kernel.memory.grow(pages);
    const edges=new Float64Array(kernel.memory.buffer,first,4*count);
    let offset=0;
    for(const {vertices} of polygons){
      const start=offset;
      for(const p of vertices){
        const x=p[0]-pivot[0],z=p[2]-pivot[2];
        const px=pivot[0]+c*x+s*z,pz=pivot[2]-s*x+c*z;
        edges[offset]=px;edges[offset+1]=pz;
        if(offset>start){edges[offset-2]=px;edges[offset-1]=pz;}
        offset+=4;
      }
      if(offset>start){edges[offset-2]=edges[start];edges[offset-1]=edges[start+1];}
    }
    kernel.tip_blocks(first,count,ranges);
    new Float64Array(kernel.memory.buffer,prof,6*envelope.length).set(envelope.flatMap(s=>
      [s.minX,s.maxX,s.m??0,s.c??0,s.z??0,s.vertical?1:0]));
    const result=new Float64Array(kernel.memory.buffer,end,4);
    return (x,z,maximum)=>{
      if(!kernel.required_tip(first,count,ranges,prof,envelope.length,x,z,maximum,end))return null;
      const edge=result[1];
      let lo=0,hi=limits.length-1;
      while(lo<hi){const mid=(lo+hi)>>>1;if(limits[mid]<=edge)lo=mid+1;else hi=mid;}
      const polygon=polygons[lo],vertices=polygon.vertices,index=edge-(lo?limits[lo-1]:0),at=4*edge;
      const a=[edges[at],vertices[index][1],edges[at+1]],b=[edges[at+2],vertices[(index+1)%vertices.length][1],edges[at+3]];
      return {a,b,polygon,shadow:envelope[result[2]],x:result[3],height:result[0]};
    };
  }catch{return null;}
}
// The arena is private to this synchronous query batch. Workers and nested
// callers never share mutable geometry, and a new plan owns a fresh batch.
export function meshSeparationQuery(){
  let kernel,offset,disabled=false;
  const meshes=new WeakMap(),boxes=new WeakMap();
  function boxOf(body){
    if(boxes.has(body))return boxes.get(body);
    const p=body.positions,t=body.triangles;
    if(!p?.length||typeof p[0]!=='number'||p.length%3||!t?.length||t.length%3||
        !t.every(i=>Number.isInteger(i)&&i>=0&&i<p.length/3))return null;
    const box=positionBounds(p);
    box.scale=Math.max(1,...box.min.map(Math.abs),...box.max.map(Math.abs));
    if(!Number.isFinite(box.scale))return null;
    boxes.set(body,box);return box;
  }
  function prepare(body){
    if(meshes.has(body))return meshes.get(body);
    if(!kernel){
      kernel=instance();
      offset=kernel.__heap_base.value;
    }
    const p=body.positions,t=body.triangles,n=p.length/3,f=t.length/3;
    if(!Number.isInteger(n)||!Number.isInteger(f)||n<3||f<1||n>150000||f>150000||
        typeof p[0]!=='number')return 0;
    const ptr=Math.ceil(offset/8)*8,end=ptr+kernel.mesh_bytes(n,f);
    if(end>32*1024*1024)return 0;
    const pages=Math.ceil((end-kernel.memory.buffer.byteLength)/65536);
    if(pages>0){
      const current=kernel.memory.buffer.byteLength/65536;
      kernel.memory.grow(Math.min(512-current,Math.max(pages,current,8)));
    }
    const start=ptr+kernel.vertex_offset();
    new Float64Array(kernel.memory.buffer,start,3*n).set(p);
    new Int32Array(kernel.memory.buffer,start+24*n,3*f).set(t);
    const mesh=kernel.prepare(ptr,n,f)?ptr:0;
    if(mesh)offset=end;
    meshes.set(body,mesh);
    return mesh;
  }
  return (a,b,x,y,z,clearance,dx=0,dy=0,dz=0)=>{
    if(disabled||!Number.isFinite(clearance)||clearance<0||![x,y,z,dx,dy,dz].every(Number.isFinite))return null;
    try{
      const one=boxOf(a),two=boxOf(b);
      if(!one||!two)return null;
      {
        const axis=(k,start,travel)=>Math.max(one.min[k]+(start+Math.min(0,travel))-two.max[k],
          two.min[k]-(one.max[k]+(start+Math.max(0,travel))));
        const pad=1e-10*(one.scale+two.scale+1+Math.abs(x)+Math.abs(y)+Math.abs(z)+Math.abs(dx)+Math.abs(dy)+Math.abs(dz));
        const gap=Math.max(axis(0,x,dx),axis(1,y,dy),axis(2,z,dz))-pad;
        if(gap>clearance)return gap;
      }
      const ma=prepare(a),mb=prepare(b);
      if(!ma||!mb)return null;
      const gap=kernel.separation(ma,mb,x,y,z,clearance,dx,dy,dz);
      return gap>clearance?gap:null;
    }catch{
      // A runtime without WebAssembly or available memory uses the full query.
      disabled=true;return null;
    }
  };
}
