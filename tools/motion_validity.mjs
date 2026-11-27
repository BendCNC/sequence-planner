/** Batch pose validity for rigid part handling between strokes, using the exact mesh detector (viewer/collision.mjs).
 * Scene: JSON from prototypes/ompl-motion/export_scene.mjs (part bodies + obstacle bodies, mm).
 * Pose: [x,y,z,qx,qy,qz,qw]; part vertex p maps to R(q)*(p - part.centroid) + [x,y,z].
 * Usage:
 *   node tools/motion_validity.mjs --scene scene.json [--clearance-mm 1] --poses poses.json   (one batch, prints JSON)
 *   node tools/motion_validity.mjs --scene scene.json [--clearance-mm 1] --stdio              (one JSON request per line)
 * Request: {"poses":[[...7]...],"distance":false}. Reply: {"valid":[bool...],"distanceMm":[number...]|null,"ms":n}.
 * With distance=true each pose also gets the exact minimum part-to-obstacle surface distance (0 when interfering).
 * Geometry only; no machine commands. */
import {readFileSync} from 'node:fs';
import {createInterface} from 'node:readline';
import {performance} from 'node:perf_hooks';
import {fileURLToPath} from 'node:url';
import {resolve} from 'node:path';
import {meshCollider,checkPair} from '../viewer/collision.mjs';

export function loadValidity(scene,{clearanceMm=1}={}){
  const obstacles=scene.obstacles.map(o=>{const c=meshCollider({id:o.id,positions:o.positions,triangles:o.triangles});return{c,box:boxOf(o.positions)};});
  const c0=scene.part.centroid;
  const parts=scene.part.bodies.map(b=>({id:b.id,triangles:b.triangles,local:Float64Array.from(b.positions,(v,i)=>v-c0[i%3])}));
  function check(pose,wantDistance){
    const [x,y,z,qx,qy,qz,qw]=pose,n=Math.hypot(qx,qy,qz,qw)||1,R=rot(qx/n,qy/n,qz/n,qw/n);
    let minD=Infinity;
    for(const p of parts){
      const pos=new Float64Array(p.local.length);
      for(let i=0;i<pos.length;i+=3){const a=p.local[i],b=p.local[i+1],c=p.local[i+2];
        pos[i]=R[0]*a+R[1]*b+R[2]*c+x;pos[i+1]=R[3]*a+R[4]*b+R[5]*c+y;pos[i+2]=R[6]*a+R[7]*b+R[8]*c+z;}
      const box=boxOf(pos);let collider=null;
      for(const o of obstacles){
        const gap=boxGap(box,o.box);
        if(!wantDistance&&gap>clearanceMm+1e-7)continue;
        if(wantDistance&&gap>=minD)continue;
        collider??=meshCollider({id:p.id,positions:pos,triangles:p.triangles});
        const r=checkPair(collider,o.c,{clearanceMm,queryMode:wantDistance?'distance':'clearance'});
        if(r.status==='INTERFERENCE'&&!wantDistance)return{valid:false,d:0};
        minD=Math.min(minD,r.distanceMm);
      }
    }
    return{valid:minD>clearanceMm+1e-7||minD===Infinity,d:minD};
  }
  return function batch({poses,distance=false}){
    const t=performance.now(),res=poses.map(p=>check(p,distance));
    return{valid:res.map(r=>r.valid),distanceMm:distance?res.map(r=>Number.isFinite(r.d)?r.d:null):null,ms:performance.now()-t};
  };
}
function rot(x,y,z,w){return[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w),2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w),2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)];}
function boxOf(p){const min=[Infinity,Infinity,Infinity],max=[-Infinity,-Infinity,-Infinity];
  for(let i=0;i<p.length;i+=3)for(let k=0;k<3;k++){const v=p[i+k];if(v<min[k])min[k]=v;if(v>max[k])max[k]=v;}return{min,max};}
function boxGap(a,b){let d=0;for(let k=0;k<3;k++){const g=Math.max(0,a.min[k]-b.max[k],b.min[k]-a.max[k]);d+=g*g;}return Math.sqrt(d);}

if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url)){
  const arg=k=>{const i=process.argv.indexOf('--'+k);return i>0?process.argv[i+1]:undefined;};
  const scenePath=arg('scene');
  if(!scenePath){process.stderr.write('Use --scene scene.json [--clearance-mm 1] (--poses poses.json | --stdio)\n');process.exit(2);}
  const batch=loadValidity(JSON.parse(readFileSync(scenePath,'utf8')),{clearanceMm:Number(arg('clearance-mm')??1)});
  if(process.argv.includes('--stdio')){
    const rl=createInterface({input:process.stdin,crlfDelay:Infinity});
    for await(const line of rl){if(!line.trim())continue;
      let out;try{out=batch(JSON.parse(line));}catch(e){out={error:e.message};}
      process.stdout.write(JSON.stringify(out)+'\n');}
  }else{
    const req=JSON.parse(readFileSync(arg('poses'),'utf8'));
    process.stdout.write(JSON.stringify(batch(Array.isArray(req)?{poses:req}:req))+'\n');
  }
}
