import {unprotect} from './protected-json.mjs';
import {test} from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {buildFingerBodies} from './finger-geometry.mjs';
import {signedMeshVolume} from './tool-geometry.mjs';

const catalog=unprotect(readFileSync(new URL('./fingers.json.bct',import.meta.url)));
const definition=catalog.definitions[0];
const close=(a,b,tol=1e-8)=>assert.ok(Math.abs(a-b)<tol,`${a} != ${b}`);
function closed(mesh){
  const edges=new Map();
  for(let i=0;i<mesh.triangles.length;i+=3){const t=mesh.triangles.slice(i,i+3);
    for(let j=0;j<3;j++){const a=t[j],b=t[(j+1)%3],key=[Math.min(a,b),Math.max(a,b)].join(',');
      const e=edges.get(key)??[0,0];e[0]++;e[1]+=a<b?1:-1;edges.set(key,e);}}
  for(const value of edges.values())assert.deepEqual(value,[2,0]);
}

test('both fingers remain obstacles regardless of a single-contact requirement',()=>{
  const bodies=buildFingerBodies({definition,lowerFaceX:140,baseZ:-6.35,stationY:[-80,75],requiredContactCount:1});
  assert.equal(bodies.length,2);assert.deepEqual(bodies.map(b=>b.id),['finger-left','finger-right']);
  assert.ok(bodies.every(b=>b.kind==='finger'&&b.collisionEnvelope));
  assert.equal(catalog.contactRequirement.nonContactedFingerRemainsAnObstacle,true);
});

test('translation preserves every vertex, closure, volume and separate station position',()=>{
  for(const collision of [false,true]){
    const left=collision?definition.collisionMesh:definition.mesh;
    const right=(collision?definition.rightCollisionMesh:definition.rightMesh)??left;
    const bodies=buildFingerBodies({definition,lowerFaceX:101.2,baseZ:-17,stationY:[-57,93],collision});
    for(const [index,body] of bodies.entries()){
      const source=index===1?right:left;
      closed(body);close(signedMeshVolume(body),source.volumeMm3,1e-6);
      for(let i=0;i<body.positions.length;i++)close(body.positions[i],source.positions[i]+[101.2,[-57,93][index],-17][i%3]);
      assert.notEqual(body.triangles,source.triangles);assert.deepEqual(body.triangles,source.triangles);
      for(let axis=0;axis<3;axis++)close(body.bounds.size[axis],source.bounds.size[axis]);
    }
  }
});

test('lower/upper stop references and upper support surface follow common X/R and individual stations',()=>{
  const [a,b]=buildFingerBodies({definition,lowerFaceX:120,baseZ:-9.525,stationY:[-40,60],collision:false});
  for(const body of [a,b]){
    close(body.contactFeatures.lower.planeX,120);close(body.contactFeatures.upper.planeX,120+definition.contactFeatures.upper.planeX);
    close(body.contactFeatures.upper.supportSurfaceZ,0);assert.equal(body.contactFeatures.lower.supportSurfaceZ,null);
    close(body.contactFeatures.lower.minZ,-9.525);close(body.contactFeatures.upper.maxZ,9.525);
    close((body.contactFeatures.upper.minY+body.contactFeatures.upper.maxY)/2,body.placement.stationY);
  }
  close(b.contactFeatures.upper.minY-a.contactFeatures.upper.minY,100);
});

test('original STEP finger: right finger is the Y mirror of the left (tongue tips inboard)',()=>{
  if(!definition.rightMesh)return;
  const l=definition.mesh.positions,r=definition.rightMesh.positions;
  assert.equal(l.length,r.length);
  for(let i=0;i<l.length;i+=3){close(r[i],l[i]);close(r[i+1],-l[i+1]);close(r[i+2],l[i+2]);}
  close(definition.rightMesh.volumeMm3,definition.mesh.volumeMm3,1e-6);
  assert.ok(definition.contactFeatures.lowerTip.centerY>0);
});

test('unknown station spacing and nonfinite poses are rejected, never defaulted',()=>{
  const valid={definition,lowerFaceX:100,baseZ:0,stationY:[-50,50]};
  for(const stationY of [undefined,[],[NaN],[0,1,2],[0,NaN],[0,Infinity],[null,1]])
    assert.throws(()=>buildFingerBodies({...valid,stationY}));
  assert.equal(buildFingerBodies({...valid,stationY:[0]}).length,1);
  for(const lowerFaceX of [undefined,null,NaN,Infinity,'100'])assert.throws(()=>buildFingerBodies({...valid,lowerFaceX}));
  assert.throws(()=>buildFingerBodies({...valid,baseZ:NaN}));
  assert.throws(()=>buildFingerBodies({...valid,definition:{id:'bad'}}));
  assert.throws(()=>buildFingerBodies({...valid,definition:{...definition,collisionMesh:{...definition.collisionMesh,triangles:[]}}}));
  assert.equal(catalog.travel.xAbsoluteLimitsMm,null);assert.equal(catalog.travel.rAbsoluteLimitsMm,null);
});
