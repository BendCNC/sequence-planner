import test from 'node:test';
import assert from 'node:assert/strict';
import {buildPrism} from './tool-geometry.mjs';
import {meshCollider,checkPair} from './collision.mjs';
import {meshSeparationQuery} from './mesh-separation.mjs';
import {preparePunchShadow,preparePartForPunch,createRequiredTipEvaluator} from './sequence-retract-geometry.mjs';

function prism(x,z,w,h,y=0,length=1){
  const tool={id:'section',status:'ready',profile:[[x,z],[x+w,z],[x+w,z+h],[x,z+h]],triangles2D:[0,1,2,0,2,3]};
  return buildPrism(tool,length,{centerY:y});
}
function moved(mesh,x,y,z){
  const d=[x,y,z];
  return {...mesh,positions:Float64Array.from(mesh.positions,(value,i)=>value+d[i%3])};
}
function combined(a,b){
  const n=a.positions.length/3;
  return {positions:[...a.positions,...b.positions],triangles:[...a.triangles,...b.triangles.map(i=>i+n)]};
}
test('clear endpoints cannot certify a sweep through a thin obstacle',()=>{
  const part=prism(-4,0,.5,.5),wall=prism(0,-1,.01,3),query=meshSeparationQuery();
  for(const shift of [0,8])assert.ok(query(part,wall,shift,0,0,.1)>.1);
  assert.equal(query(part,wall,0,0,0,.1,8,0,0),null);
  assert.equal(query(part,wall,8,0,0,.1,-8,0,0),null);
  assert.ok(query(part,wall,0,0,4,.1,8,0,0)>.1);
});
test('containment, tangency and every disconnected component retain the exact witness path',()=>{
  const outer=prism(-1,-1,2,2,0,2),inside=prism(-.2,-.2,.4,.4,0,.4),far=prism(8,8,1,1),query=meshSeparationQuery();
  for(const mesh of [inside,combined(far,inside),combined(inside,far),prism(1,0,2,1),prism(1.05,0,2,1)]){
    assert.equal(query(mesh,outer,0,0,0,.05),null);
    assert.equal(query(outer,mesh,0,0,0,.05),null);
  }
  assert.equal(query({...far,triangles:[0,1,999999]},outer,0,0,0,.01),null);
});
test('a separating certificate agrees with the independent solid solver along translated oblique paths',()=>{
  const tilted=(mesh,r)=>({...mesh,positions:Float64Array.from(mesh.positions,(v,i)=>{
    const j=i-i%3,x=mesh.positions[j],z=mesh.positions[j+2];
    return i%3===0?Math.cos(r)*x+Math.sin(r)*z:i%3===2?-Math.sin(r)*x+Math.cos(r)*z:v;
  })});
  const a=tilted(prism(-3,0,6,.1),.65),b=tilted(prism(-2,0,4,.12),.65),fixed=meshCollider(b);
  const query=meshSeparationQuery();let certificates=0;
  for(let i=0;i<25;i++){
    const x=(i-12)*.07,z=1+i*.012,dx=.5-i*.03,dz=(i%3-1)*.25;
    const gap=query(a,b,x,0,z,.15,dx,0,dz);
    if(gap===null)continue;
    certificates++;
    for(let j=0;j<=6;j++){
      const check=checkPair(meshCollider(moved(a,x+dx*j/6,0,z+dz*j/6)),fixed,{clearanceMm:.15,queryMode:'clearance'});
      assert.equal(check.status,'CLEAR');
    }
  }
  assert.ok(certificates>=12);
});
test('portable punch evaluation preserves the JS maximum and the first exact witness on ties',()=>{
  const part=combined(prism(-2,.04,4,.15),prism(-2,.04,4,.15,.3));
  const tool={id:'p',profile:[[-1,2],[0,0],[1,2],[1,3],[-1,3]]};
  const shadow=preparePunchShadow(tool,{id:'p1',lengthMm:4,centerY:0},{clearanceMm:.15});
  const prepared=preparePartForPunch([{...part,id:'part'}],shadow),pivot=[.5,.7,.1];
  for(const tiltDeg of [0,21,-47]){
    const poses=[-4,-1,0,.3,1,4].map(x=>({x,z:1+.1*x,tiltDeg}));
    const accelerated=createRequiredTipEvaluator(prepared,shadow,pivot),expected=createRequiredTipEvaluator(prepared,shadow,pivot);
    const actual=poses.map(p=>accelerated(p));const saved=globalThis.WebAssembly;
    try{globalThis.WebAssembly=undefined;assert.deepEqual(actual,poses.map(p=>expected(p)));}
    finally{globalThis.WebAssembly=saved;}
  }
});
