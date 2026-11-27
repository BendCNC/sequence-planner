import test from 'node:test';
import assert from 'node:assert/strict';
import {evaluateOperatorSupport,prepareRigidFrontProof} from './sequence-operator-support.mjs';

test('rigid front-area proof only accepts poses passing the full material-skin check',()=>{
  const points=[[-5,0,0],[-4,1,0],[-4,0,2],[-1,0,0],[0,1,0],[1,0,2],[3,0,0],[5,1,0],[4,0,2]];
  const triangles=[[0,1,2],[3,4,5],[6,7,8]].map((indices,i)=>({indices,areaMm2:[8,4,2][i]}));
  const body={bodyId:'P',triangles};
  const prepared={majorBranch:{areaMm2:14,panelIds:['P'],bodies:[body]}};
  const scene={bodies:[{id:'P',positions:points.flat()}]},pivot=[1,0,-2];
  const proof=prepareRigidFrontProof(prepared,new Map([['P',{positions:points}]]),pivot);
  let certified=0,failed=0;
  for(let a=-1.75;a<1.75;a+=.07)for(let x=-8;x<=8;x+=.25){
    const c=Math.cos(a),s=Math.sin(a),rounding=1e-10;
    const transformPoint=([u,y,v])=>[pivot[0]+c*(u-pivot[0])+s*(v-pivot[2])+x,y,pivot[2]-s*(u-pivot[0])+c*(v-pivot[2])];
    const actual=evaluateOperatorSupport(prepared,scene,{transformPoint});
    if(proof(c,s,x,rounding)){assert.equal(actual.passes,true);certified++;}
    if(!actual.passes)failed++;
  }
  assert.ok(certified>500&&failed>500);
  const collapsed=points.map((v,i)=>i===2?points[0]:v);
  assert.equal(prepareRigidFrontProof(prepared,new Map([['P',{positions:collapsed}]]),pivot)(1,0,-10,1e-10),false);
});

