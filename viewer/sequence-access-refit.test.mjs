import test from 'node:test';
import assert from 'node:assert/strict';
import {meshCollider,checkPair,rigidColliderRefitter} from './collision.mjs';
import {buildPrism} from './tool-geometry.mjs';

test('refitting a moving closed body preserves exact distances, triangle witnesses and changed degeneracy',()=>{
  const profile={id:'square',status:'ready',profile:[[-1,-1],[1,-1],[1,1],[-1,1]],triangles2D:[0,1,2,0,2,3]};
  const body=buildPrism(profile,2,{centerY:0}),fixed=meshCollider({...buildPrism(profile,.25,{centerY:0}),id:'fixed'});
  const refit=rigidColliderRefitter(body);
  for(const [angle,x] of [[0,3],[Math.PI/6,.25],[0,2.15],[0,2.1500001],[0,2**55],[0,-1.5],[0,3]]){
    const c=Math.cos(angle),s=Math.sin(angle),transform=(a,b,z)=>[c*a+s*z+x,b,-s*a+c*z];
    const positions=new Float64Array(body.positions.length);
    for(let i=0;i<positions.length;i+=3)positions.set(transform(...body.positions.slice(i,i+3)),i);
    const ordinary=meshCollider({...body,positions}),moving=refit(transform);
    assert.deepEqual(moving.diagnostics,ordinary.diagnostics);
    if(x===2**55)assert.ok(moving.diagnostics.degenerateTriangles>0);
    for(const queryMode of ['clearance','distance']){
      const options={queryMode,clearanceMm:.15};
      assert.deepEqual(checkPair(moving,fixed,options),checkPair(ordinary,fixed,options));
    }
  }
});
