import test from 'node:test';
import assert from 'node:assert/strict';
import {clipSurfaceOutsideJointBox} from './sequence-joint-collision.mjs';
import {meshCollider,checkPair} from './collision.mjs';

test('indexing a clipped joint keeps face order, first contact and exact bounds',()=>{
  const points=[[-2,-1,-0],[-2,2,0],[2,-1,0],[2,2,0],[-2,-1,0],[-2,-1+1e-16,0]];
  const body={id:'panel',positions:Float64Array.from(points.flat()),triangles:[0,1,2,2,1,3,4,0,5]};
  const obstacle=meshCollider({id:'flange',positions:[[-1,-2,.15],[-1,2,.15],[-1,0,-.15]],triangles:[[0,1,2]]});
  const expand=patch=>patch.triangles.map(i=>Array.from(patch.positions.slice(3*i,3*i+3)));
  const c=Math.cos(.34),s=Math.sin(.34),frames=[p=>p,([x,y,z])=>[c*x+s*y,-s*x+c*y,z]];
  for(const toLocal of frames)for(const lo of [[-.25,-.5,-1],[3,3,3]]){
    const hi=lo.map(x=>x+.75);
    const complete=clipSurfaceOutsideJointBox(body,toLocal,lo,hi);
    const indexed=clipSurfaceOutsideJointBox(body,toLocal,lo,hi,true);
    assert.deepEqual(expand(indexed),expand(complete));
    assert.ok(indexed.positions.length<complete.positions.length);
    for(const queryMode of ['clearance','distance']){
      const options={queryMode,clearanceMm:.15};
      assert.deepEqual(checkPair(meshCollider(indexed),obstacle,options),checkPair(meshCollider(complete),obstacle,options));
    }
  }
});
