import test from 'node:test';
import assert from 'node:assert/strict';
import {checkRecordedOperatorRoute} from './sequence-route-check.mjs';
import {buildOperatorSupport,evaluateOperatorSupport} from './sequence-operator-support.mjs';
import {transformFrontTiltPoint} from './sequence-rigid-pose.mjs';

const triangle=(id,kind,coordinates)=>({
  id,kind,positions:new Float64Array(coordinates.flat()),triangles:new Uint32Array([0,1,2]),
});

function straightRoute(obstacleHeight,pivot=[.375,0,8]){
  const main=triangle('panel:main','panel',[[-3,-1,.03125],[-1,-1,.03125],[-3,1,.03125]]);
  const tip=triangle('panel:tip','panel',[[-1.25,3,0],[-1,3,0],[-1.25,3.25,0]]);
  const die=triangle('die','die',[[-8,-1,obstacleHeight],[-4,-1,obstacleHeight],[-8,1,obstacleHeight]]);
  const panels=[main,tip].map(b=>({id:b.id.slice(6),flatPositions:b.positions,triangles:b.triangles}));
  const model={thicknessMm:1,panels,bends:[{id:'bend',parentPanelId:'main',childPanelId:'tip'}]};
  const frame=(phase,x)=>({phase,pose:{x,y:0,z:0,tiltDeg:0}});
  return {model,bendId:'bend',scene:{bodies:[main,tip,die]},pivot,clearanceMm:.15,frontMarginMm:0,
    keyframes:[frame('AT_STATION',0),frame('WITHDRAW_FRONT',-12)]};
}

test('a straight withdrawal keeps the clearance witness at a nonzero pivot',()=>{
  const threshold=.03125+.15+1e-7;
  const hit=checkRecordedOperatorRoute(straightRoute(threshold-1e-10));
  assert.equal(hit.status,'BLOCKED');
  assert.equal(hit.blockedSample.phase,'WITHDRAW_FRONT');
  assert.equal(hit.blockedSample.result.reason,'MESH_PAIR_WITNESS');
  const collision=hit.blockedSample.result.collision;
  assert.deepEqual(collision.bodyIds,['panel:main','die']);
  assert.equal(collision.reason,'WITHIN_CLEARANCE');
  assert.deepEqual(collision.triangleIds,[0,0]);
  assert.equal(hit.blockedSample.pose.x,-2);
  const clear=checkRecordedOperatorRoute(straightRoute(threshold+1e-10));
  assert.equal(clear.status,'SAMPLE_CLEAR');
  assert.equal(clear.blockedSample,null);
  assert.equal(clear.endMaxX,-13);
});

test('each straight-route sample retains the support witness of its own pose',()=>{
  // The large pivot makes (z - pivotZ) + pivotZ lose the small Z coordinate.
  // Adding translation straight to source vertices would silently change the witness.
  const args=straightRoute(10,[.375,0,2**53]);
  const {model,bendId,scene,pivot}=args,prepared=buildOperatorSupport(model,bendId);
  const route=checkRecordedOperatorRoute(args);
  assert.equal(route.status,'SAMPLE_CLEAR');
  assert.equal(route.samples.length,7);
  for(const sample of route.samples){
    const expected=evaluateOperatorSupport(prepared,scene,{
      transformPoint:p=>transformFrontTiltPoint(p,sample.pose,pivot),
    });
    assert.deepEqual(sample.operatorSupport,expected);
    assert.equal(sample.operatorSupport.centroid[2],0);
    assert.equal(sample.operatorSupport.centroidX,sample.operatorSupport.centroid[0]);
  }
  assert.notEqual(route.samples[0].operatorSupport.centroidX,route.samples.at(-1).operatorSupport.centroidX);
});
