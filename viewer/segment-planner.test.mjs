import test from 'node:test';
import assert from 'node:assert/strict';
import {planSegmentMove,solveSegmentLayout} from './segment-planner.mjs';

const clear=()=>({status:'CLEAR',distanceMm:Infinity});

test('intermediate obstruction is detected despite clear endpoints',()=>{
  const result=planSegmentMove({from:{centerY:-10,z:0},to:{centerY:10,z:0},retractDistancesMm:[0],maxStepMm:1,
    evaluatePose:p=>({status:Math.abs(p.centerY)<.5?'INTERFERENCE':'CLEAR',distanceMm:Math.max(0,Math.abs(p.centerY)-.5)})});
  assert.equal(result.status,'BLOCKED');assert.equal(result.attempts[0].blockedSample.pose.centerY,0);
  assert.equal(result.limits.continuousCollisionDetection,false);
});

test('retract-slide-approach finds a sampled route above an obstacle',()=>{
  const result=planSegmentMove({from:{centerY:-10,z:0},to:{centerY:10,z:0},retractDistancesMm:[1,5],maxStepMm:1,
    evaluatePose:p=>({status:Math.abs(p.centerY)<2&&p.z<3?'INTERFERENCE':'CLEAR'})});
  assert.equal(result.status,'SAMPLE_CLEAR');assert.equal(result.attempts.length,2);
  assert.deepEqual(result.keyframes.map(k=>k.phase),['START','RETRACT','SLIDE','APPROACH']);
  assert.equal(result.keyframes[1].pose.z,5);assert.ok(result.samples.length>10);
});

test('blocked endpoint, load, rotation and sample budget are explicit',()=>{
  assert.equal(planSegmentMove({from:{y:0,z:0},to:{y:10,z:0},underLoad:true,evaluatePose:clear}).reason,'SEGMENT_UNDER_LOAD');
  const target=planSegmentMove({from:{y:0,z:0},to:{y:10,z:0},evaluatePose:p=>({status:p.y===10&&p.z===0?'INTERFERENCE':'CLEAR'})});
  assert.equal(target.reason,'TARGET_INTERFERENCE');
  const budget=planSegmentMove({from:{y:0,z:0},to:{y:100,z:0},evaluatePose:clear,maxSamples:2});
  assert.equal(budget.reason,'SEARCH_BUDGET_EXHAUSTED');
  assert.throws(()=>planSegmentMove({from:{y:0,z:0,rotation:0},to:{y:1,z:0,rotation:1},evaluatePose:clear}),/Only translation/);
});

test('conservative distance bound is optional and still reports sampled status',()=>{
  const result=planSegmentMove({from:{y:0,z:0},to:{y:100,z:0},evaluatePose:()=>({status:'CLEAR',distanceMm:1000}),distanceIsConservative:true});
  assert.equal(result.status,'SAMPLE_CLEAR');assert.ok(result.limits.boundedIntervals>0);assert.equal(result.limits.continuousCollisionDetection,false);
});

test('relative withdrawal coordinates such as offsetZ are explicit translation axes',()=>{
  const result=planSegmentMove({from:{centerY:-10,offsetZ:0},to:{centerY:10,offsetZ:0},retractAxis:'offsetZ',
    retractDistancesMm:[5],maxStepMm:2,evaluatePose:clear});
  assert.equal(result.status,'SAMPLE_CLEAR');
  assert.equal(result.keyframes[1].pose.offsetZ,5);
  assert.equal(result.keyframes[2].pose.centerY,10);
  assert.equal('z' in result.keyframes[1].pose,false);
});

test('layout uses only actual stock and respects coverage for each tool kind',()=>{
  const stock=[{id:'P1',kind:'punch',lengthMm:100,centerY:0},{id:'D1',kind:'die',lengthMm:100,centerY:0}];
  const result=solveSegmentLayout({segments:stock,bedMinY:-100,bedMaxY:100,requiredIntervals:[[-40,40]],evaluateLayout:clear});
  assert.equal(result.status,'LAYOUT_FOUND');assert.equal(result.placements.length,2);
  assert.ok(result.placements.every(s=>s.lengthMm===100));assert.equal(result.collisionStatus,'SAMPLE_CLEAR');
  const impossible=solveSegmentLayout({segments:stock,bedMinY:-100,bedMaxY:100,requiredIntervals:[[-60,60]]});
  assert.equal(impossible.status,'BLOCKED');
});

test('layout translates around geometry or reports no tested layout',()=>{
  const args={segments:[{id:'P',kind:'punch',lengthMm:100,centerY:0}],bedMinY:-100,bedMaxY:100,
    requiredIntervals:[[-10,10]],offsetStepMm:5};
  const avoid=solveSegmentLayout({...args,evaluateLayout:ps=>({status:ps[0].centerY<20?'INTERFERENCE':'CLEAR'})});
  assert.equal(avoid.status,'LAYOUT_FOUND');assert.ok(avoid.placements[0].centerY>=20);
  assert.equal(solveSegmentLayout({...args,evaluateLayout:()=>({status:'INTERFERENCE'})}).status,'BLOCKED');
});

test('gaps and material openings remain distinct coverage constraints',()=>{
  const segments=[{id:'P1',kind:'punch',lengthMm:40,centerY:-30,fixed:true},{id:'P2',kind:'punch',lengthMm:40,centerY:30,fixed:true}];
  const args={segments,bedMinY:-60,bedMaxY:60,minGapMm:10};
  const hole=solveSegmentLayout({...args,requiredIntervals:[[-50,-10],[10,50]]});
  assert.equal(hole.status,'LAYOUT_FOUND');assert.equal(hole.collisionStatus,'NOT_EVALUATED');
  assert.equal(solveSegmentLayout({...args,requiredIntervals:[[-50,50]]}).status,'BLOCKED');
});
