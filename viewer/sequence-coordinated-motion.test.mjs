import test from 'node:test';
import assert from 'node:assert/strict';
import {coordinatedMotionAt} from './sequence-coordinated-motion.mjs';
const pivot=[0,3,-1];
test('coordinated lift keeps punch/part relative displacement constant',()=>{
  const m={pivot,frames:[{ramGapMm:0,pose:{z:0}},{ramGapMm:55.4,pose:{z:55.4}}]};
  for(const q of [0,.1,.5,.9,1]){const v=coordinatedMotionAt(m,q);assert.equal(v.ramGapMm,v.pose.z);assert.deepEqual(v.pivot,pivot);}
});
test('coordinated route preserves positive-X and tilt and reverses exactly',()=>{
  const frames=[{ramGapMm:0,pose:{x:0,z:0,tiltDeg:0}},{ramGapMm:55.4,pose:{x:0,z:55.4,tiltDeg:0}},
    {ramGapMm:55.4,pose:{x:9,z:44,tiltDeg:1}},{ramGapMm:55.4,pose:{x:-110,z:6,tiltDeg:36}}];
  for(const q of [0,.05,.2,.5,.7,.95,1]){
    const a=coordinatedMotionAt({pivot,frames},q),b=coordinatedMotionAt({pivot,frames:[...frames].reverse()},1-q);
    assert.ok(Math.abs(a.ramGapMm-b.ramGapMm)<1e-10);
    for(const k of ['x','y','z','tiltDeg'])assert.ok(Math.abs(a.pose[k]-b.pose[k])<1e-10);
  }
});
test('stationary gauge approach has a defined exact pose',()=>{
  const f={ramGapMm:0,pose:{x:0,y:0,z:0,tiltDeg:0}};
  assert.deepEqual(coordinatedMotionAt({pivot,frames:[f,f]},.5),{...f,pivot});
});
test('malformed coordinated motion is rejected instead of silently animating',()=>{
  assert.throws(()=>coordinatedMotionAt({pivot,frames:[]},0));
  assert.throws(()=>coordinatedMotionAt({pivot,frames:[{ramGapMm:-1,pose:{}},{ramGapMm:0,pose:{}}]},0));
  assert.throws(()=>coordinatedMotionAt({pivot,frames:[{ramGapMm:0,pose:{x:NaN}},{ramGapMm:0,pose:{}}]},0));
});
