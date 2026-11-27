import test from 'node:test';
import assert from 'node:assert/strict';
import {remainingPhaseQuota} from './sequence-search-variants.mjs';

test('forward search retains a positive budget after a reverse search has used 55 percent',()=>{
  const quota=remainingPhaseQuota({transitions:600,states:55,now:55000},{transitions:1000,states:100,until:100000},.4);
  assert.deepEqual(quota,{transitions:760,states:73,until:73000});
  assert.ok(quota.transitions>600&&quota.states>55&&quota.until>55000);
});
test('phase allocation cannot exceed the global budget or create time after its deadline',()=>{
  assert.deepEqual(remainingPhaseQuota({transitions:1000,states:100,now:110000},{transitions:1000,states:100,until:100000},.75),
    {transitions:1000,states:100,until:100000});
  assert.deepEqual(remainingPhaseQuota({transitions:999,states:99,now:99999},{transitions:1000,states:100,until:100000},.4),
    {transitions:1000,states:100,until:99999.4});
});
