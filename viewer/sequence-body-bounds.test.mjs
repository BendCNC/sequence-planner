import test from 'node:test';
import assert from 'node:assert/strict';
import {bodyBounds} from './sequence-scene.mjs';

test('body bounds preserve signed zero, subnormals, non-finite coordinates and current positions',()=>{
  const low=Number.MIN_VALUE,large=Number.MAX_VALUE;
  const coords=[-0,+0,-low,+0,-0,+low,large,-large,1,large,0,1];
  for(const positions of [coords,Float64Array.from(coords)]){
    const body={positions};
    assert.deepEqual(bodyBounds(body),{min:[-0,-large,-low],max:[large,+0,1]});
    positions[0]=NaN;positions[4]=-Infinity;positions[8]=Infinity;
    assert.deepEqual(bodyBounds(body),{min:[NaN,-Infinity,-low],max:[NaN,+0,Infinity]});
  }
  assert.deepEqual(bodyBounds({positions:new Float64Array()}),{
    min:[Infinity,Infinity,Infinity],max:[-Infinity,-Infinity,-Infinity],
  });
});
