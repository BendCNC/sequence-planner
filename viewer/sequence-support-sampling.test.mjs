import test from 'node:test';
import assert from 'node:assert/strict';
import {buildOperatorSupport,evaluateOperatorSupport} from './sequence-operator-support.mjs';

test('material crossing X=0 retains its area, centroid and typed-coordinate rounding',()=>{
  const triangles=[{indices:[0,1,2],areaMm2:1.875},{indices:[2,1,3],areaMm2:2.125}];
  const support={majorBranch:{panelIds:['P'],bodies:[{bodyId:'P',triangles}]}};
  const positions=Float64Array.from([-1.234,-.438,-.005,.5,.73,.2,.38,-2.51,.6,-.03,-.21,-1.72]);
  const scene={bodies:[{id:'P',positions}]};
  const flat=evaluateOperatorSupport(support,scene);
  assert.equal(flat.frontAreaMm2,1.0289862768762505);
  assert.deepEqual(flat.centroid,[.09520833333333334,-.6989583333333332,-.038697916666666665]);
  assert.equal(flat.centroidX,flat.centroid[0]);
  assert.equal(flat.totalAreaMm2,4);
  assert.equal(flat.passes,false);

  const transformPoint=p=>Float32Array.from(p);
  const points=Array.from({length:4},(_,i)=>transformPoint(positions.subarray(3*i,3*i+3)));
  const posed=evaluateOperatorSupport(support,scene,{posedPointsById:new Map([['P',points]])});
  assert.deepEqual(posed,evaluateOperatorSupport(support,scene,{transformPoint}));
  assert.equal(posed.frontAreaMm2,1.0289862230205866);
  assert.deepEqual(posed.centroid,[.09520833692901456,-.698958321629713,-.03869791277490245]);

  const front=evaluateOperatorSupport(support,scene,{transformPoint:p=>[p[0]-4,p[1],p[2]]});
  assert.equal(front.frontFraction,1);
  assert.equal(front.passes,true);
  positions[7]=NaN;
  assert.throws(()=>evaluateOperatorSupport(support,scene),/Nonfinite support geometry/);
});

test('an incomplete reference face cannot become a valid top skin',()=>{
  const panel=(id,indices)=>({id,flatPositions:[0,0,0, 1,0,0, 0,1,0, 0,0,-1],triangles:indices});
  function support(tail){
    const panels=[panel('P',[0,1,2,...tail]),panel('C',[0,1,2])];
    return buildOperatorSupport({thicknessMm:1,panels,bends:[{id:'B',parentPanelId:'P',childPanelId:'C'}]},'B');
  }
  assert.deepEqual(support([3,0]),support([]));
  assert.throws(()=>support([0,1]),TypeError);
});
