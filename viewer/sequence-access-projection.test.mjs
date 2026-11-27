import assert from 'node:assert/strict';
import test from 'node:test';
import {createAccessProjection} from './sequence-access-projection.mjs';

const pose=(x=0,tiltDeg=0,ramGapMm=0)=>({x,z:0,tiltDeg,ramGapMm});
function body(id,kind,points){
  return {id,kind,positions:Float64Array.from(points.flat()),triangles:points.map((_,i)=>i)};
}
function projection(panel,weights,tools=[]){
  const triangles=weights.map((areaMm2,i)=>({indices:[3*i,3*i+1,3*i+2],areaMm2}));
  return createAccessProjection({
    scene:{bodies:[panel,...tools]},pivot:[0,0,0],clearanceMm:.15,
    preparedSupport:{majorBranch:{bodies:[{bodyId:panel.id,triangles}]}}
  });
}

test('station projection keeps each obstacle and its ram motion when axial spans coincide',()=>{
  const footprint=[[-4,-1,0],[-2,1,0],[-4,1,1]];
  const sheet=body('panel','panel',footprint);
  const die=body('die','die',footprint.map(([x,y,z])=>[x+20,y,z]));
  const punch=body('punch','punch',footprint);
  for(const tools of [[die,punch],[punch,die]]){
    const p=projection(sheet,[1],tools);
    assert.equal(p.pairCount,2);
    assert.equal(p.triangleCount,2);
    assert.equal(p.clear(pose()),false);
    assert.equal(p.clear(pose(0,0,5)),true);
    assert.equal(p.clear(pose()),false);
  }
});

test('operator support clips the fraction at the station and retains exact threshold ties',()=>{
  const cases=[
    {x:[-2,-1,-0],fraction:1},
    {x:[0,1,2],fraction:0},
    {x:[-2,0,2],fraction:.5},
    {x:[-2,-1,1],fraction:1-1/6},
    {x:[-1,1,2],fraction:1/6}
  ];
  for(const {x,fraction} of cases){
    // Small patches well outside the cutting plane keep the centroid in front.
    for(const front of [0,.5,1,2,8]){
      for(const back of [0,.5,1,2,8]){
        const vertices=[[-102,-1,0],[-101,1,0],[-100,1,1],
          [1,-1,0],[2,1,0],[3,1,1], [x[0],-1,0],[x[1],1,0],[x[2],1,1]];
        const p=projection(body('panel','panel',vertices),[front,back,1]);
        const mean=(-101*front+2*back+(x[0]+x[1]+x[2])/3)/(front+back+1);
        assert.equal(p.clear(pose()),mean < -1e-7 && (front+fraction)/(front+back+1)>.5+1e-9);
      }
    }
  }
});
