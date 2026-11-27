import test from 'node:test';
import assert from 'node:assert/strict';
import {meshCollider,checkPair,checkScene} from './collision.mjs';

test('face separation preserves a later, closer witness against exhaustive triangle pairs',()=>{
  const tri=(x,y,z,h)=>[[x,y,z],[x+2,y+.3,z+1],[x+.1,y+2,z+h]];
  const mesh=(id,faces)=>{
    const positions=faces.flat();
    return meshCollider({id,positions,triangles:faces.map((_,i)=>[3*i,3*i+1,3*i+2])});
  };
  for(const shift of [0,1e6]){
    const left=Array.from({length:9},(_,i)=>tri(shift+i/3,i%2,i/7,i/10));
    const a=mesh('all-left',left),one=left.map(f=>mesh('left',[f]));
    for(const gap of [.0000002,.037,.5,3]){
      const right=Array.from({length:6},(_,j)=>tri(shift+j*.31,j%2+.02,gap+j*.21,j/8));
      const b=mesh('all-right',right),two=right.map(f=>mesh('right',[f]));
      let best=Infinity;
      for(const x of one)for(const y of two)best=Math.min(best,checkPair(x,y,{toleranceMm:0}).surfaceDistanceMm);
      const exact=checkPair(a,b,{toleranceMm:0});
      assert.equal(exact.surfaceDistanceMm,best);
      for(const margin of [0,Math.max(0,best-1e-9),best+1e-9]){
        assert.equal(checkPair(a,b,{queryMode:'clearance',toleranceMm:0,clearanceMm:margin}).status,
          best<=margin?'INTERFERENCE':'CLEAR');
      }
    }
  }
});

test('translating a previously queried triangle can create or clear a real collision',()=>{
  const mesh=(id,positions)=>meshCollider({id,positions,triangles:[[0,1,2]]});
  const original=[[0,0,0],[4,0,0],[0,4,0]];
  const testAt=(x,offset,expected)=>{
    const fixed=mesh('fixed',[[x,x,0],[x+.05,x,0],[x,x+.05,0]]);
    const source=mesh('tool',original);
    const opts={clearanceMm:.01,queryMode:'clearance'};
    assert.notEqual(checkPair(source,fixed,opts).status,expected);
    const translated=mesh('tool',original.map(([x,y,z])=>[x+offset,y+offset,z]));
    // As in the rigid-access sweep, retain the old item properties and the
    // BVH partition while replacing the triangle's points and bounds.
    const item={...source.items[0],...translated.items[0]};
    translated.items[0]=item;translated.bvh.items[0]=item;
    assert.equal(checkPair(translated,fixed,opts).status,expected);
  };
  testAt(2.1,1,'INTERFERENCE');
  testAt(1.1,-1,'CLEAR');
});

test('small closed features retain containment, and a collapsed face is checked in every pose',()=>{
  const distant=box('distant',[-1001,-1,-1],[-1000,0,0]);
  const tetra=[[0,0,0],[.0003,0,0],[0,.0003,0],[0,0,.0003]];
  const faces=[[0,2,1],[0,1,3],[0,3,2],[1,2,3]];
  const positions=[...distant.positions,...tetra],offset=distant.positions.length;
  const triangles=[...distant.triangles,...faces.map(face=>face.map(i=>i+offset))];
  const make=vertices=>meshCollider({id:'formed',positions:vertices,triangles});
  const normal=make(positions);
  assert.equal(normal.closed,true);
  const enclosed=box('inside',[.00003,.00003,.00003],[.00004,.00004,.00004]);
  assert.equal(checkPair(normal,enclosed,{clearanceMm:0,queryMode:'clearance'}).reason,'CONTAINMENT');
  const collapsed=make([...positions.slice(0,-1),positions[offset]]);
  assert.equal(collapsed.closed,false);
  assert.ok(collapsed.diagnostics.degenerateTriangles>0);
  assert.equal(make(positions).closed,true);
});

test('a near-parallel corner query does not invent a hit beyond the triangle edge',()=>{
  // Battery Plate at 85.5 degrees: the old intersection quotient rounded a
  // barycentric coordinate to 1, despite the segment being 0.1500012 mm away.
  const a=[[-128.08944559878188,-123.55979999999998,117.02182655909851],[-128.12468848972043,-123.55979999999998,117.05440475238065],[-128.5909651230072,-123.55979999999998,116.47928579439888]];
  const b=[[-0.9858530474023368,-123.55979999999998,-0.47152585099772065],[-127.97929634117952,-123.55979999999998,116.92000563270769],[-0.9847523333993198,-123.62585490317956,-0.4703351052974353]];
  const make=(id,p)=>meshCollider({id,positions:p,triangles:[[0,1,2]]});
  for(const queryMode of ['clearance','distance']){
    const result=checkPair(make('a',a),make('b',b),{clearanceMm:.15,queryMode});
    assert.equal(result.status,'CLEAR');
  }
  // Moving the same triangle across the edge must still report a true contact.
  assert.equal(checkPair(make('a',a.map(p=>[p[0]+.15,p[1],p[2]-.14])),make('b',b),{clearanceMm:.15}).status,'INTERFERENCE');
});

export function box(id,min,max) {
  const positions=[[min[0],min[1],min[2]],[max[0],min[1],min[2]],[max[0],max[1],min[2]],[min[0],max[1],min[2]],
    [min[0],min[1],max[2]],[max[0],min[1],max[2]],[max[0],max[1],max[2]],[min[0],max[1],max[2]]];
  const triangles=[[0,2,1],[0,3,2],[4,5,6],[4,6,7],[0,1,5],[0,5,4],[1,2,6],[1,6,5],[2,3,7],[2,7,6],[3,0,4],[3,4,7]];
  return meshCollider({id,positions,triangles});
}
function squareTube() {
  const ring=[[-3,-3],[3,-3],[3,3],[-3,3],[-1,-1],[1,-1],[1,1],[-1,1]],positions=[];
  for(const z of [-1,1])for(const [x,y] of ring)positions.push([x,y,z]);
  const triangles=[],quad=(a,b,c,d)=>triangles.push([a,b,c],[a,c,d]);
  for(let i=0;i<4;i++) {
    const j=(i+1)%4;
    quad(i,j,4+j,4+i);quad(8+i,12+i,12+j,8+j);
    quad(i,8+i,8+j,j);quad(4+i,4+j,12+j,12+i);
  }
  return meshCollider({id:'tube',positions,triangles});
}

test('distance, clearance, touching and intersection are separate static outcomes',()=>{
  const a=box('A',[0,0,0],[1,1,1]);
  let result=checkPair(a,box('B',[2,2,2],[3,3,3]),{clearanceMm:0});
  assert.equal(result.status,'CLEAR');assert.ok(Math.abs(result.distanceMm-Math.sqrt(3))<1e-10);
  result=checkPair(a,box('B',[1.15,0,0],[2,1,1]),{clearanceMm:.2});
  assert.equal(result.reason,'CLEARANCE');assert.ok(Math.abs(result.distanceMm-.15)<1e-10);
  result=checkPair(a,box('B',[1,0,0],[2,1,1]),{clearanceMm:0});
  assert.equal(result.status,'INTERFERENCE');assert.equal(result.distanceMm,0);
  result=checkPair(a,box('B',[.5,.5,.5],[1.5,1.5,1.5]),{clearanceMm:0});
  assert.equal(result.status,'INTERFERENCE');assert.equal(result.distanceMm,0);
});

test('coplanar crossing triangles and face containment both contact',()=>{
  const tri=(id,p)=>meshCollider({id,positions:p,triangles:[[0,1,2]]});
  const a=tri('A',[[-2,-.5,0],[2,-.5,0],[0,2,0]]);
  const b=tri('B',[[-2,.5,0],[2,.5,0],[0,-2,0]]);
  assert.equal(checkPair(a,b,{clearanceMm:0}).status,'INTERFERENCE');
  assert.equal(checkPair(a,tri('inside',[[0,0,0],[.1,0,0],[0,.1,0]]),{clearanceMm:0}).status,'INTERFERENCE');
  assert.equal(checkPair(a,tri('above',[[0,0,.3],[.1,0,.3],[0,.1,.3]]),{clearanceMm:.2}).status,'CLEAR');
});

test('closed-solid containment catches no triangle-surface crossings',()=>{
  const outer=box('outer',[-5,-5,-5],[5,5,5]),inner=box('inner',[-1,-1,-1],[1,1,1]);
  assert.equal(outer.closed,true);
  const result=checkPair(inner,outer,{clearanceMm:0});
  assert.equal(result.reason,'CONTAINMENT');assert.equal(result.distanceMm,0);assert.equal(result.surfaceDistanceMm,4);
});

test('real through-hole remains empty despite overlapping AABBs',()=>{
  const tube=squareTube();assert.equal(tube.closed,true);
  const inHole=box('in-hole',[-.25,-.25,-.25],[.25,.25,.25]);
  const clear=checkPair(tube,inHole,{clearanceMm:.2});
  assert.equal(clear.status,'CLEAR');assert.ok(Math.abs(clear.distanceMm-.75)<1e-10);
  const inWall=box('in-wall',[1.5,-.25,-.25],[2,.25,.25]);
  assert.equal(checkPair(tube,inWall,{clearanceMm:0}).reason,'CONTAINMENT');
});

test('disconnected mesh checks containment of every connected component',()=>{
  const a=box('a',[10,10,10],[11,11,11]),b=box('b',[-1,-1,-1],[1,1,1]);
  const combined=meshCollider({id:'combined',positions:[...a.positions,...b.positions],triangles:[...a.triangles,...b.triangles.map(t=>t.map(i=>i+a.positions.length))]});
  assert.equal(combined.components.length,2);
  assert.equal(checkPair(combined,box('outer',[-5,-5,-5],[5,5,5]),{clearanceMm:0}).reason,'CONTAINMENT');
});

test('scene ignores only explicitly allowed body pairs',()=>{
  const a=box('a',[0,0,0],[1,1,1]),b=box('b',[.5,0,0],[1.5,1,1]),c=box('c',[3,0,0],[4,1,1]);
  assert.equal(checkScene([a,b,c]).status,'INTERFERENCE');
  const result=checkScene([a,b,c],{ignorePairs:[['a','b']]});
  assert.equal(result.status,'CLEAR');assert.equal(result.pairs.length,2);
});

test('invalid indices and non-finite positions fail explicitly',()=>{
  assert.throws(()=>meshCollider({positions:[0,0,0,1,0,0,0,1,0],triangles:[0,1,3]}),/index/);
  assert.throws(()=>meshCollider({positions:[0,0,NaN,1,0,0,0,1,0],triangles:[0,1,2]}),/Non-finite/);
});

test('clearance query matches exact outcomes and labels conservative distances',()=>{
  const a=box('A',[0,0,0],[1,1,1]);
  for(const dx of [-.5,0,.5,1,1.1,1.2,1.3,2,10]) {
    const b=box('B',[dx,.2,.2],[dx+.2,.8,.8]);
    const exact=checkPair(a,b,{clearanceMm:.2}),fast=checkPair(a,b,{clearanceMm:.2,queryMode:'clearance'});
    assert.equal(fast.status,exact.status,`offset ${dx}`);
    assert.ok(fast.distanceMm<=exact.distanceMm+1e-10);
    assert.equal(fast.distanceIsExact,false);assert.equal(fast.distanceBound,'LOWER');
  }
  const throughHole=checkPair(squareTube(),box('hole',[-.25,-.25,-.25],[.25,.25,.25]),{queryMode:'clearance'});
  assert.equal(throughHole.status,'CLEAR');assert.ok(throughHole.distanceMm>.2);
  const contained=checkPair(box('outer',[-5,-5,-5],[5,5,5]),a,{queryMode:'clearance'});
  assert.equal(contained.reason,'CONTAINMENT');
});
