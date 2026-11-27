import {test} from 'node:test';
import assert from 'node:assert/strict';
import {diagnosePairedClosure,pairedClosureSurfaceIntervals} from './sequence-paired-closure.mjs';

const triangles=[0,2,1,0,3,2,4,5,6,4,6,7,0,1,5,0,5,4,1,2,6,1,6,5,2,3,7,2,7,6,3,0,4,3,4,7];
function box(x0,x1,y0,y1,z0=-1,z1=0){
 return{flatPositions:[x0,y0,z0,x1,y0,z0,x1,y1,z0,x0,y1,z0,x0,y0,z1,x1,y0,z1,x1,y1,z1,x0,y1,z1],triangles:triangles.slice()};
}
function fixture(){
 const length=2.44*Math.PI/2;
 const bend=(id,parentPanelId,childPanelId,x)=>({id,parentPanelId,childPanelId,origin:[x,80,0],axis:[0,-1,0],transverse:[1,0,0],normal:[0,0,1],
  signedAngle:Math.PI/2,neutralLength:length,neutralZ:-.56,kFactor:.44,insideRadius:2,
  band:{parameters:box(0,80,0,1).flatPositions,triangles:triangles.slice()}});
 const model={units:'mm',rootPanelId:'base',thicknessMm:1,kFactor:.44,
  panels:[{id:'base',...box(0,20,0,80)},{id:'middle',...box(20+length,60+length,0,80)},{id:'end',...box(60+2*length,80+2*length,0,80)}],
  bends:[bend('crease-17','base','middle',20),bend('crease-92','middle','end',60+length)]};
 const profile=[[-200,200],[0,0],[200,200]];
 const punch={id:'wide',kind:'punch',status:'ready',profile,vertices2D:profile,triangles2D:[0,1,2],contact:{openingAngleDegrees:90},inventory:[{lengthMm:80,count:2}]};
 const die={id:'lower',kind:'die',status:'ready',profile:[[-20,0],[20,0],[20,-20],[-20,-20]],contact:{openingAngleDegrees:90,openingMm:40}};
 return{model,catalog:{tools:[punch,die]},options:{maxUnsupportedEndMm:12.7}};
}
function surface(y0=0,y1=2,x=.5){return{positions:[x,y0,.5,x,y1,.5,x,y1,1.5,x,y0,1.5],triangles:[0,1,2,0,2,3]};}
const square={vertices2D:[[0,0],[2,0],[2,2],[0,2]],triangles2D:[0,1,2,0,2,3]};

test('surface clipping certifies an interval and retains actual wall triangle points',()=>{
 const result=pairedClosureSurfaceIntervals(surface(),square);
 assert.deepEqual(result.intervals,[[0,2]]);assert.ok(result.pieces.length>0);
 for(const piece of result.pieces)for(const point of piece.polygon){
  assert.ok(point[0]>0&&point[0]<2&&point[2]>0&&point[2]<2);assert.equal(point[0],.5);
 }
});

test('tangency, missing triangulation, a hole and a free mirrored profile stay inconclusive',()=>{
 assert.deepEqual(pairedClosureSurfaceIntervals(surface(0,2,0),square).intervals,[]);
 assert.equal(pairedClosureSurfaceIntervals(surface(),{vertices2D:square.vertices2D}),null);
 assert.deepEqual(pairedClosureSurfaceIntervals(surface(),square,{mirror:true}).intervals,[]);
 // A square ring, whose centre contains no filled triangle.
 const ring={vertices2D:[[0,0],[3,0],[3,3],[0,3],[1,1],[2,1],[2,2],[1,2]],
  triangles2D:[0,1,5,0,5,4,1,2,6,1,6,5,2,3,7,2,7,6,3,0,4,3,4,7]};
 const wall={positions:[1.4,0,1.4,1.6,0,1.4,1.6,2,1.6,1.4,2,1.6],triangles:[0,1,2,0,2,3]};
 assert.deepEqual(pairedClosureSurfaceIntervals(wall,ring).intervals,[]);
});

test('both closures require every profile, both flips and both mirrors with positive width',()=>{
 const input=fixture(),result=diagnosePairedClosure(input);
 assert.ok(result);assert.equal(result.status,'MUTUAL_CLOSURE_BLOCKED_IN_CHECKED_SCOPE');
 assert.deepEqual(result.pair,['crease-17','crease-92']);assert.equal(result.closures.length,2);
 for(const c of result.closures){
  assert.equal(c.presentations.length,4);assert.ok(c.certifiedWidthMm>.5+2*c.coverageEpsilonMm);
  for(const p of c.presentations){
   assert.ok(p.witnesses.length>0);
   for(const w of p.witnesses)for(const point of w.surfacePointsTipRelativeMm)assert.ok(point.every(Number.isFinite));
   assert.deepEqual(p.machineIntervalMm,p.flip?[-c.certifiedMaterialIntervalMm[1],-c.certifiedMaterialIntervalMm[0]]:c.certifiedMaterialIntervalMm);
  }
 }
 assert.equal(result.scope.fullManufacturingImpossibility,false);
 assert.match(result.reason,/crease-17.*crease-92/);
});

test('a compatible escaping tool or an unproved second closure prevents the conclusion',()=>{
 const input=fixture(),free=structuredClone(input.catalog.tools[0]);free.id='escape';
 free.profile=free.vertices2D=[[500,0],[510,0],[510,20]];
 input.catalog.tools.push(free);assert.equal(diagnosePairedClosure(input),null);
 input.options.allowedPunchIds=['wide'];assert.ok(diagnosePairedClosure(input));
 const one=fixture();one.model.panels.find(p=>p.id==='base').flatPositions=box(-1000,-900,0,80).flatPositions;
 assert.equal(diagnosePairedClosure(one),null);
});

test('joint strokes, a disconnected scope and unsupported mounted banks are not certified',()=>{
 for(const change of [
  i=>i.model.bends[0].strokeFollowerIds=['crease-92'],
  i=>i.model.bends[0].strokeLeadId='another',
  i=>i.model.bends[0].splitHingeSpans=2,
  i=>i.model.bends.forEach(b=>b.sourceBendId='shared-source'),
  i=>i.bendIds=['crease-17'],
  i=>i.options.setups=[],i=>i.options.initialSegments=[],i=>i.options.fixedStations=[],
 ]){const input=fixture();change(input);assert.equal(diagnosePairedClosure(input),null);}
});

test('end allowance and permitted internal joints cannot be mistaken for mandatory stock',()=>{
 const input=fixture();input.options.maxUnsupportedEndMm=39.8;
 assert.equal(diagnosePairedClosure(input),null);
 input.options.maxUnsupportedEndMm=12.7;input.options.jointGapToleranceMm=100;
 assert.equal(diagnosePairedClosure(input),null);
});

test('a real but sub-joint-width collision interval cannot certify mandatory contact',()=>{
 const input=fixture();
 // Keep the support band 80 mm wide, but make each opposite wall only .4 mm
 // wide at the centre. The permitted .5 mm internal stock joint can avoid it.
 input.model.panels.find(p=>p.id==='base').flatPositions=box(0,20,39.8,40.2).flatPositions;
 const last=input.model.panels.find(p=>p.id==='end'),xs=last.flatPositions.filter((_,i)=>i%3===0);
 last.flatPositions=box(Math.min(...xs),Math.max(...xs),39.8,40.2).flatPositions;
 assert.equal(diagnosePairedClosure(input),null);
});

test('changing the die height and reflection leaves the tip-relative certificate unchanged',()=>{
 const input=fixture(),before=diagnosePairedClosure(input);assert.ok(before);
 input.catalog.tools[1].profile=input.catalog.tools[1].profile.map(([x,z])=>[-x,z+137]);
 const after=diagnosePairedClosure(input);assert.ok(after);
 for(let i=0;i<2;i++){
  const a=before.closures[i],b=after.closures[i];
  assert.ok(Math.abs(a.certifiedWidthMm-b.certifiedWidthMm)<1e-10);
  for(let j=0;j<a.presentations.length;j++){
   const pa=a.presentations[j].witnesses.flatMap(w=>w.surfacePointsTipRelativeMm.flat());
   const pb=b.presentations[j].witnesses.flatMap(w=>w.surfacePointsTipRelativeMm.flat());
   assert.equal(pa.length,pb.length);assert.ok(pa.every((v,k)=>Math.abs(v-pb[k])<1e-10));
  }
 }
});

test('IDs and bend-array order do not determine the diagnostic; inputs remain unchanged',()=>{
 const input=fixture(),before=structuredClone(input),result=diagnosePairedClosure(input);
 assert.ok(result);assert.deepEqual(input,before);
 input.model.bends.reverse();input.model.bends.forEach((b,i)=>{b.id=`unrelated ${i*41}`;b.label=`crease ${i}`;});
 const renamed=diagnosePairedClosure(input);assert.ok(renamed);
 assert.deepEqual(renamed.closures.map(c=>c.wallPanelId).sort(),result.closures.map(c=>c.wallPanelId).sort());
});

test('guard cancellation propagates and a cancelled input never produces a proof',()=>{
 assert.throws(()=>diagnosePairedClosure(fixture(),()=>{throw new Error('cancelled');}),/cancelled/);
 let calls=0;assert.throws(()=>diagnosePairedClosure(fixture(),()=>{if(++calls===8)throw new Error('deadline');}),/deadline/);
 const input=fixture();input.options.signal={aborted:true};assert.equal(diagnosePairedClosure(input),null);
 assert.ok(diagnosePairedClosure(fixture()));
});
