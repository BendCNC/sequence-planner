import test from 'node:test';
import assert from 'node:assert/strict';
import {linkJointStrokes} from './machine.mjs';
import {compareChangeClass,longCompoundFamily,probeLongCompound} from './sequence-long-compound.mjs';

const triangles=[0,2,1,0,3,2,4,5,6,4,6,7,0,1,5,0,5,4,1,2,6,1,6,5,2,3,7,2,7,6,3,0,4,3,4,7];
function bend(id,span,origin,axis,extra={}){
  const vertices=[[-span/2,0],[span/2,0],[span/2,1],[-span/2,1]];
  return {id,parentPanelId:'base',childPanelId:'panel-'+id,origin,axis,transverse:[-axis[1],axis[0],0],signedAngle:Math.PI/2,neutralZ:0,
    band:{parameters:[-1,1].flatMap(z=>vertices.flatMap(([x,y])=>[x,y,z])),triangles},...extra};
}
function blank(){
  const x=[1,0,0],y=[0,1,0];
  const a=bend('west',65,[0,0,0],x,{strokeFollowerIds:['tab-a'],strokeRequiresFlatBendIds:['port']});
  const b=bend('east',65,[0,70,0],x,{strokeFollowerIds:['tab-b'],strokeRequiresFlatBendIds:['port']});
  const c=bend('port',70.8,[0,0,0],y),d=bend('starboard',70.3,[110,0,0],y);
  b.transverse=[0,-1,0];d.transverse=[1,0,0];
  const f=(leader,id,y0)=>bend(id,43,[55,y0,0],x,{parentPanelId:c.childPanelId,strokeLeadId:leader.id});
  return linkJointStrokes({rootPanelId:'base',bends:[d,b,f(a,'tab-a',0),c,f(b,'tab-b',70),a]});
}
function stack(lengths,offset=0){
  const length=lengths.reduce((a,b)=>a+b,0);
  return {punchId:'acute',dieId:'v',stationYmm:0,segments:['punch','die'].flatMap(kind=>{
    let x=-length/2+offset;
    return lengths.map(lengthMm=>{const s={kind,lengthMm,centerY:x+lengthMm/2};x+=lengthMm;return s;});
  })};
}
test('requires two exact over-bed joint strokes, their common flat carrier and two perpendicular sides',()=>{
  const m=blank(),family=longCompoundFamily(m,100);assert.deepEqual(family.longs.map(b=>b.id),['west','east']);
  assert.deepEqual(family.shorts.map(b=>b.id),['port','starboard']);
  assert.equal(longCompoundFamily(m,115),null);
  assert.equal(longCompoundFamily(m,100,{operationTemplates:[]}),null);
  m.bends.find(b=>b.id==='east').strokeRequiresFlatBendIds=[];
  assert.equal(longCompoundFamily(m,100),null);
});
test('uses exact shared mounting, both material spans and the nearest fully supported short bank first',async()=>{
  const model=blank(),family=longCompoundFamily(model,100),byBend=new Map();
  for(const b of family.longs)byBend.set(b.id,[stack([28,28,28,28])]);
  for(const [i,b] of family.shorts.entries())byBend.set(b.id,[stack([35,35],i*.002),stack([36,36],i*.002)]);
  const initial=Object.fromEntries(model.bends.map(b=>[b.id,0]));
  const probe=complete=>probeLongCompound({model,family,setups:{byBend},initial,
    check:async({bend,from,to,config})=>{
      if(bend.strokeFollowerIds)assert.equal(to[bend.strokeFollowerIds[0]],1);
      else assert.ok(from.west&&from.east&&from['tab-a']&&from['tab-b']);
      return {id:bend.id,punch:config.segments[0],to};
    },complete,guard:()=>{}});
  const all=await probe(stages=>stages);
  assert.deepEqual(all.map(s=>s.id),['west','east','port','starboard']);
  assert.equal(all[2].punch.lengthMm,36);assert.deepEqual(all[2].punch,all[3].punch);
  const fit=await probe(stages=>stages[2].punch.lengthMm>35?null:stages);
  assert.equal(fit[2].punch.lengthMm,35);
  assert.equal(await probe(()=>null),null);
});
test('grouped preference cannot displace fewer changes or cheaper change classes',()=>{
  const score=(all,full,part,edit)=>({toolChangeEvents:all,fullChangeEvents:full,partialChangeEvents:part,segmentEditEvents:edit});
  assert.ok(compareChangeClass(score(1,0,0,1),score(0,0,0,0))>0);
  assert.ok(compareChangeClass(score(1,0,0,1),score(1,0,1,0))<0);
});
