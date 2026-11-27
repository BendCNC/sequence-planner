import {unprotect} from './protected-json.mjs';
import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {rankToolingCandidates,selectPlanGroups,adjacentFlangeSections,flangeClearancePenalty} from './tooling-candidates.mjs';

const catalog=unprotect(readFileSync(new URL('./tooling.json.bct',import.meta.url)));
const model=(t,r,deg=90)=>({thicknessMm:t,bends:[{insideRadius:r,signedAngle:deg*Math.PI/180}]});
const dieNames=(m,maxPlans=5)=>selectPlanGroups(rankToolingCandidates(m,catalog),maxPlans)
  .map(g=>catalog.tools.find(t=>t.id===g.dieId).name);

test('20 ga with radius = t only keeps the 8 mm die',()=>{
  const names=dieNames(model(0.9119,0.9119));
  assert.deepEqual(names,['Acute 8mm Die 30° Die [BC-5623]']);
});

test('16 ga on the 8 mm die (V/t 5.3, below the rule of 8) stays a design die',()=>{
  assert.ok(dieNames(model(1.5,1.0)).includes('Acute 8mm Die 30° Die [BC-5623]'));
});

test('wide dies are rejected for thin sheet and ranking is closest radius first',()=>{
  const r=rankToolingCandidates(model(2.5,2.5),catalog);
  assert.ok(r.candidates.filter(c=>c.viable).every(c=>c.openingMm>=15&&c.openingMm<=30));
  const errs=r.candidates.filter(c=>c.viable).map(c=>Math.abs(c.radiusErrorMm));
  assert.deepEqual(errs,errs.slice().sort((a,b)=>a-b));
});

test('acute bend excludes 85° four-way dies and the 88° gooseneck',()=>{
  const r=rankToolingCandidates(model(2.5,2.5,120),catalog);
  for(const c of r.candidates.filter(c=>c.viable)){assert.ok(c.dieAngleDegrees<=60);assert.ok(c.punchAngleDegrees<=60);}
});

test('preferred pair is planned first when it matches the design',()=>{
  const r=rankToolingCandidates(model(2.5,2.5),catalog,{preferredPunchId:'28790284-76e9-482d-9a1a-091b56e3b885',
    preferredDieId:'1c1b4345-d9bd-4338-b0aa-27da63d8f28a'});
  const groups=selectPlanGroups(r,3);
  assert.equal(groups[0].dieId,'1c1b4345-d9bd-4338-b0aa-27da63d8f28a');assert.ok(groups[0].preferred);
});


const rect=(x0,x1,z0,z1)=>[[x0,z0],[x1,z0],[x1,z1],[x0,z1]];
const tool=(id,kind,profile,extra={})=>({id,name:id,kind,status:'ready',vertices2D:profile,contact:{tipRadiusMm:1,openingAngleDegrees:30,...extra}});
// Channel: base panel B between two 90 degree bends 20 mm apart, each with an 8 mm flange.
const panel=(id,x0,x1)=>({id,flatPositions:[x0,0,0,x1,0,0,x1,100,0,x0,100,0]});
const channel={thicknessMm:1,panels:[panel('L',-8,0),panel('B',0,20),panel('R',20,28)],
  bends:[{id:'b1',insideRadius:1,signedAngle:Math.PI/2,origin:[0,0],transverse:[1,0],parentPanelId:'B',childPanelId:'L'},
    {id:'b2',insideRadius:1,signedAngle:Math.PI/2,origin:[20,0],transverse:[1,0],parentPanelId:'B',childPanelId:'R'}]};
const wide=tool('wide','punch',rect(-30,30,0,120));
const slim=tool('slim','punch',[[0,0],[2,5],[2,120],[-2,120],[-2,5]]);
const die=tool('die','die',rect(-6,6,-60,0),{openingMm:8});

test('formed return flanges are located next to each bend',()=>{
  const s=adjacentFlangeSections(channel);
  assert.equal(s.length,2);
  for(const b of s){assert.equal(b.sections.length,1);assert.ok(b.sections[0].points.every(([,z])=>z>0));}
});

test('a wide punch body is penalised and a slim punch clears the flanges',()=>{
  const s=adjacentFlangeSections(channel);
  assert.ok(flangeClearancePenalty(wide,s)>0);
  assert.equal(flangeClearancePenalty(slim,s),0);
});

test('flange clearance ranks clear tools first but never rejects candidates',()=>{
  const catalog={tools:[wide,slim,die]};
  const off=rankToolingCandidates(channel,catalog,{minOpeningThickness:0,maxOpeningThickness:100,radiusTolerance:100});
  const on=rankToolingCandidates(channel,catalog,{minOpeningThickness:0,maxOpeningThickness:100,radiusTolerance:100,flangeClearanceRanking:true});
  assert.equal(on.candidates[0].punchId,'slim');
  assert.deepEqual(on.candidates.map(c=>c.viable).sort(),off.candidates.map(c=>c.viable).sort());
  assert.equal(on.candidates.length,off.candidates.length);
  assert.ok(on.candidates.find(c=>c.punchId==='wide').viable);
});

test('models without flat geometry score zero',()=>{
  const m={thicknessMm:1,bends:[{insideRadius:1,signedAngle:Math.PI/2}]};
  assert.equal(flangeClearancePenalty(wide,adjacentFlangeSections(m)),0);
});
