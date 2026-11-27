import {unprotect} from './protected-json.mjs';
import {test} from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {generateToolSetupCandidates,toolSetupKey,optimizeToolSetupSequence,scoreToolSetupSequence,classifyBendSpan,toolChangeKind,annotateToolChanges} from './tool-setup-search.mjs';

function config(label,lengths=[50.8],shift=0){
  return{punchId:`p-${label}`,dieId:`d-${label}`,segments:['punch','die'].flatMap(kind=>{
    let end=-lengths.reduce((s,x)=>s+x,0)/2+shift;
    return lengths.map((lengthMm,i)=>{const s={id:`${kind}-${i}`,kind,lengthMm,centerY:end+lengthMm/2,offsetX:0,offsetZ:0};end+=lengthMm;return s;});})};
}
const edge=(id,mask,activeIndex,setup,extra={})=>({id,mask,activeIndex,config:setup,feasible:true,uncoveredLengthMm:0,...extra});
const catalog=unprotect(readFileSync(new URL('./tooling.json.bct',import.meta.url)));

test('short 1/2-inch setups stay candidates with exact undercoverage, independently for punch and die',()=>{
  const selected={tools:catalog.tools.filter(t=>['9302','9858'].includes(t.vendorCode))};
  const result=generateToolSetupCandidates({catalog:selected,bend:{signedAngle:Math.PI/2,insideRadius:2},
    materialIntervals:[[-34,34]],partProjectedSpanMm:200});
  const single=length=>result.candidates.find(c=>c.banks.punch.lengths.length===1&&c.banks.die.lengths.length===1&&
    c.banks.punch.lengths[0]===length&&c.banks.die.lengths[0]===length);
  for(const [length,missing] of [[50.8,17.2],[25.4,42.6]]){
    const c=single(length);assert.ok(c);assert.equal(c.coverage.status,'INCOMPLETE');
    assert.ok(Math.abs(c.uncoveredLengthMm-missing)<1e-9);assert.equal(c.status,'DIAGNOSTIC_UNDERCOVERED');
    assert.equal(c.formabilityVerified,false);assert.equal(c.collisionChecked,false);
  }
  assert.equal(single(101.6).coverage.status,'COVERED');
  assert.ok(result.candidates.some(c=>c.banks.punch.totalLengthMm>=68&&c.banks.die.totalLengthMm===50.8));
  assert.equal(result.spanClassification.internalBendVerified,false);
  assert.equal(result.spanClassification.relation,'LOCAL_BEND_SHORTER_THAN_PART_PROJECTION');
});

test('full banks respect physical inventory counts, and nominal angle incompatibilities are filtered',()=>{
  const tools=['punch','die'].map(kind=>({id:kind,kind,status:'ready',contact:{openingAngleDegrees:30,tipRadiusMm:kind==='punch'?1:null},
    inventory:[{lengthMm:50.8,count:1},{lengthMm:25.4,count:1}]}));
  tools.push({id:'wide-angle',kind:'die',status:'ready',contact:{openingAngleDegrees:85},inventory:[{lengthMm:101.6,count:1}]});
  const result=generateToolSetupCandidates({catalog:{tools},bend:{signedAngle:2*Math.PI/3,insideRadius:2},materialIntervals:[[-34,34]]});
  const full=result.candidates.filter(c=>c.coverage.status==='COVERED');assert.equal(full.length,1);
  assert.deepEqual(full[0].banks.punch.lengths,[50.8,25.4]);assert.deepEqual(full[0].banks.die.lengths,[50.8,25.4]);
  assert.ok(result.diagnostics.nominalRejectedToolIds.includes('wide-angle'));
  for(const c of result.candidates)for(const kind of ['punch','die'])assert.ok(c.banks[kind].lengths.length<=2);
});

test('setup identity is profiles plus stock multiset, with placement and naming separate',()=>{
  const a=config('A',[50.8,25.4]),b=config('A',[25.4,50.8],20);
  b.segments.reverse();b.segments.forEach((s,i)=>s.id=`renamed-${i}`);
  assert.equal(toolSetupKey(a),toolSetupKey(b));
  assert.notEqual(toolSetupKey(a),toolSetupKey(config('B',[50.8,25.4])));
  assert.notEqual(toolSetupKey(a),toolSetupKey(config('A',[76.2])));
});

test('shared longer banks survive local minimal-bank pruning and still obey inventory',()=>{
  const selected={tools:catalog.tools.filter(t=>['3169','9858'].includes(t.vendorCode))};
  const result=generateToolSetupCandidates({catalog:selected,bend:{signedAngle:Math.PI/2,insideRadius:2},
    materialIntervals:[[-21.25,21.25]],sharedBankLengthsMm:[[101.6,50.8,25.4],[406.4,406.4,406.4]]});
  const shared=result.candidates.find(c=>Math.abs(c.banks.punch.totalLengthMm-177.8)<1e-8&&Math.abs(c.banks.die.totalLengthMm-177.8)<1e-8);
  assert.ok(shared);assert.ok(shared.banks.punch.families.includes('SHARED_SETUP_BANK'));
  assert.equal(shared.coverage.status,'COVERED');assert.ok(result.diagnostics.banks.every(b=>b.rejectedSharedBanks===1));
});

test('global DP preserves an initially non-greedy setup and avoids a later change',()=>{
  const a=config('A'),b=config('B');
  const transitions=[edge('first-local-A',0,0,a),edge('first-alternative-B',0,0,b),
    edge('second-B',1,1,b),edge('third-B',3,2,b)];
  const greedy=scoreToolSetupSequence([transitions[0],transitions[2],transitions[3]]);
  assert.equal(greedy.cost.toolChangeEvents,1);
  const result=optimizeToolSetupSequence({bendIds:['B3','B4','B2'],transitions});
  assert.deepEqual(result.sequence.map(e=>e.id),['first-alternative-B','second-B','third-B']);
  assert.equal(result.cost.toolChangeEvents,0);assert.equal(result.statistics.algorithm,'dynamic programming; no permutation enumeration');
});

test('among equally few changes, coverage outranks shorter banks and fewer segments',()=>{
  const short=config('A',[25.4]),full=config('A',[50.8,25.4]);
  const transitions=[edge('short-first',0,0,short,{uncoveredLengthMm:42.6}),edge('full-first',0,0,full),
    edge('short-second',1,1,short,{uncoveredLengthMm:42.6}),edge('full-second',1,1,full)];
  const result=optimizeToolSetupSequence({operationIds:['B1_pre','B1_finish'],transitions});
  assert.deepEqual(result.sequence.map(e=>e.id),['full-first','full-second']);
  assert.equal(result.cost.toolChangeEvents,0);assert.equal(result.cost.uncoveredLengthMm,0);
  assert.equal(result.formabilityVerified,false);
  const onlyShort=optimizeToolSetupSequence({operationIds:['B1_pre','B1_finish'],transitions:[transitions[0],transitions[2]]});
  assert.equal(onlyShort.status,'DIAGNOSTIC_UNDERCOVERED');assert.ok(onlyShort.coverageUnresolved);
  const contradictory=optimizeToolSetupSequence({operationIds:['op'],transitions:[edge('short',0,0,short,{uncoveredLengthMm:42.6,coverageUnresolved:false})]});
  assert.equal(contradictory.status,'DIAGNOSTIC_UNDERCOVERED');
});

test('logical operation order follows provided masks, and alternative orders remain available',()=>{
  const a=config('A'),b=config('B');
  const result=optimizeToolSetupSequence({operationIds:['bend-A','bend-B'],transitions:[
    edge('A-first',0,0,a),edge('B-after-A',1,1,b),edge('B-first',0,1,b),edge('A-after-B',2,0,b)]});
  assert.deepEqual(result.sequence.map(e=>e.operationId),['bend-B','bend-A']);assert.equal(result.cost.toolChangeEvents,0);
});

test('first mounting is not a change; an explicitly supplied different initial setup is',()=>{
  const a=config('A'),b=config('B'),transitions=[edge('only',0,0,b)];
  assert.equal(optimizeToolSetupSequence({operationIds:['op'],transitions}).cost.toolChangeEvents,0);
  assert.equal(optimizeToolSetupSequence({operationIds:['op'],transitions,initialSetup:a}).cost.toolChangeEvents,1);
  assert.equal(optimizeToolSetupSequence({operationIds:['op'],transitions,initialSetupKey:toolSetupKey(b)}).cost.toolChangeEvents,0);
  const oriented=[edge('only',0,0,b,{flip:true,stationShiftMm:100})];
  const keyed=optimizeToolSetupSequence({operationIds:['op'],transitions:oriented,initialSetupKey:toolSetupKey(b)});
  assert.equal(keyed.cost.orientationChanges,0);assert.equal(keyed.cost.stationTravelMm,0);
});

test('selected sequence costs are independently recounted, including shifts and orientation',()=>{
  const a=config('A'),moved=config('A',[50.8],10),b=config('B',[25.4,25.4]);
  const transitions=[edge('a',0,0,a),edge('move',1,1,moved,{stationShiftMm:5}),
    edge('b',3,2,b,{flip:true,stationShiftMm:20,uncoveredLengthMm:3.2})];
  const result=optimizeToolSetupSequence({operationIds:['x','y','z'],transitions});
  const cost=scoreToolSetupSequence(result.sequence).cost;
  assert.deepEqual(cost,result.cost);
  assert.deepEqual(cost,{fullChangeEvents:1,partialChangeEvents:0,segmentEditEvents:0,toolChangeEvents:1,uncoveredLengthMm:3.2,profileChanges:2,segmentEdits:6,orientationChanges:1,repositionEvents:1,stationTravelMm:20});
});

const mixed=(punchLabel,punchLengths,dieLabel,dieLengths)=>{const p=config(punchLabel,punchLengths),d=config(dieLabel,dieLengths);
  return{punchId:p.punchId,dieId:d.dieId,segments:[...p.segments.filter(s=>s.kind==='punch'),...d.segments.filter(s=>s.kind==='die')]};};

test('change kind per tool: none, segment edit, one profile, both profiles',()=>{
  const base=config('A',[50.8,25.4]);
  assert.deepEqual(toolChangeKind(null,base),{punch:'none',die:'none'});
  assert.deepEqual(toolChangeKind(base,config('A',[50.8,25.4],30)),{punch:'none',die:'none'});
  assert.deepEqual(toolChangeKind(base,config('A',[101.6,25.4])),{punch:'segments',die:'segments'});
  assert.deepEqual(toolChangeKind(base,mixed('A',[101.6,25.4],'A',[50.8,25.4])),{punch:'segments',die:'none'});
  assert.deepEqual(toolChangeKind(base,mixed('B',[50.8,25.4],'A',[50.8,25.4])),{punch:'profile',die:'none'});
  assert.deepEqual(toolChangeKind(base,config('B',[50.8,25.4])),{punch:'profile',die:'profile'});
  const stages=annotateToolChanges([{config:base},{config:config('A',[101.6,25.4])},{config:config('A',[101.6,25.4])}]);
  assert.deepEqual(stages.map(s=>s.toolChange),[{punch:'none',die:'none'},{punch:'segments',die:'segments'},{punch:'none',die:'none'}]);
});

test('segment edits are not counted as full changes; punch-only beats full replacement',()=>{
  const a=config('A',[50.8,25.4]);
  const seg=scoreToolSetupSequence([edge('1',0,0,a),edge('2',1,1,config('A',[101.6,25.4]))]).cost;
  assert.equal(seg.fullChangeEvents,0);assert.equal(seg.partialChangeEvents,0);assert.equal(seg.segmentEditEvents,1);assert.equal(seg.profileChanges,0);
  const punchOnly=scoreToolSetupSequence([edge('1',0,0,a),edge('2',1,1,mixed('B',[50.8,25.4],'A',[50.8,25.4]))]).cost;
  assert.equal(punchOnly.partialChangeEvents,1);assert.equal(punchOnly.fullChangeEvents,0);assert.equal(punchOnly.profileChanges,1);
  const result=optimizeToolSetupSequence({operationIds:['x','y'],transitions:[edge('x',0,0,a),
    edge('y-full',1,1,config('B',[50.8,25.4])),edge('y-punch',1,1,mixed('B',[50.8,25.4],'A',[50.8,25.4]))]});
  assert.equal(result.sequence[1].id,'y-punch');
  // Ranking: one profile change is worse than any number of segment edits.
  const twoEdits=optimizeToolSetupSequence({operationIds:['x','y','z'],transitions:[edge('x',0,0,a),
    edge('y-edit',1,1,config('A',[101.6])),edge('z-edit',3,2,a),
    edge('y-profile',1,1,mixed('B',[50.8,25.4],'A',[50.8,25.4])),edge('z-profile',3,2,mixed('B',[50.8,25.4],'A',[50.8,25.4]))]});
  assert.deepEqual(twoEdits.sequence.map(e=>e.id),['x','y-edit','z-edit']);assert.equal(twoEdits.cost.segmentEditEvents,2);
});

test('an already mounted longer stack is reused instead of a segment edit when supplied as feasible',()=>{
  const short=config('A',[50.8,25.4]),long=config('A',[101.6,25.4]);
  const result=optimizeToolSetupSequence({operationIds:['narrow','wide'],transitions:[
    edge('narrow-short',0,0,short),edge('narrow-long',0,0,long),edge('wide-long',1,1,long)]});
  assert.deepEqual(result.sequence.map(e=>e.id),['narrow-long','wide-long']);
  assert.equal(result.cost.toolChangeEvents,0);assert.equal(result.cost.segmentEditEvents,0);
});

test('DP retains orientation state even for the same tool setup',()=>{
  const a=config('A');
  const result=optimizeToolSetupSequence({operationIds:['x','y'],transitions:[
    edge('front',0,0,a,{flip:false}),edge('turned',0,0,a,{flip:true}),edge('finish',1,1,a,{flip:true})]});
  assert.equal(result.sequence[0].id,'turned');assert.equal(result.cost.orientationChanges,0);
});

test('infeasible edges do not become paths and incomplete graphs are reported explicitly',()=>{
  const a=config('A');
  const result=optimizeToolSetupSequence({operationIds:['x','y'],transitions:[edge('start',0,0,a),edge('blocked',1,1,a,{feasible:false})]});
  assert.equal(result.status,'NO_FEASIBLE_COMPLETE_PATH');assert.equal(result.statistics.ignoredInfeasibleTransitions,1);
  assert.throws(()=>optimizeToolSetupSequence({operationIds:Array.from({length:9},(_,i)=>i),transitions:[]}));
  assert.throws(()=>optimizeToolSetupSequence({operationIds:['x'],transitions:[edge('invalid',1,0,a)]}));
  assert.equal(classifyBendSpan({materialIntervals:[[-83.5,83.5]],partProjectedSpanMm:186.5}).internalBendVerified,false);
});
