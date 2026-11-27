import test from 'node:test';
import assert from 'node:assert/strict';
import {resolveToolingRules} from './tooling-rules.mjs';
import {planCompleteSequence} from './sequence-plan.mjs';
import {planExperimentalSequence} from './sequence-portfolio.mjs';
import {planAdvancedSequence} from './sequence-advanced-search.mjs';
import {rankToolingCandidates, selectPlanGroups} from './tooling-candidates.mjs';
import {advancedCompatibleTools} from './sequence-advanced-setups.mjs';

const panel=(id, x0, x1)=>({id,flatPositions:[x0,0,0,x1,0,0,x1,50,0,x0,50,0]});
const bend=(id,left,right,x)=>({id,label:id,parentPanelId:left,childPanelId:right,origin:[x,0],transverse:[1,0],signedAngle:Math.PI/2,insideRadius:1});
const model={thicknessMm:2.5,panels:[panel('root',0,40),panel('short',-8,0),panel('long',40,80)],
  bends:[bend('B2','root','long',40),bend('B3','root','short',0)]};
const punch=(id,radius)=>({kind:'punch',status:'ready',id,contact:{tipRadiusMm:radius,openingAngleDegrees:30}});
const die=(id,opening)=>({kind:'die',status:'ready',id,contact:{openingMm:opening,openingAngleDegrees:30}});
const catalog={tools:[punch('sharp',1),punch('round',1.3),die('V8',8),die('V16',16)]};

test('first impossible flange stops normal, Qt quick, advanced and explicit lists before geometry',async()=>{
  for(const [entry,options] of [
    [planCompleteSequence,{toolingSelection:{}}],[planExperimentalSequence,{dieRules:{}}],
    [planExperimentalSequence,{toolingSelection:{},experimentalOnly:'advanced'}],
    [planAdvancedSequence,{dieRules:{},allowedDieIds:['V16']}],
  ]){
    const result=await entry({model,catalog,options},()=>assert.fail('entered geometry'));
    assert.equal(result.status,'TOOLING_RULES_BLOCKED');
    const b=result.dieRuleDecision.blockedBend;
    assert.equal(b.label,'B3');assert.equal(b.flangeMm,8);assert.equal(b.minOpeningMm,12.5);
    assert.ok(b.blockedBy.includes('DIE_MIN_FLANGE_EXCEEDS_PART'));
    assert.equal(result.diagnostics.statesExpanded,0);assert.equal(result.diagnostics.foldChecks,0);
  }
});

test('an exception needs a preferred die or explicit fallback and applies only on permitted operations',()=>{
  let result=resolveToolingRules(model,catalog,{dieRules:{},preferredDieId:'V8'});
  assert.equal(result.decision.mode,'preferred-exception');
  assert.deepEqual(result.decision.byBend.B3.dieIds,['V8']);
  assert.deepEqual(result.decision.byBend.B2.dieIds,['V8','V16']);
  result=resolveToolingRules(model,catalog,{toolingSelection:{allowFlangeFallback:true}});
  assert.equal(result.decision.mode,'flange-fallback');
  assert.deepEqual(result.decision.byBend.B3.dieIds,['V8']);
  assert.deepEqual(result.decision.byBend.B2.dieIds,['V16','V8']);
  assert.equal(result.decision.exceptions[0].preferred,false);
  assert.equal(resolveToolingRules(model,catalog,{dieRules:{},preferredDieId:'V16'}).blocked.status,'TOOLING_RULES_BLOCKED');
  assert.equal(resolveToolingRules(model,catalog,{dieRules:{},preferredDieId:'V8',allowedDieIds:[]}).blocked.status,'TOOLING_RULES_BLOCKED');
  assert.equal(resolveToolingRules(model,catalog,{dieRules:{},preferredDieId:'V8',allowedPunchIds:[]}).blocked.status,'TOOLING_RULES_BLOCKED');
});

test('branches still flat, non-right-angle and missing dimensions cannot prove a short flange',()=>{
  const chain={...model,thicknessMm:2,panels:[panel('a',-20,0),panel('b',0,3),panel('c',3,6),panel('d',6,26)],
    bends:[bend('outer1','a','b',0),bend('inner','b','c',3),bend('outer2','c','d',6)]};
  assert.ok(resolveToolingRules(chain,catalog,{dieRules:{}}).decision.byBend.inner.dieIds.includes('V16'));
  const other={...model,bends:model.bends.map(b=>({...b,signedAngle:Math.PI/3}))};
  assert.ok(resolveToolingRules(other,catalog,{dieRules:{}}).decision.byBend.B3.dieIds.includes('V16'));
  assert.equal(resolveToolingRules({...model,panels:[]},catalog,{dieRules:{}}).blocked,undefined);
  assert.equal(resolveToolingRules(model,{tools:[punch('p',null),die('d',null)]},{dieRules:{}}).blocked,undefined);
});

test('nominal ranking skips an oversized preferred tip and preserves other punches on the same die',()=>{
  const m={...model,thicknessMm:1.5};
  const ranking=rankToolingCandidates(m,catalog,{preferredPunchId:'round',preferredDieId:'V8'});
  assert.deepEqual(selectPlanGroups(ranking,3)[0].punchIds,['sharp']);
  assert.ok(ranking.candidates.find(c=>c.preferred).reasons.includes('PUNCH_TIP_RADIUS_EXCEEDS_DESIGN'));
  assert.equal(resolveToolingRules(m,catalog,{dieRules:{maxTonnagePerMeterKn:1,tensileMpa:400},preferredDieId:'V8'}).blocked.status,'TOOLING_RULES_BLOCKED');
});

test('a wide included angle and larger punch can preform a bend that another pair must finish',()=>{
  const b={...model.bends[0],signedAngle:2*Math.PI/3,neutralLength:4};
  const m={...model,thicknessMm:1.5,bends:[b]};
  const preDie=die('open',12);preDie.contact.openingAngleDegrees=88;
  const prePunch=punch('round',2);prePunch.contact.openingAngleDegrees=88;
  const tools={tools:[punch('sharp',1),prePunch,die('V8',8),preDie]};
  const {options}=resolveToolingRules(m,tools,{dieRules:{},allowPartialBends:true});
  for(const [kind,pre,finish] of [['punch','round','sharp'],['die','open','V8']]){
    assert.deepEqual(advancedCompatibleTools(tools,b,kind,options,1).map(t=>t.id),[finish]);
    assert.ok(advancedCompatibleTools(tools,b,kind,options,.5).some(t=>t.id===pre));
  }
});
