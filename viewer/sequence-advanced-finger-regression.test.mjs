/** Actual saved baseline regressions. This test never reruns the planner or
 * edits a fixture; only its completed output/evidence are written to the lab. */
import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync,writeFileSync,mkdirSync,existsSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {fileURLToPath} from 'node:url';
import {completeLegacyFingerTransfers} from './sequence-portfolio.mjs';
import {verifyAdvancedContinuity} from './sequence-advanced-completion.mjs';
import {buildSequenceTimeline,sequenceSceneAt} from './sequence-playback.mjs';
import {meshCollider,checkPair} from './collision.mjs';
const lab=new URL('../../',import.meta.url),path=p=>fileURLToPath(new URL(p,lab));
const read=p=>JSON.parse(readFileSync(path(p),'utf8')),sha=p=>createHash('sha256').update(readFileSync(path(p))).digest('hex');
const inventoryPath='results/round6/benchmark/baseline-120-finger-continuity.json';
const available=existsSync(path(inventoryPath));
const cases=available?read(inventoryPath).records.filter(r=>r.missingAdjustmentCount):[];
const output='results/round6/legacy-finger-repair';
// Explicit recording avoids changing the exact files while another process
// independently replays/hashes a previous run's evidence.
const recordEvidence=process.env.UNFOLD_WRITE_LEGACY_FINGER_EVIDENCE==='1';
const rows=[];
const unchangedFields=plan=>plan.stages.map(s=>({id:s.id,bendId:s.bendId,from:s.from,to:s.to,config:s.config,foldSamples:s.foldSamples,foldSamplesClear:s.foldSamplesClear}));
test('laboratory regression inventory has the eight reported affected baseline routes',{skip:!available},()=>assert.equal(cases.length,8));
for(const item of cases)test('legacy fingers become continuous without changing folds: '+item.caseId,{timeout:120000},()=>{
 const modelPath='corpus/models/'+item.caseId+'.json',source='results/round6/benchmark/baseline-120/'+item.caseId+'.plan.json';
 const model=read(modelPath),catalog=read('candidate-v4/viewer/tooling.json'),options=read('results/round6/benchmark/baseline-120/options.json'),original=read(source),before=JSON.stringify(original),started=performance.now();
 catalog.fingerDefinition=read('candidate-v4/viewer/fingers.json').definitions[0];
 const prior=verifyAdvancedContinuity(model,catalog,original);
 assert.equal(original.completeGeometricRouteSampleClear,true);assert.equal(prior.status,'DISCONTINUOUS');
 const repaired=completeLegacyFingerTransfers(model,catalog,original,options);
 const resultPath=output+'/'+item.caseId+'.plan.json',serialized=JSON.stringify(repaired,null,2);
 if(recordEvidence){mkdirSync(path(output),{recursive:true});writeFileSync(path(resultPath),serialized);}
 const continuity=verifyAdvancedContinuity(model,catalog,repaired);
 const checks=[];let sampledFingerPairs=0,minimumMutualGapMm=Infinity;
 if(repaired.completeGeometricRouteSampleClear)for(const [i,transfer]of repaired.transfers.entries()){
  const proof=transfer.fingerAdjustment;assert.equal(proof.status,'EXTERIOR_MANUAL_Y_ADJUSTMENT_GEOMETRY_CLEAR');
  assert.ok(proof.proof.toolSeparationMm>.15);assert.ok(proof.proof.partSeparationMm>.15);
  if(proof.proof.minimumMutualYGapMm!==null)minimumMutualGapMm=Math.min(minimumMutualGapMm,proof.proof.minimumMutualYGapMm);
  for(const q of [0,.1,.25,.5,.75,.9,1]){const scene=sequenceSceneAt(model,catalog,repaired,i,'transfer',q),fingers=scene.bodies.filter(b=>b.kind==='finger');
   assert.equal(fingers.length,repaired.stages[i].backgauge.stationY.length);
   if(fingers.length===2){const pair=checkPair(...fingers.map(meshCollider),{clearanceMm:.15,queryMode:'clearance'});assert.equal(pair.status,'CLEAR');sampledFingerPairs++;}}
  checks.push({index:i,status:proof.status,...proof.proof});
 }
 const row={caseId:item.caseId,modelPath:path(modelPath),sourcePlanPath:path(source),sourcePlanSha256:sha(source),planPath:path(resultPath),planSha256:createHash('sha256').update(serialized).digest('hex'),
  accepted:repaired.completeGeometricRouteSampleClear===true,status:repaired.status,reason:repaired.reason??null,stages:repaired.stages.length,
  originalFingerCounts:original.stages.map(s=>s.backgauge.stationY.length),repairedFingerCounts:repaired.stages.map(s=>s.backgauge.stationY.length),
  originalContinuity:prior,repairedContinuity:continuity,foldsAndSetupsUnchanged:JSON.stringify(unchangedFields(original))===JSON.stringify(unchangedFields(repaired)),
  originalInputUnchanged:JSON.stringify(original)===before,physicalFingerCompletion:repaired.fingerLayoutCompletion??[],
  transferProofs:checks,sampledMutualFingerPairs:sampledFingerPairs,minimumMutualYGapMm:Number.isFinite(minimumMutualGapMm)?minimumMutualGapMm:null,
  elapsedMs:performance.now()-started,independentFullSceneValidation:'PENDING'};
 rows.push(row);if(recordEvidence){writeFileSync(path(output+'/'+item.caseId+'.qa.json'),JSON.stringify(row,null,2));
  writeFileSync(path(output+'/summary.json'),JSON.stringify({complete:rows.length===cases.length,expected:cases.length,checked:rows.length,accepted:rows.filter(r=>r.accepted).length,rows,
   scope:'Repairs only exterior endpoints, gauge parking/layout and explicit manual finger transfers. All original bend amounts, order, fold samples and tooling configs retained. Independent full-scene replay remains separate.'},null,2));}
 assert.equal(row.originalInputUnchanged,true);assert.equal(row.foldsAndSetupsUnchanged,true);assert.equal(row.accepted,true,row.reason);assert.equal(continuity.status,'CONTINUOUS');assert.ok(continuity.maximumErrorMm<1e-6);
 assert.equal(repaired.legacyFingerTransferCompletion?.accepted,true);
 for(const completion of row.physicalFingerCompletion){const gauge=repaired.stages[completion.stageIndex].backgauge,index=gauge.stationY.findIndex(y=>Math.abs(y-completion.primaryY)<1e-8);
  assert.equal(gauge.stationY.length,2);assert.ok(index>=0);assert.equal(gauge.contacts[index].used,true);assert.equal(gauge.status,'SAMPLE_CLEAR');}
});
