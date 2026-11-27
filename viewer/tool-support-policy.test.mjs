import {unprotect} from './protected-json.mjs';
import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {assessCoverage} from './coverage.mjs';
import {assessToolSupport,toolSupportPolicy} from './tool-support-policy.mjs';
import {generateToolSetupCandidates} from './tool-setup-search.mjs';
const both=(lengthMm,centerY=0)=>['punch','die'].map(kind=>({kind,lengthMm,centerY}));
const coverage=(materialIntervals,segments,exact=true)=>assessCoverage({materialIntervals,segments,exact});

test('8 inches under a 9 inch bend uses explicit 0.5 inch ends without falsifying coverage',()=>{
  const measured=coverage([[-114.3,114.3]],both(203.2)),support=assessToolSupport(measured);
  assert.equal(measured.status,'INCOMPLETE');assert.equal(support.status,'ALLOWED_END_GAPS');
  assert.equal(support.fullCoverage,false);assert.equal(support.accepted,true);
  assert.ok(Math.abs(support.byKind.punch.leftMm-12.7)<1e-8);assert.ok(Math.abs(support.byKind.die.rightMm-12.7)<1e-8);
  assert.equal(assessToolSupport(measured,{maxUnsupportedEndMm:0}).accepted,false);
  assert.equal(assessToolSupport(measured,{allowPartialCoverage:false}).accepted,false);
});

test('end allowance cannot excuse a tiny tool, an interior gap, a missing island or uncertainty',()=>{
  for(const measured of [coverage([[-123.56,123.56]],both(50.8)),
    coverage([[-100,100]],[...both(99,-50.5),...both(99,50.5)]),
    coverage([[-15,-10],[0,20]],both(20,10)),coverage([[-114.3,114.3]],both(203.2),false),
    coverage([[-114.3,114.3]],both(203.2,1))])assert.equal(assessToolSupport(measured).accepted,false);
  assert.throws(()=>toolSupportPolicy({maxUnsupportedEndMm:NaN}),/finite/);
});

test('the stock generator includes real 8 inch segments and near-span combinations',()=>{
  const catalog=unprotect(readFileSync(new URL('./tooling.json.bct',import.meta.url)));
  const bend={signedAngle:Math.PI/2,insideRadius:1.3462};
  const generated=generateToolSetupCandidates({catalog,bend,materialIntervals:[[-114.3,114.3]],maxUnsupportedEndMm:12.7});
  assert.ok(generated.candidates.some(c=>c.config.segments.length===2&&c.config.segments.every(s=>s.lengthMm===203.2)&&assessToolSupport(c.coverage).accepted));
  const longer=generateToolSetupCandidates({catalog,bend,materialIntervals:[[-123.5598,123.5598]],maxUnsupportedEndMm:12.7});
  assert.ok(longer.candidates.some(c=>['punch','die'].every(k=>c.banks[k].lengths.length===2&&c.banks[k].lengths.includes(203.2)&&c.banks[k].lengths.includes(25.4))&&assessToolSupport(c.coverage).accepted));
});
