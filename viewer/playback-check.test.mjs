import test from 'node:test';
import assert from 'node:assert/strict';
import {machinePlaybackDecision, playbackSetupKey} from './playback-check.mjs';

const clear = progress => ({progress, status: 'CLEAR', pairs: []});
const hit = (progress, pair) => ({progress, status: 'INTERFERENCE', pairs: [pair]});

test('a collision between clear endpoints prevents playback and selects the earliest actual sample', () => {
  const result = {status: 'INTERFERENCE', samples: [clear(1), hit(.75, ['panel:p2', 'punch-1']),
    clear(0), hit(.25, ['panel:p1', 'die-1']), clear(.5)]};
  const before = JSON.stringify(result);
  const decision = machinePlaybackDecision(result);
  assert.equal(decision.action, 'STOP_AT_SAMPLE');
  assert.deepEqual(decision.firstCollision, {progress: .25, pairs: [['panel:p1', 'die-1']]});
  assert.equal(decision.continuousCollisionDetection, false);
  assert.equal(JSON.stringify(result), before);
});

test('a clear full-cycle result authorizes sampled playback, not CCD', () => {
  const decision = machinePlaybackDecision({status: 'SAMPLE_CLEAR', samples: Array.from({length: 17}, (_, i) => clear(i / 16))});
  assert.deepEqual(decision, {action: 'PLAY_SAMPLED', sampleCount: 17, continuousCollisionDetection: false});
});

test('missing endpoints, empty scans, invalid samples and inconsistent summaries cannot authorize play', () => {
  for (const result of [null, {status: 'SAMPLE_CLEAR', samples: []},
    {status: 'SAMPLE_CLEAR', samples: [clear(.5), clear(1)]},
    {status: 'SAMPLE_CLEAR', samples: [clear(0), clear(.5)]},
    {status: 'INTERFERENCE', samples: [clear(0), clear(1)]},
    {status: 'SAMPLE_CLEAR', samples: [clear(0), clear(NaN), clear(1)]},
    {status: 'SAMPLE_CLEAR', samples: [clear(0), clear(0), clear(1)]}])
    assert.throws(() => machinePlaybackDecision(result));
  assert.equal(machinePlaybackDecision({status: 'SAMPLE_CLEAR', samples: [clear(0), hit(.5, ['band:1', 'table']), clear(1)]}).action, 'STOP_AT_SAMPLE');
});

test('a sampled cycle applies across progress changes but not to another part setup or order', () => {
  const setup = {bendId: '2', order: ['1', '2'], progress: 0, punchId: 'p1', dieId: 'd1', flip: false,
    bedLengthMm: 838.2, segments: [{id: 'die-1', kind: 'die', lengthMm: 50.8, centerY: 0}]};
  const key = playbackSetupKey(setup);
  assert.equal(playbackSetupKey({...setup, progress: .73}), key);
  for (const change of [{punchId: 'p2'}, {dieId: 'd2'}, {bendId: '1'}, {flip: true}, {order: ['2', '1']},
    {bedLengthMm: 700}, {segments: [{...setup.segments[0], centerY: 8}]}, {segments: [{...setup.segments[0], lengthMm: 25.4}]}])
    assert.notEqual(playbackSetupKey({...setup, ...change}), key);
});
