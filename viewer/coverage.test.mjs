import test from 'node:test';
import assert from 'node:assert/strict';
import {assessCoverage} from './coverage.mjs';

const segment = (kind, lo, hi) => ({kind, lengthMm: hi - lo, centerY: (lo + hi) / 2});
const both = intervals => ['punch', 'die'].flatMap(kind => intervals.map(([lo, hi]) => segment(kind, lo, hi)));
const check = (material, support, extra = {}) => assessCoverage({materialIntervals: material, segments: both(support), ...extra});

test('both tool kinds must cover every material interval', () => {
  const result = assessCoverage({materialIntervals: [[0, 10]], segments: [segment('punch', -2, 12), segment('die', 0, 6)]});
  assert.equal(result.status, 'INCOMPLETE');
  assert.equal(result.byKind.punch.covered, true);
  assert.equal(result.byKind.die.covered, false);
  assert.deepEqual(result.byKind.punch.missingIntervals, []);
  assert.deepEqual(result.byKind.die.missingIntervals, [[6, 10]]);
});

test('unions overlapping and unordered segments without double-counting coverage', () => {
  const result = check([[4, 10], [0, 5]], [[6, 10], [-2, 4], [3, 8]]);
  assert.equal(result.status, 'COVERED');
  assert.deepEqual(result.materialIntervals, [[0, 10]]);
  assert.deepEqual(result.byKind.punch.physicalIntervals, [[-2, 10]]);
  assert.deepEqual(result.byKind.punch.coveredIntervals, [[0, 10]]);
});

test('tolerates only internal joints up to the requested gap', () => {
  const atLimit = check([[0, 10]], [[0, 4], [4.5, 10]]);
  assert.equal(atLimit.status, 'COVERED');
  assert.deepEqual(atLimit.byKind.punch.acceptedJointGaps, [[4, 4.5]]);
  assert.deepEqual(atLimit.byKind.punch.physicalMissingIntervals, [[4, 4.5]]);
  assert.deepEqual(atLimit.byKind.punch.missingIntervals, []);
  const overLimit = check([[0, 10]], [[0, 4], [4.50001, 10]]);
  assert.equal(overLimit.status, 'INCOMPLETE');
  assert.deepEqual(overLimit.byKind.punch.missingIntervals, [[4, 4.50001]]);
  assert.equal(check([[0, 10]], [[0, 4], [4.4, 10]], {jointGapToleranceMm: 0}).status, 'INCOMPLETE');
});

test('never extends support by joint tolerance at an outer material boundary', () => {
  const result = check([[0, 10]], [[.2, 9.7]]);
  assert.equal(result.status, 'INCOMPLETE');
  assert.equal(result.byKind.punch.missingIntervals.length, 2);
  assert.equal(result.byKind.punch.missingIntervals[0][0], 0);
  assert.ok(Math.abs(result.byKind.punch.missingIntervals[0][1] - .2) < 1e-12);
  assert.deepEqual(result.byKind.punch.missingIntervals[1], [9.7, 10]);
  // Adjacent tooling outside the part must not hide those boundary gaps.
  const bracketed = check([[0, 10]], [[-2, -.1], [.2, 9.7], [10.1, 12]]);
  assert.equal(bracketed.status, 'INCOMPLETE');
  assert.deepEqual(bracketed.byKind.punch.acceptedJointGaps, []);
  assert.ok(Math.abs(bracketed.byKind.punch.missingIntervals[0][1] - .2) < 1e-12);
  assert.deepEqual(bracketed.byKind.punch.missingIntervals[1], [9.7, 10]);
});

test('material openings remain separate; boundary gaps beside an opening are not internal joints', () => {
  const result = check([[0, 3], [7, 10]], [[0, 3], [7, 10]]);
  assert.equal(result.status, 'COVERED');
  assert.deepEqual(result.byKind.punch.coveredIntervals, [[0, 3], [7, 10]]);
  const partial = check([[0, 3], [7, 10]], [[0, 2.8], [3.2, 10]]);
  assert.equal(partial.status, 'INCOMPLETE');
  assert.deepEqual(partial.byKind.punch.missingIntervals, [[2.8, 3]]);
});

test('reports all exact residual intervals rather than one enclosing bound', () => {
  const result = check([[-10, 10], [20, 30]], [[-8, -5], [-2, 4], [7, 10], [22, 28]]);
  assert.deepEqual(result.byKind.die.missingIntervals, [[-10, -8], [-5, -2], [4, 7], [20, 22], [28, 30]]);
});

test('a tiny numerical epsilon is distinct from the joint allowance', () => {
  assert.equal(check([[0, 10]], [[5e-8, 10 - 5e-8]]).status, 'COVERED');
  const result = check([[0, 10]], [[1e-5, 10]]);
  assert.equal(result.status, 'INCOMPLETE');
  assert.ok(result.byKind.die.missingIntervals[0][1] > 9e-6);
});

test('approximate material intervals always return UNKNOWN with explicit estimates', () => {
  const full = check([[0, 10]], [[0, 10]], {exact: false});
  assert.equal(full.status, 'UNKNOWN');
  assert.equal(full.byKind.punch.covered, null);
  assert.equal(full.byKind.punch.estimatedCovered, true);
  const partial = check([[0, 10]], [[0, 5]], {exact: false});
  assert.equal(partial.status, 'UNKNOWN');
  assert.equal(partial.byKind.die.status, 'UNKNOWN');
  assert.equal(partial.byKind.die.estimatedCovered, false);
  assert.deepEqual(partial.byKind.die.missingIntervals, [[5, 10]]);
});

test('empty stock and empty material are defined, and input objects are unchanged', () => {
  assert.deepEqual(check([[0, 10]], []).byKind.punch.missingIntervals, [[0, 10]]);
  assert.equal(check([], []).status, 'COVERED');
  const options = {materialIntervals: [[5, 10], [0, 5]], segments: both([[0, 10]])};
  const before = JSON.stringify(options);
  assessCoverage(options);
  assert.equal(JSON.stringify(options), before);
});

test('invalid geometry and tolerances fail explicitly', () => {
  for (const material of [[[2, 1]], [[0, NaN]], [[0]], [null]]) assert.throws(() => check(material, []), RangeError);
  for (const invalid of [{kind: 'table', lengthMm: 10, centerY: 0}, {kind: 'punch', lengthMm: 0, centerY: 0},
    {kind: 'die', lengthMm: 10, centerY: Infinity}])
    assert.throws(() => assessCoverage({materialIntervals: [[0, 1]], segments: [invalid]}));
  assert.throws(() => check([[0, 1]], [], {jointGapToleranceMm: -.1}), RangeError);
  assert.throws(() => check([[0, 1]], [], {epsilonMm: NaN}), RangeError);
  assert.throws(() => check([[0, 1]], [], {exact: 'false'}), TypeError);
});
