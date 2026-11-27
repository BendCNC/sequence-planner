/** Tool support along the station axis, in millimetres. No collision or load claim. */
const KINDS = ['punch', 'die'];

function union(intervals, epsilon) {
  const result = [];
  for (const interval of intervals.map(v => v.slice()).sort((a, b) => a[0] - b[0] || a[1] - b[1])) {
    const last = result.at(-1);
    if (last && interval[0] <= last[1] + epsilon) last[1] = Math.max(last[1], interval[1]);
    else result.push(interval);
  }
  return result;
}

function subtract(material, support, epsilon) {
  const missing = [];
  for (const [start, end] of material) {
    let cursor = start;
    for (const [lo, hi] of support) {
      if (hi <= cursor) continue;
      if (lo >= end) break;
      if (lo > cursor + epsilon) missing.push([cursor, Math.min(lo, end)]);
      cursor = Math.max(cursor, hi);
      if (cursor >= end) break;
    }
    if (end > cursor + epsilon) missing.push([cursor, end]);
  }
  return missing;
}

function intersection(material, support, epsilon) {
  const result = [];
  for (const [a, b] of material) for (const [c, d] of support) {
    const lo = Math.max(a, c), hi = Math.min(b, d);
    if (hi > lo + epsilon) result.push([lo, hi]);
  }
  return union(result, epsilon);
}

/**
 * Check both punch and die coverage of materialIntervals [[minY,maxY], ...].
 * A joint gap up to jointGapToleranceMm is tolerated only when strictly inside
 * a material interval: no padding is added to unsupported material boundaries.
 * missingIntervals are exact interval subtraction after those accepted joints
 * (up to epsilonMm), not bounds or sampled stations. physicalMissingIntervals
 * also reports tolerated joint openings before applying the joint allowance.
 *
 * exact=false means the supplied material intervals are approximate/fallback.
 * Then status is UNKNOWN and covered is null; estimatedCovered and intervals
 * retain useful diagnostics without declaring support verified.
 */
export function assessCoverage({materialIntervals, segments, exact = true, activeBankId,
  jointGapToleranceMm = .5, epsilonMm = 1e-7} = {}) {
  if (!Array.isArray(materialIntervals) || !Array.isArray(segments))
    throw new TypeError('materialIntervals and segments must be arrays.');
  if (typeof exact !== 'boolean') throw new TypeError('exact must be boolean.');
  for (const [name, value] of [['jointGapToleranceMm', jointGapToleranceMm], ['epsilonMm', epsilonMm]])
    if (!Number.isFinite(value) || value < 0) throw new RangeError(`${name} must be finite and nonnegative.`);
  const required = materialIntervals.map(interval => {
    if (!Array.isArray(interval) || interval.length !== 2 || !interval.every(Number.isFinite) || interval[1] < interval[0])
      throw new RangeError('Each material interval must be a finite ordered pair.');
    return interval.slice();
  }).filter(([lo, hi]) => hi > lo + epsilonMm);
  const material = union(required, epsilonMm);
  const byKind = {};
  const supports = {punch: [], die: []};
  for (const segment of segments) {
    if (!segment || !KINDS.includes(segment.kind)) throw new TypeError('Segment kind must be punch or die.');
    if (activeBankId != null && segment.bankId !== activeBankId) continue;
    if (!Number.isFinite(segment.lengthMm) || segment.lengthMm <= 0 || !Number.isFinite(segment.centerY))
      throw new RangeError('Segment lengthMm must be positive and centerY must be finite.');
    const lo = segment.centerY - segment.lengthMm / 2, hi = segment.centerY + segment.lengthMm / 2;
    if (!Number.isFinite(lo) || !Number.isFinite(hi)) throw new RangeError('Segment interval overflow.');
    supports[segment.kind].push([lo, hi]);
  }
  for (const kind of KINDS) {
    const physicalIntervals = union(supports[kind], epsilonMm);
    const effectiveIntervals = [], acceptedJointGaps = [];
    for (const interval of physicalIntervals) {
      const last = effectiveIntervals.at(-1);
      if (last) {
        const gap = [last[1], interval[0]], size = gap[1] - gap[0];
        // Both ends must be internal, so an off-part segment cannot make a
        // short unsupported outer edge appear covered via the joint tolerance.
        const internal = material.some(([lo, hi]) => gap[0] > lo + epsilonMm && gap[1] < hi - epsilonMm);
        if (internal && size <= jointGapToleranceMm + epsilonMm) {
          acceptedJointGaps.push(gap);
          last[1] = interval[1];
          continue;
        }
      }
      effectiveIntervals.push(interval.slice());
    }
    const missingIntervals = subtract(material, effectiveIntervals, epsilonMm);
    const complete = missingIntervals.length === 0;
    byKind[kind] = {
      covered: exact ? complete : null,
      estimatedCovered: complete,
      status: exact ? (complete ? 'COVERED' : 'INCOMPLETE') : 'UNKNOWN',
      missingIntervals,
      coveredIntervals: intersection(material, effectiveIntervals, epsilonMm),
      physicalMissingIntervals: subtract(material, physicalIntervals, epsilonMm),
      physicalIntervals, effectiveIntervals, acceptedJointGaps,
    };
  }
  return {
    status: !exact ? 'UNKNOWN' : KINDS.every(kind => byKind[kind].covered) ? 'COVERED' : 'INCOMPLETE',
    exact, materialIntervals: material, jointGapToleranceMm, epsilonMm, byKind,
  };
}
