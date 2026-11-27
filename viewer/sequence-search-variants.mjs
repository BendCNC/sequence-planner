/** Experimental search candidates. These do not change collision or support tolerances.
 * Axial edge/centre trials are inspired by open-mrp/dfm (Apache-2.0).
 * No upstream implementation is copied. Every generated pose needs full validation.
 */
export function axialPlacementVariants(config, materialIntervals) {
  if (config.setupFamily === 'TWO_BANK' || !materialIntervals.length) return [config];
  const banks = new Set(config.segments.map(segment => segment.bankId ?? 'single'));
  if (banks.size > 1) return [config];
  const limits = ['punch', 'die'].map(kind => {
    const stock = config.segments.filter(segment => segment.kind === kind);
    return stock.length ? [Math.min(...stock.map(s => s.centerY - s.lengthMm / 2)),
      Math.max(...stock.map(s => s.centerY + s.lengthMm / 2))] : null;
  });
  if (limits.some(value => !value)) return [config];
  const low = Math.max(...limits.map(value => value[0]));
  const high = Math.min(...limits.map(value => value[1]));
  const materialLow = Math.min(...materialIntervals.map(value => value[0]));
  const materialHigh = Math.max(...materialIntervals.map(value => value[1]));
  const values = [config.stationYmm ?? 0, low - materialLow, high - materialHigh,
    (low + high - materialLow - materialHigh) / 2];
  const seen = new Set();
  return values.flatMap((stationYmm, index) => {
    const key = stationYmm.toFixed(7);
    if (!Number.isFinite(stationYmm) || seen.has(key)) return [];
    seen.add(key);
    return [{...config, stationYmm, ...(index ? {placementTrial: 'AXIAL_EDGE_OR_CENTRE'} : {})}];
  });
}

export function partialBendTargets(bend, current, allowPartial, options) {
  const finalAngle = Math.abs(bend.signedAngle) * 180 / Math.PI;
  const targets = [1];
  if (allowPartial && current < 1e-8) {
    const angles = options.partialAnglesDegrees ?? [options.partialAngleDegrees];
    for (const angle of angles) {
      if (!Number.isFinite(angle) || angle <= 0) throw new Error('Invalid partial bend angle');
      const fraction = angle / finalAngle;
      if (fraction > current + 1e-8 && fraction < 1 - 1e-8 &&
          !targets.some(value => Math.abs(value - fraction) < 1e-8)) targets.push(fraction);
    }
  }
  return targets;
}

/** Allocate a phase from what remains at its actual start, never from time
 * already spent by another search. Global limits remain authoritative. */
export function remainingPhaseQuota(current, limits, fraction) {
  if (!(fraction > 0 && fraction <= 1)) throw new RangeError('Invalid search phase fraction');
  const count=(used,total)=>Math.min(total,used+Math.max(0,Math.min(total-used,Math.ceil((total-used)*fraction))));
  return {transitions:count(current.transitions,limits.transitions),states:count(current.states,limits.states),
    until:Math.min(limits.until,current.now+Math.max(0,limits.until-current.now)*fraction)};
}
