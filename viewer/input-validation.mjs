// Input validation against the folded CAD reference, before any sequence search.
// The declared bend table (angle per bend) must reproduce the CAD body. When it
// does not, try reassignments of the declared angles (swaps between bends and
// sign flips). A reassignment that matches is reported as an input error that
// names the bends involved; no reassignment is applied to the plan.
import {compareWithReference} from './reference-shape.mjs';

const DEG = 180 / Math.PI;
const deg = r => Math.round(Math.abs(r) * DEG * 10) / 10;

function withAngles(model, angles) {
  return {...model, bends: model.bends.map(b => ({...b, signedAngle: angles[b.id]}))};
}

function* permutations(items) {
  if (items.length <= 1) { yield items.slice(); return; }
  for (let i = 0; i < items.length; i++) {
    const rest = items.slice(0, i).concat(items.slice(i + 1));
    for (const p of permutations(rest)) yield [items[i], ...p];
  }
}

/** Alternative angle tables: magnitude permutations (n<=5, all; otherwise pair
 * swaps) keeping each bend's direction, then single sign flips. */
export function angleAlternatives(model) {
  const bends = model.bends, out = [], seen = new Set();
  const declared = Object.fromEntries(bends.map(b => [b.id, b.signedAngle]));
  const key = a => bends.map(b => a[b.id].toFixed(6)).join(',');
  seen.add(key(declared));
  const push = (angles, kind) => { const k = key(angles); if (!seen.has(k)) { seen.add(k); out.push({kind, angles}); } };
  const mags = bends.map(b => Math.abs(b.signedAngle));
  const orders = bends.length <= 5 ? permutations(bends.map((_, i) => i)) :
    (function* () { for (let i = 0; i < bends.length; i++) for (let j = i + 1; j < bends.length; j++) {
      const p = bends.map((_, k) => k); p[i] = j; p[j] = i; yield p; } })();
  for (const p of orders) push(Object.fromEntries(bends.map((b, i) => [b.id, Math.sign(b.signedAngle || 1) * mags[p[i]]])), 'ANGLE_REASSIGNMENT');
  for (const b of bends) push({...declared, [b.id]: -declared[b.id]}, 'DIRECTION_FLIP');
  return {declared, alternatives: out};
}

export function consistentInputFromComparison(model,base) {
  const near = Math.max(1.5 * (model.thicknessMm || 1), 1);
  return base.pass || base.maxDeviationMm <= near
    ? {status:'INPUT_CONSISTENT', checked:true, maxDeviationMm:base.maxDeviationMm} : null;
}

export function validateInputAgainstReference(model, reference, {toleranceMm, maxAlternatives = 130} = {}) {
  if (!reference?.positions?.length) return {status: 'NO_REFERENCE', checked: false};
  if (!model.bends?.length) return {status: 'NO_BENDS', checked: false};
  const ones = Object.fromEntries(model.bends.map(b => [b.id, 1]));
  const base = compareWithReference(model, ones, reference, {toleranceMm});
  // Gross errors only: deviations within about one sheet thickness come from bend
  // allowance/radius modelling and are judged by the final-shape check instead.
  const near = Math.max(1.5 * (model.thicknessMm || 1), 1);
  const consistent=consistentInputFromComparison(model,base);
  if (consistent) return consistent;
  const {declared, alternatives} = angleAlternatives(model);
  let best = null, tried = 0;
  for (const alt of alternatives.slice(0, maxAlternatives)) {
    tried++;
    const r = compareWithReference(withAngles(model, alt.angles), ones, reference, {toleranceMm});
    if (!best || r.maxDeviationMm < best.result.maxDeviationMm) best = {...alt, result: r};
  }
  // The flat pattern keeps the declared bend allowances, so a reassigned angle
  // cannot reach the strict tolerance; require a near-thickness fit and a large gain.
  const explains = best && (best.result.pass ||
    (best.result.maxDeviationMm <= near && best.result.maxDeviationMm <= 0.2 * base.maxDeviationMm));
  const label = b => b.label ?? b.id;
  if (explains) {
    const changed = model.bends.filter(b => Math.abs(best.angles[b.id] - declared[b.id]) > 1e-9);
    const bends = changed.map(b => ({bendId: b.id, label: label(b), declaredDeg: deg(declared[b.id]),
      cadDeg: deg(best.angles[b.id]), directionFlipped: Math.sign(best.angles[b.id]) !== Math.sign(declared[b.id])}));
    const text = best.kind === 'DIRECTION_FLIP'
      ? `${bends[0].label} is declared in the opposite direction to the 3D model.`
      : 'The bend table does not match the 3D model: ' + bends.map(b => `${b.label} is declared ${b.declaredDeg} deg but the 3D model has ${b.cadDeg} deg`).join('; ') + '.';
    return {status: 'INPUT_MISMATCH', checked: true, kind: best.kind, bends, maxDeviationMm: base.maxDeviationMm,
      alternativeMaxDeviationMm: best.result.maxDeviationMm, alternativesTried: tried,
      message: text + ' Check the bend angles in the bend table against the 3D model before planning.'};
  }
  return {status: 'SHAPE_MISMATCH_UNEXPLAINED', checked: true, maxDeviationMm: base.maxDeviationMm,
    suspectBends: base.suspectBends ?? [], alternativesTried: tried,
    bestAlternativeMaxDeviationMm: best?.result.maxDeviationMm ?? null,
    message: 'The flat pattern folded with the declared angles does not match the 3D model, and no swap or direction change of the declared angles explains it. Recreate the flat pattern or check the bend lines.'};
}
