export const toolingSelectionDefaults={maxPlans:3,radiusToOpeningRatio:0.16,radiusTolerance:0.25,radiusToleranceThickness:0.5,
  // Hard limits on V/t; the rule-of-8 band (preferredOpeningThickness) only orders candidates:
  // shops run e.g. 16 ga (V/t 5.3) on an 8 mm die. Min flange 0.707 V (die table: 0.223 in on 0.315 in).
  minOpeningThickness:5,maxOpeningThickness:12,preferredOpeningThickness:[6,10],minFlangeToOpeningRatio:0.707,tensileMpa:null,maxTonnagePerMeterKn:null,
  flangeClearanceRanking:false,allowFlangeFallback:false};

const ready = (catalog, kind) => catalog.tools.filter(t => t.kind === kind && t.status === 'ready');
const permitted = (id, fixed, list) => (fixed == null || String(id) === String(fixed)) &&
  (!Array.isArray(list) || list.some(other => String(other) === String(id)));

function groupedStrokes(model) {
  const bends = model.bends;
  const panels = new Map((model.panels ?? []).map(p => [p.id, p]));
  const degree = new Map();
  for (const b of bends) for (const id of [b.parentPanelId, b.childPanelId]) degree.set(id, (degree.get(id) ?? 0) + 1);
  function terminalFlange(bend) {
    // The 0.707 V design rule is defined for a 90-degree operation. An adjacent
    // still-flat panel can continue supporting an internal flange. Only an outer
    // panel with no further hinge provides a state-independent upper bound.
    if (Math.abs(Math.abs(bend.signedAngle) - Math.PI / 2) > 1e-6 || !bend.transverse || !bend.origin) return Infinity;
    let limit = Infinity;
    for (const id of [bend.parentPanelId, bend.childPanelId]) {
      const points = panels.get(id)?.flatPositions;
      if (degree.get(id) !== 1 || !points?.length) continue;
      let width = 0;
      for (let i = 0; i < points.length; i += 3) {
        width = Math.max(width, Math.abs((points[i] - bend.origin[0]) * bend.transverse[0] +
          (points[i + 1] - bend.origin[1]) * bend.transverse[1]));
      }
      if (Number.isFinite(width)) limit = Math.min(limit, width);
    }
    return limit;
  }
  return bends.filter(b => b.strokeLeadId == null).map(b => {
    const members = [b, ...bends.filter(f => f.strokeLeadId != null && String(f.strokeLeadId) === String(b.id))];
    const included = Math.min(...members.map(m => 180 - Math.abs(m.signedAngle) * 180 / Math.PI));
    // Banks may have different punch segments over a follower. Its nominal
    // radius cannot reject the active punch without a segment-to-span mapping.
    const radius = Number.isFinite(b.insideRadius) ? b.insideRadius : Infinity;
    const flange = Math.min(...members.map(terminalFlange));
    return {bendId: b.id, label: b.label ?? String(b.id), included, radius, flange};
  }).sort((a, b) => a.flange - b.flange);
}

export function ruleAllowedDieIds(model, catalog, settings = {}) {
  return ruleDieDecision(model, catalog, settings).allowedDieIds;
}

export function ruleDieDecision(model, catalog, settings = {}, limits = {}) {
  const s = {...toolingSelectionDefaults, ...settings}, t = model.thicknessMm;
  const dies = ready(catalog, 'die').filter(d => permitted(d.id, limits.dieId, limits.allowedDieIds));
  const punches = ready(catalog, 'punch').filter(p => permitted(p.id, limits.punchId, limits.allowedPunchIds));
  const strokes = groupedStrokes(model), byDie = new Set(), exceptions = new Map(), optionalNarrow = new Map();
  const decision = {mode: 'strict', allowedDieIds: [], byBend: Object.create(null), exceptions: [], blockedBy: [], thicknessMm: t,
    shortestFlangeMm: Number.isFinite(strokes[0]?.flange) ? strokes[0].flange : null,
    minOpeningThickness: s.minOpeningThickness, maxOpeningThickness: s.maxOpeningThickness};
  for (const b of strokes) {
    const allowed = [], fallback = [], rejectedDies = [], partialDies = [];
    const punchIds = punches.filter(p => !(p.contact?.openingAngleDegrees > b.included + 1e-6) &&
      !(p.contact?.tipRadiusMm > b.radius + 1e-6)).map(p => String(p.id));
    for (const d of dies) {
      const v = d.contact?.openingMm ?? d.contact?.geometryOpeningMm, preferred = String(d.id) === String(s.preferredDieId);
      const known = Number.isFinite(v) && v > 0;
      const reasons = [];
      if (d.contact?.openingAngleDegrees > b.included + 1e-6) reasons.push('DIE_ANGLE_TOO_OPEN');
      if (known && v * s.minFlangeToOpeningRatio > b.flange + 1e-6) reasons.push('DIE_MIN_FLANGE_EXCEEDS_PART');
      if (known && s.maxTonnagePerMeterKn > 0 && s.tensileMpa > 0 &&
          1.42 * s.tensileMpa * t * t / v > s.maxTonnagePerMeterKn) reasons.push('TONNAGE_EXCEEDS_LIMIT');
      const ratio = known && v < s.minOpeningThickness * t ? 'DIE_OPENING_BELOW_MIN_THICKNESS_RATIO' :
        known && v > s.maxOpeningThickness * t ? 'DIE_OPENING_ABOVE_MAX_THICKNESS_RATIO' : null;
      // Final included angle and the 90-degree flange bound need not apply to
      // a preparatory stroke. V/t, load and the operator selection still do.
      if (!reasons.includes('TONNAGE_EXCEEDS_LIMIT') && (!ratio || preferred)) partialDies.push(String(d.id));
      if (!reasons.length && !ratio) { allowed.push(String(d.id)); continue; }
      if (!reasons.length && ratio && (preferred || (s.allowFlangeFallback === true && v < s.minOpeningThickness * t))) {
        const why = [ratio];
        if (Number.isFinite(b.radius) && Math.abs(v * s.radiusToOpeningRatio - b.radius) >
            Math.max(b.radius * s.radiusTolerance, t * s.radiusToleranceThickness)) why.push('PREDICTED_RADIUS_OUTSIDE_TOLERANCE');
        const exc = {dieId: String(d.id), dieName: d.name, openingMm: v, openingThicknessRatio: v / t, reasons: why, preferred};
        if (preferred) { allowed.push(exc.dieId); exceptions.set(exc.dieId, exc); }
        else fallback.push(exc);
      }
      if (ratio) reasons.push(ratio);
      rejectedDies.push({dieId: String(d.id), dieName: d.name, openingMm: known ? v : null, reasons});
    }
    if (!allowed.length) for (const exc of fallback) { allowed.push(exc.dieId); exceptions.set(exc.dieId, exc); }
    else if (fallback.length) optionalNarrow.set(b.bendId, fallback);
    if (!allowed.length || !punchIds.length) {
      const blockedBy = new Set();
      if (!punchIds.length) blockedBy.add(punches.length ? 'NO_NOMINALLY_COMPATIBLE_PUNCH' : 'NO_ALLOWED_PUNCH');
      if (!allowed.length) {
        if (!dies.length) blockedBy.add('NO_ALLOWED_DIE');
        for (const d of rejectedDies) for (const why of d.reasons) blockedBy.add(why);
      }
      const blockedBend = {bendId: b.bendId, label: b.label, flangeMm: Number.isFinite(b.flange) ? b.flange : null,
        minOpeningMm: s.minOpeningThickness * t,
        maxOpeningByFlangeMm: Number.isFinite(b.flange) && s.minFlangeToOpeningRatio > 0 ? b.flange / s.minFlangeToOpeningRatio : null,
        rejectedDies, blockedBy: [...blockedBy]};
      return {...decision, mode: 'blocked', blockedBend, blockedBy: [...blockedBy]};
    }
    decision.byBend[b.bendId] = {dieIds: allowed, punchIds, partialDieIds: partialDies};
    for (const id of allowed) byDie.add(id);
  }
  // An explicitly authorized narrow die needed for one operation may remain
  // mounted for other strokes where only the same V/t exception is required.
  for (const [bendId, choices] of optionalNarrow) for (const exc of choices) {
    if (exceptions.has(exc.dieId)) decision.byBend[bendId].dieIds.push(exc.dieId);
  }
  if (limits.allowPartialBends !== false) for (const op of Object.values(decision.byBend)) {
    for (const id of op.dieIds) if (!op.partialDieIds.includes(id)) op.partialDieIds.push(id);
    for (const id of op.partialDieIds) byDie.add(id);
  }
  decision.allowedDieIds = [...byDie];
  decision.exceptions = [...exceptions.values()];
  if (decision.exceptions.length) decision.mode = decision.exceptions.some(e => !e.preferred) ? 'flange-fallback' : 'preferred-exception';
  return decision;
}

export function resolveToolingRules(model, catalog, options = {}) {
  const settings = options.dieRules ?? options.toolingSelection;
  if (!settings || !Array.isArray(catalog?.tools) || !model?.bends?.length || !(model.thicknessMm > 0) ||
      model.bends.some(b => b.id == null || !Number.isFinite(b.signedAngle))) return {options};
  const decision = ruleDieDecision(model, catalog, {...settings, preferredDieId: options.preferredDieId ?? settings.preferredDieId}, options);
  if (decision.mode !== 'blocked') return {decision, options: {...options, allowedDieIds: decision.allowedDieIds, toolingByBend: decision.byBend}};
  const {blockedBend: b} = decision;
  const size = b.flangeMm == null ? '' : ' Flange ' + b.flangeMm.toFixed(2) + ' mm.';
  const reason = b.label + ' cannot be formed with the allowed tooling.' + size + ' ' + b.blockedBy.join(', ');
  return {decision, options, blocked: {status: 'TOOLING_RULES_BLOCKED', reason, stages: [], transfers: [],
    allStagesSampleClear: false, completeGeometricRouteSampleClear: false, completeSequenceVerified: false, completeManufacturingVerified: false,
    dieRuleDecision: decision, diagnostics: {dieRuleDecision: decision, statesExpanded: 0, transitionsChecked: 0, endpointChecks: 0, foldChecks: 0}}};
}

const resolved = Symbol('resolved tooling rules');
export async function withToolingRules(payload, onProgress, search) {
  if (payload.options?.[resolved]) return search(payload, onProgress);
  const {options, decision, blocked} = resolveToolingRules(payload.model, payload.catalog, payload.options);
  if (blocked) return blocked;
  if (!decision) return search(payload, onProgress);
  const plan = await search({...payload, options: {...options, [resolved]: true}}, onProgress);
  return {...plan, dieRuleDecision: decision};
}

export function allowedForBend(toolingByBend, bendId, kind, id, partial = false) {
  const op = toolingByBend?.[bendId];
  const ids = partial ? (kind === 'die' ? op?.partialDieIds : undefined) : op?.[kind + 'Ids'];
  return !ids || ids.includes(String(id));
}

export function nominalBendAt(bend, amount) {
  if (!(amount > 0 && amount < 1)) return bend;
  // bandPoint keeps the neutral length constant. Neutral and inside radii
  // increase by the same amount during a partial bend.
  const angle = Math.abs(bend.signedAngle);
  const growth = angle > 0 && bend.neutralLength > 0 ? bend.neutralLength / angle * (1 / amount - 1) : 0;
  return {...bend, signedAngle: bend.signedAngle * amount, insideRadius: bend.insideRadius + growth};
}

export function toolAllowedAt(tool, bend, options, amount = 1) {
  const kind = tool.kind;
  if (tool.status !== 'ready' || (kind !== 'punch' && kind !== 'die')) return false;
  const list = kind === 'punch' ? options.allowedPunchIds : options.allowedDieIds;
  if (!permitted(tool.id, options[kind + 'Id'], list)) return false;
  const partial = Boolean(options.toolingByBend) && amount > 0 && amount < 1;
  if (!allowedForBend(options.toolingByBend, bend.id, kind, tool.id, partial)) return false;
  const nominal = partial ? nominalBendAt(bend, amount) : bend;
  const angle = tool.contact?.openingAngleDegrees, tip = tool.contact?.tipRadiusMm;
  return (!Number.isFinite(angle) || angle <= 180 - Math.abs(nominal.signedAngle) * 180 / Math.PI + 1e-6) &&
    (kind !== 'punch' || !Number.isFinite(tip) || tip <= nominal.insideRadius + 1e-6);
}
