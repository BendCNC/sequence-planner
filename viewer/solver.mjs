/** Pure kinematic solver. Millimetres, radians, row-major matrices / column vectors. */
export const identity = () => [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
export const clamp01 = x => Math.max(0, Math.min(1, Number(x) || 0));
export const add = (a, b) => a.map((v, i) => v + b[i]);
export const scale = (a, s) => a.map(v => v * s);
export const sub = (a, b) => a.map((v, i) => v - b[i]);
export const norm = a => Math.hypot(...a);
export const distance = (a, b) => norm(sub(a, b));

export function multiply(a, b) {
  const c = new Array(16).fill(0);
  for (let i = 0; i < 4; i++) for (let j = 0; j < 4; j++)
    for (let k = 0; k < 4; k++) c[4 * i + j] += a[4 * i + k] * b[4 * k + j];
  return c;
}
export function transform(m, p) {
  return [0, 1, 2].map(i => m[i * 4] * p[0] + m[i * 4 + 1] * p[1] + m[i * 4 + 2] * p[2] + m[i * 4 + 3]);
}
export function rotation(axis, theta) {
  const [x, y, z] = axis, c = Math.cos(theta), s = Math.sin(theta), v = 1 - c;
  return [x*x*v+c, x*y*v-z*s, x*z*v+y*s, 0,
          y*x*v+z*s, y*y*v+c, y*z*v-x*s, 0,
          z*x*v-y*s, z*y*v+x*s, z*z*v+c, 0, 0, 0, 0, 1];
}
export function sinc(x) {
  if (Math.abs(x) < 1e-4) { const x2 = x*x; return 1 - x2/6 + x2*x2/120; }
  return Math.sin(x)/x;
}
export function cosc(x) {
  if (Math.abs(x) < 1e-4) { const x2 = x*x; return x/2 - x*x2/24 + x*x2*x2/720; }
  return (1-Math.cos(x))/x;
}

/** A material point in a finite bend strip. Its neutral-axis length stays fixed. */
export function bandPoint(bend, axial, fraction, zFlat, amount) {
  const phi = bend.signedAngle * clamp01(amount), angle = phi * fraction;
  const arc = bend.neutralLength * fraction, c = Math.cos(angle), s = Math.sin(angle);
  const b = arc * sinc(angle) - (zFlat - bend.neutralZ) * s;
  const n = bend.neutralZ + arc * cosc(angle) + (zFlat - bend.neutralZ) * c;
  return add(add(add(bend.origin, scale(bend.axis, axial)), scale(bend.transverse, b)), scale(bend.normal, n));
}

// Fold one strip into a single mesh buffer. A section at the same fraction
// has the same angle at every axial coordinate and through the sheet thickness.
function foldedBandPositions(bend, amount, parent) {
  const data = bend.band.parameters;
  const positions = new Float64Array(data.length);
  const {origin, axis, transverse, normal, neutralZ, neutralLength} = bend;
  if (data.length % 3 || ![origin, axis, transverse, normal].every(v => Array.isArray(v) && v.length === 3)) {
    // Typed input vectors round intermediate results in bandPoint's array operations.
    for (let i = 0; i < data.length; i += 3)
      positions.set(transform(parent, bandPoint(bend, data[i], data[i+1], data[i+2], amount)), i);
    return positions;
  }
  const phi = bend.signedAngle * amount;
  let lastFraction, sine, cosine, arcSinc, arcCosc;
  for (let i = 0; i < data.length; i += 3) {
    const axial = data[i], fraction = data[i+1], height = data[i+2] - neutralZ;
    if (i === 0 || !Object.is(fraction, lastFraction)) {
      const angle = phi * fraction;
      const arc = neutralLength * fraction;
      sine = Math.sin(angle);
      cosine = Math.cos(angle);
      arcSinc = arc * sinc(angle);
      arcCosc = arc * cosc(angle);
      lastFraction = fraction;
    }
    const b = arcSinc - height * sine;
    const n = neutralZ + arcCosc + height * cosine;
    for (let k = 0; k < 3; k++)
      positions[i+k] = origin[k] + axis[k]*axial + transverse[k]*b + normal[k]*n;
  }
  return transformPositions(positions, parent, positions);
}

/** Rigid child motion attached at the moving tangent, in the common flat frame. */
export function jointTransform(bend, amount) {
  if (clamp01(amount) === 0) return identity();
  const r = rotation(bend.axis, bend.signedAngle * clamp01(amount));
  const a = add(add(bend.origin, scale(bend.transverse, bend.neutralLength)), scale(bend.normal, bend.neutralZ));
  const e = bandPoint(bend, 0, 1, bend.neutralZ, amount);
  const t = sub(e, transform(r, a));
  r[3] = t[0]; r[7] = t[1]; r[11] = t[2];
  return r;
}

export function amountsFor(model, progress, mode = 'sequential', order = model.bends.map(b => b.id)) {
  const q = clamp01(progress), amounts = {};
  if (mode === 'simultaneous') for (const bend of model.bends) amounts[bend.id] = q;
  else {
    if (new Set(order).size !== model.bends.length || order.some(id => !model.bends.some(b => b.id === id)))
      throw new Error('La secuencia debe contener una vez cada pliegue.');
    order.forEach((id, index) => { amounts[id] = clamp01(q * order.length - index); });
  }
  return amounts;
}

export function evaluateModel(model, progress, {mode = 'sequential', order, amounts} = {}) {
  const q = amounts ?? amountsFor(model, progress, mode, order);
  const panelMatrices = {[model.rootPanelId]: identity()}, bandPositions = {};
  const waiting = [...model.bends];
  while (waiting.length) {
    const before = waiting.length;
    for (let i = waiting.length - 1; i >= 0; i--) {
      const bend = waiting[i], parent = panelMatrices[bend.parentPanelId];
      if (!parent) continue;
      if (panelMatrices[bend.childPanelId]) throw new Error('El panel tiene más de un padre o el grafo contiene un ciclo.');
      const amount = clamp01(q[bend.strokeLeadId ?? bend.id]);
      panelMatrices[bend.childPanelId] = multiply(parent, jointTransform(bend, amount));
      bandPositions[bend.id] = foldedBandPositions(bend, amount, parent);
      waiting.splice(i, 1);
    }
    if (before === waiting.length) throw new Error('El grafo de paneles está desconectado o contiene un ciclo.');
  }
  for (const panel of model.panels) if (!panelMatrices[panel.id]) throw new Error(`Panel sin transformación: ${panel.id}`);
  return {panelMatrices, bandPositions, amounts: q};
}

export function transformedPanel(panel, matrix) {
  return transformPositions(panel.flatPositions, matrix);
}

/** Transform an entire mesh without allocating an array per vertex. The output may be the input. */
export function transformPositions(positions, matrix, output = new Float64Array(positions.length)) {
  for (let i = 0; i < positions.length; i += 3) {
    const x = positions[i], y = positions[i+1], z = positions[i+2];
    output[i]   = matrix[0]*x + matrix[1]*y + matrix[2]*z + matrix[3];
    output[i+1] = matrix[4]*x + matrix[5]*y + matrix[6]*z + matrix[7];
    output[i+2] = matrix[8]*x + matrix[9]*y + matrix[10]*z + matrix[11];
  }
  return output;
}

export function endpointAudit(model) {
  const final = evaluateModel(model, 1), flat = evaluateModel(model, 0);
  let maxDeviation = 0, count = 0, referenced = 0, minZ = Infinity, maxZ = -Infinity;
  const parts = [];
  function inspect(id, points, reference, flatPoints) {
    const vertexCount = points.length / 3;
    let referenceIssue = reference == null ? 'MISSING_REFERENCE' :
      !(Array.isArray(reference) || (ArrayBuffer.isView(reference) && typeof reference.length === 'number')) ? 'INVALID_REFERENCE_TYPE' :
      reference.length !== points.length || !reference.length || reference.length % 3 ? 'REFERENCE_LENGTH_MISMATCH' : null;
    // Reject a malformed body reference as a whole: truncated data does not
    // establish a reliable correspondence for just its apparent prefix.
    if (!referenceIssue) for (let i = 0; i < reference.length; i++) {
      if (!Number.isFinite(reference[i])) { referenceIssue = 'NONFINITE_REFERENCE'; break; }
    }
    let max = 0;
    for (let i = 0; i < points.length; i += 3) {
      if (!referenceIssue) {
        const deviation = Math.hypot(points[i]-reference[i], points[i+1]-reference[i+1], points[i+2]-reference[i+2]);
        if (!Number.isFinite(deviation)) referenceIssue = 'NONFINITE_DEVIATION';
        else max = Math.max(max, deviation);
      }
      minZ = Math.min(minZ, flatPoints[i+2]); maxZ = Math.max(maxZ, flatPoints[i+2]); count++;
    }
    const valid = referenceIssue === null;
    if (valid) { referenced += vertexCount; maxDeviation = Math.max(maxDeviation, max); }
    parts.push({id, maxDeviationMm: valid ? max : null, vertexCount,
      referencedVertexCount: valid ? vertexCount : 0, referenceStatus: valid ? 'REFERENCED' : 'NO_REFERENCE', referenceIssue});
  }
  for (const p of model.panels) inspect(p.id, transformedPanel(p, final.panelMatrices[p.id]), p.sourcePositions, transformedPanel(p, flat.panelMatrices[p.id]));
  for (const b of model.bends) inspect(b.id, final.bandPositions[b.id], b.band.sourcePositions, flat.bandPositions[b.id]);
  const referenceStatus = referenced === 0 ? 'NO_REFERENCE' : parts.every(p => p.referenceStatus === 'REFERENCED') ? 'REFERENCED' : 'PARTIAL_REFERENCE';
  // A whole-model maximum is meaningful only when every vertex was compared.
  return {maxDeviationMm: referenceStatus === 'REFERENCED' ? maxDeviation : null,
    maxReferencedDeviationMm: referenced ? maxDeviation : null, referenceStatus, referencedVertexCount: referenced,
    invalidReferencePartCount: parts.filter(p => p.referenceIssue && p.referenceIssue !== 'MISSING_REFERENCE').length,
    vertexCount: count, flatThicknessMm: Number.isFinite(minZ) && Number.isFinite(maxZ) ? maxZ-minZ : null, parts};
}
