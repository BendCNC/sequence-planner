// Flat-pattern reconstruction from the folded 3D reference body.
// The flat model's panel/bend topology is kept; per bend we recover the signed
// angle (so also the direction) and, when the child panel lands on a planar face
// of the reference, the bend-line position (flange length). Bend allowance is
// recomputed from the model's inside radius / K-factor / thickness. A result is
// only accepted when the existing input validation passes on the corrected model.
import {evaluateModel, jointTransform, transform, bandPoint} from './solver.mjs';
import {surfaceIndex, compareWithReference} from './reference-shape.mjs';
import {validateInputAgainstReference} from './input-validation.mjs';

const DEG = 180 / Math.PI;
const sub = (a, b) => [a[0]-b[0], a[1]-b[1], a[2]-b[2]];
const add = (a, b) => [a[0]+b[0], a[1]+b[1], a[2]+b[2]];
const scale = (a, s) => [a[0]*s, a[1]*s, a[2]*s];
const dot = (a, b) => a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
const cross = (a, b) => [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];
const norm = a => { const l = Math.hypot(...a) || 1; return scale(a, 1 / l); };
const centroid = pts => pts.reduce((s, p) => add(s, scale(p, 1 / pts.length)), [0, 0, 0]);
const applyPose = (pose, p) => add([dot(pose.R[0], p), dot(pose.R[1], p), dot(pose.R[2], p)], pose.t);
const rotVec = (pose, v) => [dot(pose.R[0], v), dot(pose.R[1], v), dot(pose.R[2], v)];
const matVec = (m, v) => [m[0]*v[0]+m[1]*v[1]+m[2]*v[2], m[4]*v[0]+m[5]*v[1]+m[6]*v[2], m[8]*v[0]+m[9]*v[1]+m[10]*v[2]];

function jacobi(A) {
  const n = A.length, a = A.map(r => r.slice()), v = a.map((_, i) => a.map((_, j) => +(i === j)));
  for (let sweep = 0; sweep < 60; sweep++) {
    let off = 0; for (let i = 0; i < n; i++) for (let j = i+1; j < n; j++) off += a[i][j]**2;
    if (off < 1e-20) break;
    for (let p = 0; p < n; p++) for (let q = p+1; q < n; q++) {
      if (Math.abs(a[p][q]) < 1e-30) continue;
      const th = (a[q][q]-a[p][p]) / (2*a[p][q]);
      const t = Math.sign(th || 1) / (Math.abs(th) + Math.sqrt(th*th+1)), c = 1/Math.sqrt(t*t+1), s = t*c;
      for (let k = 0; k < n; k++) { const kp = a[k][p], kq = a[k][q]; a[k][p] = c*kp - s*kq; a[k][q] = s*kp + c*kq; }
      for (let k = 0; k < n; k++) { const pk = a[p][k], qk = a[q][k]; a[p][k] = c*pk - s*qk; a[q][k] = s*pk + c*qk; }
      for (let k = 0; k < n; k++) { const kp = v[k][p], kq = v[k][q]; v[k][p] = c*kp - s*kq; v[k][q] = s*kp + c*kq; }
    }
  }
  return {values: a.map((r, i) => r[i]), vectors: v};
}

function bestRotation(src, dst) {
  const cs = centroid(src), cd = centroid(dst), S = [[0,0,0],[0,0,0],[0,0,0]];
  for (let i = 0; i < src.length; i++) {
    const a = sub(src[i], cs), b = sub(dst[i], cd);
    for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) S[r][c] += a[r]*b[c];
  }
  const [[xx,xy,xz],[yx,yy,yz],[zx,zy,zz]] = S;
  const {values, vectors} = jacobi([[xx+yy+zz, yz-zy, zx-xz, xy-yx], [yz-zy, xx-yy-zz, xy+yx, zx+xz],
    [zx-xz, xy+yx, -xx+yy-zz, yz+zy], [xy-yx, zx+xz, yz+zy, -xx-yy+zz]]);
  const k = values.indexOf(Math.max(...values)), [w, x, y, z] = vectors.map(r => r[k]);
  const R = [[w*w+x*x-y*y-z*z, 2*(x*y-w*z), 2*(x*z+w*y)], [2*(x*y+w*z), w*w-x*x+y*y-z*z, 2*(y*z-w*x)],
    [2*(x*z-w*y), 2*(y*z+w*x), w*w-x*x-y*y+z*z]];
  return {R, t: sub(cd, [dot(R[0], cs), dot(R[1], cs), dot(R[2], cs)])};
}

function principalAxes(pts) {
  const c = centroid(pts), C = [[0,0,0],[0,0,0],[0,0,0]];
  for (const p of pts) { const d = sub(p, c); for (let r = 0; r < 3; r++) for (let k = 0; k < 3; k++) C[r][k] += d[r]*d[k]; }
  const {values, vectors} = jacobi(C), order = [0, 1, 2].sort((i, j) => values[j] - values[i]);
  const ax = order.map(k => [vectors[0][k], vectors[1][k], vectors[2][k]]); ax[2] = cross(ax[0], ax[1]);
  return {c, ax};
}

const pick = (pts, n) => pts.length <= n ? pts : pts.filter((_, i) => i % Math.ceil(pts.length / n) === 0);
const P3 = (arr, i) => [arr[3*i], arr[3*i+1], arr[3*i+2]];

/** Vertices plus triangle centroids of a flat-frame mesh. */
function meshPoints(positions, triangles) {
  const out = [];
  for (let i = 0; i < positions.length; i += 3) out.push([positions[i], positions[i+1], positions[i+2]]);
  for (let t = 0; t < triangles.length; t += 3) {
    const a = P3(positions, triangles[t]), b = P3(positions, triangles[t+1]), c = P3(positions, triangles[t+2]);
    out.push(scale(add(add(a, b), c), 1/3));
  }
  return out;
}

/** Planar regions of a mesh: triangles clustered by normal (within angleDeg) and plane offset. */
export function segmentPlanarRegions(positions, triangles, {angleDeg = 2, offsetMm = 0.5} = {}) {
  const tris = [];
  for (let t = 0; t < triangles.length; t += 3) {
    const a = P3(positions, triangles[t]), b = P3(positions, triangles[t+1]), c = P3(positions, triangles[t+2]);
    const n = cross(sub(b, a), sub(c, a)), area = Math.hypot(...n) / 2;
    if (area > 1e-9) tris.push({t, area, n: scale(n, 1 / (2*area)), c: scale(add(add(a, b), c), 1/3), pts: [a, b, c]});
  }
  tris.sort((x, y) => y.area - x.area);
  const cosTol = Math.cos(angleDeg / DEG), regions = [];
  for (const tr of tris) {
    const r = regions.find(g => dot(g.normal, tr.n) >= cosTol && Math.abs(dot(g.normal, tr.c) - g.d) <= offsetMm);
    if (r) { r.area += tr.area; r.points.push(...tr.pts, tr.c); r.triangles.push(tr.t); }
    else regions.push({normal: tr.n, d: dot(tr.n, tr.c), area: tr.area, points: [...tr.pts, tr.c], triangles: [tr.t]});
  }
  return regions.sort((x, y) => y.area - x.area);
}

function nearestIndex(points, cell) {
  const map = new Map(), ix = v => Math.floor(v / cell), key = (x, y, z) => `${x},${y},${z}`;
  points.forEach((p, i) => { const k = key(ix(p[0]), ix(p[1]), ix(p[2])); if (!map.has(k)) map.set(k, []); map.get(k).push(i); });
  return p => {
    const c = p.map(ix); let best = -1, bd = Infinity;
    for (let r = 0; r < 200; r++) {
      if (best >= 0 && bd <= ((r - 1) * cell) ** 2) break;
      for (let x = -r; x <= r; x++) for (let y = -r; y <= r; y++) for (let z = -r; z <= r; z++) {
        if (Math.max(Math.abs(x), Math.abs(y), Math.abs(z)) !== r) continue;
        for (const i of map.get(key(c[0]+x, c[1]+y, c[2]+z)) ?? []) {
          const q = points[i], d = (q[0]-p[0])**2 + (q[1]-p[1])**2 + (q[2]-p[2])**2;
          if (d < bd) { bd = d; best = i; }
        }
      }
    }
    return points[best];
  };
}

function trimmedCost(dists, keep = 0.8) {
  const d = dists.slice().sort((a, b) => a - b), n = Math.max(1, Math.floor(d.length * keep));
  let s = 0; for (let i = 0; i < n; i++) s += d[i];
  return {cost: s / n, median: d[d.length >> 1] ?? 0};
}

function children(model) {
  const map = new Map(); for (const b of model.bends) { if (!map.has(b.parentPanelId)) map.set(b.parentPanelId, []); map.get(b.parentPanelId).push(b); }
  return map;
}
function bfsBends(model) {
  const kids = children(model), out = [], queue = [model.rootPanelId];
  while (queue.length) for (const b of kids.get(queue.shift()) ?? []) { out.push(b); queue.push(b.childPanelId); }
  return out;
}
function subtree(model, bend) {
  const kids = children(model), panels = new Set(), bends = new Set(), queue = [bend.childPanelId];
  while (queue.length) { const p = queue.shift(); panels.add(p); for (const b of kids.get(p) ?? []) { bends.add(b.id); queue.push(b.childPanelId); } }
  return {panels, bends};
}

/** Neutral length for a new angle: the model's own allowance per radian, else r + k*t. */
function neutralLengthFor(model, bend, angle) {
  const perRad = Math.abs(bend.signedAngle) > 1e-6 ? bend.neutralLength / Math.abs(bend.signedAngle)
    : (bend.insideRadius ?? 0) + (bend.kFactor ?? model.kFactor ?? 0.44) * (model.thicknessMm ?? 0);
  return Math.abs(angle) * perRad;
}

const translateArray = (arr, v) => { const out = Array.from(arr); for (let i = 0; i < out.length; i += 3) { out[i] += v[0]; out[i+1] += v[1]; out[i+2] += v[2]; } return out; };

/** Apply {bendId: {signedAngle, lineOffsetMm}} to a copy of the model. The bend line moves
 * along its transverse direction (parent flange grows), the child subtree follows, and
 * the flat length of the bend zone follows the new angle (bend allowance). */
export function applyFlatCorrections(model, corrections) {
  let m = structuredClone(model);
  for (const ref of bfsBends(m)) {
    const c = corrections[ref.id]; if (!c) continue;
    const bend = m.bends.find(b => b.id === ref.id), T = bend.transverse;
    const angle = c.signedAngle ?? bend.signedAngle, offset = c.lineOffsetMm ?? 0;
    const L = neutralLengthFor(m, bend, angle), shift = scale(T, offset + L - bend.neutralLength);
    const {panels, bends} = subtree(m, bend);
    if (offset) {
      const parent = m.panels.find(p => p.id === bend.parentPanelId), pos = Array.from(parent.flatPositions);
      for (let i = 0; i < pos.length; i += 3) {
        const p = [pos[i], pos[i+1], pos[i+2]], u = dot(sub(p, bend.origin), T);
        if (Math.abs(u) < 0.05) { pos[i] += T[0]*offset; pos[i+1] += T[1]*offset; pos[i+2] += T[2]*offset; }
      }
      parent.flatPositions = pos;
    }
    for (const p of m.panels) if (panels.has(p.id)) p.flatPositions = translateArray(p.flatPositions, shift);
    for (const b of m.bends) if (bends.has(b.id)) { b.origin = add(b.origin, shift); b.band = {...b.band, flatPositions: translateArray(b.band.flatPositions, shift)}; }
    bend.origin = add(bend.origin, scale(T, offset));
    bend.signedAngle = angle; bend.neutralLength = L;
    const par = bend.band.parameters, fp = new Array(par.length);
    for (let j = 0; j < par.length; j += 3) { const q = bandPoint(bend, par[j], par[j+1], par[j+2], 0); fp[j] = q[0]; fp[j+1] = q[1]; fp[j+2] = q[2]; }
    bend.band = {...bend.band, flatPositions: fp};
  }
  return m;
}

function golden(f, lo, hi, iters = 18) {
  const g = (Math.sqrt(5) - 1) / 2; let a = lo, b = hi, c = b - g*(b-a), d = a + g*(b-a), fc = f(c), fd = f(d);
  for (let i = 0; i < iters; i++) {
    if (fc < fd) { b = d; d = c; fd = fc; c = b - g*(b-a); fc = f(c); } else { a = c; c = d; fc = fd; d = a + g*(b-a); fd = f(d); }
  }
  return fc < fd ? {x: c, f: fc} : {x: d, f: fd};
}

/** Fit one bend: child panel samples against the reference surface, parent fixed. */
function fitBend(model, bend, parentMatrix, pose, childPts, surface, regions, {maxOffsetMm, inlierMm}) {
  const T = bend.transverse;
  const place = (angle, offset) => {
    const L = neutralLengthFor(model, bend, angle);
    const b = {...bend, signedAngle: angle, neutralLength: L, origin: add(bend.origin, scale(T, offset))};
    const J = jointTransform(b, 1), shift = scale(T, offset + L - bend.neutralLength);
    return childPts.map(p => applyPose(pose, transform(parentMatrix, transform(J, add(p, shift)))));
  };
  const cost = (angle, offset) => trimmedCost(place(angle, offset).map(p => surface.distance(p))).cost;
  const declared = {angle: bend.signedAngle, offset: 0, f: cost(bend.signedAngle, 0)};
  let best = declared;
  for (let a = -175; a <= 175; a += 5) { const f = cost(a / DEG, 0); if (f < best.f) best = {angle: a / DEG, offset: 0, f}; }
  for (let round = 0; round < 2; round++) {
    const ga = golden(x => cost(x, best.offset), best.angle - 5 / DEG, best.angle + 5 / DEG);
    if (ga.f < best.f) best = {...best, angle: ga.x, f: ga.f};
    if (maxOffsetMm > 0) {
      let grid = best;
      for (let o = -maxOffsetMm; o <= maxOffsetMm; o += 1) { const f = cost(best.angle, o); if (f < grid.f) grid = {...best, offset: o, f}; }
      const go = golden(x => cost(best.angle, x), grid.offset - 1, grid.offset + 1);
      best = go.f < grid.f ? {...grid, offset: go.x, f: go.f} : grid;
    }
  }
  // Planar-face refinement: align the child face normal with the matching reference region.
  let region = null;
  const pts = place(best.angle, best.offset), wAxis = norm(rotVec(pose, matVec(parentMatrix, bend.axis)));
  const nChild = norm(rotVec(pose, matVec(parentMatrix, matVec(jointTransform({...bend, signedAngle: best.angle}, 1), bend.normal))));
  for (const r of regions.slice(0, 64)) {
    if (Math.abs(dot(r.normal, nChild)) < Math.cos(15 / DEG)) continue;
    const near = pts.filter(p => Math.abs(dot(r.normal, p) - r.d) <= 3 * inlierMm).length / pts.length;
    if (near >= 0.3 && (!region || near > region.near || (near === region.near && r.area > region.r.area))) region = {r, near};
  }
  if (region) {
    const target = dot(region.r.normal, nChild) < 0 ? scale(region.r.normal, -1) : region.r.normal;
    const pa = v => sub(v, scale(wAxis, dot(wAxis, v))), u = pa(nChild), v = pa(target);
    const phi = Math.atan2(dot(wAxis, cross(u, v)), dot(u, v));
    for (const cand of [best.angle + phi, best.angle - phi]) {
      let c = {angle: cand, offset: best.offset, f: cost(cand, best.offset)};
      if (maxOffsetMm > 0) { const go = golden(x => cost(cand, x), best.offset - 1, best.offset + 1); if (go.f < c.f) c = {...c, offset: go.x, f: go.f}; }
      if (c.f < best.f) best = c;
    }
  }
  const med = trimmedCost(place(best.angle, best.offset).map(p => surface.distance(p))).median;
  return {...best, declaredCost: declared.f, matched: med <= inlierMm, regionArea: region?.r.area ?? 0};
}

function poseStarts(rootPts, regions, refPts) {
  const nearest = nearestIndex(refPts, 4), starts = [], fr = principalAxes(rootPts);
  for (const reg of regions.slice(0, 4)) {
    const fp = principalAxes(pick(reg.points, 2000));
    for (const [s0, s1] of [[1, 1], [1, -1], [-1, 1], [-1, -1]]) {
      const sb = [scale(fp.ax[0], s0), scale(fp.ax[1], s1), scale(fp.ax[2], s0*s1)];
      const R = [0, 1, 2].map(r => [0, 1, 2].map(c => sb[0][r]*fr.ax[0][c] + sb[1][r]*fr.ax[1][c] + sb[2][r]*fr.ax[2][c]));
      let pose = {R, t: sub(fp.c, [dot(R[0], fr.c), dot(R[1], fr.c), dot(R[2], fr.c)])};
      for (let it = 0; it < 25; it++) pose = bestRotation(rootPts, rootPts.map(p => nearest(applyPose(pose, p))));
      starts.push(pose);
    }
  }
  return starts;
}

/** Recover angles/directions/bend-line positions. Returns {status, model?, changes, ...}. */
export function reconstructFlat(model, reference, {toleranceMm, maxOffsetMm = 25, maxStarts = 6} = {}) {
  const t0 = Date.now();
  if (!reference?.positions?.length) return {status: 'FAILED', reason: 'No 3D reference body was provided.', changes: []};
  const unit = reference.units === 'cm' ? 10 : reference.units === 'in' ? 25.4 : 1;
  const refPos = unit === 1 ? reference.positions : reference.positions.map(v => v * unit);
  const ref = {...reference, units: 'mm', positions: refPos};
  const ones = Object.fromEntries(model.bends.map(b => [b.id, 1]));
  const before = compareWithReference(model, ones, ref, {toleranceMm});
  // Same acceptance rule as input validation (strict tolerance, or within ~1.5 sheet thicknesses).
  const near = Math.max(1.5 * (model.thicknessMm || 1), 1);
  const report = {maxDeviationMmBefore: before.maxDeviationMm};
  if (before.pass || before.maxDeviationMm <= near)
    return {...report, status: 'NOT_NEEDED', reason: 'The flat pattern already matches the 3D model.', changes: [], maxDeviationMmAfter: before.maxDeviationMm};
  const thickness = model.thicknessMm || 1, inlierMm = Math.max(1, 2 * thickness);
  const surface = surfaceIndex(refPos, reference.triangles), regions = segmentPlanarRegions(refPos, reference.triangles);
  const refPts = pick(regions.flatMap(r => r.points), 20000);
  const panelPts = Object.fromEntries(model.panels.map(p => [p.id, pick(meshPoints(p.flatPositions, p.triangles), 120)]));
  const rootPts = panelPts[model.rootPanelId];
  const scored = poseStarts(rootPts, regions, refPts).map(pose => ({pose, ...trimmedCost(rootPts.map(p => surface.distance(applyPose(pose, p))))}))
    .sort((a, b) => a.cost - b.cost);
  const starts = [];
  for (const s of scored) {
    if (s.median > inlierMm) continue;
    if (starts.some(o => Math.hypot(...sub(o.pose.t, s.pose.t)) < 1 && o.pose.R.every((row, i) => dot(row, s.pose.R[i]) > 0.999))) continue;
    starts.push(s); if (starts.length >= maxStarts) break;
  }
  if (!starts.length) return {...report, status: 'FAILED', reason: 'The base panel could not be located on a flat face of the 3D model.', changes: [], maxDeviationMmAfter: before.maxDeviationMm};
  let best = null;
  for (const start of starts) {
    let work = model; const fits = {}; let total = start.cost * rootPts.length, n = rootPts.length;
    for (const ref0 of bfsBends(model)) {
      const bend = work.bends.find(b => b.id === ref0.id), state = evaluateModel(work, 1, {amounts: ones});
      const child = work.panels.find(p => p.id === bend.childPanelId);
      const fit = fitBend(work, bend, state.panelMatrices[bend.parentPanelId], start.pose, pick(meshPoints(child.flatPositions, child.triangles), 120),
        surface, regions, {maxOffsetMm, inlierMm});
      fits[bend.id] = fit; total += fit.f * 120; n += 120;
      work = applyFlatCorrections(work, {[bend.id]: {signedAngle: fit.angle, lineOffsetMm: fit.offset}});
    }
    const score = total / n;
    if (!best || score < best.score) best = {score, fits};
  }
  // Keep only meaningful changes, rounded (0.1 deg, 0.01 mm).
  const corrections = {}, changes = [], unmatched = [];
  for (const b of model.bends) {
    const f = best.fits[b.id], label = b.label ?? b.id; if (!f.matched) unmatched.push(label);
    const c = {};
    const angle = Math.round(f.angle * DEG * 10) / 10 / DEG;
    if (Math.abs(angle - b.signedAngle) * DEG >= 0.25 && f.f < f.declaredCost) {
      c.signedAngle = angle;
      if (Math.abs(Math.abs(angle) - Math.abs(b.signedAngle)) * DEG >= 0.05)
        changes.push({bendId: b.id, label, field: 'angleDeg', from: +(b.signedAngle * DEG).toFixed(1), to: +(angle * DEG).toFixed(1)});
      if (Math.sign(angle) !== Math.sign(b.signedAngle)) changes.push({bendId: b.id, label, field: 'direction', from: Math.sign(b.signedAngle) < 0 ? 'down' : 'up', to: Math.sign(angle) < 0 ? 'down' : 'up'});
    }
    const off = Math.round(f.offset * 100) / 100;
    if (Math.abs(off) >= 0.25) { c.lineOffsetMm = off; changes.push({bendId: b.id, label, field: 'bendLineOffsetMm', from: 0, to: off}); }
    if (Object.keys(c).length) corrections[b.id] = c;
  }
  const corrected = applyFlatCorrections(model, corrections);
  for (const ch of changes.slice()) if (ch.field === 'angleDeg') {
    const b0 = model.bends.find(b => b.id === ch.bendId), b1 = corrected.bends.find(b => b.id === ch.bendId);
    changes.push({bendId: ch.bendId, label: ch.label, field: 'neutralLengthMm', from: +b0.neutralLength.toFixed(3), to: +b1.neutralLength.toFixed(3)});
  }
  const after = compareWithReference(corrected, ones, ref, {toleranceMm});
  const afterValidation = validateInputAgainstReference(corrected, ref, {toleranceMm});
  const out = {...report, changes, unmatchedBends: unmatched, maxDeviationMmAfter: after.maxDeviationMm,
    validationAfter: afterValidation.status, elapsedMs: Date.now() - t0};
  if (afterValidation.status === 'INPUT_CONSISTENT' && changes.length) {
    corrected.metadata = {...(corrected.metadata ?? {}), flatReconstruction: {source: '3D reference body', changes}};
    return {...out, status: 'RECONSTRUCTED', model: corrected};
  }
  const why = unmatched.length ? `no flat face of the 3D model matches the panel after ${unmatched.join(', ')} (the panel layout differs from the 3D body)`
    : `the best fit still deviates ${after.maxDeviationMm} mm from the 3D model`;
  return {...out, status: changes.length && after.maxDeviationMm < before.maxDeviationMm ? 'PARTIAL' : 'FAILED',
    reason: `The flat pattern could not be rebuilt from the 3D model: ${why}.`};
}

export function describeChanges(changes) {
  const parts = [];
  for (const c of changes) {
    if (c.field === 'angleDeg') parts.push(`${c.label} angle ${Math.abs(c.from)} -> ${Math.abs(c.to)} deg`);
    else if (c.field === 'direction') parts.push(`${c.label} direction ${c.from} -> ${c.to}`);
    else if (c.field === 'bendLineOffsetMm') parts.push(`${c.label} bend line moved ${c.to > 0 ? '+' : ''}${c.to} mm`);
  }
  return parts.join(', ');
}
