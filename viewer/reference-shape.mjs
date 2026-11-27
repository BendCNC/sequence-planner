// Final-shape check against a reference mesh of the real folded part (from the CAD,
// never rebuilt from the bend angles). Rigid registration only: no scale, no mirror,
// no per-panel fitting. Distances are measured surface-to-surface in both directions.
import {evaluateModel, transformedPanel} from './solver.mjs';

const sub = (a, b) => [a[0]-b[0], a[1]-b[1], a[2]-b[2]];
const dot = (a, b) => a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
const cross = (a, b) => [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];

function pointTriangleDistance(p, a, b, c) {
  const q = closestOnTriangle(p, a, b, c);
  return Math.hypot(p[0]-q[0], p[1]-q[1], p[2]-q[2]);
}

function closestOnTriangle(p, a, b, c) {
  // Ericson, Real-Time Collision Detection 5.1.5.
  const ab = sub(b, a), ac = sub(c, a), ap = sub(p, a);
  const d1 = dot(ab, ap), d2 = dot(ac, ap);
  const at = q => q;
  if (d1 <= 0 && d2 <= 0) return at(a);
  const bp = sub(p, b), d3 = dot(ab, bp), d4 = dot(ac, bp);
  if (d3 >= 0 && d4 <= d3) return at(b);
  const vc = d1*d4 - d3*d2;
  if (vc <= 0 && d1 >= 0 && d3 <= 0) { const v = d1/(d1-d3); return at([a[0]+v*ab[0], a[1]+v*ab[1], a[2]+v*ab[2]]); }
  const cp = sub(p, c), d5 = dot(ab, cp), d6 = dot(ac, cp);
  if (d6 >= 0 && d5 <= d6) return at(c);
  const vb = d5*d2 - d1*d6;
  if (vb <= 0 && d2 >= 0 && d6 <= 0) { const w = d2/(d2-d6); return at([a[0]+w*ac[0], a[1]+w*ac[1], a[2]+w*ac[2]]); }
  const va = d3*d6 - d5*d4;
  if (va <= 0 && d4-d3 >= 0 && d5-d6 >= 0) {
    const w = (d4-d3)/((d4-d3)+(d5-d6));
    return at([b[0]+w*(c[0]-b[0]), b[1]+w*(c[1]-b[1]), b[2]+w*(c[2]-b[2])]);
  }
  const den = 1/(va+vb+vc), v = vb*den, w = vc*den;
  return at([a[0]+ab[0]*v+ac[0]*w, a[1]+ab[1]*v+ac[1]*w, a[2]+ab[2]*v+ac[2]*w]);
}

// Prefer the split with the lowest surface-area-weighted primitive count.
// Overlapping siblings make nearest-point queries visit both subtrees.
function buildDistanceTree(boxes) {
  const emptyBounds = () => [Infinity, Infinity, Infinity, -Infinity, -Infinity, -Infinity];
  function include(bounds, id) {
    const start = id * 6;
    for (let k = 0; k < 3; k++) {
      bounds[k] = Math.min(bounds[k], boxes[start+k]);
      bounds[k+3] = Math.max(bounds[k+3], boxes[start+k+3]);
    }
  }
  function area(bounds) {
    const x = bounds[3]-bounds[0], y = bounds[4]-bounds[1], z = bounds[5]-bounds[2];
    return x*y + y*z + z*x;
  }
  const center = (id, axis) => boxes[id*6+axis] + boxes[id*6+axis+3];
  function build(ids) {
    const bounds = emptyBounds();
    for (const id of ids) include(bounds, id);
    if (ids.length <= 8) return {bounds, ids};
    let bestCost = Infinity, best = null;
    for (let k = 0; k < 3; k++) {
      const mid = bounds[k]+bounds[k+3], left = [], right = [];
      const a = emptyBounds(), b = emptyBounds();
      for (const id of ids) {
        if (center(id,k) < mid) { left.push(id); include(a,id); }
        else { right.push(id); include(b,id); }
      }
      // Bound the depth even on a cloud with one distant outlier.
      if (Math.min(left.length,right.length)*16 < ids.length) continue;
      const cost = left.length*area(a) + right.length*area(b);
      if (cost < bestCost) { bestCost = cost; best = [left,right]; }
    }
    if (best) return {bounds, left: build(best[0]), right: build(best[1])};
    let axis = 0;
    for (let k = 1; k < 3; k++) {
      if (bounds[k + 3] - bounds[k] > bounds[axis + 3] - bounds[axis]) axis = k;
    }
    ids.sort((a, b) => center(a,axis) - center(b,axis) || a - b);
    const mid = ids.length >>> 1;
    return {bounds, left: build(ids.slice(0, mid)), right: build(ids.slice(mid))};
  }
  return build(Array.from({length: boxes.length / 6}, (_, id) => id));
}

function distanceToBoxSquared(p, bounds, i = 0) {
  const dx = Math.max(bounds[i] - p[0], 0, p[0] - bounds[i+3]);
  const dy = Math.max(bounds[i+1] - p[1], 0, p[1] - bounds[i+4]);
  const dz = Math.max(bounds[i+2] - p[2], 0, p[2] - bounds[i+5]);
  return dx * dx + dy * dy + dz * dz;
}

function visitNearest(tree, p, limitSquared, consider) {
  function visit(node, bound) {
    // Keep ties and roundoff at the boundary for the exact primitive test.
    if (bound > limitSquared() * (1 + 1e-12) + 1e-16) return;
    if (node.ids) {
      for (const id of node.ids) consider(id);
      return;
    }
    const left = distanceToBoxSquared(p, node.left.bounds);
    const right = distanceToBoxSquared(p, node.right.bounds);
    if (left <= right) { visit(node.left, left); visit(node.right, right); }
    else { visit(node.right, right); visit(node.left, left); }
  }
  visit(tree, 0);
}

function indexedSurface(positions, triangles) {
  const vertices = Array.from({length: positions.length / 3}, (_, i) =>
    [positions[3*i], positions[3*i+1], positions[3*i+2]]);
  const boxes = new Float64Array(triangles.length * 2);
  for (let t = 0; t < triangles.length; t += 3) {
    const a = vertices[triangles[t]], b = vertices[triangles[t+1]], c = vertices[triangles[t+2]];
    for (let k = 0; k < 3; k++) {
      boxes[2*t+k] = Math.min(a[k], b[k], c[k]);
      boxes[2*t+3+k] = Math.max(a[k], b[k], c[k]);
    }
  }
  const tree = buildDistanceTree(boxes);
  const pointOn = (p, id) => closestOnTriangle(p, vertices[triangles[3*id]],
    vertices[triangles[3*id+1]], vertices[triangles[3*id+2]]);
  let last = 0;
  function closest(p) {
    if (!triangles.length) return {d: Infinity, q: null};
    const seed = last;
    let hit = seed, q = pointOn(p, seed), d = Math.hypot(p[0]-q[0], p[1]-q[1], p[2]-q[2]);
    visitNearest(tree, p, () => d*d, id => {
      if (id === seed) return;
      // A leaf holds several triangles. Its box may overlap the current
      // distance ball even when this triangle's own box is strictly outside.
      if (distanceToBoxSquared(p, boxes, 6*id) > d*d * (1 + 1e-12) + 1e-16) return;
      const next = pointOn(p, id), dist = Math.hypot(p[0]-next[0], p[1]-next[1], p[2]-next[2]);
      // Preserve seeded input-order traversal on ties, including distinct equidistant faces.
      if (dist < d || (dist === d && hit !== seed && id < hit)) {
        hit = id; q = next; d = dist;
      }
    });
    last = hit;
    return {d, q};
  }
  return {closest, distance: p => closest(p).d};
}

export function surfaceIndex(positions, triangles, cell) {
  if (triangles.length / 3 <= 50000) return indexedSurface(positions, triangles);
  // Keep the existing grid traversal and its tie order for large meshes.
  const lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
  for (let i = 0; i < positions.length; i += 3) for (let k = 0; k < 3; k++) {
    lo[k] = Math.min(lo[k], positions[i+k]); hi[k] = Math.max(hi[k], positions[i+k]);
  }
  const size = cell ?? Math.max(1, Math.hypot(hi[0]-lo[0], hi[1]-lo[1], hi[2]-lo[2]) / 40);
  const cells = new Map(), key = (x, y, z) => `${x},${y},${z}`;
  const ix = v => Math.floor(v / size);
  const P = i => [positions[3*i], positions[3*i+1], positions[3*i+2]];
  for (let t = 0; t < triangles.length; t += 3) {
    const a = P(triangles[t]), b = P(triangles[t+1]), c = P(triangles[t+2]);
    const l = [0, 1, 2].map(k => ix(Math.min(a[k], b[k], c[k]))), h = [0, 1, 2].map(k => ix(Math.max(a[k], b[k], c[k])));
    for (let x = l[0]; x <= h[0]; x++) for (let y = l[1]; y <= h[1]; y++) for (let z = l[2]; z <= h[2]; z++) {
      const k = key(x, y, z); if (!cells.has(k)) cells.set(k, []); cells.get(k).push(t);
    }
  }
  const maxRing = Math.ceil(Math.max(hi[0]-lo[0], hi[1]-lo[1], hi[2]-lo[2]) / size) + 2;
  const closest = p => {
      const c = [ix(p[0]), ix(p[1]), ix(p[2])], seen = new Set();
      let best = Infinity, bq = null;
      for (let r = 0; r <= maxRing; r++) {
        // Anything outside ring r is at least (r - 1) * size away.
        if (best <= (r - 1) * size) break;
        // Visit only the shell of ring r (z steps over the interior).
        for (let x = -r; x <= r; x++) for (let y = -r; y <= r; y++) {
          const edge = Math.abs(x) === r || Math.abs(y) === r;
          for (let z = -r; z <= r; z += edge || r === 0 ? 1 : 2 * r) {
            const list = cells.get(key(c[0]+x, c[1]+y, c[2]+z)); if (!list) continue;
            for (const t of list) {
              if (seen.has(t)) continue; seen.add(t);
              const q = closestOnTriangle(p, P(triangles[t]), P(triangles[t+1]), P(triangles[t+2]));
              const d = Math.hypot(p[0]-q[0], p[1]-q[1], p[2]-q[2]);
              if (d < best) { best = d; bq = q; }
            }
          }
        }
      }
      return {d: best, q: bq};
  };
  return {closest, distance: p => closest(p).d};
}

// Proper rotation (det = +1) best aligning point pairs, Horn's quaternion method.
function bestRotation(src, dst, cs, cd) {
  const S = [[0,0,0],[0,0,0],[0,0,0]];
  for (let i = 0; i < src.length; i++) {
    const a = sub(src[i], cs), b = sub(dst[i], cd);
    for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) S[r][c] += a[r]*b[c];
  }
  const [[xx,xy,xz],[yx,yy,yz],[zx,zy,zz]] = S;
  const N = [[xx+yy+zz, yz-zy, zx-xz, xy-yx], [yz-zy, xx-yy-zz, xy+yx, zx+xz],
             [zx-xz, xy+yx, -xx+yy-zz, yz+zy], [xy-yx, zx+xz, yz+zy, -xx-yy+zz]];
  const {values, vectors} = jacobi(N);
  const k = values.indexOf(Math.max(...values));
  const [w, x, y, z] = vectors.map(row => row[k]);
  return [[w*w+x*x-y*y-z*z, 2*(x*y-w*z), 2*(x*z+w*y)],
          [2*(x*y+w*z), w*w-x*x+y*y-z*z, 2*(y*z-w*x)],
          [2*(x*z-w*y), 2*(y*z+w*x), w*w-x*x-y*y+z*z]];
}

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

const apply = (R, t, p) => [dot(R[0], p)+t[0], dot(R[1], p)+t[1], dot(R[2], p)+t[2]];
const centroid = pts => pts.reduce((s, p) => [s[0]+p[0]/pts.length, s[1]+p[1]/pts.length, s[2]+p[2]/pts.length], [0, 0, 0]);

function principalAxes(pts, c) {
  const C = [[0,0,0],[0,0,0],[0,0,0]];
  for (const p of pts) { const d = sub(p, c); for (let r = 0; r < 3; r++) for (let k = 0; k < 3; k++) C[r][k] += d[r]*d[k]; }
  const {values, vectors} = jacobi(C);
  const order = [0, 1, 2].sort((i, j) => values[j] - values[i]);
  return order.map(k => [vectors[0][k], vectors[1][k], vectors[2][k]]);
}

// Nearest-vertex lookup for ICP. Preserve the grid's deterministic choice between
// different equidistant points, and its bounded domain for very distant queries.
function vertexIndex(points, cell) {
  const boxes = new Float64Array(points.length * 6);
  for (let i = 0; i < points.length; i++) {
    boxes.set(points[i], 6*i);
    boxes.set(points[i], 6*i+3);
  }
  const tree = buildDistanceTree(boxes), bounds = tree.bounds;
  const diagonal = Math.hypot(bounds[3]-bounds[0], bounds[4]-bounds[1], bounds[5]-bounds[2]);
  const coarseCell = Math.max(cell * 8, diagonal / 32);
  const guaranteedRangeSquared = (62 * coarseCell) ** 2;
  let fineGrid, coarseGrid;
  return p => {
    let best = Infinity, hit = -1, tied = false;
    visitNearest(tree, p, () => best, id => {
      const q = points[id];
      const dist = (q[0]-p[0])**2 + (q[1]-p[1])**2 + (q[2]-p[2])**2;
      if (dist < best) { best = dist; hit = id; tied = false; }
      else if (dist === best && hit >= 0 && q.some((v, k) => v !== points[hit][k])) tied = true;
    });
    if (hit < 0 || (!tied && best < guaranteedRangeSquared)) return hit;
    // The original fine grid only accepted a nearest point proven within five cells.
    if (best <= (5 * cell) ** 2) {
      fineGrid ??= gridNearest(points, cell, 6);
      const fine = fineGrid(p);
      if (fine >= 0) return fine;
    }
    coarseGrid ??= gridNearest(points, coarseCell, 64);
    return coarseGrid(p);
  };
}

function gridNearest(points, cell, maxShells) {
  const map = new Map(), ix = v => Math.floor(v / cell), key = (x, y, z) => `${x},${y},${z}`;
  points.forEach((p, i) => { const k = key(ix(p[0]), ix(p[1]), ix(p[2])); if (!map.has(k)) map.set(k, []); map.get(k).push(i); });
  return p => {
    const c = [ix(p[0]), ix(p[1]), ix(p[2])]; let best = -1, bd = Infinity;
    for (let r = 0; r < maxShells; r++) {
      if (best >= 0 && bd <= ((r - 1) * cell) ** 2) break;
      for (let x = -r; x <= r; x++) for (let y = -r; y <= r; y++) {
        const edge = Math.abs(x) === r || Math.abs(y) === r;
        for (let z = -r; z <= r; z += edge || r === 0 ? 1 : 2 * r) {
        for (const i of map.get(key(c[0]+x, c[1]+y, c[2]+z)) ?? []) {
          const q = points[i], d = (q[0]-p[0])**2 + (q[1]-p[1])**2 + (q[2]-p[2])**2;
          if (d < bd) { bd = d; best = i; }
        }
      }
      }
    }
    // Exact only when proven nearest within the searched shells; else the caller falls back.
    return maxShells < 64 && !(bd <= ((maxShells - 1) * cell) ** 2) ? -1 : best;
  };
}

function areaSamples(positions, triangles, spacing) {
  const P = i => [positions[3*i], positions[3*i+1], positions[3*i+2]], out = [];
  for (let t = 0; t < triangles.length; t += 3) {
    const a = P(triangles[t]), b = P(triangles[t+1]), c = P(triangles[t+2]);
    const area = Math.hypot(...cross(sub(b, a), sub(c, a))) / 2;
    const n = Math.max(1, Math.min(12, Math.round(Math.sqrt(area) / spacing)));
    for (let i = 0; i <= n; i++) for (let j = 0; i + j <= n; j++) {
      const u = (i + 1/3) / (n + 1), v = (j + 1/3) / (n + 1), w = 1 - u - v;
      if (w < 0) continue;
      out.push([a[0]*w+b[0]*u+c[0]*v, a[1]*w+b[1]*u+c[1]*v, a[2]*w+b[2]*u+c[2]*v]);
    }
  }
  return out;
}

// Samples of the largest planar region (all coplanar triangles sharing a normal).
function refPlate(positions, triangles, spacing) {
  const P = i => [positions[3*i], positions[3*i+1], positions[3*i+2]], groups = new Map();
  for (let t = 0; t < triangles.length; t += 3) {
    const a = P(triangles[t]), n = cross(sub(P(triangles[t+1]), a), sub(P(triangles[t+2]), a));
    const area = Math.hypot(...n) / 2; if (area < 1e-9) continue;
    const u = n.map(v => v / (2 * area)), d = dot(u, a);
    const key = `${u.map(v => Math.round(v * 200)).join(',')},${Math.round(d / spacing)}`;
    const g = groups.get(key) ?? {area: 0, tris: []}; g.area += area; g.tris.push(t); groups.set(key, g);
  }
  let best = null; for (const g of groups.values()) if (!best || g.area > best.area) best = g;
  if (!best) return [];
  const tris = best.tris.flatMap(t => [triangles[t], triangles[t+1], triangles[t+2]]);
  return areaSamples(positions, tris, spacing);
}

// Planner part at the given amounts: one owner (panel/bend) per triangle.
export function plannerSurface(model, amounts) {
  const state = evaluateModel(model, 1, {amounts});
  const positions = [], triangles = [], owners = [];
  const add = (owner, pos, tris) => {
    const base = positions.length / 3; for (const v of pos) positions.push(v);
    for (let t = 0; t < tris.length; t += 3) { triangles.push(base+tris[t], base+tris[t+1], base+tris[t+2]); owners.push(owner); }
  };
  for (const p of model.panels) add({kind: 'panel', id: p.id, label: p.label}, transformedPanel(p, state.panelMatrices[p.id]), p.triangles);
  for (const b of model.bends) add({kind: 'bend', id: b.id, label: b.label}, state.bandPositions[b.id], b.band.triangles);
  return {positions, triangles, owners, state, internal: internalFaces(positions, triangles, owners)};
}

// Panels and bend bands are separate closed solids, so each panel/band joint has
// two coincident cross-section faces inside the real part. Such a face uses only
// vertices shared with another part; a real face never does (the shared vertices of
// one joint are collinear per sheet side, so any proper triangle on them crosses the
// thickness). Measuring them would report ~t/3 at every joint (centroid in the core).
export function internalFaces(positions, triangles, owners) {
  const key = i => `${Math.round(positions[3*i]*1e3)},${Math.round(positions[3*i+1]*1e3)},${Math.round(positions[3*i+2]*1e3)}`;
  const ownersAt = new Map();
  for (let k = 0; k < owners.length; k++) {
    const o = `${owners[k].kind}:${owners[k].id}`;
    for (let j = 0; j < 3; j++) {
      const kk = key(triangles[3*k+j]); if (!ownersAt.has(kk)) ownersAt.set(kk, new Set()); ownersAt.get(kk).add(o);
    }
  }
  const internal = new Uint8Array(owners.length);
  for (let k = 0; k < owners.length; k++) {
    const o = `${owners[k].kind}:${owners[k].id}`;
    const shared = [0, 1, 2].map(j => ownersAt.get(key(triangles[3*k+j])));
    // Same other part at all three corners.
    internal[k] = [...shared[0]].some(other => other !== o && shared[1].has(other) && shared[2].has(other)) ? 1 : 0;
  }
  return internal;
}

export function compareWithReference(model, amounts, reference, {toleranceMm} = {}) {
  if (!reference?.positions?.length || !reference?.triangles?.length) return {status: 'NO_REFERENCE', pass: null};
  const unit = reference.units === 'cm' ? 10 : reference.units === 'in' ? 25.4 : 1;
  const refPos = unit === 1 ? reference.positions : reference.positions.map(v => v * unit);
  const plan = plannerSurface(model, amounts), state = plan.state;
  const thickness = model.thicknessMm || 1;
  const spacing = Math.max(1, thickness * 2);
  const outer = plan.triangles.filter((_, i) => !plan.internal[Math.floor(i / 3)]);
  const planPts = areaSamples(plan.positions, outer, spacing);
  const refPts = areaSamples(refPos, reference.triangles, spacing);
  const refSurface = surfaceIndex(refPos, reference.triangles);
  // Coarse alignment: principal axes, the 4 proper (det +1) sign choices.
  const cp = centroid(planPts), cr = centroid(refPts);
  const ap = principalAxes(planPts, cp), ar = principalAxes(refPts, cr);
  ap[2] = cross(ap[0], ap[1]); ar[2] = cross(ar[0], ar[1]);  // right-handed frames
  const stride = Math.max(1, Math.floor(planPts.length / 1500));
  const icpSrc = planPts.filter((_, i) => i % stride === 0);
  const nearest = vertexIndex(refPts, spacing * 4);
  const inlierMm = Math.max(1, 2 * thickness);
  // Plain ICP, then trimmed ICP: a wrong flange stays an outlier instead of pulling
  // the whole part. Starts are scored by inliers, so the pose matching most of the
  // part wins and the deviation lands on the panels that really differ.
  const refine = (R, t, cloud = icpSrc) => {
    for (const [iters, keep] of [[20, 1], [30, 0.5]]) {
      for (let it = 0; it < iters; it++) {
        const pairs = cloud.map(p => { const q = apply(R, t, p), r = refPts[nearest(q)];
          return {p, r, d: (q[0]-r[0])**2 + (q[1]-r[1])**2 + (q[2]-r[2])**2}; });
        if (keep < 1) pairs.sort((a, b) => a.d - b.d);
        const kept = pairs.slice(0, Math.max(3, Math.floor(pairs.length * keep)));
        const src = kept.map(k => k.p), dst = kept.map(k => k.r), cs = centroid(src), cd = centroid(dst);
        const R2 = bestRotation(src, dst, cs, cd), t2 = sub(cd, apply(R2, [0, 0, 0], cs));
        const step = Math.hypot(...sub(t2, t)); R = R2; t = t2;
        if (step < 1e-5) break;
      }
    }
    let inliers = 0, sq = 0;
    for (const p of icpSrc) {
      const q = apply(R, t, p), r = refPts[nearest(q)], d = (q[0]-r[0])**2 + (q[1]-r[1])**2 + (q[2]-r[2])**2;
      sq += d; if (d <= inlierMm ** 2) inliers++;
    }
    return {R, t, inliers, rms: Math.sqrt(sq / icpSrc.length)};
  };
  const frameStarts = (a, b, ca, cb) => [[1, 1], [1, -1], [-1, 1], [-1, -1]].map(([s0, s1]) => {
    const sb = [b[0].map(v => v*s0), b[1].map(v => v*s1), b[2].map(v => v*s0*s1)];
    const R = [0, 1, 2].map(r => [0, 1, 2].map(c => sb[0][r]*a[0][c] + sb[1][r]*a[1][c] + sb[2][r]*a[2][c]));
    return {R, t: sub(cb, apply(R, [0, 0, 0], ca))};
  });
  const starts = frameStarts(ap, ar, cp, cr);
  // Root-plate starts: the planner root panel against the largest planar region of
  // the reference (the part may be mostly flanges, which defeats whole-part axes).
  const root = model.panels.find(p => p.id === model.rootPanelId);
  const plate = refPlate(refPos, reference.triangles, spacing);
  const rootPts = root ? areaSamples(Array.from(root.flatPositions), root.triangles, spacing) : [];
  const rootStarts = [];
  if (root && plate.length >= 3) {
    const cRoot = centroid(rootPts), cPlate = centroid(plate);
    const fr = principalAxes(rootPts, cRoot), fp = principalAxes(plate, cPlate);
    fr[2] = cross(fr[0], fr[1]); fp[2] = cross(fp[0], fp[1]);
    rootStarts.push(...frameStarts(fr, fp, cRoot, cPlate));
    starts.push(...rootStarts);
  }
  let best = null;
  for (const s of starts) {
    const r = refine(s.R, s.t);
    if (!best || r.inliers > best.inliers || (r.inliers === best.inliers && r.rms < best.rms)) best = r;
  }
  // Point-to-surface polish: the ICP above pairs with area samples ~2t apart, which
  // leaves a registration error of a few tenths of a mm on large parts. Pair with
  // the closest point on the reference surface instead (inliers only, so a wrong
  // flange stays an outlier).
  {
    let {R, t} = best;
    for (let it = 0; it < 25; it++) {
      const src = [], dst = [];
      for (const p of icpSrc) {
        const c = refSurface.closest(apply(R, t, p));
        if (c.q && c.d <= inlierMm) { src.push(p); dst.push(c.q); }
      }
      if (src.length < 3) break;
      const cs = centroid(src), cd = centroid(dst);
      const R2 = bestRotation(src, dst, cs, cd), t2 = sub(cd, apply(R2, [0, 0, 0], cs));
      const step = Math.hypot(...sub(t2, t)); R = R2; t = t2;
      if (step < 1e-5) break;
    }
    best = {...best, R, t};
  }
  const {R, t} = best;
  // Two-way surface distance.
  const moved = plan.positions.slice();
  for (let i = 0; i < moved.length; i += 3) {
    const q = apply(R, t, [moved[i], moved[i+1], moved[i+2]]); moved[i] = q[0]; moved[i+1] = q[1]; moved[i+2] = q[2];
  }
  const planSurface = surfaceIndex(moved, outer);
  const parts = new Map();
  const P = (arr, i) => [arr[3*i], arr[3*i+1], arr[3*i+2]];
  // Where two bend zones meet (corners, relief tips of a joint stroke) the flat
  // model has no single bend to fold the material with, and the CAD fills it with
  // its own corner geometry. Those few mm are not a fold error: skip them.
  const cornerR = 5 * thickness, bendsAt = new Map();
  const vkey = i => `${Math.round(plan.positions[3*i]*1e3)},${Math.round(plan.positions[3*i+1]*1e3)},${Math.round(plan.positions[3*i+2]*1e3)}`;
  // A corner is a vertex shared by two bend zones, or by a bend zone and a panel it
  // does not connect (a neighbouring flange touching the zone at its end).
  const bendById = new Map(model.bends.map(b => [b.id, b]));
  for (let k = 0; k < plan.owners.length; k++)
    for (let j = 0; j < 3; j++) { const v = plan.triangles[3*k+j], kk = vkey(v);
      if (!bendsAt.has(kk)) bendsAt.set(kk, {v, bends: new Set(), panels: new Set()});
      bendsAt.get(kk)[plan.owners[k].kind === 'bend' ? 'bends' : 'panels'].add(plan.owners[k].id); }
  const isCorner = e => e.bends.size > 1 || [...e.bends].some(id => { const b = bendById.get(id);
    return [...e.panels].some(p => p !== b?.parentPanelId && p !== b?.childPanelId); });
  const junctions = [...bendsAt.values()].filter(isCorner).map(e => P(moved, e.v));
  const nearJunction = p => junctions.some(q => (q[0]-p[0])**2 + (q[1]-p[1])**2 + (q[2]-p[2])**2 <= cornerR ** 2);
  let cornerPointsSkipped = 0;
  for (let k = 0, tri = 0; tri < plan.triangles.length; tri += 3, k++) {
    const owner = plan.owners[k], key = `${owner.kind}:${owner.id}`;
    const entry = parts.get(key) ?? {...owner, maxDeviationMm: 0}; parts.set(key, entry);
    if (plan.internal[k]) continue;
    const a = P(moved, plan.triangles[tri]), b = P(moved, plan.triangles[tri+1]), c = P(moved, plan.triangles[tri+2]);
    for (const [j, p] of [a, b, c, [(a[0]+b[0]+c[0])/3, (a[1]+b[1]+c[1])/3, (a[2]+b[2]+c[2])/3]].entries()) {
      if (junctions.length && nearJunction(p)) { cornerPointsSkipped++; continue; }
      const d = refSurface.distance(p);
      if (d > entry.maxDeviationMm) { entry.maxDeviationMm = d; entry.worstVertex = j < 3 ? plan.triangles[tri + j] : plan.triangles[tri]; }
    }
  }
  let planToRef = 0; for (const e of parts.values()) planToRef = Math.max(planToRef, e.maxDeviationMm);
  let refToPlan = 0; for (const p of refPts) refToPlan = Math.max(refToPlan, planSurface.distance(p));
  for (let i = 0; i < refPos.length; i += 3) refToPlan = Math.max(refToPlan, planSurface.distance(P(refPos, i/3)));
  const tol = toleranceMm ?? Math.max(0.5, (reference.toleranceMm ?? 0.05) * 2 + 0.5 * thickness);
  const list = [...parts.values()].map(e => ({...e, maxDeviationMm: +e.maxDeviationMm.toFixed(4)}));
  const maxDeviationMm = Math.max(planToRef, refToPlan);
  // Localise a mismatch: hold the root panel on the reference and see which panels
  // still land on it. A bend whose parent lands but whose child does not is wrong
  // (side, angle or length); everything beyond it inherits the error.
  let suspectBends = [], alternativeSuspectBends = [];
  if (maxDeviationMm > tol && rootStarts.length) {
    const panelPts = model.panels.map(p => ({id: p.id,
      pts: areaSamples(Array.from(transformedPanel(p, state.panelMatrices[p.id])), p.triangles, spacing)}));
    let anchored = null;
    const rootCloud = rootPts.filter((_, i) => i % Math.max(1, Math.floor(rootPts.length / 1500)) === 0);
    // Poses: the whole-part registration (right when one flange is wrong) and the
    // root-plate fits (right when most flanges are wrong). Most landed area wins.
    const maximumScore = panelPts.reduce((sum, panel) => sum + panel.pts.length, 0);
    for (const [i,start] of [best, ...rootStarts].entries()) {
      const r = i === 0 ? best : refine(start.R, start.t, rootCloud);
      const fit = new Map(panelPts.map(({id, pts}) => {
        const d = pts.map(p => refSurface.distance(apply(r.R, r.t, p))).sort((a, b) => a - b);
        return [id, {inside: d.filter(v => v <= inlierMm).length / Math.max(1, d.length), medianMm: d[d.length >> 1] ?? 0}];
      }));
      const score = panelPts.reduce((s, {id, pts}) => s + (fit.get(id).inside >= 0.9 ? pts.length : 0), 0);
      if (!anchored || score > anchored.score) anchored = {score, fit};
      // A tie retains the earlier registration. Every panel already lands
      // on the CAD, so later fits cannot improve this diagnostic.
      if (score === maximumScore) break;
    }
    const ok = id => anchored.fit.get(id)?.inside >= 0.9;
    suspectBends = model.bends.filter(b => ok(b.parentPanelId) && !ok(b.childPanelId))
      .map(b => ({id: b.id, label: b.label ?? b.id, sourceBendId: b.sourceBendId ?? null,
                  signedAngleDeg: +(b.signedAngle * 180 / Math.PI).toFixed(2),
                  amount: amounts?.[b.id] ?? 1, childMedianDeviationMm: +anchored.fit.get(b.childPanelId).medianMm.toFixed(3)}));
    // Inverting every bend on the root is the same shape seen from the other sheet
    // side, so when only root bends are suspect the complementary set explains it too.
    if (suspectBends.length && suspectBends.every(s => model.bends.find(b => b.id === s.id).parentPanelId === model.rootPanelId)) {
      const named = new Set(suspectBends.map(s => s.id));
      alternativeSuspectBends = model.bends.filter(b => b.parentPanelId === model.rootPanelId && !named.has(b.id))
        .map(b => ({id: b.id, label: b.label ?? b.id, sourceBendId: b.sourceBendId ?? null, signedAngleDeg: +(b.signedAngle * 180 / Math.PI).toFixed(2)}));
    }
  }
  list.sort((a, b) => b.maxDeviationMm - a.maxDeviationMm);
  // Every flange lands but something is off: name where (local feature, not a fold).
  const localDeviations = maxDeviationMm > tol && !suspectBends.length
    ? list.filter(p => p.maxDeviationMm > tol).slice(0, 3).map(p => ({kind: p.kind, id: p.id, label: p.label, maxDeviationMm: p.maxDeviationMm}))
    : [];
  return {status: maxDeviationMm <= tol ? 'MATCH' : 'MISMATCH', pass: maxDeviationMm <= tol,
          toleranceMm: tol, maxDeviationMm: +maxDeviationMm.toFixed(4),
          planToReferenceMm: +planToRef.toFixed(4), referenceToPlanMm: +refToPlan.toFixed(4),
          registrationRmsMm: +best.rms.toFixed(4), cornerPointsSkipped, suspectBends, alternativeSuspectBends, localDeviations,
          parts: list};
}
