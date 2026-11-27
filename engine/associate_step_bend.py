"""Associate BCBEND v5 labels with an independently imported STEP model.

No STEP geometry, joint frames, radii, angles, K factors or mesh coordinates are
changed. Matching uses material contours/holes, straight bend locations and a
connected undirected topology under one planar isometry. IDs never score a match.
Legacy direction strings do not establish a sign; explicit signedSweepDegrees
is checked about its exported hinge direction against the authoritative STEP.
"""
from __future__ import annotations

import copy
import math
import time
from collections import defaultdict

import shapely
from shapely.affinity import affine_transform
from shapely.geometry import Polygon, LineString, box, Point
from shapely.ops import unary_union

RESERVED_IDS = {'__proto__', 'constructor', 'prototype', 'toString', 'valueOf', 'hasOwnProperty',
                'isPrototypeOf', 'propertyIsEnumerable', 'toLocaleString', '__defineGetter__',
                '__defineSetter__', '__lookupGetter__', '__lookupSetter__'}


class AssociationInputError(ValueError):
    pass


class _Budget(Exception):
    pass


def require(condition, message):
    if not condition: raise AssociationInputError(message)


def finite(value, name):
    require(isinstance(value, (int, float)) and not isinstance(value, bool), 'Número inválido: '+name)
    try: ok = math.isfinite(value)
    except OverflowError: ok = False
    require(ok, 'Número no finito: '+name)
    return float(value)


def polys(g):
    if g.geom_type == 'Polygon': return [] if g.is_empty else [g]
    return [p for child in getattr(g, 'geoms', []) for p in polys(child)]


def lines(g):
    if g.geom_type == 'LineString': return [] if g.is_empty else [g]
    return [p for child in getattr(g, 'geoms', []) for p in lines(child)]


def holes(g): return sum(len(p.interiors) for p in polys(g))
def dot(a, b): return sum(x*y for x, y in zip(a, b))
def sub(a, b): return [x-y for x, y in zip(a, b)]
def unit(a):
    norm = math.hypot(*a); require(norm > 1e-9, 'Eje de longitud nula.')
    return [x/norm for x in a]
def left(a): return [-a[1], a[0]]
def shape_signature(g): return {'areaMm2': g.area, 'holeCount': holes(g), 'components': len(polys(g))}


def prepare_bend_document(document):
    """Validate the v5 planar contract without guessing any folded sign."""
    require(isinstance(document, dict), 'Se esperaba un documento BEND.')
    g = document.get('sheetMetalGraph')
    require(isinstance(g, dict) and str(g.get('schemaVersion')) in ['5', '5.0'], 'Se requiere sheetMetalGraph v5.')
    require(g.get('units') in ['mm', 'in'], 'Unidades no admitidas.')
    units = g['units']; factor = 25.4 if units == 'in' else 1.

    def length(obj, key, default=None):
        require(isinstance(obj, dict), 'Objeto dimensional inválido.')
        for candidate in [key+'_'+units, key+'Mm' if units == 'mm' else key+'In', key]:
            if candidate in obj: return finite(obj[candidate], candidate)*factor
        require(default is not None, 'Falta dimensión '+key)
        return default

    def point(p): return [length(p, 'x'), length(p, 'y')]
    count = 0

    def polygon(record, use_frame=False):
        nonlocal count
        require(isinstance(record, dict) and isinstance(record.get('outerLoop'), list)
                and isinstance(record.get('holeLoops', []), list), 'Contorno ausente o inválido.')
        origin, u, v = [0., 0.], [1., 0.], [0., 1.]
        if use_frame:
            origin = point(record.get('origin', {'x': 0., 'y': 0.}))
            ur, vr = record.get('u', {'x': 1., 'y': 0.}), record.get('v', {'x': 0., 'y': 1.})
            require(isinstance(ur, dict) and isinstance(vr, dict), 'Marco planar inválido.')
            u = [finite(ur.get(k), 'u') for k in ['x', 'y']]
            v = [finite(vr.get(k), 'v') for k in ['x', 'y']]
            require(abs(dot(u, u)-1) < 1e-6 and abs(dot(v, v)-1) < 1e-6 and abs(dot(u, v)) < 1e-6
                    and u[0]*v[1]-u[1]*v[0] > .999999, 'Marco planar no ortonormal/directo.')
        loops = []
        for ring in [record['outerLoop'], *record.get('holeLoops', [])]:
            require(isinstance(ring, list) and len(ring) >= 3, 'Contorno incompleto.')
            count += len(ring); require(count <= 100000, 'Límite de puntos excedido.')
            transformed = []
            for p in ring:
                x, y = point(p); q = [origin[i]+x*u[i]+y*v[i] for i in range(2)]
                require(max(map(abs, q)) < 1e6, 'Coordenadas fuera de rango.')
                transformed.append(q)
            loops.append(transformed)
        p = Polygon(loops[0], loops[1:])
        require(p.is_valid and p.area > 1e-8, 'Contorno cruzado, vacío o inválido.')
        return shapely.set_precision(p, 1e-7)

    records, edge_records = g.get('panels'), g.get('bendEdges')
    require(isinstance(records, list) and 1 <= len(records) <= 200 and isinstance(edge_records, list)
            and len(edge_records) == len(records)-1, 'Se requiere un árbol de 1 a 200 paneles.')
    material = polygon(g.get('materialBoundary')); panels, names = {}, {}
    for p in records:
        require(isinstance(p, dict), 'Panel inválido.')
        pid = str(p.get('panelId', '')); require(pid and pid not in panels, 'ID de panel ausente o duplicado.')
        require(pid not in RESERVED_IDS, 'Identificador reservado por el consumidor JavaScript.')
        panels[pid] = polygon(p, True); names[pid] = str(p.get('label', p.get('name', pid)))
    root = str(g.get('rootPanelId', '')); require(root in panels, 'Raíz inexistente.')
    union = unary_union(list(panels.values())); area_tol = max(1e-5, material.area*1e-8)
    require(union.symmetric_difference(material).area <= area_tol and
            sum(p.area for p in panels.values())-union.area <= area_tol,
            'Los paneles no particionan el contorno material sin huecos y solapes.')
    mat = g.get('material', {}); require(isinstance(mat, dict), 'Material inválido.')
    thickness = length(mat, 'thickness'); k = finite(mat.get('kFactor'), 'kFactor')
    require(0 < thickness < 1000 and 0 <= k <= 1, 'Espesor o K inválidos.')
    graph = {pid: {} for pid in panels}; incoming, edges, bands = {}, {}, {}
    for e in edge_records:
        require(isinstance(e, dict), 'Relación inválida.')
        bid = str(e.get('bendId', '')); parent, child = str(e.get('parentPanelId')), str(e.get('childPanelId'))
        require(bid not in RESERVED_IDS, 'Identificador reservado por el consumidor JavaScript.')
        require(bid and bid not in edges and parent in panels and child in panels and parent != child
                and child != root and child not in incoming, 'Relaciones duplicadas o inválidas.')
        require(e.get('operationRelationCount', 1) == 1, 'Operación compuesta con varias relaciones no admitida.')
        incoming[child] = parent
        require(abs(length(e, 'thickness', thickness)-thickness) <= 1e-6, 'Espesor no uniforme.')
        radius = length(e, 'insideRadius'); ek = finite(e.get('kFactor', k), 'bend.kFactor')
        explicit = finite(e['signedSweepDegrees'], 'signedSweepDegrees') if 'signedSweepDegrees' in e else None
        sweep = finite(e.get('foldAngleDegrees', abs(explicit) if explicit is not None else None), 'foldAngleDegrees')
        require(0 < sweep < 180-1e-6 and radius >= 0 and 0 <= ek <= 1, 'Barrido, radio o K inválidos.')
        require(explicit is None or abs(abs(explicit)-sweep) < 1e-6, 'Barridos firmado y absoluto contradictorios.')
        start, end = point(e.get('hingeStart')), point(e.get('hingeEnd')); direction = unit(sub(end, start))
        side = finite(e.get('childSideSign'), 'childSideSign'); require(side in [-1, 1], 'Lado del hijo inválido.')
        segments = e.get('hingeSegments', [[e['hingeStart'], e['hingeEnd']]])
        require(isinstance(segments, list) and 1 <= len(segments) <= 1000, 'Segmentos de eje inválidos.')
        segments_xy, intervals, total, weighted = [], [], 0., [0., 0.]
        for s in segments:
            require(isinstance(s, list) and len(s) == 2, 'Segmento de eje inválido.')
            a, b = map(point, s); width = math.dist(a, b)
            require(width > 1e-6 and all(abs(dot(sub(p, start), left(direction))) < 1e-5 for p in [a, b]), 'Segmentos no coaxiales.')
            intervals.append(sorted([dot(sub(p, start), direction) for p in [a, b]])); segments_xy.append(LineString([a, b]))
            total += width
            for i in range(2): weighted[i] += width*(a[i]+b[i])/2
            for fraction in [.25, .5, .75]:
                mid = [a[i]+fraction*(b[i]-a[i]) for i in range(2)]; epsilon = min(.01, thickness*.1)
                transverse = [side*x for x in left(direction)]
                require(panels[parent].buffer(1e-7).covers(Point([mid[i]-epsilon*transverse[i] for i in range(2)])) and
                        panels[child].buffer(1e-7).covers(Point([mid[i]+epsilon*transverse[i] for i in range(2)])),
                        'childSideSign o intervalo material no coincide con los paneles.')
        intervals.sort(); require(all(a[1] <= b[0]+1e-6 for a, b in zip(intervals, intervals[1:])), 'Segmentos superpuestos.')
        axis_shape = unary_union(segments_xy)
        for pid in [parent, child]:
            require(axis_shape.difference(panels[pid].boundary.buffer(1e-5)).length < 1e-4, 'Eje fuera de la frontera compartida.')
        neutral = math.radians(sweep)*(radius+ek*thickness)
        require(neutral > 1e-7, 'Banda neutra de longitud nula.')
        pair = panels[parent].union(panels[child]); projection = []
        for p in polys(pair):
            projection.extend(dot(sub(v[:2], start), direction) for v in p.exterior.coords)
        full_line = LineString([[start[i]+direction[i]*(min(projection)-1) for i in range(2)],
                               [start[i]+direction[i]*(max(projection)+1) for i in range(2)]])
        band = pair.intersection(shapely.set_precision(full_line.buffer(neutral/2, cap_style=2), 1e-7))
        band = unary_union([p for p in polys(band) if p.intersection(axis_shape.buffer(1e-6)).area > 1e-9])
        require(not band.is_empty, 'Banda sin material.')
        axis_actual = band.intersection(full_line)
        require(axis_actual.difference(axis_shape.buffer(1e-5)).length < 1e-4 and
                axis_shape.difference(band.buffer(1e-5)).length < 1e-4, 'Intervalos materiales de eje incompletos.')
        bands[bid] = band
        source_id = str(e.get('sourceBendId', bid)); label = e.get('label', e.get('name'))
        if label is None: label = 'B'+source_id if source_id.isdigit() else source_id
        edges[bid] = {'id': bid, 'parent': parent, 'child': child, 'axis': direction,
                      'anchor': [v/total for v in weighted], 'line': axis_shape, 'width': total,
                      'sweep': sweep, 'radius': radius, 'k': ek, 'neutralLength': neutral,
                      'explicitSignedSweep': explicit, 'sourceBendId': source_id, 'label': str(label),
                      'sequenceHint': e.get('sequenceHint')}
        graph[parent][child] = bid; graph[child][parent] = bid
    seen, queue = {root}, [root]
    for pid in queue:
        for neighbor in graph[pid]:
            if neighbor not in seen: seen.add(neighbor); queue.append(neighbor)
    require(len(seen) == len(panels), 'Grafo desconectado o cíclico.')
    band_list = list(bands.items())
    for i in range(len(band_list)):
        for j in range(i+1, len(band_list)):
            (_, first), (oid, second) = band_list[i], band_list[j]
            overlap = first.intersection(second).area
            if area_tol < overlap <= 1e-3*min(first.area, second.area):
                band_list[j] = (oid, second.difference(first))  # corner sliver, same rule as import_bcbend
    bands.update(band_list)
    for i, (_, first) in enumerate(band_list):
        for _, second in band_list[i+1:]: require(first.intersection(second).area <= area_tol, 'Bandas finitas solapadas.')
    band_union = unary_union(list(bands.values()))
    require(all(p.difference(band_union).area > 1e-8 for p in panels.values()), 'Las bandas consumen un panel completo.')
    return {'panels': panels, 'panelLabels': names, 'edges': edges, 'graph': graph, 'root': root,
            'material': material, 'thickness': thickness, 'k': k}


def _skin(mesh, thickness):
    require(isinstance(mesh, dict), 'Malla CAD inválida.')
    points, triangles = mesh.get('flatPositions'), mesh.get('triangles')
    require(isinstance(points, list) and len(points) % 3 == 0 and isinstance(triangles, list)
            and len(triangles) % 3 == 0 and points and triangles, 'Malla CAD incompleta.')
    require(all(math.isfinite(v) for v in points), 'Malla CAD no finita.')
    triangles_xy = []; tolerance = min(.005, thickness*.01)
    for i in range(0, len(triangles), 3):
        ids = triangles[i:i+3]
        require(all(isinstance(k, int) and 0 <= k < len(points)//3 for k in ids), 'Índice de malla CAD inválido.')
        xyz = [points[3*k:3*k+3] for k in ids]
        if any(abs(p[2]) > tolerance for p in xyz): continue
        tri = Polygon([p[:2] for p in xyz])
        if tri.area > 1e-12: triangles_xy.append(tri)
    require(triangles_xy, 'El CAD no ofrece una piel plana z=0 en su marco desplegado.')
    return unary_union([shapely.set_precision(t, 1e-7) for t in triangles_xy])


def _prepare_cad(model):
    require(isinstance(model, dict) and model.get('units') == 'mm', 'Se requiere modelo CAD preparado en mm.')
    records, joints = model.get('panels'), model.get('bends')
    require(isinstance(records, list) and records and isinstance(joints, list) and len(joints) == len(records)-1, 'Árbol CAD inválido.')
    thickness = finite(model.get('thicknessMm'), 'CAD thickness'); require(thickness > 0, 'Espesor CAD inválido.')
    panels = {str(p['id']): _skin(p, thickness) for p in records}; require(len(panels) == len(records), 'Panel CAD duplicado.')
    graph, edges = {pid: {} for pid in panels}, {}
    for b in joints:
        bid, parent, child = str(b['id']), str(b['parentPanelId']), str(b['childPanelId'])
        require(bid not in edges and parent in panels and child in panels, 'Relación CAD inválida.')
        require(math.dist(b['normal'], [0, 0, 1]) < 1e-6 and abs(b['axis'][2])+abs(b['transverse'][2])+abs(b['origin'][2]) < 1e-5,
                'Marco CAD desplegado no planar XY.')
        a, t, o, length = b['axis'][:2], b['transverse'][:2], b['origin'][:2], b['neutralLength']
        band = _skin(b['band'], thickness)
        local = affine_transform(band, [a[0], a[1], t[0], t[1], -dot(a, o), -dot(t, o)])
        lo, _, hi, _ = local.bounds
        matrix = [a[0], t[0], a[1], t[1], o[0], o[1]]
        lower = affine_transform(local.intersection(box(lo-1, -1, hi+1, length/2)), matrix)
        upper = affine_transform(local.intersection(box(lo-1, length/2, hi+1, length+1)), matrix)
        panels[parent] = panels[parent].union(lower); panels[child] = panels[child].union(upper)
        axis_shape = band.intersection(LineString([[o[i]+a[i]*(lo-1)+t[i]*length/2 for i in range(2)],
                                                   [o[i]+a[i]*(hi+1)+t[i]*length/2 for i in range(2)]]))
        axis_lines = lines(axis_shape); require(axis_lines, 'Banda CAD sin sección material central.')
        width = sum(line.length for line in axis_lines)
        anchor = [sum(line.length*line.centroid.coords[0][i] for line in axis_lines)/width for i in range(2)]
        edges[bid] = {'id': bid, 'parent': parent, 'child': child, 'axis': a, 'anchor': anchor, 'line': axis_shape,
                      'width': width, 'sweep': abs(math.degrees(b['signedAngle'])), 'signedAngle': math.degrees(b['signedAngle']),
                      'radius': b['insideRadius'], 'k': b.get('kFactor', model.get('kFactor')),
                      'neutralLength': length}
        graph[parent][child] = bid; graph[child][parent] = bid
    panels = {pid: shapely.set_precision(g, 1e-7) for pid, g in panels.items()}
    return {'panels': panels, 'edges': edges, 'graph': graph, 'material': unary_union(list(panels.values())), 'thickness': thickness}


def _transform(origin_a, axis_a, origin_b, axis_b, determinant):
    u, v = axis_a, left(axis_a); x, y = axis_b, [determinant*n for n in left(axis_b)]
    r = [x[0]*u[0]+y[0]*v[0], x[0]*u[1]+y[0]*v[1],
         x[1]*u[0]+y[1]*v[0], x[1]*u[1]+y[1]*v[1]]
    return [*r, origin_b[0]-r[0]*origin_a[0]-r[1]*origin_a[1], origin_b[1]-r[2]*origin_a[0]-r[3]*origin_a[1]]


def _edge_compatible(a, b, linear, angle, dimension):
    return (abs(a['sweep']-b['sweep']) <= angle and abs(a['radius']-b['radius']) <= dimension
            and abs(a['neutralLength']-b['neutralLength']) <= linear and abs(a['width']-b['width']) <= 2*linear
            and abs(a['k']-b['k']) <= 1e-6)


def _distance(a, b):
    return float(shapely.hausdorff_distance(a, b, densify=.25))


def associate(document, step_model, *, linear_tolerance_mm=.1, angle_tolerance_deg=.1,
              area_relative_tolerance=.005, dimension_tolerance_mm=.01,
              max_candidates=20000, time_budget_seconds=30):
    """Return {status, model (matched only), report}; never mutate either input.

    A complete generated axis/anchor registration search is required before
    reporting one unique mapping. This is not a proof over the continuous space
    of every noisy planar pose. Budget exhaustion remains inconclusive even if
    one candidate matched.
    """
    started = time.monotonic()
    report = {'method': 'UNDIRECTED_TOPOLOGY_AND_FULL_FLAT_CONTOURS_UNDER_ONE_ISOMETRY',
              'uniquenessScope': 'all-generated-axis-anchor-registrations',
              'mappingCount': 0, 'panelMappingCount': 0, 'candidateCount': 0, 'assignmentNodes': 0, 'survivingMappings': 0,
              'budgetExhausted': False, 'tolerances': {'linearMm': linear_tolerance_mm, 'angleDegrees': angle_tolerance_deg,
                'relativeSymmetricDifferenceArea': area_relative_tolerance, 'thicknessAndRadiusMm': dimension_tolerance_mm},
              'limits': {'maxCandidates': max_candidates, 'timeBudgetSeconds': time_budget_seconds},
              'geometryModified': False, 'legacyDirectionStringsUsed': False}

    def fail(code, message, status='mismatch', **extra):
        report.update(code=code, message=message, elapsedSeconds=time.monotonic()-started, **extra)
        return {'status': status, 'report': report}

    if not all(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) and x > 0 for x in
               [linear_tolerance_mm, angle_tolerance_deg, area_relative_tolerance, dimension_tolerance_mm, time_budget_seconds]) or not isinstance(max_candidates, int) or isinstance(max_candidates, bool) or max_candidates < 1:
        return fail('invalid-options', 'Tolerancias o presupuesto inválidos.')
    try: bend = prepare_bend_document(document)
    except (AssociationInputError, KeyError, TypeError, ValueError, OverflowError, shapely.errors.GEOSException) as exc:
        return fail('invalid-bend-document', str(exc))
    try: cad = _prepare_cad(step_model)
    except (AssociationInputError, KeyError, TypeError, ValueError, OverflowError, shapely.errors.GEOSException) as exc:
        return fail('invalid-step-model', str(exc))
    report['sourceCounts'] = {'bendPanels': len(bend['panels']), 'stepPanels': len(cad['panels']),
                              'bendEdges': len(bend['edges']), 'stepEdges': len(cad['edges'])}
    if len(bend['panels']) != len(cad['panels']) or len(bend['edges']) != len(cad['edges']):
        return fail('topology-size-mismatch', 'El STEP y BEND tienen diferente número de paneles o bandas.')
    if abs(bend['thickness']-cad['thickness']) > dimension_tolerance_mm:
        return fail('thickness-mismatch', 'El espesor del STEP no coincide con BEND.')
    if holes(bend['material']) != holes(cad['material']):
        return fail('material-hole-count-mismatch', 'El número de agujeros materiales no coincide.',
                    bendHoles=holes(bend['material']), stepHoles=holes(cad['material']))
    for be in bend['edges'].values():
        if not any(_edge_compatible(be, ce, linear_tolerance_mm, angle_tolerance_deg, dimension_tolerance_mm) for ce in cad['edges'].values()):
            return fail('bend-dimensions-mismatch', 'Un pliegue no tiene candidato CAD de barrido/radio/longitud/K compatibles.', bendEdgeId=be['id'])
    candidate_matrices, seen_matrices = [], set()
    def guard():
        if time.monotonic()-started > time_budget_seconds: raise _Budget()
    def candidate(matrix):
        key = tuple(round(v, 9) for v in matrix)
        if key not in seen_matrices: seen_matrices.add(key); candidate_matrices.append(matrix)
    try:
        if bend['edges']:
            for be in bend['edges'].values():
                for ce in cad['edges'].values():
                    guard()
                    if not _edge_compatible(be, ce, linear_tolerance_mm, angle_tolerance_deg, dimension_tolerance_mm): continue
                    for sign in [1, -1]:
                        for determinant in [1, -1]:
                            candidate(_transform(be['anchor'], be['axis'], ce['anchor'], [sign*x for x in ce['axis']], determinant))
                            if len(candidate_matrices) > max_candidates: raise _Budget()
        else:
            pa, pc = bend['material'], cad['material']
            aa = list(pa.minimum_rotated_rectangle.exterior.coords); ac = list(pc.minimum_rotated_rectangle.exterior.coords)
            for i in range(4):
                for j in range(4):
                    for determinant in [1, -1]:
                        candidate(_transform(list(pa.centroid.coords)[0], unit(sub(aa[i+1], aa[i])),
                                             list(pc.centroid.coords)[0], unit(sub(ac[j+1], ac[j])), determinant))
    except _Budget:
        return fail('association-budget-exhausted', 'No se pudo demostrar unicidad dentro del presupuesto.', 'ambiguous', budgetExhausted=True)
    surviving, best_rejected = {}, None
    signatures_b = {pid: shape_signature(g) for pid, g in bend['panels'].items()}
    signatures_c = {pid: shape_signature(g) for pid, g in cad['panels'].items()}
    sign_conflicts = 0
    try:
        for matrix in candidate_matrices:
            guard()
            if report['candidateCount'] >= max_candidates: raise _Budget()
            report['candidateCount'] += 1
            moved_material = affine_transform(bend['material'], matrix)
            if any(abs(a-b) > linear_tolerance_mm for a, b in zip(moved_material.bounds, cad['material'].bounds)): continue
            outer_distance = _distance(moved_material.boundary, cad['material'].boundary)
            if best_rejected is None or outer_distance < best_rejected: best_rejected = outer_distance
            if outer_distance > linear_tolerance_mm: continue
            moved = {pid: affine_transform(p, matrix) for pid, p in bend['panels'].items()}
            candidates, residuals = {}, {}
            for pid, geometry in moved.items():
                candidates[pid] = []
                for cid, target in cad['panels'].items():
                    guard()
                    a, b = signatures_b[pid], signatures_c[cid]
                    if len(bend['graph'][pid]) != len(cad['graph'][cid]) or a['holeCount'] != b['holeCount'] or a['components'] != b['components']: continue
                    if abs(a['areaMm2']-b['areaMm2'])/max(a['areaMm2'], b['areaMm2']) > area_relative_tolerance: continue
                    if any(abs(a-b) > linear_tolerance_mm for a, b in zip(geometry.bounds, target.bounds)): continue
                    boundary = _distance(geometry.boundary, target.boundary)
                    if boundary > linear_tolerance_mm: continue
                    symmetric = geometry.symmetric_difference(target).area/max(geometry.area, target.area)
                    if symmetric > area_relative_tolerance: continue
                    candidates[pid].append(cid); residuals[pid, cid] = {'boundaryHausdorffMm': boundary, 'relativeSymmetricDifferenceArea': symmetric}
            if any(not c for c in candidates.values()): continue
            order = sorted(candidates, key=lambda pid: (len(candidates[pid]), -len(bend['graph'][pid])))
            mappings = []
            def recurse(index, mapping, used):
                guard()
                report['assignmentNodes'] += 1
                if report['assignmentNodes'] > max_candidates: raise _Budget()
                if index == len(order): mappings.append(dict(mapping)); return
                pid = order[index]
                for cid in candidates[pid]:
                    if cid in used: continue
                    valid = True
                    for neighbor, other in mapping.items():
                        be = bend['graph'][pid].get(neighbor); ce = cad['graph'][cid].get(other)
                        if bool(be) != bool(ce) or (be and not _edge_compatible(bend['edges'][be], cad['edges'][ce], linear_tolerance_mm, angle_tolerance_deg, dimension_tolerance_mm)):
                            valid = False; break
                    if valid:
                        mapping[pid] = cid; used.add(cid); recurse(index+1, mapping, used); used.remove(cid); mapping.pop(pid)
            recurse(0, {}, set())
            for mapping in mappings:
                edge_map, valid = [], True
                for be in bend['edges'].values():
                    cp, cc = mapping[be['parent']], mapping[be['child']]
                    ce = cad['edges'][cad['graph'][cp][cc]]
                    axis = [matrix[0]*be['axis'][0]+matrix[1]*be['axis'][1], matrix[2]*be['axis'][0]+matrix[3]*be['axis'][1]]
                    alignment = dot(axis, ce['axis'])
                    axis_error = math.degrees(math.acos(min(1., max(-1., abs(alignment)))))
                    line_distance = _distance(affine_transform(be['line'], matrix), ce['line'])
                    if axis_error > angle_tolerance_deg or line_distance > linear_tolerance_mm: valid = False; break
                    reversed_relation = cp != ce['parent']
                    recovered = ce['signedAngle']*(-1 if reversed_relation else 1)*(1 if alignment >= 0 else -1)
                    if be['explicitSignedSweep'] is not None and abs(recovered-be['explicitSignedSweep']) > angle_tolerance_deg:
                        sign_conflicts += 1; valid = False; break
                    edge_map.append({'stepBendId': ce['id'], 'bendEdgeId': be['id'], 'sourceBendId': be['sourceBendId'],
                                     'label': be['label'], 'reversedRelation': reversed_relation,
                                     'axisAngularErrorDegrees': axis_error, 'centerlineHausdorffMm': line_distance,
                                     'sourceSignedSweepDegrees': be['explicitSignedSweep'], 'recoveredSignedSweepDegrees': recovered,
                                     'signSource': 'explicit-confirmed' if be['explicitSignedSweep'] is not None else 'recovered-from-matched-step'})
                if not valid: continue
                key = tuple(sorted(mapping.items()))
                found = {'mapping': mapping, 'matrix': matrix, 'edgeMap': edge_map,
                         'materialBoundaryHausdorffMm': outer_distance,
                         'panelResiduals': [{'bendPanelId': pid, 'stepPanelId': cid, **residuals[pid, cid]} for pid, cid in mapping.items()]}
                if key not in surviving or outer_distance < surviving[key]['materialBoundaryHausdorffMm']: surviving[key] = found
                if len(surviving) > 1:
                    return fail('ambiguous-correspondence', 'Más de una asociación geométrica y topológica cumple las tolerancias.', 'ambiguous',
                                survivingMappings=len(surviving), alternatives=[dict(v['mapping']) for v in surviving.values()])
    except _Budget:
        return fail('association-budget-exhausted', 'No se pudo demostrar unicidad dentro del presupuesto.', 'ambiguous',
                    budgetExhausted=True, survivingMappings=len(surviving))
    report['bestCandidateMaterialBoundaryMm'] = best_rejected
    if not surviving:
        return fail('signed-sweep-conflict' if sign_conflicts else 'geometry-mismatch',
                    'El sentido firmado declarado contradice el STEP.' if sign_conflicts else 'No coincide el registro global de contornos, agujeros, ejes y conexiones.')
    found = next(iter(surviving.values())); mapping = found['mapping']; inverse = {cid: pid for pid, cid in mapping.items()}
    bend_inverse = {entry['stepBendId']: entry['bendEdgeId'] for entry in found['edgeMap']}
    output = copy.deepcopy(step_model)
    original_root = str(output['rootPanelId']); output['rootPanelId'] = inverse[original_root]
    for p in output['panels']:
        original = str(p['id']); p['originalStepPanelId'] = original; p['id'] = inverse[original]; p['label'] = bend['panelLabels'][p['id']]
    for b in output['bends']:
        original = str(b['id']); b['originalStepBendId'] = original; b['id'] = bend_inverse[original]
        b['parentPanelId'], b['childPanelId'] = inverse[str(b['parentPanelId'])], inverse[str(b['childPanelId'])]
        source = bend['edges'][b['id']]; b['label'] = source['label']; b['sourceBendId'] = source['sourceBendId']
        b['sourceSequenceHint'] = source['sequenceHint']
    for seam in output.get('seams', []):
        for ref in seam: ref['id'] = (inverse if ref['kind'] == 'panel' else bend_inverse)[str(ref['id'])]
    report.update(code='unique-association', message='Correspondencia única entre todos los registros de ejes y centros generados, bajo las tolerancias declaradas.',
                  mappingCount=len(bend_inverse), panelMappingCount=len(inverse), survivingMappings=1,
                  panelMapping=[{'stepPanelId': cid, 'bendPanelId': pid} for pid, cid in mapping.items()],
                  bendMapping=found['edgeMap'], panelResiduals=found['panelResiduals'],
                  registration={'bendXYToStepFlatXY': found['matrix'], 'determinant': found['matrix'][0]*found['matrix'][3]-found['matrix'][1]*found['matrix'][2],
                                'materialBoundaryHausdorffMm': found['materialBoundaryHausdorffMm']},
                  preservedStepRoot={'originalStepPanelId': original_root, 'mappedBendPanelId': output['rootPanelId'],
                                     'declaredBendRootPanelId': bend['root'], 'rootWasReoriented': False},
                  originalCadAuditNamespace='Unchanged source metadata/validation/faceOwners retain original STEP importer region IDs, not relabelled BEND IDs.',
                  elapsedSeconds=time.monotonic()-started)
    output.setdefault('metadata', {})['stepAssociation'] = {'status': 'matched', 'report': copy.deepcopy(report)}
    output['metadata']['cadAuditIdNamespace'] = report['originalCadAuditNamespace']
    return {'status': 'matched', 'model': output, 'report': report}


match_bend_to_step = associate
