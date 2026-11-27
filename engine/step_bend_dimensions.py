"""Measure split cylindrical bends when a complete STEP mesh cannot be associated."""
import math
from collections import defaultdict
from itertools import combinations

from import_step import dependencies, kind, coaxial, plane_partner


def interval_signature(spans):
    """Axial material and gaps at half sweep, independent of axis origin."""
    merged = []
    for lo, hi in sorted(tuple(sorted(s)) for s in spans):
        if merged and lo <= merged[-1][1] + 1e-6:
            merged[-1][1] = max(merged[-1][1], hi)
        else:
            merged.append([lo, hi])
    values = [x for pair in merged for x in pair]
    return [b-a for a,b in zip(values, values[1:])]


def same_spans(a, b):
    return bool(a) and len(a) == len(b) and any(
        all(abs(x-y) <= .03 for x,y in zip(a,order)) for order in (b, b[::-1]))


def skin_bends(shape, root, unfolder, part):
    graph = unfolder.build_graph_of_tangent_faces(shape, root)
    groups = []
    for i in sorted(graph):
        face = shape.Faces[i]
        if kind(face) == 'Plane':
            continue
        peers = set(graph.neighbors(i))
        if kind(face) != 'Cylinder' or len(peers)<2 or any(kind(shape.Faces[n]) != 'Plane' for n in peers):
            return []
        u0,u1,v0,v1 = face.ParameterRange
        sweep = math.degrees(u1-u0)
        if not 0 < sweep < 180:
            return []
        surface = face.Surface
        group = next((g for g in groups if coaxial(g['surface'],surface) and
                      abs(g['radius']-surface.Radius)<1e-5 and abs(g['sweep']-sweep)<1e-5), None)
        if group is None:
            group = dict(surface=surface, radius=surface.Radius, sweep=sweep, planes=set(), faces=[], spans=[])
            groups.append(group)
        # Intersect the trimmed surface, so a slot through a single CAD face still
        # splits the contact interval. Project all sections on the same group axis.
        u = (u0+u1)/2
        line = part.makeLine(face.valueAt(u,v0-1),face.valueAt(u,v1+1))
        sections = line.common(face).Edges
        if not sections or any(len(edge.Vertexes)!=2 for edge in sections):
            return []
        group['spans'].extend([v.Point.dot(group['surface'].Axis) for v in e.Vertexes] for e in sections)
        group['faces'].append(i+1)
        group['planes'].update(peers)
    for g in groups:
        g['signature'] = interval_signature(g['spans'])
    return groups


def measure_split_bends(shape, graph, thickness_mm):
    """Return dimensions only for a unique, two-skin, topology-consistent match."""
    _,part,_,unfolder = dependencies()
    if len(shape.Solids)!=1 or not shape.isValid() or not shape.isClosed():
        return {}
    shape = shape.Solids[0]
    planes = [i for i,f in enumerate(shape.Faces) if kind(f)=='Plane']
    root = min(planes, key=lambda i:(-round(shape.Faces[i].Area,7),i))
    opposite, thickness = plane_partner(shape,root)
    if abs(thickness-thickness_mm)>.01:
        return {}
    primary = skin_bends(shape,root,unfolder,part)
    secondary = skin_bends(shape,opposite,unfolder,part)
    operations = defaultdict(list)
    for e in graph['bendEdges']:
        operations[str(e.get('sourceBendId',e['bendId']))].append(e)
    if not primary or len(primary)!=len(operations) or len(primary)!=len(secondary):
        return {}
    units = graph['units']
    factor = 25.4 if units=='in' else 1.
    def point(p):
        return [float(p[k+'_'+units] if k+'_'+units in p else p[k])*factor for k in ('x','y')]
    matched, source_panels, result = {}, {}, {}
    for bid, edges in operations.items():
        start,end = (point(edges[0][k]) for k in ('hingeStart','hingeEnd'))
        length = math.dist(start,end)
        axis = [(y-x)/length for x,y in zip(start,end)]
        spans = [[sum(x*y for x,y in zip(point(p),axis)) for p in span]
                 for e in edges for span in e.get('hingeSegments',[[e['hingeStart'],e['hingeEnd']]])]
        signature = interval_signature(spans)
        panels = {e[k] for e in edges for k in ('parentPanelId','childPanelId')}
        on_root = graph['rootPanelId'] in panels
        candidates = [g for g in primary if same_spans(signature,g['signature']) and
                      len(g['planes'])==len(panels) and (root in g['planes'])==on_root]
        if len(candidates)!=1:
            return {}
        g = candidates[0]
        partners = [h for h in secondary if coaxial(g['surface'],h['surface']) and
                    abs(abs(g['radius']-h['radius'])-thickness)<1e-4 and abs(g['sweep']-h['sweep'])<1e-5 and
                    same_spans(signature,h['signature']) and len(h['planes'])==len(panels) and
                    (opposite in h['planes'])==on_root]
        if len(partners)!=1 or any(other is g for other in matched.values()):
            return {}
        h = partners[0]
        matched[bid],source_panels[bid] = g,panels
        result[bid] = {'sweepDegrees':g['sweep'], 'insideRadiusMm':min(g['radius'],h['radius']),
                       'stepFaces':sorted(g['faces']+h['faces'])}
    for a,b in combinations(operations,2):
        if len(source_panels[a]&source_panels[b]) != len(matched[a]['planes']&matched[b]['planes']):
            return {}
    return result
