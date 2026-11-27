"""Reconstruct nominal developable sheet geometry from a BCBEND v5 planar graph.

Requires Python 3 and Shapely >=2.1. No FreeCAD, STEP or CAD kernel is imported.
The exported hinge is interpreted as the centre of the developed neutral strip.
Explicit signedSweepDegrees is a right-handed rotation about hingeStart->hingeEnd
in the common flat XY frame. Legacy signs require an explicit caller declaration.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import json
import math
import re
from pathlib import Path
import sys

import numpy as np
import shapely
from shapely.geometry import LineString, Point, Polygon, box
from shapely.ops import unary_union
from shapely.errors import GEOSException
from bcbend_embedded_step import read_bcbend_document, BCBendJsonError

RESERVED_IDS = {'__proto__', 'constructor', 'prototype', 'toString', 'valueOf', 'hasOwnProperty',
                'isPrototypeOf', 'propertyIsEnumerable', 'toLocaleString', '__defineGetter__',
                '__defineSetter__', '__lookupGetter__', '__lookupSetter__'}


class ImportBCBendError(ValueError):
    def __init__(self, code, message, **details):
        super().__init__(message)
        self.code, self.details = code, details

    def as_dict(self):
        return {'status': 'error', 'error': str(self), 'message': str(self),
                'code': self.code, 'details': self.details}


def require(condition, code, message, **details):
    if not condition:
        raise ImportBCBendError(code, message, **details)


def number(value, label):
    require(isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(value), 'invalid-number', 'Número ausente o no finito: '+label)
    return float(value)


def polygons(geometry):
    if geometry.geom_type == 'Polygon':
        return [geometry] if not geometry.is_empty else []
    return [p for g in getattr(geometry, 'geoms', []) for p in polygons(g)]


def coordinates(geometry):
    for p in polygons(geometry):
        yield from p.exterior.coords
        for ring in p.interiors:
            yield from ring.coords


def add(a, b): return [x+y for x, y in zip(a, b)]
def sub(a, b): return [x-y for x, y in zip(a, b)]
def scale(a, s): return [x*s for x in a]
def dot(a, b): return sum(x*y for x, y in zip(a, b))
def identity(): return [1., 0, 0, 0, 0, 1., 0, 0, 0, 0, 1., 0, 0, 0, 0, 1.]
def transform(m, p):
    x, y, z = p
    return [sum((m[0]*x, m[1]*y, m[2]*z))+m[3],
            sum((m[4]*x, m[5]*y, m[6]*z))+m[7],
            sum((m[8]*x, m[9]*y, m[10]*z))+m[11]]

def multiply(a, b): return [sum(a[4*i+k]*b[4*k+j] for k in range(4)) for i in range(4) for j in range(4)]


def band_point(bend, axial, fraction, z, amount):
    angle = bend['signedAngle']*amount*fraction
    sn = math.sin(angle)/angle if abs(angle) > 1e-5 else 1-angle*angle/6+angle**4/120
    cs = (1-math.cos(angle))/angle if abs(angle) > 1e-5 else angle/2-angle**3/24+angle**5/720
    arc = bend['neutralLength']*fraction
    transverse = arc*sn-(z-bend['neutralZ'])*math.sin(angle)
    normal = bend['neutralZ']+arc*cs+(z-bend['neutralZ'])*math.cos(angle)
    return add(add(add(bend['origin'], scale(bend['axis'], axial)),
                   scale(bend['transverse'], transverse)), scale(bend['normal'], normal))


def bounded_pose_inputs(model, meshes):
    """Bound every coordinate entering band_point and the affine vertex transform."""
    def bounded(values):
        return all(abs(x) <= 1e12 for x in values)

    for bend in model['bends']:
        if not bounded([bend['signedAngle'], bend['neutralZ'], bend['neutralLength'],
                        *bend['axis'], *bend['origin'], *bend['normal'], *bend['transverse']]):
            return False
    return all(bounded(mesh['flatPositions'] if kind == 'panel' else mesh['parameters'])
               for (kind, _), mesh in meshes.items())


def evaluate_frames(model, amounts):
    frames = {model['rootPanelId']: identity()}
    for b in model['bends']:
        x, y, z = b['axis']; theta = b['signedAngle']*amounts[b['id']]
        c, s, v = math.cos(theta), math.sin(theta), 1-math.cos(theta)
        r = [x*x*v+c, x*y*v-z*s, x*z*v+y*s, 0,
             y*x*v+z*s, y*y*v+c, y*z*v-x*s, 0,
             z*x*v-y*s, z*y*v+x*s, z*z*v+c, 0, 0, 0, 0, 1]
        anchor = add(add(b['origin'], scale(b['transverse'], b['neutralLength'])), scale(b['normal'], b['neutralZ']))
        offset = sub(band_point(b, 0, 1, b['neutralZ'], amounts[b['id']]), transform(r, anchor))
        r[3], r[7], r[11] = offset
        frames[b['childPanelId']] = multiply(frames[b['parentPanelId']], r)
    return frames


def _triangle_rows(collection):
    """Iterate CDT faces in GEOS order, bounding temporary geometry and coordinate arrays."""
    count = len(collection.geoms)
    batch_size = 64

    def scalar(triangles):
        for triangle in triangles:
            area = triangle.area
            if not area < 1e-14:
                yield area, list(triangle.exterior.coords)[:3]

    if count < 4 or shapely.get_coordinate_dimension(collection) != 2:
        yield from scalar(collection.geoms)
        return
    for start in range(0, count, batch_size):
        stop = min(start + batch_size, count)
        try:
            triangles = (shapely.get_parts(collection) if count <= batch_size else
                         shapely.get_geometry(collection, range(start, stop)))
            areas = shapely.area(triangles)
            normal = ((shapely.get_type_id(triangles) == shapely.GeometryType.POLYGON).all() and
                      (shapely.get_coordinate_dimension(triangles) == 2).all() and
                      (shapely.get_num_coordinates(triangles) == 4).all())
            if normal:
                coords = shapely.get_coordinates(triangles).reshape(-1, 4, 2)
                normal = np.isfinite(coords).all() and (coords[:, 0] == coords[:, 3]).all()
        except (GEOSException, ValueError, TypeError):
            normal = False
        if not normal:
            yield from scalar(collection.geoms[i] for i in range(start, stop))
            continue
        for area, ring in zip(areas.tolist(), coords.tolist()):
            if not area < 1e-14:
                yield area, ring[:3]


def _cell_faces(cells):
    # Preserve GEOS face order across slabs; cap temporary coordinate arrays.
    for start in range(0, len(cells), 16):
        batch = cells[start:start+16]
        faces = shapely.constrained_delaunay_triangles(shapely.set_precision(batch, 1e-8))
        if len(faces) > 1 and sum(shapely.get_num_geometries(faces)) <= 256:
            yield from _triangle_rows(shapely.GeometryCollection(shapely.get_parts(faces)))
        else:
            for collection in faces:
                yield from _triangle_rows(collection)


def closed_mesh(region, thickness, bend=None):
    """Constrained skins plus every exterior/hole wall; retain all disconnected pieces.

    Band slabs enforce angular tessellation. Walls are derived from the triangulated
    skin boundary so each resulting solid has exactly two incident triangles/edge.
    """
    if bend:
        # A rounded hinge end a few nm off the strip edge leaves a micro
        # zig-zag (two vertices ~3e-6 mm apart) that slices into overlapping
        # edges; drop such sub-micron detours before tessellation.
        region = shapely.set_precision(region.simplify(1e-5, preserve_topology=True), 1e-8)
    cells = polygons(region)
    if bend:
        # Slabs are created in local (axial, transverse) coordinates, then mapped
        # back. This keeps interrupted intervals and holes rather than filling them.
        from shapely.affinity import affine_transform
        a, t, o = bend['axis'], bend['transverse'], bend['origin']
        local = affine_transform(region, [a[0], a[1], t[0], t[1], -dot(a, o), -dot(t, o)])
        lo, _, hi, _ = local.bounds
        divisions = max(2, math.ceil(abs(bend['signedAngle'])/math.radians(3)))
        edges = np.array([bend['neutralLength']*i/divisions for i in range(divisions+1)])
        slabs = shapely.intersection(local, shapely.box(lo-1, edges[:-1], hi+1, edges[1:]))
        def to_flat(coords):
            x, y = coords.T
            return np.column_stack((a[0]*x+t[0]*y+o[0], a[1]*x+t[1]*y+o[1]))
        cells = [p for slab in shapely.transform(slabs, to_flat) for p in polygons(slab)]
        # A boundary step lying on a slice line (0.2 um on Part 001 B7) leaves a
        # corner of one slab mid-edge on its neighbour: add it there (T-junction).
        corners = shapely.multipoints(shapely.get_coordinates(cells))
        cells = [polygons(c)[0] for c in shapely.snap(cells, corners, 1e-7)]
    points, lookup, skin = [], {}, []

    def index(p):
        # Vertices of neighbouring slabs that differ by float noise (< 1 um,
        # e.g. merged spans on a large-radius band) are one vertex; otherwise a
        # shared edge is meshed twice with near-duplicate ends (non-manifold).
        x, y = float(p[0]), float(p[1])
        key = (round(x, 8), round(y, 8))
        if key not in lookup:
            coarse = (round(x, 6), round(y, 6))
            if coarse in lookup:
                lookup[key] = lookup[coarse]
            else:
                lookup[key] = lookup[coarse] = len(points); points.append(key)
        return lookup[key]

    triangle_area = 0.
    for area, xyz in _cell_faces(cells):
        ids = [index(p) for p in xyz]
        if len(set(ids)) < 3 and 2*area/max(math.dist(xyz[i], xyz[i-1]) for i in range(3)) < 1e-6:
            continue  # sub-micron sliver between merged near-duplicate vertices
        require(len(set(ids)) == 3, 'mesh-resolution-limit', 'Un detalle es menor que la resolución de mallado de 1e-8 mm.')
        a, b, c = [points[i] for i in ids]
        cross = (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
        if cross < 0: ids.reverse()
        require(abs(cross) > 1e-14, 'degenerate-mesh', 'Triángulo degenerado en el contorno.')
        skin.append(ids); triangle_area += abs(cross)/2
    require(skin, 'empty-region', 'Una región no contiene material triangulable.')
    require(abs(triangle_area-region.area) <= max(1e-5, region.area*1e-8),
            'triangulation-area-mismatch', 'La triangulación no conserva el área material.',
            originalAreaMm2=region.area, triangleAreaMm2=triangle_area)
    edges = defaultdict(list)
    for tri in skin:
        for i in range(3):
            a, b = tri[i], tri[(i+1) % 3]; edges[tuple(sorted((a, b)))].append((a, b))
    require(all(len(v) in [1, 2] for v in edges.values()), 'non-manifold-skin', 'La piel triangulada no es una variedad.')
    n = len(points)
    positions = [v for z in [0., -thickness] for p in points for v in [*p, z]]
    triangles = [i for tri in skin for i in tri]
    triangles += [i+n for tri in skin for i in reversed(tri)]
    for occurrences in edges.values():
        if len(occurrences) == 1:
            a, b = occurrences[0]
            triangles += [a, a+n, b+n, a, b+n, b]
    edge_count, directed = Counter(), Counter()
    for i in range(0, len(triangles), 3):
        tri = triangles[i:i+3]
        for j in range(3):
            a, b = tri[j], tri[(j+1) % 3]
            edge_count[tuple(sorted((a, b)))] += 1; directed[a, b] += 1
    require(all(count == 2 for count in edge_count.values()) and
            all(directed[b, a] == count for (a, b), count in directed.items()),
            'mesh-not-closed', 'La malla no es cerrada y orientada.', bendId=(bend or {}).get('id'), bad=[[points[i % n] for i in e] for e, c in edge_count.items() if c != 2][:6])
    mesh = {'flatPositions': positions, 'triangles': triangles}
    if bend:
        params = []
        for i in range(0, len(positions), 3):
            p = sub(positions[i:i+3], bend['origin'])
            f = dot(p, bend['transverse'])/bend['neutralLength']
            require(-1e-7 <= f <= 1+1e-7, 'band-domain-error', 'Material fuera de la banda finita.')
            params += [dot(p, bend['axis']), min(1., max(0., f)), positions[i+2]]
        mesh['parameters'] = params
    audit = {'closed': True, 'oriented': True, 'vertices': len(positions)//3,
             'triangles': len(triangles)//3, 'areaMm2': region.area,
             'triangulatedAreaMm2': triangle_area, 'volumeMm3': triangle_area*thickness,
             'components': len(polygons(region)), 'holeCount': sum(len(p.interiors) for p in polygons(region))}
    return mesh, audit


CORNER_CLEARANCE_MM = .5


def coupled_span_clip(intervals, start, directed, transverse, neutral_length, band, material, precision=0.):
    """Retain both shoulders of a concave free end of a single coupled span."""
    if len(intervals) != 1:
        return intervals
    bounds = list(intervals[0])
    half_width = neutral_length / 2
    for index, sign in ((0, -1), (1, 1)):
        tip = add(start, scale(directed, bounds[index]))
        if material.exterior.distance(Point(tip)) > 1e-5:
            continue
        outer = add(tip, scale(directed, sign * half_width))
        cross_section = LineString([add(outer, scale(transverse, width))
                                    for width in (-half_width, half_width)])
        # A cut through continued material is still a square cap. Only admit
        # the measured outline when the centre ray exits the sheet and the
        # entire local strip ends before the outer cross section.
        if material.intersection(LineString([tip, outer])).length >= 1e-5 or band.distance(cross_section) <= 1e-5:
            continue
        # On both sides of the centreline the edge curves back into material.
        # A one-sided tab or a sub-precision sliver does not establish that
        # both rigid panels were cut through the same re-entrant relief.
        reach = scale(directed, sign * max(precision, 1e-5))
        shoulders = [add(tip, scale(transverse, width)) for width in (-half_width / 2, half_width / 2)]
        if all(band.covers(LineString([point, add(point, reach)])) for point in shoulders):
            bounds[index] += sign * half_width
    return [bounds]


def trim_band_corner(band, other, bend):
    """Remove other's overlap as a box aligned to band's own axes so the
    angular slicing in closed_mesh sees exactly axial notch edges."""
    from shapely.affinity import affine_transform
    a, t, o = bend['axis'], bend['transverse'], bend['origin']
    lo_a, lo_t, hi_a, hi_t = affine_transform(other.buffer(CORNER_CLEARANCE_MM, join_style=2), [a[0], a[1], t[0], t[1], -dot(a, o), -dot(t, o)]).bounds
    local = affine_transform(band, [a[0], a[1], t[0], t[1], -dot(a, o), -dot(t, o)]).difference(box(lo_a, lo_t, hi_a, hi_t))
    return affine_transform(local, [a[0], t[0], a[1], t[1], o[0], o[1]])


def split_relation_parameters(edge, sense, length, thickness, default_k, convention, direction_key):
    """Express the bend and child side relative to one directed hinge."""
    signed = edge.get('signedSweepDegrees')
    if signed is not None:
        sweep = number(signed, 'signedSweepDegrees')
        angle = number(edge.get('foldAngleDegrees', abs(sweep)), 'foldAngleDegrees')
        require(abs(abs(sweep)-angle)<1e-6, 'conflicting-sweep', 'El ángulo y el barrido firmado no coinciden.')
        rotation = ('signed', math.copysign(1, sweep)*sense)
    else:
        angle = number(edge.get('foldAngleDegrees'), 'foldAngleDegrees')
        rotation = ('axis', sense) if convention == 'directed-positive' else ('child', edge.get(direction_key, edge.get('direction')))
    side = number(edge.get('childSideSign'), 'childSideSign')*sense
    require(abs(length(edge,'thickness',thickness)-thickness)<1e-6, 'nonuniform-thickness', 'El espesor cambia en la operación.')
    return rotation, side, angle, length(edge,'insideRadius'), number(edge.get('kFactor',default_k),'kFactor')


def on_axis(p, start, end, snap_radius):
    """True when p lies on the line start-end within the snapping error.

    start, end and p may each be displaced by up to snap_radius from the exact
    CAD line; the line through two displaced points tilts, so the admissible
    offset grows with p's parameter t along start-end.
    """
    d = sub(end, start)
    n = math.hypot(*d)
    t = dot(sub(p, start), d)/(n*n)
    offset = abs(d[0]*(p[1]-start[1]) - d[1]*(p[0]-start[0]))/n
    return offset < 1e-5 + snap_radius*(1 + abs(t) + abs(1-t))


def couple_joint_stroke(source, members, resolved_panel, incoming, depth, point, length, thickness,
                        default_k, convention, direction_key, panel_geometry, axis_tol=0.):
    """One continuous CAD bend line whose spans hang from different panels.

    In the flat part those panels are coplanar, so the whole line is formed in
    one stroke, but only while every bend between them is still flat. Spans stay
    separate tree edges; followers copy the lead's amount and the lead records
    the panels whose own bends must remain flat when the stroke is made.
    """
    # A parent may contribute several spans (e.g. two tabs per side flange);
    # each stays its own edge and follows the lead like any other span.
    lead = min(members, key=lambda m: (depth(str(m['parentPanelId'])), m['operationRelationIndex']))
    lead_parent = resolved_panel(str(lead['parentPanelId']))
    start, end = point(lead['hingeStart']), point(lead['hingeEnd'])
    direction = sub(end, start)
    require(math.hypot(*direction) > 1e-6, 'zero-length-hinge', 'Eje de pliegue de longitud nula.')
    direction = scale(direction, 1/math.hypot(*direction))
    transverse = [-direction[1], direction[0]]
    reference = split_relation_parameters(lead, 1, length, thickness, default_k, convention, direction_key)
    flat_panels = []
    for m in members:
        a, b = point(m['hingeStart']), point(m['hingeEnd'])
        require(math.dist(a, b) > 1e-6 and all(on_axis(p, start, end, axis_tol) for p in (a, b)),
                'noncollinear-hinge-segments', 'Las relaciones de la operación deben compartir la misma recta.', sourceBendId=source)
        sense = 1 if dot(sub(b, a), direction) > 0 else -1
        params = split_relation_parameters(m, sense, length, thickness, default_k, convention, direction_key)
        require(params[:2] == reference[:2] and all(abs(v-r) < tol for v, r, tol in
                zip(params[2:], reference[2:], (1e-6, 1e-6, 1e-8))),
                'split-hinge-mismatch', 'Los tramos deben compartir el giro físico, radio, espesor y factor K.', sourceBendId=source)
        m.update({'operationRelationCount': 1, 'operationRelationIndex': 0})
        if m is lead:
            continue
        # Walk up to the nearest common ancestor with the lead's parent (the
        # spans may hang from sibling flanges): every panel below it on either
        # side must stay flat so the spans are collinear at the stroke.
        lead_chain, panel = [], str(lead['parentPanelId'])
        while True:
            lead_chain.append(resolved_panel(panel))
            if panel not in incoming: break
            panel = incoming[panel]
        panel, seen = str(m['parentPanelId']), set()
        while resolved_panel(panel) not in lead_chain:
            require(panel in incoming and panel not in seen, 'split-hinge-mismatch',
                    'El padre del tramo debe colgar del padre del tramo principal.', sourceBendId=source)
            seen.add(panel)
            flat_panels.append(resolved_panel(panel))
            panel = incoming[panel]
        flat_panels.extend(lead_chain[:lead_chain.index(resolved_panel(panel))])
        m['strokeLeadBendId'] = str(lead['bendId'])
    lead['strokeFollowerBendIds'] = [str(m['bendId']) for m in members if m is not lead]
    lead['strokeRequiresFlatPanels'] = sorted(set(flat_panels))


def merge_split_hinge_edges(edges, panel_geometry, point, length, thickness, default_k, root, convention, direction_key, axis_tol=0.,
                            counter_tabs='folded'):
    """Fold a split hinge (one CAD bend, several coaxial spans) as one stroke.

    Spans sharing sourceBendId and parent become one bend with every span as
    a hinge segment; their child panels fold together, so they are merged
    into one rigid (multi-polygon) panel. Mutates panel_geometry.
    """
    groups = defaultdict(list)
    incoming, ids = {}, set()
    for e in edges:
        require(isinstance(e, dict), 'invalid-bend-edge', 'Cada relación debe ser un objeto.')
        bid, parent, child = (str(e.get(k, '')) for k in ('bendId', 'parentPanelId', 'childPanelId'))
        require(bid and bid not in ids and bid not in RESERVED_IDS and parent in panel_geometry and
                child in panel_geometry and child != parent and child != root and child not in incoming,
                'invalid-tree-edge', 'La relación original no pertenece a un árbol de paneles válido.', bendId=bid)
        incoming[child] = parent
        ids.add(bid)
        groups[str(e.get('sourceBendId', bid))].append(e)
    for child in incoming:
        seen = set()
        while child != root:
            require(child in incoming and child not in seen, 'cyclic-or-disconnected-graph', 'El grafo original no llega a la raíz.')
            seen.add(child)
            child = incoming[child]
    rename, merged_ids, result = {}, set(), []
    def resolved_panel(panel):
        # A counter tab can join a span that later joins the lead span itself.
        seen = set()
        while panel in rename:
            require(panel not in seen, 'cyclic-or-disconnected-graph', 'La fusión de paneles contiene un ciclo.')
            seen.add(panel)
            panel = rename[panel]
        return panel

    def depth(panel):
        n = 0
        while panel != root and panel in incoming:
            panel, n = incoming[panel], n + 1
        return n
    # Root-first, so a split hinge whose spans hang from the panels of another
    # split hinge sees those panels already merged into one rigid reference.
    ordered = sorted(groups.items(), key=lambda g: min(depth(str(m.get('parentPanelId'))) for m in g[1]))

    def ancestors(panel):
        out = set()
        while panel in incoming:
            panel = incoming[panel]
            out.add(resolved_panel(panel))
        return out
    queue = list(ordered)
    while queue:
        source, members = queue.pop(0)
        if len(members) == 1 and members[0].get('operationRelationCount', 1) == 1: continue
        require(len(members) > 1 and all(type(m.get('operationRelationCount')) is int and
                m['operationRelationCount'] == len(members) and type(m.get('operationRelationIndex')) is int
                for m in members) and sorted(m['operationRelationIndex'] for m in members) == list(range(len(members))),
                'compound-operation-unsupported', 'Faltan relaciones o índices únicos de la operación compuesta.', sourceBendId=source)
        members.sort(key=lambda m: m['operationRelationIndex'])
        # Counter span: a tab in a slot of the parent, hanging from a folded
        # span's child on the same hinge line with the opposite sweep. The flat
        # pattern cannot tell how it folds; the folded CAD body decides (the
        # add-in compares both). 'folded': the tab rides rigidly with the folded
        # span and its counter span is a seam. 'parent': the tab stays with the
        # stroke's parent and its counter span is one more hinge of the stroke.
        children = {resolved_panel(str(m['childPanelId'])) for m in members}
        counter = [m for m in members if resolved_panel(str(m['parentPanelId'])) in children]
        members = [m for m in members if m not in counter]
        require(members, 'split-hinge-mismatch', 'Los tramos deben compartir la misma referencia rígida.', sourceBendId=source)
        parents = {resolved_panel(str(m['parentPanelId'])) for m in members}
        by_parent = defaultdict(list)
        for m in members:
            by_parent[resolved_panel(str(m['parentPanelId']))].append(m)
        chain = all(p == q or p in ancestors(q) or q in ancestors(p) for p in parents for q in parents)
        # One span on a panel every other span's parent hangs from: the whole
        # line is collinear in the flat part and is formed in one stroke.
        rooted = any(all(q == p or p in ancestors(q) for q in parents) for p in parents)
        if len(parents) > 1 and not counter and not chain and not rooted:
            # Spans on sibling flanges (e.g. two tabs per side flange): one
            # stroke per flange, so each is pressed on its own short setup
            # instead of one long stroke across both flanges.
            for group in by_parent.values():
                for i, m in enumerate(group):
                    m.update({'operationRelationCount': len(group), 'operationRelationIndex': i})
                if len(group) > 1:
                    queue.append((source, group))
            continue
        if len(parents) > 1 and not counter:
            couple_joint_stroke(source, members, resolved_panel, incoming, depth, point, length, thickness,
                                default_k, convention, direction_key, panel_geometry, axis_tol)
            continue
        lead = members[0]
        start, end = point(lead['hingeStart']), point(lead['hingeEnd'])
        direction = sub(end, start)
        require(math.hypot(*direction) > 1e-6, 'zero-length-hinge', 'Eje de pliegue de longitud nula.')
        direction = scale(direction, 1/math.hypot(*direction))
        transverse = [-direction[1], direction[0]]
        segments = []
        parent = resolved_panel(str(lead['parentPanelId']))
        reference = split_relation_parameters(lead, 1, length, thickness, default_k, convention, direction_key)
        for m in members:
            child = str(m['childPanelId'])
            require(resolved_panel(str(m['parentPanelId'])) == parent, 'split-hinge-mismatch',
                    'Los tramos deben compartir la misma referencia rígida.', sourceBendId=source)
            a, b = point(m['hingeStart']), point(m['hingeEnd'])
            require(math.dist(a,b)>1e-6 and all(on_axis(p, start, end, axis_tol) for p in (a,b)),
                    'noncollinear-hinge-segments', 'Las relaciones de la operación deben compartir la misma recta.', sourceBendId=source)
            sense = 1 if dot(sub(b,a),direction)>0 else -1
            params = split_relation_parameters(m, sense, length, thickness, default_k, convention, direction_key)
            require(params[:2] == reference[:2] and all(abs(v-r)<tol for v,r,tol in
                    zip(params[2:], reference[2:], (1e-6,1e-6,1e-8))),
                    'split-hinge-mismatch','Los tramos deben compartir el giro físico, radio, espesor y factor K.',sourceBendId=source)
            spans = m.get('hingeSegments', [[m['hingeStart'],m['hingeEnd']]])
            require(isinstance(spans,list) and spans, 'invalid-hinge-segments','Faltan tramos materiales.',sourceBendId=source)
            for span in spans:
                require(isinstance(span,list) and len(span)==2,'invalid-hinge-segment','Cada tramo necesita dos extremos.')
                pts = [point(p) for p in span]
                require(all(on_axis(p, start, end, axis_tol) for p in pts),
                        'noncollinear-hinge-segments','Un tramo queda fuera del eje.',sourceBendId=source)
                line = LineString(pts)
                require(line.length>1e-6 and all(line.difference(panel_geometry[pid].boundary.buffer(1e-5)).length<1e-4
                        for pid in (parent,child)), 'hinge-not-panel-boundary','El tramo debe pertenecer a ambos paneles de su relación.', bendId=m['bendId'])
            segments.extend(spans)
        for m in counter:
            tab, folded = str(m['childPanelId']), resolved_panel(str(m['parentPanelId']))
            a, b = point(m['hingeStart']), point(m['hingeEnd'])
            require(math.dist(a,b)>1e-6 and all(on_axis(p, start, end, axis_tol) for p in (a,b)),
                    'noncollinear-hinge-segments', 'Las relaciones de la operación deben compartir la misma recta.', sourceBendId=source)
            sense = 1 if dot(sub(b,a),direction)>0 else -1
            rotation, side, *rest = split_relation_parameters(m, sense, length, thickness, default_k, convention, direction_key)
            # Seen from the tab, the folded span turns the other way on the other side.
            require(rotation[0] == 'signed' and (('signed', -rotation[1]), -side) == reference[:2] and
                    all(abs(v-r)<tol for v,r,tol in zip(rest, reference[2:], (1e-6,1e-6,1e-8))),
                    'split-hinge-mismatch','El tramo opuesto debe invertir exactamente el giro del pliegue.',sourceBendId=source)
            spans = m.get('hingeSegments', [[m['hingeStart'],m['hingeEnd']]])
            require(isinstance(spans,list) and spans, 'invalid-hinge-segments','Faltan tramos materiales.',sourceBendId=source)
            for span in spans:
                require(isinstance(span,list) and len(span)==2,'invalid-hinge-segment','Cada tramo necesita dos extremos.')
                pts = [point(p) for p in span]
                require(all(on_axis(p, start, end, axis_tol) for p in pts),
                        'noncollinear-hinge-segments','Un tramo queda fuera del eje.',sourceBendId=source)
                line = LineString(pts)
                require(line.length>1e-6 and all(line.difference(panel_geometry[pid].boundary.buffer(1e-5)).length<1e-4
                        for pid in (folded,tab)), 'hinge-not-panel-boundary','El tramo debe pertenecer a ambos paneles de su relación.', bendId=m['bendId'])
            host = parent if counter_tabs == 'parent' else folded
            if counter_tabs == 'parent':
                segments.extend(spans)
            panel_geometry[host] = shapely.set_precision(unary_union([panel_geometry[host], panel_geometry[tab]]), 1e-8)
            rename[tab] = host; del panel_geometry[tab]
            merged_ids.add(id(m))
        ends = sorted((p for s in segments for p in s), key=lambda p: dot(sub(point(p), start), direction))
        child = str(lead['childPanelId'])
        panel_geometry[child] = shapely.set_precision(unary_union([panel_geometry[str(m['childPanelId'])] for m in members]), 1e-8)
        for m in members[1:]:
            rename[str(m['childPanelId'])] = child; del panel_geometry[str(m['childPanelId'])]
        merged_ids.update(id(m) for m in members[1:])
        lead.update({'hingeSegments': segments, 'hingeStart': ends[0], 'hingeEnd': ends[-1],
                     'operationRelationCount': 1, 'operationRelationIndex': 0, 'mergedSplitHingeSpans': len(members)+len(counter),
                     'sourceRelations': [{'bendId': m['bendId'], 'childPanelId': m['childPanelId'],
                                          'operationRelationIndex': m['operationRelationIndex']} for m in members] +
                                        [{'bendId': m['bendId'], 'childPanelId': m['childPanelId'],
                                          'operationRelationIndex': m['operationRelationIndex'],
                                          'counterSpanJoinsPanel': parent if counter_tabs == 'parent' else child,
                                          'counterTabs': counter_tabs} for m in counter]})
    for e in edges:
        if id(e) in merged_ids: continue
        if isinstance(e, dict) and str(e.get('parentPanelId')) in rename:
            e = {**e, 'parentPanelId': resolved_panel(str(e['parentPanelId']))}
        if isinstance(e, dict) and e.get('strokeRequiresFlatPanels'):
            # A later split hinge may have merged a required panel into another.
            e = {**e, 'strokeRequiresFlatPanels': sorted({resolved_panel(p) for p in e['strokeRequiresFlatPanels']})}
        result.append(e)
    return result



def retain_ba_rounding_wings_with_parent(band, parent, start, directed, transverse,
                                        intervals, neutral_length, exported_length, coordinate_precision):
    # Only the added parent-side strip caused by a small BA discrepancy qualifies.
    # Real oblique extensions reach the original strip and remain untouched.
    delta = neutral_length - exported_length
    if not (math.isfinite(coordinate_precision) and 0 < coordinate_precision and
            0 < exported_length and 0 < delta <= min(coordinate_precision, max(.005, neutral_length * 1e-3))):
        return band, []
    lo, hi = min(iv[0] for iv in intervals), max(iv[1] for iv in intervals)
    extent = neutral_length + 1
    axial_domain = Polygon([
        add(start, add(scale(directed, lo), scale(transverse, -extent))),
        add(start, add(scale(directed, hi), scale(transverse, -extent))),
        add(start, add(scale(directed, hi), scale(transverse, extent))),
        add(start, add(scale(directed, lo), scale(transverse, extent)))])
    wings = band.difference(axial_domain)
    retained = []
    for component in polygons(wings):
        radial = [dot(sub(p[:2], start), transverse) for p in coordinates(component)]
        # One set_precision grid unit, not the collision/contact tolerance.
        if (min(radial) < -neutral_length / 2 - 1e-8 or
                max(radial) > -exported_length / 2 + 1e-8):
            continue
        if component.difference(parent).area > 1e-12:
            continue
        retained.append(component)
    if not retained:
        return band, []
    result = shapely.set_precision(band.difference(unary_union(retained)), 1e-8)
    records = []
    for component in retained:
        contact = component.boundary.intersection(axial_domain.boundary)
        lines = list(contact.geoms) if hasattr(contact, 'geoms') else [contact]
        segments = [[list(a), list(b)] for line in lines if line.geom_type == 'LineString'
                    for a, b in zip(line.coords, list(line.coords)[1:]) if math.dist(a,b) > 1e-12]
        records.append({'areaMm2': component.area, 'boundsMm': list(component.bounds),
                        'capSegmentsFlatMm': segments})
    return result, records


def on_ba_rounding_cap(bend, panel_id, a, c=None):
    # A new non-weld contact is allowed only on a boundary actually reassigned
    # to this parent. A whole end plane or the opposite end is not sufficient.
    evidence = bend['source'].get('baRoundingRetainedWings')
    if not evidence or panel_id != bend['parentPanelId']:
        return False
    segments = [LineString(segment) for record in evidence['components']
                for segment in record['capSegmentsFlatMm']]
    if not segments:
        return False
    boundary = unary_union(segments)
    if c is None:
        return Point(a[:2]).distance(boundary) <= 1e-8
    return LineString([a[:2], c[:2]]).difference(boundary.buffer(1e-8)).length <= 1e-8


def import_bcbend(source, output=None, sign_convention='auto', merge_split_hinges=True, *, step_dimensions=None,
                 counter_tabs=None, _source_info=None):
    """Return a model dict, optionally writing model + .verification.json sidecar.

    auto: every bend must supply signedSweepDegrees, about its exported directed
    hinge. directed-positive: caller declares foldAngleDegrees positive about that
    hinge. direction-child: caller declares direction UP/DOWN means the child bends
    toward +Z/-Z from the parent, independent of hinge direction. Explicit signed
    sweep wins in every mode; conflicting magnitudes are rejected.
    """
    require(sign_convention in ['auto', 'directed-positive', 'direction-child'],
            'unknown-sign-convention', 'Convención de signo no admitida.')
    require(hasattr(shapely, 'constrained_delaunay_triangles'), 'missing-shapely-2.1', 'Se requiere Shapely 2.1 o posterior.')
    source = Path(source)
    try:
        source_info = read_bcbend_document(source) if _source_info is None else _source_info
        data = source_info['document']
    except BCBendJsonError as exc:
        raise ImportBCBendError('invalid-json', str(exc)) from exc
    require(isinstance(data, dict), 'invalid-schema', 'Se esperaba un objeto BCBEND.')
    g = data.get('sheetMetalGraph')
    require(isinstance(g, dict) and str(g.get('schemaVersion')) in ['5', '5.0'],
            'unsupported-schema', 'Se requiere sheetMetalGraph versión 5 con contorno y partición planar.')
    # The exporter records which counter-tab reading the folded CAD body confirmed.
    counter_tabs = counter_tabs or g.get('counterTabs') or 'folded'
    require(counter_tabs in ('folded', 'parent'), 'invalid-counter-tabs', 'counterTabs debe ser folded o parent.')
    units = g.get('units')
    require(units in ['mm', 'in'], 'unsupported-units', 'Las unidades deben ser mm o in.')
    unit = 25.4 if units == 'in' else 1.
    # Exporters that snap vertices to a declared grid can displace a point by up
    # to half a cell per axis (half the diagonal); hinge collinearity must accept that.
    grid = g.get('coordinateGrid_'+units)
    collinearity_tolerance = 0.
    if isinstance(grid, (int, float)) and 0 < grid*unit <= 0.01:
        collinearity_tolerance = grid*unit*math.sqrt(2)/2
    warnings = []

    def length_keys(name):
        return [name+'_'+units, name+'Mm' if units == 'mm' else name+'In', name]

    def has_length(obj, name):
        return any(key in obj for key in length_keys(name))

    def length(obj, name, default=None):
        for key in length_keys(name):
            if key in obj: return number(obj[key], key)*unit
        if default is not None: return default
        raise ImportBCBendError('missing-dimension', 'Falta dimensión '+name)

    def point(p):
        require(isinstance(p, dict), 'invalid-point', 'Cada punto debe contener coordenadas x/y.')
        return [length(p, 'x'), length(p, 'y')]

    point_count = 0

    def polygon(record, label, panel=False):
        nonlocal point_count
        require(isinstance(record, dict) and isinstance(record.get('outerLoop'), list),
                'missing-contour', 'Falta contorno material: '+label)
        require(isinstance(record.get('holeLoops', []), list), 'invalid-holes', 'Los agujeros deben ser una lista: '+label)
        loops = [record['outerLoop'], *record.get('holeLoops', [])]
        origin, u, v = [0., 0.], [1., 0.], [0., 1.]
        if panel:
            origin = point(record.get('origin', {'x': 0., 'y': 0.}))
            u = [number(record.get('u', {'x': 1., 'y': 0.}).get(k), 'u.'+k) for k in ['x', 'y']]
            v = [number(record.get('v', {'x': 0., 'y': 1.}).get(k), 'v.'+k) for k in ['x', 'y']]
            require(abs(dot(u, u)-1) < 1e-6 and abs(dot(v, v)-1) < 1e-6 and abs(dot(u, v)) < 1e-6
                    and u[0]*v[1]-u[1]*v[0] > .999999,
                    'invalid-panel-frame', 'El marco planar debe ser ortonormal y directo: '+label)
        transformed = []
        for loop in loops:
            require(isinstance(loop, list) and len(loop) >= 3, 'invalid-loop', 'Contorno con menos de tres puntos: '+label)
            point_count += len(loop)
            require(point_count <= 100000, 'point-limit', 'Más de 100000 puntos de contorno.')
            points = []
            for item in loop:
                xy = point(item); q = add(origin, add(scale(u, xy[0]), scale(v, xy[1])))
                require(max(map(abs, q)) < 1e6, 'coordinate-limit', 'Coordenadas fuera del rango admitido.')
                points.append(q)
            transformed.append(points)
        p = Polygon(transformed[0], transformed[1:])
        require(p.is_valid and not p.is_empty and p.area > 1e-8, 'invalid-polygon', 'Contorno inválido, cruzado o vacío: '+label)
        return shapely.set_precision(p, 1e-8)

    source_panels, source_edges = g.get('panels'), g.get('bendEdges')
    require(isinstance(source_panels, list) and 1 <= len(source_panels) <= 200 and
            isinstance(source_edges, list) and len(source_edges) == len(source_panels)-1,
            'invalid-tree-size', 'Se requiere un árbol conectado de 1 a 200 paneles.')
    root = str(g.get('rootPanelId', ''))
    panel_geometry = {}
    for p in source_panels:
        require(isinstance(p, dict), 'invalid-panel', 'Cada panel debe ser un objeto.')
        pid = str(p.get('panelId', ''))
        require(pid and pid not in panel_geometry, 'duplicate-panel-id', 'Identidad de panel ausente o duplicada.')
        require(pid not in RESERVED_IDS, 'reserved-identifier', 'Identificador reservado por el consumidor JavaScript.')
        panel_geometry[pid] = polygon(p, pid, panel=True)
    require(root in panel_geometry, 'missing-root', 'El panel raíz no existe.')
    has_compound = any(isinstance(e, dict) and e.get('operationRelationCount', 1) != 1 for e in source_edges)
    material = polygon(g.get('materialBoundary'), 'materialBoundary')
    area_tol = max(1e-5, material.area*1e-9)
    panel_union = unary_union(list(panel_geometry.values()))
    partition_error = panel_union.symmetric_difference(material).area
    overlap_area = sum(p.area for p in panel_geometry.values())-panel_union.area
    require(partition_error <= area_tol and overlap_area <= area_tol,
            'invalid-panel-partition', 'Los paneles deben particionar el material sin huecos ni solapes.',
            partitionErrorMm2=partition_error, overlapAreaMm2=overlap_area)
    mat = g.get('material', {})
    require(isinstance(mat, dict), 'invalid-material', 'El material debe ser un objeto.')
    thickness = length(mat, 'thickness')
    require(0 < thickness < 1000, 'invalid-thickness', 'Espesor uniforme fuera de rango.')
    default_k = number(mat.get('kFactor'), 'material.kFactor')
    require(0 <= default_k <= 1, 'invalid-k-factor', 'El factor K debe estar entre 0 y 1.')
    exporter_direction = False
    generator = data.get('generator')
    fusion_v5 = (isinstance(generator, dict) and re.fullmatch(r'com\.(?:[a-z0-9_]+\.)*?bend(?:control|cnc)\.export', str(generator.get('id') or ''), re.I)
                 and generator.get('revision') == '2026-08-06-compound-operation-relations-v5')
    direction_key = 'flatPatternDirection' if fusion_v5 else 'direction'
    if has_compound and merge_split_hinges:
        if (sign_convention == 'auto' and fusion_v5
                and any(isinstance(e, dict) and 'signedSweepDegrees' not in e for e in source_edges)):
            sign_convention = 'direction-child'
            exporter_direction = True
            warnings.append({'code':'exporter-direction-child','message':'Convención UP/DOWN de la revisión v5 del exportador Fusion.'})
        source_edges = merge_split_hinge_edges(source_edges, panel_geometry, point, length, thickness, default_k, root, sign_convention, direction_key,
                                                   axis_tol=collinearity_tolerance, counter_tabs=counter_tabs)
    # Missing or malformed precision supplies no evidence for reassignment.
    # length() converts the declared graph-unit dimension to millimetres.
    try:
        diagnostics = g.get('diagnostics', {})
        coordinate_precision = length(diagnostics, 'coordinatePrecision', 0.) if isinstance(diagnostics, dict) else 0.
    except ImportBCBendError:
        coordinate_precision = 0.
    incoming, by_id, bands = {}, {}, {}
    for e in source_edges:
        require(isinstance(e, dict), 'invalid-bend-edge', 'Cada relación de pliegue debe ser un objeto.')
        bid = str(e.get('bendId', '')); parent, child = str(e.get('parentPanelId')), str(e.get('childPanelId'))
        require(bid not in RESERVED_IDS, 'reserved-identifier', 'Identificador reservado por el consumidor JavaScript.')
        require(bid and bid not in by_id and parent in panel_geometry and child in panel_geometry and
                parent != child and child != root and child not in incoming,
                'invalid-tree-edge', 'Identidad o relación de pliegue inválida.', bendId=bid)
        # Without merging, each physical relation is its own stroke (one per span).
        require(not merge_split_hinges or e.get('operationRelationCount', 1) == 1,
                'compound-operation-unsupported', 'La operación requiere agrupar sus tramos coaxiales.', bendId=bid)
        incoming[child] = parent
        require(abs(length(e, 'thickness', thickness)-thickness) <= 1e-6,
                'nonuniform-thickness', 'Los pliegues no comparten el espesor del material.', bendId=bid)
        ri = length(e, 'insideRadius'); k = number(e.get('kFactor', default_k), 'kFactor')
        require(0 <= ri < 1e5 and 0 <= k <= 1, 'invalid-bend-material', 'Radio o factor K inválidos.', bendId=bid)
        explicit = number(e['signedSweepDegrees'], 'signedSweepDegrees') if 'signedSweepDegrees' in e else None
        angle = number(e.get('foldAngleDegrees', abs(explicit) if explicit is not None else None), 'foldAngleDegrees')
        require(0 < angle < 180-1e-6, 'unsupported-sweep', 'Se requiere un barrido físico entre 0 y 180 grados.', bendId=bid)
        start, end = point(e.get('hingeStart')), point(e.get('hingeEnd'))
        delta = sub(end, start); span = math.hypot(*delta)
        require(span > 1e-6, 'zero-length-hinge', 'Eje de pliegue de longitud nula.', bendId=bid)
        directed = scale(delta, 1/span)
        side = number(e.get('childSideSign'), 'childSideSign')
        require(side in [-1, 1], 'invalid-child-side', 'childSideSign debe ser -1 o 1.')
        axis = [directed[0]*side, directed[1]*side, 0.]
        transverse = [-axis[1], axis[0], 0.]
        if 'signedSweepDegrees' in e:
            directed_angle = number(e['signedSweepDegrees'], 'signedSweepDegrees')
            require(abs(abs(directed_angle)-angle) < 1e-6, 'conflicting-sweep', 'El barrido firmado contradice foldAngleDegrees.', bendId=bid)
            theta = math.radians(directed_angle)*side; sign_source = 'explicit-signedSweepDegrees-about-directed-hinge'
        elif sign_convention == 'directed-positive':
            theta = math.radians(angle)*side; sign_source = 'caller-declared-directed-positive'
        elif sign_convention == 'direction-child':
            direction = e.get(direction_key, e.get('direction'))
            require(direction in ['UP', 'DOWN'], 'missing-direction', 'direction-child requiere direction UP/DOWN.', bendId=bid)
            theta = math.radians(angle)*(1 if direction == 'UP' else -1)
            sign_source = 'exporter-direction-child' if exporter_direction else 'caller-declared-direction-child'
            if direction_key == 'flatPatternDirection' and direction_key in e:
                sign_source = 'fusion-flatPatternDirection'
        else:
            raise ImportBCBendError('ambiguous-bend-sign', 'Falta signedSweepDegrees. Declare la convención de signo; no se deduce de tokens heredados.', bendId=bid)
        if 'signedSweepDegrees' not in e and not exporter_direction:
            warnings.append({'code': 'declared-legacy-sign', 'bendId': bid, 'convention': sign_convention,
                             'message': 'Signo declarado por el usuario; no verificado contra geometría 3D original.'})
        measurement = (step_dimensions or {}).get(str(e.get('sourceBendId', bid)))
        if measurement:
            declared = {'bendFoldDegrees':angle, 'bendInsideRadiusMm':ri}
            angle, ri = measurement['sweepDegrees'], measurement['insideRadiusMm']
            require(explicit is None or abs(abs(explicit)-angle)<1e-4, 'conflicting-sweep',
                    'El barrido firmado del BEND difiere del STEP.', bendId=bid)
            theta = math.copysign(math.radians(angle), theta)
        neutral_length = abs(theta)*(ri+k*thickness)
        require(neutral_length > 1e-7, 'zero-neutral-length', 'La longitud neutra debe ser positiva.', bendId=bid)
        segments = e.get('hingeSegments', [[e['hingeStart'], e['hingeEnd']]])
        require(isinstance(segments, list) and 1 <= len(segments) <= 1000, 'invalid-hinge-segments', 'Segmentos de eje inválidos.')
        intervals = []
        for segment in segments:
            require(isinstance(segment, list) and len(segment) == 2, 'invalid-hinge-segment', 'Cada segmento requiere dos extremos.')
            p0, p1 = map(point, segment)
            require(math.dist(p0, p1) > 1e-6 and all(on_axis(p, start, end, collinearity_tolerance) for p in [p0, p1]),
                    'noncollinear-hinge-segments', 'Los segmentos deben ser rectos y coaxiales.', bendId=bid)
            intervals.append(sorted([dot(sub(p, start), directed) for p in [p0, p1]]))
        intervals.sort()
        require(all(a[1] <= b[0]+1e-6 for a, b in zip(intervals, intervals[1:])),
                'overlapping-hinge-segments', 'Los intervalos de eje se solapan.', bendId=bid)
        pair = panel_geometry[parent].union(panel_geometry[child])
        # A hinge segment gives material at the centre line, not a square end
        # cap. Oblique exterior edges / round reliefs can extend axially beyond
        # its endpoints on either skin of the strip. Trim by actual material.
        projections = [dot(sub(p[:2], start), directed) for p in coordinates(pair)]
        extended_start = add(start, scale(directed, min(projections)-1))
        extended_end = add(start, scale(directed, max(projections)+1))
        strip = shapely.set_precision(LineString([extended_start, extended_end]).buffer(neutral_length/2, cap_style=2), 1e-8)
        band = strip.intersection(pair)
        retained_wings = []
        if has_length(e, 'bendAllowance'):
            band, retained_wings = retain_ba_rounding_wings_with_parent(
                band, panel_geometry[parent], start, directed, transverse[:2],
                intervals, neutral_length, length(e, 'bendAllowance'), coordinate_precision)
            if retained_wings:
                warnings.append({'code': 'ba-rounding-wing-retained-in-parent', 'bendId': bid,
                                 'computedMm': neutral_length, 'exportedMm': length(e, 'bendAllowance'),
                                 'components': retained_wings, 'declaredCoordinatePrecisionMm': coordinate_precision,
                                 'message': 'Los extremos creados por el redondeo de BA permanecen en el panel padre; se conserva el material.'})
        declared_line = unary_union([LineString([point(s[0]), point(s[1])]) for s in segments])
        for pid in [parent, child]:
            require(declared_line.difference(panel_geometry[pid].boundary.buffer(1e-5)).length < 1e-4,
                    'hinge-not-panel-boundary', 'El eje declarado no coincide con la frontera planar de ambos paneles.',
                    bendId=bid, panelId=pid)
        band = unary_union([component for component in polygons(band)
                            if component.intersection(declared_line.buffer(1e-7)).area > 1e-9])
        actual_line = band.intersection(LineString([extended_start, extended_end]))
        # Merged split hinge: the parent stays flat between spans, so material
        # there is not part of the bend; only the declared spans must be material.
        # Coupled-stroke spans (tabs on sibling flanges) likewise: the parent
        # edge continues flat past the tab.
        trimmed = e.get('mergedSplitHingeSpans') or e.get('strokeLeadBendId') or e.get('strokeFollowerBendIds')
        require((trimmed or actual_line.difference(declared_line.buffer(1e-5)).length < 1e-4) and
                declared_line.difference(band.buffer(1e-5)).length < 1e-4,
                'hinge-material-interval-mismatch', 'Los segmentos no describen todos los intervalos materiales de la banda.', bendId=bid)
        if trimmed:
            # Between spans the parent stays flat: keep only the strip over
            # each declared span (relief cuts separate the rest).
            clip_intervals = intervals
            if e.get('strokeLeadBendId') or e.get('strokeFollowerBendIds'):
                clip_intervals = coupled_span_clip(intervals, start, directed, transverse[:2],
                                                   neutral_length, band, material, collinearity_tolerance)
            band = unary_union([band.intersection(Polygon([
                add(start, add(scale(directed, iv[0]), scale(transverse[:2], s*neutral_length))),
                add(start, add(scale(directed, iv[1]), scale(transverse[:2], s*neutral_length))),
                add(start, add(scale(directed, iv[1]), scale(transverse[:2], -s*neutral_length))),
                add(start, add(scale(directed, iv[0]), scale(transverse[:2], -s*neutral_length)))]))
                for iv in clip_intervals for s in [1]])
            band = shapely.set_precision(band, 1e-8)
        require(not band.is_empty and band.is_valid and band.area > 1e-8, 'empty-band', 'La banda no contiene material.', bendId=bid)
        o = sub([*start, 0.], scale(transverse, neutral_length/2))
        b = {'id': bid, 'label': 'B'+str(e.get('sourceBendId', bid)), 'parentPanelId': parent, 'childPanelId': child,
             'origin': o, 'axis': axis, 'transverse': transverse, 'normal': [0., 0., 1.],
             'signedAngle': theta, 'neutralLength': neutral_length,
             'neutralZ': -k*thickness if theta > 0 else -(1-k)*thickness,
             'insideRadius': ri, 'kFactor': k, 'sourceBendId': str(e.get('sourceBendId', bid)),
             'source': {'signConvention': sign_source, 'foldSweepDegrees': angle,
                        'directedHingeStartMm': start, 'directedHingeEndMm': end,
                        'childSideSign': side, 'sequenceHint': e.get('sequenceHint'),
                        'hingeMeaning': 'developed-neutral-strip-centreline'}}
        if retained_wings:
            b['source']['baRoundingRetainedWings'] = {
                'maximumParentDepthMm': (neutral_length-length(e, 'bendAllowance'))/2,
                'declaredCoordinatePrecisionMm': coordinate_precision,
                'components': retained_wings}
        if measurement:
            b['source']['stepDimensions'] = {**measurement, **declared}
            if abs(declared['bendFoldDegrees']-angle)>.05:
                warnings.append({'code':'step-sweep-differs','bendId':bid,
                                 'message':f"Barrido BEND {declared['bendFoldDegrees']:g}°, STEP {angle:g}°. Se usa el STEP."})
            if abs(declared['bendInsideRadiusMm']-ri)>.01:
                warnings.append({'code':'step-radius-differs','bendId':bid,
                                 'message':f"Radio interior BEND {declared['bendInsideRadiusMm']:.3f} mm, STEP {ri:.3f} mm."})
        if sign_source=='fusion-flatPatternDirection' and e.get('direction')!=e['flatPatternDirection']:
            b['source'].update(flatPatternDirection=e['flatPatternDirection'],processDirection=e.get('direction'))
            warnings.append({'code':'flat-process-direction-differ','bendId':bid,
                             'message':f"El desarrollo indica {e['flatPatternDirection']} y la cadena de proceso indica {e.get('direction')}. El giro usa el desarrollo."})
        if has_length(e, 'bendAllowance'):
            exported = length(e, 'bendAllowance'); difference = exported-neutral_length
            if abs(difference) > max(.005, neutral_length*1e-3):
                warnings.append({'code': 'bend-allowance-recomputed', 'bendId': bid, 'exportedMm': exported,
                                 'computedMm': neutral_length, 'differenceMm': difference,
                                 'message': 'BA recalculada desde el barrido físico, radio interior, espesor y K; el valor heredado difiere.'})
        if e.get('mergedSplitHingeSpans'):
            b['splitHingeSpans'] = e['mergedSplitHingeSpans']
            b['source']['relations'] = e['sourceRelations']
            b['source']['materialIntervalsAxialMm'] = sorted(sorted(v*side for v in iv) for iv in intervals)
        elif len(intervals) > 1:
            # One relation split by a hole/cutout: its caps are span ends too.
            b['source']['segmentIntervalsAxialMm'] = sorted(sorted(v*side for v in iv) for iv in intervals)
        if e.get('strokeLeadBendId'):
            b['strokeLeadId'] = e['strokeLeadBendId']
        if e.get('strokeFollowerBendIds'):
            b['strokeFollowerIds'] = list(e['strokeFollowerBendIds'])
            b['strokeRequiresFlatPanels'] = list(e['strokeRequiresFlatPanels'])
        by_id[bid] = b; bands[bid] = band
    by_child = {b['childPanelId']: b['id'] for b in by_id.values()}
    for b in by_id.values():
        if 'strokeRequiresFlatPanels' in b:
            require(all(p in by_child for p in b['strokeRequiresFlatPanels']) and
                    all(f in by_id and by_id[f].get('strokeLeadId') == b['id'] for f in b['strokeFollowerIds']),
                    'split-hinge-mismatch', 'Golpe conjunto sin pliegues intermedios resolubles.', bendId=b['id'])
            # Joint stroke precondition: these bends are still flat when the line is formed.
            b['strokeRequiresFlatBendIds'] = [by_child[p] for p in b.pop('strokeRequiresFlatPanels')]
    ordered, reached = [], {root}
    while len(ordered) < len(by_id):
        next_bends = [b for b in by_id.values() if b['parentPanelId'] in reached and b['childPanelId'] not in reached]
        require(next_bends, 'cyclic-or-disconnected-graph', 'El grafo contiene un ciclo o está desconectado.')
        for b in sorted(next_bends, key=lambda b: b['id']):
            ordered.append(b); reached.add(b['childPanelId'])
    require(len(reached) == len(panel_geometry), 'disconnected-panels', 'Hay paneles desconectados.')
    band_list = list(bands.items())
    if has_compound and merge_split_hinges:
        # Split-hinge tabs often sit at box corners with reliefs narrower than
        # the neutral strip; the corner stays flat with the parent, clear of both.
        for i in range(len(band_list)):
            for j in range(i+1, len(band_list)):
                left, right = (by_id[band_list[k][0]] for k in (i,j))
                # Joint stroke: a follower span ends at the relief of the bend that
                # carries its parent; that bend's strip keeps the corner.
                lead_of = lambda b: by_id.get(b.get('strokeLeadId'), {})
                pair_follower = next(((k, f, o) for k, f, o in ((i, left, right), (j, right, left))
                                      if o['id'] in lead_of(f).get('strokeRequiresFlatBendIds', [])), None)
                if pair_follower:
                    k, f, o = pair_follower
                    overlap = band_list[i][1].intersection(band_list[j][1]).area
                    if area_tol < overlap < .05*min(band_list[i][1].area, band_list[j][1].area):
                        corner = band_list[i][1].intersection(band_list[j][1])
                        band_list[k] = (band_list[k][0], trim_band_corner(band_list[k][1], corner, f))
                        warnings.append({'code':'joint-stroke-corner','bends':[f['label'],o['label']],'overlapMm2':overlap,
                                         'message':'La esquina del alivio queda en la banda del pliegue que sostiene al tramo.'})
                    continue
                if not (left.get('splitHingeSpans') and right.get('splitHingeSpans') and
                        left['parentPanelId'] == right['parentPanelId']):
                    continue
                overlap = band_list[i][1].intersection(band_list[j][1]).area
                if area_tol < overlap < .05*min(band_list[i][1].area, band_list[j][1].area):
                    corner = band_list[i][1].intersection(band_list[j][1])
                    if not all(all(min(abs(dot(sub([*xy[:2],0],b['origin']),b['axis'])-end)
                                       for iv in b['source']['materialIntervalsAxialMm'] for end in iv)
                                   <= b['neutralLength']/2+1e-5 for xy in coordinates(corner)) for b in (left,right)):
                        continue
                    band_list[i] = (band_list[i][0], trim_band_corner(band_list[i][1], corner, by_id[band_list[i][0]]))
                    band_list[j] = (band_list[j][0], trim_band_corner(band_list[j][1], corner, by_id[band_list[j][0]]))
                    warnings.append({'code':'nominal-split-corner','bends':[left['label'],right['label']],
                                     'overlapMm2':overlap,'cornerClearanceMm':CORNER_CLEARANCE_MM,
                                     'message':'El reparto del material en la esquina de las bandas es nominal.'})
        bands.update(band_list)
    # Corner slivers (relief rounding in the flat pattern) are numerical, not
    # physical: assign them to the earlier band and record a warning.
    sliver_tol = 1e-3
    for i in range(len(band_list)):
        for j in range(i+1, len(band_list)):
            (bid, band), (oid, other) = band_list[i], band_list[j]
            shared = band.intersection(other)
            overlap = shared.area
            # A corner patch no larger than the sheet thickness in either
            # direction is relief rounding, whatever the band sizes.
            x0, y0, x1, y1 = shared.bounds if overlap > area_tol else (0, 0, 0, 0)
            corner = max(x1-x0, y1-y0) <= thickness + 1e-6 and overlap < .05*min(band.area, other.area)
            if area_tol < overlap and (overlap <= sliver_tol*min(band.area, other.area) or corner):
                trimmed = other.difference(band)
                # Re-node the kept band so both share the corner vertices exactly.
                band = band.union(trimmed).difference(trimmed)
                band_list[i], band_list[j] = (bid, band), (oid, trimmed)
                warnings.append({'code': 'band-corner-sliver', 'bends': [by_id[bid]['label'], by_id[oid]['label']],
                                 'overlapMm2': overlap,
                                 'message': 'Solape numérico mínimo en la esquina de bandas; asignado a la primera banda.'})
    bands.update(band_list)
    for i, (bid, band) in enumerate(band_list):
        for oid, other in band_list[i+1:]:
            overlap = band.intersection(other).area
            require(overlap <= area_tol, 'overlapping-bands', 'Las bandas finitas se solapan; se requiere una partición física inequívoca.',
                    bendId=bid, otherBendId=oid, overlapAreaMm2=overlap)
    band_union = unary_union(list(bands.values()))
    rigid = {pid: p.difference(band_union) for pid, p in panel_geometry.items()}
    # A relief-corner sliver of a split-hinge child left inside its own band's
    # strip (the band was notched by a sibling parent) is band material: a
    # rigid island there would hang off the parent and open a seam on folding.
    for b in ordered:
        pid, band = b['childPanelId'], bands[b['id']]
        if not (b.get('splitHingeSpans') or b.get('mergedSplitHingeSpans') or b.get('strokeLeadId')) or pid not in rigid: continue
        parts = list(getattr(rigid[pid], 'geoms', [rigid[pid]]))
        if len(parts) < 2: continue
        strip = band.minimum_rotated_rectangle.buffer(1e-6)
        islands = [g for g in parts if g.area < 1. and strip.contains(g)]
        if islands and len(islands) < len(parts):
            bands[b['id']] = shapely.set_precision(unary_union([band, *islands]), 1e-8)
            rigid[pid] = unary_union([g for g in parts if g not in islands])
    for pid, region in rigid.items():
        require(region.is_valid and not region.is_empty and region.area > 1e-8,
                'collapsed-panel', 'Las bandas consumen un panel completo.', panelId=pid)
    reassembled = unary_union([*rigid.values(), *bands.values()])
    reconstruction_error = reassembled.symmetric_difference(material).area
    require(reconstruction_error <= area_tol, 'material-area-loss', 'La división de bandas pierde material.')
    # The two tangent boundaries must connect only to the specified rigid panels.
    # Axial caps in the middle of material cannot be modelled as a cylindrical joint.
    for b in ordered:
        band = bands[b['id']]; lengths = {}
        for pid, region in rigid.items():
            interface = band.boundary.intersection(region.boundary)
            if band.boundary.distance(region.boundary) < 1e-6:
                # Coincident edges that differ by float noise (merged spans
                # rebuilt from unions) intersect only at points; match with a
                # sub-micron tolerance when that recovers a real edge.
                tolerant = band.boundary.intersection(region.boundary.buffer(1e-6))
                if tolerant.length > interface.length + 1e-3:
                    interface = tolerant
            if interface.length < 1e-6: continue
            if pid not in [b['parentPanelId'], b['childPanelId']] and interface.length <= 4*CORNER_CLEARANCE_MM + 1e-6 and any(
                    f.get('childPanelId') == pid and b['id'] in by_id.get(f.get('strokeLeadId'), {}).get('strokeRequiresFlatBendIds', [])
                    for f in ordered):
                continue  # relief corner of a joint-stroke follower (stays flat while b folds after it)
            require(pid in [b['parentPanelId'], b['childPanelId']], 'unexpected-band-neighbour', 'La banda toca un tercer panel.', bendId=b['id'], panelId=pid)
            lines = list(interface.geoms) if hasattr(interface, 'geoms') else [interface]
            expected = 0 if pid == b['parentPanelId'] else 1
            lengths[pid] = 0.
            ends = [x for span in b['source'].get('materialIntervalsAxialMm',[]) for x in span]
            if not ends and (b.get('strokeLeadId') or b.get('strokeFollowerIds')):
                ext = [dot(sub([*p[:2],0.], b['origin']), b['axis']) for p in coordinates(band)]
                ends = [min(ext), max(ext)]
            ends += [x for span in b['source'].get('segmentIntervalsAxialMm', []) for x in span]
            for line in lines:
                for a,c in zip(line.coords, list(line.coords)[1:]):
                    local = [sub([*p[:2],0.], b['origin']) for p in (a,c)]
                    residual = max(abs(dot(p,b['transverse'])/b['neutralLength']-expected) for p in local)
                    if residual < 1e-5:
                        lengths[pid] += math.dist(a,c)
                        continue
                    axial = [dot(p,b['axis']) for p in local]
                    rounding_cap = on_ba_rounding_cap(b, pid, a, c)
                    # Mid-band cap: half the band, within 1 um of float/grid noise.
                    require(rounding_cap or (residual <= .5+max(1e-5, 1e-3/b['neutralLength']) and any(max(abs(x-end) for x in axial) <= b['neutralLength']/2+CORNER_CLEARANCE_MM+1e-5 for end in ends)),
                            'invalid-tangent-interface','El contacto fuera de tangencia debe corresponder al extremo de un tramo.',bendId=b['id'],panelId=pid,axial=axial,residual=residual,ends=ends)
                    b.setdefault('reliefContacts',{}).setdefault(pid,[]).append([min(axial),max(axial)])
        require(all(lengths.get(pid, 0) > 1e-5 for pid in [b['parentPanelId'], b['childPanelId']]),
                'missing-tangent-interface', 'La banda no conecta ambas tangencias.', bendId=b['id'])
    model = {'schemaVersion': 1, 'units': 'mm', 'rootPanelId': root, 'thicknessMm': thickness,
             'kFactor': default_k, 'panels': [], 'bends': ordered, 'seams': []}
    mesh_audits, mesh_lookup = {}, {}
    for pid, region in rigid.items():
        mesh, audit = closed_mesh(region, thickness)
        p = {'id': pid, 'label': pid, **mesh}; model['panels'].append(p)
        mesh_audits['panel:'+pid] = audit; mesh_lookup[('panel', pid)] = p
    for b in ordered:
        b['band'], audit = closed_mesh(bands[b['id']], thickness, b)
        mesh_audits['band:'+b['id']] = audit; mesh_lookup[('band', b['id'])] = b['band']
    require(sum(a['vertices'] for a in mesh_audits.values()) <= 1000000,
            'vertex-limit', 'La reconstrucción supera un millón de vértices.')
    shared = defaultdict(list)
    for (kind, mid), mesh in mesh_lookup.items():
        for i in range(0, len(mesh['flatPositions']), 3):
            key = tuple(round(v, 6) for v in mesh['flatPositions'][i:i+3])
            shared[key].append({'kind': kind, 'id': mid, 'vertexIndex': i//3})
    for occ in shared.values():
        if len(occ)<2: continue
        # Two different bend zones touching only at a relief corner are not a weld.
        if all(r['kind']=='band' for r in occ) and len({r['id'] for r in occ})>1: continue
        if not any(r['kind']=='band' and (by_id[r['id']].get('splitHingeSpans') or by_id[r['id']].get('strokeLeadId') or by_id[r['id']].get('strokeFollowerIds')) for r in occ):
            welded = []
            for r in occ:
                if r['kind'] == 'band':
                    bend = by_id[r['id']]
                    i = 3*r['vertexIndex']
                    fraction = bend['band']['parameters'][i+1]
                    flat = bend['band']['flatPositions'][i:i+3]
                    if (fraction >= 1e-7 and on_ba_rounding_cap(bend, bend['parentPanelId'], flat) and
                        any(other['kind']=='panel' and other['id']==bend['parentPanelId'] and
                            on_ba_rounding_cap(bend, other['id'],
                                mesh_lookup[('panel',other['id'])]['flatPositions'][3*other['vertexIndex']:3*other['vertexIndex']+3])
                            for other in occ)):
                        continue
                welded.append(r)
            if len(welded)>1:
                model['seams'].append(welded)
            continue
        # Coincident relief caps in the flat blank are not welded. Rigid panel
        # and band-end vertices share a frame precisely at their own tangency.
        by_frame = defaultdict(list)
        for r in occ:
            if r['kind']=='panel':
                by_frame[r['id']].append(r)
            else:
                bend = by_id[r['id']]
                fraction = bend['band']['parameters'][3*r['vertexIndex']+1]
                if fraction<1e-7: by_frame[bend['parentPanelId']].append(r)
                elif fraction>1-1e-7: by_frame[bend['childPanelId']].append(r)
        model['seams'].extend(refs for refs in by_frame.values() if len(refs)>1)
    # Joint stroke: the relief corner between a follower span and the bend that
    # carries its parent is a cut, not a joint (it opens once that bend folds).
    torn = set()
    for f in ordered:
        for oid in by_id.get(f.get('strokeLeadId'), {}).get('strokeRequiresFlatBendIds', []):
            torn |= {frozenset({x, y}) for x in (f['id'], f['childPanelId']) for y in (oid, by_id[oid]['childPanelId'])}
    if torn:
        ids = lambda seam: {r['id'] for r in seam}
        model['seams'] = [s for s in model['seams'] if not any(p <= ids(s) for p in torn)]
    states = [{b['id']: q for b in ordered} for q in [0., 1e-12, .1, .5, .9, 1.]]
    states += [{b['id']: 1. if i < cut else .37 if i == cut else 0. for i, b in enumerate(ordered)} for cut in range(len(ordered))]
    states += [{b['id']: ((i*37+23) % 100)/100 for i, b in enumerate(ordered)}]
    bounded_mesh = bounded_pose_inputs(model, mesh_lookup)
    checks = []
    for state in states:
        frames = evaluate_frames(model, state); gap, finite, worst = 0., True, []

        def position(ref):
            key = (ref['kind'], ref['id']); mesh = mesh_lookup[key]; i = 3*ref['vertexIndex']
            if key[0] == 'panel': return transform(frames[key[1]], mesh['flatPositions'][i:i+3])
            b = by_id[key[1]]
            return transform(frames[b['parentPanelId']], band_point(b, *mesh['parameters'][i:i+3], state[b['id']]))

        for seam in model['seams']:
            points = [position(ref) for ref in seam]
            seam_gap = max(math.dist(points[0], p) for p in points)
            if seam_gap > gap: gap, worst = seam_gap, sorted({r['id'] for r in seam})
        # Each state still evaluates its exact seam vertices above. For the
        # separate all-finite check, |sin|, |cos| <= 1 and the two quotients
        # in band_point are <= 2e5 (the near-zero series is bounded by 2).
        # Inputs and frames <= 1e12 therefore keep every intermediate below
        # 1e56, including the final three-product affine sum. Large or
        # nonfinite inputs use the per-vertex check below.
        bounded_state = (bounded_mesh and all(abs(x) <= 1 for x in state.values())
                         and all(abs(x) <= 1e12 for frame in frames.values() for x in frame))
        for (kind, mid), mesh in (() if bounded_state else mesh_lookup.items()):
            if kind == 'panel':
                positions, frame = mesh['flatPositions'], frames[mid]
                vertices = (transform(frame, positions[i:i+3]) for i in range(0, len(positions), 3))
            else:
                bend, parameters = by_id[mid], mesh['parameters']
                frame, amount = frames[bend['parentPanelId']], state[mid]
                vertices = (transform(frame, band_point(bend, *parameters[i:i+3], amount))
                            for i in range(0, len(parameters), 3))
            for x, y, z in vertices:
                finite &= math.isfinite(x) and math.isfinite(y) and math.isfinite(z)
        checks.append({'amounts': state, 'maximumSeamGapMm': gap, 'allFinite': finite, 'worstSeam': worst})
    max_gap = max(check['maximumSeamGapMm'] for check in checks)
    require(max_gap < 1e-5 and all(c['allFinite'] for c in checks), 'kinematic-seam-failed',
            'El plegado abre uniones o produce coordenadas no finitas.', maximumSeamGapMm=max_gap,
            worstSeam=max(checks, key=lambda c: c['maximumSeamGapMm'])['worstSeam'])
    source_hash = source_info['sourceSha256']
    verification = {'status': 'nominal-geometry-verified', 'independentFinalShapeVerified': False,
                    'sourceSha256': source_hash, 'sourceKind': 'bcbend', 'sourceUsesStep': False,
                    'panelCount': len(rigid), 'bendCount': len(bands), 'thicknessMm': thickness,
                    'materialAreaMm2': material.area, 'sourceHoleCount': len(material.interiors),
                    'sourcePartitionErrorMm2': partition_error, 'reconstructedMaterialErrorMm2': reconstruction_error,
                    'meshChecks': mesh_audits, 'allMeshesClosed': True,
                    'triangles': sum(a['triangles'] for a in mesh_audits.values()),
                    'vertices': sum(a['vertices'] for a in mesh_audits.values()),
                    'maximumSeamGapMm': max_gap, 'sharedSeamVertexCount': len(model['seams']),
                    'flatThicknessMm': thickness, 'checks': checks, 'warnings': warnings}
    part_metadata = data.get('part') if isinstance(data.get('part'), dict) else {}
    model['metadata'] = {'partName': str(part_metadata.get('name', source.stem)),
                         'sourceKind': 'bcbend', 'sourceSha256': {source.name: source_hash},
                         'sampleSpecific': False, 'geometryClass': 'nominal-planar-cylindrical-tree',
                         'geometryAuthority': 'nominal-reconstruction-from-bcbend',
                         'independentFinalShapeVerified': False, 'sourceUsesStep': False,
                         'signConvention': sign_convention, 'sourceFrame': 'BCBEND planar frame in mm; top skin z=0, bottom z=-thickness',
                         'rootSelection': {'mode': 'declared-graph-root', 'panelId': root},
                         'sourceSchemaVersion': 5, 'sourceGenerator': data.get('generator'),
                         'embeddedStep': source_info['embeddedStep'],
                         'sourceMetadata': {'part': part_metadata, 'material': mat,
                                            'exportedFrom': data.get('exportedFrom'), 'exportDate': data.get('exportDate')},
                         'provenance': {'engine': 'engine/import_bcbend.py', 'shapelyVersion': shapely.__version__,
                                        'sourceGraphAuthority': g.get('geometryAuthority'),
                                        'sourceGraphProvenance': g.get('provenance')},
                         'tolerances': {'seamMm': 1e-5, 'meshVertexRoundingMm': 1e-8,
                                        'maximumBandAngleStepDegrees': 3, 'planarAreaMm2': area_tol},
                         'warnings': warnings, 'validation': verification,
                         'limitations': ['Nominal geometry from sampled 2D loops, not an independent original 3D surface.',
                                         'No beveled walls, local forming, springback, contact mechanics or manufacturing certification.',
                                         'Hinges must be neutral-band centre lines in the declared common flat frame.',
                                         'Coaxial relations with a common rigid parent and the same physical sweep share a stroke. Split-band corner trims are nominal.'],
                         'sequenceNote': 'Topological geometry order only; does not certify a manufacturing sequence.'}
    if output is not None:
        output = Path(output); output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(model, separators=(',', ':'), allow_nan=False), encoding='utf-8')
        output.with_suffix('.verification.json').write_text(json.dumps(verification, indent=2, allow_nan=False), encoding='utf-8')
    return model


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bcbend', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--sign-convention', choices=['auto', 'directed-positive', 'direction-child'], default='auto')
    parser.add_argument('--merge-split-hinges', action=argparse.BooleanOptionalAction, default=True, help='fold compatible coaxial spans as one stroke (default)')
    parser.add_argument('--counter-tabs', choices=['folded', 'parent'], default=None,
                        help="counter-span tabs ride with the folded span or stay with the stroke parent (default: the graph's counterTabs, else folded)")
    args = parser.parse_args(argv)
    try:
        model = import_bcbend(args.bcbend, args.output, args.sign_convention, args.merge_split_hinges,
                              counter_tabs=args.counter_tabs)
        print(json.dumps({'status': 'ok', 'output': str(args.output.resolve()),
                          'verification': str(args.output.with_suffix('.verification.json').resolve()),
                          'panels': len(model['panels']), 'bends': len(model['bends']),
                          'thicknessMm': model['thicknessMm'], 'warnings': model['metadata']['warnings']}, ensure_ascii=False))
        return 0
    except ImportBCBendError as exc:
        print(json.dumps(exc.as_dict(), ensure_ascii=False), file=sys.stderr); return 2
    except (KeyError, TypeError, ValueError, OSError, GEOSException) as exc:
        print(json.dumps({'status': 'error', 'error': str(exc), 'code': 'invalid-bcbend', 'message': str(exc)}, ensure_ascii=False), file=sys.stderr); return 2


if __name__ == '__main__':
    sys.exit(main())
