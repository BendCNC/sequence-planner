"""Import a developable, constant-thickness STEP into the finite-band viewer.

Run with FreeCAD's bundled Python. This adapter calls the unmodified LGPL
FreeCAD SheetMetal NewUnfolder module; its copyright/license remain under
analysis/freecad/upstream/ (Alex Neufeld and the SheetMetal contributors).
No GUI, subprocess, fixed input hash or fixed face index is used.

Supported: one closed solid; planar skin regions; straight cylindrical bends
with a connected tree after grouping interrupted coaxial bands. Unsupported or
ambiguous topology is rejected rather than silently replaced with a hinge model.
"""
from __future__ import annotations

import argparse
import collections
import contextlib
import hashlib
import io
import json
import math
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
UPSTREAM = ROOT / 'analysis/freecad/upstream'


class ImportStepError(RuntimeError):
    def __init__(self, code, message, **details):
        super().__init__(message)
        self.code, self.details = code, details

    def as_dict(self):
        return {'status': 'unsupported', 'code': self.code,
                'message': str(self), 'details': self.details}


def require(condition, code, message, **details):
    if not condition:
        raise ImportStepError(code, message, **details)


def dependencies():
    try:
        import FreeCAD as App
        import Part
    except ImportError as exc:
        raise ImportStepError('freecad-runtime-required',
                              'Ejecute el importador con Python incluido en FreeCAD.') from exc
    # NetworkX is pure Python. Find an existing installation; never install or
    # import ABI-dependent modules from another Python runtime.
    try:
        import networkx as nx
    except ImportError:
        roots = []
        if os.environ.get('UNFOLD_NETWORKX_PATH'):
            roots.append(Path(os.environ['UNFOLD_NETWORKX_PATH']))
        # A sandbox can redirect LOCALAPPDATA while keeping the existing user
        # Python installations under the real home directory.
        local_roots = [Path.home()/'AppData/Local']
        if os.environ.get('LOCALAPPDATA'):
            local_roots.insert(0, Path(os.environ['LOCALAPPDATA']))
        for local in local_roots:
            roots += sorted((local/'Programs/Python').glob('Python*/Lib/site-packages'), reverse=True)
        found = next((p for p in roots if (p/'networkx/__init__.py').is_file()), None)
        require(found is not None, 'networkx-required',
                'Falta NetworkX. Configure UNFOLD_NETWORKX_PATH hacia una instalación compatible existente.')
        sys.path.append(str(found))
        import networkx as nx
    require((UPSTREAM/'SheetMetalNewUnfolder.py').is_file(), 'sheetmetal-runtime-required',
            'No se encontró el adaptador FreeCAD SheetMetal incluido en este proyecto.')
    if str(UPSTREAM) not in sys.path:
        sys.path.insert(0, str(UPSTREAM))
    import SheetMetalNewUnfolder as U
    return App, Part, nx, U


def xyz(v):
    return [v.x, v.y, v.z]


def kind(face):
    return type(face.Surface).__name__


def outward(face):
    a, b, c, d = face.ParameterRange
    return face.normalAt((a+b)/2, (c+d)/2)


def coaxial(a, b, length_tol=1e-5):
    return (abs(abs(a.Axis.dot(b.Axis))-1) < 1e-8
            and (b.Center-a.Center).cross(a.Axis).Length < length_tol)


def plane_partner(shape, index, thickness=None):
    """Find the opposite planar skin by geometry, not face ordering."""
    f = shape.Faces[index]
    n = outward(f)
    candidates = []
    for j, g in enumerate(shape.Faces):
        if j == index or kind(g) != 'Plane' or n.dot(outward(g)) > -1+1e-8:
            continue
        shift = g.CenterOfMass-f.CenterOfMass
        t = -shift.dot(n)
        if t <= 1e-7 or (thickness is not None and abs(t-thickness) > max(1e-5, thickness*1e-5)):
            continue
        if thickness is None and (shift+n*t).Length > max(1e-4, t*1e-4):
            continue
        if thickness is None and abs(f.Area-g.Area) > max(1e-4, f.Area*1e-6):
            continue
        moved = f.copy(); moved.translate(-n*t)
        try:
            overlap = moved.common(g).Area
        except Exception:
            continue
        # Once thickness is established on the root, oblique cuts can make the
        # two planar trims differ. Require substantial geometric overlap and a
        # unique partner, while retaining the exact sidewall geometry.
        if (abs(overlap-f.Area) <= max(1e-4, f.Area*1e-6) if thickness is None
                else overlap/min(f.Area, g.Area) > .5):
            candidates.append((t, j))
    candidates.sort()
    require(candidates, 'opposite-planar-skin-not-found',
            'No se encontró una piel plana opuesta de igual contorno y espesor uniforme.', face=index+1)
    best = candidates[0]
    require(len(candidates) == 1 or abs(candidates[1][0]-best[0]) > 1e-5,
            'opposite-planar-skin-ambiguous', 'La piel plana opuesta no es única.', face=index+1)
    return best[1], best[0]


def cylinder_partner(shape, index, thickness, excluded):
    f = shape.Faces[index]; s = f.Surface
    sweep = f.ParameterRange[1]-f.ParameterRange[0]
    candidates = []
    for j, g in enumerate(shape.Faces):
        if j in excluded or kind(g) != 'Cylinder' or not coaxial(s, g.Surface):
            continue
        r = g.Surface.Radius
        if abs(abs(r-s.Radius)-thickness) > max(1e-5, thickness*1e-5):
            continue
        if abs(g.ParameterRange[1]-g.ParameterRange[0]-sweep) > 1e-6:
            continue
        delta = f.CenterOfMass-s.Center
        axial = s.Axis*delta.dot(s.Axis)
        predicted = s.Center+axial+(delta-axial)*(r/s.Radius)
        error = (predicted-g.CenterOfMass).Length
        # A pair must have matching trimmed angular/axial material, not merely
        # two coaxial cylinders whose radii happen to differ by the thickness.
        area_error = abs(f.Area/s.Radius-g.Area/r)
        if error <= max(.005, thickness*.002) and area_error <= max(.005, f.Area/s.Radius*1e-5):
            candidates.append((error, j))
    require(len(candidates) == 1, 'opposite-cylinder-skin-ambiguous',
            'No se pudo asociar de forma única la piel cilíndrica opuesta.',
            face=index+1, candidates=[j+1 for _, j in candidates])
    return candidates[0][1]


def topology(shape, root_index, thickness, App, Part, nx, U):
    skin = U.build_graph_of_tangent_faces(shape, root_index)
    require(all(kind(shape.Faces[i]) in ('Plane', 'Cylinder') for i in skin),
            'non-developable-skin', 'La piel contiene superficies distintas de planos y cilindros.',
            faces=[i+1 for i in skin if kind(shape.Faces[i]) not in ('Plane', 'Cylinder')])
    planes = [i for i in skin if kind(shape.Faces[i]) == 'Plane']
    cylinders = [i for i in skin if kind(shape.Faces[i]) == 'Cylinder']
    plane_groups = sorted((sorted(c) for c in nx.connected_components(skin.subgraph(planes))), key=lambda c: min(c))
    groups = {}; face_group = {}
    for index, faces in enumerate(plane_groups):
        pid = 'ROOT' if root_index in faces else 'P'+str(index+1)
        groups[pid] = {'kind': 'panel', 'facesA': faces}
        n, p = outward(shape.Faces[faces[0]]), shape.Faces[faces[0]].CenterOfMass
        require(all(abs(outward(shape.Faces[i]).dot(n)-1) < 1e-8
                    and abs((shape.Faces[i].CenterOfMass-p).dot(n)) < 1e-5 for i in faces),
                'planar-group-not-coplanar', 'Las caras de un panel no son coplanares.', faces=[i+1 for i in faces])
        for i in faces: face_group[i] = pid
    require(not any(a in cylinders and b in cylinders for a, b in skin.edges),
            'adjacent-cylinder-patches-unsupported',
            'Cilindros subdivididos angularmente requieren consolidación CAD antes de importar.')
    band_groups = []
    for i in sorted(cylinders):
        neighbors = sorted({face_group[n] for n in skin.neighbors(i) if n in face_group})
        require(len(neighbors) == 2, 'bend-panel-count-unsupported',
                'Cada banda cilíndrica debe conectar exactamente dos regiones planas.',
                face=i+1, adjacentPanels=neighbors)
        sweep = shape.Faces[i].ParameterRange[1]-shape.Faces[i].ParameterRange[0]
        require(1e-6 < sweep < math.pi-1e-6, 'bend-sweep-unsupported',
                'Este importador admite barridos mayores que cero y menores que 180°; hems requieren otro modelo.',
                face=i+1, sweepDegrees=math.degrees(sweep))
        match = next((b for b in band_groups if b['neighbors'] == neighbors
                      and coaxial(shape.Faces[b['facesA'][0]].Surface, shape.Faces[i].Surface)
                      and abs(shape.Faces[b['facesA'][0]].Surface.Radius-shape.Faces[i].Surface.Radius) < 1e-5
                      and abs(b['sweep']-sweep) < 1e-6), None)
        if match: match['facesA'].append(i)
        else: band_groups.append({'kind': 'bend', 'facesA': [i], 'neighbors': neighbors, 'sweep': sweep})
    graph = nx.Graph(); graph.add_nodes_from(groups)
    for index, group in enumerate(band_groups, 1):
        bid = 'B'+str(index); group['id'] = str(index); groups[bid] = group
        for i in group['facesA']: face_group[i] = bid
        for parent in group['neighbors']: graph.add_edge(parent, bid)
    require(nx.is_tree(graph), 'closed-or-disconnected-kinematics',
            'La conectividad no forma un árbol de paneles y bandas; no se descartarán restricciones de cierre.',
            nodes=graph.number_of_nodes(), edges=graph.number_of_edges())
    matched = set(skin)
    for gid, group in groups.items():
        partner_indices = []
        for i in group['facesA']:
            j = (plane_partner(shape, i, thickness)[0] if group['kind'] == 'panel'
                 else cylinder_partner(shape, i, thickness, set(skin)))
            require(j not in matched, 'skin-mapping-not-bijective',
                    'Una cara opuesta coincide con varias regiones.', face=j+1)
            matched.add(j); partner_indices.append(j); face_group[j] = gid
        group['facesB'] = partner_indices
    edge_faces = {}
    face_edges = collections.defaultdict(list)
    face_hash = {f.hashCode(): i for i, f in enumerate(shape.Faces)}
    for ei, edge in enumerate(shape.Edges):
        ancestors = [face_hash[f.hashCode()] for f in shape.ancestorsOfType(edge, Part.Face)]
        edge_faces[ei] = ancestors
        for i in ancestors: face_edges[i].append(ei)
    skin_owners = dict(face_group)
    split_walls = {}
    for i in range(len(shape.Faces)):
        if i in face_group: continue
        adjacent_owners = {skin_owners[j] for ei in face_edges[i] for j in edge_faces[ei] if j in skin_owners}
        require(adjacent_owners, 'side-wall-without-skin',
                'Una pared lateral no tiene piel asociada; requiere subdivisión CAD.', face=i+1)
        if len(adjacent_owners) == 1:
            face_group[i] = adjacent_owners.pop()
        else:
            split_walls[i] = sorted(adjacent_owners)
    return groups, graph, face_group, edge_faces, face_edges, skin, split_walls


def region_volume(shape, group, thickness, Part):
    """Exact finite planar prism / annular sectors used only to split sidewalls.

    The sector covers its surface's axial parameter interval. Skin trimming and
    reliefs remain in the original side face intersected with this volume.
    """
    pieces = []
    for index in group['facesA']:
        face = shape.Faces[index]
        if group['kind'] == 'panel':
            pieces.append(face.extrude(-outward(face)*thickness))
            continue
        s = face.Surface; u0, u1, v0, v1 = face.ParameterRange
        radius = min(shape.Faces[i].Surface.Radius for i in group['facesA']+group['facesB'])
        sample = face.valueAt(u0, v0)
        center = s.Center+s.Axis*(sample-s.Center).dot(s.Axis)
        def at(u, r):
            point = face.valueAt(u, v0)
            return center+(point-center)*(r/s.Radius)
        ia, im, ib = [at(u, radius) for u in (u0, (u0+u1)/2, u1)]
        oa, om, ob = [at(u, radius+thickness) for u in (u0, (u0+u1)/2, u1)]
        wire = Part.Wire([Part.Arc(ia, im, ib).toShape(), Part.makeLine(ib, ob),
                          Part.Arc(ob, om, oa).toShape(), Part.makeLine(oa, ia)])
        pieces.append(Part.Face(wire).extrude(face.valueAt(u0, v1)-sample))
    return pieces[0] if len(pieces) == 1 else pieces[0].multiFuse(pieces[1:])


def shared_tangent_edges(shape, face_indices, band_indices, edge_faces):
    result = []
    for ei, ancestors in edge_faces.items():
        if any(i in face_indices for i in ancestors) and any(i in band_indices for i in ancestors):
            result.append(ei)
    require(result, 'missing-tangent-edge', 'No se encontró el borde de tangencia.')
    require(all(shape.Edges[i].Curve.TypeId == 'Part::GeomLine' for i in result),
            'non-straight-tangency', 'La banda tiene tangencias no rectas.')
    return sorted(result, key=lambda i: (-shape.Edges[i].Length, i))


def terms(phi):
    if abs(phi) < 1e-4:
        sq = phi*phi
        return 1-sq/6+sq*sq/120, phi*(.5-sq/24+sq*sq/720)
    return math.sin(phi)/phi, 2*math.sin(phi/2)**2/phi


def new_mesh():
    return {'flatPositions': [], 'sourcePositions': [], 'triangles': [], 'faceIds': [], 'triangleFaceIds': []}


def import_model(step_path, root_face=None, k_factor=.44, mesh_tolerance=.08):
    App, Part, nx, U = dependencies()
    step_path = Path(step_path).resolve()
    require(step_path.is_file(), 'input-not-found', 'No existe el archivo STEP.', path=str(step_path))
    require(math.isfinite(k_factor) and 0 <= k_factor <= 1, 'invalid-k-factor', 'K debe estar entre 0 y 1 (ANSI).')
    require(math.isfinite(mesh_tolerance) and 0 < mesh_tolerance <= 1, 'invalid-mesh-tolerance', 'Tolerancia de malla inválida.')
    shape = Part.Shape()
    try: shape.read(str(step_path))
    except Exception as exc: raise ImportStepError('step-read-failed', 'FreeCAD no pudo leer el STEP.', error=str(exc)) from exc
    require(len(shape.Solids) == 1 and shape.isValid() and shape.isClosed(), 'single-valid-solid-required',
            'Se requiere un único sólido STEP válido y cerrado.', solids=len(shape.Solids), valid=shape.isValid())
    shape = shape.Solids[0]
    planar = [i for i, f in enumerate(shape.Faces) if kind(f) == 'Plane']
    require(planar, 'no-planar-root', 'No hay una cara plana que pueda ser la raíz.')
    if root_face is not None:
        require(1 <= root_face <= len(shape.Faces) and root_face-1 in planar,
                'invalid-root-face', 'La raíz solicitada debe ser una cara plana existente.', rootFace=root_face)
        ri = root_face-1
    else:
        # Area ties normally select one of the two equivalent skins. Face index
        # is merely a deterministic tie-break within this imported revision.
        ri = min(planar, key=lambda i: (-round(shape.Faces[i].Area, 7), i))
    opposite_root, thickness = plane_partner(shape, ri)
    require(thickness > 1e-6, 'invalid-thickness', 'El espesor detectado es degenerado.')
    groups, graph, owners, edge_faces, face_edges, skin, split_walls = topology(shape, ri, thickness, App, Part, nx, U)
    bac = U.BendAllowanceCalculator.from_single_value(k_factor, 'ansi')
    root_shapes = [shape.Faces[i] for i in groups['ROOT']['facesA']]
    align = U.SketchExtraction.move_to_origin(Part.makeCompound([e for f in root_shapes for e in f.Edges]), shape.Faces[ri])
    combined = {'ROOT': App.Matrix()}
    flat_matrices = {'ROOT': align}
    bands = []; internals = {}; band_by_group = {}
    parent_by_group = {'ROOT': None}
    for a, b in nx.bfs_edges(graph, 'ROOT', sort_neighbors=lambda it: sorted(it)):
        parent_by_group[b] = a
    panel_order = ['ROOT']
    for gid in nx.bfs_tree(graph, 'ROOT', sort_neighbors=lambda it: sorted(it)).nodes:
        group = groups[gid]
        if group['kind'] != 'bend': continue
        parent = parent_by_group[gid]
        child_candidates = [g for g in graph.neighbors(gid) if g != parent]
        require(len(child_candidates) == 1, 'ambiguous-bend-child', 'No se pudo determinar el panel hijo.')
        child = child_candidates[0]
        pe = shared_tangent_edges(shape, groups[parent]['facesA'], group['facesA'], edge_faces)
        ce = shared_tangent_edges(shape, groups[child]['facesA'], group['facesA'], edge_faces)
        representative = next(i for i in group['facesA'] if i in edge_faces[pe[0]])
        try:
            _, unbend, _ = U.compute_unbend_transform(shape.Faces[representative], shape.Edges[pe[0]], thickness, bac)
        except Exception as exc:
            raise ImportStepError('unbend-transform-failed', 'No se pudo calcular la tangencia desplegada.', face=representative+1, error=str(exc)) from exc
        combined[child] = combined[parent]*unbend
        flat_matrices[child] = align*combined[child]
        panel_order.append(child)
        pm, cm = flat_matrices[parent], flat_matrices[child]
        O = pm.multVec(shape.Edges[pe[0]].CenterOfMass)
        E = cm.multVec(shape.Edges[ce[0]].CenterOfMass)
        cylinder = shape.Faces[representative].Surface
        C = pm.multVec(cylinder.Center)
        axis0 = pm.multVec(cylinder.Center+cylinder.Axis)-C; axis0.normalize()
        delta = E-O
        transverse = delta-axis0*delta.dot(axis0); transverse.z = 0
        require(transverse.Length > 1e-7, 'zero-neutral-band-width', 'La banda desplegada tiene ancho nulo.')
        transverse.normalize(); normal = App.Vector(0, 0, 1)
        axis = transverse.cross(normal); axis.normalize(); O.z = 0
        sign = 1 if C.z > 0 else -1
        radii = [shape.Faces[i].Surface.Radius for i in group['facesA']+group['facesB']]
        inner_radius = min(radii); rn = inner_radius+k_factor*thickness
        require(rn > 1e-7, 'zero-neutral-radius', 'Radio de fibra neutra degenerado.')
        theta = sign*group['sweep']; L = rn*abs(theta)
        nz = -(k_factor if sign > 0 else 1-k_factor)*thickness
        require(abs((delta-axis0*delta.dot(axis0)).Length-L) < max(.005, L*1e-5),
                'tangent-allowance-mismatch', 'Las tangencias desplegadas no coinciden con la banda neutra.', bend=group['id'])
        mesh = new_mesh(); mesh['parameters'] = []
        data = {'id': group['id'], 'parentPanelId': parent, 'childPanelId': child,
                'origin': xyz(O), 'axis': xyz(axis), 'transverse': xyz(transverse), 'normal': [0, 0, 1],
                'signedAngle': theta, 'neutralLength': L, 'neutralZ': nz,
                'insideRadius': inner_radius, 'thickness': thickness, 'kFactor': k_factor,
                'sourceFaces': [i+1 for i in group['facesA']+group['facesB']],
                'materialIntervalsAlongAxis': [], 'band': mesh}
        # Finite material intervals are distinct from the infinite mechanical
        # axis; interrupted coaxial patches remain one joint with several spans.
        for ei in pe:
            pts = [pm.multVec(v.Point)-O for v in shape.Edges[ei].Vertexes]
            data['materialIntervalsAlongAxis'].append(sorted([p.dot(axis) for p in pts]))
        bands.append(data); band_by_group[gid] = data
        internals[gid] = {'json': data, 'O': O, 'a': axis, 'b': transverse, 'pm': pm, 'rn': rn, 'sign': sign}
    panels = []
    for gid in panel_order:
        mesh = new_mesh(); mesh.update({'id': gid, 'sourceSkinFaces': [i+1 for i in groups[gid]['facesA']+groups[gid]['facesB']]})
        panels.append(mesh)
    meshes = {p['id']: p for p in panels}
    meshes.update({g: b['band'] for g, b in band_by_group.items()})
    caches = {g: {} for g in meshes}; seam_occurrences = collections.defaultdict(list)
    face_extents = {}; depth_errors = {}; clamped_angles = 0
    nominal_skin_depth = {i: z for group in groups.values() for field, z in [('facesA', 0), ('facesB', -thickness)] for i in group[field]}

    def inverse_band(gid, point):
        nonlocal clamped_angles
        b = internals[gid]; j = b['json']; q = b['pm'].multVec(point)-b['O']
        center_z = j['insideRadius'] if b['sign'] > 0 else -(j['insideRadius']+thickness)
        rb, rz = q.dot(b['b']), q.z-center_z
        angle = math.atan2(b['sign']*rb, -b['sign']*rz)
        fraction = angle/j['signedAngle']
        require(-1e-6 <= fraction <= 1+1e-6, 'vertex-outside-bend-sector',
                'Una cara asignada a banda excede su sector cilíndrico.', bend=j['id'], fraction=fraction)
        if not 0 <= fraction <= 1: clamped_angles += 1
        z = j['neutralZ']+b['sign']*(b['rn']-math.hypot(rb, rz))
        return [q.dot(b['a']), max(0, min(1, fraction)), z]

    shape.tessellate(mesh_tolerance)
    face_pieces = [(i, owners[i], face) for i, face in enumerate(shape.Faces) if i in owners]
    volumes = {}
    split_audit = []
    for i, candidates in split_walls.items():
        original = shape.Faces[i]; pieces = []
        for gid in candidates:
            if gid not in volumes: volumes[gid] = region_volume(shape, groups[gid], thickness, Part)
            try: faces = original.common(volumes[gid]).Faces
            except Exception as exc:
                raise ImportStepError('side-wall-split-failed', 'No se pudo subdividir una pared por tangencias.', face=i+1, error=str(exc)) from exc
            pieces += [(gid, f) for f in faces if f.Area > 1e-10]
        area = sum(f.Area for _, f in pieces)
        require(abs(area-original.Area) < max(1e-6, original.Area*1e-7), 'side-wall-split-area-mismatch',
                'La subdivisión de una pared no conserva su área.', face=i+1, sourceArea=original.Area, splitArea=area)
        for gid, face in pieces:
            face.tessellate(mesh_tolerance)
            face_pieces.append((i, gid, face))
        split_audit.append({'face': i+1, 'regions': sorted({g for g, _ in pieces}), 'pieces': len(pieces),
                            'sourceAreaMm2': original.Area, 'areaErrorMm2': abs(area-original.Area)})
    face_pieces.sort(key=lambda item: (item[0], item[1]))
    triangle_count = 0
    for i, gid, face in face_pieces:
        mesh = meshes[gid]; cache = caches[gid]
        if i+1 not in mesh['faceIds']: mesh['faceIds'].append(i+1)
        vertices, triangles = face.tessellate(mesh_tolerance)
        require(vertices and triangles, 'empty-face-mesh', 'Una cara no se pudo triangular.', face=i+1)
        remap = []; depths = []
        for p in vertices:
            if groups[gid]['kind'] == 'panel':
                flat = flat_matrices[gid].multVec(p); par = None
            else:
                b = internals[gid]; par = inverse_band(gid, p)
                flat = b['O']+b['a']*par[0]+b['b']*(par[1]*b['json']['neutralLength'])+App.Vector(0, 0, par[2])
            depths.append(flat.z)
            if i in nominal_skin_depth:
                depth_errors[i+1] = max(depth_errors.get(i+1, 0), abs(flat.z-nominal_skin_depth[i]))
            key = tuple(round(x, 7) for x in p)
            idx = cache.get(key)
            if idx is None:
                idx = len(mesh['sourcePositions'])//3; cache[key] = idx
                mesh['sourcePositions'] += xyz(align.multVec(p)); mesh['flatPositions'] += xyz(flat)
                if par is not None: mesh['parameters'] += par
                seam_occurrences[key].append((gid, idx))
            remap.append(idx)
        for triangle in triangles:
            mesh['triangles'] += [remap[v] for v in triangle]; mesh['triangleFaceIds'].append(i+1)
        triangle_count += len(triangles)
        prior = face_extents.get(i+1, {'minDepthMm': math.inf, 'maxDepthMm': -math.inf})
        face_extents[i+1] = {'type': kind(face), 'minDepthMm': min(min(depths), prior['minDepthMm']),
                           'maxDepthMm': max(max(depths), prior['maxDepthMm'])}
    normal = App.Vector(0, 0, 1)

    def deform(gid, par, amount):
        b = internals[gid]; j = b['json']; x, fraction, z = par
        angle = j['signedAngle']*amount*fraction; sn, cs = terms(angle)
        arc = j['neutralLength']*fraction
        return (b['O']+b['a']*x+normal*j['neutralZ']+b['b']*(arc*sn)+normal*(arc*cs)
                +(normal*math.cos(angle)-b['b']*math.sin(angle))*(z-j['neutralZ']))

    def frames(state):
        result = {'ROOT': App.Matrix()}
        for gid, b in internals.items():
            j = b['json']; angle = j['signedAngle']*state[j['id']]
            rotation = App.Rotation(b['a'], math.degrees(angle)).toMatrix()
            anchor = b['O']+b['b']*j['neutralLength']+normal*j['neutralZ']
            target = deform(gid, [0, 1, j['neutralZ']], state[j['id']])
            translation = target-rotation.multVec(anchor)
            rotation.A14, rotation.A24, rotation.A34 = xyz(translation)
            result[j['childPanelId']] = result[j['parentPanelId']]*rotation
        return result

    def evaluate(gid, idx, state, transforms):
        mesh = meshes[gid]; off = idx*3
        if groups[gid]['kind'] == 'panel': return transforms[gid].multVec(App.Vector(*mesh['flatPositions'][off:off+3]))
        j = internals[gid]['json']
        return transforms[j['parentPanelId']].multVec(deform(gid, mesh['parameters'][off:off+3], state[j['id']]))

    states = [{b['id']: q for b in bands} for q in [0, 1e-12, .1, .5, .9, 1]]
    states += [{b['id']: 1 if i < cut else .37 if i == cut else 0 for i, b in enumerate(bands)} for cut in range(len(bands))]
    states += [{b['id']: ((i*37+23) % 100)/100 for i, b in enumerate(bands)}]
    seams = [occ for occ in seam_occurrences.values() if len({g for g, _ in occ}) > 1]
    checks = []
    for state in states:
        transforms = frames(state); gap = 0; endpoint = 0; finite = True
        for occurrences in seams:
            positions = [evaluate(g, idx, state, transforms) for g, idx in occurrences]
            gap = max(gap, max((p-positions[0]).Length for p in positions))
        for gid, mesh in meshes.items():
            for idx in range(len(mesh['sourcePositions'])//3):
                p = evaluate(gid, idx, state, transforms)
                finite &= all(math.isfinite(x) for x in p)
                if all(q == 1 for q in state.values()):
                    endpoint = max(endpoint, (p-App.Vector(*mesh['sourcePositions'][idx*3:idx*3+3])).Length)
        checks.append({'state': state, 'maxSeamGapMm': gap, 'maxFinalSourceErrorMm': endpoint, 'allFinite': finite})
    max_gap = max(c['maxSeamGapMm'] for c in checks); max_endpoint = max(c['maxFinalSourceErrorMm'] for c in checks)
    min_z = min(x['minDepthMm'] for x in face_extents.values()); max_z = max(x['maxDepthMm'] for x in face_extents.values())
    flat_excess = max(0, -thickness-min_z, max_z)
    flat_tol = .005  # explicit CAD/tessellation tolerance; never silently healed
    require(max_gap < 1e-4 and max_endpoint < 1e-4 and all(c['allFinite'] for c in checks),
            'kinematic-validation-failed', 'La reconstrucción no coincide con el STEP o abre uniones.',
            maximumSeamGapMm=max_gap, maximumFinalSourceErrorMm=max_endpoint)
    require(flat_excess <= flat_tol, 'flat-thickness-validation-failed',
            'El desplegado excede la tolerancia del espesor uniforme.', excessMm=flat_excess, toleranceMm=flat_tol)
    planar_depth_error = max((e for i, e in depth_errors.items() if kind(shape.Faces[i-1]) == 'Plane'), default=0)
    require(planar_depth_error < 1e-5, 'planar-thickness-validation-failed',
            'Las pieles planas no tienen espesor uniforme.', errorMm=planar_depth_error)
    provenance = {'repository': 'https://github.com/shaise/FreeCAD_SheetMetal',
                  'commit': 'd6a60a9911e612cf7ecb593b869f658baca22cdd',
                  'license': 'LGPL-2.1-or-later; see analysis/freecad/upstream/LICENSE',
                  'moduleSha256': hashlib.sha256((UPSTREAM/'SheetMetalNewUnfolder.py').read_bytes()).hexdigest(),
                  'apiUsed': ['build_graph_of_tangent_faces', 'compute_unbend_transform', 'SketchExtraction.move_to_origin']}
    source_hash = hashlib.sha256(step_path.read_bytes()).hexdigest()
    warnings = []
    if flat_excess > 1e-6 or max(depth_errors.values(), default=0) > 1e-5:
        warnings.append({'code': 'cad-boundary-deviation', 'message': 'Se conservaron pequeñas desviaciones de borde del STEP sin corrección artificial.',
                         'flatEnvelopeExcessMm': flat_excess, 'maximumSkinDepthErrorMm': max(depth_errors.values(), default=0),
                         'toleranceMm': flat_tol})
    verification = {'status': 'verified', 'sourceSha256': source_hash, 'rootFace': ri+1,
                    'oppositeRootFace': opposite_root+1, 'autoRoot': root_face is None,
                    'thicknessMm': thickness, 'kFactor': k_factor, 'kFactorSource': 'CLI/user supplied ANSI',
                    'panelCount': len(panels), 'bendCount': len(bands), 'faceCount': len(shape.Faces),
                    'triangles': triangle_count, 'meshToleranceMm': mesh_tolerance,
                    'maximumSeamGapMm': max_gap, 'maximumFinalSourceErrorMm': max_endpoint,
                    'flatThicknessMm': max_z-min_z, 'flatEnvelopeToleranceMm': flat_tol,
                    'maximumPlanarSkinFlatDepthErrorMm': planar_depth_error,
                    'maximumSkinFlatDepthErrorMm': max(depth_errors.values(), default=0),
                    'faceDepths': face_extents, 'skinDepthErrorsMm': depth_errors,
                    'faceOwners': [{'face': i+1, 'regions': sorted({g for fi, g, _ in face_pieces if fi == i})} for i in range(len(shape.Faces))],
                    'subdividedSideWalls': split_audit,
                    'sharedSeamVertexCount': len(seams), 'checks': checks,
                    'clampedEndpointParameters': clamped_angles, 'warnings': warnings, 'upstream': provenance,
                    'limitations': ['Kinematic geometry only; no collision or manufacturing-sequence certification.',
                                    'One solid, planar/cylindrical skin, tree topology after grouping coaxial interrupted patches.',
                                    'Angular cylinder subdivisions require explicit preprocessing. Cross-region walls are geometrically split and area-checked.',
                                    'Root thickness detection requires a congruent opposite planar skin. Other paired planar trims require unique >50% overlap.',
                                    'Transient face numbers identify this imported revision, not persistent CAD feature IDs.']}
    model = {'schemaVersion': 1, 'units': 'mm', 'rootPanelId': 'ROOT', 'thicknessMm': thickness, 'kFactor': k_factor,
             'metadata': {'partName': step_path.stem, 'sourceSha256': {step_path.name: source_hash},
                          'sampleSpecific': False, 'geometryClass': 'developable-planar-cylindrical-tree',
                          'rootFace': ri+1,
                          'rootSelection': {'mode': 'largest-planar-face' if root_face is None else 'explicit-face',
                                            'face': ri+1, 'oppositeFace': opposite_root+1,
                                            'note': 'Geometric reference only; not a press-brake fixture or sequence decision.'},
                          'tolerances': {'meshMm': mesh_tolerance, 'endpointMm': 1e-4, 'seamMm': 1e-4,
                                         'flatEnvelopeMm': flat_tol, 'planarSkinDepthMm': 1e-5},
                          'unsupported': ['multiple-solids', 'non-planar/cylindrical-skins', 'closed-kinematic-cycles',
                                          'ambiguous-opposite-skins', 'angular-cylinder-subdivisions', 'sweeps-at-or-over-180-degrees'],
                          'sourceFrame': 'STEP transformed by rootPlaneToFlatXY16',
                          'rootPlaneToFlatXY16': list(align.A), 'matrixConvention': 'row-major, column vectors',
                          'upstream': provenance, 'warnings': warnings,
                          'validation': {k: verification[k] for k in ['maximumSeamGapMm', 'maximumFinalSourceErrorMm', 'flatThicknessMm', 'triangles', 'maximumSkinFlatDepthErrorMm', 'maximumPlanarSkinFlatDepthErrorMm', 'flatEnvelopeToleranceMm']},
                          'sequenceNote': 'Automatically rooted geometry order; not a certified manufacturing sequence.'},
             'panels': panels, 'bends': bands,
             'seams': [[{'kind': groups[g]['kind'], 'id': g if groups[g]['kind'] == 'panel' else band_by_group[g]['id'], 'vertexIndex': idx} for g, idx in occ] for occ in seams]}
    return model, verification


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--step', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--root-face', type=int)
    parser.add_argument('--k-factor', type=float, default=.44)
    parser.add_argument('--mesh-tolerance', type=float, default=.08)
    args = parser.parse_args(argv)
    try:
        model, verification = import_model(args.step, args.root_face, args.k_factor, args.mesh_tolerance)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(model, separators=(',', ':'), allow_nan=False), encoding='utf-8')
        audit = args.output.with_suffix('.verification.json')
        audit.write_text(json.dumps(verification, indent=2, allow_nan=False), encoding='utf-8')
        print(json.dumps({'status': 'ok', 'output': str(args.output.resolve()), 'verification': str(audit.resolve()),
                          'panels': len(model['panels']), 'bends': len(model['bends']), 'thicknessMm': model['thicknessMm'],
                          'warnings': verification['warnings']}, ensure_ascii=False))
        return 0
    except ImportStepError as exc:
        print(json.dumps(exc.as_dict(), ensure_ascii=False)); return 2
    except Exception as exc:
        print(json.dumps({'status': 'error', 'code': 'cad-import-failed', 'message': str(exc)}, ensure_ascii=False)); return 3


if __name__ == '__main__':
    sys.exit(main())
