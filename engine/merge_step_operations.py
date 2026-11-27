"""Combine explicitly identified coaxial CAD relations without rebuilding material.

Only joints with identical directed finite-band frames and rigid parents can
share an operation. Gaps remain gaps; source meshes and final references survive.
"""
import copy
import math
from collections import defaultdict


def merge_step_operations(model):
    out = copy.deepcopy(model)
    groups = defaultdict(list)
    for bend in out['bends']:
        groups[str(bend.get('sourceBendId', bend['id']))].append(bend)
    incoming = {b['childPanelId']: b for b in out['bends']}

    def depth(b):
        seen = set()
        parent = b['parentPanelId']
        while parent != out['rootPanelId']:
            if parent in seen or parent not in incoming:
                raise ValueError('CAD operation graph is cyclic or disconnected.')
            seen.add(parent)
            parent = incoming[parent]['parentPanelId']
        return len(seen)

    manifest = {'panels': [], 'bands': [], 'operations': []}
    panel_map = {p['id']: (p['id'], 0) for p in out['panels']}
    band_map = {b['id']: (b['id'], 0) for b in out['bends']}

    def combine(meshes):
        merged = copy.deepcopy(meshes[0])
        offsets, count = [], 0
        for mesh in meshes:
            offsets.append(count)
            count += len(mesh['flatPositions']) // 3
        for key in ('flatPositions', 'sourcePositions', 'parameters', 'triangleFaceIds'):
            if all(key in mesh for mesh in meshes):
                merged[key] = [v for mesh in meshes for v in mesh[key]]
            else:
                merged.pop(key, None)
        merged['triangles'] = [v + offset for mesh, offset in zip(meshes, offsets) for v in mesh['triangles']]
        for key in ('faceIds', 'sourceSkinFaces'):
            if any(key in m for m in meshes):
                merged[key] = sorted({v for mesh in meshes for v in mesh.get(key, [])})
        return merged, offsets

    ordered = sorted(groups.items(), key=lambda pair: min(depth(b) for b in pair[1]))
    for source, members in ordered:
        if len(members) == 1:
            continue
        lead = members[0]
        for b in members[1:]:
            if b['parentPanelId'] != lead['parentPanelId']:
                raise ValueError('Coaxial operations must share the same rigid parent after upstream grouping.')
            for field in ('signedAngle', 'neutralLength', 'neutralZ', 'insideRadius', 'thickness', 'kFactor'):
                if abs(b[field] - lead[field]) > 1e-7:
                    raise ValueError('Incompatible CAD bend parameter: ' + field)
            if any(math.dist(b[field], lead[field]) > 1e-7 for field in ('axis', 'transverse', 'normal')):
                raise ValueError('Directed CAD joint frames differ; normalize before grouping.')
            delta = [x-y for x,y in zip(b['origin'], lead['origin'])]
            along = sum(x*y for x,y in zip(delta, lead['axis']))
            if math.dist(delta, [along*x for x in lead['axis']]) > 1e-7:
                raise ValueError('CAD operation bands are not coaxial.')
        children = [b['childPanelId'] for b in members]
        if len(set(children)) != len(children):
            raise ValueError('Repeated child panel in CAD operation.')
        panels = [next(p for p in out['panels'] if p['id'] == child) for child in children]
        panel, offsets = combine(panels)
        panel['id'] = children[0]
        for p, offset in zip(panels, offsets):
            panel_map[p['id']] = (panel['id'], offset)
        out['panels'] = [p for p in out['panels'] if p['id'] not in children] + [panel]
        bands, offsets = combine([b['band'] for b in members])
        params = []
        for i in range(0, len(bands['flatPositions']), 3):
            delta = [bands['flatPositions'][i+k]-lead['origin'][k] for k in range(3)]
            params += [sum(x*y for x,y in zip(delta, lead['axis'])),
                       sum(x*y for x,y in zip(delta, lead['transverse']))/lead['neutralLength'],
                       sum(x*y for x,y in zip(delta, lead['normal']))]
        bands['parameters'] = params
        intervals = []
        for b, offset in zip(members, offsets):
            band_map[b['id']] = (lead['id'], offset)
            along = sum((x-y)*a for x,y,a in zip(b['origin'], lead['origin'], lead['axis']))
            intervals += [[v+along for v in span] for span in b.get('materialIntervalsAlongAxis', [])]
        record = {'sourceBendId': source, 'bendIds': [b['id'] for b in members],
                  'childPanelIds': children, 'mergedBendId': lead['id']}
        lead['band'] = bands
        lead['sourceFaces'] = sorted({v for b in members for v in b.get('sourceFaces', [])})
        lead['materialIntervalsAlongAxis'] = sorted(intervals)
        lead['mergedSplitHingeSpans'] = len(members)
        lead['sourceRelations'] = record
        out['bends'] = [b for b in out['bends'] if b not in members[1:]]
        for b in out['bends']:
            if b['parentPanelId'] in children:
                b['parentPanelId'] = children[0]
        manifest['operations'].append(record)
    for seam in out.get('seams', []):
        for ref in seam:
            target, offset = (panel_map if ref['kind'] == 'panel' else band_map)[ref['id']]
            ref['id'] = target
            ref['vertexIndex'] += offset
    for key, mapping in [('panels', panel_map), ('bands', band_map)]:
        manifest[key] = [{'sourceId': source, 'targetId': target, 'vertexOffset': offset}
                         for source, (target, offset) in mapping.items()]
    out.setdefault('metadata', {})['compoundOperationMerge'] = manifest
    return out
