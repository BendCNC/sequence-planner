"""Build this audited STEP sample's finite-radius animation model.

Run with the Python bundled with FreeCAD 1.1. Source BRep is never modified.
The face mapping is intentionally pinned to the audited input fingerprints.
"""
import collections
import hashlib
import json
import math
from pathlib import Path

import FreeCAD as App
import Part

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'viewer'
OUT.mkdir(exist_ok=True)
mapping = json.loads((ROOT/'analysis/step/mapping.json').read_text())
adapter = json.loads((ROOT/'analysis/freecad/adapter_graph.json').read_text())
inspection = json.loads((ROOT/'analysis/step/inspection.json').read_text())
input_dir = ROOT/'analysis/inputs'
for filename, expected in mapping['input_sha256'].items():
    actual = hashlib.sha256((input_dir/filename).read_bytes()).hexdigest()
    if actual != expected:
        raise ValueError(f'{filename}: input changed; regenerate and audit the face mapping before building.')
export = json.loads((input_dir/'complex.bcbend').read_text(encoding='utf-8'))
shape = Part.Shape(); shape.read(str(input_dir/'part.step'))
assert shape.isValid() and len(shape.Solids)==1

def vector(a): return App.Vector(*a)
def xyz(v): return [v.x,v.y,v.z]
def matrix(a): return App.Matrix(*a)
def transform(m,p): return m.multVec(p)
def shared_edge(a,b):
    found=[e for e in a.Edges if any(e.isSame(z) for z in b.Edges)]
    if len(found)!=1: raise ValueError('Expected a unique tangent edge')
    return found[0]
def inverse(m): return m.inverse()
def flatten(a):return [x for v in a for x in v]

nodes={n['faceNumber']:n for n in adapter['nodes']}
panel_map={p['panel_id']:p for p in mapping['panels']}
align=matrix(adapter['rootPlaneToFlatXY16'])
normal=App.Vector(0,0,1)
flat_matrices={p['panel_id']:matrix(nodes[p['skin_A']['face_index_1based']]['foldedToFlatXY16']) for p in mapping['panels']}
known={}
for p in mapping['panels']:
    for skin in ['skin_A','skin_B']:
        known[p[skin]['face_index_1based']]=('panel',p['panel_id'])
for b in mapping['bends']:
    for field in ['inner_cylinder_face','outer_cylinder_face']:
        known[b[field]]=('bend',b['source_bend_id'])
owners=dict(known)
for f in inspection['faces']:
    if f['id'] in owners:continue
    choices={known[n] for n in f['adjacent_faces'] if n in known}
    if len(choices)!=1:raise ValueError(f'Face {f["id"]}: ambiguous owner {choices}')
    owners[f['id']]=choices.pop()

bends=[]
internal_bends={}
for b in mapping['bends']:
    pid,cid=b['parent_panel_id'],b['child_panel_id']
    parent=shape.Faces[panel_map[pid]['skin_A']['face_index_1based']-1]
    child=shape.Faces[panel_map[cid]['skin_A']['face_index_1based']-1]
    cylinder=shape.Faces[b['skin_A_cylinder_face']-1]
    pm,cm=flat_matrices[pid],flat_matrices[cid]
    start=pm.multVec(shared_edge(parent,cylinder).CenterOfMass)
    end=cm.multVec(shared_edge(child,cylinder).CenterOfMass)
    center=pm.multVec(cylinder.Surface.Center)
    oldaxis=pm.multVec(cylinder.Surface.Center+cylinder.Surface.Axis)-center
    oldaxis.normalize()
    delta=end-start
    transverse=delta-oldaxis*delta.dot(oldaxis)
    transverse.z=0; transverse.normalize()
    axis=transverse.cross(normal);axis.normalize()
    start.z=0
    sign=1 if center.z>0 else -1
    theta=sign*math.radians(b['sweep_from_flat_deg'])
    t=b['thickness_mm']; k=b['k_factor']; rn=b['neutral_radius_mm']
    L=rn*abs(theta)
    nz=-(k if sign>0 else 1-k)*t
    info={'id':b['source_bend_id'],'parentPanelId':pid,'childPanelId':cid,
          'origin':xyz(start),'axis':xyz(axis),'transverse':xyz(transverse),'normal':[0,0,1],
          'signedAngle':theta,'neutralLength':L,'neutralZ':nz,
          'insideRadius':b['inside_radius_mm'],'thickness':t,'kFactor':k,
          'sourceFaces':[b['inner_cylinder_face'],b['outer_cylinder_face']],
          'band':{'parameters':[],'triangles':[],'sourcePositions':[],'flatPositions':[],
                  'faceIds':[],'triangleFaceIds':[]}}
    bends.append(info)
    internal_bends[b['source_bend_id']]={'json':info,'O':start,'a':axis,'b':transverse,
                                      'n':normal,'parentFlat':pm,'rn':rn,'sign':sign}

panels=[{'id':p['panel_id'],'flatPositions':[],'sourcePositions':[],'triangles':[],
         'faceIds':[],'triangleFaceIds':[],
         'sourceSkinFaces':[p['skin_A']['face_index_1based'],p['skin_B']['face_index_1based']]}
        for p in mapping['panels']]
panel_outputs={p['id']:p for p in panels}
owner_meshes={('panel',p['id']):p for p in panels}
owner_meshes.update({('bend',b['id']):b['band'] for b in bends})
vertex_cache={owner:{} for owner in owner_meshes}
seam_occurrences=collections.defaultdict(list)
clamps=[]
radial_reliefs=[]
flat_face_extents={}
expected_skin_depth={}
for p in mapping['panels']:
    expected_skin_depth[p['skin_A']['face_index_1based']]=0
    expected_skin_depth[p['skin_B']['face_index_1based']]=-2.5
for b in mapping['bends']:
    expected_skin_depth[b['skin_A_cylinder_face']]=0
    expected_skin_depth[b['skin_B_cylinder_face']]=-2.5
maximum_skin_depth_error=0
maximum_planar_skin_depth_error=0
maximum_other_skin_depth_error=0

def band_inverse(b,p):
    j=b['json']; O,a,bt,n=b['O'],b['a'],b['b'],b['n']
    q=b['parentFlat'].multVec(p)-O
    x=q.dot(a)
    # Parent-side cylinder center lies on the normal through the tangent line.
    center_z=(j['insideRadius'] if b['sign']>0 else -(j['insideRadius']+j['thickness']))
    rb=q.dot(bt); rz=q.z-center_z
    angle=math.atan2(b['sign']*rb,-b['sign']*rz)
    f=angle/j['signedAngle']
    if f < -1e-7 or f > 1+1e-7:raise ValueError(f'Bend {j["id"]} angular parameter outside band: {f}')
    if f<0 or f>1:clamps.append(f)
    f=max(0,min(1,f))
    z=j['neutralZ']+b['sign']*(b['rn']-math.hypot(rb,rz))
    return [x,f,z]

# Generate the whole shape first to seed shared-edge triangulation caches.
shape.tessellate(0.08)
triangle_count=0
for face_number,f in enumerate(shape.Faces,1):
    owner=owners[face_number];mesh=owner_meshes[owner];cache=vertex_cache[owner]
    mesh['faceIds'].append(face_number)
    vertices,triangles=f.tessellate(0.08)
    remap=[]
    face_depths=[]
    for p in vertices:
        if owner[0]=='panel':
            flat=flat_matrices[owner[1]].multVec(p)
        else:
            b=internal_bends[owner[1]]
            par=band_inverse(b,p)
            flat=b['O']+b['a']*par[0]+b['b']*(par[1]*b['json']['neutralLength'])+normal*par[2]
        face_depths.append(flat.z)
        if face_number in expected_skin_depth:
            skin_error=abs(flat.z-expected_skin_depth[face_number])
            maximum_skin_depth_error=max(maximum_skin_depth_error,skin_error)
            if type(f.Surface).__name__=='Plane':maximum_planar_skin_depth_error=max(maximum_planar_skin_depth_error,skin_error)
            if face_number not in [2,45]:maximum_other_skin_depth_error=max(maximum_other_skin_depth_error,skin_error)
        key=tuple(round(x,7) for x in p)
        idx=cache.get(key)
        if idx is None:
            idx=len(mesh['sourcePositions'])//3;cache[key]=idx
            source=align.multVec(p)
            mesh['sourcePositions']+=xyz(source)
            if owner[0]=='panel':
                mesh['flatPositions']+=xyz(flat)
            else:
                mesh['parameters']+=par
                excess=max(0,-2.5-par[2],par[2])
                if excess>1e-8:
                    radial_reliefs.append({'face':face_number,'vertex':idx,'bend':owner[1],
                                          'flatDepthMm':par[2],'outsideNominalThicknessMm':excess})
                mesh['flatPositions']+=xyz(flat)
            seam_occurrences[key].append((owner,idx))
        remap.append(idx)
    flat_face_extents[face_number]={'faceType':type(f.Surface).__name__,
                                   'minDepthMm':min(face_depths),'maxDepthMm':max(face_depths),
                                   'outsideNominalCount':sum(z < -2.5-1e-8 or z>1e-8 for z in face_depths)}
    if face_number in expected_skin_depth:
        flat_face_extents[face_number]['maxNominalDepthErrorMm']=max(abs(z-expected_skin_depth[face_number]) for z in face_depths)
        flat_face_extents[face_number]['nominalDepthErrorCountAbove1e8']=sum(abs(z-expected_skin_depth[face_number])>1e-8 for z in face_depths)
    for tri in triangles:
        mesh['triangles'] += [remap[i] for i in tri]
        mesh['triangleFaceIds'].append(face_number)
    triangle_count+=len(triangles)

def stable_terms(phi):
    if abs(phi)<1e-4:
        p2=phi*phi
        return 1-p2/6+p2*p2/120, phi*(.5-p2/24+p2*p2/720)
    return math.sin(phi)/phi,2*math.sin(phi/2)**2/phi

def band_local(b,parameters,q):
    j=b['json'];x,f,z=parameters;phi=j['signedAngle']*q
    sn,cs=stable_terms(phi*f)
    L=j['neutralLength']*f
    ang=phi*f
    return (b['O']+b['a']*x+normal*j['neutralZ']+b['b']*(L*sn)+normal*(L*cs)
            +(normal*math.cos(ang)-b['b']*math.sin(ang))*(z-j['neutralZ']))

def transforms(state):
    result={'ROOT':App.Matrix()}
    for bj in bends:
        b=internal_bends[bj['id']];phi=bj['signedAngle']*state.get(bj['id'],0)
        rot=App.Rotation(b['a'],math.degrees(phi)).toMatrix()
        anchor=b['O']+b['b']*bj['neutralLength']+normal*bj['neutralZ']
        sn,cs=stable_terms(phi)
        target=b['O']+normal*bj['neutralZ']+b['b']*(bj['neutralLength']*sn)+normal*(bj['neutralLength']*cs)
        tr=target-rot.multVec(anchor)
        rot.A14,rot.A24,rot.A34=tr.x,tr.y,tr.z
        result[bj['childPanelId']]=result[bj['parentPanelId']]*rot
    return result

def evaluate_vertex(owner,idx,state,frames):
    mesh=owner_meshes[owner];off=idx*3
    if owner[0]=='panel':return frames[owner[1]].multVec(vector(mesh['flatPositions'][off:off+3]))
    b=internal_bends[owner[1]];j=b['json']
    return frames[j['parentPanelId']].multVec(band_local(b,mesh['parameters'][off:off+3],state.get(j['id'],0)))

def evaluate_source_point(owner,p,state,frames):
    if owner[0]=='panel':return frames[owner[1]].multVec(flat_matrices[owner[1]].multVec(p))
    b=internal_bends[owner[1]];j=b['json']
    return frames[j['parentPanelId']].multVec(band_local(b,band_inverse(b,p),state.get(j['id'],0)))

states=[{b['id']:q for b in bends} for q in [0,1e-12,.1,.5,.9,1]]
states += [{b['id']:(1 if i<cut else .37 if i==cut else 0) for i,b in enumerate(bends)} for cut in range(len(bends))]
states += [{'1':.71,'5':.19,'6':.83,'2':.42,'3':.24,'4':.93}]
seams=[v for v in seam_occurrences.values() if len({x[0] for x in v})>1]
edge_owner_sets=collections.defaultdict(set)
for f in inspection['faces']:
    for edge in f['edges']:edge_owner_sets[edge].add(owners[f['id']])
analytic_seams=[]
for edge_number,own in sorted(edge_owner_sets.items()):
    if len(own)<2:continue
    edge=shape.Edges[edge_number-1]
    u0,u1=edge.ParameterRange
    analytic_seams.append((edge_number,sorted(own),[edge.valueAt(u0+(u1-u0)*i/10) for i in range(11)]))
checks=[]
for state in states:
    frames=transforms(state);max_seam=0;max_analytic_seam=0;max_endpoint=0;zvalues=[];finite=True
    for occurrences in seams:
        positions=[evaluate_vertex(owner,idx,state,frames) for owner,idx in occurrences]
        max_seam=max(max_seam,max((p-positions[0]).Length for p in positions))
    for edge_number,own,samples in analytic_seams:
        for sample in samples:
            positions=[evaluate_source_point(owner,sample,state,frames) for owner in own]
            max_analytic_seam=max(max_analytic_seam,max((p-positions[0]).Length for p in positions))
    for owner,mesh in owner_meshes.items():
        for idx in range(len(mesh['sourcePositions'])//3):
            p=evaluate_vertex(owner,idx,state,frames)
            finite=finite and all(math.isfinite(v) for v in p)
            zvalues.append(p.z)
            if all(v==1 for v in state.values()):
                target=vector(mesh['sourcePositions'][idx*3:idx*3+3])
                max_endpoint=max(max_endpoint,(p-target).Length)
    checks.append({'state':state,'maxSeamGapMm':max_seam,'maxAnalyticSeamGapMm':max_analytic_seam,'maxFinalSourceErrorMm':max_endpoint,
                   'allFinite':finite,'zRangeMm':[min(zvalues),max(zvalues)]})
final_check=next(c for c in checks if all(v==1 for v in c['state'].values()))
flat_check=checks[0]
verification={'inputSha256':mapping['input_sha256'],'faceCount':len(shape.Faces),
              'faceOwners':[{'face':n,'kind':owner[0],'id':owner[1]} for n,owner in sorted(owners.items())],
              'triangles':triangle_count,'outputTriangles':sum(len(m['triangles'])//3 for m in owner_meshes.values()),
              'ownerVertexCounts':{kind+':'+id:len(m['sourcePositions'])//3 for (kind,id),m in owner_meshes.items()},
              'sharedSeamVertexCount':len(seams),'parameterClampsWithin1e7':len(clamps),
              'analyticSeamEdges':[x[0] for x in analytic_seams],
              'analyticSeamSamplesPerState':sum(len(x[2]) for x in analytic_seams),
              'maximumAnalyticSeamGapMm':max(c['maxAnalyticSeamGapMm'] for c in checks),
              'flatEnvelopeToleranceMm':.005,'maximumSkinFlatDepthErrorMm':maximum_skin_depth_error,
              'maximumPlanarSkinFlatDepthErrorMm':maximum_planar_skin_depth_error,
              'maximumSkinFlatDepthErrorExceptB2Mm':maximum_other_skin_depth_error,
              'specificToleranceFaces':[2,3,8,14,19,45],
              'flatFaceDepths':flat_face_extents,
              'reliefWallDeviation':{'method':'Preserve source STEP triangulation without correction. Four B-spline side walls and the shared trim vertices of the two B2 cylindrical skins deviate microscopically from ideal coaxial cylinder/plane bands. Specific 0.005 mm depth tolerance applies only to faces 2,3,8,14,19,45. The 14 planar skins and the other 10 cylinder skins independently require 0.000001 mm nominal flat-depth error.',
                                     'count':len(radial_reliefs),'maximumMm':max([r['outsideNominalThicknessMm'] for r in radial_reliefs] or [0]),
                                     'maximumB2SkinDepthErrorMm':max(flat_face_extents[f]['maxNominalDepthErrorMm'] for f in [2,45]),
                                     'vertices':radial_reliefs},
              'checks':checks,'maximumSeamGapMm':max(c['maxSeamGapMm'] for c in checks),
              'maximumFinalSourceErrorMm':final_check['maxFinalSourceErrorMm'],
              'flatThicknessMm':flat_check['zRangeMm'][1]-flat_check['zRangeMm'][0],
              'meshToleranceMm':.08,'method':'Original STEP triangulation; panel rigid frames; cylindrical material coordinates and conserved neutral arc length; side walls assigned by unique adjacent skin owner.',
              'limitation':'Kinematic animation, no contact/material/strain calculation. Final mesh sampled from STEP. Audited input hashes required.'}
assert verification['maximumFinalSourceErrorMm']<1e-4,verification
assert verification['maximumSeamGapMm']<1e-4,verification
assert verification['maximumAnalyticSeamGapMm']<1e-4,verification
assert maximum_planar_skin_depth_error<1e-6,verification
assert maximum_other_skin_depth_error<1e-6,verification
assert maximum_skin_depth_error<verification['flatEnvelopeToleranceMm'],verification
assert all(flat_face_extents[r['face']]['faceType']=='BSplineSurface' for r in radial_reliefs),verification
assert verification['reliefWallDeviation']['maximumMm']<verification['flatEnvelopeToleranceMm'],verification
assert all(c['allFinite'] for c in checks)
assert triangle_count==verification['outputTriangles']
model={'schemaVersion':1,'units':'mm','rootPanelId':'ROOT','thicknessMm':2.5,'kFactor':.44,
       'metadata':{'partName':export['part']['name'],'sourceSha256':mapping['input_sha256'],
                   'sampleSpecific':True,'matrixConvention':'row-major, column vectors',
                   'sourceFrame':'STEP transformed by rootPlaneToFlatXY16','rootPlaneToFlatXY16':adapter['rootPlaneToFlatXY16'],
                   'method':verification['method'],'validation':{k:verification[k] for k in ['maximumSeamGapMm','maximumFinalSourceErrorMm','flatThicknessMm','triangles','maximumSkinFlatDepthErrorMm','maximumPlanarSkinFlatDepthErrorMm','flatEnvelopeToleranceMm']},
                   'reliefWallDeviation':{k:v for k,v in verification['reliefWallDeviation'].items() if k!='vertices'},
                   'sequenceNote':'Viewer order is a kinematic presentation order, not an approved manufacturing sequence.'},
       'panels':panels,'bends':bends,
       'seams':[[{'kind':o[0],'id':o[1],'vertexIndex':i} for o,i in occ] for occ in seams]}
(OUT/'model.json').write_text(json.dumps(model,separators=(',',':')),encoding='utf-8')
(OUT/'verification.json').write_text(json.dumps(verification,indent=2),encoding='utf-8')
print(json.dumps({'modelBytes':(OUT/'model.json').stat().st_size,
                  'triangles':triangle_count,'seams':len(seams),
                  'maxEndpointErrorMm':verification['maximumFinalSourceErrorMm'],
                  'maxSeamGapMm':verification['maximumSeamGapMm'],'flatThicknessMm':verification['flatThicknessMm']},indent=2))
