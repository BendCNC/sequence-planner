"""Reconstruct a conservative finger envelope from frozen orthographic DXFs.

Run with FreeCAD's bundled Python. This is a visual-hull envelope, not a claim
that two projections uniquely determine the manufactured three-dimensional part.
"""
from __future__ import annotations

from collections import Counter
import hashlib
import json
import math
from pathlib import Path

import FreeCAD as App
import Part
from shapely.geometry import LineString, Polygon, box
from shapely.ops import polygonize_full, unary_union
from shapely.geometry.polygon import orient

from build_tooling import ASSETS, ROOT, UNITS_TO_MM, bulge_segment, sample_arc

OUT=ROOT/'analysis/front-sequence/fingers'
ARC_TOL=.015
JOIN_DIGITS=9
ASSEMBLY='titan25t-assembly'
SPEC='titan25t'


def sha(data):return hashlib.sha256(data).hexdigest()


def read_dxf(name):
    path=ASSETS/name;data=path.read_bytes()
    manifest=json.loads((ASSETS/'manifest.json').read_text(encoding='utf-8'))
    entry=next(item for item in manifest['files'] if item['relativePath'].replace('\\','/')==name)
    if sha(data)!=entry['sha256']:raise ValueError('Frozen finger DXF hash changed')
    lines=data.decode('utf-8-sig').splitlines()
    pairs=[(int(lines[i].strip()),lines[i+1].strip()) for i in range(0,len(lines),2)]
    units=next(int(pairs[i+1][1]) for i,p in enumerate(pairs[:-1]) if p==(9,'$INSUNITS'))
    scale=UNITS_TO_MM[units];section='';i=0;entities=[]
    while i<len(pairs):
        code,value=pairs[i]
        if code==0 and value=='SECTION':section=pairs[i+1][1];i+=2;continue
        if code==0 and value=='ENDSEC':section=''
        if section=='ENTITIES' and code==0:
            j=i+1;group=[]
            while j<len(pairs) and pairs[j][0]!=0:group.append(pairs[j]);j+=1
            values=dict(group);kind=value;points=[];circle=None
            if kind=='LINE':points=[[float(values[10])*scale,float(values[20])*scale],[float(values[11])*scale,float(values[21])*scale]]
            elif kind=='ARC':
                center=[float(values[10])*scale,float(values[20])*scale];radius=float(values[40])*scale
                start=math.radians(float(values[50]));sweep=math.radians((float(values[51])-float(values[50]))%360)
                points=sample_arc(center,radius,start,sweep,ARC_TOL)
            elif kind=='CIRCLE':
                circle={'center':[float(values[10])*scale,float(values[20])*scale],'radiusMm':float(values[40])*scale}
            elif kind=='LWPOLYLINE':
                verts=[]
                for c,v in group:
                    if c==10:verts.append({'x':float(v)*scale,'y':0,'bulge':0})
                    elif c==20:verts[-1]['y']=float(v)*scale
                    elif c==42:verts[-1]['bulge']=float(v)
                closed=bool(int(values.get(70,'0'))&1)
                for k in range(len(verts) if closed else len(verts)-1):
                    a,b=verts[k],verts[(k+1)%len(verts)]
                    arc,_=bulge_segment([a['x'],a['y']],[b['x'],b['y']],a['bulge'])
                    points.extend(arc[:-1])
                points.append(points[0] if closed else [verts[-1]['x'],verts[-1]['y']])
            else:raise ValueError(f'Unsupported finger DXF entity: {kind}')
            entities.append({'kind':kind,'handle':values.get(5),'points':[[round(v,JOIN_DIGITS) for v in p] for p in points],
                             'circle':circle,'sourceLine':2*i+1})
            i=j;continue
        i+=1
    # Node intersections, then polygonize: the supplied side view has one
    # internal 1.27 mm overrun at the step, and top view has degenerate
    # two-vertex polylines. These remain evidence, not invented closure edges.
    linework=unary_union([LineString(e['points']) for e in entities if len(e['points'])>=2])
    polygons,cuts,dangles,invalid=polygonize_full(linework)
    silhouette=unary_union(list(polygons.geoms))
    if not isinstance(silhouette,Polygon) or not silhouette.is_valid:raise ValueError(f'No single valid silhouette: {name}')
    if len(silhouette.interiors):raise ValueError('Unexpected unresolved holes in silhouette')
    return {'polygon':orient(silhouette,sign=1),'entities':entities,
            'source':{'path':path.relative_to(ROOT).as_posix(),'sha256':sha(data),'insunits':units,'scaleToMm':scale,
                      'entityCount':len(entities),'ignoredCircularVoids':[e for e in entities if e['circle']],
                      'unusedLinework':{'cuts':[list(g.coords) for g in cuts.geoms],
                                        'dangles':[list(g.coords) for g in dangles.geoms],
                                        'invalid':[list(g.coords) for g in invalid.geoms]}}}


def remap(poly,func):return orient(Polygon([func(x,y) for x,y in poly.exterior.coords[:-1]]),sign=1)


def extrusion(poly,plane,start,length):
    if plane=='XZ':points=[App.Vector(x,start,z) for x,z in poly.exterior.coords];direction=App.Vector(0,length,0)
    else:points=[App.Vector(x,y,start) for x,y in poly.exterior.coords];direction=App.Vector(0,0,length)
    return Part.Face(Part.makePolygon(points)).extrude(direction)


def visual_hull(side,top):
    minx,miny,maxx,maxy=top.bounds;_,minz,_,maxz=side.bounds
    result=extrusion(side,'XZ',miny,maxy-miny).common(extrusion(top,'XY',minz,maxz-minz)).removeSplitter()
    if not result.isValid() or len(result.Solids)!=1 or result.Volume<=0:raise ValueError('Invalid finger volume')
    return result


def mesh(shape):
    points,faces=shape.tessellate(.025);positions=[];lookup={};remap_ids=[]
    for p in points:
        key=tuple(round(v,8) for v in p)
        if key not in lookup:lookup[key]=len(positions)//3;positions.extend([float(v) for v in p])
        remap_ids.append(lookup[key])
    triangles=[]
    for tri in faces:
        mapped=[remap_ids[i] for i in tri]
        if len(set(mapped))==3:triangles.extend(mapped)
    volume=0
    for i in range(0,len(triangles),3):
        a,b,c=[positions[j*3:j*3+3] for j in triangles[i:i+3]]
        volume+=(a[0]*(b[1]*c[2]-b[2]*c[1])+a[1]*(b[2]*c[0]-b[0]*c[2])+a[2]*(b[0]*c[1]-b[1]*c[0]))/6
    if volume<0:
        for i in range(0,len(triangles),3):triangles[i+1],triangles[i+2]=triangles[i+2],triangles[i+1]
        volume=-volume
    edges=Counter();orientation=Counter()
    for i in range(0,len(triangles),3):
        tri=triangles[i:i+3]
        for a,b in zip(tri,tri[1:]+tri[:1]):
            key=tuple(sorted((a,b)));edges[key]+=1;orientation[key]+=1 if a<b else -1
    if any(n!=2 for n in edges.values()) or any(orientation.values()):raise ValueError('Finger mesh is not closed and consistently oriented')
    if abs(volume-shape.Volume)>1e-6:raise ValueError('Finger mesh volume disagrees with solid')
    minimum=[min(positions[j::3]) for j in range(3)];maximum=[max(positions[j::3]) for j in range(3)]
    return {'positions':positions,'triangles':triangles,'bounds':{'min':minimum,'max':maximum,'size':[maximum[i]-minimum[i] for i in range(3)]},
            'volumeMm3':volume,'validation':{'closed':True,'consistentWinding':True,'vertices':len(positions)//3,
            'triangles':len(triangles)//3,'solidVolumeMm3':shape.Volume,'volumeErrorMm3':abs(volume-shape.Volume)}}


def span_at_x(poly,x):
    cut=poly.intersection(LineString([(x,-1000),(x,1000)]))
    geometries=list(cut.geoms) if hasattr(cut,'geoms') else [cut]
    points=[p for g in geometries if hasattr(g,'coords') for p in g.coords]
    return min(p[1] for p in points),max(p[1] for p in points)


def build():
    side=read_dxf('Finger Side.dxf');top=read_dxf('Finger Top.dxf')
    length=side['polygon'].bounds[2];station_center=(top['polygon'].bounds[1]+top['polygon'].bounds[3])/2
    side_local=remap(side['polygon'],lambda x,z:(length-x,z))
    top_local=remap(top['polygon'],lambda x,y:(length-x,y-station_center))
    nominal=visual_hull(side_local,top_local)
    # Sampling uses chords, which lie inside convex arcs. An outward buffer
    # at the maximum chord error bounds the true supplied curves. Unknown
    # interior void depths stay filled, yielding a conservative visual hull.
    padded_side=side_local.buffer(ARC_TOL+1e-8,join_style='mitre')
    padded_top=top_local.buffer(ARC_TOL+1e-8,join_style='mitre')
    collision=visual_hull(padded_side,padded_top)
    main=next(e for e in top['entities'] if e['handle']=='100')
    front=next(e for e in top['entities'] if e['handle']=='109')
    main_poly=remap(Polygon(main['points']),lambda x,y:(length-x,y-station_center))
    front_poly=remap(Polygon(front['points']),lambda x,y:(length-x,y-station_center))
    lower_y=span_at_x(front_poly,0);upper_y=span_at_x(main_poly,25.4)
    terrace=front_poly.intersection(box(-1,-100,25.4,100))
    features={'lower':{'planeX':0,'minY':lower_y[0],'maxY':lower_y[1],'minZ':0,'maxZ':9.525,
                       'outwardNormal':[-1,0,0],'sourceHandles':{'side':['107','108','109'],'top':['109']},
                       'supportSurfaceZ':None,'supportMeaning':'Vertical stop face; no forward support ledge established by these drawings.'},
              'upper':{'planeX':25.4,'minY':upper_y[0],'maxY':upper_y[1],'minZ':9.525,'maxZ':19.05,
                       'outwardNormal':[-1,0,0],'sourceHandles':{'side':['105','106'],'top':['100']},
                       'supportSurfaceZ':9.525,'supportTerraceXY':[list(p) for p in terrace.exterior.coords[:-1]]},
              'hingeAxis':{'point':[length-9.525,0,9.525],'direction':[0,1,0],'boreDiameterMm':7.9375,
                           'motionModeled':False,'sourceHandle':'side:100'}}
    references=[
        {'type':'official','url':ASSEMBLY,'section':'Backgauge assembly, parts and calibration',
         'retrievedDate':'2026-11-06','facts':['Two finger assemblies, adjusted manually along the gantry rail.',
          'Upper reference edge lies 25.4 mm behind the lower edge in positive X.',
          'R zero is established by aligning the chosen material support surface with the die top.']},
        {'type':'official','url':SPEC,'section':'Specifications / Backgauge Travel','retrievedDate':'2026-11-06',
         'facts':['Nominal X travel 431.8 mm; nominal R travel 121.92 mm. Travel lengths do not establish absolute calibrated limits.']},
        {'type':'snapshot','path':'analysis/qt-review/snapshot/qt/qml/RunExecutionCanvas.qml',
         'sha256':sha((ROOT/'analysis/qt-review/snapshot/qt/qml/RunExecutionCanvas.qml').read_bytes()),
         'snapshotManifest':'analysis/qt-review/snapshot/manifest.json',
         'lines':[14746,14778,14779,14783,16020,17085,17887],
         'facts':['Qt constructs a nominal side silhouette and uses the top-view width for extrusion.',
                  'Qt loaded-DXF anchor is at the lowest support level; fallback anchor uses 9.525 mm instead. The new model avoids importing this ambiguity.']},
        {'type':'user','facts':['The machine has two backgauge fingers moving in X and R.',
                               'Both fingers remain physically present; one or two may be used for locating and squaring the workpiece.',
                               'The user has not supplied the longitudinal spacing.']},
        {'type':'model-assumption','facts':['The two fingers share X/R motion.',
              'Longitudinal station positions are fixed for one analyzed sequence because no Y actuation or manual station reconfiguration is modeled.']}
    ]
    definition={'id':'titan25t-finger','kind':'backgauge-finger','status':'conservative-from-two-projections',
                'mesh':mesh(nominal),'collisionMesh':mesh(collision),'contactFeatures':features,
                'sourceToLocal':{'x':'101.6 - DXF x','y':'DXF top y + 25.4','z':'DXF side y','units':'mm'},
                'profiles':{'sideXZ':[list(p) for p in side_local.exterior.coords[:-1]],'topXY':[list(p) for p in top_local.exterior.coords[:-1]]},
                'arcToleranceMm':ARC_TOL,'collisionPaddingMm':ARC_TOL+1e-8,
                'modelMeaning':'Intersection of two extruded orthographic silhouettes, with unknown circular void depths left solid.',
                'limits':['This visual hull may overestimate material in hidden chamfers, holes and reliefs.',
                          'Finger mounting brackets, pivot hardware, gantry and other machine bodies are absent.',
                          'The finger is treated as rigid and level; pivoting/lift-off is not modeled.',
                          'Contact faces use their distinct top-view contours, not the inflated collision shell.'],
                'sources':[side['source'],top['source']]}
    payload={'schemaVersion':1,'units':'mm','coordinateSystem':{'x':'feed; front negative, rear positive','y':'station along bed','z':'up; die shoulder reference'},
             'definitions':[definition],
             'instances':[{'id':'finger-left','definitionId':definition['id'],'stationY':None},
                          {'id':'finger-right','definitionId':definition['id'],'stationY':None}],
             'commonMotion':{'axes':['x','r'],'rMapsTo':'z','lowerFaceX':None,'baseZ':None,'stationPositionsFixedForSequence':True,
                             'stationActuation':'manual setup; no assumed Y motor',
                             'stationPlacementStatus':'model assumption; spacing not supplied by user'},
             'contactRequirement':{'allowedCounts':[1,2],'requiredCount':None,
                                   'obstacleCount':2,'nonContactedFingerRemainsAnObstacle':True},
             'travel':{'xTravelMm':431.8,'rTravelMm':121.92,'xAbsoluteLimitsMm':None,'rAbsoluteLimitsMm':None,
                       'stationLimitsMm':None,'calibrationStatus':'absolute bounds and station positions not supplied'},
             'provenance':references}
    # Keep the exact STEP finger (bend-control tools/step_finger_to_dxf.py) as the
    # default definitions[0]; this script only regenerates the legacy DXF hull.
    previous=ROOT/'viewer/fingers.json'
    if previous.exists():
        kept=[d for d in json.loads(previous.read_text(encoding='utf-8')).get('definitions',[]) if d.get('id')=='titan25t-finger-original']
        if kept:
            payload['definitions']=kept+payload['definitions']
            for instance in payload['instances']:instance['definitionId']=kept[0]['id']
    OUT.mkdir(parents=True,exist_ok=True)
    (ROOT/'viewer/fingers.json').write_text(json.dumps(payload,separators=(',',':'),ensure_ascii=False,allow_nan=False),encoding='utf-8')
    evidence={'definitionId':definition['id'],'contactFeatures':features,'nominalMesh':definition['mesh']['validation'],
              'collisionMesh':definition['collisionMesh']['validation'],'nominalBounds':definition['mesh']['bounds'],
              'collisionBounds':definition['collisionMesh']['bounds'],'source':definition['sources'],'references':references,
              'containment':{'nominalMinusCollisionVolumeMm3':nominal.cut(collision).Volume,'method':'FreeCAD solid subtraction'}}
    if evidence['containment']['nominalMinusCollisionVolumeMm3']>1e-7:raise ValueError('Padded hull does not contain nominal hull')
    (OUT/'verification.json').write_text(json.dumps(evidence,indent=2,ensure_ascii=False),encoding='utf-8')
    print(json.dumps({'output':'viewer/fingers.json','nominal':definition['mesh']['validation'],'collision':definition['collisionMesh']['validation'],
                      'contactFeatures':features},ensure_ascii=False))
    return payload


if __name__=='__main__':build()
