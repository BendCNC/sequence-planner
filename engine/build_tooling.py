"""Build local tool profiles from the frozen Qt catalog and DXF snapshots.

Run with FreeCAD 1.1's bundled Python (Shapely 2.1). No source workspace writes.
Profiles use [x,z] millimetres; the prism axis is +y along the machine bed.
"""
from __future__ import annotations
import hashlib
import json
import math
import re
from pathlib import Path

import shapely
from shapely.geometry import LineString, Point, Polygon
from shapely.geometry.polygon import orient

ROOT = Path(__file__).resolve().parents[1]
SNAPSHOT = ROOT / 'analysis/qt-review/snapshot'
CATALOG = SNAPSHOT / 'qt/qml/data/current'
ASSETS = ROOT / 'analysis/qt-review/tooling/assets'
OUTPUT = ROOT / 'viewer/tooling.json'
ARC_TOLERANCE_MM = 0.015
UNITS_TO_MM = {1: 25.4, 2: 304.8, 4: 1.0, 5: 10.0, 6: 1000.0}
PUNCH_FILES = {'3169': 'Acute 30° Punch [3169].dxf', '9302': 'Standart 30° Punch [9302].dxf',
               '2287': 'Heavy Duty 35° Punch [2287].dxf', '8392': 'Goodeneck 88° Punch [8392].dxf',
               '3924': 'Extreme Gooseneck 85° Punch [3924].dxf'}
DIE_FILES = {'9355': 'Acute 26mm 30° Die [9355].dxf', '9858': 'Acute 16.5mm 30° Die [9858].dxf',
             '5623': 'Acute 8mm 30° Die [5623].dxf', '4387': 'Standart 4-Way Die [4387].dxf'}
FOUR_WAY = {'TOP': {'rotation2DDegrees': 0, 'openingMm': 16.0},
            'RIGHT': {'rotation2DDegrees': 90, 'openingMm': 35.0},
            'BOTTOM': {'rotation2DDegrees': 180, 'openingMm': 22.0},
            'LEFT': {'rotation2DDegrees': 270, 'openingMm': 50.0}}

def digest(data): return hashlib.sha256(data).hexdigest()
def load_catalog(name):
    path=CATALOG/name;data=path.read_bytes()
    manifest=json.loads((SNAPSHOT/'manifest.json').read_text(encoding='utf-8'))
    relative=path.relative_to(SNAPSHOT).as_posix()
    expected=next((item['sha256'] for item in manifest['files'] if item['path']==relative),None)
    if not expected or digest(data)!=expected:raise ValueError(f'Catalog snapshot hash changed: {relative}')
    return json.loads(data.decode('utf-8'))
def signed_area(points): return sum(a[0]*b[1]-b[0]*a[1] for a,b in zip(points,points[1:]+points[:1]))/2
def finite(value): return isinstance(value,(int,float)) and math.isfinite(value)
def clean_points(points):
    result=[]
    for point in points:
        if not result or math.dist(point,result[-1])>1e-9: result.append([float(point[0]),float(point[1])])
    if len(result)>1 and math.dist(result[0],result[-1])<1e-9: result.pop()
    return result

def sample_arc(center,radius,start,sweep,tolerance=ARC_TOLERANCE_MM):
    if radius<=0 or abs(sweep)<1e-14: raise ValueError('Invalid circular arc')
    delta=2*math.acos(max(-1,min(1,1-tolerance/radius))) if tolerance<2*radius else math.pi
    count=max(1,math.ceil(abs(sweep)/max(delta,1e-5)))
    fractions={i/count for i in range(count+1)}
    # Include exact extrema; contact anchors must not depend on segment count.
    for k in range(-8,9):
        fraction=(k*math.pi/2-start)/sweep
        if 0<fraction<1: fractions.add(fraction)
    return [[center[0]+radius*math.cos(start+sweep*f),center[1]+radius*math.sin(start+sweep*f)] for f in sorted(fractions)]

def bulge_segment(a,b,bulge):
    if abs(bulge)<1e-14:return [a,b],None
    chord=math.dist(a,b)
    if chord<1e-10:raise ValueError('Zero-length bulge segment')
    mid=[(a[0]+b[0])/2,(a[1]+b[1])/2]
    offset=chord*(1-bulge*bulge)/(4*bulge)
    center=[mid[0]-(b[1]-a[1])/chord*offset,mid[1]+(b[0]-a[0])/chord*offset]
    radius=chord*(1+bulge*bulge)/(4*abs(bulge))
    start=math.atan2(a[1]-center[1],a[0]-center[0]);sweep=4*math.atan(bulge)
    points=sample_arc(center,radius,start,sweep);points[0]=list(a);points[-1]=list(b)
    return points,{'center':center,'radiusMm':radius,'startRadians':start,'sweepRadians':sweep,'bulge':bulge}

def parse_dxf(path):
    data=path.read_bytes();lines=data.decode('utf-8-sig').splitlines()
    if len(lines)%2: raise ValueError(f'Odd DXF token count: {path}')
    pairs=[(int(lines[i].strip()),lines[i+1].strip()) for i in range(0,len(lines),2)]
    insunits=None
    for i,(code,value) in enumerate(pairs[:-1]):
        if code==9 and value=='$INSUNITS':insunits=int(pairs[i+1][1])
    if insunits not in UNITS_TO_MM:raise ValueError(f'Unsupported or unspecified DXF units: {insunits}')
    scale=UNITS_TO_MM[insunits];entities=[];section='';i=0
    while i<len(pairs):
        code,value=pairs[i]
        if code==0 and value=='SECTION':section=pairs[i+1][1];i+=2;continue
        if code==0 and value=='ENDSEC':section=''
        if section=='ENTITIES' and code==0 and value not in ('ENDSEC','EOF'):
            group=[];j=i+1
            while j<len(pairs) and pairs[j][0]!=0:group.append(pairs[j]);j+=1
            entities.append((value,group));i=j;continue
        i+=1
    loops=[];open_paths=[];arcs=[];types={}
    for kind,group in entities:
        types[kind]=types.get(kind,0)+1
        values={code:value for code,value in group}
        if kind=='LWPOLYLINE':
            verts=[]
            for code,value in group:
                if code==10:verts.append({'x':float(value)*scale,'z':None,'bulge':0})
                elif code==20:verts[-1]['z']=float(value)*scale
                elif code==42:verts[-1]['bulge']=float(value)
            if any(v['z'] is None for v in verts):raise ValueError('Incomplete polyline vertex')
            closed=bool(int(values.get(70,'0'))&1);points=[]
            for idx in range(len(verts) if closed else len(verts)-1):
                a,b=verts[idx],verts[(idx+1)%len(verts)]
                segment,arc=bulge_segment([a['x'],a['z']],[b['x'],b['z']],a['bulge'])
                points.extend(segment[:-1])
                if arc:arcs.append(arc)
            if not closed:points.append([verts[-1]['x'],verts[-1]['z']])
            (loops if closed else open_paths).append(clean_points(points))
        elif kind=='LINE':
            open_paths.append([[float(values[10])*scale,float(values[20])*scale],[float(values[11])*scale,float(values[21])*scale]])
        elif kind in ('ARC','CIRCLE'):
            center=[float(values[10])*scale,float(values[20])*scale];radius=float(values[40])*scale
            start=math.radians(float(values.get(50,'0')))
            sweep=2*math.pi if kind=='CIRCLE' else math.radians((float(values[51])-float(values[50]))%360)
            points=sample_arc(center,radius,start,sweep)
            arcs.append({'center':center,'radiusMm':radius,'startRadians':start,'sweepRadians':sweep})
            (loops if kind=='CIRCLE' else open_paths).append(clean_points(points) if kind=='CIRCLE' else points)
        else:raise ValueError(f'Unsupported profile entity {kind} in {path.name}')
    # Join LINE/ARC fragments with an explicit endpoint tolerance.
    while open_paths:
        chain=open_paths.pop(0)
        while math.dist(chain[0],chain[-1])>1e-7:
            found=False
            for index,fragment in enumerate(open_paths):
                if math.dist(chain[-1],fragment[0])<1e-7:chain+=fragment[1:];found=True
                elif math.dist(chain[-1],fragment[-1])<1e-7:chain+=list(reversed(fragment))[1:];found=True
                if found:open_paths.pop(index);break
            if not found:raise ValueError(f'Open profile in {path.name}')
        loops.append(clean_points(chain))
    if not loops:raise ValueError(f'No closed profile in {path.name}')
    loops.sort(key=lambda p:abs(signed_area(p)),reverse=True)
    outer=Polygon(loops[0]);holes=[]
    for loop in loops[1:]:
        poly=Polygon(loop)
        if not outer.contains(poly):raise ValueError('Multiple disjoint profiles are not silently merged')
        holes.append(loop)
    polygon=Polygon(loops[0],holes)
    if not polygon.is_valid:raise ValueError(f'Invalid DXF polygon {path.name}: {shapely.is_valid_reason(polygon)}')
    return {'polygon':orient(polygon,sign=1),'arcs':arcs,'source':{'path':str(path.relative_to(ROOT)).replace('\\','/'),
        'sha256':digest(data),'dxfInsunits':insunits,'units':'in' if insunits==1 else 'mm-scaled','scaleToMm':scale,
        'entityTypes':types,'arcCount':len(arcs),'arcToleranceMm':ARC_TOLERANCE_MM}}

def rotated(poly,degrees):
    angle=math.radians(degrees);c=math.cos(angle);s=math.sin(angle)
    def change(loop):return [[c*x-s*z,s*x+c*z] for x,z in loop.coords[:-1]]
    return Polygon(change(poly.exterior),[change(r) for r in poly.interiors])

def profile_transform(poly,x_origin,z_origin):
    # DXF left view: +X (drawing right) is the operator front; machine +X is the rear (backgauge).
    def shift(loop):return [[x_origin-x,z-z_origin] for x,z in loop.coords[:-1]]
    return orient(Polygon(shift(poly.exterior),[shift(r) for r in poly.interiors]),sign=1)

def upper_gap(poly):
    minx,minz,maxx,maxz=poly.bounds
    points=list(poly.exterior.coords[:-1]);xs=sorted(set(round(x,10) for x,z in points if abs(z-maxz)<1e-7))
    gaps=[]
    for left,right in zip(xs,xs[1:]):
        if right-left>1e-6 and not poly.covers(Point((left+right)/2,maxz-1e-5)):
            gaps.append((left,right))
    if not gaps: return None
    return min(gaps,key=lambda g:abs((g[0]+g[1]-minx-maxx)/2))

def near_top_gap(poly,depth_mm=0.01):
    minx,minz,maxx,maxz=poly.bounds
    cut=poly.intersection(LineString([(minx-1,maxz-depth_mm),(maxx+1,maxz-depth_mm)]))
    pieces=list(cut.geoms) if hasattr(cut,'geoms') else [cut]
    spans=sorted((min(p[0] for p in piece.coords),max(p[0] for p in piece.coords)) for piece in pieces if hasattr(piece,'coords') and not piece.is_empty)
    gaps=[(a[1],b[0]) for a,b in zip(spans,spans[1:]) if b[0]-a[1]>1e-7]
    return min(gaps,key=lambda g:abs((g[0]+g[1]-minx-maxx)/2)) if gaps else None

def top_material_at(poly,x):
    minx,minz,maxx,maxz=poly.bounds
    result=poly.intersection(LineString([(x,minz-1),(x,maxz+1)]))
    pieces=list(result.geoms) if hasattr(result,'geoms') else [result]
    zs=[p[1] for piece in pieces if hasattr(piece,'coords') for p in piece.coords]
    return max(zs) if zs else None

def triangulate(poly):
    loops=[list(poly.exterior.coords[:-1])]+[list(r.coords[:-1]) for r in poly.interiors]
    vertices=[list(p) for loop in loops for p in loop];lookup={(round(x,10),round(z,10)):i for i,(x,z) in enumerate(vertices)}
    triangles=[];area=0
    for triangle in shapely.constrained_delaunay_triangles(poly).geoms:
        coords=list(triangle.exterior.coords[:-1])
        if len(coords)!=3 or not poly.covers(triangle):raise ValueError('Invalid constrained triangle')
        if signed_area(coords)<0:coords.reverse()
        triangles.extend(lookup[(round(x,10),round(z,10))] for x,z in coords)
        area+=triangle.area
    if abs(area-poly.area)>1e-7*max(1,poly.area):raise ValueError('Triangulation area mismatch')
    offset=0;indices=[]
    for loop in loops:indices.append(list(range(offset,offset+len(loop))));offset+=len(loop)
    return {'profile':vertices[:len(loops[0])],'holes':[list(map(list,p)) for p in loops[1:]],
            'vertices2D':vertices,'loops':indices,'triangles2D':triangles,'areaMm2':poly.area}

def build_tool(record,kind,parsed,vendor_code=None):
    catalog_height=record.get('heightInch');height=catalog_height*25.4 if finite(catalog_height) else None
    counts={str(length*25.4):int(round(record.get(f'segmentCount{length}In',0))) for length in (16,8,4,2,1)}
    lengths=[float(length) for length,count in counts.items() if count>0]
    base={'id':record['id'],'name':record['name'],'kind':kind,'vendorCode':vendor_code,'status':'ready' if parsed else 'missing',
          'heightMm':height,'availableLengthsMm':lengths,'stockLengthsMm':lengths,'inventory':[
              {'lengthMm':float(length),'count':count} for length,count in counts.items() if count>0],
          'catalogSourceHash':digest((CATALOG/('punches.json' if kind=='punch' else 'dies.json')).read_bytes()) if kind in ('punch','die') else None}
    if parsed is None:
        return {**base,'profile':[],'holes':[],'vertices2D':[],'loops':[],'triangles2D':[],
                'sourceHash':None,'bounds':None,'missingReason':'Exact tool profile is absent from the frozen asset manifest. No substitute geometry is generated.',
                'contact':{'openingMm':record.get('vdieOpeningInch',0)*25.4,'openingAngleDegrees':record.get('dieAngleDegree'),
                           'tipRadiusMm':None,'apexDepthMm':None,'shoulderXs':None,'heightMm':height}}
    poly=parsed['polygon'];rotation=0;face='TOP';needs_riser='+ Riser' in record['name']
    if kind=='die' and vendor_code=='4387':
        opening=record['vdieOpeningInch']*25.4
        face=min(FOUR_WAY,key=lambda key:abs(FOUR_WAY[key]['openingMm']-opening))
        rotation=FOUR_WAY[face]['rotation2DDegrees'];poly=rotated(poly,rotation)
    minx,minz,maxx,maxz=poly.bounds
    x_origin=(minx+maxx)/2;z_origin=maxz;gap=None;tip_radius=None;shoulder_method='top-plane profile vertices'
    if kind=='punch':
        bottom=[x for x,z in poly.exterior.coords if abs(z-minz)<1e-7]
        x_origin=sum(bottom)/len(bottom);z_origin=minz
        for arc in parsed['arcs']:
            if abs(arc['center'][0]-x_origin)<1e-5 and abs(arc['center'][1]-arc['radiusMm']-minz)<1e-5:
                tip_radius=arc['radiusMm'];break
    elif kind in ('die','table'):
        gap=upper_gap(poly)
        if gap and abs((gap[0]+gap[1])/2-x_origin)>max(.1,(maxx-minx)*.05):
            # Some supplied traced profiles have a slightly sloping shoulder.
            # Preserve the symmetric body centerline, and report an approximate
            # near-top opening rather than falsely anchoring on the outer edge.
            gap=near_top_gap(poly);shoulder_method='profile intersection 0.01 mm below top; approximate shoulder registration'
        elif gap:x_origin=(gap[0]+gap[1])/2
    poly=profile_transform(poly,x_origin,z_origin)
    minx,minz,maxx,maxz=poly.bounds
    geometry_height=maxz-minz
    contact={'heightMm':height,'geometryHeightMm':geometry_height,'tipRadiusMm':tip_radius,
             'openingMm':record.get('vdieOpeningInch',0)*25.4 if kind=='die' else None,
             'openingAngleDegrees':record.get('dieAngleDegree') if kind=='die' else record.get('punchAngleDegree'),
             'apexDepthMm':None,'shoulderXs':None,'originMeaning':'punch tip' if kind=='punch' else 'top contact plane'}
    if gap:
        contact['shoulderXs']=[x_origin-gap[1],x_origin-gap[0]]
        contact['geometryOpeningMm']=gap[1]-gap[0]
        contact['shoulderRegistration']=shoulder_method
        center_top=top_material_at(poly,0)
        contact['apexDepthMm']=-center_top if center_top is not None else None
    if kind=='die':
        bare_height=height-6.35 if needs_riser else height
        mount_z=-bare_height
        contact.update({'mountPlaneZMm':mount_z,'seatingZMm':mount_z,'tangDepthMm':max(0,mount_z-minz),
                        'mountRegistration':'inferred from catalog working height; mating assembly not certified',
                        'catalogHeightIncludesRiser':needs_riser,'bareWorkingHeightMm':bare_height})
    elif kind=='punch':
        contact.update({'mountPlaneZMm':height,'tangDepthMm':max(0,maxz-height),
                        'mountRegistration':'inferred from catalog working height; holder geometry not supplied'})
    elif kind=='riser':
        seat=top_material_at(poly,0);levels={}
        coords=list(poly.exterior.coords[:-1])
        for a,b in zip(coords,coords[1:]+coords[:1]):
            if abs(a[1]-b[1])<1e-7 and a[1]<seat-1e-7:
                key=round(a[1],8);levels[key]=levels.get(key,0)+abs(a[0]-b[0])
        mount=max(levels,key=levels.get)
        contact.update({'seatingZMm':seat,'mountPlaneZMm':mount,'workingLiftMm':seat-mount,
                        'tangDepthMm':mount-minz,'mountRegistration':'planes derived from the supplied riser section'})
    elif kind=='table':contact.update({'seatingZMm':0,'mountRegistration':'top plane; center groove preserved'})
    return {**base,**triangulate(poly),'source':parsed['source'],'sourceHash':parsed['source']['sha256'],
            'bounds':{'minX':minx,'maxX':maxx,'minZ':minz,'maxZ':maxz,'widthMm':maxx-minx,'heightMm':geometry_height},
            'contact':contact,'variant':{'activeFace':face,'profileRotation2DDegrees':rotation,
               'physicalRotationAroundYDegrees':-rotation,'requiresRiser':needs_riser,
               'availableFaces':FOUR_WAY if vendor_code=='4387' else None},
            'sourceToProfile':{'rotation2DDegrees':rotation,'translatedOriginAfterRotation':[x_origin,z_origin],'mirrorX':True,'frame':'DXF +X (left view right) = operator front = machine -X'},
            'validation':{'polygonValid':True,'closed':True,'holeCount':len(poly.interiors),'bodySideValid':minz>=-1e-7 if kind=='punch' else maxz<=1e-7}}

def build():
    manifest=json.loads((ASSETS/'manifest.json').read_text(encoding='utf-8'))
    for entry in manifest['files']:
        p=ASSETS/entry['relativePath'].replace('\\','/')
        if digest(p.read_bytes())!=entry['sha256']:raise ValueError(f'Asset hash changed: {p}')
    parsed={};tools=[]
    for kind,filename,files,folder in [('punch','punches.json',PUNCH_FILES,'Punches'),('die','dies.json',DIE_FILES,'Dies')]:
        for record in load_catalog(filename):
            match=re.search(r'\[(?:BC-|LS-)?([^\]]+)\]',record['name']);code=match.group(1) if match else None
            asset=files.get(code);item=None
            if asset:
                key=f'{folder}/{asset}'
                if key not in parsed:parsed[key]=parse_dxf(ASSETS/key)
                item=parsed[key]
            tools.append(build_tool(record,kind,item,code))
    for kind,id_,name in [('table','machine-table','Table.dxf'),('riser','four-way-riser','4-Way Die Holder [RISER].dxf')]:
        item=parse_dxf(ASSETS/'Dies'/name);parsed[f'Dies/{name}']=item
        tools.append(build_tool({'id':id_,'name':name.removesuffix('.dxf')},kind,item))
    result={'schemaVersion':1,'units':'mm','coordinateSystem':{'profile':['x','z'],'extrusionAxis':'+y','handedness':'right',
        'profileWinding':'outer CCW / holes CW in x,z','prismOrigin':'profile origin at segment centerY unless overridden'},
        'arcToleranceMm':ARC_TOLERANCE_MM,'tools':tools,'byId':{t['id']:i for i,t in enumerate(tools)},
        'provenance':{'catalogSnapshot':'analysis/qt-review/snapshot/manifest.json','assetManifest':'analysis/qt-review/tooling/assets/manifest.json',
                      'builder':'engine/build_tooling.py','shapelyVersion':shapely.__version__},
        'limits':['Profiles preserve supplied geometry, including mounting tangs.',
                  'Catalog working height and full profile height are distinct.',
                  'Mating registration inferred from catalog is not a certified assembly fit.',
                  '2288 has no exact profile in this snapshot and remains explicitly missing.',
                  'Tip radius is null unless supported by an arc at the tip.']}
    OUTPUT.write_text(json.dumps(result,separators=(',',':'),ensure_ascii=False,allow_nan=False),encoding='utf-8')
    print(json.dumps({'tools':len(tools),'ready':sum(t['status']=='ready' for t in tools),'missing':[t['name'] for t in tools if t['status']!='ready'],
        'profileVertices':sum(len(t.get('vertices2D',[])) for t in tools),'output':str(OUTPUT)},ensure_ascii=False))
    return result

if __name__=='__main__':build()
