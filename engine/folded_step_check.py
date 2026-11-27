"""Register the stationary panel, then compare the two final external surfaces."""
import math

from import_bcbend import band_point, evaluate_frames, transform
from import_step import kind, outward, plane_partner


def root_registration(shape, model, measures):
    import numpy as np
    z = np.array([0.,0.,1.])
    base = model['rootPanelId']
    hinges = [b for b in model['bends'] if b['parentPanelId']==base]
    if not hinges:
        return None
    axes = [np.array(b['axis']) for b in hinges]
    if not any(abs(a @ axes[0]) < .98 for a in axes):
        return None
    root = min((i for i,f in enumerate(shape.Faces) if kind(f)=='Plane'),
               key=lambda i:(-round(shape.Faces[i].Area,7),i))
    other,_ = plane_partner(shape,root)
    cylinders = [shape.Faces[measures[b['sourceBendId']]['stepFaces'][0]-1].Surface for b in hinges]
    panel = next(p for p in model['panels'] if p['id']==base)
    vertices = np.reshape(panel['flatPositions'],(-1,3))
    tri = vertices[np.reshape(panel['triangles'],(-1,3))]
    top = tri[np.all(abs(tri[:,:,2])<1e-7,axis=1)]
    areas = np.linalg.norm(np.cross(top[:,1]-top[:,0],top[:,2]-top[:,0]),axis=1)
    center = np.average(top.mean(axis=1),weights=areas,axis=0)
    candidates = []
    for index in (root,other):
        face = shape.Faces[index]
        normal = np.array(list(outward(face)))
        for sense in (1,-1):
            axis = sense*np.array(list(cylinders[0].Axis))
            rotation = (np.stack([axis,np.cross(normal,axis),normal]).T @
                        np.stack([axes[0],np.cross(z,axes[0]),z]))
            rows, rhs = [normal], [normal @ list(face.CenterOfMass)]
            for b,s in zip(hinges,cylinders):
                v = np.array(list(s.Axis))
                if abs((rotation @ b['axis']) @ v)<.99999:
                    break
                lateral = np.cross(normal,v)
                rows.append(lateral)
                rhs.append(lateral @ (np.array(list(s.Center))-rotation @ b['origin']))
            else:
                shift,_,rank,_ = np.linalg.lstsq(rows,rhs,rcond=None)
                if rank==3:
                    # Only the stationary panel and its tangent lines select
                    # the frame. Moving flanges cannot pull it towards the fit.
                    line_error = max(abs(np.array(rows) @ shift - rhs))
                    centroid_error = np.linalg.norm(rotation @ center+shift-list(face.CenterOfMass))
                    candidates.append((float(max(line_error,centroid_error)),rotation,shift,index+1))
    candidates.sort(key=lambda v:v[0])
    if not candidates or candidates[0][0]>.5 or (len(candidates)>1 and candidates[1][0]-candidates[0][0]<.1):
        return None
    return candidates[0]


def model_shell(model, graph):
    import numpy as np
    import shapely
    from shapely.geometry import Polygon
    units = graph['units']
    scale = 25.4 if units=='in' else 1.
    def loop(values):
        return [[float(p[k+'_'+units] if k+'_'+units in p else p[k])*scale for k in ('x','y')] for p in values]
    boundary = graph['materialBoundary']
    outline = Polygon(loop(boundary['outerLoop']),[loop(r) for r in boundary.get('holeLoops',[])]).boundary.buffer(1e-5)
    shapely.prepare(outline)
    frames = evaluate_frames(model,{b['id']:1 for b in model['bends']})
    points,indices,labels = [],[],[]
    for piece in [*model['panels'],*model['bends']]:
        mesh = piece.get('band',piece)
        flat = np.reshape(mesh['flatPositions'],(-1,3))
        triangles = np.reshape(mesh['triangles'],(-1,3))
        tv = flat[triangles]
        skin = np.ptp(tv[:,:,2],axis=1)<1e-7
        side = tv[~skin].mean(axis=1)
        # Internal mating caps are fabrication-model partitions. Their flat
        # projection lies inside the blank, away from exterior edges and holes.
        skin[~skin] = shapely.covers(outline,shapely.points(side[:,:2]))
        if 'band' in piece:
            f = frames[piece['parentPanelId']]
            final = [transform(f,band_point(piece,*q,1)) for q in np.reshape(mesh['parameters'],(-1,3))]
        else:
            final = [transform(frames[piece['id']],p) for p in flat]
        offset = len(points)
        indices.extend(triangles[skin]+offset)
        labels.extend([piece['label']]*int(skin.sum()))
        points.extend(final)
    return np.asarray(points),np.asarray(indices),np.asarray(labels)


def surface_distance(source, target):
    import numpy as np
    from vtkmodules.vtkCommonCore import vtkPoints, reference
    from vtkmodules.vtkCommonDataModel import vtkCellArray, vtkPolyData, vtkStaticCellLocator
    from vtkmodules.util.numpy_support import numpy_to_vtk, numpy_to_vtkIdTypeArray
    v,t = target
    points = vtkPoints()
    points.SetData(numpy_to_vtk(np.ascontiguousarray(v,dtype='float64'),deep=True))
    ids = numpy_to_vtkIdTypeArray(np.ascontiguousarray(t,dtype=np.int64).ravel(),deep=True)
    offsets = numpy_to_vtkIdTypeArray(np.arange(0,len(t)*3+1,3,dtype=np.int64),deep=True)
    cells = vtkCellArray()
    cells.SetData(offsets,ids)
    data = vtkPolyData()
    data.SetPoints(points)
    data.SetPolys(cells)
    locator = vtkStaticCellLocator()
    locator.SetDataSet(data)
    locator.BuildLocator()
    v,t = source
    used = np.unique(t)
    samples = np.concatenate((v[used],v[t].mean(axis=1)))
    hit,cell,sub,dist = [0.,0.,0.],reference(0),reference(0),reference(0.)
    values = []
    for p in samples:
        locator.FindClosestPoint(p,hit,cell,sub,dist)
        values.append(math.sqrt(float(dist)))
    values = np.array(values)
    return {'samples':len(values),'maxMm':float(values.max()),'p95Mm':float(np.percentile(values,95))},values[len(used):]


def compare_folded_step(shape, model, graph, measures):
    import numpy as np
    fit = root_registration(shape,model,measures)
    if fit is None:
        return {'status':'unregistered','message':'La base no define una alineación única con el STEP.'},None
    error,rotation,shift,face = fit
    vertices,triangles = shape.tessellate(.05)
    cad = ((np.array([list(v) for v in vertices])-shift) @ rotation,np.asarray(triangles))
    p,t,labels = model_shell(model,graph)
    forward,face_errors = surface_distance((p,t),cad)
    reverse,_ = surface_distance(cad,(p,t))
    result = {'status':'sampled','method':'stationary root registration; external triangle vertices and centroids, both directions',
              'rootFace':face,'rootFitMm':error,'stepTessellationMm':.05,
              'modelToStep':forward,'stepToModel':reverse,
              'maxMm':max(forward['maxMm'],reverse['maxMm']),
              'modelTriangleCentroidMaxMm':{str(k):float(face_errors[labels==k].max()) for k in np.unique(labels)}}
    reference_mesh = {'positions':cad[0].flatten().tolist(),'triangles':cad[1].flatten().tolist()}
    return result,reference_mesh
