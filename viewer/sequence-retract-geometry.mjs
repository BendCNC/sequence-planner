/** Vertical punch clearance for prismatic tooling. No machine travel limit is assumed. */
import {punchHeightKernel} from './mesh-separation.mjs';
const EPS=1e-9;
const point=(a,i)=>[a[i*3],a[i*3+1],a[i*3+2]];
const lerp=(a,b,t)=>a.map((v,i)=>v+(b[i]-v)*t);
const unique=values=>values.sort((a,b)=>a-b).filter((v,i,a)=>!i||v-a[i-1]>1e-9);

function line(a,b,sourceEdge){
  const minX=Math.min(a[0],b[0]),maxX=Math.max(a[0],b[0]);
  return maxX-minX<EPS?{minX,maxX,vertical:true,z:Math.min(a[1],b[1]),sourceEdge}:
    {minX,maxX,m:(b[1]-a[1])/(b[0]-a[0]),c:a[1]-(b[1]-a[1])*a[0]/(b[0]-a[0]),sourceEdge};
}
const valueAt=(l,x)=>l.vertical?l.z:l.m*x+l.c;

/** Lowest filled-profile Z at each X, optionally expanded by an L-infinity box. */
export function preparePunchShadow(tool,segment,{clearanceMm=0}={}){
  if(!Number.isFinite(clearanceMm)||clearanceMm<0)throw new RangeError('Invalid numerical clearance.');
  if(!(segment.lengthMm>0)||!Number.isFinite(segment.centerY))throw new RangeError('Invalid punch segment.');
  const r=clearanceMm,mirror=segment.mirror?-1:1,vertices=(tool.vertices2D??tool.profile).map(([x,z])=>
    [mirror*x+(segment.offsetX??0),z+(segment.offsetZ??0)]);
  const loops=tool.loops??[vertices.map((_,i)=>i)],lines=[];
  for(const loop of loops)for(let i=0;i<loop.length;i++){
    const ia=loop[i],ib=loop[(i+1)%loop.length],a=vertices[ia],b=vertices[ib],sourceEdge=[ia,ib];
    if(r===0)lines.push(line(a,b,sourceEdge));
    else{
      // The lower envelope of a segment swept by a square is represented by
      // its two shifted copies and the horizontal sweeps of its endpoints.
      lines.push(line([a[0]-r,a[1]-r],[b[0]-r,b[1]-r],sourceEdge));
      lines.push(line([a[0]+r,a[1]-r],[b[0]+r,b[1]-r],sourceEdge));
      lines.push(line([a[0]-r,a[1]-r],[a[0]+r,a[1]-r],sourceEdge));
      lines.push(line([b[0]-r,b[1]-r],[b[0]+r,b[1]-r],sourceEdge));
    }
  }
  const breaks=lines.flatMap(l=>[l.minX,l.maxX]);
  for(let i=0;i<lines.length;i++)for(let j=i+1;j<lines.length;j++){
    const a=lines[i],b=lines[j];if(a.vertical||b.vertical||Math.abs(a.m-b.m)<EPS)continue;
    const x=(b.c-a.c)/(a.m-b.m);
    if(x>Math.max(a.minX,b.minX)+EPS&&x<Math.min(a.maxX,b.maxX)-EPS)breaks.push(x);
  }
  const xs=unique(breaks),envelope=[];
  for(let i=1;i<xs.length;i++){
    const minX=xs[i-1],maxX=xs[i],x=(minX+maxX)/2;
    let best=null;
    for(const l of lines)if(!l.vertical&&x>=l.minX-EPS&&x<=l.maxX+EPS&&(!best||valueAt(l,x)<valueAt(best,x)))best=l;
    if(!best)continue;
    const last=envelope.at(-1);
    if(last&&!last.vertical&&Math.abs(last.maxX-minX)<EPS&&Math.abs(last.m-best.m)<EPS&&Math.abs(last.c-best.c)<EPS)last.maxX=maxX;
    else envelope.push({...best,minX,maxX});
  }
  for(const l of lines.filter(l=>l.vertical)){
    const covered=envelope.filter(e=>l.minX>=e.minX-EPS&&l.minX<=e.maxX+EPS);
    if(!covered.length||l.z<Math.min(...covered.map(e=>valueAt(e,l.minX)))-EPS)envelope.push(l);
  }
  envelope.sort((a,b)=>a.minX-b.minX);
  return{segmentId:segment.id,toolId:tool.id,clearanceMm:r,
    minY:segment.centerY-segment.lengthMm/2-r,maxY:segment.centerY+segment.lengthMm/2+r,
    minX:Math.min(...vertices.map(v=>v[0]))-r,maxX:Math.max(...vertices.map(v=>v[0]))+r,envelope,
    marginModel:r?'CONSERVATIVE_AXIS_ALIGNED_BOX':'EXACT_PROFILE_VERTICAL_SHADOW'};
}

function clipY(poly,y,keepAbove){
  const out=[];
  for(let i=0;i<poly.length;i++){
    const a=poly[i],b=poly[(i+1)%poly.length],da=(a[1]-y)*(keepAbove?1:-1),db=(b[1]-y)*(keepAbove?1:-1);
    if(da>=-EPS)out.push(a);
    if((da>=0)!==(db>=0))out.push(lerp(a,b,da/(da-db)));
  }return out;
}

/** Clip actual piece triangles against the finite segment's Y interval. */
export function preparePartForPunch(bodies,punch,points=new WeakMap()){
  const polygons=[];
  for(const body of bodies){
    let xyz=points.get(body);
    if(!xyz){xyz=new Array(body.positions.length/3);points.set(body,xyz);}
    for(let i=0;i<body.triangles.length;i+=3){
      const a=body.triangles[i],b=body.triangles[i+1],c=body.triangles[i+2],p=body.positions;
      const min=Math.min(p[3*a+1],p[3*b+1],p[3*c+1]),max=Math.max(p[3*a+1],p[3*b+1],p[3*c+1]);
      if(max<punch.minY-EPS||min>punch.maxY+EPS)continue;
      let vertices=[xyz[a]??=point(p,a),xyz[b]??=point(p,b),xyz[c]??=point(p,c)];
      if(min<punch.minY)vertices=clipY(vertices,punch.minY,true);
      if(max>punch.maxY)vertices=clipY(vertices,punch.maxY,false);
      if(vertices.length)polygons.push({bodyId:body.id,triangleIndex:i/3,vertices});
    }
  }
  return{segmentId:punch.segmentId,polygons};
}

/** Evaluate a single prepared punch against consecutive poses of one rigid route.
 * At a fixed tilt only the X/Z translations change. Precompute the identical
 * rotation terms and X extrema and retain polygon/edge traversal order.
 */
export function createRequiredTipEvaluator(preparedPart,punch,pivot=[0,0,0]){
  let priorAngle,projected,scan;
  return (pose,knownMaximum=-Infinity)=>{
    const angle=(pose.tiltDeg??0)*Math.PI/180,tx=pose.x??0,tz=pose.z??0;
    if(!Object.is(priorAngle,angle)){
      const c=Math.cos(angle),s=Math.sin(angle);
      scan=punchHeightKernel(preparedPart.polygons,punch.envelope,pivot,c,s);
      if(!scan)projected=preparedPart.polygons.map(polygon=>{
        let minX=Infinity,maxX=-Infinity;
        const vertices=polygon.vertices.map(point=>{
          const x=point[0]-pivot[0],z=point[2]-pivot[2],px=pivot[0]+c*x+s*z;
          minX=Math.min(minX,px);maxX=Math.max(maxX,px);
          const pz=pivot[2]-s*x+c*z;
          return [px,point[1],pz];
        });
        return {polygon,vertices,minX,maxX};
      });
      priorAngle=angle;
    }
    if(scan){
      const hit=scan(tx,tz,knownMaximum);
      if(!hit)return {requiredTipZMm:null,witness:null};
      const {a,b,polygon,shadow,x,height}=hit,ax=a[0]+tx,bx=b[0]+tx,az=a[2]+tz,bz=b[2]+tz;
      const dx=bx-ax,t=Math.abs(dx)<EPS?0:(x-ax)/dx;
      const partPoint=Math.abs(dx)<EPS?(az>bz?[ax,a[1],az]:[bx,b[1],bz]):
        [ax+dx*t,a[1]+(b[1]-a[1])*t,az+(bz-az)*t];
      return {requiredTipZMm:height,witness:{partBodyId:polygon.bodyId,triangleIndex:polygon.triangleIndex,
        punchSegmentId:punch.segmentId,toolId:punch.toolId,partPoint,punchShadowPointAtTipZero:[x,partPoint[1],valueAt(shadow,x)],
        sourceProfileEdge:shadow.sourceEdge,marginModel:punch.marginModel,requiredTipZMm:height}};
    }
    let maximum=knownMaximum,witness=null;
    for(const {polygon,vertices,minX,maxX} of projected){
      if(maxX+tx<punch.minX-EPS||minX+tx>punch.maxX+EPS)continue;
      for(let i=0;i<vertices.length;i++){
        const a=vertices[i],b=vertices[(i+1)%vertices.length],ax=a[0]+tx,bx=b[0]+tx,az=a[2]+tz,bz=b[2]+tz;
        const edgeMin=Math.min(ax,bx),edgeMax=Math.max(ax,bx),dx=bx-ax,dz=bz-az,vertical=Math.abs(dx)<EPS;
        for(const shadow of punch.envelope){
          const low=Math.max(edgeMin,shadow.minX),high=Math.min(edgeMax,shadow.maxX);
          if(high<low-EPS)continue;
          for(let boundary=0;boundary<(low===high?1:2);boundary++){
            const x=boundary?high:low,t=vertical?0:(x-ax)/dx;
            const partZ=vertical?(az>bz?az:bz):az+dz*t,toolZ=valueAt(shadow,x),required=partZ-toolZ;
            if(required>maximum){
              const partPoint=vertical?(az>bz?[ax,a[1],az]:[bx,b[1],bz]):[ax+dx*t,a[1]+(b[1]-a[1])*t,az+dz*t];
              maximum=required;
              witness={partBodyId:polygon.bodyId,triangleIndex:polygon.triangleIndex,punchSegmentId:punch.segmentId,
                toolId:punch.toolId,partPoint,punchShadowPointAtTipZero:[x,partPoint[1],toolZ],
                sourceProfileEdge:shadow.sourceEdge,marginModel:punch.marginModel,requiredTipZMm:required};
            }
          }
        }
      }
    }
    return{requiredTipZMm:witness?maximum:null,witness};
  };
}

/** Minimum upper clear component of vertical translation, for one rigid pose. */
export function requiredTipForPose(preparedPart,punch,pose,pivot=[0,0,0]){
  return createRequiredTipEvaluator(preparedPart,punch,pivot)(pose);
}

export function sampleRigidRoute({keyframes,partBodies,pivot=[0,0,0],pointStepBoundMm=1}={}){
  if(!Array.isArray(keyframes)||!keyframes.length||!(pointStepBoundMm>0))throw new Error('Route and positive sampling step are required.');
  let radius=0;
  for(const body of partBodies)for(let i=0;i<body.positions.length;i+=3)radius=Math.max(radius,Math.hypot(body.positions[i]-pivot[0],body.positions[i+2]-pivot[2]));
  const poses=[];
  const normalize=p=>{const n={x:p.x??0,y:p.y??0,z:p.z??0,tiltDeg:p.tiltDeg??0};
    if(Object.values(n).some(v=>!Number.isFinite(v))||Math.abs(n.y)>EPS)throw new Error('This route sampler supports finite X/Z/Y-tilt poses and fixed station Y.');return n;};
  poses.push({pose:normalize(keyframes[0].pose),segmentIndex:0,fraction:0});
  let observed=0;
  for(let i=1;i<keyframes.length;i++){
    const a=normalize(keyframes[i-1].pose),b=normalize(keyframes[i].pose),bound=Math.hypot(b.x-a.x,b.z-a.z)+radius*Math.abs(b.tiltDeg-a.tiltDeg)*Math.PI/180;
    const steps=Math.max(1,Math.ceil(bound/pointStepBoundMm));observed=Math.max(observed,bound/steps);
    for(let j=1;j<=steps;j++){const f=j/steps;poses.push({pose:{x:a.x+(b.x-a.x)*f,y:0,z:a.z+(b.z-a.z)*f,tiltDeg:a.tiltDeg+(b.tiltDeg-a.tiltDeg)*f},segmentIndex:i,fraction:f});}
  }
  return{poses,maximumPartRadiusFromPivotMm:radius,maxObservedPointDisplacementBoundMm:observed,continuousCollisionDetection:false};
}
