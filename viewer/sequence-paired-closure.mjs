/** A conservative necessary-condition diagnostic, not a route search.
 * Two independent neighbouring hinges must eventually reach their final
 * amounts together. Test the other hinge's rigid wall in either final setup.
 * The proof is limited to the current, tip-aligned prismatic tooling model.
 */
import {evaluateModel} from './solver.mjs';
import {machinePlacement,partBodies,materialIntervals} from './machine.mjs';
import {advancedCompatibleTools} from './sequence-advanced-setups.mjs';
import {toolSupportPolicy} from './tool-support-policy.mjs';

const INTERIOR_MM=1e-7, COVERAGE_EPS_MM=1e-7, DEFAULT_JOINT_GAP_MM=.5;
const point=(positions,i)=>[positions[i],positions[i+1],positions[i+2]];
const interpolate=(a,b,t)=>a.map((x,i)=>x+t*(b[i]-x));

function clip(polygon,distance){
 const out=[];
 for(let i=0;i<polygon.length;i++){
  const a=polygon[i],b=polygon[(i+1)%polygon.length],da=distance(a),db=distance(b);
  if(da>=0)out.push(a);
  if((da<0&&db>0)||(da>0&&db<0))out.push(interpolate(a,b,da/(da-db)));
 }
 return out;
}
function union(intervals){
 const out=[];
 for(const interval of intervals.map(v=>v.slice()).sort((a,b)=>a[0]-b[0]||a[1]-b[1])){
  const last=out.at(-1);
  // Do not bridge a positive hole, even at a triangulation seam.
  if(last&&interval[0]<=last[1])last[1]=Math.max(last[1],interval[1]);else out.push(interval);
 }
 return out;
}
function intersect(a,b){
 const out=[];
 for(const x of a)for(const y of b){const lo=Math.max(x[0],y[0]),hi=Math.min(x[1],y[1]);if(hi>lo)out.push([lo,hi]);}
 return union(out);
}
function meshValid(positions,indices,stride){
 if(!positions||!indices||!Number.isInteger(positions.length)||positions.length<stride*3||positions.length%stride||
    !Number.isInteger(indices.length)||indices.length<3||indices.length%3)return false;
 for(let i=0;i<positions.length;i++)if(!Number.isFinite(positions[i]))return false;
 const count=positions.length/stride;
 for(let i=0;i<indices.length;i++)if(!Number.isInteger(indices[i])||indices[i]<0||indices[i]>=count)return false;
 return true;
}

function prepareClosureProfile(tool,mirror){
 const vertices=tool?.vertices2D??tool?.profile,indices=tool?.triangles2D;
 if(!Array.isArray(vertices)||!vertices.every(p=>Array.isArray(p)&&p.length===2&&p.every(Number.isFinite))||
  !meshValid(vertices.flat(),indices,2))return null;
 const triangles=[];
 let loX=Infinity,hiX=-Infinity,loZ=Infinity,hiZ=-Infinity;
 for(let i=0;i<indices.length;i+=3){
  const tri=Array.from(indices.slice(i,i+3),j=>[vertices[j][0]*(mirror?-1:1),vertices[j][1]]);
  const [a,b,c]=tri,area=(b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]);
  if(!Number.isFinite(area)||Math.abs(area)<1e-14)return null;
  const sign=Math.sign(area),planes=tri.map((p,j)=>{
   const q=tri[(j+1)%3],dx=q[0]-p[0],dz=q[1]-p[1],length=Math.hypot(dx,dz);
   return v=>sign*(dx*(v[2]-p[1])-dz*(v[0]-p[0]))/length-INTERIOR_MM;
  });
  const triangle={index:i/3,planes,minX:Math.min(a[0],b[0],c[0]),maxX:Math.max(a[0],b[0],c[0]),
    minZ:Math.min(a[1],b[1],c[1]),maxZ:Math.max(a[1],b[1],c[1])};
  triangles.push(triangle);
  loX=Math.min(loX,triangle.minX);hiX=Math.max(hiX,triangle.maxX);
  loZ=Math.min(loZ,triangle.minZ);hiZ=Math.max(hiZ,triangle.maxZ);
 }
 return {triangles,loX,hiX,loZ,hiZ};
}

/** Immutable tool profiles can be shared by all walls/poses in one proof. */
export function closureProfiles(){
 const normal=new Map(),reflected=new Map();
 return (tool,mirror)=>{
  const map=mirror?reflected:normal;
  if(!map.has(tool))map.set(tool,prepareClosureProfile(tool,mirror));
  return map.get(tool);
 };
}

/** Analytic clipping of real wall SURFACE triangles against the filled profile
 * triangles extruded along Y. Every returned polygon lies on its original
 * wall triangle and strictly inside a filled tool triangle by INTERIOR_MM.
 * Convexity certifies all intervening Y sections, not just sampled points.
 * Artificial triangle seams can only remove a certificate, never create one.
 * Coordinates are already relative to the registered punch tip; Y is the
 * unflipped material station coordinate. Exported for focused geometry tests.
 */
export function pairedClosureSurfaceIntervals(wall,tool,{mirror=false,stopAtPiece,profiles=prepareClosureProfile}={},guard=()=>{}){
 const profile=profiles(tool,mirror);
 if(!profile||!meshValid(wall?.positions,wall?.triangles,3))return null;
 const {triangles,loX,hiX,loZ,hiZ}=profile;
 const pieces=[];
 const p=wall.positions,faces=wall.triangles;
 for(let i=0;i<faces.length;i+=3){
  guard();
  const a=faces[i]*3,b=faces[i+1]*3,c=faces[i+2]*3;
  const minX=Math.min(p[a],p[b],p[c]),maxX=Math.max(p[a],p[b],p[c]);
  const minZ=Math.min(p[a+2],p[b+2],p[c+2]),maxZ=Math.max(p[a+2],p[b+2],p[c+2]);
  if(maxX<loX||minX>hiX||maxZ<loZ||minZ>hiZ)continue;
  const surface=[point(p,a),point(p,b),point(p,c)];
  const u=surface[1].map((v,j)=>v-surface[0][j]),v=surface[2].map((x,j)=>x-surface[0][j]);
  if(Math.hypot(u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0])<1e-14)continue;
  for(const t of triangles){
   if(maxX<t.minX||minX>t.maxX||maxZ<t.minZ||minZ>t.maxZ)continue;
   let polygon=surface;
   for(const plane of t.planes){polygon=clip(polygon,plane);if(!polygon.length)break;}
   if(polygon.length<3)continue;
   const lo=Math.min(...polygon.map(p=>p[1])),hi=Math.max(...polygon.map(p=>p[1]));
   if(hi>lo){
    const piece={interval:[lo,hi],wallTriangle:i/3,profileTriangle:t.index,polygon};
    if(stopAtPiece?.(piece))return {pieces:[piece],intervals:[piece.interval],strictInteriorMm:INTERIOR_MM};
    pieces.push(piece);
   }
  }
 }
 return{intervals:union(pieces.map(p=>p.interval)),pieces,strictInteriorMm:INTERIOR_MM};
}

function atY(polygon,y){
 const points=[];
 for(let i=0;i<polygon.length;i++){
  const a=polygon[i],b=polygon[(i+1)%polygon.length];
  if(a[1]===y)points.push(a);
  if((a[1]<y&&b[1]>y)||(a[1]>y&&b[1]<y))points.push(interpolate(a,b,(y-a[1])/(b[1]-a[1])));
 }
 if(!points.length)return null;
 return [0,1,2].map(i=>points.reduce((sum,p)=>sum+p[i],0)/points.length);
}
function witnesses(pieces,interval,flip){
 const selected=[];let cursor=interval[0];
 while(cursor<interval[1]){
  let best=null;
  for(const p of pieces)if(p.interval[0]<=cursor&&p.interval[1]>cursor&&(!best||p.interval[1]>best.interval[1]))best=p;
  if(!best)return null;
  const end=Math.min(interval[1],best.interval[1]),samples=[cursor,(cursor+end)/2,end].map(y=>atY(best.polygon,y));
  if(samples.some(p=>!p||!p.every(Number.isFinite)))return null;
  selected.push({wallTriangle:best.wallTriangle,profileTriangle:best.profileTriangle,materialIntervalMm:[cursor,end],
   surfacePointsTipRelativeMm:samples.map(p=>[p[0],p[1]*(flip?-1:1),p[2]])});
  cursor=end;
 }
 return selected;
}
function independent(model,bend){
 if(bend.strokeLeadId!=null||bend.strokeFollowerIds?.length||bend.splitHingeSpans>1||bend.source?.segmentIntervalsAxialMm?.length>1)return false;
 if(model.bends.some(b=>b!==bend&&(b.strokeFollowerIds??[]).map(String).includes(String(bend.id))))return false;
 if(bend.sourceBendId!=null&&model.bends.some(b=>b!==bend&&String(b.sourceBendId)===String(bend.sourceBendId)))return false;
 const material=materialIntervals(bend);
 return material.exact&&material.intervals.length===1;
}
function unsupportedScope(options){
 // Explicit inventories are harmless restrictions. Explicit mounted banks can
 // include another height, an offset or a rotation and need a separate proof.
 return ['setups','initialSegments','fixedStations','initialSetup','segments','stations'].some(k=>options[k]!=null)||
  ['offsetX','offsetZ','rotationYRad'].some(k=>options[k]!=null&&options[k]!==0);
}

function closure(model,catalog,active,other,shared,options,guard,profiles){
 const wallId=other.parentPanelId===shared?other.childPanelId:other.parentPanelId;
 const punches=advancedCompatibleTools(catalog,active,'punch',options),dies=advancedCompatibleTools(catalog,active,'die',options);
 if(!punches.length||!dies.length)return null;
 const allowance=toolSupportPolicy(options).maxUnsupportedEndMm;
 const epsilon=Math.max(COVERAGE_EPS_MM,options.coverageEpsilonMm??COVERAGE_EPS_MM);
 const gap=Math.max(DEFAULT_JOINT_GAP_MM,options.jointGapToleranceMm??DEFAULT_JOINT_GAP_MM);
 if(!Number.isFinite(epsilon)||!Number.isFinite(gap))return null;
 const material=materialIntervals(active).intervals[0],mandatory=[material[0]+allowance+epsilon,material[1]-allowance-epsilon];
 if(mandatory[1]-mandatory[0]<=gap+2*epsilon)return null;
 const amounts=Object.fromEntries(model.bends.map(b=>[b.id,b===active||b===other?1:0]));
 const state=evaluateModel(model,0,{amounts}),presentations=[];
 let common=[mandatory];
 for(const flip of [false,true])for(const punch of punches){
  guard();
  const placement=machinePlacement(model,state,active.id,dies[0],{flip,contactGapMm:options.contactGapMm??.3,stationYmm:0},punch);
  const body=partBodies(model,state,placement.frame).find(b=>String(b.panelId)===String(wallId));
  if(!body)return null;
  const wall={...body,positions:Array.from(body.positions,(v,i)=>i%3===2?v-placement.punchTipZ:i%3===1&&flip?-v:v)};
  for(const mirror of [false,true]){
   guard();const result=pairedClosureSurfaceIntervals(wall,punch,{mirror,profiles},guard);if(!result)return null;
   common=intersect(common,result.intervals);
   if(!common.some(([a,b])=>b-a>gap+2*epsilon))return null;
   presentations.push({punchId:punch.id,flip,mirror,result,punchProfileLiftMm:placement.punchProfileLiftMm});
  }
 }
 const widest=common.reduce((a,b)=>b[1]-b[0]>a[1]-a[0]?b:a),reserve=Math.max(1e-6,epsilon*2);
 const certified=[widest[0]+reserve,widest[1]-reserve];
 if(certified[1]-certified[0]<=gap+2*epsilon)return null;
 const proofs=[];
 for(const p of presentations){
  const proof=witnesses(p.result.pieces,certified,p.flip);if(!proof)return null;
  proofs.push({punchId:p.punchId,flip:p.flip,mirror:p.mirror,punchProfileLiftMm:p.punchProfileLiftMm,
   machineIntervalMm:p.flip?[-certified[1],-certified[0]]:certified.slice(),witnesses:proof});
 }
 return{bendId:active.id,alreadyClosedBendId:other.id,wallPanelId:wallId,amounts,
  materialIntervalMm:material,mandatoryIntervalMm:mandatory,certifiedMaterialIntervalMm:certified,
  certifiedWidthMm:certified[1]-certified[0],maximumAcceptedJointGapMm:gap,coverageEpsilonMm:epsilon,
  strictInteriorMm:INTERIOR_MM,referenceDieId:dies[0].id,compatibleDieIds:dies.map(d=>d.id),presentations:proofs};
}

/** null means inconclusive, NEVER feasible. A positive result only rules out
 * both final placements within the stated scope, including partial schedules.
 * No claim about other tools, independent X/Z shifts, mixed-profile banks,
 * alternative clamps, tool deformation or physical manufacturing is made.
 * Die support translates wall and punch tip equally in machinePlacement, so
 * their relative section does not depend on the selected die or its mirror.
 */
export function diagnosePairedClosure({model,catalog,options={},bendIds=null},guard=()=>{}){
 guard();
 if(!Array.isArray(model?.bends)||!Array.isArray(model?.panels)||!Array.isArray(catalog?.tools)||unsupportedScope(options))return null;
 if(options.signal?.aborted)return null;
 const filter=bendIds==null?null:new Set(bendIds.map(String));
 const bends=model.bends.filter(b=>(!filter||filter.has(String(b.id)))&&independent(model,b)),profiles=closureProfiles();
 for(let i=0;i<bends.length;i++)for(let j=i+1;j<bends.length;j++){
  guard();const a=bends[i],b=bends[j],ends=[a.parentPanelId,a.childPanelId],shared=ends.filter(id=>id===b.parentPanelId||id===b.childPanelId);
  if(shared.length!==1)continue;
  const first=closure(model,catalog,a,b,shared[0],options,guard,profiles);if(!first)continue;
  const second=closure(model,catalog,b,a,shared[0],options,guard,profiles);if(!second)continue;
  const A=a.label??a.id,B=b.label??b.id;
  return{status:'MUTUAL_CLOSURE_BLOCKED_IN_CHECKED_SCOPE',pair:[a.id,b.id],sharedPanelId:shared[0],closures:[first,second],
   reason:`Al cerrar ${A} después de ${B}, o ${B} después de ${A}, la pared opuesta invade los perfiles de punzón comprobados en una zona que debe quedar apoyada.`,
   scope:{kind:'TIP_ALIGNED_PRISMATIC_CATALOG',uniformPunchProfileRequired:true,independentPhysicalBends:true,continuousExactMaterialBands:true,
    bothPartFlips:true,bothPunchMirrors:true,partialSchedulesAlsoBlocked:true,fullManufacturingImpossibility:false},
   assumptions:['Model rigid-panel/cylindrical-hinge kinematics and catalog triangulations are authoritative.',
    'One uniform punch profile per bank, without independent X/Z offsets or mixed profiles/mounting heights.',
    'Current support policy: outer end allowance only, internal joints at most the reported gap.',
    'Different die support translates the part and registered punch tip equally; the relative section is unchanged.',
    'Other tools, displaced or rotated mounts, clamps and out-of-model forming remain unverified.']};
 }
 return null;
}
