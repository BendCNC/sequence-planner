import {movesWithRam} from './tool-geometry.mjs';
import {isPart,sceneWithAmounts} from './sequence-scene.mjs';
import {transformFrontTiltPoint} from './sequence-rigid-pose.mjs';
import {buildFingerBodies} from './finger-geometry.mjs';

const descriptions={
  NO_FINAL_TOOL_SETUP_IN_CANDIDATES:'Ningún montaje evaluado permitió colocar la pieza terminada para el último pliegue.',
  FIXED_STATIONS_BLOCKED:'La secuencia probada quedó bloqueada con estas estaciones fijas.',
  NO_SEQUENCE_IN_SEARCHED_CANDIDATES:'La búsqueda terminó sin encontrar una secuencia completa entre los montajes explorados.',
  SEARCH_BUDGET_EXHAUSTED:'La búsqueda alcanzó su límite antes de encontrar una secuencia completa. Pueden quedar alternativas por evaluar.',
  FLAT_BLANK_SELF_CONTACT:'La pieza plana aparece en contacto consigo misma: es un falso contacto del modelo o de las excepciones de colisión, no un límite de fabricación.',
  FINAL_SHAPE_SELF_CONTACT:'La forma final diseñada aparece en contacto consigo misma: es un falso contacto del modelo o de las excepciones de colisión, no un límite de fabricación.',
  NO_COMPATIBLE_TOOL_CANDIDATES:'Al menos un pliegue quedó sin un montaje compatible con las herramientas y la cobertura disponibles.',
  DIAGNOSTIC_UNDERCOVERED:'La longitud de apoyo de las herramientas queda fuera del margen permitido.',
  BACKGAUGE_REVIEW_REQUIRED:'El backgauge no pudo completar el posicionamiento en la ruta calculada.'
};
const causes={
  MAIN_BRANCH_NOT_FRONT:'El cuerpo principal queda del lado de la máquina.',
  INSUFFICIENT_TOOL_SUPPORT:'El apoyo de la línea de pliegue queda fuera del margen configurado.',
  NO_REFERENCE_EDGE:'Ningún finger alcanza un borde de referencia.',
  INSUFFICIENT_RETRACTION:'La posición de retirada del backgauge deja poco recorrido.',
  NO_FRONT_ROUTE_IN_TESTED_LIFTS:'Ninguna de las elevaciones probadas permitió el acceso frontal.',
  GAUGE_SAMPLE_BUDGET:'Se alcanzó el límite de muestras del backgauge.',
  ACCESS_SEARCH_BUDGET:'Se alcanzó el límite de muestras del acceso frontal.'
};
const usable=w=>w?.collision?.bodyIds?.length===2&&w.collision.point?.length===3&&w.collision.point.every(Number.isFinite);
const unique=items=>[...new Set(items)];

export function rejectionReport(model,result,bendName=String){
  if(!result||result.completeGeometricRouteSampleClear||result.alternativePlans?.some(a=>a.plan?.completeGeometricRouteSampleClear)||
     ['CANCELLED','FLAT_NO_BENDS'].includes(result.status))return null;
  const diag=result.diagnostics??{},notes=[],cases=[];
  if(result.status==='NO_FINAL_TOOL_SETUP_IN_CANDIDATES'){
    const amounts=Object.fromEntries(model.bends.map(b=>[b.id,1]));
    for(const row of diag.finalSetupCheck.byBend){
      const sample=row.blocked.find(b=>b.config.setupFamily!=='TWO_BANK'&&usable(b))??row.blocked.find(usable);
      if(sample)cases.push({...sample,bendId:row.bendId,phase:'final',amounts});
      else if(row.frontRejected&&row.frontRejected===row.checkedPoses)notes.push(bendName(row.bendId)+' · '+causes.MAIN_BRANCH_NOT_FRONT);
    }
    notes.push(diag.finalSetupCheck.checkedPoses+' posiciones finales evaluadas.');
  }else if(result.status==='BACKGAUGE_REVIEW_REQUIRED'){
    for(const [i,s] of result.stages.entries()){
      const g=s.backgauge;if(!g||g.status==='SAMPLE_CLEAR')continue;
      if(usable(g)&&g.blockedPose)cases.push({bendId:s.bendId,prefix:result.stages.slice(0,i).map(b=>b.bendId),phase:'gauge',
        amounts:s.from,config:s.access.entry.config,collision:g.collision,fingers:{...g.blockedPose,stationY:g.stationY}});
      else notes.push(bendName(s.bendId)+' · '+(causes[g.status]??g.status));
    }
  }else if(result.status==='DIAGNOSTIC_UNDERCOVERED'){
    for(const s of result.stages??[])if(!s.toolSupport?.accepted)notes.push(bendName(s.bendId)+' · '+causes.INSUFFICIENT_TOOL_SUPPORT);
  }else if(result.transferError)notes.push(result.transferError);
  else{
    const samples=(diag.collisionWitnesses??[]).filter(usable).sort((a,b)=>(b.prefix?.length??0)-(a.prefix?.length??0));
    // Put distinct motion phases side by side before repeating the same kind of rejection.
    const first=unique(samples.map(s=>s.phase)).map(phase=>samples.find(s=>s.phase===phase));
    cases.push(...first,...samples.filter(s=>!first.includes(s)));
    if(!cases.length){
      if(result.blocked&&causes[result.blocked.reason])notes.push(causes[result.blocked.reason]);
      for(const reason of Object.keys(diag.rejections??{}))if(causes[reason])notes.push(causes[reason]);
      for(const row of diag.candidates?.generatedByBend??[])if(row.candidates===0)notes.push(bendName(row.bendId)+' · Sin montaje compatible.');
    }
  }
  return{message:descriptions[result.status]??result.reason??diag.error??result.transferError??'La búsqueda terminó sin una secuencia comprobada.',
    notes:unique(notes),cases};
}

// A slice through a segment end may coincide with a whole triangulated face.
// Keep its perimeter, remove internal diagonals, and merge edges shared with the sides.
export function sectionContours(body,y){
  const eps=1e-7,edges=new Map(),points=new Map(),neighbors=new Map();
  const key=p=>Math.round(p[0]/eps)+','+Math.round(p[1]/eps);
  function edge(a,b,coplanar){
    const ka=key(a),kb=key(b);if(ka===kb)return;
    const id=ka<kb?ka+'|'+kb:kb+'|'+ka;
    const record=edges.get(id)??{ka,kb,flat:0,cross:false};
    if(coplanar)record.flat++;else record.cross=true;
    edges.set(id,record);points.set(ka,a);points.set(kb,b);
  }
  for(let i=0;i<body.triangles.length;i+=3){
    const verts=[0,1,2].map(j=>{const k=3*body.triangles[i+j];return [body.positions[k],body.positions[k+1]-y,body.positions[k+2]];});
    if(verts.every(p=>p[1]>eps)||verts.every(p=>p[1]<-eps))continue;
    const flat=verts.every(p=>Math.abs(p[1])<=eps),cuts=new Map();
    for(let j=0;j<3;j++){
      const a=verts[j],b=verts[(j+1)%3],p=[a[0],a[2]];
      if(flat){edge(p,[b[0],b[2]],true);continue;}
      if(Math.abs(a[1])<=eps)cuts.set(key(p),p);
      if((a[1]>eps&&b[1]<-eps)||(a[1]<-eps&&b[1]>eps)){
        const t=a[1]/(a[1]-b[1]),q=[a[0]+t*(b[0]-a[0]),a[2]+t*(b[2]-a[2])];cuts.set(key(q),q);
      }
    }
    if(cuts.size===2)edge(...cuts.values(),false);
  }
  for(const e of edges.values())if(e.cross||e.flat%2===1){
    for(const [a,b] of [[e.ka,e.kb],[e.kb,e.ka]]){
      if(!neighbors.has(a))neighbors.set(a,new Set());neighbors.get(a).add(b);
    }
  }
  const contours=[];
  function walk(start){
    let current=start;const line=[points.get(start)];
    do{
      const next=neighbors.get(current).values().next().value;
      neighbors.get(current).delete(next);neighbors.get(next).delete(current);
      current=next;line.push(points.get(next));
    }while(current!==start&&neighbors.get(current).size===1);
    contours.push({points:line,closed:current===start});
  }
  for(const [id,adj] of neighbors)if(adj.size!==2)while(adj.size)walk(id);
  for(const [id,adj] of neighbors)while(adj.size)walk(id);
  return contours;
}

export function sectionForWitness(model,catalog,w){
  let {bodies}=sceneWithAmounts(model,catalog,w.config,w.amounts);
  if(w.fixedOnly)bodies=bodies.filter(b=>!movesWithRam(b));
  if(w.fingers)bodies.push(...buildFingerBodies({definition:catalog.fingerDefinition,...w.fingers}));
  if(w.partPose)bodies=bodies.map(b=>{
    if(!isPart(b))return b;
    const positions=new Float64Array(b.positions.length);
    for(let i=0;i<positions.length;i+=3)positions.set(transformFrontTiltPoint(b.positions.slice(i,i+3),w.partPose,w.pivot),i);
    return{...b,positions};
  });
  const hit=w.collision,point=hit.point,distance=hit.witnessedSurfaceDistanceMm??hit.surfaceDistanceMm;
  const kind=hit.reason==='CONTAINMENT'?'inside':Number.isFinite(distance)&&distance<=1e-7?'contact':'margin';
  return{point,kind,distance,margin:w.clearanceMm??hit.clearanceMm??hit.diagnostics?.clearanceMm,
    bodies:bodies.map(b=>({id:b.id,kind:b.kind,toolId:b.toolId,lengthMm:b.lengthMm,contours:sectionContours(b,point[1])})),
    pair:hit.bodyIds,closestPoints:hit.closestPoints};
}
