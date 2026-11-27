import * as THREE from 'three';
import {ViewCamera} from './view-camera.mjs';
import {evaluateModel,endpointAudit} from './solver.mjs';
import {validateModel} from './model-validator.mjs';
import {buildMachineScene,partBodies,defaultSegments,toolById} from './machine.mjs';
import {assessCoverage} from './coverage.mjs';
import {assessToolSupport} from './tool-support-policy.mjs';
import {playbackSetupKey} from './playback-check.mjs';
import {sequenceSceneAt,buildSequenceTimeline,sequencePositionAt,phaseLabels} from './sequence-playback.mjs';
import {clearRejectionView,showRejectionView} from './rejection-view.mjs';
import {experimentalOutcomeText} from './sequence-portfolio.mjs';

const $=id=>document.getElementById(id), fmt=(n,d=1)=>Number(n).toLocaleString('es',{maximumFractionDigits:d});
const palette=[0xb0cad8,0x96b7cc,0xc6d7df,0xadc6d1,0x8aaebf,0xc1d1da,0xa8c6d3];
let model,catalog,activeBend,selectedSegment,segments=[],progress=0,machineMode=true,playing=false;
let order=[],setups=new Map();
let scene,camera,renderer,controls,viewCamera,grid,currentScene,reference;
let worker,workerId=0,revision=0,pending=new Map(),checking=false,lastCheck=0,job=false,motion=null;
let lastTime=0,lastGeometry=0,dirty=true,collisionIds=new Set(),importing=false;
let playbackRequestId=0,playbackApproval=null;
let operationPreview=null;
let sequencePlan=null,sequenceTimeline=null,sequencePreview=false,sequenceStage=0,plannerWorker=null,plannerReject=null,plannerRequest=0;
let frontArrow,frontLabel;
let manualSnapshot=null;
let activeImportSource=null,pendingImportSource=null;
let stationTrial=false;
const gaugeSettings=()=>({separationMm:Number($('fingerSeparation').value),centerYmm:Number($('fingerCenter').value)});
const supportSettings=()=>({maxUnsupportedEndMm:Number($('endAllowance').value)});
const meshes=new Map(),labels=new Map(),raycaster=new THREE.Raycaster(),pointer=new THREE.Vector2();
const reasonText={
  UNDER_LOAD:'La operación está bajo carga. Lleva el ciclo al inicio o al final antes de mover herramientas.',
  START_IN_COLLISION:'El segmento ya interfiere en su posición inicial.',
  END_IN_COLLISION:'El destino está ocupado.',
  NO_CLEAR_PATH:'No se encontró un recorrido sin interferencias entre las retiradas probadas.',
  LAYOUT_FOUND_BUT_MOVE_BLOCKED:'Se encontró una distribución, pero no un recorrido para alcanzarla.',
  SEARCH_BUDGET_EXHAUSTED:'La búsqueda alcanzó su límite sin encontrar una distribución comprobada.',
  NO_LAYOUT:'No se encontró una distribución con la cobertura y separación necesarias.',
  OUT_OF_BED:'El segmento quedaría fuera de la longitud de la mesa.',
  INEXACT_COVERAGE:'No se pudo determinar con certeza la cobertura de esta banda.'
};
Object.assign(reasonText,{
  START_OUTSIDE_BED:'La posición inicial del segmento está fuera de la mesa.',
  TARGET_OUTSIDE_BED:'El destino excede la longitud de la mesa.',
  SEGMENT_UNDER_LOAD:'Retira la herramienta: el segmento está en la fase de plegado o a menos de 2 mm de la pieza.',
  COVERAGE_UNVERIFIED:'No se pudo verificar qué material debe quedar cubierto.',
  MOVE_SEARCH_BUDGET_EXHAUSTED:'Se agotó la búsqueda de órdenes de movimiento; no se encontró un recorrido completo.',
  START_INTERFERENCE:'El segmento ya interfiere o tiene menos de 0.15 mm de holgura al iniciar.',
  TARGET_INTERFERENCE:'El destino interfiere o tiene menos de 0.15 mm de holgura.',
  NO_SAMPLED_ROUTE:'No se encontró un recorrido libre entre las retiradas probadas.',
  NO_CANDIDATE_LAYOUT:'No se encontró una distribución con las restricciones actuales.',
  REQUIRED_COVERAGE_OUTSIDE_STOCK_OR_BED:'Las longitudes configuradas o la mesa no permiten cubrir la banda.'
});
const config=()=>({bendId:activeBend,progress,order,punchId:$('punchSelect').value,dieId:$('dieSelect').value,
  bedLengthMm:Number($('bedLength').value),flip:$('flipPart').checked,segments:structuredClone(segments),backgauge:gaugeSettings()});
const operationKey=()=>JSON.stringify([playbackSetupKey(config()),supportSettings()]);
const currentOperationPreview=()=>operationPreview?.model===model&&operationPreview.key===operationKey()?operationPreview:null;
const bendName=id=>model?.bends.find(b=>String(b.id)===String(id))?.label??(/^\d+$/.test(String(id))?'B'+id:String(id));
const bodyName=id=>{
  if(id==='table')return 'Mesa';
  if(id==='finger-left'||id==='finger-right')return id==='finger-left'?'Finger 1':'Finger 2';
  if(id==='bed-envelope')return 'Límite de mesa';
  if(id.startsWith('panel:'))return 'Panel '+id.slice(6);
  if(id.startsWith('band:'))return 'Banda '+bendName(id.slice(5));
  if(id.startsWith('riser:'))return 'Elevador '+id.slice(6);
  const s=segments.find(s=>s.id===id);return s?(s.kind==='punch'?'Punzón ':'Matriz ')+id.split('-').at(-1):id;
};
function status(message){$('jobStatus').textContent=message;}
function fail(error){console.error(error);$('loading').hidden=true;$('failure').hidden=false;$('failure').textContent='No se pudo abrir la simulación. '+error.message;}
function stopPlay(){playing=false;$('play').textContent='▶';}
function importSourceFromJob(job){
  const info=job.embeddedStep;
  if(!job.id||!info?.present)return null;
  // Keep only the small descriptor. The encoded STEP never enters the viewer.
  return{jobId:String(job.id),name:String(job.name??''),fileName:String(info.fileName??'').slice(0,180),
    sizeBytes:Number.isSafeInteger(info.sizeBytes)&&info.sizeBytes>=0?info.sizeBytes:null,busy:false,result:null,error:null};
}
function renderEmbeddedSteps(){
  function draw(prefix,record,buttonId){
    $(prefix+'Panel').hidden=!record;if(!record)return;
    $(prefix+'Label').textContent='STEP · '+(record.sizeBytes===null?'tamaño sin declarar':record.sizeBytes>=1024?fmt(record.sizeBytes/1024,1)+' KiB':record.sizeBytes+' bytes');
    $(prefix+'Name').textContent=record.fileName;
    $(buttonId).disabled=record.busy;$(buttonId).textContent=record.busy?'Extrayendo…':'Extraer STEP';
    const link=$(prefix+'Download');link.hidden=!record.result;
    if(record.result){link.href=record.result.url;link.download=record.result.fileName;link.textContent='Descargar '+record.result.fileName;}
    else link.removeAttribute('href');
    $(prefix+'Status').textContent=record.error??(record.result?'Extraído en la carpeta de importación local. Tamaño y SHA-256 verificados.':'');
  }
  draw('embeddedStep',activeImportSource,'extractEmbeddedStep');
  draw('importEmbeddedStep',pendingImportSource,'importExtractEmbeddedStep');
}
function renderAssociation(){
  const association=model?.metadata?.stepAssociation??model?.stepAssociation;
  const panel=$('associationPanel');panel.hidden=!association;
  if(!association)return;
  const matched=association.status==='matched';
  panel.classList.toggle('warning',!matched);
  $('associationTitle').textContent=matched?'STEP + BEND asociados':association.status==='dimensions-matched'?'BEND + dimensiones STEP':'BEND · geometría nominal';
  $('associationMessage').textContent=matched?
    'Geometría CAD verificada. '+model.bends.length+' pliegues con los identificadores BEND. Los sentidos de plegado proceden del STEP.':
    String(association.message??'No se pudo asociar inequívocamente el STEP. Se conserva la reconstrucción nominal.');
}
async function extractStepFor(record){
  if(!record||record.busy)return;
  record.busy=true;record.error=null;renderEmbeddedSteps();
  try{
    const response=await fetch('/api/extract-embedded-step/'+encodeURIComponent(record.jobId),{method:'POST',signal:AbortSignal.timeout(60000)});
    const result=await response.json();
    if(!response.ok||!result.ok)throw new Error(result.error??'No se pudo extraer el STEP.');
    if(typeof result.url!=='string'||!result.url.startsWith('/.work/imports/'))throw new Error('La ruta del STEP extraído no es válida.');
    record.result={url:result.url,fileName:String(result.fileName),sizeBytes:result.sizeBytes,sha256:result.sha256};
  }catch(error){record.error=error.message;}
  finally{record.busy=false;renderEmbeddedSteps();}
}
function markChanged({preservePlaybackCheck=false}={}){
  revision++;dirty=true;collisionIds.clear();
  if(sequencePreview)return;
  if(operationPreview&&!currentOperationPreview())operationPreview=null;
  if(machineMode&&currentOperationPreview()){
    $('collisionBadge').className='collision-badge clear';
    $('collisionBadge').textContent='Operación comprobada por muestras · salida incluida';
    $('collisionDetails').replaceChildren();return;
  }
  if(preservePlaybackCheck&&playbackApproval?.model===model&&playbackApproval.key===playbackSetupKey(config()))return;
  playbackApproval=null;if(playing&&machineMode)stopPlay();
  if(machineMode){$('collisionBadge').className='collision-badge pending';$('collisionBadge').textContent='Comprobación pendiente';$('collisionDetails').replaceChildren();}
}
function makeGeometry(positions,triangles){
  const g=new THREE.BufferGeometry();g.setAttribute('position',new THREE.Float32BufferAttribute(positions,3));g.setIndex(triangles);g.computeVertexNormals();g.computeBoundingSphere();g.computeBoundingBox();return g;
}
function disposeObject(object){object.traverse(o=>{o.geometry?.dispose();if(Array.isArray(o.material))o.material.forEach(m=>m.dispose());else o.material?.dispose();});scene.remove(object);}
function clearMeshes(){for(const mesh of meshes.values())disposeObject(mesh);meshes.clear();for(const label of labels.values())label.remove();labels.clear();if(reference){disposeObject(reference);reference=null;}}
function initializeScene(){
  scene=new THREE.Scene();scene.background=null;
  renderer=new THREE.WebGLRenderer({antialias:true,alpha:true});renderer.setPixelRatio(Math.min(devicePixelRatio,2));
  renderer.outputColorSpace=THREE.SRGBColorSpace;renderer.setClearColor(0,0);$('viewport').append(renderer.domElement);
  viewCamera=new ViewCamera(renderer.domElement,{direction:[1,-1.25,.85],dampingFactor:.09,onChange:(nextCamera,nextControls,mode)=>{
    camera=nextCamera;controls=nextControls;
    for(const [id,view]of [['sideView','side-right'],['leftView','side-left'],['isoView','iso'],['topView','top'],['topOrthoView','top-ortho']])$(id).setAttribute('aria-pressed',String(mode===view));
    const fixedView={'side-right':'Lateral derecho','side-left':'Lateral izquierdo','top-ortho':'Superior · frente abajo'}[mode];
    $('cameraHint').textContent=fixedView?fixedView+' sin perspectiva · Arrastra para desplazar · Rueda para acercar · mm':'Arrastra para girar · Rueda para acercar · mm';
  }});
  scene.add(new THREE.HemisphereLight(0xd7e9ff,0x354c55,2.5));
  for(const [position,intensity]of[[[300,-400,600],3],[[-400,200,200],1.5]]){const light=new THREE.DirectionalLight(0xffffff,intensity);light.position.set(...position);scene.add(light);}
  grid=new THREE.GridHelper(2000,40,0x4b687a,0x2d4659);grid.rotation.x=Math.PI/2;grid.material.transparent=true;grid.material.opacity=.22;scene.add(grid);
  frontArrow=new THREE.ArrowHelper(new THREE.Vector3(-1,0,0),new THREE.Vector3(-40,450,-20),100,0x86f3d2,22,12);scene.add(frontArrow);
  frontLabel=document.createElement('span');frontLabel.className='label3d';frontLabel.textContent='FRENTE · operador';$('viewport').append(frontLabel);
  new ResizeObserver(()=>{const r=$('viewport').getBoundingClientRect();renderer.setSize(r.width,r.height);viewCamera.resize(r.width,r.height);}).observe($('viewport'));
  let down;
  renderer.domElement.addEventListener('pointerdown',e=>{down=[e.clientX,e.clientY];});
  renderer.domElement.addEventListener('pointerup',e=>{
    if(!down||Math.hypot(e.clientX-down[0],e.clientY-down[1])>5||job||sequencePreview)return;
    const r=renderer.domElement.getBoundingClientRect();pointer.set((e.clientX-r.left)/r.width*2-1,-(e.clientY-r.top)/r.height*2+1);raycaster.setFromCamera(pointer,camera);
    const hit=raycaster.intersectObjects([...meshes.values()])[0];if(!hit)return;
    if(hit.object.userData.segmentId)selectSegment(hit.object.userData.segmentId);
    else if(hit.object.userData.bendId!==undefined)selectBend(hit.object.userData.bendId);
  });
}
function colorFor(body,index){
  if(collisionIds.has(body.id))return 0xe77953;
  if(body.segmentId===selectedSegment)return 0x7be5d0;
  if(body.kind==='panel')return palette[index%palette.length];
  if(body.kind==='band')return String(body.bendId)===String(activeBend)?0x7de1c8:0x73b3ab;
  return {punch:0x8fabcc,die:0xba956d,table:0x526b7d,riser:0x737d87,finger:0xf1c653}[body.kind]??0xaaaaaa;
}
function paintBodies(bodies){
  const wanted=new Set(bodies.map(b=>b.id));
  for(const[id,mesh]of meshes)if(!wanted.has(id)){disposeObject(mesh);meshes.delete(id);}
  bodies.forEach((body,index)=>{
    let mesh=meshes.get(body.id);
    if(!mesh){mesh=new THREE.Mesh(makeGeometry(body.positions,body.triangles),new THREE.MeshStandardMaterial({roughness:.55,metalness:.32,side:THREE.DoubleSide}));scene.add(mesh);meshes.set(body.id,mesh);}
    else{const attr=mesh.geometry.getAttribute('position'),index=mesh.geometry.getIndex();if(attr.count!==body.positions.length/3||index.count!==body.triangles.length||body.triangles.some((v,i)=>v!==index.array[i])){mesh.geometry.dispose();mesh.geometry=makeGeometry(body.positions,body.triangles);}else{attr.array.set(body.positions);attr.needsUpdate=true;mesh.geometry.computeVertexNormals();mesh.geometry.computeBoundingSphere();mesh.geometry.computeBoundingBox();}}
    mesh.userData={...body,positions:undefined,triangles:undefined};
    mesh.material.color.setHex(colorFor(body,index));
    const isTool=!['panel','band'].includes(body.kind);mesh.material.transparent=isTool&&$('toolTransparency').checked;mesh.material.opacity=mesh.material.transparent?.28:1;mesh.material.depthWrite=!mesh.material.transparent;
    if(mesh.children[0]){const old=mesh.children[0];mesh.remove(old);old.geometry.dispose();old.material.dispose();}
    if($('edgesToggle').checked){const edge=new THREE.LineSegments(new THREE.EdgesGeometry(mesh.geometry,24),new THREE.LineBasicMaterial({color:isTool?0x172635:0x273f4a,transparent:true,opacity:isTool?.45:.65}));mesh.add(edge);}
  });
}
function createReference(){
  reference=new THREE.Group();
  const standalone=model.stepReferenceMesh;
  const surfaces=standalone?[{sourcePositions:standalone.positions,triangles:standalone.triangles}]:[...model.panels,...model.bends.map(b=>b.band)];
  for(const p of surfaces){
    if(!p.sourcePositions)continue;
    const g=makeGeometry(p.sourcePositions,p.triangles),edges=new THREE.EdgesGeometry(g,25);g.dispose();
    reference.add(new THREE.LineSegments(edges,new THREE.LineBasicMaterial({color:0xe3b16f,transparent:true,opacity:.5})));
  }
  scene.add(reference);
}
function updateGeometry(){
  if(!model)return;
  if(sequencePreview){
    const position=$('sequencePhase').value==='all'?sequencePositionAt(sequenceTimeline,progress):{stageIndex:sequenceStage,phase:$('sequencePhase').value,q:progress};
    sequenceStage=position.stageIndex;
    currentScene=sequenceSceneAt(model,catalog,sequencePlan,sequenceStage,position.phase,position.q);
    activeBend=currentScene.stage.bendId;
    const stage=currentScene.stage,angle=Math.abs(model.bends.find(b=>String(b.id)===String(stage.bendId)).signedAngle)*180/Math.PI;
    const required=stage.retraction.gapFromEndMm;
    const coordinatedRetraction=Boolean(stage.coordinatedMotion?.retract);
    $('sequenceDetail').textContent='Operación '+(sequenceStage+1)+' / '+sequencePlan.stages.length+' · '+bendName(stage.bendId)+' · '+fmt(currentScene.amounts[stage.bendId]*angle)+'°. '+
      (coordinatedRetraction?'Retracción coordinada del punzón y la pieza. ':'Retracción '+fmt(required,2)+' mm, con '+fmt(stage.retraction.additionalRetractMm??6.35,2)+' mm adicionales. ')+
      (position.phase==='retract'?(coordinatedRetraction?'La pieza acompaña al punzón según la trayectoria comprobada.':'Pieza quieta mientras sube el punzón.'):position.phase==='transfer'&&sequencePlan.transfers[sequenceStage]?.requiresManualSetupChange?'Cambio de montaje con la pieza fuera; el movimiento físico de las herramientas no está simulado.':'Cuerpo principal del lado del operador.');
    if(position.phase==='transfer'&&sequencePlan.transfers[sequenceStage]?.fingerAdjustment?.manualSetup)
      $('sequenceDetail').textContent+=' Ajustar manualmente la separación lateral de los topes mientras la pieza está fuera.';
    if(position.phase==='exit'&&stage.access.exit.openingMethod==='SAMPLED_FRONT_ROLL')
      $('sequenceDetail').textContent+=' La pieza se inclina hacia el operador para salir del cuello del punzón.';
    const cfg=stage.config,activeBank=cfg.segments.find(s=>s.bankId&&s.kind==='punch'&&Math.abs(s.centerY-cfg.stationYmm)<=s.lengthMm/2+1e-7)?.bankId;
    $('sequenceDetail').textContent+=' Punzón '+(toolById(catalog,cfg.punchId).vendorCode??cfg.punchId)+' · matriz '+(toolById(catalog,cfg.dieId).vendorCode??cfg.dieId)+'. '+
      (activeBank?'Estación activa (Y '+fmt(cfg.stationYmm,1)+' mm): ':'')+
      ['punch','die'].map(kind=>(kind==='punch'?'Punzón':'Matriz')+': '+cfg.segments.filter(s=>s.kind===kind&&(!activeBank||s.bankId===activeBank)).map(s=>fmt(s.lengthMm/25.4,2)+'″').join(' + ')).join(' · ')+'.';
    if(stage.toolSupport?.usesEndAllowance){const s=stage.toolSupport.byKind;
      $('sequenceDetail').textContent+=' Sin contacto en extremos: punzón '+fmt(s.punch.leftMm,2)+' / '+fmt(s.punch.rightMm,2)+' mm; matriz '+fmt(s.die.leftMm,2)+' / '+fmt(s.die.rightMm,2)+' mm.';}
    if(position.phase==='transfer'&&sequencePlan.stationStudy&&cfg.stationYmm!==sequencePlan.stages[sequenceStage+1]?.config.stationYmm)
      $('sequenceDetail').textContent+=' La pieza pasa por el frente del 3169 al 3924. Ambas estaciones permanecen montadas.';
    for(const button of $('sequenceSteps').children)button.classList.toggle('selected',Number(button.dataset.index)===sequenceStage);
    $('sequencePhase').querySelector('[value="transfer"]').disabled=sequenceStage===sequencePlan.stages.length-1;
  }
  else if(machineMode&&currentOperationPreview()){
    const preview=currentOperationPreview().result,position=sequencePositionAt(preview.timeline,progress);
    currentScene=sequenceSceneAt(model,catalog,preview.previewPlan,0,position.phase,position.q);
  }
  else if(machineMode&&model.bends.length)currentScene=buildMachineScene(model,catalog,config());
  else{const state=evaluateModel(model,progress,{mode:$('mode').value,order});currentScene={state,bodies:partBodies(model,state),cycle:{phase:'Plegado libre'}};}
  paintBodies(currentScene.bodies);
  $('backgaugeControls').hidden=!machineMode&&!sequencePreview;
  const gauge=currentScene.backgauge;
  if(gauge)$('backgaugeStatus').textContent=(gauge.status==='SAMPLE_CLEAR'?
    gauge.contactCount+' finger(s) alcanzan un borde · recorrido X/R revisado en '+gauge.samplesChecked+' posiciones.':
    gauge.status==='NO_REFERENCE_EDGE'?'Ningún borde alcanzable con esta separación y altura. Revisa la posición de referencia.':'Recorrido de fingers bloqueado: '+gauge.status+'.')+
    ' X geom. '+fmt(gauge.pose.lowerFaceX)+' mm · Z base '+fmt(gauge.pose.baseZ)+' mm · '+(gauge.phase==='gauge'?'posicionando':gauge.phase==='ungauge'?'retirando':'retirados')+'.';
  reference.visible=!machineMode&&!sequencePreview&&$('referenceToggle').checked;
  frontArrow.visible=machineMode||sequencePreview;frontLabel.hidden=!frontArrow.visible;
  frontArrow.position.y=(sequencePreview?currentScene.stage.config.bedLengthMm:Number($('bedLength').value))/2+35;
  const minimum=Math.min(...currentScene.bodies.filter(b=>b.kind==='table'||!machineMode).map(b=>{let z=Infinity;for(let i=2;i<b.positions.length;i+=3)z=Math.min(z,b.positions[i]);return z;}));
  grid.position.z=Number.isFinite(minimum)?minimum-1:-100;
  $('phaseBadge').textContent=machineMode||sequencePreview?currentScene.cycle.phase:'Plegado libre';
  if(machineMode&&!sequencePreview){
    const includesExit=Boolean(currentOperationPreview());
    $('modeCaption').textContent=includesExit?'Operación activa · entrada, plegado y salida':'Ciclo de la operación activa · sin salida calculada';
    $('flatButton').textContent=includesExit?'Inicio · pieza fuera':'Inicio · abierto';
    $('finalButton').textContent=includesExit?'Final · pieza fuera':'Final · punzón arriba';
  }
  if(machineMode&&!sequencePreview){
    const coverage=assessCoverage({materialIntervals:currentScene.coverage.intervals,segments,exact:currentScene.coverage.exact});
    const support=assessToolSupport(coverage,supportSettings());
    $('coverageStatus').classList.toggle('warning',!support.accepted);
    $('coverageStatus').textContent=coverage.status==='COVERED'?'Cobertura axial completa de punzón y matriz · juntas ≤ 0.5 mm.':
      support.accepted?'Extremos sin contacto dentro del margen de '+fmt(support.maxUnsupportedEndMm,2)+' mm por lado.':
      coverage.status==='UNKNOWN'?'Cobertura no verificada para esta banda.':
      'Cobertura incompleta: '+['punch','die'].filter(k=>!coverage.byKind[k].covered).map(k=>k==='punch'?'punzón':'matriz').join(' y ')+'. Reubica o añade segmentos.';
  }
  $('progress').value=Math.round(progress*1000);$('progress').style.setProperty('--progress',(progress*100)+'%');$('progressText').textContent=Math.round(progress*100)+'%';
  for(const b of model.bends){let label=labels.get(b.id);if(!label){label=document.createElement('span');label.className='label3d';label.textContent=bendName(b.id);$('viewport').append(label);labels.set(b.id,label);}label.classList.toggle('selected',String(activeBend)===String(b.id));}
  dirty=false;
}
function updateLabels(){
  if(!currentScene)return;
  const r=$('viewport').getBoundingClientRect();
  if(frontArrow.visible){const p=frontArrow.position.clone().add(new THREE.Vector3(-135,0,0)).project(camera);frontLabel.hidden=p.z>1||p.z< -1||Math.abs(p.x)>1||Math.abs(p.y)>1;frontLabel.style.left=((p.x+1)/2*r.width)+'px';frontLabel.style.top=((-p.y+1)/2*r.height)+'px';}
  for(const[id,label]of labels){
    const body=currentScene.bodies.find(b=>b.kind==='band'&&String(b.bendId)===String(id));if(!body)continue;
    const box=meshes.get(body.id).geometry.boundingBox??new THREE.Box3().setFromBufferAttribute(meshes.get(body.id).geometry.getAttribute('position'));
    const v=box.getCenter(new THREE.Vector3()).project(camera);
    label.hidden=v.z>1||v.z< -1||Math.abs(v.x)>1||Math.abs(v.y)>1;
    label.style.left=((v.x+1)/2*r.width)+'px';label.style.top=((-v.y+1)/2*r.height)+'px';
  }
}
function fit(partOnly=false,view){
  const box=new THREE.Box3();
  for(const mesh of meshes.values())if(!partOnly||['panel','band'].includes(mesh.userData.kind))box.expandByObject(mesh);
  if(!partOnly&&frontArrow.visible)box.expandByObject(frontArrow);
  if(!partOnly&&sequencePreview&&$('sequencePhase').value==='all')for(const transfer of sequencePlan.transfers??[]){
    const bounds=transfer.requiredFreeFrontSpace?.conservativeEnvelope;
    if(bounds){box.expandByPoint(new THREE.Vector3(...bounds.min));box.expandByPoint(new THREE.Vector3(...bounds.max));}
  }
  if(box.isEmpty())return;
  if(view)viewCamera.setView(view,box);else viewCamera.fit(box);
}
function fillTools(){
  for(const kind of ['punch','die']){
    const select=$(kind+'Select');select.replaceChildren();
    for(const t of catalog.tools.filter(t=>t.kind===kind)){
      const opt=document.createElement('option');opt.value=t.id;opt.textContent=(t.vendorCode?t.vendorCode+' · ':'')+t.name+(t.status==='missing'?' · sin perfil':'');opt.disabled=t.status==='missing';select.append(opt);
    }
    select.value=catalog.tools.find(t=>t.kind===kind&&t.vendorCode===(kind==='punch'?'3169':'9355')).id;
  }
}
function bendUI(){
  $('bendCount').textContent=model.bends.length+' pliegues';$('bendList').replaceChildren();
  for(const id of order){const b=model.bends.find(b=>String(b.id)===String(id)),button=document.createElement('button');button.className='bend-row'+(String(b.id)===String(activeBend)?' selected':'');button.textContent=bendName(b.id);button.title=fmt(b.signedAngle*180/Math.PI)+'° · R '+fmt(b.insideRadius)+' mm';button.onclick=()=>selectBend(b.id);button.disabled=job;$('bendList').append(button);}
  const b=model.bends.find(b=>String(b.id)===String(activeBend));
  $('selectedId').textContent=b?bendName(b.id):'Placa plana';$('selectedAngle').textContent=b?fmt(b.signedAngle*180/Math.PI)+'°':'';$('selectedRadius').textContent=b?'R '+fmt(b.insideRadius)+' mm':'Sin articulaciones';
  const index=order.findIndex(id=>String(id)===String(activeBend));
  const spans=b?.source?.materialIntervalsAxialMm;
  $('operationIndex').textContent=index<0?'Sin pliegues':'Operación '+(index+1)+' / '+order.length+
    (spans?.length>1?' · '+spans.length+' tramos en un golpe':'');
  $('earlierBend').disabled=job||index<1;$('laterBend').disabled=job||index<0||index===order.length-1;
}
function selectBend(id){
  if(job||String(id)===String(activeBend))return;
  setups.set(String(activeBend),{segments:structuredClone(segments),punchId:$('punchSelect').value,dieId:$('dieSelect').value,flip:$('flipPart').checked});
  stopPlay();activeBend=id;progress=0;
  const saved=setups.get(String(id));segments=saved?structuredClone(saved.segments):defaultSegments(model,id);
  if(saved){$('punchSelect').value=saved.punchId;$('dieSelect').value=saved.dieId;$('flipPart').checked=saved.flip;}
  selectedSegment=segments[0]?.id;bendUI();segmentsUI();markChanged();status('Montaje de '+bendName(id)+'. Los pliegues anteriores se muestran terminados.');
}
function segmentsUI(){
  $('segmentList').replaceChildren();
  for(const s of segments){
    const button=document.createElement('button');button.className='segment-row'+(s.id===selectedSegment?' selected':'');button.disabled=job;
    const dot=document.createElement('i');dot.className='kind-mark '+s.kind;button.append(dot,document.createTextNode((s.kind==='punch'?'P':'M')+s.id.split('-').at(-1)+' · '+fmt(s.lengthMm)));
    button.title=bodyName(s.id)+' · centro Y '+fmt(s.centerY)+' mm';button.onclick=()=>selectSegment(s.id);$('segmentList').append(button);
  }
  const s=segments.find(s=>s.id===selectedSegment);
  $('segmentName').textContent=s?bodyName(s.id):'Sin segmento';$('segmentPosition').textContent=s?'Y '+fmt(s.centerY)+' mm':'—';
  $('segmentLength').value=s?.lengthMm??'';$('moveTarget').value=s?Number(s.centerY.toFixed(3)):'';
  for(const id of ['segmentLength','moveTarget','moveSegment','removeSegment'])$(id).disabled=job||!s;
}
function selectSegment(id){if(job)return;selectedSegment=id;segmentsUI();dirty=true;}
function setMode(on){
  if(job)return;stopPlay();
  if(sequencePreview&&manualSnapshot){activeBend=manualSnapshot.bendId;segments=structuredClone(manualSnapshot.segments);$('punchSelect').value=manualSnapshot.punchId;$('dieSelect').value=manualSnapshot.dieId;$('flipPart').checked=manualSnapshot.flip;selectedSegment=segments[0]?.id;bendUI();segmentsUI();}
  sequencePreview=false;$('sequenceMode').classList.remove('active');$('sequenceControls').hidden=true;machineMode=on&&model.bends.length>0;progress=0;
  for(const selector of ['.inspector-title','.bend-detail','.sequence-tools'])document.querySelectorAll('.inspector>'+selector).forEach(e=>e.hidden=false);$('bendList').hidden=false;
  $('machineMode').classList.toggle('active',machineMode);$('freeMode').classList.toggle('active',!machineMode);$('machineControls').hidden=!machineMode;$('freeControls').hidden=machineMode;
  $('sceneTitle').textContent=machineMode?'Pieza y prensa.':'De plano a pieza.';
  $('stageSubtitle').textContent=machineMode?'Perfiles DXF reales · segmentos independientes':'Paneles rígidos · bandas con radio finito';
  $('modeCaption').textContent=machineMode?'Ciclo de la operación activa':'Secuencia ilustrativa de todos los pliegues';
  $('flatButton').textContent=machineMode?'Inicio · abierto':'Plano · 0%';$('finalButton').textContent=machineMode?'Final · retirado':'Pieza final · 100%';
  $('collisionBadge').hidden=!machineMode;markChanged();updateGeometry();fit();
}
async function loadModel(data,name,importSource=null){
  clearRejectionView();
  validateModel(data);cancelWork(false);sequencePlan=null;sequenceTimeline=null;sequencePreview=false;manualSnapshot=null;$('sequenceMode').disabled=true;$('downloadSequence').disabled=true;
  $('sequenceBlockers').hidden=true;$('sequenceBlockerDetails').replaceChildren();
  $('sequenceStatus').textContent='Secuencia pendiente de calcular para esta pieza.';model=data;order=model.bends.map(b=>b.id);setups=new Map();activeBend=model.bends[0]?.id;selectedSegment=undefined;progress=0;
  activeImportSource=importSource;renderEmbeddedSteps();renderAssociation();
  stationTrial=false;
  $('sm1StationLink').hidden=!data.metadata?.sourceSha256?.['sm1).step']||data.metadata.sourceSha256['sm1).step']!=='db32ceed6589d1cc56d821d05526a0f0ad59e5b11266cda47fdb1b00eb6773b4';
  // The historical five-bend report is not bundled with this independent copy.
  $('partialReviewLink').hidden=true;
  segments=model.bends.length?defaultSegments(model,activeBend):[];selectedSegment=segments[0]?.id;
  clearMeshes();createReference();$('partName').textContent=name??data.metadata?.partName??data.partName??'Chapa · modelo geométrico';
  bendUI();segmentsUI();$('machineMode').disabled=!model.bends.length;setMode(model.bends.length>0);
  const audit=endpointAudit(model),hasRef=audit.referenceStatus==='REFERENCED';
  const surfaceCheck=model.metadata?.foldedStepCheck;
  const sampled=surfaceCheck?.status==='sampled'&&Number.isFinite(surfaceCheck.maxMm);
  const warning=!hasRef||audit.maxDeviationMm>.0001;$('validation').classList.toggle('warning',warning);
  $('validationTitle').textContent=hasRef||sampled?'Comparación con el STEP final':model.metadata?.sourceKind==='bcbend'?'BCBEND · geometría nominal reconstruida':'Modelo sin referencia final';
  const warnings=model.metadata?.warnings??model.importWarnings??model.warnings??model.provenance?.warnings??[];
  const warningGroups=new Map();for(const w of warnings){const message=typeof w==='string'?w:w.message??w.code,ids=warningGroups.get(message)??[];if(w.bendId!==undefined)ids.push(bendName(w.bendId));warningGroups.set(message,ids);}
  const warningText=[...warningGroups].map(([message,ids])=>(ids.length?ids.join(', ')+': ':'')+message).join(' ');
  const referenceNote=audit.invalidReferencePartCount?'Referencias finales inválidas: '+audit.invalidReferencePartCount+' regiones. ':
    audit.referenceStatus==='PARTIAL_REFERENCE'?'Referencia final parcial: '+audit.referencedVertexCount+' de '+audit.vertexCount+' vértices; no valida la pieza completa. ':'';
  const comparison=referenceNote+(sampled?'Desviación máxima muestreada en ambos sentidos '+fmt(surfaceCheck.maxMm,3)+' mm. ':
    hasRef?'Desviación máxima '+fmt(audit.maxDeviationMm,6)+' mm. ':surfaceCheck?'Comparación de superficies pendiente. ':'');
  $('validationText').textContent=comparison+model.panels.length+' paneles · espesor '+fmt(model.thicknessMm)+' mm.'+(warningText?' '+warningText:'');
  $('referenceToggle').disabled=!(hasRef||model.stepReferenceMesh);$('referenceToggle').checked=false;$('planSequence').disabled=!model.bends.length;
  $('loading').hidden=true;$('failure').hidden=true;$('play').disabled=false;$('progress').disabled=false;
  await resetWorker();status('Selecciona un segmento para comprobar su desplazamiento.');
}
function rpc(type,payload,onProgress){
  const id=++workerId;return new Promise((resolve,reject)=>{pending.set(id,{resolve,reject,onProgress});worker.postMessage({id,type,payload});});
}
async function resetWorker(){
  worker?.terminate();for(const p of pending.values())p.reject(new Error('Cálculo cancelado.'));pending.clear();checking=false;
  worker=new Worker('./machine-worker.mjs',{type:'module'});
  worker.onmessage=event=>{const {id,type,result,error}=event.data,p=pending.get(id);if(!p)return;if(type==='progress'){p.onProgress?.(result);return;}pending.delete(id);error?p.reject(new Error(error)):p.resolve(result);};
  worker.onerror=event=>{for(const p of pending.values())p.reject(new Error(event.message||'Error de cálculo.'));pending.clear();};
  if(model)await rpc('init',{model,catalog});
}
function showCollisions(result,sample=false){
  collisionIds=new Set(sample?[]:(result.pairs??[]).flatMap(p=>p.bodyIds??[]));dirty=true;
  const hit=result.status==='INTERFERENCE';$('collisionBadge').className='collision-badge '+(hit?'hit':'clear');
  $('collisionBadge').textContent=hit?(sample?'Interferencia / holgura en el ciclo':'Interferencia / holgura < 0.15 mm'):sample?'17 posiciones sin interferencia':'Sin interferencia en esta posición';
  $('collisionDetails').replaceChildren();
  for(const pair of (result.pairs??[]).slice(0,12)){
    const button=document.createElement('button');button.textContent=pair.bodyIds.map(bodyName).join(' ↔ ')+(pair.reason==='BED_LIMIT'?' · fuera de mesa':' · revisar contacto / holgura');
    button.onclick=()=>{
      if(sample){const first=result.samples.find(s=>s.pairs.some(ids=>ids.join('|')===pair.bodyIds.join('|')));if(first){stopPlay();progress=first.progress;markChanged();updateGeometry();}}
      const box=new THREE.Box3();for(const id of pair.bodyIds){const mesh=meshes.get(id);if(mesh)box.expandByObject(mesh);}if(!box.isEmpty()){controls.target.copy(box.getCenter(new THREE.Vector3()));controls.update();}
    };$('collisionDetails').append(button);
  }
}
async function checkPose(){
  if(checking||job||sequencePreview||currentOperationPreview()||!machineMode||!worker||!model)return;
  checking=true;lastCheck=performance.now();const at=revision;
  try{const result=await rpc('check',{config:config()});if(at===revision)showCollisions(result);}
  catch(error){if(at===revision)status(error.message);}finally{checking=false;}
}
function setBusy(on){
  job=on;stopPlay();$('cancelJob').hidden=!on;
  for(const id of ['openImport','machineMode','freeMode','punchSelect','dieSelect','bedLength','flipPart','resetSegments','addPunch','addDie','optimize','scan','progress','play','flatButton','finalButton','planSequence','partialBends','experimentalFallback','searchBudget','sequencePhase','fingerSeparation','fingerCenter','viewFingers','endAllowance'])$(id).disabled=on;
  $('sequenceMode').disabled=on||!sequencePlan?.stages.length||!sequencePlan.completeGeometricRouteSampleClear;
  $('downloadSequence').disabled=on||!sequencePlan;
  $('planSequence').disabled=on||!model?.bends.length;
  if(!model?.bends.length)$('machineMode').disabled=true;
  bendUI();segmentsUI();
}
function cancelWork(restart=true){
  plannerRequest++;plannerWorker?.terminate();plannerWorker=null;if(plannerReject){plannerReject(new Error('Búsqueda cancelada.'));plannerReject=null;}
  playbackRequestId++;playbackApproval=null;operationPreview=null;
  stopPlay();if(motion){motion.resolve(false);motion=null;}worker?.terminate();for(const p of pending.values())p.reject(new Error('Cálculo cancelado.'));pending.clear();checking=false;
  if(model){setBusy(false);markChanged();}if(restart&&model)resetWorker().catch(error=>status(error.message));
}
const explain=result=>reasonText[result.reason]??('No se completó: '+(result.reason??result.status)+'.');
async function requestPlayback(){
  if(job||!model)return;
  if(playing){stopPlay();markChanged();return;}
  if(sequencePreview||!machineMode){if(progress>=1)progress=0;playing=true;$('play').textContent='Ⅱ';return;}
  if(currentOperationPreview()){
    if(progress>=1)progress=0;playing=true;$('play').textContent='Ⅱ';return;
  }
  const requestId=++playbackRequestId,sourceModel=model,snapshot=structuredClone(config()),key=operationKey();
  const isCurrent=()=>requestId===playbackRequestId&&sourceModel===model&&machineMode&&!sequencePreview&&key===operationKey();
  let startPlayback=false;
  markChanged();setBusy(true);
  try{
    await resetWorker();if(!isCurrent())return;
    status('Comprobando entrada, plegado, subida del punzón y salida de esta operación…');
    const result=await rpc('operation-preview',{config:snapshot,options:{...supportSettings(),timeBudgetMs:30000}},p=>{
      if(isCurrent())status(p.message??'Comprobando recorrido de la operación y topes…');
    });
    if(!isCurrent())return;
    if(result.status!=='OPERATION_SAMPLE_CLEAR'){
      status('No se anima una salida sin comprobar: '+(result.message??explain(result))+' Puedes examinar posiciones con el control de progreso.');
      return;
    }
    operationPreview={model:sourceModel,key,result};progress=0;markChanged();updateGeometry();fit();
    status('Operación '+bendName(activeBend)+' comprobada por muestras: el punzón sube '+fmt(result.stage.retraction.gapFromEndMm,2)+' mm y después la pieza sale al frente. Incluye topes; no valida la secuencia completa de la pieza.');
    startPlayback=true;
  }catch(error){if(requestId===playbackRequestId)status('No se inicia la reproducción: '+error.message);}
  finally{if(requestId===playbackRequestId){setBusy(false);if(startPlayback&&isCurrent()){playing=true;$('play').textContent='Ⅱ';}}}
}

function animatePaths(paths){
  if(!paths.length)return Promise.resolve(true);
  const legs=[];
  for(const {segmentId,path}of paths)for(let i=1;i<path.keyframes.length;i++){
    const a=path.keyframes[i-1].pose,b=path.keyframes[i].pose,d=Math.hypot((b.centerY??0)-(a.centerY??0),(b.offsetZ??0)-(a.offsetZ??0));
    if(d>1e-7)legs.push({segmentId,a,b,phase:path.keyframes[i].phase,duration:Math.max(.35,d/100)});
  }
  if(!legs.length)return Promise.resolve(true);
  return new Promise(resolve=>{motion={legs,index:0,elapsed:0,resolve};});
}
function showSequence(stageIndex=0){
  if(job||!sequencePlan?.stages.length||!sequencePlan.completeGeometricRouteSampleClear)return;
  stopPlay();if(!sequencePreview)manualSnapshot=structuredClone(config());sequencePreview=true;machineMode=true;sequenceStage=stageIndex;progress=0;revision++;collisionIds.clear();
  $('machineMode').classList.remove('active');$('freeMode').classList.remove('active');$('sequenceMode').classList.add('active');
  $('machineControls').hidden=true;$('freeControls').hidden=true;$('sequenceControls').hidden=false;$('bendList').hidden=true;
  for(const selector of ['.inspector-title','.bend-detail','.sequence-tools'])document.querySelectorAll('.inspector>'+selector).forEach(e=>e.hidden=true);
  $('sceneTitle').textContent='Secuencia de esta pieza.';$('stageSubtitle').textContent='Acceso frontal · retracción calculada · herramientas presentes';
  $('modeCaption').textContent=$('sequencePhase').value==='all'?'Secuencia completa':'Fase de la operación seleccionada';
  $('flatButton').textContent='Inicio';$('finalButton').textContent='Final';$('collisionBadge').hidden=false;$('collisionBadge').className='collision-badge clear';
  $('collisionBadge').textContent='Plegados y accesos comprobados por muestras';dirty=true;updateGeometry();fit();
}
async function runSequenceSearch(){
  if(job||!model?.bends.length)return;
  clearRejectionView();
  if(sequencePreview)setMode(true);stopPlay();setBusy(true);
  const token=++plannerRequest,sourceModel=model,budgetMs=Number($('searchBudget').value);
  let expandedAttempt=false;
  const options={bedLengthMm:Number($('bedLength').value),timeBudgetMs:budgetMs,
    maxStates:Math.max(24,Math.ceil(budgetMs/1500)),maxTransitions:Math.max(150,Math.ceil(budgetMs/200)),
    allowPartialBends:$('partialBends').checked,...supportSettings(),backgauge:gaugeSettings(),
    additionalRetractMm:6.35,experimentalFallback:$('experimentalFallback').checked};
  $('sequenceStatus').textContent='Buscando órdenes, montajes y recorridos…';status('Calculando una secuencia para la pieza cargada. Puedes cancelar la búsqueda.');
  $('sequenceBlockers').hidden=true;$('sequenceBlockerDetails').replaceChildren();
  try{
    plannerWorker?.terminate();plannerWorker=new Worker('./sequence-worker.mjs',{type:'module'});
    const result=await new Promise((resolve,reject)=>{
      plannerReject=reject;
      plannerWorker.onmessage=event=>{const message=event.data;if(message.id!==token)return;
        if(message.type==='progress'){
          const p=message.result;
          if(p.phase==='experimental-fallback'){
            expandedAttempt=true;
            $('sequenceStatus').textContent='EXPERIMENTAL · La búsqueda normal no obtuvo una ruta comprobada. Probando más alternativas con hasta '+fmt(budgetMs/1000)+' segundos adicionales de búsqueda…';return;
          }
          if(p.phase==='experimental-orientations'){
            expandedAttempt=true;
            $('sequenceStatus').textContent='EXPERIMENTAL V2 · Las búsquedas anteriores no encontraron ruta. Probando orientaciones de perfiles y más alternativas con hasta '+fmt(budgetMs/1000)+' segundos adicionales…';return;
          }
          if(p.phase==='experimental-advanced'){
            expandedAttempt=true;
            $('sequenceStatus').textContent='EXPERIMENTAL V4 · Buscando prepliegues, montajes por zonas y movimientos coordinados con hasta '+fmt(p.timeBudgetMs/1000)+' segundos adicionales…';return;
          }
          const detail=p.phase==='candidate-found'?'Ruta de '+p.depth+' operaciones encontrada; comparando alternativas…':
            p.phase==='checking-final-setups'?'Comprobando si alguna herramienta permite terminar el último pliegue…':
            ['checking-backgauge','advanced-backgauge'].includes(p.phase)?'Comprobando posicionamiento, retirada y ajustes de los topes…':(p.bendId!==undefined?'Revisando '+bendName(p.bendId):'Comparando secuencias')+' · '+(p.transitionsChecked??p.transitions??0)+' comprobaciones…';
          $('sequenceStatus').textContent=(expandedAttempt?'EXPERIMENTAL · Búsqueda ampliada · ':'')+detail;return;
        }
        plannerReject=null;message.error?reject(new Error(message.error)):resolve(message.result);
      };
      plannerWorker.onerror=event=>{plannerReject=null;reject(new Error(event.message||'No se pudo completar el cálculo.'));};
      plannerWorker.postMessage({id:token,payload:{model,catalog,options,stationTrial}});
    });
    if(token!==plannerRequest||model!==sourceModel)return;
    sequencePlan=result;sequenceTimeline=result.stages?.length&&result.transfersVerified?buildSequenceTimeline(result):null;
    $('sequenceSteps').replaceChildren();
    for(const [i,s]of (result.stages??[]).entries()){
      const b=model.bends.find(b=>String(b.id)===String(s.bendId)),angle=Math.abs(b.signedAngle)*180/Math.PI;
      const button=document.createElement('button');button.className='bend-row';button.dataset.index=i;button.textContent=(i+1)+'. '+bendName(s.bendId)+' · '+fmt(s.from[s.bendId]*angle)+'° → '+fmt(s.to[s.bendId]*angle)+'°';
      button.onclick=()=>{if(job)return;$('sequencePhase').value='fold';showSequence(i);};$('sequenceSteps').append(button);
    }
    const complete=result.completeGeometricRouteSampleClear===true;
    const cost=result.cost?.toolChangeEvents??result.cost?.cost?.toolChangeEvents??result.toolChangesAfterInitialSetup;
    $('sequenceStatus').textContent=complete?'Secuencia encontrada: '+result.stages.length+' operaciones'+(cost!==undefined?' · '+cost+(cost===1?' cambio':' cambios')+' de herramientas':'')+'. Retracción calculada por operación. '+
      (result.usesEndAllowance?'Extremos sin contacto dentro del margen de '+fmt(result.toolSupportPolicy.maxUnsupportedEndMm,2)+' mm por lado. ':'')+
      (result.toolChangeMotionVerified?'':'Los cambios físicos de montaje requieren preparación manual con la pieza fuera. ')+
      (result.diagnostics?.stoppedAtPrimaryCostLowerBound?'Se reutiliza el montaje en toda la secuencia.':result.diagnostics?.searchComplete===true?'':'Mejor alternativa encontrada en esta búsqueda.'):
      'No se obtuvo una secuencia completa comprobada dentro de esta búsqueda. '+(result.transferError??result.reason??result.diagnostics?.error??result.diagnostics?.reason??'Puedes ampliar el tiempo o revisar las herramientas. Esto no demuestra que la pieza sea imposible.');
    if(result.stationStudy){
      const study=result.stationStudy;
      $('sequenceStatus').textContent=(complete?result.stages.length+' operaciones · '+cost+' cambios de herramienta. ':$('sequenceStatus').textContent+' ')+
        '3169 y 3924 de 1″, a '+fmt(study.spacingMm/25.4,2)+'″ entre centros. Dos matrices 5623 de 1″. Inventario registrado: '+study.dieCountInCatalog+'.';
    }
    if(result.status==='NO_FINAL_TOOL_SETUP_IN_CANDIDATES'){
      const check=result.diagnostics.finalSetupCheck;
      $('sequenceStatus').textContent='Bloqueo de herramientas detectado en '+fmt(result.diagnostics.elapsedMs/1000,2)+' s. '+
        'Ningún montaje probado permite terminar el último pliegue: la pieza interfiere o no queda al frente. Más tiempo con estos montajes no elimina el bloqueo.';
      $('sequenceBlockers').hidden=false;$('sequenceBlockers').open=true;
      const addLine=text=>{const p=document.createElement('p');p.textContent=text;$('sequenceBlockerDetails').append(p);};
      const punches=[...new Set(result.diagnostics.candidates.generatedByBend.flatMap(b=>b.retainedPunchIds))];
      addLine(check.checkedPoses+' posiciones finales · punzones '+punches.map(id=>toolById(catalog,id).vendorCode??id).join(', ')+'.');
      for(const row of check.byBend){
        // Prefer a single bank witness so its stock length is unambiguous.
        const blocked=row.blocked.find(b=>b.config.setupFamily!=='TWO_BANK')??row.blocked[0];
        if(!blocked){addLine(bendName(row.bendId)+': ninguna orientación probada mantiene el cuerpo principal al frente.');continue;}
        const cfg=blocked.config,punch=toolById(catalog,cfg.punchId),stock=cfg.segments.filter(s=>s.kind==='punch').reduce((n,s)=>n+s.lengthMm,0);
        addLine(bendName(row.bendId)+' · pliegue '+fmt(row.materialSpanMm,2)+' mm. Punzón '+(punch.vendorCode??punch.id)+
          (cfg.setupFamily==='TWO_BANK'?'':': banco de '+fmt(stock,2)+' mm ('+fmt(stock/25.4,2)+'″)')+'. Choque: '+
          blocked.collision.bodyIds.map(id=>id.startsWith('panel:')?'Panel '+id.slice(6):id.startsWith('band:')?'Banda '+bendName(id.slice(5)):id).join(' ↔ ')+'.');
      }
      addLine('Son ejemplos de los choques encontrados con el margen de extremos configurado. Otros largos, perfiles o montajes requieren otra comprobación; este resultado no declara imposible fabricar la pieza.');
    }
    const comparison=result.experimentalComparison;
    if(comparison){
      const outcome=experimentalOutcomeText(result);
      $('sequenceStatus').textContent='EXPERIMENTAL · '+outcome+' '+$('sequenceStatus').textContent;
    }
    $('downloadSequence').disabled=false;
    if(!complete)showRejectionView(model,catalog,result,bendName);
    status(complete?'La secuencia está disponible en la pestaña «Secuencia».':'Revisa el resultado de la búsqueda; no se ha marcado una ruta incompleta como libre.');
  }catch(error){if(token===plannerRequest){$('sequenceStatus').textContent=error.message;status(error.message);}}
  finally{if(token===plannerRequest){plannerWorker?.terminate();plannerWorker=null;setBusy(false);if(sequencePlan?.completeGeometricRouteSampleClear){$('sequencePhase').value='all';showSequence(0);}}}
}
async function runJob(type,extra={}){
  if(job||!model?.bends.length)return;
  operationPreview=null;
  stopPlay();setBusy(true);const snapshot=config();
  try{
    // Discard pending pose checks so a long search never queues behind animation frames.
    await resetWorker();status(type==='scan'?'Revisando el ciclo en 17 posiciones…':type==='layout'?'Buscando distribución y recorridos…':'Comprobando retirada, desplazamiento y aproximación…');
    const result=await rpc(type,{config:snapshot,...extra},p=>status(p.stage==='layout'?'Distribuciones evaluadas: '+p.candidates+'…':p.stage==='motion'?'Probando orden de movimientos: '+p.attempts+' / '+p.maxAttempts+'…':'Comprobando posición '+p.done+' de '+p.total+'…'));
    if(type==='scan'){
      showCollisions(result,true);status(result.status==='INTERFERENCE'?'Hay interferencias en el ciclo. Selecciona un par para localizarlo.':'Las 17 posiciones evaluadas no presentan interferencias. No es una comprobación continua.');
    }else if(type==='move'&&result.status==='SAMPLE_CLEAR'){
      status('Recorrido comprobado por muestreo. Moviendo segmento…');const done=await animatePaths([{segmentId:extra.segmentId,path:result}]);if(done)status('Segmento colocado. Se revisaron '+result.samples.length+' posiciones del recorrido.');
    }else if(type==='layout'&&result.status==='LAYOUT_FOUND'&&result.trajectoryChecked){
      status('Distribución encontrada. Ejecutando recorridos comprobados…');const done=await animatePaths(result.motionPaths);
      if(done)status('Distribución colocada; cobertura y 3 estados de plegado comprobados. Usa «Revisar ciclo» para ampliar el muestreo.');
    }else status(explain(result));
  }catch(error){status(error.message);}finally{setBusy(false);if(type!=='scan')markChanged();}
}
function addSegment(kind){
  if(job)return;stopPlay();progress=0;
  const same=segments.filter(s=>s.kind===kind),lengthMm=25.4,edge=same.length?Math.max(...same.map(s=>s.centerY+s.lengthMm/2))+.4:-lengthMm/2;
  let n=1;while(segments.some(s=>s.id===kind+'-'+n))n++;
  const s={id:kind+'-'+n,kind,lengthMm,centerY:edge+lengthMm/2,offsetX:0,offsetZ:0};segments.push(s);selectedSegment=s.id;segmentsUI();markChanged();status('Segmento añadido al montaje inicial. Comprueba su posición y cobertura.');
}
function connectUI(){
  const labelButton=$('toggleLabels'),stage=labelButton.closest('.stage');
  const labelPreference='unfold.viewer.labelsHidden';
  function setLabelsHidden(hidden){
    stage.classList.toggle('labels-hidden',hidden);
    labelButton.textContent=hidden?'Mostrar etiquetas':'Ocultar etiquetas';
  }
  try{setLabelsHidden(localStorage.getItem(labelPreference)==='true');}catch{setLabelsHidden(false);}
  labelButton.onclick=()=>{
    const hidden=!stage.classList.contains('labels-hidden');
    setLabelsHidden(hidden);
    try{localStorage.setItem(labelPreference,String(hidden));}catch{/* The toggle also works without storage. */}
  };
  $('endAllowance').onchange=()=>{
    clearRejectionView();
    if(!$('endAllowance').checkValidity()||!Number.isFinite(Number($('endAllowance').value)))$('endAllowance').value=12.7;
    if(sequencePreview)setMode(true);stopPlay();sequencePlan=null;sequenceTimeline=null;$('sequenceMode').disabled=true;$('downloadSequence').disabled=true;
    $('sequenceBlockers').hidden=true;$('sequenceStatus').textContent='Margen de extremos actualizado. Vuelve a calcular la secuencia.';markChanged();
  };
  for(const id of ['fingerSeparation','fingerCenter'])$(id).onchange=()=>{
    clearRejectionView();$('sequenceBlockers').hidden=true;
    if(!$(id).checkValidity()||!Number.isFinite(Number($(id).value))){$(id).value=id==='fingerSeparation'?100:0;}
    if(sequencePreview)setMode(true);stopPlay();sequencePlan=null;sequenceTimeline=null;$('sequenceMode').disabled=true;$('downloadSequence').disabled=true;
    $('sequenceStatus').textContent='Posición de fingers cambiada. Vuelve a calcular la secuencia.';progress=.09;markChanged();
  };
  $('viewFingers').onclick=()=>{
    if(job)return;stopPlay();
    operationPreview=null;
    if(sequencePreview){$('sequencePhase').value='gauge';progress=1;}else{if(!machineMode)setMode(true);progress=.09;}
    markChanged();updateGeometry();
    const box=new THREE.Box3();for(const mesh of meshes.values())if(['finger','panel','band','die'].includes(mesh.userData.kind))box.expandByObject(mesh);
    if(!box.isEmpty())viewCamera.fit(box);
  };
  $('fitView').onclick=()=>fit();$('partView').onclick=()=>fit(true);
  $('topView').onclick=()=>fit(false,'top');
  $('topOrthoView').onclick=()=>fit(false,'top-ortho');
  $('sideView').onclick=()=>fit(false,'side-right');$('leftView').onclick=()=>fit(false,'side-left');$('isoView').onclick=()=>fit(false,'iso');
  $('machineMode').onclick=()=>setMode(true);$('freeMode').onclick=()=>setMode(false);
  $('sequenceMode').onclick=()=>{$('sequencePhase').value='all';showSequence(0);};
  $('planSequence').onclick=runSequenceSearch;
  $('sequencePhase').onchange=()=>{if(sequencePreview){progress=0;stopPlay();$('modeCaption').textContent=$('sequencePhase').value==='all'?'Secuencia completa':'Fase de la operación seleccionada';markChanged();updateGeometry();fit();}};
  $('downloadSequence').onclick=()=>{if(!sequencePlan)return;const url=URL.createObjectURL(new Blob([JSON.stringify({schemaVersion:1,units:'mm',modelSource:model.metadata??model.provenance??null,...sequencePlan},null,2)],{type:'application/json'}));const a=document.createElement('a');a.href=url;a.download='secuencia-'+(model.metadata?.partName??model.partName??'pieza').replace(/[^\p{L}\p{N}_-]+/gu,'-')+'.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);};
  $('play').onclick=()=>requestPlayback();
  $('progress').oninput=()=>{stopPlay();progress=Number($('progress').value)/1000;markChanged();};
  $('flatButton').onclick=()=>{stopPlay();progress=0;markChanged();};$('finalButton').onclick=()=>{stopPlay();progress=1;markChanged();};
  for(const id of ['edgesToggle','toolTransparency','referenceToggle'])$(id).onchange=()=>{dirty=true;};
  $('mode').onchange=()=>markChanged();
  for(const[id,direction]of [['earlierBend',-1],['laterBend',1]])$(id).onclick=()=>{
    if(job)return;const i=order.findIndex(id=>String(id)===String(activeBend)),j=i+direction;if(j<0||j>=order.length)return;
    [order[i],order[j]]=[order[j],order[i]];stopPlay();progress=0;bendUI();markChanged();status('Orden actualizado. Revisa la pieza y las herramientas con los pliegues previos en su nueva posición.');
  };
  for(const id of ['punchSelect','dieSelect','flipPart'])$(id).onchange=()=>{stopPlay();progress=0;markChanged();status('Montaje actualizado. Revisa el ciclo con las herramientas elegidas.');};
  $('bedLength').onchange=()=>{if(!$('bedLength').checkValidity()||!Number.isFinite(Number($('bedLength').value))){$('bedLength').value=838.2;}markChanged();};
  $('resetSegments').onclick=()=>{stopPlay();progress=0;segments=defaultSegments(model,activeBend);selectedSegment=segments[0].id;segmentsUI();markChanged();};
  $('segmentLength').onchange=()=>{
    const s=segments.find(s=>s.id===selectedSegment),length=Number($('segmentLength').value),tool=toolById(catalog,$(s.kind+'Select').value);
    if(!tool.availableLengthsMm.some(l=>Math.abs(l-length)<.01)){status('Longitudes del catálogo: '+tool.availableLengthsMm.map(l=>fmt(l)).join(', ')+' mm.');segmentsUI();return;}
    stopPlay();progress=0;s.lengthMm=length;segmentsUI();markChanged();status('Longitud del montaje actualizada; vuelve a comprobar cobertura e interferencias.');
  };
  $('moveSegment').onclick=()=>{
    const s=segments.find(s=>s.id===selectedSegment),y=Number($('moveTarget').value);
    if(!Number.isFinite(y)||$('moveTarget').value===''){status('Introduce una posición Y válida.');return;}
    runJob('move',{segmentId:s.id,to:{...s,centerY:y}});
  };
  $('removeSegment').onclick=()=>{stopPlay();progress=0;segments=segments.filter(s=>s.id!==selectedSegment);selectedSegment=segments[0]?.id;segmentsUI();markChanged();status('Segmento retirado del montaje inicial. Revisa la cobertura antes de plegar.');};
  $('addPunch').onclick=()=>addSegment('punch');$('addDie').onclick=()=>addSegment('die');
  $('optimize').onclick=()=>runJob('layout');$('scan').onclick=()=>runJob('scan');
  $('cancelJob').onclick=()=>{cancelWork();status('Operación cancelada. La escena conserva la posición alcanzada.');};
  $('openImport').onclick=async()=>{
    pendingImportSource=null;renderEmbeddedSteps();$('importStatus').textContent='El archivo se procesa localmente en este workspace.';
    $('importDialog').showModal();
    try{const response=await fetch('/api/examples');if(!response.ok)throw new Error('Reinicia el servidor para habilitar la importación.');
      const examples=await response.json();$('exampleSelect').replaceChildren();for(const x of examples){const o=document.createElement('option');o.value=x.url;o.textContent=x.name;$('exampleSelect').append(o);}
    }catch(error){$('importStatus').textContent=error.message;}
  };
  $('loadExample').onclick=async()=>{
    if(importing)return;try{const url=$('exampleSelect').value;if(!url)return;const data=await fetch(url).then(r=>r.json());await loadModel(data,$('exampleSelect').selectedOptions[0].textContent);$('importDialog').close();}
    catch(error){$('importStatus').textContent=error.message;}
  };
  $('extractEmbeddedStep').onclick=()=>extractStepFor(activeImportSource);
  $('importExtractEmbeddedStep').onclick=()=>extractStepFor(pendingImportSource);
  $('fileInput').onchange=async()=>{
    const file=$('fileInput').files[0];if(!file||importing)return;importing=true;$('loadExample').disabled=true;$('fileInput').disabled=true;
    pendingImportSource=null;renderEmbeddedSteps();
    try{
      if(file.size>30*1024*1024)throw new Error('El límite de archivo es 30 MB.');
      let data;
      const isBcbend=/\.(bcbend|bend)$/i.test(file.name),isJson=/\.json$/i.test(file.name);
      {
        $('importStatus').textContent=(isBcbend||isJson)&&$('useEmbeddedStep').checked?'Leyendo la pieza y comparando el STEP embebido, si existe…':'Reconociendo espesor, paneles y bandas…';
        // Send the original Blob once. Parsing and attachment handling stay on the server.
        const response=await fetch(isBcbend?'/api/import-bcbend':isJson?'/api/import-json':'/api/import-step',{method:'POST',headers:{'Content-Type':isBcbend||isJson?'application/json':'application/step','X-Bend-Sign-Convention':$('signConvention').value,'X-Embedded-Step-Mode':$('useEmbeddedStep').checked?'auto':'ignore','X-K-Factor':$('kFactor').value,'X-Root-Face':$('rootFace').value,'X-File-Name':encodeURIComponent(file.name)},body:file});
        const started=await response.json();if(!response.ok)throw new Error(started.error??'No se pudo iniciar la importación.');
        pendingImportSource=importSourceFromJob(started);renderEmbeddedSteps();
        const deadline=performance.now()+270000;
        for(;;){
          if(performance.now()>deadline)throw new Error('La importación no respondió dentro del tiempo permitido. Puedes volver a cargar el archivo.');
          await new Promise(resolve=>setTimeout(resolve,1000));
          const poll=await fetch('/api/import-status/'+started.id,{signal:AbortSignal.timeout(15000)}),state=await poll.json();
          if(!poll.ok||state.status==='failed'){
            const association=state.errorDetails?.stepAssociation;
            throw new Error([association?.message,state.error??'No se pudo consultar la importación.'].filter(Boolean).join(' '));
          }
          if(state.status==='done'){
            const prepared=await fetch(state.modelUrl,{signal:AbortSignal.timeout(15000)});
            if(!prepared.ok)throw new Error('No se pudo cargar la geometría importada.');
            data=await prepared.json();break;
          }
          if(!['queued','running'].includes(state.status))throw new Error('El servidor devolvió un estado de importación desconocido.');
        }
      }
      await loadModel(data,file.name,pendingImportSource);$('importDialog').close();
    }catch(error){$('importStatus').textContent=error.message;}
    finally{importing=false;$('loadExample').disabled=false;$('fileInput').disabled=false;$('fileInput').value='';}
  };
}
function tick(now){
  requestAnimationFrame(tick);const dt=Math.min(.1,(now-lastTime)/1000||0);lastTime=now;
  if(motion){
    const m=motion,leg=m.legs[m.index];m.elapsed+=dt;const q=Math.min(1,m.elapsed/leg.duration),s=segments.find(s=>s.id===leg.segmentId);
    for(const key of ['centerY','offsetZ','offsetX'])s[key]=(leg.a[key]??0)+((leg.b[key]??0)-(leg.a[key]??0))*q;
    selectedSegment=s.id;dirty=true;revision++;$('phaseBadge').textContent={RETRACT:'Retirada de segmento',SLIDE:'Desplazamiento',APPROACH:'Aproximación de segmento'}[leg.phase]??'Movimiento';
    if(q===1){Object.assign(s,leg.b);m.index++;m.elapsed=0;if(m.index===m.legs.length){motion=null;segmentsUI();m.resolve(true);}}
  }else if(playing){progress=Math.min(1,progress+dt/Number($('duration').value));markChanged({preservePlaybackCheck:machineMode});if(progress===1)stopPlay();}
  if(dirty&&now-lastGeometry>33){try{updateGeometry();}catch(error){stopPlay();status(error.message);}lastGeometry=now;}
  controls?.update();if(camera){updateLabels();renderer.render(scene,camera);}
  if(machineMode&&!job&&!playing&&now-lastCheck>900&&$('collisionBadge').classList.contains('pending'))checkPose();
}
try{
  initializeScene();connectUI();requestAnimationFrame(tick);
  const query=new URLSearchParams(location.search),example=query.get('ejemplo');
  if(example&&!/^[a-zA-Z0-9_-]+$/.test(example))throw new Error('Nombre de ejemplo inválido.');
  const modelPath=example?'./examples/'+example+'.json':'./model.json';
  const [data,toolData,fingerData]=await Promise.all([fetch(modelPath).then(r=>{if(!r.ok)throw new Error('No se encontró la pieza seleccionada.');return r.json();}),fetch('./tooling.json').then(r=>r.json()),fetch('./fingers.json').then(r=>r.json())]);catalog={...toolData,fingerDefinition:fingerData.definitions[0]};fillTools();await loadModel(data,example?(data.partName??data.metadata?.partName??example):'Complex · STEP auditado');
  stationTrial=example==='sm1-step'&&query.get('montaje')==='dos-estaciones'&&!$('sm1StationLink').hidden;
  if(stationTrial)$('sm1StationLink').hidden=true;
  if(query.has('extremo')&&query.get('extremo').trim()!==''){
    const field=$('endAllowance'),old=field.value;field.value=query.get('extremo');
    if(field.value===''||!field.checkValidity())field.value=old;
  }
  if([...$('searchBudget').options].some(option=>option.value===query.get('tiempo')))
    $('searchBudget').value=query.get('tiempo');
  if(query.get('calcular')==='secuencia')await runSequenceSearch();
  else if(query.get('reproducir')==='operacion'){setMode(true);progress=0;markChanged();updateGeometry();fit();await requestPlayback();}
  else if(example){setMode(false);progress=1;markChanged();updateGeometry();fit();}
}catch(error){fail(error);}
