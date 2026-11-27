/** Checked playback of one selected operation. It never certifies the whole part. */
import {validateModel} from './model-validator.mjs';
import {bandPoint} from './solver.mjs';
import {defaultSegments,operationAmounts} from './machine.mjs';
import {formingSegmentSeated} from './tool-setup-search.mjs';
import {sceneWithAmounts,isPart,bodyBounds} from './sequence-scene.mjs';
import {evaluateSequenceTransition} from './sequence-transition.mjs';
import {planBackgauge,parkingPose} from './backgauge.mjs';
import {buildFingerBodies} from './finger-geometry.mjs';
import {transformFrontTiltPoint} from './sequence-rigid-pose.mjs';
import {buildSequenceTimeline,sequencePositionAt,sequenceSceneAt} from './sequence-playback.mjs';

const defaults={clearanceMm:.15,contactGapMm:.3,maxUnsupportedEndMm:12.7,additionalRetractMm:6.35,
 foldSamples:21,foldStepDegrees:5,accessPointStepMm:2,ramStepMm:2,maxAccessSamples:2500,frontMarginMm:10,timeBudgetMs:120000};
const scope='Only the selected operation in the supplied order. Earlier bends stay finished; later bends stay flat. Not a complete sequence or manufacturing verification.';
const baseResult=()=>({completeGeometricRouteSampleClear:false,completeManufacturingVerified:false,motionApplied:false,scope});
function normalize(model){return{...model,bends:model.bends.map(b=>b.band.flatPositions?b:{...b,band:{...b.band,
 flatPositions:b.band.parameters.flatMap((v,i,a)=>i%3?[]:bandPoint(b,v,a[i+1],a[i+2],0))}})};}

function validateSetup(model,catalog,config){
 if(!Number.isFinite(config.bedLengthMm)||config.bedLengthMm<=0)throw new Error('Invalid bed length.');
 const active=model.bends.find(b=>String(b.id)===String(config.bendId)),included=180-Math.abs(active.signedAngle)*180/Math.PI;
 for(const kind of ['punch','die']){
  const tool=catalog.tools.find(t=>String(t.id)===String(config[kind+'Id']));
  if(!tool||tool.kind!==kind||tool.status!=='ready')throw new Error('Missing ready '+kind+' profile.');
  if(Number.isFinite(tool.contact?.openingAngleDegrees)&&tool.contact.openingAngleDegrees>included+1e-6||
      kind==='punch'&&Number.isFinite(tool.contact?.tipRadiusMm)&&tool.contact.tipRadiusMm>active.insideRadius+1e-6)
    throw new Error('Selected '+kind+' fails the nominal bend angle/radius constraints.');
  if(!config.segments.some(s=>s.kind===kind))throw new Error('Missing '+kind+' stock.');
 }
 const ids=new Set(),needed=new Map();
 for(const segment of config.segments){
  if(!segment.id||ids.has(segment.id)||segment.id==='table'||/^(panel|band|riser):/.test(segment.id))throw new Error('Invalid or duplicate segment ID.');
  ids.add(segment.id);
  if(!['punch','die'].includes(segment.kind)||!Number.isFinite(segment.lengthMm)||segment.lengthMm<=0||!Number.isFinite(segment.centerY))throw new Error('Invalid stock segment.');
  // An overhanging section must still rest mostly on the bed (16+8+4+16 in bank).
  if(!formingSegmentSeated(segment,config.bedLengthMm))throw new Error('Stock exceeds the bed limits.');
  if([segment.offsetX??0,segment.offsetZ??0].some(v=>!Number.isFinite(v)||Math.abs(v)>1e-8))throw new Error('Forming stock must be seated at zero X/Z offsets.');
  const toolId=segment.kind==='punch'?(segment.punchId??config.punchId):config.dieId;
  const tool=catalog.tools.find(t=>String(t.id)===String(toolId));
  if(!tool||tool.status!=='ready'||tool.kind!==segment.kind)throw new Error('Missing mounted stock profile.');
  const key=String(toolId)+'|'+segment.lengthMm.toFixed(6),count=(needed.get(key)??0)+1;needed.set(key,count);
  const stock=tool.inventory?.length?tool.inventory:(tool.availableLengthsMm??tool.stockLengthsMm??[]).map(lengthMm=>({lengthMm,count:1}));
  const available=stock.reduce((sum,s)=>sum+(Math.abs(s.lengthMm-segment.lengthMm)<1e-5&&Number.isInteger(s.count)&&s.count>0?s.count:0),0);
  if(count>available)throw new Error('Requested segment length/count is not available in the selected profile inventory.');
 }
}

/** Exact X bounds at the already checked fold/access samples; ram travel is Z only. */
function sampledOperationEnvelope(model,catalog,stage,guard){
 let maximumX=-Infinity,sampledScenes=0;
 const include=scene=>{sampledScenes++;for(const body of scene.bodies)maximumX=Math.max(maximumX,bodyBounds(body).max[0]);};
 for(const sample of stage.foldSamples){guard();include(sceneWithAmounts(model,catalog,stage.config,sample.amounts));}
 for(const [access,amounts]of [[stage.access.entry,stage.from],[stage.access.exit,stage.to]]){
  const scene=sceneWithAmounts(model,catalog,access.config,amounts);include(scene);
  for(const sample of access.path.samples){guard();sampledScenes++;
   for(const body of scene.bodies.filter(isPart))for(let i=0;i<body.positions.length;i+=3)
    maximumX=Math.max(maximumX,transformFrontTiltPoint(body.positions.slice(i,i+3),sample.pose,access.path.pivot)[0]);
  }
 }
 return{maximumX,sampledScenes};
}

export async function checkOperationPreview({model:inputModel,catalog,config:inputConfig,options:inputOptions={}},onProgress=()=>{}){
 const started=performance.now();let from,to,config;
 const blocked=(reason,details={})=>({...baseResult(),status:'BLOCKED',operationSampleClear:false,reason,from,to,config,...details,elapsedMs:performance.now()-started});
 try{
  validateModel(inputModel);const model=normalize(inputModel);
  config=structuredClone(inputConfig??{});config.bendId??=model.bends[0]?.id;
  const order=config.order??model.bends.map(b=>b.id);
  from=operationAmounts(model,config.bendId,0,order);to=operationAmounts(model,config.bendId,1,order);
  config.bedLengthMm??=838.2;config.stationYmm??=0;config.flip=Boolean(config.flip);
  config.segments??=defaultSegments(model,config.bendId);validateSetup(model,catalog,config);
  const options={...defaults,...inputOptions,bedLengthMm:config.bedLengthMm,allowPartialCoverage:false,
    backgauge:inputOptions.backgauge??config.backgauge};
  for(const key of ['clearanceMm','contactGapMm','maxUnsupportedEndMm','additionalRetractMm','frontMarginMm'])
    if(!Number.isFinite(options[key])||options[key]<0)throw new Error('Invalid '+key+'.');
  for(const key of ['foldStepDegrees','accessPointStepMm','ramStepMm','timeBudgetMs'])
    if(!Number.isFinite(options[key])||options[key]<=0)throw new Error('Invalid '+key+'.');
  for(const key of ['foldSamples','maxAccessSamples'])if(!Number.isInteger(options[key])||options[key]<2)throw new Error('Invalid '+key+'.');
  const guard=()=>{if(options.signal?.aborted)throw new Error('OPERATION_CANCELLED');if(performance.now()-started>options.timeBudgetMs)throw new Error('OPERATION_TIME_BUDGET');};
  onProgress({phase:'checking-operation',bendId:config.bendId});await new Promise(resolve=>setTimeout(resolve,0));guard();
  const checked=evaluateSequenceTransition({model,catalog,from,to,bendId:config.bendId,config,operationId:'operation-preview:'+config.bendId,options},guard);
  if(!checked.feasible)return blocked(checked.reason,{...checked,operationSampleClear:false});
  const stage=checked.stage;let parkedFingerCheck=null;
  if(options.backgauge){
   if(!catalog.fingerDefinition)return blocked('MISSING_FINGER_DEFINITION');
   onProgress({phase:'checking-operation-backgauge',bendId:config.bendId});await new Promise(resolve=>setTimeout(resolve,0));guard();
   const envelope=sampledOperationEnvelope(model,catalog,stage,guard);
   const park=parkingPose([{positions:new Float64Array([envelope.maximumX,0,0])}]);
   const entry=sceneWithAmounts(model,catalog,stage.access.entry.config,stage.from);
   const settings={fixedStations:true,...options.backgauge};
   stage.backgauge=planBackgauge({scene:entry,definition:catalog.fingerDefinition,settings,park,stepMm:options.accessPointStepMm,clearanceMm:options.clearanceMm});
   guard();if(stage.backgauge.status!=='SAMPLE_CLEAR')return blocked('BACKGAUGE_REVIEW_REQUIRED',{backgauge:stage.backgauge});
   const fingers=buildFingerBodies({definition:catalog.fingerDefinition,...park,stationY:stage.backgauge.stationY});
   const minimumFingerX=Math.min(...fingers.map(f=>bodyBounds(f).min[0])),separationMm=minimumFingerX-envelope.maximumX;
   parkedFingerCheck={status:separationMm>options.clearanceMm?'SAMPLE_CLEAR':'BLOCKED',method:'X_SEPARATION_OF_SAMPLED_OPERATION_ENVELOPE',
    ...envelope,minimumFingerX,separationMm,clearanceMm:options.clearanceMm,includes:['entry','fold','ram-approach/retract','exit'],
    ramArgument:'Pure Z ram motion leaves every tool X bound unchanged.',continuousCollisionDetection:false};
   if(parkedFingerCheck.status!=='SAMPLE_CLEAR')return blocked('PARKED_FINGER_INTERFERENCE',{parkedFingerCheck});
  }
  const previewPlan={...baseResult(),stages:[stage],transfers:[],operationPreviewOnly:true};
  const result={...baseResult(),status:'OPERATION_SAMPLE_CLEAR',operationSampleClear:true,from,to,config:stage.config,stage,previewPlan,
    timeline:buildSequenceTimeline(previewPlan),backgaugeSampleClear:options.backgauge?true:null,parkedFingerCheck,
    checkedOptions:{clearanceMm:options.clearanceMm,contactGapMm:options.contactGapMm,maxUnsupportedEndMm:options.maxUnsupportedEndMm,
      additionalRetractMm:options.additionalRetractMm},elapsedMs:performance.now()-started};
  onProgress({phase:'operation-preview-ready',bendId:config.bendId});return result;
 }catch(error){return blocked(error.message==='OPERATION_CANCELLED'?'OPERATION_CANCELLED':error.message==='OPERATION_TIME_BUDGET'?'OPERATION_TIME_BUDGET':'INVALID_OPERATION_INPUT',{error:error.message});}
}

export function operationPreviewSceneAt(model,catalog,preview,progress){
 if(preview?.status!=='OPERATION_SAMPLE_CLEAR'||preview.operationSampleClear!==true||!preview.previewPlan)throw new Error('Only a verified single-operation preview can be played.');
 if(!Number.isFinite(progress))throw new Error('Invalid operation preview progress.');
 const position=sequencePositionAt(preview.timeline??buildSequenceTimeline(preview.previewPlan),progress);
 if(!position)throw new Error('Missing operation preview phases.');
 return sequenceSceneAt(model,catalog,preview.previewPlan,position.stageIndex,position.phase,position.q);
}
