import {buildMachineScene,collisionPairs,defaultSegments} from './machine.mjs';
import {meshCollider,checkPair} from './collision.mjs';
import {planSegmentMove,solveSegmentLayout} from './segment-planner.mjs';
import {assessCoverage} from './coverage.mjs';
import {checkScene} from './sequence-scene.mjs';
import {checkOperationPreview} from './operation-preview.mjs';

const copy=value=>structuredClone(value);
const CLEARANCE_MM=.15;
const unchanged=(a,b)=>Math.abs((a??0)-(b??0))<1e-8;
const blocked=(reason,extra={})=>({status:'BLOCKED',reason,keyframes:[],samples:[],motionApplied:false,...extra});
const engaged=cycle=>cycle.phase==='Plegado'||cycle.gap<2;

/** Bounded search of sequential motions. A failed proposal never returns executable partial paths. */
export function planSequentialMoves(config,placements,planMove,{maxAttempts=32,onProgress=()=>{}}={}){
  const original=copy(config),targets=placements.filter(p=>{
    const current=config.segments.find(s=>s.id===p.id);
    if(!current)throw new Error(`Segmento objetivo desconocido: ${p.id}`);
    return !unchanged(current.centerY,p.centerY);
  });
  let attempts=0,longestPartial=0,budgetExhausted=false;
  const failedStates=new Set();
  function search(working,pending,paths){
    longestPartial=Math.max(longestPartial,paths.length);
    if(!pending.length)return{paths,segments:copy(working.segments)};
    const state=pending.map(p=>p.id).sort().join('|');
    if(failedStates.has(state))return null;
    for(let i=0;i<pending.length;i++){
      if(attempts>=maxAttempts){budgetExhausted=true;return null;}
      const target=pending[i],from=working.segments.find(s=>s.id===target.id),to={...from,centerY:target.centerY};
      attempts++;onProgress({stage:'motion',attempts,maxAttempts,segmentId:target.id});
      const path=planMove(working,target.id,to);
      if(path.status!=='SAMPLE_CLEAR')continue;
      const next=copy(working);Object.assign(next.segments.find(s=>s.id===target.id),to);
      const result=search(next,pending.filter((_,j)=>j!==i),[...paths,{segmentId:target.id,path}]);
      if(result)return result;
      if(budgetExhausted)return null;
    }
    failedStates.add(state);return null;
  }
  const result=search(original,targets,[]);
  return result?{status:'SAMPLE_CLEAR',motionPaths:result.paths,targetSegments:result.segments,
    trajectoryChecked:true,motionApplied:false,attempts,continuousCollisionDetection:false}:
    {status:'BLOCKED',reason:budgetExhausted?'MOVE_SEARCH_BUDGET_EXHAUSTED':'LAYOUT_FOUND_BUT_MOVE_BLOCKED',
      motionPaths:[],targetSegments:copy(config.segments),trajectoryChecked:false,motionApplied:false,
      attempts,longestPartialPathCount:longestPartial,continuousCollisionDetection:false};
}

/** Pure request handler shared by browser Worker and Node integration tests. */
export function createMachineWorkerRuntime(){
  let model,catalog;
  function normalizedConfig(input){
    if(!model||!catalog)throw new Error('Inicializa el modelo y las herramientas antes de consultar.');
    const config=copy(input??{}),ids=model.bends.map(b=>String(b.id));
    config.bendId=config.bendId??model.bends[0].id;
    if(!ids.includes(String(config.bendId)))throw new Error('Pliegue activo desconocido.');
    if(config.order&&(config.order.length!==ids.length||new Set(config.order.map(String)).size!==ids.length||config.order.some(id=>!ids.includes(String(id)))))
      throw new Error('El orden debe contener todos los pliegues exactamente una vez.');
    config.bedLengthMm=config.bedLengthMm??838.2;
    if(!Number.isFinite(config.bedLengthMm)||config.bedLengthMm<=0)throw new Error('Longitud de bancada inválida.');
    for(const key of ['progress','amount'])if(config[key]!==undefined&&(!Number.isFinite(config[key])||config[key]<0||config[key]>1))
      throw new Error(`Valor ${key} fuera del intervalo 0..1.`);
    if(config.ramGapMm!==undefined&&(!Number.isFinite(config.ramGapMm)||config.ramGapMm<0))throw new Error('Retirada del ram inválida.');
    config.segments=config.segments??defaultSegments(model,config.bendId);
    const segmentIds=new Set();
    for(const s of config.segments){
      if(!s.id||segmentIds.has(s.id)||s.id==='table'||/^(panel|band|riser):/.test(s.id))throw new Error('Identificador de segmento duplicado o reservado.');
      segmentIds.add(s.id);
      if(!['punch','die'].includes(s.kind)||!Number.isFinite(s.lengthMm)||s.lengthMm<=0||!Number.isFinite(s.centerY))throw new Error('Segmento con tipo o dimensiones inválidos.');
      for(const key of ['offsetX','offsetZ']){s[key]=s[key]??0;if(!Number.isFinite(s[key]))throw new Error('Desplazamiento de segmento inválido.');}
    }
    return config;
  }
  function bedViolation(segment,config){
    const min=segment.centerY-segment.lengthMm/2,max=segment.centerY+segment.lengthMm/2;
    return min< -config.bedLengthMm/2-1e-7||max>config.bedLengthMm/2+1e-7;
  }
  function checkBodies(scene,config,{onlyIds=null,stopAtFirst=false}={}){
    const pairs=[];
    let minimum=Infinity;
    for(const segment of config.segments)if((!onlyIds||onlyIds.has(segment.id))&&bedViolation(segment,config))
      pairs.push({status:'INTERFERENCE',reason:'BED_LIMIT',bodyIds:[segment.id,'bed-envelope'],distanceMm:0,distanceIsExact:false,point:null});
    if(!pairs.length||!stopAtFirst){
      const result=checkScene(model,scene,{clearanceMm:CLEARANCE_MM,firstOnly:stopAtFirst,onlyIds});
      minimum=result.distanceMm;pairs.push(...result.hits);
    }
    return{status:pairs.length?'INTERFERENCE':'CLEAR',pairs,distanceMm:pairs.length?0:minimum,
      distanceIsExact:false,distanceBound:'LOWER',continuousCollisionDetection:false,
      scope:'Pieza-herramientas, herramientas, todos los paneles/bandas y límites Y de bancada. Solo se exceptúan uniones locales de chapa verificadas y contactos de montaje. No incluye fuerza, abrazaderas, límites Z ni certificación de inventario.'};
  }
  const checkConfig=(config,options)=>checkBodies(buildMachineScene(model,catalog,config),config,options);

  function moveOne(config,segmentId,requestedTo){
    const from=config.segments.find(s=>s.id===segmentId);
    if(!from)throw new Error('Segmento no encontrado.');
    const to={...from,...requestedTo};
    for(const key of Object.keys({...from,...to}))if(!['centerY','offsetZ'].includes(key)&&JSON.stringify(from[key])!==JSON.stringify(to[key]))
      throw new Error(`El movimiento solo admite centerY/offsetZ; no puede cambiar ${key}.`);
    if(!Number.isFinite(to.centerY)||!Number.isFinite(to.offsetZ))throw new Error('Destino no finito.');
    if(bedViolation(from,config))return blocked('START_OUTSIDE_BED');
    if(bedViolation(to,config))return blocked('TARGET_OUTSIDE_BED');
    const base=buildMachineScene(model,catalog,config),movingIds=new Set([segmentId,`riser:${segmentId}`]);
    if((config.underLoad===true||engaged(base.cycle))&&(!unchanged(from.centerY,to.centerY)||!unchanged(from.offsetZ,to.offsetZ)))
      return blocked('SEGMENT_UNDER_LOAD',{loadAssessment:'Conservative engagement inference from cycle/gap; no force measurement.'});
    const pairs=collisionPairs(model,base.bodies),staticColliders=base.bodies.map(meshCollider);
    function evaluatePose(pose){
      if(bedViolation(pose,config))return{status:'INTERFERENCE',distanceMm:0,reason:'BED_LIMIT'};
      const colliders=staticColliders.slice();
      for(let i=0;i<base.bodies.length;i++)if(movingIds.has(base.bodies[i].id)){
        const body=base.bodies[i],positions=Float64Array.from(body.positions);
        for(let j=0;j<positions.length;j+=3){positions[j+1]+=pose.centerY-from.centerY;positions[j+2]+=pose.offsetZ-from.offsetZ;}
        colliders[i]=meshCollider({...body,positions});
      }
      let distance=Infinity;
      for(const[i,j]of pairs){
        if(!movingIds.has(base.bodies[i].id)&&!movingIds.has(base.bodies[j].id))continue;
        const result=checkPair(colliders[i],colliders[j],{clearanceMm:CLEARANCE_MM,queryMode:'clearance'});
        distance=Math.min(distance,result.distanceMm);
        if(result.status==='INTERFERENCE')return{status:'INTERFERENCE',distanceMm:0,pair:result.bodyIds};
      }
      return{status:'CLEAR',distanceMm:distance,distanceIsExact:false,distanceBound:'LOWER'};
    }
    const result=planSegmentMove({from,to,evaluatePose,retractAxis:'offsetZ',
      retractDistancesMm:[0,20,60,120,200],maxStepMm:2,minStepMm:.25,maxSamples:3500,clearanceMm:CLEARANCE_MM,
      underLoad:config.underLoad===true||engaged(base.cycle),distanceIsConservative:true});
    return{...result,motionApplied:false,segmentId,
      scope:'One segment and its attached riser translated against fixed scene geometry. Other static pairs are not revalidated by this path.',
      mechanicalLimits:'Y bed envelope only. Z withdrawal, clamps and machine access are not represented.'};
  }

  function handle({type,payload},onProgress=()=>{}){
    if(type==='init'){model=payload.model;catalog=payload.catalog;return{ready:true};}
    const config=normalizedConfig(payload.config);
    if(type==='operation-preview')return checkOperationPreview({model,catalog,config,options:payload.options},onProgress);
    if(type==='check')return checkConfig(config);
    if(type==='scan'){
      const samples=[],allPairs=new Map();
      // Explicit amount overrides progress in machine.mjs; remove it for a true cycle scan.
      const cycleConfig={...config};delete cycleConfig.amount;delete cycleConfig.ramGapMm;
      const backgauge=buildMachineScene(model,catalog,{...cycleConfig,progress:0}).backgauge;
      if(backgauge&&backgauge.status!=='SAMPLE_CLEAR')throw new Error('Backgauge sin recorrido comprobado: '+backgauge.status);
      for(let i=0;i<=16;i++){
        const q=i/16,scene=buildMachineScene(model,catalog,{...cycleConfig,progress:q});
        const result=checkBodies(scene,cycleConfig);
        samples.push({progress:q,amount:scene.cycle.amount,ramGapMm:scene.cycle.gap,status:result.status,pairs:result.pairs.map(p=>p.bodyIds)});
        for(const pair of result.pairs)allPairs.set(pair.bodyIds.join('|'),pair);
        onProgress({done:i+1,total:17});
      }
      return{status:allPairs.size?'INTERFERENCE':'SAMPLE_CLEAR',samples,pairs:[...allPairs.values()],
        backgauge:backgauge?{status:backgauge.status,samplesChecked:backgauge.samplesChecked,contactCount:backgauge.contactCount,calibrated:false}:null,
        maxProgressStep:1/16,continuousCollisionDetection:false};
    }
    if(type==='move')return moveOne(config,payload.segmentId,payload.to);
    if(type==='layout'){
      const base=buildMachineScene(model,catalog,config),checkpointAmounts=[0,.5,1];
      if(!['punch','die'].every(kind=>config.segments.some(s=>s.kind===kind)))return blocked('REQUIRED_COVERAGE_OUTSIDE_STOCK_OR_BED',{placements:[],motionPaths:[],trajectoryChecked:false});
      if(!base.coverage.exact)return blocked('COVERAGE_UNVERIFIED',{placements:[],motionPaths:[],trajectoryChecked:false,coverageExact:false});
      if(config.underLoad===true||engaged(base.cycle))return blocked('SEGMENT_UNDER_LOAD',{placements:[],motionPaths:[],trajectoryChecked:false,coverageExact:true});
      let candidates=0;
      const evaluateLayout=placements=>{
        candidates++;onProgress({stage:'layout',candidates});
        const candidate=config.segments.map(s=>({...s,...placements.find(p=>p.id===s.id)}));
        const coverage=assessCoverage({materialIntervals:base.coverage.intervals,segments:candidate,
          exact:base.coverage.exact,jointGapToleranceMm:.5});
        if(coverage.status!=='COVERED')return{status:'INTERFERENCE',reason:'INCOMPLETE_COVERAGE',coverage};
        for(const amount of checkpointAmounts){
          const result=checkConfig({...config,segments:candidate,amount,ramGapMm:amount===0?50:0},{stopAtFirst:true});
          if(result.status==='INTERFERENCE')return result;
        }
        return{status:'CLEAR'};
      };
      // Anchors at required bounds recover simple bank shifts that a coarse Cartesian
      // grid can miss. Every stock length/gap is preserved; trajectories are still required.
      const requiredMin=Math.min(...base.coverage.intervals.map(v=>v[0]));
      const requiredMax=Math.max(...base.coverage.intervals.map(v=>v[1]));
      const offsets=['punch','die'].map(kind=>{
        const bank=config.segments.filter(s=>s.kind===kind);
        if(!bank.length||bank.some(s=>s.fixed))return[0];
        const lo=Math.min(...bank.map(s=>s.centerY-s.lengthMm/2)),hi=Math.max(...bank.map(s=>s.centerY+s.lengthMm/2));
        return[...new Set([0,requiredMin-lo,requiredMax-hi,(requiredMin+requiredMax-lo-hi)/2])];
      });
      const seeds=[];
      for(const punchOffset of offsets[0])for(const dieOffset of offsets[1]){
        const placements=config.segments.map(s=>({...s,centerY:s.centerY+(s.kind==='punch'?punchOffset:dieOffset)}));
        const cost=placements.reduce((sum,p,i)=>sum+Math.abs(p.centerY-config.segments[i].centerY),0);
        seeds.push({placements,cost});
      }
      seeds.sort((a,b)=>a.cost-b.cost);
      let result=null,seedsVisited=0;
      for(const seed of seeds){
        seedsVisited++;
        if(seed.placements.some(s=>bedViolation(s,config)))continue;
        let fits=true;
        for(const kind of ['punch','die']){
          const bank=seed.placements.filter(s=>s.kind===kind).sort((a,b)=>a.centerY-b.centerY);
          for(let i=1;i<bank.length;i++)if(bank[i].centerY-bank[i].lengthMm/2-(bank[i-1].centerY+bank[i-1].lengthMm/2)<.3-1e-7)fits=false;
        }
        if(!fits||evaluateLayout(seed.placements).status!=='CLEAR')continue;
        result={status:'LAYOUT_FOUND',placements:seed.placements,cost:seed.cost,collisionStatus:'SAMPLE_CLEAR',unusedSegmentIds:[],
          search:{method:'ALIGNED_BANK_SEEDS',visited:seedsVisited,evaluated:candidates,complete:false,
            budgetExhausted:false,scope:'Finite existing-bank translations anchored at material bounds; not a global optimum.'}};
        break;
      }
      result=result??solveSegmentLayout({segments:config.segments.map(s=>({...s,required:true})),
        bedMinY:-config.bedLengthMm/2,bedMaxY:config.bedLengthMm/2,
        requiredIntervals:base.coverage.intervals.flatMap(([minY,maxY])=>['punch','die'].map(kind=>({minY,maxY,kind}))),
        minGapMm:.3,coverageToleranceMm:.5,offsetStepMm:5,maxCandidates:1600,maxCentersPerSegment:28,evaluateLayout});
      result.search.seedCandidatesChecked=seedsVisited;
      Object.assign(result,{checkedAmounts:checkpointAmounts,trajectoryChecked:false,coverageExact:true,jointGapToleranceMm:.5,
        motionPaths:[],motionApplied:false,continuousCollisionDetection:false});
      if(result.status==='LAYOUT_FOUND'){
        const motion=planSequentialMoves(config,result.placements,moveOne,{onProgress});
        result.motionPaths=motion.motionPaths;result.trajectoryChecked=motion.trajectoryChecked;
        result.moveAttempts=motion.attempts;result.startSegments=copy(config.segments);
        if(motion.status==='SAMPLE_CLEAR')result.targetSegments=motion.targetSegments;
        else{
          result.status='BLOCKED';result.reason=motion.reason;result.proposedPlacements=result.placements;
          result.placements=[];result.targetSegments=copy(config.segments);result.longestPartialPathCount=motion.longestPartialPathCount;
        }
      }
      return result;
    }
    throw new Error('Operación desconocida.');
  }
  return{handle};
}

if(typeof self!=='undefined'&&typeof document==='undefined'){
  const runtime=createMachineWorkerRuntime();
  self.onmessage=async event=>{
    const{id,type}=event.data;
    try{
      const result=await runtime.handle(event.data,result=>self.postMessage({id,type:'progress',result}));
      self.postMessage({id,type,result});
    }catch(error){self.postMessage({id,type,error:error.message});}
  };
}
