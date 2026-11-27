/** Generic fixed-station trial: every punch used by a single-station plan is
 * mounted at once in its own station (shared ram, shared table). Each stroke is
 * re-verified in 3D against the whole installation, so moving the part between
 * stations is a transfer, not a tool change. Geometry only. */
import {evaluateSequenceTransition} from './sequence-transition.mjs';
import {finishSequencePlan,checkSequenceBackgauge} from './sequence-plan.mjs';

const EPS=1e-6;

// Allocate die pieces per station from real inventory: the shortest length that
// still covers the punch, never using more pieces than are in stock.
export function allocateDies(die,punchLengths){
  const stock=(die.inventory??[]).map(i=>({lengthMm:i.lengthMm,count:i.count??null})).sort((a,b)=>a.lengthMm-b.lengthMm);
  const used=new Map(),picks=[];
  for(const need of punchLengths){
    const option=stock.find(s=>s.lengthMm+EPS>=need&&(s.count===null||(used.get(s.lengthMm)??0)<s.count));
    if(!option)return {ok:false,picks,shortage:{dieCode:die.vendorCode,lengthMm:need}};
    used.set(option.lengthMm,(used.get(option.lengthMm)??0)+1);picks.push(option.lengthMm);
  }
  // Report the simultaneous demand per length against stock (null = unknown quantity).
  const demand=[...used].map(([lengthMm,required])=>({dieCode:die.vendorCode,lengthMm,required,
    available:stock.find(s=>s.lengthMm===lengthMm)?.count??null}));
  return {ok:true,picks,demand};
}

function inventoryAdvice(die,punchLengths){
  // Same-length variant: what would be needed if every station used the punch length.
  const advice=[];
  const byLength=new Map();
  for(const l of punchLengths)byLength.set(l,(byLength.get(l)??0)+1);
  for(const [lengthMm,required] of byLength){
    const available=die.inventory?.find(i=>Math.abs(i.lengthMm-lengthMm)<EPS)?.count??null;
    if(available===null||available<required)advice.push({dieCode:die.vendorCode,lengthMm,required,available});
  }
  return advice;
}

export function verifyFixedStations({model,catalog,basePlan,options:input={}},onProgress=()=>{}){
  const stagesIn=basePlan?.stages??[];
  // A station is a punch in one orientation: turning a punch around mid-program is a
  // tool change too, so the same punch used both ways gets two stations mounted at
  // once (Clamp SM: B1-B5 normal, B6 mirrored).
  const punchMirror=s=>!!s.config?.segments?.find(g=>g.kind==='punch')?.mirror;
  const stationKey=s=>s.config?.punchId+'|'+punchMirror(s);
  const punchIds=[...new Set(stagesIn.map(s=>s.config?.punchId))];
  const keys=[...new Set(stagesIn.map(stationKey))];
  const dieIds=[...new Set(stagesIn.map(s=>s.config?.dieId))];
  if(keys.length<2||keys.length>3||dieIds.length!==1)
    return {status:'FIXED_STATIONS_NOT_APPLICABLE',reason:'needs 2-3 punch mounts over one die profile'};
  const toolById=id=>catalog.tools.find(t=>t.id===id);
  const bothWays=new Set(punchIds.filter(id=>keys.includes(id+'|true')&&keys.includes(id+'|false')));
  const mounts=(bothWays.size?keys:punchIds.map(id=>keys.find(k=>k.startsWith(id+'|'))))
    .map(k=>{const [id,m]=k.split('|');return {key:k,punch:toolById(id),fixedMirror:bothWays.has(id)?m==='true':null};});
  const die=toolById(dieIds[0]),punches=mounts.map(m=>m.punch);
  if(!die||punches.some(p=>!p))return {status:'FIXED_STATIONS_NOT_APPLICABLE',reason:'tool missing from catalog'};
  const options={clearanceMm:.15,contactGapMm:.3,additionalRetractMm:6.35,foldSamples:21,foldStepDegrees:5,
    accessPointStepMm:2,ramStepMm:2,maxAccessSamples:2500,frontMarginMm:10,maxUnsupportedEndMm:12.7,
    bedLengthMm:stagesIn[0].config.bedLengthMm??838.2,...input,
    backgauge:{separationMm:100,centerYmm:0,...input.backgauge}};
  const spacing=options.stationSpacingMm??101.6;
  const punchLength=stagesIn[0].config.segments?.find(s=>s.kind==='punch')?.lengthMm??25.4;
  const n=punches.length;
  if(spacing<punchLength||(n-1)*spacing+punchLength>options.bedLengthMm)
    return {status:'FIXED_STATIONS_NOT_APPLICABLE',reason:'stations do not fit on the bed'};
  // Stations ordered by first use (station A = first punch of the sequence).
  const dies=allocateDies(die,punches.map(()=>punchLength));
  const study={spacingMm:spacing,dieCode:die.vendorCode,inventoryOk:dies.ok,dieDemand:dies.demand??[],
    shortage:dies.shortage??null,sameLengthShortage:inventoryAdvice(die,punches.map(()=>punchLength)),attempts:[]};
  if(!dies.ok)return {status:'FIXED_STATIONS_INVENTORY_SHORT',stationStudy:study};
  // Same punch in two orientations needs two pieces of it.
  for(const id of bothWays){
    const p=toolById(id),need=mounts.filter(m=>m.punch.id===id).length;
    if(p.inventory?.length){
      const have=p.inventory.filter(i=>i.lengthMm+EPS>=punchLength).reduce((a,i)=>a+(i.count??Infinity),0);
      if(have<need)return {status:'FIXED_STATIONS_NOT_APPLICABLE',reason:`punch ${p.vendorCode}: ${need} pieces needed, ${have} in stock`,stationStudy:study};
    }
  }
  const ops=stagesIn.map(s=>({bendId:s.bendId,punchId:s.config.punchId,flip:!!s.config.flip,
    mount:bothWays.has(s.config.punchId)?stationKey(s):null,from:s.from,to:s.to}));
  // Try every punch orientation; first (non-mirrored) wins ties so the installed direction is kept.
  const perms=a=>a.length<2?[a]:a.flatMap((x,i)=>perms([...a.slice(0,i),...a.slice(i+1)]).map(p=>[x,...p]));
  const diePlans=[...new Map(perms(dies.picks).map(p=>[p.join(),p])).values()];
  const combos=[];
  for(let m=0;m<(1<<n);m++){
    if(mounts.some((x,i)=>x.fixedMirror!==null&&x.fixedMirror!==!!(m>>i&1)))continue;
    for(const slots of perms([...punches.keys()]))for(const picks of diePlans)
      combos.push({picks,slots,mirrors:punches.map((_,i)=>!!(m>>i&1))});
  }
  // Setups usually fail on the same stroke: check the one that blocked the previous
  // setup first (absolute entry/exit states), and only then the whole sequence.
  let best=null,fallback=null,lastBlocked=-1;
  for(const {picks,slots,mirrors} of combos){
    const stations=punches.map((punch,i)=>({code:punch.vendorCode+(mounts[i].fixedMirror===null?'':mounts[i].fixedMirror?'-R':'-N'),
      key:mounts[i].key,punch,mirror:mirrors[i],centerY:(slots[i]-(n-1)/2)*spacing,dieLengthMm:picks[i]}));
    const segments=stations.flatMap(st=>[
      {id:'punch-'+st.code,kind:'punch',bankId:st.code,centerY:st.centerY,lengthMm:punchLength,punchId:st.punch.id,mirror:st.mirror},
      {id:'die-'+st.code,kind:'die',bankId:st.code,centerY:st.centerY,lengthMm:st.dieLengthMm}]);
    const stages=[],attempt={dieLengthsMm:Object.fromEntries(stations.map(s=>[s.code,s.dieLengthMm])),mirrors:Object.fromEntries(stations.map(s=>[s.code,s.mirror]))};
    const trialFor=(op,from,to)=>{
      const station=stations.find(s=>op.mount?s.key===op.mount:s.punch.id===op.punchId);
      onProgress({phase:'checking-fixed-station',bendId:op.bendId,mirrors:attempt.mirrors});
      const config={punchId:station.punch.id,dieId:die.id,segments,bedLengthMm:options.bedLengthMm,
        stationYmm:station.centerY,activeBankId:station.code,flip:op.flip,contactGapMm:options.contactGapMm};
      return {station,trial:evaluateSequenceTransition({model,catalog,from,to,bendId:op.bendId,config,operationId:'B'+op.bendId,options})};
    };
    const blockedBy=(op,station,trial)=>({bendId:op.bendId,station:station.code,reason:trial.reason,
      bodies:trial.witness?.bodyIds??trial.hits?.[0]?.bodyIds??null});
    let from=ops[0].from??Object.fromEntries(model.bends.map(b=>[b.id,0])),blocked=null,pre=null;
    if(lastBlocked>0&&ops[lastBlocked].from&&ops[lastBlocked].to){
      const op=ops[lastBlocked];pre=trialFor(op,op.from,op.to);
      if(!pre.trial.feasible)blocked=blockedBy(op,pre.station,pre.trial);
    }
    if(!blocked)for(const [i,op] of ops.entries()){
      const to=op.to??{...from,[op.bendId]:1};
      const {station,trial}=i===lastBlocked&&pre?pre:trialFor(op,from,to);
      if(!trial.feasible){blocked=blockedBy(op,station,trial);lastBlocked=i;break;}
      stages.push(trial.stage);from=to;
    }
    attempt.blocked=blocked;study.attempts.push(attempt);
    if(blocked)continue;
    const result={status:'CANDIDATE_SEQUENCE',stages,completeManufacturingVerified:false,completeGeometricRouteSampleClear:false,
      diagnostics:{pending:[]},allStagesSampleClear:true,allStagesOperatorSupportPass:true,
      stationStudy:{...study,stations:stations.map(s=>({punchCode:s.code,punchId:s.punch.id,mirror:s.mirror,
        centerYmm:s.centerY,punchLengthMm:punchLength,dieLengthMm:s.dieLengthMm}))}};
    finishSequencePlan(model,catalog,result,options);
    if(!result.transfersVerified){attempt.blocked={reason:'TRANSFER_NOT_VERIFIED',detail:result.transferError??null};continue;}
    const transfers=stages.slice(1).filter((s,i)=>s.config.stationYmm!==stages[i].config.stationYmm).length;
    result.cost={toolChangeEvents:result.transfers.filter(t=>t.requiresManualSetupChange).length,
      profileChanges:0,orientationChanges:0,segmentEdits:0,stationTransfers:transfers,uncoveredLengthMm:basePlan.cost?.uncoveredLengthMm??0};
    result.stationStudy.stationTransfers=transfers;
    checkSequenceBackgauge(model,catalog,result,options,onProgress);
    if(!result.completeGeometricRouteSampleClear){attempt.blocked={reason:result.status};fallback??=result;continue;}
    if(result.status==='CANDIDATE_SEQUENCE')result.status='COMPLETE_GEOMETRIC_ROUTE_SAMPLE_CLEAR';
    best=result;break;
  }
  return best??fallback??{status:'FIXED_STATIONS_BLOCKED',stationStudy:study};
}
