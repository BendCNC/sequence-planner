/** Renderer-independent entry point, including the transfers between operations. */
import {planSequence} from './sequence-engine.mjs';
import {rankToolingCandidates,selectPlanGroups} from './tooling-candidates.mjs';
import {withToolingRules} from './tooling-rules.mjs';
import {buildExteriorTransfers} from './sequence-playback.mjs';
import {assessCoverage} from './coverage.mjs';
import {sceneWithAmounts} from './sequence-scene.mjs';
import {autoFingerLayout,planBackgauge,planBackgaugeOnPanel,parkingPose} from './backgauge.mjs';
import {assessToolSupport,toolSupportPolicy} from './tool-support-policy.mjs';
import {evaluateSequenceTransition} from './sequence-transition.mjs';

export function finishSequencePlan(model,catalog,result,options={maxUnsupportedEndMm:0}){
  result.completeGeometricRouteSampleClear=false;
  result.toolSupportPolicy=toolSupportPolicy(options);
  // Recompute from the actual bank at each active station. A long bank parked
  // elsewhere cannot cover a short bank's bend; serialized flags are not proof.
  const supportChecks=(result.stages??[]).map(stage=>{
    const scene=sceneWithAmounts(model,catalog,stage.config,stage.from);
    const coverage=assessCoverage({materialIntervals:scene.coverage.intervals,segments:stage.config.segments,activeBankId:stage.config.activeBankId,exact:scene.coverage.exact});
    return stage.toolSupport=assessToolSupport(coverage,result.toolSupportPolicy);
  });
  result.fullToolCoverageVerified=Boolean(supportChecks.length)&&supportChecks.every(s=>s.fullCoverage);
  result.toolSupportPolicyVerified=Boolean(supportChecks.length)&&supportChecks.every(s=>s.accepted);
  result.coverageUnresolved=Boolean(supportChecks.length)&&!result.toolSupportPolicyVerified;
  result.usesEndAllowance=supportChecks.some(s=>s.usesEndAllowance);
  if(result.stages?.length&&!result.toolSupportPolicyVerified){result.status='DIAGNOSTIC_UNDERCOVERED';return result;}
  const complete=result.stages?.length&&result.allStagesSampleClear===true&&
    result.stages.every(s=>s.status==='SAMPLE_CLEAR'&&s.foldSamplesClear===true&&
      ['entry','exit'].every(name=>s.access?.[name]?.path?.status==='SAMPLE_CLEAR'&&s.access?.[name]?.ram?.status==='SAMPLE_CLEAR'))&&
    model.bends.every(b=>Math.abs(result.stages.at(-1).to[b.id]-1)<1e-8);
  if(!complete)return result;
  try{
    result.transfers=buildExteriorTransfers(model,catalog,result.stages);
    result.transfersVerified=result.transfers.every(t=>t.status==='PART_EXTERIOR_ROUTE_ANALYTIC_CLEAR');
    result.toolChangeMotionVerified=result.transfers.every(t=>t.toolChangeMotionVerified);
    result.completeGeometricRouteSampleClear=result.transfersVerified;
    if(result.transfersVerified&&result.diagnostics?.pending){
      result.diagnostics.pending=result.diagnostics.pending.filter(p=>!p.startsWith('Exterior transfer'));
    }
  }catch(error){result.transferError=error.message;result.transfersVerified=false;}
  return result;
}

/** Material/thickness tooling preference: plan with the preferred punch/die
 * first; if that yields no complete route, run the normal tooling search. */
export async function planCompleteSequence(payload,onProgress=()=>{}){
  const result=await withToolingRules(payload,onProgress,planWithRules);
  addFlangeGaugeOption(payload.model,payload.catalog,result,payload.options??{});
  return result;
}

/** Operator option, computed once on the final plan only (never during the
 * search): for strokes whose widest-spacing fingers rest on another panel,
 * try both fingers on the rear edge of the flange being bent. Route and
 * tooling are unchanged; only finger X/R/Y differ. Bounded by
 * options.flangeGaugeBudgetMs (default 300 ms); skipped when out of time. */
export function addFlangeGaugeOption(model,catalog,result,options){
  if(!options.backgauge||options.backgauge.fixedStations||!catalog?.fingerDefinition||
    !result?.completeGeometricRouteSampleClear||!result.backgaugeSampleClear||options.flangeGaugeBudgetMs===0)return result;
  const started=performance.now(),budget=options.flangeGaugeBudgetMs??300;
  const steps=[];let skipped=0;
  for(const [i,stage]of result.stages.entries()){
    const bend=model.bends.find(b=>String(b.id)===String(stage.bendId));
    const x=stage.panelX??{},ids=bend?[bend.parentPanelId,bend.childPanelId]:[];
    const flange=ids.filter(id=>Number.isFinite(x[id])).sort((a,b)=>x[b]-x[a])[0];
    if(flange===undefined||!(x[flange]>0))continue;
    const current=stage.backgauge;
    if(current?.contacts?.every(k=>!k.used||k.witness?.bodyId===`panel:${flange}`))continue;
    if(performance.now()-started>budget){skipped++;continue;}
    const scene=sceneWithAmounts(model,catalog,stage.access.entry.config,stage.from);
    const alt=planBackgaugeOnPanel({scene,definition:catalog.fingerDefinition,settings:options.backgauge,
      park:current.park},flange);
    if(alt)steps.push({stageIndex:i,bendId:stage.bendId,panelId:flange,backgauge:alt});
  }
  if(steps.length)result.flangeGaugeOption={steps,elapsedMs:performance.now()-started,skippedForBudget:skipped};
  return result;
}

async function planWithRules(payload,onProgress){
  const {toolingSelection,dieRules,...rest}=payload.options??{};
  const pairOnly=rest.preferredPairOnly===true;
  if(pairOnly&&(!toolingSelection||!rest.preferredPunchId||!rest.preferredDieId)){
    const error='preferredPairOnly requires toolingSelection, preferredPunchId and preferredDieId';
    return {status:'UNSUPPORTED_INPUT',reason:error,stages:[],transfers:[],completeGeometricRouteSampleClear:false,diagnostics:{error}};
  }
  if(!toolingSelection){
    return planWithPreference({...payload,options:rest},onProgress);
  }
  const {preferredPunchId,preferredDieId,stopWhenPreferredClear,stopAtFirstClear,...options}=rest;
  const ranking=rankToolingCandidates(payload.model,payload.catalog,{...toolingSelection,preferredPunchId,preferredDieId});
  const permitted=Object.values(rest.toolingByBend??{});
  let groups=selectPlanGroups({...ranking,candidates:ranking.candidates.filter(c=>
    permitted.every(b=>b.dieIds.includes(String(c.dieId))&&b.punchIds.includes(String(c.punchId))))},ranking.settings.maxPlans);
  const preferredCandidate=ranking.candidates.find(c=>c.preferred)??null;
  const summary={ruleOf8:ranking.ruleOf8,designInsideRadiusMm:ranking.designInsideRadiusMm,thicknessMm:ranking.thicknessMm,
    includedAngleDegrees:ranking.includedAngleDegrees,radiusToleranceMm:ranking.radiusToleranceMm,shortestFlangeMm:ranking.shortestFlangeMm,
    viableCandidateCount:ranking.candidates.filter(c=>c.viable).length,candidateCount:ranking.candidates.length,
    rejectedDies:rejectedDies(ranking.candidates),
    preferred:preferredPunchId||preferredDieId?{preferredPunchId:preferredPunchId??null,preferredDieId:preferredDieId??null,
      matchesDesign:!!preferredCandidate?.viable,reasons:preferredCandidate?.reasons??['PREFERRED_TOOL_NOT_IN_CATALOG']}:null,
    designMatch:groups.length>0,alternatives:[]};
  let searchPayload=payload;
  if(pairOnly){
    const pair=groups.find(g=>g.preferred);
    if(!pair){
      const status=!preferredCandidate?'PREFERRED_TOOL_NOT_IN_CATALOG':!preferredCandidate.viable?
        'PREFERRED_TOOLING_DOES_NOT_MATCH_DESIGN':'PREFERRED_TOOLING_DOES_NOT_MATCH_RULES';
      const reason=summary.preferred.reasons.join(',')||status;
      return {status:'NO_COMPATIBLE_TOOL_CANDIDATES',reason,stages:[],transfers:[],completeGeometricRouteSampleClear:false,
        toolingSelection:{...summary,selectedIndex:-1},toolingPreference:{preferredPunchId,preferredDieId,applied:false,preferredResult:{status,reason}},
        diagnostics:{statesExpanded:0,transitionsChecked:0}};
    }
    groups=[{...pair,punchIds:[preferredPunchId]}];
    // Keep auxiliary collision bodies. A supplied multi-bank setup must also
    // draw all forming segments from this pair's real inventory.
    const tools=payload.catalog.tools.filter(t=>(t.kind!=='punch'||String(t.id)===String(preferredPunchId))&&
      (t.kind!=='die'||String(t.id)===String(preferredDieId)));
    searchPayload={...payload,catalog:{...payload.catalog,tools}};
  }
  const plans=[];
  let preferredAttempt;
  for(const [index,group] of groups.entries()){
    // Import mode: preferred (calibrated) tooling already gives a verified route, skip alternatives.
    if(stopWhenPreferredClear&&index>0&&groups[0].preferred&&summary.alternatives[0]?.preferred&&plans[0]?.completeGeometricRouteSampleClear)break;
    // Export mode: groups are already ranked by design match; the first verified route wins.
    if(stopAtFirstClear&&plans.some(p=>p.completeGeometricRouteSampleClear))break;
    onProgress({phase:'tooling-alternative',index,count:groups.length,dieId:group.dieId});
    let punchIds=group.preferred?[preferredPunchId]:group.punchIds;
    // Once a route is found, alternatives get at most 2x that search time so they cannot dominate the import.
    const reference=plans.find(p=>p.completeGeometricRouteSampleClear);
    const budget=reference&&options.timeBudgetMs?Math.min(options.timeBudgetMs,
      Math.max(10000,2*(reference.diagnostics?.completePlanElapsedMs??options.timeBudgetMs))):options.timeBudgetMs;
    let plan=await planOnce({...searchPayload,options:{...options,dieId:group.dieId,allowedPunchIds:punchIds,
      ...(budget?{timeBudgetMs:budget}:{})}},onProgress);
    if(group.preferred&&!plan.completeGeometricRouteSampleClear&&group.punchIds.length>1){
      // The same die can succeed with another punch. Keep the actual outcome
      // of the preferred-only attempt before the broader search replaces it.
      preferredAttempt={status:plan.status,reason:plan.reason??plan.diagnostics?.budgetReason??null};
      punchIds=group.punchIds;
      const other=await planOnce({...payload,options:{...options,dieId:group.dieId,allowedPunchIds:punchIds,
        ...(budget?{timeBudgetMs:budget}:{})}},onProgress);
      if(other.completeGeometricRouteSampleClear||!plan.stages?.length)plan=other;
    }
    // Fewest setups first: a route that swaps punches (later turned into fixed
    // stations) is only kept when no single punch of the group clears alone.
    if(plan.completeGeometricRouteSampleClear&&(plan.cost?.toolChangeEvents??0)>0&&(punchIds?.length??0)>1){
      const used=(plan.stages??[]).map(s=>String(s.config?.punchId));
      const order=[...punchIds].sort((a,b)=>used.lastIndexOf(String(b))-used.lastIndexOf(String(a)));
      for(const punchId of order){
        onProgress({phase:'single-setup-check',dieId:group.dieId,punchId});
        const single=await planOnce({...payload,options:{...options,dieId:group.dieId,allowedPunchIds:[punchId],
          ...(budget?{timeBudgetMs:budget}:{})}},onProgress);
        if(single.completeGeometricRouteSampleClear&&(single.cost?.toolChangeEvents??0)===0){plan=single;break;}
      }
    }
    const usedPunchIds=[...new Set((plan.stages??[]).map(s=>s.config?.punchId).filter(Boolean))];
    const lead=ranking.candidates.find(c=>String(c.dieId)===String(group.dieId)&&usedPunchIds.map(String).includes(String(c.punchId)))??group.lead;
    summary.alternatives.push({index,dieId:group.dieId,dieName:lead.dieName,punchIds:usedPunchIds.length?usedPunchIds:punchIds,
      punchName:lead.punchName,openingMm:lead.openingMm,dieAngleDegrees:lead.dieAngleDegrees,punchTipRadiusMm:lead.punchTipRadiusMm,predictedInsideRadiusMm:lead.predictedInsideRadiusMm,
      radiusErrorMm:lead.radiusErrorMm,minFlangeMm:lead.minFlangeMm,tonnagePerMeterKn:lead.tonnagePerMeterKn,
      compromise:lead.compromise??null,preferred:group.preferred&&(!usedPunchIds.length||usedPunchIds.every(id=>String(id)===String(preferredPunchId))),status:plan.status,reason:plan.reason??null,clear:!!plan.completeGeometricRouteSampleClear,
      stageCount:plan.stages?.length??0,setupCount:new Set((plan.stages??[]).map(s=>`${s.config?.punchId}|${s.config?.dieId}`)).size});
    plans.push(plan);
  }
  let main=summary.alternatives.findIndex(a=>a.clear);
  // No tooling gives a verified route: keep the operator's preferred tooling
  // instead of silently swapping punches (status stays NOT fully verified).
  if(main<0)main=summary.alternatives.findIndex(a=>a.preferred&&a.stageCount>0);
  if(main<0&&pairOnly)main=0; // Return this pair's actual outcome, including its search budget.
  let result;
  if(main>=0)result=plans[main];
  else{
    onProgress({phase:'tooling-search-fallback'});
    result=await planWithPreference({...payload,options:{...options,preferredPunchId,preferredDieId}},onProgress);
    summary.fallback=true;
  }
  summary.selectedIndex=main;
  result.alternativePlans=plans.map((p,i)=>i!==main&&summary.alternatives[i].clear?{index:i,plan:p}:null).filter(Boolean);
  result.toolingSelection=summary;
  if(main>=0&&(preferredPunchId||preferredDieId)){
    const applied=summary.alternatives[main].preferred&&(!pairOnly||result.completeGeometricRouteSampleClear||result.stages?.length>0);
    if(pairOnly&&!result.completeGeometricRouteSampleClear)
      preferredAttempt={status:result.status,reason:result.reason??result.diagnostics?.budgetReason??null};
    result.toolingPreference={preferredPunchId:preferredPunchId??null,preferredDieId:preferredDieId??null,
      applied,
      preferredResult:applied&&(!pairOnly||result.completeGeometricRouteSampleClear)?undefined:
        (preferredAttempt??summary.alternatives.find(a=>a.preferred)??{status:'PREFERRED_TOOLING_DOES_NOT_MATCH_DESIGN',reason:summary.preferred?.reasons?.join(',')})};
  }
  return result;
}

function rejectedDies(candidates){
  const byDie=new Map();
  for(const c of candidates){const d=byDie.get(String(c.dieId))??{dieId:c.dieId,dieName:c.dieName,openingMm:c.openingMm,
    predictedInsideRadiusMm:c.predictedInsideRadiusMm,viable:false,reasons:new Set()};
    d.viable||=c.viable;c.reasons.forEach(r=>d.reasons.add(r));byDie.set(String(c.dieId),d);}
  return[...byDie.values()].filter(d=>!d.viable).map(({viable,...d})=>({...d,reasons:[...d.reasons]}));
}

async function planWithPreference(payload,onProgress){
  const {preferredPunchId,preferredDieId,...options}=payload.options??{};
  if(!preferredPunchId&&!preferredDieId)return planOnce({...payload,options},onProgress);
  const known=kind=>id=>!id||payload.catalog.tools.some(t=>t.kind===kind&&String(t.id)===String(id));
  const preference={preferredPunchId:preferredPunchId??null,preferredDieId:preferredDieId??null};
  let preferred=null;
  const permitted=Object.values(options.toolingByBend??{}).every(b=>
    (!preferredDieId||b.dieIds.includes(String(preferredDieId)))&&(!preferredPunchId||b.punchIds.includes(String(preferredPunchId))));
  if(permitted&&known('punch')(preferredPunchId)&&known('die')(preferredDieId)){
    onProgress({phase:'preferred-tooling',...preference});
    preferred=await planOnce({...payload,options:{...options,
      ...(preferredPunchId?{punchId:preferredPunchId}:{}),...(preferredDieId?{dieId:preferredDieId}:{})}},onProgress);
    if(preferred.completeGeometricRouteSampleClear){
      preferred.toolingPreference={...preference,applied:true};return preferred;}
    preference.preferredResult={status:preferred.status,reason:preferred.reason??null};
  }else preference.preferredResult={status:permitted?'PREFERRED_TOOL_NOT_IN_CATALOG':'PREFERRED_TOOLING_DOES_NOT_MATCH_RULES',reason:null};
  onProgress({phase:'tooling-search-fallback'});
  const fallback=await planOnce({...payload,options},onProgress);
  if(!fallback.completeGeometricRouteSampleClear&&preferred?.stages?.length){
    preferred.toolingPreference={...preference,applied:true};return preferred;}
  fallback.toolingPreference={...preference,applied:false};
  return fallback;
}

async function planOnce(payload,onProgress){
  const started=performance.now();
  const {model,catalog,options={}}=payload;
  const validatedPlans=new WeakMap();
  const orientedCandidate=candidate=>{
    // A successful fold sequence can still fail transfers or gauge placement.
    // Reject that complete candidate and allow the search to continue. Clone
    // first: orientation selection must not mutate shared cached transitions.
    const checked=structuredClone(candidate);
    if(options.backgauge&&catalog.fingerDefinition)preferGaugeFaceOrientation(model,catalog,checked,options,onProgress);
    return checked;
  };
  const validateCandidate=candidate=>{
    const checked=orientedCandidate(candidate);
    finishSequencePlan(model,catalog,checked,options);
    checkSequenceBackgauge(model,catalog,checked,options,onProgress);
    if(checked.completeGeometricRouteSampleClear)validatedPlans.set(candidate.stages,checked);
    return{accepted:checked.completeGeometricRouteSampleClear===true,reason:checked.status==='CANDIDATE_SEQUENCE'?
      (checked.transferError?'EXTERIOR_TRANSFER_REJECTED':'COMPLETE_ROUTE_REJECTED'):checked.status};
  };
  const validateGroupedCandidate=async candidate=>{
    // Verify movement of the fingers between consecutive sides and timeline
    // continuity before displacing the route selected by the ordinary search.
    const {finishAdvancedSequencePlan}=await import('./sequence-advanced-completion.mjs');
    const checked=finishAdvancedSequencePlan(model,catalog,orientedCandidate(candidate),options,onProgress);
    if(checked.completeGeometricRouteSampleClear)validatedPlans.set(candidate.stages,checked);
    return {accepted:checked.completeGeometricRouteSampleClear===true,reason:checked.reason??checked.status};
  };
  const sequence=await planSequence({...payload,options:{...options,
    ...(options.validateFullRouteDuringSearch?{validateCompleteCandidate:validateCandidate}:{})}},onProgress,validateGroupedCandidate);
  const validated=validatedPlans.get(sequence.stages);
  if(validated){
    // Reuse the independently completed check for this exact array of stages.
    // Do not repeat expensive transfer and gauge validation after acceptance.
    return{...sequence,...validated,diagnostics:{...sequence.diagnostics,
      pending:sequence.diagnostics?.pending?.filter(p=>!p.startsWith('Exterior transfer')),
      completePlanElapsedMs:performance.now()-started}};
  }
  if(options.backgauge&&catalog.fingerDefinition)preferGaugeFaceOrientation(model,catalog,sequence,options,onProgress);
  const result=finishSequencePlan(model,catalog,sequence,options);
  checkSequenceBackgauge(model,catalog,result,options,onProgress);
  result.diagnostics.completePlanElapsedMs=performance.now()-started;
  return result;
}

/** Gauge on the largest available face: when the 180Â° opposite orientation of
 * a stage is also feasible and lets the finger touch a taller face, use it. */
export function preferGaugeFaceOrientation(model,catalog,sequence,options,onProgress=()=>{}){
  const touched=(config,amounts)=>{const g=planBackgauge({scene:sceneWithAmounts(model,catalog,config,amounts),definition:catalog.fingerDefinition});
    return g.status==='SAMPLE_CLEAR'&&g.touchedFaceZ?g.touchedFaceZ.maxZ-g.touchedFaceZ.minZ:0;};
  for(const [i,stage] of (sequence.stages??[]).entries()){
    if(stage.status!=='SAMPLE_CLEAR'||!stage.access?.entry?.config)continue;
    onProgress({phase:'checking-gauge-orientation',bendId:stage.bendId});
    const alt=evaluateSequenceTransition({model,catalog,from:stage.from,to:stage.to,bendId:stage.bendId,
      config:{...stage.config,flip:!stage.flip},operationId:stage.id,options});
    if(!alt.feasible||alt.stage.status!=='SAMPLE_CLEAR')continue;
    const altStage={...alt.stage,id:stage.id};
    const alternate=touched(altStage.access.entry.config,altStage.from);
    // A face height is nonnegative. Measure the current side only when
    // the alternative could exceed it by the required millimetre.
    if(alternate>1&&alternate>touched(stage.access.entry.config,stage.from)+1){
      sequence.stages[i]=altStage;
      if(sequence.edges?.[i])sequence.edges[i]={...sequence.edges[i],config:altStage.config,flip:altStage.flip,coverage:alt.coverage};
    }
  }
}

export function checkSequenceBackgauge(model,catalog,result,options,onProgress=()=>{}){
  if(options.backgauge&&result.completeGeometricRouteSampleClear){
    onProgress({phase:'checking-backgauge'});
    if(!catalog.fingerDefinition)throw new Error('Falta la geometrÃ­a de los fingers.');
    function* sampledBodies(){
      for(const stage of result.stages)for(const sample of stage.foldSamples){
        yield* sceneWithAmounts(model,catalog,stage.config,sample.amounts).bodies;
      }
    }
    const park=parkingPose(sampledBodies());
    const entries=result.stages.map(s=>sceneWithAmounts(model,catalog,s.access.entry.config,s.from));
    let settings=options.backgauge;
    const auto=settings.fixedStations?null:entries.map(e=>planBackgauge({scene:e,definition:catalog.fingerDefinition,settings,park,widest:true}));
    if(!settings.fixedStations&&entries.every(e=>e.placement.stationYmm===entries[0].placement.stationYmm)){
      const layouts=entries.map(e=>autoFingerLayout(e.bodies,catalog.fingerDefinition,settings.fingerGapMm));
      if(layouts.every(l=>l?.layout==='TWO_FINGERS'&&l.separationMm===layouts[0].separationMm))
        settings={...settings,fixedStations:true,separationMm:layouts[0].separationMm,centerYmm:settings.centerYmm??entries[0].placement.stationYmm};
    }
    for(const [i,stage] of result.stages.entries()){
      // Stability first: two fingers as far apart as that stroke allows;
      // the common minimum-gap spacing is only the fallback.
      const common=auto?.[i]?.widestSpacing?null:planBackgauge({scene:entries[i],definition:catalog.fingerDefinition,settings,park});
      stage.backgauge=!common?auto[i]:auto&&auto[i].status==='SAMPLE_CLEAR'&&
        (common.status!=='SAMPLE_CLEAR'||auto[i].contactCount>common.contactCount)?auto[i]:common;
      // Machine-pose authority for replay: mean X of every panel at stage
      // entry (mm from the bend line, +X toward the backgauge).
      stage.panelX=Object.fromEntries(entries[i].bodies.filter(b=>b.kind==='panel').map(b=>{
        let s=0;for(let k=0;k<b.positions.length;k+=3)s+=b.positions[k];return[b.panelId,s/(b.positions.length/3)];}));
    }
    result.backgaugeSampleClear=result.stages.every(s=>s.backgauge.status==='SAMPLE_CLEAR');
    result.backgaugeCalibrationVerified=false;
    if(!result.backgaugeSampleClear){result.completeGeometricRouteSampleClear=false;result.status='BACKGAUGE_REVIEW_REQUIRED';
      result.reason='La ruta de la pieza pasa, pero el posicionamiento del backgauge requiere revisiÃ³n con la separaciÃ³n de referencia configurada.';}
  }
  return result;
}
