/** Isolated experimental fallback. Preserve every verified result of the normal
 * search and the tested V1 expansion. A third bounded attempt adds tool
 * orientations and search fixes, never changing geometry tolerances. Each
 * attempt has its own budget (up to three budgets). No physical certification.
 */
import {planCompleteSequence} from './sequence-plan.mjs';
import {finishAdvancedSequencePlan} from './sequence-advanced-completion.mjs';
import {withToolingRules} from './tooling-rules.mjs';

export const EXPANDED_SEARCH_OPTIONS=Object.freeze({
  reverseAllClosures:true,axialPlacementMode:'edge-first',partialAnglesDegrees:[15,30,45,60],
  validateFullRouteDuringSearch:true,maxStates:500,maxTransitions:4000,maxCandidatesPerBend:72,
  maxChecksPerState:192,maxSuccessorsPerState:32,beamWidth:12,
});
export const ORIENTATION_SEARCH_OPTIONS=Object.freeze({...EXPANDED_SEARCH_OPTIONS,
  allowMirroredToolProfiles:true,maxCandidatesIncludingMirrors:288,
  relativePhaseBudgets:true,diverseAxialCandidates:true,preserveSearchHistory:true});

/** Older plans positioned fingers per operation but omitted the Y movement
 * between them. Complete that movement before retaining a successful route. */
export function completeLegacyFingerTransfers(model,catalog,plan,options={},onProgress=()=>{}){
  if(!options.backgauge||!plan.completeGeometricRouteSampleClear)return plan;
  const missing=(plan.transfers??[]).some((transfer,i)=>{
    const a=plan.stages[i]?.backgauge,b=plan.stages[i+1]?.backgauge;
    if(!a||!b)return true;
    const changes=a.stationY.length!==b.stationY.length||a.stationY.some((v,k)=>Math.abs(v-b.stationY[k])>1e-8)||
      ['lowerFaceX','baseZ'].some(k=>Math.abs(a.park[k]-b.park[k])>1e-8);
    return changes&&transfer.fingerAdjustment?.status!=='EXTERIOR_MANUAL_Y_ADJUSTMENT_GEOMETRY_CLEAR';
  });
  if(!missing)return plan;
  const start=performance.now(),checked=finishAdvancedSequencePlan(model,catalog,plan,options,onProgress);
  checked.legacyFingerTransferCompletion={required:true,accepted:checked.completeGeometricRouteSampleClear,elapsedMs:performance.now()-start};
  checked.diagnostics.completePlanElapsedMs=(plan.diagnostics?.completePlanElapsedMs??0)+checked.legacyFingerTransferCompletion.elapsedMs;
  return checked;
}

/** A later exhausted search cannot erase a necessary-condition witness from
 * an earlier attempt. This selects diagnostics only and never sets clear=true. */
export function chooseIncompleteEvidence(original,attempts){
  if(original.diagnostics?.finalSetupCheck?.status==='NO_FINAL_TOOL_SETUP_IN_CANDIDATES'){
    for(const attempt of [...attempts].reverse())if(attempt.plan.status!=='CANCELLED'&&
        attempt.plan.diagnostics?.finalSetupCheck?.status==='POSSIBLE_FINAL_TOOL_SETUP')return attempt;
  }
  return{plan:original,variant:'original'};
}

export function experimentalOutcomeText(result){
  const comparison=result.experimentalComparison;if(!comparison)return'';
  if(result.status==='CANCELLED')return'Búsqueda cancelada; no se declara una ruta nueva.';
  if(result.completeGeometricRouteSampleClear){
    if(comparison.selectedVariant==='original')return'Se conserva la ruta normal; no se necesitó un intento adicional.';
    if(comparison.selectedVariant==='advanced')return'Ruta encontrada automáticamente con prepliegues, montajes por zonas y movimientos coordinados comprobados.';
    return comparison.selectedVariant==='expanded'?'Ruta obtenida con la búsqueda ampliada V1.':'Ruta obtenida al ampliar orientaciones y búsqueda.';
  }
  if(comparison.selectedVariant!=='original'&&result.diagnostics?.finalSetupCheck?.status==='POSSIBLE_FINAL_TOOL_SETUP')
    return'Aparece una colocación final candidata, pero falta una ruta completa de acceso, plegado y retirada.';
  return comparison.attempts.length>1?'Los '+comparison.attempts.length+' intentos terminaron sin una ruta completa comprobada.':'No se ejecutó un intento adicional para este resultado.';
}

export async function planExperimentalSequence(payload,onProgress=()=>{}){
  return withToolingRules(payload,onProgress,planPortfolio);
}

async function planPortfolio(payload,onProgress){
  const started=performance.now(),{experimentalFallback=true,experimentalExtraBudgetMs,
    experimentalOrientationSearch=true,experimentalOrientationBudgetMs,
    experimentalAdvancedSearch=true,experimentalAdvancedBudgetMs,sharedTransitionCache=true,...rawOptions}=payload.options??{};
  // One cache for every attempt on this model; keys carry all check parameters.
  const options=sharedTransitionCache&&!rawOptions.transitionCache?{...rawOptions,transitionCache:Object.assign(new Map(),{toJSON:()=>undefined})}:rawOptions;
  const extraBudget=experimentalExtraBudgetMs??options.timeBudgetMs??90000;
  const orientationBudget=experimentalOrientationBudgetMs??extraBudget;
  const advancedBudget=experimentalAdvancedBudgetMs??extraBudget;
  const complete=plan=>completeLegacyFingerTransfers(payload.model,payload.catalog,plan,options,onProgress);
  // experimentalOnly runs a single variant so callers can race the variants in
  // separate processes (the search is main-thread bound) and keep their priority.
  const only=options.preferredPairOnly===true?undefined:rawOptions.experimentalOnly;
  if(only==='expanded'||only==='advanced'){
    delete options.experimentalOnly;
    const t0=performance.now();
    const plan=only==='expanded'
      ?complete(await planCompleteSequence({...payload,options:{...options,...EXPANDED_SEARCH_OPTIONS,timeBudgetMs:extraBudget}},onProgress))
      :await (await import('./sequence-advanced-search.mjs')).planAdvancedSequence({...payload,options:{...options,timeBudgetMs:advancedBudget}},onProgress);
    return {...plan,experimentalComparison:{selectedVariant:only,attempts:[{variant:only,status:plan.status,
      clear:plan.completeGeometricRouteSampleClear===true,stages:plan.stages?.length??0,cost:plan.cost??null,
      elapsedMs:performance.now()-t0,budgetReason:plan.diagnostics?.budgetReason??null}],
      elapsedMs:performance.now()-started,geometryPolicyUnchanged:true,
      additionalSearchBudgetMs:only==='expanded'?extraBudget:advancedBudget,scope:'Single variant run (raced by the caller).'}};
  }
  const original=complete(await planCompleteSequence({...payload,options},onProgress));
  const summary=plan=>({status:plan.status,clear:plan.completeGeometricRouteSampleClear===true,
    stages:plan.stages?.length??0,cost:plan.cost??null,elapsedMs:plan.diagnostics?.completePlanElapsedMs??null,
    budgetReason:plan.diagnostics?.budgetReason??null});
  const attempts=[{variant:'original',...summary(original)}];
  const finish=(plan,selectedVariant)=>({...plan,experimentalComparison:{selectedVariant,attempts,
    elapsedMs:performance.now()-started,geometryPolicyUnchanged:true,
    additionalSearchBudgetMs:attempts.reduce((sum,a)=>sum+(a.variant==='expanded'?extraBudget:a.variant==='orientations'?orientationBudget:a.variant==='advanced'?advancedBudget:0),0),
    scope:'Earlier routes are retained after completing any missing finger transfers. Failed searches may try mirrored profiles, material-island banks, geometry-derived pre-bends and coordinated access. Clearance and support rules are unchanged; holders, physical travel and manufacturing are not certified.'}});
  if(options.preferredPairOnly===true||!experimentalFallback||original.completeGeometricRouteSampleClear||
      ['FLAT_NO_BENDS','UNSUPPORTED_INPUT','CANCELLED','NO_COMPATIBLE_TOOL_CANDIDATES','FLAT_BLANK_SELF_CONTACT','FINAL_SHAPE_SELF_CONTACT'].includes(original.status))return finish(original,'original');
  if(options.signal?.aborted)return finish({...original,status:'CANCELLED'},'original');
  onProgress({phase:'experimental-fallback',originalStatus:original.status});
  const expanded=complete(await planCompleteSequence({...payload,options:{...options,...EXPANDED_SEARCH_OPTIONS,
    timeBudgetMs:extraBudget}},onProgress));
  attempts.push({variant:'expanded',...summary(expanded)});
  if(expanded.completeGeometricRouteSampleClear)return finish(expanded,'expanded');
  if(expanded.status==='CANCELLED'||options.signal?.aborted)return finish({...expanded,status:'CANCELLED'},'expanded');
  // Neither more directions nor more time can repair an unsupported mesh budget.
  if(expanded.diagnostics?.budgetReason==='mesh-vertex-budget'){
    const selected=chooseIncompleteEvidence(original,[{plan:expanded,variant:'expanded'}]);return finish(selected.plan,selected.variant);
  }
  // The original and V1 strategies retain the previously recovered routes.
  // Then try the new capability before spending another budget on the older
  // orientation-only search, which remains available if this attempt fails.
  const incomplete=[{plan:expanded,variant:'expanded'}];let advanced=null;
  if(experimentalAdvancedSearch){
    if(options.signal?.aborted)return finish({...expanded,status:'CANCELLED'},'expanded');
    onProgress({phase:'experimental-advanced',timeBudgetMs:advancedBudget});
    const {planAdvancedSequence}=await import('./sequence-advanced-search.mjs');
    const advancedOptions={...options,timeBudgetMs:advancedBudget};
    // The advanced planner owns normalization and passes that same model to
    // folding, access and completion. Do not replace it with the raw input.
    advanced=await planAdvancedSequence({...payload,options:advancedOptions},onProgress);
    attempts.push({variant:'advanced',...summary(advanced)});
    if(advanced.completeGeometricRouteSampleClear||advanced.status==='CANCELLED')return finish(advanced,'advanced');
    incomplete.push({plan:advanced,variant:'advanced'});
  }
  if(experimentalOrientationSearch){
    if(options.signal?.aborted)return finish({...expanded,status:'CANCELLED'},'expanded');
    onProgress({phase:'experimental-orientations',originalStatus:original.status,expandedStatus:expanded.status});
    const oriented=complete(await planCompleteSequence({...payload,options:{...options,...ORIENTATION_SEARCH_OPTIONS,timeBudgetMs:orientationBudget}},onProgress));
    attempts.push({variant:'orientations',...summary(oriented)});
    if(oriented.completeGeometricRouteSampleClear||oriented.status==='CANCELLED')return finish(oriented,'orientations');
    incomplete.push({plan:oriented,variant:'orientations'});
  }
  // Preserve the useful new necessary-condition evidence without claiming a route.
  const selected=chooseIncompleteEvidence(original,incomplete),result=finish(selected.plan,selected.variant);
  if(advanced)result.advancedSearchDiagnostics=advanced.diagnostics;
  return result;
}
