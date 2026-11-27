/** File-based adapter for desktop applications; geometry only, no machine commands. */
import {readFile,writeFile,mkdir,realpath} from 'node:fs/promises';
import {resolve,dirname} from 'node:path';
import {fileURLToPath} from 'node:url';
import {planCompleteSequence} from '../viewer/sequence-plan.mjs';
import {verifyFixedStations} from '../viewer/fixed-stations.mjs';
import {planExperimentalSequence} from '../viewer/sequence-portfolio.mjs';
import {validateInputAgainstReference} from '../viewer/input-validation.mjs';
import {annotateToolChanges} from '../viewer/tool-setup-search.mjs';
import {readCatalogJson} from '../viewer/protected-json.mjs';
import {serializePlanEvidence} from '../viewer/sequence-fold-evidence.mjs';

const allowed=new Set(['model','catalog','output','bed-length-mm','time-ms','max-transitions','options','reference','format']);
export async function runPlanFile(args,{stdout=process.stdout,stderr=process.stderr}={}){
const options={};
try{
  for(let i=0;i<args.length;i+=2){
    const key=args[i].replace(/^--/,'');
    if(!args[i].startsWith('--')||!allowed.has(key)||!args[i+1]||args[i+1].startsWith('--'))throw new Error('Invalid argument: '+args[i]);
    options[key]=args[i+1];
  }
  if(!options.model||!options.output)throw new Error('Use --model model.json --output plan.json --bed-length-mm 838.2 [--time-ms 120000] [--options options.json]');
  const format=options.format??(stdout.isTTY?'pretty':'compact');
  if(!['compact','pretty'].includes(format))throw new Error('Use --format compact or --format pretty');
  const indent=format==='pretty'?2:undefined;
  const catalogPath=options.catalog??fileURLToPath(new URL('../viewer/tooling.json',import.meta.url));
  const canonical=async path=>{const p=await realpath(resolve(path)).catch(()=>resolve(path));return process.platform==='win32'?p.toLowerCase():p;};
  const output=resolve(options.output),outputKey=await canonical(output);
  for(const input of [options.model,catalogPath,options.options,options.reference].filter(Boolean))if(await canonical(input)===outputKey)throw new Error('Output must not overwrite an input file.');
  const json=async path=>JSON.parse(await readFile(path,'utf8'));
  const model=await json(options.model),catalog=await readCatalogJson(catalogPath);
  const searchOptions=options.options?await json(options.options):{};
  if(searchOptions.backgauge&&!catalog.fingerDefinition&&!options.catalog)
    catalog.fingerDefinition=(await readCatalogJson(fileURLToPath(new URL('../viewer/fingers.json',import.meta.url)))).definitions[0];
  for(const [flag,key] of [['bed-length-mm','bedLengthMm'],['time-ms','timeBudgetMs'],['max-transitions','maxTransitions']]){
    if(options[flag]!==undefined){const n=Number(options[flag]);if(!Number.isFinite(n)||n<=0)throw new Error('Invalid '+flag);searchOptions[key]=n;}
  }
  if(searchOptions.rankedTerminalProof){
    const {bindRankedTerminal}=await import('../viewer/sequence-ranked-terminal.mjs');
    const proof=searchOptions.rankedTerminalProof;delete searchOptions.rankedTerminalProof;
    await bindRankedTerminal(model,catalog,searchOptions,proof);
  }
  let lastProgress=0;
  const report=progress=>{
    if(Date.now()-lastProgress>1000){stderr.write(JSON.stringify({type:'progress',...progress})+'\n');lastProgress=Date.now();}
  };
  // Input check before any search: the bend table must fold into the CAD body.
  const inputValidation=options.reference?validateInputAgainstReference(model,await json(options.reference)):{status:'NO_REFERENCE',checked:false};
  if(['INPUT_MISMATCH','SHAPE_MISMATCH_UNEXPLAINED'].includes(inputValidation.status)){
    const plan={status:'INPUT_MISMATCH',completeGeometricRouteSampleClear:false,stages:[],transfers:[],inputValidation,
      diagnostics:{error:inputValidation.message,inputValidation}};
    await mkdir(dirname(output),{recursive:true});
    await writeFile(output,JSON.stringify({schemaVersion:1,units:'mm',modelPath:resolve(options.model),modelSource:model.metadata??null,...plan},null,indent));
    stdout.write(JSON.stringify({status:plan.status,output,completeGeometricRouteSampleClear:false,stages:0,error:inputValidation.message,inputValidation})+'\n');
    return 3;
  }
  // seedSetups: tooling configs already accepted by a previous plan (e.g. the
  // caller re-plans a manual order). Verify only those first; the full search
  // runs only when they do not give a complete clear route.
  const {seedSetups,...baseOptions}=searchOptions;
  let plan=null;
  if(Array.isArray(seedSetups)&&seedSetups.length){
    // The seeds fix the tooling: stop at the first clear tooling group.
    const seeded=await planExperimentalSequence({model,catalog,options:{...baseOptions,setups:seedSetups,
      experimentalFallback:false,stopWhenPreferredClear:true,stopAtFirstClear:true}},report);
    if(seeded.completeGeometricRouteSampleClear)plan={...seeded,seedSetupsUsed:true};
  }
  plan??=await planExperimentalSequence({model,catalog,options:baseOptions},report);
  plan.inputValidation=inputValidation;
  // Tool changes between already-owned punches: try mounting them all at once.
  // The advanced variant reports toolChangeEvents at the top level, without cost.
  plan.cost??=Number.isFinite(plan.toolChangeEvents)?{toolChangeEvents:plan.toolChangeEvents}:undefined;
  if(plan.completeGeometricRouteSampleClear&&(plan.cost?.toolChangeEvents??0)>0&&searchOptions.fixedStations!==false){
    const fixed=verifyFixedStations({model,catalog,basePlan:plan,options:{bedLengthMm:searchOptions.bedLengthMm,
      backgauge:searchOptions.backgauge,stationSpacingMm:searchOptions.stationSpacingMm}},report);
    const study=fixed.stationStudy??null;
    if(fixed.completeGeometricRouteSampleClear&&fixed.cost.toolChangeEvents<plan.cost.toolChangeEvents){
      fixed.singleStationCost=plan.cost;
      plan={...plan,...fixed,diagnostics:{...plan.diagnostics,...fixed.diagnostics}};
      plan.requiresManualSetupChange=fixed.cost.toolChangeEvents>0;
    }else if(study){
      plan.fixedStationStudy={status:fixed.status,...study};
    }
    // Missing stock: verify the alternative with one extra die piece before advising it.
    if(fixed.status==='FIXED_STATIONS_INVENTORY_SHORT'&&study?.shortage){
      const x=study.shortage,extra=JSON.parse(JSON.stringify(catalog));
      const die=extra.tools.find(t=>t.vendorCode===x.dieCode&&t.kind==='die');
      const item=die.inventory?.find(i=>Math.abs(i.lengthMm-x.lengthMm)<1e-6);
      const available=item?.count??null;
      if(item)item.count=(item.count??0)+1;else (die.inventory??=[]).push({lengthMm:x.lengthMm,count:1});
      const trial=verifyFixedStations({model,catalog:extra,basePlan:plan,options:{bedLengthMm:searchOptions.bedLengthMm,
        backgauge:searchOptions.backgauge,stationSpacingMm:searchOptions.stationSpacingMm}},report);
      if(trial.completeGeometricRouteSampleClear&&trial.cost.toolChangeEvents<plan.cost.toolChangeEvents){
        const required=trial.stationStudy.dieDemand.find(d=>Math.abs(d.lengthMm-x.lengthMm)<1e-6)?.required??null;
        plan.toolingAdvice=[{dieCode:x.dieCode,lengthMm:x.lengthMm,available,required,verifiedChanges:trial.cost.toolChangeEvents,
          message:`Con el inventario actual, la secuencia requiere ${plan.cost.toolChangeEvents} cambio(s). Con 1 matriz ${x.dieCode} de ${x.lengthMm} mm adicional, se verific\u00f3 una alternativa de ${trial.cost.toolChangeEvents} cambios. `+
            (available==null||required==null?'Cantidad por confirmar.':`Disponibles: ${available}. Necesarias simult\u00e1neamente: ${required}.`)}];
      }
    }
  }
  const ex=plan.diagnostics?.extraStockAdvice;
  if(ex&&!plan.toolingAdvice?.length&&(plan.cost?.toolChangeEvents??0)>ex.verifiedChanges){
    const name=id=>{const t=catalog.tools.find(x=>String(x.id)===String(id));return t?.vendorCode??t?.name??id;};
    const inch=(ex.lengthMm/25.4).toFixed(3).replace(/\.?0+$/,'');
    plan.toolingAdvice=[{kind:'punch+die',punchId:ex.punchId,dieId:ex.dieId,punchCode:name(ex.punchId),dieCode:name(ex.dieId),lengthMm:ex.lengthMm,
      punchAvailable:ex.punchAvailable,dieAvailable:ex.dieAvailable,currentChanges:ex.currentChanges,verifiedChanges:ex.verifiedChanges,
      message:`Current inventory requires ${ex.currentChanges} tool change(s). With 1 extra punch ${name(ex.punchId)} and 1 extra die ${name(ex.dieId)} of ${ex.lengthMm} mm (${inch} in), a route with ${ex.verifiedChanges} change(s) was verified using a separate station (in stock: punch ${ex.punchAvailable}, die ${ex.dieAvailable}).`}];
  }
  // No route with owned stock: retry with catalog profiles whose segment inventory is 0.
  // Advice is given only when that complete route verifies clear.
  const totalStock=t=>Array.isArray(t.inventory)&&t.inventory.length?t.inventory.reduce((n,i)=>n+(i.count>0?i.count:0),0):
    (t.availableLengthsMm??t.stockLengthsMm??[]).length;
  const missing=catalog.tools.filter(t=>['punch','die'].includes(t.kind)&&totalStock(t)===0);
  if(!plan.completeGeometricRouteSampleClear&&plan.status!=='FLAT_NO_BENDS'&&!searchOptions.toolingSelection&&missing.length){
    const STANDARD_SET=[[406.4,1],[203.2,1],[101.6,2],[50.8,2],[25.4,1]];
    const trialCatalog=JSON.parse(JSON.stringify(catalog)),ids=new Set(missing.map(t=>String(t.id)));
    for(const t of trialCatalog.tools)if(ids.has(String(t.id)))
      t.inventory=STANDARD_SET.map(([lengthMm,count])=>({lengthMm,count}));
    const trial=await planCompleteSequence({model,catalog:trialCatalog,options:{...searchOptions,stopAtFirstClear:true,
      timeBudgetMs:Math.min(searchOptions.timeBudgetMs??30000,30000)}},report);
    if(trial.completeGeometricRouteSampleClear){
      const need=new Map();
      for(const stage of trial.stages??[]){const c=stage.config??{};
        for(const kind of['punch','die']){const id=String(c[`${kind}Id`]);if(!ids.has(id))continue;
          const counts=new Map();for(const s of c.segments??[])if(s.kind===kind)counts.set(s.lengthMm,(counts.get(s.lengthMm)??0)+1);
          const entry=need.get(id)??new Map();for(const [len,n]of counts)entry.set(len,Math.max(entry.get(len)??0,n));need.set(id,entry);}}
      plan.toolingAdvice=[...need].map(([id,lengths])=>{const t=catalog.tools.find(x=>String(x.id)===id);
        const segs=[...lengths].sort((a,b)=>b[0]-a[0]).map(([len,n])=>({lengthMm:len,count:n}));
        const text=segs.map(s=>`${s.count} x ${(s.lengthMm/25.4).toFixed(3).replace(/\.?0+$/,'')} in`).join(', ');
        return{kind:t.kind,toolId:id,toolName:t.name,vendorCode:t.vendorCode??null,notInInventory:true,segments:segs,
          verifiedStrokes:trial.stages.length,verifiedChanges:trial.cost?.toolChangeEvents??0,
          message:`Not feasible with current inventory: ${t.name} has 0 segments in stock. A clear route was verified with ${text} of this ${t.kind}.`};});
    }
  }
  annotateToolChanges(plan.stages);
  const frameSections=catalog.machine?.frameSections??[];
  plan.machineCollisionGeometry={
    tableProfile:catalog.tools.some(t=>t.kind==='table'&&t.status!=='missing'),
    ramSections:frameSections.filter(s=>s.kind==='ram').map(s=>s.id),
    lowerBedSections:frameSections.filter(s=>s.kind==='bed').map(s=>s.id)
  };
  await mkdir(dirname(output),{recursive:true});
  await writeFile(output,await serializePlanEvidence({schemaVersion:1,units:'mm',modelPath:resolve(options.model),modelSource:model.metadata??null,...plan},{model,catalog,options:baseOptions,indent}));
  stdout.write(JSON.stringify({status:plan.status,output,completeGeometricRouteSampleClear:plan.completeGeometricRouteSampleClear,
    stages:plan.stages.length,cost:plan.cost,elapsedMs:plan.diagnostics.completePlanElapsedMs,searchElapsedMs:plan.diagnostics.elapsedMs,budgetReason:plan.diagnostics.budgetReason??null,
    ...(plan.diagnostics.pairedClosure?{reason:plan.reason,blockedBendIds:plan.diagnostics.pairedClosure.pair}:{}),
    error:plan.diagnostics.error??null,transferError:plan.transferError??null})+'\n');
  return plan.completeGeometricRouteSampleClear||plan.status==='FLAT_NO_BENDS'?0:2;
}catch(error){stderr.write(JSON.stringify({status:'error',message:error.message})+'\n');return 1;}
}
