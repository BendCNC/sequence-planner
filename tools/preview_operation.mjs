/** Checked preview of one stroke (geometry only, no machine commands). Explains where it collides. */
import {readFile,writeFile,mkdir} from 'node:fs/promises';
import {resolve,dirname} from 'node:path';
import {fileURLToPath} from 'node:url';
import {checkOperationPreview} from '../viewer/operation-preview.mjs';
import {readCatalogJson} from '../viewer/protected-json.mjs';

export const bendLabel=(model,bend)=>bend?.label??'B'+(model.bends.indexOf(bend)+1);

/** Picks the stroke setup: a plan stage (index or bend id), else the first collision witness for that bend. */
export function selectOperation(model,plan,stage){
  const stages=plan?.stages??[],key=String(stage);
  const bendOf=s=>String(s.bendId??s.config?.bendId),labelled=model.bends.find(b=>bendLabel(model,b)===key);
  // A bend id or label wins over a numeric stage index (fixture ids can be '1', '2', ...).
  let index=stages.findIndex(s=>bendOf(s)===key||labelled&&bendOf(s)===String(labelled.id));
  if(index<0&&!model.bends.some(b=>String(b.id)===key)&&/^\d+$/.test(key)&&Number(key)<stages.length)index=Number(key);
  if(index>=0){
    const s=stages[index],bendId=String(s.bendId??s.config.bendId),done=stages.slice(0,index).map(x=>String(x.bendId??x.config.bendId));
    const order=[...done,bendId,...model.bends.map(b=>String(b.id)).filter(id=>id!==bendId&&!done.includes(id))];
    return{source:'stage',stageIndex:index,bendId,config:{...s.config,bendId,order}};
  }
  const bend=model.bends.find(b=>String(b.id)===key||bendLabel(model,b)===key);
  if(!bend)return null;
  const witnesses=[...(plan?.diagnostics?.collisionWitnesses??[]),...(plan?.diagnostics?.rejectionExamples??[]).map(r=>r.witness)];
  const w=witnesses.find(x=>x?.config&&String(x.bendId)===String(bend.id));
  if(!w)return null;
  const done=model.bends.filter(b=>b.id!==bend.id&&(w.amounts?.[b.id]??0)>=1).map(b=>String(b.id));
  const order=[...done,String(bend.id),...model.bends.map(b=>String(b.id)).filter(id=>id!==String(bend.id)&&!done.includes(id))];
  const {amount,ramGapMm,contactGapMm,...config}=w.config;
  return{source:'collision-witness',stageIndex:null,bendId:String(bend.id),config:{...config,bendId:String(bend.id),order}};
}

/** Human name of a scene body: bend label, flange of a bend, or tool kind. */
export function describeBody(model,id,kind){
  const s=String(id);
  if(s.startsWith('band:')){const b=model.bends.find(x=>String(x.id)===s.slice(5));return'bend '+(b?bendLabel(model,b):s.slice(5));}
  if(s.startsWith('panel:')){
    const p=s.slice(6);if(p===model.rootPanelId)return'base flange';
    const b=model.bends.find(x=>x.childPanelId===p);return b?'flange '+bendLabel(model,b):'flange';
  }
  return({punch:'punch',die:'die',finger:'backgauge finger',table:'table',riser:'riser'})[kind]??kind??s;
}

export function summarizePreview(model,selection,preview){
  const bend=model.bends.find(b=>String(b.id)===selection.bendId),w=preview.witness;
  const out={status:preview.status==='OPERATION_SAMPLE_CLEAR'?'CLEAR':w?.collision?.point?'INTERFERENCE':'BLOCKED',
    bendId:selection.bendId,bendLabel:bend?bendLabel(model,bend):null,source:selection.source,stageIndex:selection.stageIndex,
    reason:preview.reason??null,error:preview.error??null,collision:null,elapsedMs:preview.elapsedMs};
  if(w?.collision?.point){
    const c=w.collision,ids=[].concat(c.bodyIds??[]),kinds=[].concat(c.bodyKinds??[]);
    const fold=w.phase==='fold'?Number(w.amounts?.[selection.bendId]):null;
    out.collision={phase:w.phase,point:c.point.map(Number),bodyIds:ids,parts:ids.map((id,i)=>describeBody(model,id,kinds[i])),
      category:c.category??null,progress:Number.isFinite(fold)?fold:Number.isFinite(preview.fraction)?preview.fraction:null,
      ramGapMm:w.config?.ramGapMm??null};
  }
  return out;
}

const isMain=process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url);
if(isMain){
  const args=process.argv.slice(2),options={};
  const allowed=new Set(['model','catalog','output','stage','plan','time-ms']);
  try{main:{
    for(let i=0;i<args.length;i+=2){
      const key=args[i].replace(/^--/,'');
      if(!args[i].startsWith('--')||!allowed.has(key)||args[i+1]===undefined||args[i+1].startsWith('--'))throw new Error('Invalid argument: '+args[i]);
      options[key]=args[i+1];
    }
    if(!options.model||!options.output||options.stage===undefined)throw new Error('Use --model model.json --output preview.json --stage <index|bendId> [--plan plan.json] [--catalog tooling.json] [--time-ms 120000]');
    const json=async path=>JSON.parse(await readFile(path,'utf8'));
    const model=await json(options.model),catalog=await readCatalogJson(options.catalog??fileURLToPath(new URL('../viewer/tooling.json',import.meta.url)));
    const plan=options.plan?await json(options.plan):null,output=resolve(options.output);
    const selection=selectOperation(model,plan,options.stage);
    await mkdir(dirname(output),{recursive:true});
    if(!selection){
      const result={schemaVersion:1,units:'mm',status:'INPUT_MISMATCH',bendId:null,stage:String(options.stage),
        error:'No plan stage or collision witness for this stage/bend.',collision:null};
      await writeFile(output,JSON.stringify(result,null,2));process.stdout.write(JSON.stringify(result)+'\n');process.exitCode=3;break main;
    }
    const timeBudgetMs=options['time-ms']?Number(options['time-ms']):undefined;
    if(timeBudgetMs!==undefined&&!(timeBudgetMs>0))throw new Error('Invalid time-ms');
    const preview=await checkOperationPreview({model,catalog,config:selection.config,options:timeBudgetMs?{timeBudgetMs}:{}},
      p=>process.stderr.write(JSON.stringify({type:'progress',...p})+'\n'));
    const result={schemaVersion:1,units:'mm',...summarizePreview(model,selection,preview)};
    await writeFile(output,JSON.stringify(result,null,2));
    process.stdout.write(JSON.stringify(result)+'\n');
    process.exitCode=preview.reason==='INVALID_OPERATION_INPUT'?3:0;
  }}catch(error){process.stderr.write(JSON.stringify({status:'error',message:error.message})+'\n');process.exitCode=1;}
}
