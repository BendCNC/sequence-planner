/** Check numerical fold evidence at the boundary of the desktop JSON adapter. */
import {consistentSupportCentroid,buildOperatorSupport,evaluateOperatorSupport} from './sequence-operator-support.mjs';
import {checkScene,sceneWithAmounts} from './sequence-scene.mjs';

// The operator evaluator emits the three coordinates immediately before the
// public X alias. An ordinary export only scans the text; parsing a complete
// document again is reserved for an actual disagreement in serialized bytes.
const pairs=/"centroid"\s*:\s*\[\s*([^,\]]+)\s*,\s*[^,\]]+\s*,\s*[^,\]]+\s*\]\s*,\s*"centroidX"\s*:\s*([-+.0-9eE]+|null)/g;
export function inconsistentSerializedCentroid(text){
  for(const match of text.matchAll(pairs)){
    if(!Number.isFinite(Number(match[1]))||match[2]==='null'||Number(match[1])!==Number(match[2]))return true;
  }
  return false;
}
function inconsistentSamples(plan){
  const rows=[];
  for(const [i,stage] of (plan.stages??[]).entries())for(const [j,sample] of (stage.foldSamples??[]).entries()){
    const s=sample.operatorSupport;if(s&&('centroidX' in s||'centroid' in s)&&!consistentSupportCentroid(s))rows.push([i,j]);
  }
  return rows;
}
async function replay(rows,plan,{model,catalog,options}){
  const {normalizedAdvancedModel}=await import('./sequence-advanced-search.mjs');
  const geometry=normalizedAdvancedModel(model),prepared=new Map();
  for(const [i,j] of rows){
    const stage=plan.stages[i],sample=stage.foldSamples[j],bendId=stage.bendId;
    if(!prepared.has(bendId))prepared.set(bendId,buildOperatorSupport(geometry,bendId));
    const scene=sceneWithAmounts(geometry,catalog,{...stage.config,bendId,ramGapMm:0},sample.amounts);
    const support=evaluateOperatorSupport(prepared.get(bendId),scene);
    if(sample.status!=='SAMPLE_CLEAR'||!support.passes||!consistentSupportCentroid(support)||
       checkScene(geometry,scene,{clearanceMm:options?.clearanceMm??.15}).status!=='SAMPLE_CLEAR')
      throw new Error('Unable to verify fold evidence at stage '+(i+1)+', sample '+j);
    sample.operatorSupport=support;
  }
}
export async function serializePlanEvidence(plan,settings={}){
  const rows=inconsistentSamples(plan);
  if(rows.length)await replay(rows,plan,settings);
  let text=JSON.stringify(plan,null,settings.indent);
  if(!inconsistentSerializedCentroid(text))return text;
  const serializedRows=inconsistentSamples(JSON.parse(text));
  if(serializedRows.length)await replay(serializedRows,plan,settings);
  // A fresh geometry reading and the alternative serialization path must
  // agree on the actual emitted numbers before any bytes are written.
  text=JSON.stringify(plan,(key,value)=>value,settings.indent);
  if(inconsistentSerializedCentroid(text))throw new Error('Published operator-support evidence is inconsistent');
  return text;
}

