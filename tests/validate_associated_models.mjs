/** Exercise associated CAD through the independent JavaScript consumer. */
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {validateModel} from '../viewer/model-validator.mjs';
import {evaluateModel,transformedPanel,endpointAudit} from '../viewer/solver.mjs';

const results=[];
for(const path of process.argv.slice(2)){
  const model=JSON.parse(readFileSync(path,'utf8'));
  const validation=validateModel(model);
  assert.equal(model.metadata.stepAssociation.status,'matched');
  assert.equal(model.metadata.sourceUsesStep,true);
  assert.equal(model.metadata.geometryAuthority,'embedded-step');
  const final=endpointAudit(model);
  assert.ok(final.maxDeviationMm<1e-5,`Source STEP endpoint mismatch: ${final.maxDeviationMm}`);
  const states=[...Array.from({length:9},(_,i)=>Object.fromEntries(model.bends.map(b=>[b.id,i/8]))),
    ...model.bends.map((_,cut)=>Object.fromEntries(model.bends.map((b,i)=>[b.id,i<cut?1:i===cut?.37:0]))),
    Object.fromEntries(model.bends.map((b,i)=>[b.id,((i*13+5)%29)/29]))];
  let maximumSeamGapMm=0,coordinatesChecked=0;
  for(const amounts of states){
    const state=evaluateModel(model,0,{amounts}),positions=new Map();
    for(const p of model.panels)positions.set(`panel:${p.id}`,transformedPanel(p,state.panelMatrices[p.id]));
    for(const b of model.bends)positions.set(`band:${b.id}`,state.bandPositions[b.id]);
    for(const data of positions.values()){
      assert.ok(data.every(Number.isFinite),'Nonfinite geometry after reassignment');coordinatesChecked+=data.length;
    }
    for(const seam of model.seams??[]){
      const points=seam.map(ref=>{
        const data=positions.get(`${ref.kind==='bend'?'band':ref.kind}:${ref.id}`);
        assert.ok(data,`Missing remapped seam region ${ref.kind}:${ref.id}`);
        assert.ok(ref.vertexIndex>=0&&3*ref.vertexIndex+2<data.length);
        return Array.from(data.slice(3*ref.vertexIndex,3*ref.vertexIndex+3));
      });
      for(const p of points)maximumSeamGapMm=Math.max(maximumSeamGapMm,Math.hypot(...p.map((v,i)=>v-points[0][i])));
    }
  }
  assert.ok(maximumSeamGapMm<1e-5,`Broken remapped seams: ${maximumSeamGapMm}`);
  results.push({path,...validation,states:states.length,coordinatesChecked,
    maximumFinalSourceErrorMm:final.maxDeviationMm,maximumSeamGapMm,status:'PASS'});
}
assert.ok(results.length,'At least one associated model is required');
console.log(JSON.stringify({status:'PASS',models:results},null,2));
