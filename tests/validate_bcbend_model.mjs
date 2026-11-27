/** Independent consumer integration: actual JS solver, validator and collider. */
import assert from 'node:assert/strict';
import fs from 'node:fs';
import {validateModel} from '../viewer/model-validator.mjs';
import {evaluateModel, transformedPanel, transform} from '../viewer/solver.mjs';
import {meshCollider} from '../viewer/collision.mjs';

for (const path of process.argv.slice(2)) {
  const m=JSON.parse(fs.readFileSync(path,'utf8'));
  validateModel(m);
  assert.equal(m.metadata.independentFinalShapeVerified,false);
  assert.equal(m.metadata.sourceUsesStep,false);
  assert.ok(m.panels.every(p=>p.sourcePositions===undefined));
  const states=[...Array.from({length:7},(_,i)=>Object.fromEntries(m.bends.map(b=>[b.id,i/6]))),
    ...m.bends.map((_,cut)=>Object.fromEntries(m.bends.map((b,i)=>[b.id,i<cut?1:i===cut?.37:0])))];
  let maximumSeamGapMm=0;
  for (const amounts of states) {
    const state=evaluateModel(m,0,{amounts});
    const positions=new Map();
    for(const p of m.panels)positions.set(`panel:${p.id}`,transformedPanel(p,state.panelMatrices[p.id]));
    for(const b of m.bends)positions.set(`band:${b.id}`,state.bandPositions[b.id]);
    for(const data of positions.values())assert.ok(data.every(Number.isFinite));
    for(const seam of m.seams){
      const ps=seam.map(ref=>Array.from(positions.get(`${ref.kind}:${ref.id}`).slice(3*ref.vertexIndex,3*ref.vertexIndex+3)));
      for(const p of ps)maximumSeamGapMm=Math.max(maximumSeamGapMm,Math.hypot(...p.map((v,i)=>v-ps[0][i])));
    }
    for(const p of m.panels){
      const collider=meshCollider({id:p.id,positions:positions.get(`panel:${p.id}`),triangles:p.triangles});
      assert.equal(collider.closed,true,`${path}: panel ${p.id}: ${JSON.stringify(collider.diagnostics)}`);
    }
    for(const b of m.bends){
      const collider=meshCollider({id:b.id,positions:positions.get(`band:${b.id}`),triangles:b.band.triangles});
      assert.equal(collider.closed,true,`${path}: band ${b.id}: ${JSON.stringify(collider.diagnostics)}`);
    }
  }
  assert.ok(maximumSeamGapMm<1e-5);
  // Independently verify the analytical final radii, using the neutral-axis
  // centre and the solver rather than an importer-generated reference mesh.
  for(const b of m.bends){
    const q=evaluateModel(m,0,{amounts:Object.fromEntries(m.bends.map(x=>[x.id,x.id===b.id?1:0]))});
    const center=b.origin.map((x,i)=>x+b.normal[i]*(b.neutralZ+b.neutralLength/b.signedAngle));
    const data=b.band.parameters,final=q.bandPositions[b.id];
    for(let i=0;i<data.length;i+=3){
      const axisPoint=center.map((x,j)=>x+b.axis[j]*data[i]);
      const point=Array.from(final.slice(i,i+3));
      const radius=Math.hypot(...point.map((x,j)=>x-axisPoint[j]));
      const expected=b.insideRadius+((b.signedAngle>0?data[i+2]<0:data[i+2]===0)?m.thicknessMm:0);
      assert.ok(Math.abs(radius-expected)<1e-7,`radius ${radius} vs ${expected}`);
    }
  }
  console.log(JSON.stringify({file:path,states:states.length,maximumSeamGapMm,status:'PASS'}));
}
