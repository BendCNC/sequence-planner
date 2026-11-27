import {sceneWithAmounts} from './sequence-scene.mjs';
import {planBackgauge} from './backgauge.mjs';

export function checkStageGauge(model,catalog,{config,from,settings,park,clearance}){
  const scene=sceneWithAmounts(model,catalog,config,from);
  // Widest accepts only a clear two-contact layout (the maximum), then
  // falls back to the same standard layout and full checks otherwise.
  const backgauge=planBackgauge({scene,definition:catalog.fingerDefinition,settings,park,
    clearanceMm:clearance,widest:!settings.fixedStations});
  const panelX=Object.fromEntries(scene.bodies.filter(b=>b.kind==='panel').map(b=>{
    let sum=0;
    for(let i=0;i<b.positions.length;i+=3)sum+=b.positions[i];
    return [b.panelId,sum/(b.positions.length/3)];
  }));
  return {backgauge,panelX};
}
