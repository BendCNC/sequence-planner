/** Validate a prepared geometric package before it reaches the renderer. */
export function validateModel(model) {
  const fail = message => { throw new Error(`Modelo no admitido: ${message}`); };
  if (!model || !Array.isArray(model.panels) || !Array.isArray(model.bends)) fail('faltan paneles o pliegues.');
  if (model.units && model.units !== 'mm') fail('las coordenadas preparadas deben estar en mm.');
  if (!model.panels.length || model.panels.length > 1000 || model.bends.length > 999) fail('cantidad de regiones fuera del límite.');
  if (!(model.thicknessMm > 0) || !Number.isFinite(model.thicknessMm)) fail('espesor inválido.');
  const validateK = k => { if (k !== undefined && (!Number.isFinite(k) || k < 0 || k > 1)) fail('factor K inválido.'); };
  validateK(model.kFactor);
  const ids = new Set(model.panels.map(p => String(p.id))), bends = new Set();
  if (ids.size !== model.panels.length || !ids.has(String(model.rootPanelId))) fail('identidad de paneles o raíz inválida.');
  const reached = new Set([String(model.rootPanelId)]), incoming = new Map();
  let vertices = 0;
  const vector = (v, name) => {
    if (!Array.isArray(v) || v.length !== 3 || !v.every(Number.isFinite)) fail(`${name}: vector inválido.`);
  };
  const mesh = (p, positionsKey, referenceOnly=false) => {
    const data=p[positionsKey], triangles=p.triangles;
    if (!Array.isArray(data) || data.length%3 || data.length<9 || !data.every(Number.isFinite)) fail('vértices inválidos.');
    if (!referenceOnly) vertices += data.length/3;
    if (vertices>1000000 || data.length/3>1000000) fail('más de un millón de vértices.');
    if (!Array.isArray(triangles) || triangles.length%3 || triangles.length<3 || !triangles.every(i=>Number.isInteger(i)&&i>=0&&i<data.length/3)) fail('triángulos inválidos.');
    if (p.sourcePositions && (p.sourcePositions.length!==data.length || !p.sourcePositions.every(Number.isFinite))) fail('referencia final incompatible.');
  };
  for (const p of model.panels) mesh(p,'flatPositions');
  if (model.stepReferenceMesh) mesh(model.stepReferenceMesh,'positions',true);
  for (const b of model.bends) {
    const id=String(b.id), parent=String(b.parentPanelId), child=String(b.childPanelId);
    if (bends.has(id) || !ids.has(parent) || !ids.has(child) || incoming.has(child) || child===String(model.rootPanelId)) fail('relaciones de pliegue duplicadas o inválidas.');
    bends.add(id); incoming.set(child,parent);
    for (const key of ['origin','axis','transverse','normal']) vector(b[key],key);
    for (const key of ['axis','transverse','normal']) if(Math.abs(Math.hypot(...b[key])-1)>1e-5)fail(`${key} no es unitario.`);
    const dot=(a,c)=>a.reduce((sum,v,i)=>sum+v*c[i],0);
    if (Math.max(Math.abs(dot(b.axis,b.transverse)),Math.abs(dot(b.axis,b.normal)),Math.abs(dot(b.transverse,b.normal)))>1e-5)fail('base de pliegue no ortogonal.');
    const cross=[b.axis[1]*b.transverse[2]-b.axis[2]*b.transverse[1],b.axis[2]*b.transverse[0]-b.axis[0]*b.transverse[2],b.axis[0]*b.transverse[1]-b.axis[1]*b.transverse[0]];
    if(dot(cross,b.normal)<.99999)fail('base de pliegue con orientación incompatible.');
    if (!(Math.abs(b.signedAngle)>1e-8 && Math.abs(b.signedAngle)<Math.PI-1e-8) || !Number.isFinite(b.signedAngle) || !(b.neutralLength>0) || !Number.isFinite(b.neutralLength) || !Number.isFinite(b.neutralZ)) fail('ángulo o banda inválidos: se requiere un barrido menor de 180°.');
    if (!(b.insideRadius>=0) || !Number.isFinite(b.insideRadius))fail('radio inválido.');
    mesh(b.band,'parameters');
    validateK(b.kFactor);
    if(b.band.parameters.some((v,i)=>i%3===1&&(v < -1e-6 || v > 1+1e-6)))fail('fracción material fuera de la banda.');
    if (b.band.flatPositions && (b.band.flatPositions.length!==b.band.parameters.length || !b.band.flatPositions.every(Number.isFinite))) fail('banda plana inválida.');
  }
  let changed=true;
  while(changed){changed=false;for(const b of model.bends)if(reached.has(String(b.parentPanelId))&&!reached.has(String(b.childPanelId))){reached.add(String(b.childPanelId));changed=true;}}
  if(reached.size!==ids.size || model.bends.length!==model.panels.length-1)fail('grafo desconectado, cíclico o con restricciones cerradas.');
  return {panels:ids.size,bends:bends.size,vertices};
}
