/** Inventory-qualified banks following actual material islands. Necessary
 * component filters only prune; accepted setups still require full fold checks. */
import {toolAllowedAt} from './tooling-rules.mjs';
import {strokeMaterialIntervals as materialIntervals,linkJointStrokes,createEndpointSceneBuilder,buildMountedBank,toolById} from './machine.mjs';
import {strokeAllowed,strokeStateError} from './sequence-stroke.mjs';
import {isPart,checkScene} from './sequence-scene.mjs';
import {meshCollider,checkPair} from './collision.mjs';
import {buildOperatorSupport,evaluateOperatorSupport} from './sequence-operator-support.mjs';
import {assessCoverage} from './coverage.mjs';
import {assessToolSupport,toolSupportPolicy} from './tool-support-policy.mjs';
import {formingSegmentSeated,generateCenteredStockBanks} from './tool-setup-search.mjs';
export const advancedConfigKey=c=>JSON.stringify([c.punchId,c.dieId,c.stationYmm??0,Boolean(c.flip),c.activeBankId??null,c.segments.map(s=>[s.kind,s.lengthMm,s.centerY,s.mirror??false,s.offsetX??0,s.offsetZ??0,s.bankId??null,s.punchId??null])]);

// Frame placement at this endpoint is independent of the axial tool bank.
// Boxes can prove separation. Only the actual triangles can prove obstruction.
function partFrameHit(scene,kind,clearanceMm){
 const frame=scene.bodies.map((b,i)=>({body:b,box:scene.bounds[i]}))
   .filter(({body})=>kind==='punch'?body.kind==='ram':body.kind==='table'||body.kind==='bed');
 for(let i=0;i<scene.bodies.length;i++){
  const part=scene.bodies[i];if(!isPart(part))continue;
  const box=scene.bounds[i];let collider;
  for(const member of frame){
   if(box.min.some((v,k)=>v>member.box.max[k]+clearanceMm+1e-7||box.max[k]<member.box.min[k]-clearanceMm-1e-7))continue;
   member.collider??=meshCollider(member.body);
   collider??=meshCollider(part);
   const hit=checkPair(collider,member.collider,{clearanceMm,queryMode:'clearance'});
   if(hit.status==='INTERFERENCE')return hit;
  }
 }
 return null;
}
// The flank of an extruded tool is the same X/Z edge throughout its axial
// span. A cap point of a closed prism also belongs to its filled profile at
// every interior Y. Reuse a witness only strictly inside the next segment
// and strictly inside the clearance threshold.
function sideObstructionStation(bodies,hit,clearance){
 if(!hit||hit.reason!=='WITHIN_CLEARANCE'||!hit.triangleIds||!hit.closestPoints)return null;
 for(const body of bodies){
  const side=hit.bodyIds.indexOf(body.id);if(side<0)continue;
  const face=hit.triangleIds[side],p=hit.closestPoints[1-side],points=[];
  for(let i=0;i<3;i++){
   const index=body.triangles[face*3+i]*3;points.push([body.positions[index],body.positions[index+1],body.positions[index+2]]);
  }
  const [a,b,c]=points;
  if(a[1]===b[1]&&a[1]===c[1]){
   const q=hit.closestPoints[side];
   const slack=1e-7+512*Number.EPSILON*Math.max(...p.map(Math.abs),...q.map(Math.abs));
   if(Math.hypot(p[0]-q[0],p[2]-q[2])+slack<clearance&&meshCollider(body).closed)return{y:p[1],slack};
   continue;
  }
  const end=a[0]!==b[0]||a[2]!==b[2]?b:c,dx=end[0]-a[0],dz=end[2]-a[2],len2=dx*dx+dz*dz;
  if(!(len2>0))continue;
  const t=Math.min(1,Math.max(0,((p[0]-a[0])*dx+(p[2]-a[2])*dz)/len2));
  const gap=Math.hypot(p[0]-(a[0]+t*dx),p[2]-(a[2]+t*dz));
  const slack=1e-7+512*Number.EPSILON*Math.max(...p.map(Math.abs),...a.map(Math.abs),...end.map(Math.abs));
  if(gap+slack<clearance)return{y:p[1],slack};
 }
 return null;
}
/** Same round-robin order as materializing each profile pair's Cartesian
 * product, but storage grows with banks and profile pairs, not bank pairs. */
export function* advancedBankPairs(punches,dies){
 const group=values=>{const map=new Map();for(const value of values){if(!map.has(value.toolId))map.set(value.toolId,[]);map.get(value.toolId).push(value);}return[...map.values()];};
 const punchGroups=group(punches),dieGroups=group(dies),groups=[];let maximum=0;
 for(const p of punchGroups)for(const d of dieGroups){const length=p.length*d.length;maximum=Math.max(maximum,length);groups.push({p,d,length});}
 for(let i=0;i<maximum;i++)for(const g of groups)if(i<g.length)yield[g.p[Math.floor(i/g.d.length)],g.d[i%g.d.length]];
}
/** Filtered, finite profile streams in the same order as advancedBankPairs.
 * Each source represents ONE profile group. A die's length is needed only
 * after requesting index i exhausts it; until then the pair uses punch 0.
 * Memoized streams are shared by every pair involving that profile.
 */
export function* advancedLazyBankPairs(punchSources,dieSources){
 const memo=source=>({values:[],iterator:null,done:false,
  at(index){
   while(this.values.length<=index&&!this.done){
    this.iterator??=source[Symbol.iterator]();
    const next=this.iterator.next();if(next.done)this.done=true;else this.values.push(next.value);
   }
   return this.values[index];
  }});
 const punches=punchSources.map(memo),dies=dieSources.map(memo);
 try{
  for(let i=0;;i++){
   let any=false;
   for(const p of punches){
    const first=p.at(0);if(first===undefined)continue;
    for(const d of dies){
     const next=d.at(i);
     if(next!==undefined){any=true;yield[first,next];continue;}
     const length=d.values.length;if(!length)continue;
     const punch=p.at(Math.floor(i/length));if(punch===undefined)continue;
     any=true;yield[punch,d.values[i%length]];
    }
   }
   if(!any)return;
  }
 }finally{
  for(const stream of [...punches,...dies])if(!stream.done)stream.iterator?.return?.();
 }
}
export function advancedInventoryFits(catalog,config,bedLengthMm){
 if(!Array.isArray(config.segments)||!Number.isFinite(config.stationYmm??0))return false;
 if(config.segments.some(s=>s.id===undefined||s.id===null||String(s.id).length===0)||new Set(config.segments.map(s=>String(s.id))).size!==config.segments.length)return false;
 for(const s of config.segments)if(!['punch','die'].includes(s.kind)||!formingSegmentSeated(s,bedLengthMm)||![s.offsetX??0,s.offsetZ??0].every(v=>Number.isFinite(v)&&Math.abs(v)<1e-8))return false;
 for(const kind of ['punch','die']){
  const tool=catalog.tools.find(t=>String(t.id)===String(config[kind+'Id'])),segments=config.segments.filter(s=>s.kind===kind);if(!tool||!segments.length)return false;
  if(kind==='punch'&&segments.some(s=>s.punchId&&String(s.punchId)!==String(tool.id)))return false;
  const stock=tool.inventory?.length?tool.inventory:(tool.availableLengthsMm??tool.stockLengthsMm??[]).map(lengthMm=>({lengthMm,count:1})),need=new Map();
  for(const s of segments)need.set(s.lengthMm,(need.get(s.lengthMm)??0)+1);
  for(const [length,count]of need)if(stock.reduce((sum,s)=>sum+(Math.abs(s.lengthMm-length)<1e-7&&Number.isInteger(s.count)&&s.count>0?s.count:0),0)<count)return false;
  for(let i=0;i<segments.length;i++)for(let j=0;j<i;j++)if(Math.abs(segments[i].centerY-segments[j].centerY)<(segments[i].lengthMm+segments[j].lengthMm)/2-1e-7)return false;
 }return true;
}
export function advancedCompatibleTools(catalog,bend,kind,options={},amount=1){
 return catalog.tools.filter(t=>t.kind===kind&&toolAllowedAt(t,bend,options,amount)).sort((a,b)=>{
   // The press's preferred (mounted) tool first: fewer changes from the current setup.
   const pref=options[kind==='punch'?'preferredPunchId':'preferredDieId'],pa=pref!=null&&String(a.id)===String(pref),pb=pref!=null&&String(b.id)===String(pref);
   return(pb-pa)||(a.areaMm2??Infinity)-(b.areaMm2??Infinity);});
}
/** Enable the extra stock family for the WHOLE part when any physical stroke
 * spans beyond the bed, or a supplied/current bank already overhangs it. This
 * is candidate enumeration only: seating, inventory and collision checks still
 * apply. Summing the catalog inventory would enable this for every short part.
 * Callers may explicitly opt in/out with allowFullStockBanks. */
export function shouldGenerateFullStockBanks({model,bedLengthMm,options={},preferred=[]}){
 if(options.allowFullStockBanks!==undefined)return options.allowFullStockBanks===true;
 const mounted=[options.initialSetup,...preferred,...(options.setups??[])].map(s=>s?.config??s);
 if(options.initialSegments)mounted.push({segments:options.initialSegments});
 if(mounted.some(c=>c?.segments?.some(s=>Number.isFinite(s.lengthMm)&&Number.isFinite(s.centerY)&&
   Math.abs(s.centerY)+s.lengthMm/2>bedLengthMm/2+1e-7)))return true;
 return model.bends.some(b=>{
  if(b.strokeLeadId!=null)return false;
  const material=materialIntervals(b);
  return material.exact&&material.intervals.length>0&&material.intervals.at(-1)[1]-material.intervals[0][0]>bedLengthMm+1e-7;
 });
}
export function generateIslandBanks(tool,intervals,{bedLengthMm,maxUnsupportedEndMm=12.7,maxIslandLayoutsPerTool=1200,allowMirroredToolProfiles=true,maxAdvancedSegments=4,
 allowContiguousIslandBanks=true,maxContiguousIslandLayoutsPerTool=256,maxContiguousIslandNodesPerTool=4096,
 allowFullStockBanks,maxFullBanksPerTool=6}={}){
 const stock=(tool.inventory?.length?tool.inventory:(tool.availableLengthsMm??tool.stockLengthsMm??[]).map(lengthMm=>({lengthMm,count:1}))).filter(s=>Number.isFinite(s.lengthMm)&&s.lengthMm>0&&Number.isInteger(s.count)&&s.count>0).sort((a,b)=>a.lengthMm-b.lengthMm);
 const layouts=[],seen=new Set(),span=intervals.at(-1)[1]-intervals[0][0],limit=Math.max(span*2,stock[0]?.lengthMm??0);
 const groups=[[[intervals[0][0],intervals.at(-1)[1]]],...(intervals.length>1&&intervals.length<=maxAdvancedSegments?[intervals]:[])];
 for(const islands of groups){
  function visit(i,segments,used){
   if(layouts.length>=maxIslandLayoutsPerTool)return;
   if(i===islands.length){
    const surrogate=segments.flatMap(s=>[{...s,kind:'punch'},{...s,kind:'die'}]);
    if(!assessToolSupport(assessCoverage({materialIntervals:intervals,segments:surrogate,exact:true}),{maxUnsupportedEndMm}).accepted)return;
    const key=JSON.stringify(segments.map(s=>[s.lengthMm,s.centerY]));if(!seen.has(key)){seen.add(key);layouts.push(segments);}return;
   }
   const [a,b]=islands[i];for(const item of stock){
    if(item.lengthMm>limit+1e-7||(used.get(item.lengthMm)??0)>=item.count)continue;
    const lo=b-item.lengthMm/2-(i===islands.length-1?maxUnsupportedEndMm:0),hi=a+item.lengthMm/2+(i===0?maxUnsupportedEndMm:0);if(lo>hi+1e-7)continue;
    const centers=[(a+b)/2,a+item.lengthMm/2,b-item.lengthMm/2,...[.5,.25,.75,0,1].map(t=>lo+(hi-lo)*t)];
    for(const centerY of [...new Set(centers.map(v=>Number(v.toFixed(9))))]){
     if(centerY<lo-1e-7||centerY>hi+1e-7||Math.abs(centerY)+item.lengthMm/2>bedLengthMm/2+1e-7||segments.some(s=>Math.abs(s.centerY-centerY)<(s.lengthMm+item.lengthMm)/2-1e-7))continue;
     const next=new Map(used);next.set(item.lengthMm,(next.get(item.lengthMm)??0)+1);visit(i+1,[...segments,{id:`${tool.kind}-island-${i}`,kind:tool.kind,lengthMm:item.lengthMm,centerY,offsetX:0,offsetZ:0}],next);
    }
   }
  }visit(0,[],new Map());
 }
 // Append new layouts only after the historical one-segment-per-island pass.
 // Its complete retained prefix and ordering remain unchanged. The additional
 // family has its own small count/node limits; it does not consume that prefix.
 if(allowContiguousIslandBanks&&maxAdvancedSegments>=2){
  const counts=new Map();for(const s of stock)counts.set(s.lengthMm,(counts.get(s.lengthMm)??0)+s.count);
  const lengths=[...counts.keys()],originalCount=layouts.length;let nodes=0;
  const exhausted=()=>layouts.length-originalCount>=maxContiguousIslandLayoutsPerTool||nodes>=maxContiguousIslandNodesPerTool;
  function* choices(){
   for(let a=0;a<lengths.length;a++)for(let b=a;b<lengths.length;b++){
    if(a===b&&counts.get(lengths[a])<2)continue;
    yield[lengths[a],lengths[b]];if(a!==b)yield[lengths[b],lengths[a]];
   }
   for(const length of lengths)yield[length];
  }
  for(const islands of groups){
   function visitContiguous(i,segments,used,hasPair){
    if(exhausted())return;nodes++;
    if(i===islands.length){
     if(!hasPair)return;
     const surrogate=segments.flatMap(s=>[{...s,kind:'punch'},{...s,kind:'die'}]);
     if(!assessToolSupport(assessCoverage({materialIntervals:intervals,segments:surrogate,exact:true}),{maxUnsupportedEndMm}).accepted)return;
     const key=JSON.stringify(segments.map(s=>[s.lengthMm,s.centerY]));if(!seen.has(key)){seen.add(key);layouts.push(segments);}return;
    }
    const[a,b]=islands[i];for(const bank of choices()){
     if(exhausted())return;
     if(segments.length+bank.length+islands.length-i-1>maxAdvancedSegments)continue;
     const need=new Map(used);for(const length of bank)need.set(length,(need.get(length)??0)+1);
     if([...need].some(([length,count])=>count>counts.get(length)))continue;
     const length=bank.reduce((sum,v)=>sum+v,0);if(length>limit+1e-7||length>bedLengthMm+1e-7)continue;
     const lo=b-length/2-(i===islands.length-1?maxUnsupportedEndMm:0),hi=a+length/2+(i===0?maxUnsupportedEndMm:0);if(lo>hi+1e-7)continue;
     const centers=[(a+b)/2,a+length/2,b-length/2,...[.5,.25,.75,0,1].map(t=>lo+(hi-lo)*t)];
     for(const center of [...new Set(centers.map(v=>Number(v.toFixed(9))))]){
      if(center<lo-1e-7||center>hi+1e-7)continue;
      let left=center-length/2;
      const added=bank.map((lengthMm,j)=>{const segment={id:`${tool.kind}-contiguous-${i}-${j}`,kind:tool.kind,lengthMm,centerY:left+lengthMm/2,offsetX:0,offsetZ:0};left+=lengthMm;return segment;});
      if(added.some(s=>Math.abs(s.centerY)+s.lengthMm/2>bedLengthMm/2+1e-7||segments.some(old=>Math.abs(old.centerY-s.centerY)<(old.lengthMm+s.lengthMm)/2-1e-7)))continue;
      visitContiguous(i+1,[...segments,...added],need,hasPair||bank.length===2);
     }
    }
   }
   visitContiguous(0,[],new Map(),false);if(exhausted())break;
  }
 }
 const result=layouts.flatMap(segments=>(allowMirroredToolProfiles?[false,true]:[false]).map(mirror=>({toolId:tool.id,segments:segments.map(s=>({...s,mirror,...(tool.kind==='punch'?{punchId:tool.id}:{})}))})));
 // Keep all historical island/contiguous layouts first. The normal stock
 // generator additionally supplies multi-section long banks, with the two
 // longest sections at their ends. No machine geometry or clearance changes.
 if(allowFullStockBanks??span>bedLengthMm+1e-7){
  const keys=new Set(result.map(b=>JSON.stringify([0,b.segments.map(s=>[s.lengthMm,s.centerY,!!s.mirror])])));
  for(const bank of generateCenteredStockBanks(tool,intervals,{bedLengthMm,maxFullBanksPerTool,maxUnsupportedEndMm})){
   if(bank.segments.length>maxAdvancedSegments||bank.segments.length<3&&bank.totalLengthMm<=bedLengthMm+1e-7)continue;
   const shifted=intervals.map(([a,b])=>[a+bank.stationYmm,b+bank.stationYmm]);
   const surrogate=bank.segments.flatMap(s=>[{...s,kind:'punch'},{...s,kind:'die'}]);
   if(!assessToolSupport(assessCoverage({materialIntervals:shifted,segments:surrogate,exact:true}),{maxUnsupportedEndMm}).accepted)continue;
   for(const mirror of(allowMirroredToolProfiles?[false,true]:[false])){
    const segments=bank.segments.map((s,i)=>({...s,id:`${tool.kind}-stock-bank-${i}`,mirror,...(tool.kind==='punch'?{punchId:tool.id}:{})}));
    const key=JSON.stringify([bank.stationYmm,segments.map(s=>[s.lengthMm,s.centerY,!!s.mirror])]);if(keys.has(key))continue;keys.add(key);
    result.push({toolId:tool.id,segments,stationYmm:bank.stationYmm,setupFamily:'CENTERED_STOCK_BANK'});
   }
  }
 }
 return result;
}

export function findAdvancedEndpointSetups(payload,guard=()=>{}){
 const stream=streamAdvancedEndpointSetups(payload,guard);
 for(let step=stream.next();;step=stream.next())if(step.done)return step.value;
}

/** Yield only a guaranteed prefix of retained configurations. The return value
 * always contains the full retained list when the search runs to completion. */
export function* streamAdvancedEndpointSetups({model,catalog,bendId,amounts,options={},preferred=[],terminalRow},guard=()=>{},observed={}){
 linkJointStrokes(model);
 const bend=model.bends.find(b=>String(b.id)===String(bendId)),material=materialIntervals(bend),prepared=buildOperatorSupport(model,bend.id),bedLengthMm=options.bedLengthMm??catalog.machine?.bedLengthMm;
 if(!(bedLengthMm>0))throw new Error('bedLengthMm is required');
 const punches=advancedCompatibleTools(catalog,bend,'punch',options,amounts[bend.id]),dies=advancedCompatibleTools(catalog,bend,'die',options,amounts[bend.id]),configs=[],seen=new Set(),diagnostics={componentsTested:0,combinationsTested:0,selfRejected:0,frontRejected:0};
 Object.assign(observed,{configs,diagnostics});
 if(strokeStateError(model,amounts)||!strokeAllowed(model,bend,amounts))return{configs,diagnostics:{...diagnostics,jointRejected:1}};
 if(!punches.length||!dies.length||!material.exact)return{configs,diagnostics};
 const sceneAt=createEndpointSceneBuilder(model,catalog,bend.id,amounts),immutablePartColliders=new WeakMap();
 const base={bendId:bend.id,punchId:punches[0].id,dieId:dies[0].id,bedLengthMm,stationYmm:0,contactGapMm:options.contactGapMm??.3,ramGapMm:0,amount:0};
 // Within this endpoint the active bend and all fold amounts are fixed. The
 // builder transforms those same part meshes only by placement.frame; banks,
 // tools and frame sections are still checked on every assembly. Key the
 // exact frame so a different die height, station or orientation is rechecked.
 const selfClearByFrame=new Map(),clearanceMm=options.clearanceMm??.15;
 function partIsClear(scene){
  const key=scene.placement.frame.map(v=>Object.is(v,-0)?'-0':v).join(',');
  if(!selfClearByFrame.has(key)){
   const partScene={...scene,bodies:scene.bodies.filter(isPart)};
   selfClearByFrame.set(key,checkScene(model,partScene,{clearanceMm,immutablePartColliders,projectJointBounds:true}).status==='SAMPLE_CLEAR');
  }
  return selfClearByFrame.get(key);
 }
 function accept(config){
  guard();if(!advancedInventoryFits(catalog,config,bedLengthMm)||!punches.some(t=>String(t.id)===String(config.punchId))||!dies.some(t=>String(t.id)===String(config.dieId)))return;
  const key=advancedConfigKey(config);if(seen.has(key))return;seen.add(key);diagnostics.combinationsTested++;
  const scene=sceneAt(config),support=assessToolSupport(assessCoverage({materialIntervals:scene.coverage.intervals,segments:config.segments,activeBankId:config.activeBankId,exact:scene.coverage.exact}),options);
  if(!support.accepted||!evaluateOperatorSupport(prepared,scene).passes)return;
  if(!partIsClear(scene))return;
  const onlyIds=new Set(scene.bodies.filter(b=>!isPart(b)).map(b=>b.id));
  if(checkScene(model,scene,{clearanceMm,onlyIds,immutablePartColliders}).status==='SAMPLE_CLEAR')configs.push(config);
 }
 const supplied=(options.setups??[]).map(s=>s.config??s);
 if(options.initialSegments)supplied.push({...options.initialSetup,punchId:options.punchId??options.initialSetup?.punchId,dieId:options.dieId??options.initialSetup?.dieId,segments:options.initialSegments,stationYmm:options.stationYmm??options.initialSetup?.stationYmm??0});
 if(supplied.length){for(const c of supplied)for(const flip of(c.flip===undefined?[false,true]:[Boolean(c.flip)]))accept({...c,bendId:bend.id,bedLengthMm,flip});diagnostics.explicitSetupsOnly=true;return{configs,diagnostics};}
 if(options.fixedStations!==undefined)throw new Error('Advanced search does not yet support fixedStations without explicit setups');
 const allowFullStockBanks=shouldGenerateFullStockBanks({model,bedLengthMm,options,preferred});
 diagnostics.fullStockBanksEnabled=allowFullStockBanks;
 for(const config of preferred)for(const flip of[false,true])accept({...config,bendId:bend.id,flip});
 // Duplicate catalog IDs have legacy grouping semantics based on the first
 // clear occurrence. Keep that ambiguous input on the unchanged eager path.
 const lazy=options.advancedLazyEndpointBanks!==false&&[punches,dies].every(tools=>new Set(tools.map(t=>t.id)).size===tools.length),limit=options.maxAdvancedEndpointSetups??48;
 const blockedAtClosure=(tool,flip,mirror)=>terminalRow?.presentations?.some(w=>w.punchId===tool.id&&w.flip===flip&&w.mirror===mirror&&w.amount===1);
 if(lazy)diagnostics.lazyBanks={profilesGenerated:0,layoutsGenerated:0,layoutsVisited:0,clearBanksConsumed:0};
 for(const flip of[false,true]){
  // Later flips cannot change an already retained prefix. The eager branch is
  // deliberately unchanged for differential testing and rollback.
  if(lazy&&Number.isInteger(limit)&&limit>=0&&configs.length>=limit){guard();break;}
  const intervals=flip?material.intervals.map(([a,b])=>[-b,-a]).reverse():material.intervals;
  const dummy=[{id:'dummy-punch',kind:'punch',lengthMm:25.4,centerY:0},{id:'dummy-die',kind:'die',lengthMm:25.4,centerY:0}];
  const probe=sceneAt({...base,flip,segments:dummy});
  if(!evaluateOperatorSupport(prepared,probe).passes){diagnostics.frontRejected++;continue;}
  if(!partIsClear(probe)){diagnostics.selfRejected++;continue;}
  const componentTemplates=new Map(),frameBlocked=new Map(),stockStation=-(intervals[0][0]+intervals.at(-1)[1])/2;
  const possibleStations=allowFullStockBanks?[...new Set([0,stockStation])]:[0];
  // The part presentation, upper registration and lower datum do not depend
  // on segment length or center. Use the same mounted meshes as the combined
  // assembly without rebuilding unrelated frame sections for every segment.
  function componentTemplate(kind,tool,stationYmm){
   const key=JSON.stringify([kind,tool.id,stationYmm]);
   if(!componentTemplates.has(key)){
    const config={...base,flip,stationYmm,[kind+'Id']:tool.id,segments:dummy};
    const scene=tool.id===base[kind+'Id']&&stationYmm===0?probe:sceneAt(config);
    componentTemplates.set(key,{scene,parts:scene.bodies.filter(isPart),punch:toolById(catalog,config.punchId),die:toolById(catalog,config.dieId)});
   }
   return componentTemplates.get(key);
  }
  function obstructed(kind,tool,stationYmm=0){
   const key=JSON.stringify([kind,tool.id,stationYmm]);if(frameBlocked.has(key))return frameBlocked.get(key);
   const upper=kind==='punch';
   if(!probe.bodies.some(b=>upper?b.kind==='ram':b.kind==='table'||b.kind==='bed'))return false;
   const {scene}=componentTemplate(kind,tool,stationYmm);
   // The lower datum and part frame depend on the active die. Relative to the
   // punch mount the die's support height cancels (the part and ram both receive
   // the same Z offset). Neither datum moves with a segment's length or mirror.
   const clearance=options.clearanceMm??.15;
   const hit=partFrameHit(scene,kind,upper?Math.max(0,clearance-1e-6):clearance);
   frameBlocked.set(key,Boolean(hit));if(!hit)return false;
   (diagnostics.frameRejections??=[]).push({bendId:bend.id,flip,stationYmm,toolId:tool.id,collision:hit});
   return true;
  }
  let pairs;
  if(lazy){
   function sources(kind,tools){
    const groups=new Map(),segmentCache=new Map(),axial=new Map();
    for(const tool of tools){if(!groups.has(tool.id))groups.set(tool.id,[]);groups.get(tool.id).push(tool);}
    return [...groups.values()].map(group=>({*[Symbol.iterator](){
     for(const tool of group){
      guard();diagnostics.lazyBanks.profilesGenerated++;
      if(kind==='punch'&&blockedAtClosure(tool,flip,false)&&blockedAtClosure(tool,flip,true))continue;
      if(possibleStations.every(station=>obstructed(kind,tool,station)))continue;
      const layouts=generateIslandBanks(tool,intervals,{...options,...toolSupportPolicy(options),bedLengthMm,allowFullStockBanks});
      diagnostics.lazyBanks.layoutsGenerated+=layouts.length;
      for(const layout of layouts){guard();diagnostics.lazyBanks.layoutsVisited++;
       if(kind==='punch'&&blockedAtClosure(tool,flip,Boolean(layout.segments[0]?.mirror)))continue;
       const stationYmm=layout.stationYmm??0;if(obstructed(kind,tool,stationYmm))continue;
       const eachClear=layout.segments.every(segment=>{
        const key=JSON.stringify([tool.id,stationYmm,segment.lengthMm,segment.centerY,segment.mirror]);if(segmentCache.has(key))return segmentCache.get(key);
        diagnostics.componentsTested++;
        const profileKey=JSON.stringify([tool.id,stationYmm,segment.mirror]);
        let prior=axial.get(profileKey);if(!prior){prior={blocked:[],clear:[]};axial.set(profileKey,prior);}
        const lo=segment.centerY-segment.lengthMm/2,hi=segment.centerY+segment.lengthMm/2;
        if(prior.blocked.some(w=>w.y-w.slack>lo&&w.y+w.slack<hi)){segmentCache.set(key,false);return false;}
        // A component inside a longer clear extrusion is still presented to
        // accept() for a full independent assembled-bank collision check.
        if(prior.clear.some(range=>range[0]<=lo&&range[1]>=hi)){segmentCache.set(key,true);return true;}
        const {scene,parts,punch,die}=componentTemplate(kind,tool,stationYmm);
        const bank=buildMountedBank(catalog,punch,die,[segment],scene.placement.punchTipZ,scene.cycle.gap);
        const relevant={...scene,bodies:[...parts,...bank.bodies]};
        const result=checkScene(model,relevant,{immutablePartColliders,clearanceMm,onlyIds:new Set(bank.bodies.map(b=>b.id))});
        const passed=result.status==='SAMPLE_CLEAR';
        if(passed&&result.distanceMm>clearanceMm+1e-6&&bank.bodies.every(body=>meshCollider(body).closed))prior.clear.push([lo,hi]);
        else{const witness=sideObstructionStation(bank.bodies,result.hits[0],clearanceMm);if(witness)prior.blocked.push(witness);}
        segmentCache.set(key,passed);return passed;
       });if(eachClear){diagnostics.lazyBanks.clearBanksConsumed++;yield layout;}
      }
     }
    }}));
   }
   pairs=advancedLazyBankPairs(sources('punch',punches),sources('die',dies));
  }else{
   const byKind={};for(const [kind,tools]of[['punch',punches],['die',dies]]){
   const clear=[],segmentCache=new Map();for(const tool of tools){
    if(possibleStations.every(station=>obstructed(kind,tool,station)))continue;
    const layouts=generateIslandBanks(tool,intervals,{...options,...toolSupportPolicy(options),bedLengthMm,allowFullStockBanks});
    for(const layout of layouts){guard();
     const stationYmm=layout.stationYmm??0;if(obstructed(kind,tool,stationYmm))continue;
     const eachClear=layout.segments.every(segment=>{
      const key=JSON.stringify([tool.id,stationYmm,segment.lengthMm,segment.centerY,segment.mirror]);if(segmentCache.has(key))return segmentCache.get(key);
      diagnostics.componentsTested++;const other=kind==='punch'?'die':'punch',config={...base,flip,stationYmm,[kind+'Id']:tool.id,segments:[segment,dummy.find(s=>s.kind===other)]},scene=sceneAt(config);
      const relevant={...scene,bodies:scene.bodies.filter(b=>isPart(b)||b.kind===kind||(kind==='die'&&b.kind==='riser'))};
      const passed=checkScene(model,relevant,{immutablePartColliders,clearanceMm:options.clearanceMm??.15,onlyIds:new Set(relevant.bodies.filter(b=>!isPart(b)).map(b=>b.id))}).status==='SAMPLE_CLEAR';segmentCache.set(key,passed);return passed;
     });if(eachClear)clear.push(layout);
    }
   }byKind[kind]=clear;
   if(!clear.length)break;
  }
   const ps=byKind.punch??[],ds=byKind.die??[];
   pairs=advancedBankPairs(ps,ds);
  }
  // Visit each profile pair before taking extra axial variants of one pair.
  for(const [p,d]of pairs){
   if((p.stationYmm??0)!==(d.stationYmm??0))continue;
   const previous=configs.length;
   accept({...base,flip,stationYmm:p.stationYmm??0,punchId:p.toolId,dieId:d.toolId,segments:[...p.segments,...d.segments],setupFamily:'MATERIAL_ISLAND_BANKS'});
   if(lazy&&!preferred.length&&Number.isSafeInteger(limit)&&limit>0&&configs.length>previous&&configs.length<=limit)yield configs.at(-1);
   if(configs.length>=(options.maxAdvancedEndpointSetups??48))break;
  }
 }
 return{configs:configs.slice(0,options.maxAdvancedEndpointSetups??48),diagnostics};
}
