/** Candidate tooling and minimum-change planning over supplied feasible transitions.
 * Pure JavaScript; this module neither performs collision checks nor predicts forming.
 */
import {assessCoverage} from './coverage.mjs';

const KINDS=['punch','die'],EPS=1e-7;
/** Lexicographic: both profiles replaced > one profile (punch-only/die-only) >
 * same profiles with a different segment stack > support/secondary metrics.
 * toolChangeEvents (any of the three) is reported but derived, not ranked. */
export const COST_ORDER=['fullChangeEvents','partialChangeEvents','segmentEditEvents','uncoveredLengthMm','profileChanges','segmentEdits','orientationChanges','repositionEvents','stationTravelMm'];
const METRICS=[...COST_ORDER,'toolChangeEvents'];
const finite=(value,label)=>{if(typeof value!=='number'||!Number.isFinite(value))throw new TypeError(`${label} must be finite.`);return value;};
const rounded=value=>Math.round(value*1e6)/1e6;
const sum=values=>values.reduce((a,b)=>a+b,0);

function union(intervals){
  const result=[];
  for(const [a,b] of intervals.map(v=>v.slice()).sort((a,b)=>a[0]-b[0])){
    if(!Number.isFinite(a)||!Number.isFinite(b)||b<a)throw new TypeError('Invalid material interval.');
    const last=result.at(-1);if(last&&a<=last[1]+EPS)last[1]=Math.max(last[1],b);else result.push([a,b]);
  }return result;
}
const lengthOf=intervals=>sum(union(intervals).map(([a,b])=>b-a));

/** Physical uncovered material is counted once, even when absent under both tools. */
export function coverageMetrics(coverage){
  if(!coverage?.byKind||!Array.isArray(coverage.materialIntervals))throw new TypeError('Coverage interval data is required.');
  const materialLengthMm=lengthOf(coverage.materialIntervals);
  const physicalMissing=union(KINDS.flatMap(k=>coverage.byKind[k]?.physicalMissingIntervals??coverage.byKind[k]?.missingIntervals??[]));
  const effectiveMissing=union(KINDS.flatMap(k=>coverage.byKind[k]?.missingIntervals??[]));
  const uncoveredLengthMm=lengthOf(physicalMissing);
  return{materialLengthMm,uncoveredLengthMm,effectiveUncoveredLengthMm:lengthOf(effectiveMissing),
    coveredFraction:materialLengthMm?Math.max(0,1-uncoveredLengthMm/materialLengthMm):1,
    coverageUnresolved:coverage.status!=='COVERED',exact:coverage.exact===true,
    physicalMissingIntervals:physicalMissing,effectiveMissingIntervals:effectiveMissing};
}

function configOf(value){const config=value?.config??value;if(!config?.punchId||!config?.dieId||!Array.isArray(config.segments))throw new TypeError('A tooling config with both profiles and segments is required.');return config;}
function lengthsByKind(config,kind){return config.segments.filter(s=>s.kind===kind).map(s=>{
  const length=finite(s.lengthMm,'segment length');if(length<=0)throw new RangeError('Segment length must be positive.');return rounded(length);
}).sort((a,b)=>a-b);}

/** Stock identity excludes offsets, station positions, ordering and segment names. */
export function toolSetupKey(value){
  const config=configOf(value);
  return JSON.stringify(KINDS.map(kind=>[kind,String(config[`${kind}Id`]),lengthsByKind(config,kind)]));
}
function placementKey(config){
  const segments=config.segments.map(s=>[s.kind,rounded(s.lengthMm),rounded(finite(s.centerY,'centerY')),
    rounded(finite(s.offsetX??0,'offsetX')),rounded(finite(s.offsetZ??0,'offsetZ')),Boolean(s.mirror)]);
  segments.sort((a,b)=>JSON.stringify(a).localeCompare(JSON.stringify(b)));
  return JSON.stringify(segments);
}

/** A short local span is evidence of differing spans, not proof of an enclosed/internal bend. */
export function classifyBendSpan({materialIntervals,partProjectedSpanMm}={}){
  const material=union(materialIntervals??[]),bendSpanMm=material.length?material.at(-1)[1]-material[0][0]:0;
  if(partProjectedSpanMm===undefined||partProjectedSpanMm===null)return{bendSpanMm,partProjectedSpanMm:null,
    relation:'PART_PROJECTION_NOT_SUPPLIED',internalBendVerified:false};
  finite(partProjectedSpanMm,'partProjectedSpanMm');if(partProjectedSpanMm<=0)throw new RangeError('Part projected span must be positive.');
  return{bendSpanMm,partProjectedSpanMm,bendToPartSpanRatio:bendSpanMm/partProjectedSpanMm,
    relation:bendSpanMm<partProjectedSpanMm-EPS?'LOCAL_BEND_SHORTER_THAN_PART_PROJECTION':'NO_SHORTER_LOCAL_SPAN',
    internalBendVerified:false,note:'Span comparison alone does not establish tool access, an enclosed bend or a required short tool.'};
}

function stockFor(tool){
  const known=Array.isArray(tool.inventory)&&tool.inventory.length>0;
  const source=known?tool.inventory:(tool.availableLengthsMm??tool.stockLengthsMm??[]).map(lengthMm=>({lengthMm,count:1}));
  const stock=new Map();
  for(const item of source){const length=Number(item.lengthMm),count=Math.floor(Number(item.count));
    if(Number.isFinite(length)&&length>0&&Number.isFinite(count)&&count>0)stock.set(rounded(length),(stock.get(rounded(length))??0)+count);}
  return{items:[...stock].map(([lengthMm,count])=>({lengthMm,count})).sort((a,b)=>b.lengthMm-a.lengthMm),countsKnown:known};
}
/** A bank longer than the bed is mountable when each end section (the two
 * longest, placed at the ends) has more than half its length over the bed.
 * Used only when the bend itself is longer than the bed (thin stock, low tonnage). */
export function overhangBankFits(lengths,bedLength){
  const total=sum(lengths);if(total<=bedLength+EPS)return true;
  if(lengths.length<2)return false;
  const ends=[...lengths].sort((a,b)=>b-a);return total-ends[1]<bedLength-EPS;
}

/** A forming segment is fully seated, or strictly more than half of its
 * actual length overlaps the bed. A long bank never licenses a floating end. */
export function formingSegmentSeated(segment,bedLength){
  const {lengthMm,centerY}=segment;
  if(!Number.isFinite(bedLength)||bedLength<=0||!Number.isFinite(lengthMm)||lengthMm<=0||!Number.isFinite(centerY))return false;
  const overlap=Math.max(0,Math.min(centerY+lengthMm/2,bedLength/2)-Math.max(centerY-lengthMm/2,-bedLength/2));
  return overlap>lengthMm/2+EPS;
}
function banksFor(tool,required,bedLength,maxFull,shared=[],maxUnsupportedEndMm=0){
  const stock=stockFor(tool),found=[];let enumerated=0;
  const limit=required>bedLength+EPS?bedLength+(stock.items[0]?.lengthMm??0):bedLength;
  function visit(index,lengths,total){
    if(total>=required-EPS){
      if(!overhangBankFits(lengths,bedLength))return;
      // Inclusion-minimal full banks; adding redundant sections would create
      // an unbounded candidate space. Every available full-length single is retained.
      if(lengths.every(v=>total-v<required-EPS)){found.push({lengths:[...lengths],totalLengthMm:total});enumerated++;}
      return;
    }
    if(index===stock.items.length)return;
    const item=stock.items[index],max=Math.min(item.count,Math.floor((limit-total+EPS)/item.lengthMm));
    for(let count=0;count<=max;count++)visit(index+1,[...lengths,...Array(count).fill(item.lengthMm)],total+count*item.lengthMm);
  }
  visit(0,[],0);
  found.sort((a,b)=>a.totalLengthMm-b.totalLengthMm||a.lengths.length-b.lengths.length||JSON.stringify(a.lengths).localeCompare(JSON.stringify(b.lengths)));
  const selected=found.filter(b=>b.lengths.length===1);
  for(const bank of found)if(bank.lengths.length!==1&&selected.length<maxFull)selected.push(bank);
  const byKey=new Map();
  const add=(bank,family)=>{const key=JSON.stringify([...bank.lengths].sort((a,b)=>a-b));
    if(byKey.has(key))byKey.get(key).families.push(family);else byKey.set(key,{...bank,families:[family]});};
  selected.forEach(b=>add(b,'FULL_SPAN_BANK'));
  // Also enumerate the longest real stock combinations immediately below the
  // span. They are useful when full-length tooling clips already formed walls.
  // Include every available single length (1/2/4/8/16 in), not only tiny tools.
  if(maxUnsupportedEndMm>0){
    const shorter=[];
    function visitShort(index,lengths,total){
      if(total>=required-EPS||total>bedLength+EPS)return;
      if(index===stock.items.length){if(total>0&&total>=required-2*maxUnsupportedEndMm-EPS)shorter.push({lengths,totalLengthMm:total});return;}
      const item=stock.items[index],max=Math.min(item.count,Math.floor((Math.min(required,bedLength)-total+EPS)/item.lengthMm));
      for(let count=0;count<=max;count++)visitShort(index+1,[...lengths,...Array(count).fill(item.lengthMm)],total+count*item.lengthMm);
    }
    visitShort(0,[],0);
    shorter.sort((a,b)=>b.totalLengthMm-a.totalLengthMm||a.lengths.length-b.lengths.length);
    shorter.slice(0,maxFull).forEach(b=>add(b,'END_ALLOWANCE_BANK'));
  }
  for(const target of [25.4,50.8,101.6,203.2,406.4]){
    const item=stock.items.find(s=>Math.abs(s.lengthMm-target)<1e-6);
    if(item&&item.lengthMm<=bedLength+EPS)add({lengths:[item.lengthMm],totalLengthMm:item.lengthMm},'SINGLE_STOCK_DIAGNOSTIC');
  }
  let rejectedSharedBanks=0;
  for(const supplied of shared){
    if(!Array.isArray(supplied)||!supplied.length||supplied.some(v=>!Number.isFinite(v)||v<=0))throw new TypeError('Shared banks must contain positive lengths.');
    const lengths=supplied.map(rounded),counts=new Map();for(const value of lengths)counts.set(value,(counts.get(value)??0)+1);
    if(!overhangBankFits(lengths,bedLength)||[...counts].some(([length,count])=>(stock.items.find(s=>s.lengthMm===length)?.count??0)<count)){rejectedSharedBanks++;continue;}
    add({lengths:[...lengths].sort((a,b)=>b-a),totalLengthMm:sum(lengths)},'SHARED_SETUP_BANK');
  }
  return{banks:[...byKey.values()],stock,enumeratedFullBanks:enumerated,retainedFullBanks:selected.length,
    fullBankSearchTruncated:selected.length<found.length,rejectedSharedBanks};
}
function compatible(tool,bend){
  const included=180-Math.abs(bend.signedAngle)*180/Math.PI,angle=tool.contact?.openingAngleDegrees;
  if(Number.isFinite(angle)&&angle>included+1e-6)return false;
  return !(tool.kind==='punch'&&Number.isFinite(tool.contact?.tipRadiusMm)&&Number.isFinite(bend.insideRadius)&&tool.contact.tipRadiusMm>bend.insideRadius+1e-6);
}
function positionedBank(bank,kind,centerY,bedLength=Infinity){
  let edge=centerY-bank.totalLengthMm/2;
  // Longest section first, second longest last: overhanging banks rest on their end sections.
  const sorted=[...bank.lengths].sort((a,b)=>b-a),order=sorted.length>2&&bank.totalLengthMm>bedLength+EPS?[sorted[0],...sorted.slice(2),sorted[1]]:sorted;
  return order.map((lengthMm,i)=>{const s={id:`${kind}-${i+1}`,kind,lengthMm,centerY:edge+lengthMm/2,offsetX:0,offsetZ:0};edge+=lengthMm;return s;});
}

/** The normal search's real-stock bank family, also used by island search.
 * Off-centre material moves to a bed-centred bank when required; tool stock,
 * the end-section seating rule and the bank enumeration are unchanged. */
export function generateCenteredStockBanks(tool,materialIntervals,{bedLengthMm,maxFullBanksPerTool=6,maxUnsupportedEndMm=0}={}){
  const material=union(materialIntervals),mid=(material[0][0]+material.at(-1)[1])/2;
  const required=material.at(-1)[1]-material[0][0];
  const room=required>bedLengthMm-2*Math.abs(mid)+EPS?bedLengthMm:Math.max(0,bedLengthMm-2*Math.abs(mid));
  return banksFor(tool,required,room,maxFullBanksPerTool,[],maxUnsupportedEndMm).banks
    .filter(bank=>bank.families.some(f=>f==='FULL_SPAN_BANK'||f==='END_ALLOWANCE_BANK'))
    .map(bank=>{
      const stationYmm=Math.abs(mid)+bank.totalLengthMm/2>bedLengthMm/2+EPS?-mid:0;
      return {stationYmm,segments:positionedBank(bank,tool.kind,mid+stationYmm,bedLengthMm),totalLengthMm:bank.totalLengthMm};
    }).filter(bank=>bank.segments.every(segment=>formingSegmentSeated(segment,bedLengthMm)));
}

/**
 * Full-span banks, optional near-span banks, and single 1/2/4/8/16-inch stock. Punch
 * and die bank choices are independent. Incomplete coverage never rejects a
 * candidate. These are centered layouts, not an exhaustive station-position search.
 * sharedBankLengthsMm may be [[...lengths]] for both kinds or {punch:[...],die:[...]}.
 * It preserves longer multi-section banks worth reusing across different operations.
 */
export function generateToolSetupCandidates({catalog,bend,materialIntervals,exact=true,centerY,
  partProjectedSpanMm,bedLengthMm=838.2,maxFullBanksPerTool=6,sharedBankLengthsMm=[],maxUnsupportedEndMm=0}={}){
  if(!Array.isArray(catalog?.tools)||!Number.isFinite(bend?.signedAngle))throw new TypeError('Catalog and bend geometry are required.');
  const material=union(materialIntervals??[]);
  if(!material.length||lengthOf(material)<=EPS)throw new RangeError('A nonempty material interval is required.');
  finite(bedLengthMm,'bedLengthMm');if(bedLengthMm<=0)throw new RangeError('Bed length must be positive.');
  if(!Number.isInteger(maxFullBanksPerTool)||maxFullBanksPerTool<1)throw new RangeError('maxFullBanksPerTool must be a positive integer.');
  if(!Number.isFinite(maxUnsupportedEndMm)||maxUnsupportedEndMm<0)throw new RangeError('Invalid maxUnsupportedEndMm.');
  centerY=finite(centerY??(material[0][0]+material.at(-1)[1])/2,'centerY');
  const required=2*Math.max(Math.abs(centerY-material[0][0]),Math.abs(material.at(-1)[1]-centerY));
  // A bend longer than the bed is re-centred (station shift), so the whole bed counts.
  // An off-centre stroke (e.g. joint spans) that does not fit a bank centred on
  // its material uses the whole bed; the part station moves instead.
  const maxCenteredLength=required>bedLengthMm-2*Math.abs(centerY)+EPS?bedLengthMm:Math.max(0,bedLengthMm-2*Math.abs(centerY));
  const diagnostics={nominalRejectedToolIds:[],missingProfileToolIds:[],banks:[],exhaustiveToolPlacementSearch:false};
  const tools={punch:[],die:[]};
  for(const tool of catalog.tools){
    if(!KINDS.includes(tool.kind))continue;
    if(tool.status!=='ready'){diagnostics.missingProfileToolIds.push(tool.id);continue;}
    if(!compatible(tool,bend)){diagnostics.nominalRejectedToolIds.push(tool.id);continue;}
    const shared=Array.isArray(sharedBankLengthsMm)?sharedBankLengthsMm:sharedBankLengthsMm?.[tool.kind]??[];
    const bankData=banksFor(tool,required,maxCenteredLength,maxFullBanksPerTool,shared,maxUnsupportedEndMm);
    diagnostics.banks.push({toolId:tool.id,countsKnown:bankData.stock.countsKnown,
      enumeratedFullBanks:bankData.enumeratedFullBanks,retainedFullBanks:bankData.retainedFullBanks,
      fullBankSearchTruncated:bankData.fullBankSearchTruncated,rejectedSharedBanks:bankData.rejectedSharedBanks});
    tools[tool.kind].push({tool,...bankData});
  }
  const candidates=[],seen=new Set(),spanClassification=classifyBendSpan({materialIntervals:material,partProjectedSpanMm});
  // Each profile's physical and effective support depends only on that bank
  // and station. Prepare it once in this invocation; each returned candidate
  // owns its mutable segment and interval arrays.
  const prepared=new WeakMap();
  function mounted(bank,tool,station){
    let rows=prepared.get(bank);
    if(!rows){rows=new Map();prepared.set(bank,rows);}
    if(rows.has(station))return rows.get(station);
    const segments=positionedBank(bank,tool.kind,centerY+station,bedLengthMm);
    const coverage=assessCoverage({materialIntervals:material.map(([a,b])=>[a+station,b+station]),segments,exact});
    const key=JSON.stringify([tool.kind,String(tool.id),bank.lengths.map(rounded).sort((a,b)=>a-b)]);
    const row={segments,coverage,key};rows.set(station,row);return row;
  }
  function copySide(side){
    const result={...side};
    for(const key of ['missingIntervals','coveredIntervals','physicalMissingIntervals','physicalIntervals','effectiveIntervals','acceptedJointGaps'])
      result[key]=side[key].map(pair=>pair.slice());
    return result;
  }
  for(const punch of tools.punch)for(const die of tools.die)for(const pBank of punch.banks)for(const dBank of die.banks){
    // An overhanging bank is centred on the bed; the part station moves instead.
    const station=Math.abs(centerY)+Math.max(pBank.totalLengthMm,dBank.totalLengthMm)/2>bedLengthMm/2+EPS?-centerY:0;
    const upper=mounted(pBank,punch.tool,station),lower=mounted(dBank,die.tool,station);
    const setupKey='['+upper.key+','+lower.key+']';if(seen.has(setupKey))continue;seen.add(setupKey);
    const config={punchId:punch.tool.id,dieId:die.tool.id,segments:[...upper.segments,...lower.segments].map(s=>({...s})),
      bedLengthMm,contactGapMm:.3,stationYmm:station};
    const byKind={punch:copySide(upper.coverage.byKind.punch),die:copySide(lower.coverage.byKind.die)};
    const coverage={...upper.coverage,materialIntervals:upper.coverage.materialIntervals.map(x=>x.slice()),byKind,
      status:!exact?'UNKNOWN':byKind.punch.covered&&byKind.die.covered?'COVERED':'INCOMPLETE'};
    const metrics=coverageMetrics(coverage);
    candidates.push({setupKey,config,coverage,...metrics,spanClassification,
      status:coverage.status==='COVERED'?'GEOMETRIC_CANDIDATE':coverage.status==='UNKNOWN'?'DIAGNOSTIC_COVERAGE_UNKNOWN':'DIAGNOSTIC_UNDERCOVERED',
      formabilityVerified:false,collisionChecked:false,
      banks:{punch:{lengths:pBank.lengths,totalLengthMm:pBank.totalLengthMm,families:pBank.families},
             die:{lengths:dBank.lengths,totalLengthMm:dBank.totalLengthMm,families:dBank.families}},
      nominalCompatibility:{passes:true,insideRadiusKnown:Number.isFinite(bend.insideRadius),punchTipRadiusKnown:Number.isFinite(punch.tool.contact?.tipRadiusMm),
        note:'Nominal angle/radius filters only; no force, load distribution or partial-length forming calculation.'},
      inventoryCountsKnown:punch.stock.countsKnown&&die.stock.countsKnown});
  }
  // Equal coverage: fewest physical sections first (one 16 in die beats 8+4+2+2).
  candidates.sort((a,b)=>a.uncoveredLengthMm-b.uncoveredLengthMm||a.config.segments.length-b.config.segments.length||a.setupKey.localeCompare(b.setupKey));
  diagnostics.candidateCount=candidates.length;diagnostics.undercoveredCount=candidates.filter(c=>c.coverage.status==='INCOMPLETE').length;
  return{candidates,diagnostics,spanClassification};
}

function segmentDifference(a,b){
  let count=0;
  for(const kind of KINDS){const counts=new Map();
    for(const value of lengthsByKind(a,kind))counts.set(value,(counts.get(value)??0)+1);
    for(const value of lengthsByKind(b,kind))counts.set(value,(counts.get(value)??0)-1);
    count+=sum([...counts.values()].map(Math.abs));
  }return count;
}
function normalizeSetup(value){
  const config=configOf(value),setupKey=toolSetupKey(config);
  if(value.setupKey!==undefined&&value.setupKey!==setupKey)throw new Error('setupKey does not match profiles and segment lengths.');
  return{config,setupKey,placementKey:placementKey(config),flip:Boolean(value.flip??config.flip),
    stationShiftMm:finite(value.stationShiftMm??config.stationYmm??0,'stationShiftMm')};
}
function normalizeEdge(edge,index){
  const setup=normalizeSetup(edge.setup??edge.candidate??edge),coverage=edge.coverage??edge.setup?.coverage??edge.candidate?.coverage;
  const metrics=coverage?coverageMetrics(coverage):null;
  const uncoveredLengthMm=edge.uncoveredLengthMm??edge.setup?.uncoveredLengthMm??edge.candidate?.uncoveredLengthMm??metrics?.uncoveredLengthMm;
  finite(uncoveredLengthMm,'uncoveredLengthMm (or coverage interval data)');if(uncoveredLengthMm<0)throw new RangeError('Uncovered length cannot be negative.');
  return{...edge,...setup,id:edge.id??`transition-${index}`,uncoveredLengthMm,
    coverageUnresolved:uncoveredLengthMm>EPS||Boolean(edge.coverageUnresolved??edge.setup?.coverageUnresolved??edge.candidate?.coverageUnresolved??
      (coverage?coverage.status!=='COVERED':uncoveredLengthMm>EPS)),
    flip:Boolean(edge.flip??setup.flip),stationShiftMm:finite(edge.stationShiftMm??setup.stationShiftMm,'stationShiftMm')};
}
const zeroCost=()=>Object.fromEntries(METRICS.map(k=>[k,0]));
/** Per-kind change between consecutive setups: none | segments | profile. */
export function toolChangeKind(previous,next){
  if(!previous)return{punch:'none',die:'none'};
  const a=configOf(previous),b=configOf(next),profiles=(c,kind)=>JSON.stringify([...new Set(c.segments.filter(s=>s.kind===kind)
    .map(s=>String(s[`${kind}Id`]??c[`${kind}Id`])))].sort()),kindOf=kind=>{
    if(profiles(a,kind)!==profiles(b,kind))return'profile';
    const x=lengthsByKind(a,kind),y=lengthsByKind(b,kind);
    return x.length===y.length&&x.every((v,i)=>v===y[i])?'none':'segments';};
  return{punch:kindOf('punch'),die:kindOf('die')};
}
/** Adds stage.toolChange={punch,die} (none|segments|profile) relative to the previous stage. */
export function annotateToolChanges(stages){
  for(const [i,stage] of (stages??[]).entries())
    if(stage?.config?.segments)stage.toolChange=toolChangeKind(i?stages[i-1].config:null,stage.config);
  return stages;
}
function increment(previous,next){
  const cost=zeroCost();cost.uncoveredLengthMm=next.uncoveredLengthMm;
  if(!previous)return cost; // mounting the first selected setup is not a change
  const kind=toolChangeKind(previous.config,next.config),profiles=KINDS.filter(k=>kind[k]==='profile').length;
  cost.fullChangeEvents=Number(profiles===2);
  cost.partialChangeEvents=Number(profiles===1);
  cost.segmentEditEvents=Number(profiles===0&&KINDS.some(k=>kind[k]==='segments'));
  cost.toolChangeEvents=Number(previous.setupKey!==next.setupKey);
  cost.profileChanges=profiles;
  cost.segmentEdits=segmentDifference(previous.config,next.config);
  cost.orientationChanges=Number(typeof previous.flip==='boolean'&&previous.flip!==next.flip);
  cost.repositionEvents=Number(previous.placementKey!==null&&previous.setupKey===next.setupKey&&previous.placementKey!==next.placementKey);
  cost.stationTravelMm=Number.isFinite(previous.stationShiftMm)?Math.abs(previous.stationShiftMm-next.stationShiftMm):0;
  return cost;
}
const plus=(a,b)=>Object.fromEntries(METRICS.map(k=>[k,a[k]+b[k]]));
function compare(a,b){for(const k of COST_ORDER){const d=a[k]-b[k];if(Math.abs(d)>1e-9)return d;}return 0;}
function initialState({initialSetup,initialSetupKey},edges){
  if(initialSetup){const state=normalizeSetup(initialSetup),cfg=configOf(initialSetup);
    if(initialSetup.flip===undefined&&cfg.flip===undefined)state.flip=undefined;
    if(initialSetup.stationShiftMm===undefined&&cfg.stationYmm===undefined)state.stationShiftMm=undefined;
    return state;}
  if(initialSetupKey!==undefined){const match=edges.find(e=>e.setupKey===initialSetupKey);if(!match)throw new Error('initialSetupKey must identify a supplied setup; otherwise pass initialSetup.');
    return{...normalizeSetup(match),placementKey:null,flip:undefined,stationShiftMm:undefined};}
  return null;
}
function recount(edges,initial){let cost=zeroCost(),previous=initial;const events=[];
  for(const edge of edges){const delta=increment(previous,edge);
    events.push({transitionId:edge.id,...delta,changeKind:previous?toolChangeKind(previous.config,edge.config):{punch:'none',die:'none'}});
    cost=plus(cost,delta);previous=edge;}
  return{cost,events};
}

/** Recount physical setup changes and the documented secondary metrics. */
export function scoreToolSetupSequence(sequence,options={}){
  const edges=sequence.map(normalizeEdge);return recount(edges,initialState(options,edges));
}

/**
 * DP over (completed mask, last stock identity, last placement/orientation).
 * operationIds (alias bendIds) identify logical operations: B1_pre/B1_finish
 * may refer to the same physical bend. Caller-supplied masks must encode their
 * dependencies and independently checked physical state. No dependency is inferred.
 *
 * Each edge requires mask, activeIndex, feasible:boolean, config/setup, and
 * uncoveredLengthMm or full coverage data. Multiple feasible setups remain
 * alternatives. Missing support is diagnostic, never silently removed.
 *
 * Lexicographic cost: setup changes, total physical uncovered length, profile
 * replacements, inserted+removed segments, part orientation changes, relocation
 * of the same stock layout, and station travel. First setup has zero change cost
 * unless initialSetup/initialSetupKey is supplied. Optimal only in this input graph.
 */
export function optimizeToolSetupSequence({operationIds,bendIds,transitions,initialSetup,initialSetupKey}={}){
  const ids=operationIds??bendIds;
  if(!Array.isArray(ids)||ids.length<1||ids.length>8||new Set(ids.map(String)).size!==ids.length)throw new RangeError('Provide 1..8 unique logical operation IDs.');
  if(!Array.isArray(transitions))throw new TypeError('transitions must be an array.');
  const full=(1<<ids.length)-1,edges=[],byMask=new Map();let ignored=0;
  for(const [index,t] of transitions.entries()){
    if(typeof t.feasible!=='boolean')throw new TypeError('Each transition must explicitly declare feasible.');
    if(!Number.isInteger(t.mask)||t.mask<0||t.mask>full||!Number.isInteger(t.activeIndex)||t.activeIndex<0||t.activeIndex>=ids.length||(t.mask&(1<<t.activeIndex)))throw new RangeError('Invalid transition mask/activeIndex.');
    if(!t.feasible){ignored++;continue;}
    const edge=normalizeEdge(t,index);edges.push(edge);if(!byMask.has(edge.mask))byMask.set(edge.mask,[]);byMask.get(edge.mask).push(edge);
  }
  const initial=initialState({initialSetup,initialSetupKey},edges),states=Array.from({length:full+1},()=>new Map());
  states[0].set('initial',{cost:zeroCost(),last:initial,path:[]});
  let edgeRelaxations=0,statesVisited=0;
  for(let mask=0;mask<=full;mask++)for(const node of states[mask].values()){
    statesVisited++;
    for(const edge of byMask.get(mask)??[]){
      edgeRelaxations++;const nextMask=mask|(1<<edge.activeIndex),cost=plus(node.cost,increment(node.last,edge));
      const stateKey=JSON.stringify([edge.setupKey,edge.placementKey,edge.flip,edge.stationShiftMm]);
      const current=states[nextMask].get(stateKey);
      if(!current||compare(cost,current.cost)<0)states[nextMask].set(stateKey,{cost,last:edge,path:[...node.path,edge]});
    }
  }
  let best=null;for(const node of states[full].values())if(!best||compare(node.cost,best.cost)<0)best=node;
  const statistics={providedTransitions:transitions.length,feasibleTransitions:edges.length,ignoredInfeasibleTransitions:ignored,
    statesVisited,statesStored:sum(states.map(m=>m.size)),edgeRelaxations,algorithm:'dynamic programming; no permutation enumeration'};
  const scope='Minimum cost only among supplied feasible transitions; no claim of physical or global manufacturing optimality.';
  if(!best)return{status:'NO_FEASIBLE_COMPLETE_PATH',operationIds:ids,sequence:[],cost:null,costOrder:COST_ORDER,statistics,scope,formabilityVerified:false};
  const checked=recount(best.path,initial);if(compare(checked.cost,best.cost)!==0)throw new Error('Internal sequence cost recount mismatch.');
  return{status:best.path.some(e=>e.coverageUnresolved&&e.uncoveredLengthMm>EPS)?'DIAGNOSTIC_UNDERCOVERED':best.path.some(e=>e.coverageUnresolved)?'DIAGNOSTIC_COVERAGE_UNKNOWN':'INPUT_GRAPH_OPTIMUM',operationIds:ids,
    sequence:best.path.map(e=>({...e,operationId:ids[e.activeIndex],maskBefore:e.mask,maskAfter:e.mask|(1<<e.activeIndex)})),
    cost:checked.cost,events:checked.events,costOrder:COST_ORDER,statistics,scope,formabilityVerified:false,
    coverageUnresolved:best.path.some(e=>e.coverageUnresolved),multipleOperationsMayReferToOnePhysicalBend:true};
}
