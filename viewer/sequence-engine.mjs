/** Bounded generic sequence search. It produces reviewable geometry, never machine commands. */
import {allowedForBend,nominalBendAt,toolAllowedAt} from './tooling-rules.mjs';
import {validateModel} from './model-validator.mjs';
import {checkFlatSelfContact,checkFinalSelfContact} from './sequence-flat-self-check.mjs';
import {bandPoint} from './solver.mjs';
import {strokeMaterialIntervals as materialIntervals,linkJointStrokes} from './machine.mjs';
import {strokeSearchAllowed as strokeAllowed,strokeAmounts} from './sequence-stroke.mjs';
import {generateToolSetupCandidates,toolSetupKey,toolChangeKind,scoreToolSetupSequence,COST_ORDER,coverageMetrics,formingSegmentSeated} from './tool-setup-search.mjs';
import {buildOperatorSupport,consistentSupportCentroid} from './sequence-operator-support.mjs';
import {evaluateSequenceTransition,screenTransitionEndpoints} from './sequence-transition.mjs';
import {assessCoverage} from './coverage.mjs';
import {checkFinalToolSetups} from './sequence-final-setup.mjs';
import {diagnosePairedClosure} from './sequence-paired-closure.mjs';
import {toolSupportPolicy,assessToolSupport} from './tool-support-policy.mjs';
import {probeFixedClosurePrograms} from './sequence-fixed-closure.mjs';
import {axialPlacementVariants,partialBendTargets,remainingPhaseQuota} from './sequence-search-variants.mjs';
import {appendMirroredToolSetups,TOOL_ORIENTATION_SCOPE} from './sequence-tool-orientations.mjs';
import {longCompoundFamily,spanCapableFinalSetups,probeLongCompound,compareChangeClass} from './sequence-long-compound.mjs';

const foldPool=globalThis.process?.versions?.node?await import('./sequence-fold-pool.mjs'):null;

const sum=values=>values.reduce((a,b)=>a+b,0),EPS=1e-8;
const defaults={beamWidth:6,maxStates:80,maxTransitions:600,maxCandidatesPerBend:24,timeBudgetMs:90000,
  allowPartialBends:true,partialAngleDegrees:30,additionalRetractMm:6.35,clearanceMm:.15,contactGapMm:.3,
  foldSamples:21,foldStepDegrees:5,accessPointStepMm:2,ramStepMm:2,maxAccessSamples:2500,frontMarginMm:10,
  maxVertices:50000,maxTwoBankTemplates:6,maxFullBanksPerTool:6,maxSuccessorsPerState:8,maxChecksPerState:48};
class Budget extends Error{constructor(reason){super(reason);this.reason=reason;}}
const amountKey=(model,a)=>model.bends.map(b=>Number(a[b.id]).toPrecision(15)).join('|');
const sameAmounts=(model,a,b)=>model.bends.every(j=>Math.abs(a[j.id]-b[j.id])<EPS);
const compareCost=(a,b)=>{for(const k of COST_ORDER){const d=a[k]-b[k];if(Math.abs(d)>1e-8)return d;}return 0;};
function normalizedModel(model){
  return linkJointStrokes({...model,bends:model.bends.map(b=>b.band.flatPositions?b:{...b,band:{...b.band,
    flatPositions:b.band.parameters.flatMap((v,i,a)=>i%3?[]:bandPoint(b,v,a[i+1],a[i+2],0))}})});
}
function compatible(catalog,config,bend){
  const punch=catalog.tools.find(t=>String(t.id)===String(config.punchId)),die=catalog.tools.find(t=>String(t.id)===String(config.dieId));
  if(!punch||!die||punch.status!=='ready'||die.status!=='ready')return false;
  const included=180-Math.abs(bend.signedAngle)*180/Math.PI;
  return[punch,die].every(t=>!Number.isFinite(t.contact?.openingAngleDegrees)||t.contact.openingAngleDegrees<=included+1e-6)&&
    (!Number.isFinite(punch.contact?.tipRadiusMm)||punch.contact.tipRadiusMm<=bend.insideRadius+1e-6);
}
function fitsInventory(catalog,config){
  if(!Array.isArray(config.segments)||config.segments.some(s=>!['punch','die'].includes(s.kind)||!Number.isFinite(s.lengthMm)||s.lengthMm<=0))return false;
  for(const kind of['punch','die']){
    const tool=catalog.tools.find(t=>String(t.id)===String(config[`${kind}Id`]));if(!tool)return false;
    if(!config.segments.some(s=>s.kind===kind))return false;
    const stock=Array.isArray(tool.inventory)&&tool.inventory.length?tool.inventory:(tool.availableLengthsMm??tool.stockLengthsMm??[]).map(lengthMm=>({lengthMm,count:1}));
    const needed=new Map();for(const s of config.segments.filter(s=>s.kind===kind)){const key=s.lengthMm.toFixed(6);needed.set(key,(needed.get(key)??0)+1);}
    for(const [length,count]of needed)if(stock.reduce((n,s)=>n+(Math.abs(s.lengthMm-Number(length))<1e-5&&Number.isInteger(s.count)&&s.count>0?s.count:0),0)<count)return false;
  }return true;
}
const configKey=c=>JSON.stringify([toolSetupKey(c),c.stationYmm??0,c.activeBankId??null,c.segments.map(s=>[s.kind,s.lengthMm,s.centerY,s.offsetX??0,s.offsetZ??0,s.mirror??false,s.bankId??null,s.punchId??null])]);
/** Inside the bed, or more than half of the section rests on it: 16+8+4+16 in with
 * both 16 in ends overhanging (also kept mounted under a shorter bend). */
function segmentSeated(s,bedLengthMm){
  return formingSegmentSeated(s,bedLengthMm);
}
function fullSpan(bend){const m=materialIntervals(bend);return{material:m,width:m.intervals.at(-1)[1]-m.intervals[0][0]};}

// Round-robin profile families before applying the candidate cap. Sorting by
// stock length/profile area alone can spend the entire cap on straight punches.
function diversePunches(values,config=value=>value.config){
  const groups=new Map();for(const value of values){const key=String(config(value).punchId);
    if(!groups.has(key))groups.set(key,[]);groups.get(key).push(value);}
  const result=[];for(let i=0;result.length<values.length;i++)for(const group of groups.values())if(i<group.length)result.push(group[i]);
  return result;
}

/** Generate centered stock and two separated banks from actual inventory, without part-specific dimensions. */
export function generateSequenceSetups({model,catalog,options={}}){
  linkJointStrokes(model);
  const o={...defaults,...options},bedLengthMm=o.bedLengthMm??catalog.machine?.bedLengthMm;
  const policy=toolSupportPolicy(o);
  if(!(bedLengthMm>0)||!Number.isFinite(bedLengthMm))throw new Error('bedLengthMm is required for a part with bends.');
  const filtered={...catalog,tools:catalog.tools.filter(t=>
    (!o.punchId||t.kind!=='punch'||String(t.id)===String(o.punchId))&&(!o.dieId||t.kind!=='die'||String(t.id)===String(o.dieId))&&
    (!o.allowedPunchIds||t.kind!=='punch'||o.allowedPunchIds.map(String).includes(String(t.id)))&&
    (!o.allowedDieIds||t.kind!=='die'||o.allowedDieIds.map(String).includes(String(t.id))))};
  const forBend=bend=>!o.toolingByBend?filtered:{...filtered,tools:filtered.tools.filter(t=>allowedForBend(o.toolingByBend,bend.id,t.kind,t.id))};
  const profileArea=(c,kind)=>filtered.tools.find(t=>String(t.id)===String(c[`${kind}Id`]))?.areaMm2??Infinity;
  // Rounded to 1e-6 mm: float sums (8+4+2+2 in = 812.7999…) must tie with a single 16 in.
  const lengths=new WeakMap();
  const totalLength=c=>{
    if(!lengths.has(c))lengths.set(c,Math.round(sum(c.config.segments.map(s=>s.lengthMm))*1e6));
    return lengths.get(c);
  };
  const candidateOrder=(a,b)=>totalLength(a)-totalLength(b)||
    profileArea(a.config,'punch')-profileArea(b.config,'punch')||profileArea(a.config,'die')-profileArea(b.config,'die')||
    // Same length and profiles: fewest physical sections (one 16 in die, not 8+4+2+2).
    a.config.segments.length-b.config.segments.length;
  const spans=model.bends.map(b=>({bend:b,...fullSpan(b)})),widest=spans.slice().sort((a,b)=>b.width-a.width)[0];
  const mostRestrictive={...widest.bend,signedAngle:Math.max(...model.bends.map(b=>Math.abs(b.signedAngle))),
    insideRadius:Math.min(...model.bends.map(b=>b.insideRadius))};
  const common=generateToolSetupCandidates({catalog:filtered,bend:mostRestrictive,materialIntervals:widest.material.intervals,
    exact:widest.material.exact,bedLengthMm,maxFullBanksPerTool:o.maxFullBanksPerTool,...policy});
  const full=common.candidates.filter(c=>!c.coverageUnresolved).sort(candidateOrder);
  const endBanks=common.candidates.filter(c=>c.coverageUnresolved&&assessToolSupport(c.coverage,policy).accepted)
    .sort((a,b)=>a.uncoveredLengthMm-b.uncoveredLengthMm||candidateOrder(a,b));
  const firstPerPunch=values=>{const seen=new Set();return values.filter(c=>!seen.has(c.config.punchId)&&seen.add(c.config.punchId));};
  const commonPreferred=[...firstPerPunch(full),...firstPerPunch(endBanks)];
  const profilePairs=new Set(),templates=[],sharedBanks=[];
  for(const c of commonPreferred)sharedBanks.push(c.banks.punch.lengths,c.banks.die.lengths);
  // Each bend's own preferred stacks (full span and accepted end allowance) are
  // offered to every other bend, so an already mounted longer or shorter stack
  // can be kept when the collision check allows it (segment edit avoided).
  const reusePairs=new Set();
  for(const entry of spans){
    const own=generateToolSetupCandidates({catalog:forBend(entry.bend),bend:entry.bend,materialIntervals:entry.material.intervals,
      exact:entry.material.exact,bedLengthMm,maxFullBanksPerTool:o.maxFullBanksPerTool,...policy}).candidates;
    const ownFull=own.filter(c=>!c.coverageUnresolved).sort(candidateOrder),
      ownEnds=own.filter(c=>c.coverageUnresolved&&assessToolSupport(c.coverage,policy).accepted).sort((a,b)=>a.uncoveredLengthMm-b.uncoveredLengthMm||candidateOrder(a,b));
    for(const c of[...firstPerPunch(ownFull),...firstPerPunch(ownEnds)]){
      sharedBanks.push(c.banks.punch.lengths,c.banks.die.lengths);reusePairs.add(c.setupKey);}
  }
  const reusePunches=new Set(),reuseDies=new Set();
  for(const key of reusePairs){
    const [punch,die]=JSON.parse(key);
    reusePunches.add(JSON.stringify(punch));reuseDies.add(JSON.stringify(die));
  }
  const profileReuse=new Map();
  function hasReusableProfile(key){
    if(profileReuse.has(key))return profileReuse.get(key);
    const [punch,die]=JSON.parse(key);
    const reusable=reusePunches.has(JSON.stringify(punch))||reuseDies.has(JSON.stringify(die));
    profileReuse.set(key,reusable);
    return reusable;
  }
  for(const c of diversePunches(full)){
    const pair=`${c.config.punchId}|${c.config.dieId}`;if(profilePairs.has(pair))continue;profilePairs.add(pair);
    sharedBanks.push(c.banks.punch.lengths,c.banks.die.lengths);
    // Source banks follow their original bend's material midpoint, which need
    // not be zero (e.g. disjoint joint-stroke spans). Relocate the actual bank
    // envelope, not that inherited bend frame, to the new long station.
    const longLo=Math.min(...c.config.segments.map(s=>s.centerY-s.lengthMm/2)),
      longHi=Math.max(...c.config.segments.map(s=>s.centerY+s.lengthMm/2)),rawLongOrigin=(longLo+longHi)/2,
      longOrigin=Math.abs(rawLongOrigin)<EPS?0:rawLongOrigin;
    for(const shortLength of[50.8,25.4,101.6]){
      const longLength=longHi-longLo;
      if(longLength+shortLength>bedLengthMm+EPS)continue;
      const gap=(bedLengthMm-longLength-shortLength)/2,mid=o.backgauge?.centerYmm??0;
      const pitch=Math.max((o.backgauge?.separationMm??100)+shortLength,(shortLength+longLength)/2+o.clearanceMm*2);
      const locations=[[mid-pitch/2,mid+pitch/2,'COMPACT'],[-(longLength+gap)/2,(shortLength+gap)/2,'WIDE']];
      let added=false;
      for(const [shortCenter,longCenter,layout] of locations){
        const punchOf=(kind,s)=>kind==='punch'?{punchId:s?.punchId??c.config.punchId}:{};
        const segments=[...['punch','die'].map(kind=>({id:`${kind}-short`,kind,bankId:'short',lengthMm:shortLength,centerY:shortCenter,offsetX:0,offsetZ:0,...punchOf(kind)})),
          ...c.config.segments.map(s=>({...s,id:`${s.id}-long`,bankId:'long',centerY:s.centerY-longOrigin+longCenter,...punchOf(s.kind,s)}))];
        const config={...c.config,segments,bedLengthMm,layout};
        if(!fitsInventory(filtered,config)||segments.some(s=>Math.abs(s.centerY)+s.lengthMm/2>bedLengthMm/2+EPS))continue;
        if(segments.some((s,i)=>segments.slice(0,i).some(t=>s.kind===t.kind&&Math.abs(s.centerY-t.centerY)<(s.lengthMm+t.lengthMm)/2-EPS)))continue;
        if(templates.length>=o.maxTwoBankTemplates)break;
        templates.push({config,stations:[{centerY:shortCenter,bankId:'short'},{centerY:longCenter,bankId:'long'}]});added=true;
      }
      if(added)break;
    }
    if(templates.length>=o.maxTwoBankTemplates)break;
  }
  const byBend=new Map(),diagnostics={twoBankTemplates:templates.length,generatedByBend:[],centeredPlacementOnly:true,
    supportPolicy:policy,candidateOrdering:'Reusable banks for the longest bend (full and allowed end gaps), then local banks; round-robin punch profiles. Short bends first, no forming adequacy inference.',
    placementScope:'Centered single banks plus inventory-qualified compact two-bank layouts around the gauge and wider separated layouts; bounded, not exhaustive.'};
  for(const entry of spans){
    const localCatalog=forBend(entry.bend);
    const generated=generateToolSetupCandidates({catalog:localCatalog,bend:entry.bend,materialIntervals:entry.material.intervals,
      exact:entry.material.exact,bedLengthMm,maxFullBanksPerTool:o.maxFullBanksPerTool,sharedBankLengthsMm:sharedBanks,...policy});
    const supplied=(o.setups??[]).map(s=>({... (s.config??s),bedLengthMm})),initial=[];
    if(o.initialSegments){const punchId=o.punchId??o.initialSetup?.punchId,dieId=o.dieId??o.initialSetup?.dieId;
      if(!punchId||!dieId)throw new Error('initialSegments requires an initialSetup or explicit profile IDs.');
      initial.push({...o.initialSetup,punchId,dieId,segments:o.initialSegments,stationYmm:o.stationYmm??o.initialSetup?.stationYmm??0,bedLengthMm});}
    const materialMid=(entry.material.intervals[0][0]+entry.material.intervals.at(-1)[1])/2;
    const two=templates.flatMap(t=>{
      const declaredPrecision=model.metadata?.validation?.flatEnvelopeToleranceMm,
        stationPrecision=Number.isFinite(declaredPrecision)&&declaredPrecision>0?Math.min(.01,declaredPrecision):EPS;
      const corrected=t.stations.map(station=>({...t.config,
        stationYmm:station.centerY-(Math.abs(materialMid)<EPS?0:materialMid),activeBankId:station.bankId,setupFamily:'TWO_BANK'}));
      // A microscopic source-envelope asymmetry should not add a new physical
      // station when the existing one really supports the bend. This is only
      // candidate ordering: no material, support or collision tolerance changes.
      const base=corrected.map((config,i)=>{
        if(Math.abs(materialMid)>stationPrecision)return config;
        const stationYmm=t.stations[i].centerY,shifted=entry.material.intervals.map(([a,b])=>[a+stationYmm,b+stationYmm]);
        return assessToolSupport(assessCoverage({materialIntervals:shifted,segments:config.segments,
          activeBankId:config.activeBankId,exact:entry.material.exact}),policy).accepted?{...config,stationYmm}:config;
      });
      // Keep both legacy stations first. Substantial asymmetric material also
      // needs its reflected part placement; transition checks the selected bank.
      return Math.abs(materialMid)<EPS?base:[...base,...corrected,
        ...corrected.map(c=>({...c,flip:true,stationYmm:c.stationYmm+2*materialMid}))];
    });
    const shorts=generated.candidates.filter(c=>c.banks.punch.lengths.length===1&&c.banks.die.lengths.length===1&&
      [25.4,50.8,101.6].some(n=>Math.abs(c.banks.punch.totalLengthMm-n)<EPS&&Math.abs(c.banks.die.totalLengthMm-n)<EPS));
    const shortFirst=[25.4,50.8,101.6].map(length=>shorts.find(c=>Math.abs(c.banks.punch.totalLengthMm-length)<EPS)).filter(Boolean);
    const singles=diversePunches(generated.candidates.filter(c=>!c.coverageUnresolved).sort(candidateOrder));
    const localEnds=diversePunches(generated.candidates.filter(c=>c.coverageUnresolved&&assessToolSupport(c.coverage,policy).accepted)
      .sort((a,b)=>a.uncoveredLengthMm-b.uncoveredLengthMm||candidateOrder(a,b)));
    const punchCount=new Set(singles.map(c=>c.config.punchId)).size;
    // Other bends' preferred stacks that also support this bend, mixed per kind
    // (punch of one stack with die of another enables punch-only/die-only edits).
    const reuse=generated.candidates.filter(c=>assessToolSupport(c.coverage,policy).accepted&&
      hasReusableProfile(c.setupKey))
      .sort((a,b)=>Number(!reusePairs.has(a.setupKey))-Number(!reusePairs.has(b.setupKey))||candidateOrder(a,b));
    // Common stacks were generated for the widest bend: its station centres
    // that bend's material. Re-centre this bend's material on the same bank
    // (overhanging banks stay centred on the bed; the part moves instead).
    const matLo=Math.min(...entry.material.intervals.map(v=>v[0])),matHi=Math.max(...entry.material.intervals.map(v=>v[1]));
    const recentre=c=>{
      if(c.setupFamily==='TWO_BANK')return c;
      if(!(c.stationYmm)||!Number.isFinite(matLo)||!Number.isFinite(matHi))return c;
      const ps=c.segments.filter(s=>s.kind==='punch');if(!ps.length)return c;
      const bankMid=(Math.min(...ps.map(s=>s.centerY-s.lengthMm/2))+Math.max(...ps.map(s=>s.centerY+s.lengthMm/2)))/2;
      return{...c,stationYmm:bankMid-(matLo+matHi)/2};
    };
    // Any off-centre stack (reused from another bend, supplied by tooling
    // selection) is tried centred on this bend's material first; the original
    // station stays as fallback when the centred one is not supported.
    const centredFirst=c=>{const r=recentre(c);return Math.abs((r.stationYmm??0)-(c.stationYmm??0))>EPS?[r,c]:[c];};
    const ordered=(supplied.length?supplied:[...initial,...commonPreferred.map(c=>recentre(c.config)),...singles.slice(0,punchCount).map(c=>c.config),
      ...reuse.slice(0,8).map(c=>c.config),
      ...localEnds.slice(0,punchCount).map(c=>c.config),
      ...two.slice(0,2),...(o.allowPartialCoverage===true?shortFirst.map(c=>c.config):[]),
      ...diversePunches([...singles.slice(punchCount,punchCount*2).map(c=>c.config),...two,
        ...singles.slice(punchCount*2).map(c=>c.config),...shorts.map(c=>c.config)],c=>c)]).flatMap(centredFirst);
    const seen=new Set(),configs=[];let rejectedInventory=0,rejectedUnseated=0,rejectedCoverage=0;
    const placementGroups=o.axialPlacementMode?ordered.map(c=>{
      const variants=axialPlacementVariants(c,entry.material.intervals);
      if(o.axialPlacementMode!=='edge-first')return variants;
      // Edge trials only escape collisions: a clear centred placement wins
      // (otherwise a fitting part is pushed against a stack end).
      const edgeFirst=[...variants.slice(1),variants[0]],mid=recentre({...c,stationYmm:c.stationYmm||1e-9}).stationYmm;
      const centre=edgeFirst.findIndex(v=>Math.abs(v.stationYmm-mid)<1e-3);
      return centre>0?[edgeFirst[centre],...edgeFirst.filter((_,i)=>i!==centre)]:edgeFirst;
    }):ordered.map(c=>[c]);
    const placements=o.diverseAxialCandidates?
      Array.from({length:Math.max(0,...placementGroups.map(v=>v.length))},(_,i)=>placementGroups.flatMap(v=>v[i]?[v[i]]:[])).flat():placementGroups.flat();
    for(const c of placements){
      if(!compatible(localCatalog,c,entry.bend))continue;
      if(!fitsInventory(filtered,c)){rejectedInventory++;continue;}
      if(c.segments.some(s=>![s.offsetX??0,s.offsetZ??0].every(v=>Number.isFinite(v)&&Math.abs(v)<1e-8))){rejectedUnseated++;continue;}
      if(o.allowPartialCoverage!==true){
        const intervals=c.setupFamily==='TWO_BANK'&&c.flip?entry.material.intervals.map(([a,b])=>[-b,-a]):entry.material.intervals;
        const shifted=intervals.map(([a,b])=>[a+(c.stationYmm??0),b+(c.stationYmm??0)]);
        if(!assessToolSupport(assessCoverage({materialIntervals:shifted,segments:c.segments,activeBankId:c.activeBankId,exact:entry.material.exact}),policy).accepted){rejectedCoverage++;continue;}}
      if(c.segments.some(s=>!Number.isFinite(s.centerY)||!Number.isFinite(s.lengthMm)||s.lengthMm<=0||!segmentSeated(s,bedLengthMm)))continue;
      const key=configKey(c);if(seen.has(key))continue;seen.add(key);configs.push(c);
      if(configs.length>=o.maxCandidatesPerBend)break;
    }
    const oriented=o.allowMirroredToolProfiles?appendMirroredToolSetups(configs,{maxCandidates:o.maxCandidatesIncludingMirrors??configs.length*4}):configs;
    byBend.set(String(entry.bend.id),oriented);diagnostics.generatedByBend.push({bendId:entry.bend.id,candidates:oriented.length,
      generatedAvailable:generated.candidates.length,rejectedInventory,rejectedUnseated,rejectedCoverage,
      retainedPunchIds:[...new Set(configs.map(c=>c.punchId))],retainedDieIds:[...new Set(configs.map(c=>c.dieId))],
      truncated:configs.length<placements.length,axialPlacementMode:o.axialPlacementMode??null,
      undercoverageCandidatesIncluded:policy.maxUnsupportedEndMm>0||o.allowPartialCoverage===true});
  }
  // Keep one kind mounted: this bend's punch bank with another bend's die bank (and
  // vice versa), so a shorter bend after a long one needs a punch-only edit.
  const policyMixes=new Map(),keptMixes=new Map();
  const bankTokens=new Map(),parts=new WeakMap();
  function bankParts(c,kind){
    let pair=parts.get(c);if(!pair){pair={};parts.set(c,pair);}
    if(!pair[kind]){
      const segments=c.segments.filter(s=>s.kind===kind);
      const key=JSON.stringify([c[kind+'Id'],segments.map(s=>[s.lengthMm,s.centerY,s.offsetX??0,s.offsetZ??0,s.mirror??false,s.bankId??null,s.punchId??null])]);
      if(!bankTokens.has(key))bankTokens.set(key,bankTokens.size);
      const stockPosition=JSON.stringify([kind,c[kind+'Id'],segments.map(s=>[s.lengthMm,s.centerY])]);
      pair[kind]={segments,token:bankTokens.get(key),stockPosition};
    }
    return pair[kind];
  }
  for(const entry of spans){
    const id=String(entry.bend.id),own=byBend.get(id)??[],seen=new Set(own.map(configKey)),mixes=[];
    let acceptedMixes=0;
    // Every distinct bank another bend may leave mounted (not only its first choices).
    const kept=new Map();
    for(const [otherId,others]of byBend)if(otherId!==id)for(const other of others)for(const kind of['die','punch']){
      const k=bankParts(other,kind).stockPosition;
      if(!kept.has(k))kept.set(k,other);}
    {const others=[...new Set(kept.values())],tried=new Set();
      for(const other of others)for(const mine of own){
        // Segments are bed positions, so a kept bank does not depend on the part station.
        if(mine.mirror||other.mirror)continue;
        for(const keep of['die','punch']){
          const edit=keep==='die'?'punch':'die';
          const edited=bankParts(mine,edit),retained=bankParts(other,keep);
          const mixKey=JSON.stringify([keep,mine.stationYmm??0,mine.activeBankId??null,edited.token,retained.token]);
          if(tried.has(mixKey))continue;
          tried.add(mixKey);
          const c={...mine,[`${keep}Id`]:other[`${keep}Id`],keptKind:keep,keptSameProfiles:mine[`${edit}Id`]===other[`${edit}Id`],
            segments:[...edited.segments,...retained.segments]};
          const key=configKey(c);if(seen.has(key))continue;seen.add(key);
          if(!allowedForBend(o.toolingByBend,entry.bend.id,'punch',c.punchId)||!allowedForBend(o.toolingByBend,entry.bend.id,'die',c.dieId)||
             !compatible(filtered,c,entry.bend)||!fitsInventory(filtered,c)||c.segments.some(s=>!segmentSeated(s,bedLengthMm)))continue;
          const intervals=c.flip?entry.material.intervals.map(([a,b])=>[-b,-a]):entry.material.intervals;
          const shifted=intervals.map(([a,b])=>[a+(c.stationYmm??0),b+(c.stationYmm??0)]);
          if(!assessToolSupport(assessCoverage({materialIntervals:shifted,segments:c.segments,activeBankId:c.activeBankId,exact:entry.material.exact}),policy).accepted)continue;
          acceptedMixes++;
          if(c.keptSameProfiles)mixes.push(c);
        }
      }
    }
    policyMixes.set(id,acceptedMixes);
    // Banks of longer bends first: those are the ones already mounted when this bend runs.
    const span=c=>Math.max(...c.segments.map(s=>Math.abs(s.centerY)+s.lengthMm/2));
    // Same profiles as the kept setup first: switching to them is only a segment edit.
    mixes.sort((a,b)=>span(b)-span(a));
    keptMixes.set(id,mixes.slice(0,o.maxCandidatesPerBend));
  }
  diagnostics.keptKindMixes=Object.fromEntries(policyMixes);
  if(o.allowMirroredToolProfiles)diagnostics.toolOrientationSearch={baseCandidatesPreserved:true,mountingFixturesVerified:false,scope:TOOL_ORIENTATION_SCOPE};
  return{byBend,keptMixes,diagnostics,bedLengthMm};
}

function operations(model,node,allowPartial,options){
  if(options.operationTemplates){
    return options.operationTemplates.flatMap((t,index)=>{
      const bend=model.bends.find(b=>String(b.id)===String(t.bendId));if(!bend)throw new Error('Operation template references an unknown bend.');
      const fromAmount=t.fromAmount??t.from?.[bend.id]??0,target=t.toAmount??t.to?.[bend.id]??1;
      if(!(target>fromAmount&&target<=1&&fromAmount>=0))throw new Error('Template amounts must increase within [0,1].');
      if(Math.abs(node.amounts[bend.id]-fromAmount)>EPS||t.from&&!sameAmounts(model,node.amounts,t.from))return[];
      if(t.requires&&Object.entries(t.requires).some(([id,q])=>(node.amounts[id]??-1)<q-EPS))return[];
      if(!strokeAllowed(model,bend,node.amounts))return[];
      const to=strokeAmounts(bend,{...node.amounts,[bend.id]:target});
      if(t.to&&model.bends.some(b=>b.strokeLeadId==null&&Math.abs((t.to[b.id]??to[b.id])-to[b.id])>EPS))throw new Error('A template must change only its active bend.');
      return[{bend,target,to,id:t.operationId??t.id??`operation-${index}`}];
    });
  }
  return model.bends.flatMap(bend=>{
    const q=node.amounts[bend.id];if(q>=1-EPS||!strokeAllowed(model,bend,node.amounts))return[];
    const targets=partialBendTargets(bend,q,allowPartial,options);
    return targets.map(target=>({bend,target,to:strokeAmounts(bend,{...node.amounts,[bend.id]:target}),id:`${String(bend.id)}:${q.toFixed(7)}>${target.toFixed(7)}`}));
  });
}

/**
 * Async, bounded beam search over arbitrary bend IDs and counts. Feasibility is
 * sampled; tool changes are optimized only within visited states and generated
 * setups. Transfer/reorientation between stages remains explicitly pending.
 */
/** Options that only steer or bound the search. Every other option may change a
 * transition check, so it is part of a shared cache key. */
const SEARCH_ONLY_OPTIONS=new Set(['timeBudgetMs','maxStates','maxTransitions','maxChecksPerState','maxSuccessorsPerState',
  'beamWidth','maxCandidatesPerBend','maxCandidatesIncludingMirrors','relativePhaseBudgets','diverseAxialCandidates',
  'preserveSearchHistory','stopAtFirstClear','signal','transitionCache','reverseAllClosures','axialPlacementMode',
  'punchId','dieId','allowedPunchIds','allowedDieIds','toolingByBend',
  'partialAnglesDegrees','allowMirroredToolProfiles','validateFullRouteDuringSearch','validateCompleteCandidate','candidateOrdering','settleAfterCompleteMs','incumbentComparisonTransitions']);
/** Optional cheap ordering (options.candidateOrdering==='heuristic'): bends with fewer
 * failed transitions so far first, then the smaller formed side (short flange).
 * Only changes the order of attempts; every transition is still checked. */
export function heuristicBendCompare(a,b,failures,areaOf){
  const fa=failures.get(String(a))??0,fb=failures.get(String(b))??0;
  return fa-fb||areaOf(a)-areaOf(b);
}
const stableJson=v=>Array.isArray(v)?'['+v.map(stableJson).join(',')+']':v&&typeof v==='object'?
  '{'+Object.keys(v).sort().filter(k=>typeof v[k]!=='function'&&v[k]!==undefined).map(k=>JSON.stringify(k)+':'+stableJson(v[k])).join(',')+'}':JSON.stringify(v)??'null';
/** Transition results shared between attempts on the same model and catalog
 * (options.transitionCache: Map). The key already holds amounts, bend, target,
 * tool config and orientation; this prefix adds every check parameter. */
const cacheInputs=new WeakMap();
export function transitionCacheView(o,model,catalog){
  const shared=o.transitionCache;if(!(shared instanceof Map))return new Map();
  // A caller can supply the same Map after editing an input in place. Compare
  // once per attempt, never for each sampled transition.
  const input=JSON.stringify([model,catalog]);
  let scope=cacheInputs.get(shared);
  if(!scope||scope.input!==input){
    scope={input,revision:(scope?.revision??0)+1};
    cacheInputs.set(shared,scope);
  }
  const prefix=scope.revision+'#'+stableJson(Object.fromEntries(Object.entries(o).filter(([k])=>!SEARCH_ONLY_OPTIONS.has(k))))+'#';
  return{get:k=>shared.get(prefix+k),set:(k,v)=>{shared.set(prefix+k,v);return this;},has:k=>shared.has(prefix+k)};
}

// Use the exact material states and the configuration recorded by the
// validator, including segment IDs for collision witnesses and playback.
// Search-state equivalence (amountKey/configKey) permits coarser identities.
function transitionKey(from,to,bendId,config,operationId){
  const {amount,ramGapMm,contactGapMm,...setup}=config;
  return JSON.stringify([from,to,String(bendId),operationId,stableJson({...setup,bendId})]);
}

export async function planSequence({model:inputModel,catalog,options={}}={},onProgress=()=>{},validateGroupedCandidate=null){
  const started=performance.now(),o={...defaults,...options},diagnostics={startedUtc:new Date().toISOString(),statesExpanded:0,
    transitionsChecked:0,cacheHits:0,beamPruned:0,rejections:{},rejectionExamples:[],partialFallbackUsed:false,
    searchComplete:false,globalOptimumProven:false,continuousCollisionDetection:false,physicalMachineCommandsIssued:false};
  let model,validation,supportPolicy;
  const emit=value=>onProgress({...value,elapsedMs:performance.now()-started,statesExpanded:diagnostics.statesExpanded,transitionsChecked:diagnostics.transitionsChecked});
  const resultBase=()=>({stages:[],transfers:[],cost:null,toolSupportPolicy:supportPolicy,completeSequenceVerified:false,completeManufacturingVerified:false,
    transfersVerified:false,diagnostics:{...diagnostics,elapsedMs:performance.now()-started}});
  try{
    if(o.plannerMode!==undefined&&!['lite','full'].includes(o.plannerMode))throw new Error('Invalid plannerMode');
    supportPolicy=toolSupportPolicy(o);
    validation=validateModel(inputModel);model=normalizedModel(inputModel);
    if(!model.bends.length)return{...resultBase(),status:'FLAT_NO_BENDS',allStagesSampleClear:true,allStagesOperatorSupportPass:true,
      cost:scoreToolSetupSequence([]).cost,diagnostics:{...diagnostics,input:validation,reason:'The part has no bends.'}};
    if(!Array.isArray(catalog?.tools))throw new Error('A real tooling catalog is required.');
    for(const key of['beamWidth','maxStates','maxTransitions','maxCandidatesPerBend','foldSamples','maxAccessSamples','maxVertices','maxSuccessorsPerState','maxChecksPerState'])
      if(!Number.isInteger(o[key])||o[key]<(key==='foldSamples'?2:1))throw new Error(`Invalid ${key}`);
    for(const key of['timeBudgetMs','foldStepDegrees','accessPointStepMm','ramStepMm','frontMarginMm','partialAngleDegrees'])
      if(!Number.isFinite(o[key])||o[key]<=0)throw new Error(`Invalid ${key}`);
    for(const key of['clearanceMm','contactGapMm','additionalRetractMm'])if(!Number.isFinite(o[key])||o[key]<0)throw new Error(`Invalid ${key}`);
    if(validation.vertices>o.maxVertices)return{...resultBase(),status:'SEARCH_BUDGET_EXHAUSTED',diagnostics:{...diagnostics,input:validation,budgetReason:'mesh-vertex-budget'}};
  }catch(error){return{...resultBase(),status:'UNSUPPORTED_INPUT',diagnostics:{...diagnostics,error:error.message}};}
  // Invariant: the flat blank never touches itself. A hit is a model/exception
  // defect; report it instead of searching (every transition would fail).
  let setups,checks,setupError;
  const prepare=()=>generateSequenceSetups({model,catalog,options:o});
  try{
    if(o.plannerMode!=='full'&&validateGroupedCandidate&&foldPool?.foldPoolSize()>1&&
       longCompoundFamily(model,o.bedLengthMm??catalog.machine?.bedLengthMm,o)){
      const jobs=['flat','final'].map(kind=>({kind,clearanceMm:o.clearanceMm}));
      checks=foldPool.checkIndependentJobs(model,catalog,jobs,
        job=>(job.kind==='flat'?checkFlatSelfContact:checkFinalSelfContact)(model,job),
        ()=>{try{setups=prepare();}catch(error){setupError=error;}}).results;
    }
    const flat=checks?.[0]??checkFlatSelfContact(model,{clearanceMm:o.clearanceMm});diagnostics.flatSelfCheck=flat;
    if(flat.status!=='FLAT_CLEAR')return{...resultBase(),status:'FLAT_BLANK_SELF_CONTACT',
      diagnostics:{...diagnostics,input:validation,elapsedMs:performance.now()-started,
        reason:'The unbent blank reports self-contact: false contact in the model or collision exceptions, not a manufacturing limit.'}};
    const final=checks?.[1]??checkFinalSelfContact(model,{clearanceMm:o.clearanceMm});diagnostics.finalSelfCheck=final;
    if(final.status!=='FINAL_CLEAR')return{...resultBase(),status:'FINAL_SHAPE_SELF_CONTACT',
      diagnostics:{...diagnostics,input:validation,elapsedMs:performance.now()-started,
        reason:'The designed final shape reports self-contact: false contact in the model or collision exceptions, not a manufacturing limit.'}};}
  catch(error){return{...resultBase(),status:'UNSUPPORTED_INPUT',diagnostics:{...diagnostics,error:error.message}};}
  try{if(setupError)throw setupError;setups??=prepare();}catch(error){return{...resultBase(),status:'UNSUPPORTED_INPUT',diagnostics:{...diagnostics,error:error.message}};}
  diagnostics.input=validation;diagnostics.candidates=setups.diagnostics;diagnostics.budgets={...o,signal:undefined,initialSetup:undefined,setups:undefined,initialSegments:undefined,operationTemplates:undefined,validateCompleteCandidate:undefined,transitionCache:undefined};
  if(model.bends.some(b=>b.strokeLeadId==null&&!setups.byBend.get(String(b.id))?.length))return{...resultBase(),status:'NO_COMPATIBLE_TOOL_CANDIDATES'};
  const partialBanks=new Map();
  function configsAt(operation){
    const {bend,target}=operation,id=String(bend.id),base=setups.byBend.get(id);
    const rule=o.toolingByBend?.[id];
    if(!rule||!(target>0&&target<1))return base;
    const key=id+'|'+target;
    if(partialBanks.has(key))return partialBanks.get(key);
    const nominal=nominalBendAt(bend,target);
    const tools=catalog.tools.filter(t=>toolAllowedAt(t,bend,o,target));
    let choices=base;
    if(tools.some(t=>!allowedForBend(o.toolingByBend,id,t.kind,t.id))){
      const filtered={...catalog,tools},material=materialIntervals(bend),policy=toolSupportPolicy(o);
      const candidates=o.setups?.length?o.setups.map(s=>s.config??s):
        generateToolSetupCandidates({catalog:filtered,bend:nominal,materialIntervals:material.intervals,exact:material.exact,
          bedLengthMm:setups.bedLengthMm,maxFullBanksPerTool:o.maxFullBanksPerTool,...policy}).candidates
        .filter(c=>assessToolSupport(c.coverage,policy).accepted).map(c=>c.config);
      // Only add the newly admitted profiles; final-stroke candidates retain
      // their complete original ordering. Every trial still runs the full checks.
      const added=candidates.filter(c=>(!rule.dieIds.includes(String(c.dieId))||!rule.punchIds.includes(String(c.punchId)))&&
        compatible(filtered,c,nominal)&&fitsInventory(filtered,c)&&c.segments.every(s=>segmentSeated(s,setups.bedLengthMm)&&
          [s.offsetX??0,s.offsetZ??0].every(x=>Number.isFinite(x)&&Math.abs(x)<EPS)));
      if(added.length)choices=[...base,...added.slice(0,o.maxCandidatesPerBend)];
    }
    partialBanks.set(key,choices);return choices;
  }
  const initial=Object.fromEntries(model.bends.map(b=>[b.id,0])),cache=transitionCacheView(o,inputModel,catalog),support=new Map(model.bends.map(b=>[String(b.id),buildOperatorSupport(model,b.id)]));
  const scoreOptions=o.initialSetup?{initialSetup:o.initialSetup}:{};
  const zero=scoreToolSetupSequence([],scoreOptions).cost,difficulty=new Map(),bendFailures=new Map(),successfulFull=new Set(),fallbackPrefixes=new Map();let best=null,budgetReason=null;
  const family=longCompoundFamily(model,setups.bedLengthMm,o);
  async function groupedRoute(stats,guardProbe,early){
    const screened=new Set(),limit=Math.min(48,o.maxTransitions);
    const inputFor=({bend,from,to,config})=>{
      const bendId=bend.id,operationId=String(bendId)+':0.0000000>1.0000000';
      const key=transitionKey(from,to,bendId,config,operationId);
      return {key,stage:{bendId,operationId,from,to,config}};
    };
    function checkLocal(stage,options=o){
      return evaluateSequenceTransition({model,catalog,...stage,options,preparedSupport:support.get(String(stage.bendId))},guardProbe);
    }
    async function screenAndStart(batch){
      const rows=new Map();
      for(const item of batch){
        const id=String(item.bend.id);
        if(!rows.has(id))rows.set(id,[]);
        rows.get(id).push(inputFor(item));
      }
      for(const orientations of [...rows.values()].reverse()){
        let possible=false;
        for(const {key,stage} of orientations){
          const hit=cache.get(key);
          if(hit){if(hit.feasible){possible=true;break;}continue;}
          if(screened.has(key)){possible=true;break;}
          guardProbe();
          const entry=screenTransitionEndpoints({model,catalog,...stage,options:o,preparedSupport:support.get(String(stage.bendId))},guardProbe);
          stats.endpointChecks++;
          if(entry.prechecked){screened.add(key);possible=true;break;}
          cache.set(key,entry);
        }
        if(!possible)return false;
      }
      if(!foldPool||foldPool.foldPoolSize()<2)return;
      const fresh=batch.filter(row=>!row.config.flip).map(inputFor).filter(({key})=>!cache.has(key)).slice(0,3);
      if(fresh.length<3||stats.checks+fresh.length>limit)return;
      const serial={...o,parallelWithinStage:false};
      const transferable=Object.fromEntries(Object.entries(serial).filter(([key,value])=>typeof value!=='function'&&
        !['signal','transitionCache'].includes(key)));
      const jobs=fresh.slice(1).map(({stage})=>({kind:'ordinary',stage,options:transferable}));
      stats.checks+=fresh.length;
      const {main,results}=foldPool.checkIndependentJobs(model,catalog,jobs,job=>checkLocal(job.stage,serial),()=>checkLocal(fresh[0].stage,serial));
      for(const [i,answer] of [main,...results].entries()){
        const rejected=answer?.feasible===false&&typeof answer.reason==='string';
        const verified=answer?.feasible&&answer.stage?.bendId===fresh[i].stage.bendId&&
          answer.stage.foldSamples?.length>=o.foldSamples&&answer.stage.foldSamples.every(sample=>consistentSupportCentroid(sample.operatorSupport));
        if(rejected||verified)cache.set(fresh[i].key,answer);
      }
      guardProbe();
    }
    const checkedStage=async row=>{
      const {key,stage}=inputFor(row);
      let hit=cache.get(key);
      if(!hit){
        if(stats.checks>=limit)throw new Budget('probe-transition-limit');
        stats.checks++;await new Promise(resolve=>setTimeout(resolve,0));guardProbe();
        hit=checkLocal(stage);cache.set(key,hit);
      }
      return hit.feasible&&hit.stage.toolSupport.accepted?hit.stage:null;
    };
    const routeOf=async stages=>{
      const edges=stages.map(s=>({id:s.id,config:s.config,flip:s.flip,stationShiftMm:s.config.stationYmm??0,
        coverage:s.coverage,uncoveredLengthMm:s.coverageMetrics.uncoveredLengthMm,coverageUnresolved:s.coverageMetrics.coverageUnresolved}));
      const cost=scoreToolSetupSequence(edges,scoreOptions).cost;
      if(early && (cost.toolChangeEvents!==1 || cost.segmentEditEvents!==1 || cost.partialChangeEvents || cost.fullChangeEvents))return null;
      if(best&&compareChangeClass(cost,best.cost)>0)return null;
      const complete=validateGroupedCandidate??o.validateCompleteCandidate;
      if(complete){
        const verdict=await complete({...resultBase(),status:'CANDIDATE_SEQUENCE',stages,cost,allStagesSampleClear:true,allStagesOperatorSupportPass:true});
        if(verdict!==true&&verdict?.accepted!==true)return null;
      }
      return {amounts:stages.at(-1).to,stages,edges,cost};
    };
    return probeLongCompound({model,family,setups,initial,check:checkedStage,complete:routeOf,guard:guardProbe,warm:early?screenAndStart:null});
  }

  // A verified two-bank program can replace the ordinary search only after every
  // retained final pose wide enough for the long strokes has been ruled out.
  if(family && o.plannerMode!=='full' && validateGroupedCandidate && o.foldSamples>=21 &&
      o.foldStepDegrees<=5 && o.maxTransitions>=8){
    const wide=spanCapableFinalSetups(family,setups,supportPolicy.maxUnsupportedEndMm,configKey);
    if(wide){
      const stats={checks:0,endpointChecks:0};
      const guardEarly=()=>{
        if(o.signal?.aborted)throw new Budget('cancelled');
        if(performance.now()-started>=o.timeBudgetMs*.15)throw new Budget('grouped-probe-budget');
      };
      try{
        guardEarly();
        const proof=await checkFinalToolSetups({model,catalog,setups:{...setups,byBend:wide.byBend},
          options:o,support,stopOnFirstClear:false},guardEarly);
        if(proof.allCandidatesChecked && proof.checkedPoses>0 && proof.clearPoses===0 &&
            proof.status==='NO_FINAL_TOOL_SETUP_IN_CANDIDATES'){
          const grouped=await groupedRoute(stats,guardEarly,true);
          guardEarly();
          if(grouped){
            diagnostics.transitionsChecked=stats.checks;
            diagnostics.liteGroupedProof={scope:'Retained span-capable final mounts only. Ordinary alternatives were not explored.',
              requiredSpan:wide.requiredSpan,foldChecks:stats.checks,screenedEndpoints:stats.endpointChecks,finalPoses:proof.checkedPoses,
              frontRejected:proof.frontRejected,collisions:proof.collisionChecks,proof};
            return {...resultBase(),status:'CANDIDATE_SEQUENCE',stages:grouped.stages,cost:grouped.cost,costOrder:COST_ORDER,
              allStagesSampleClear:true,allStagesOperatorSupportPass:true,coverageUnresolved:false};
          }
        }
      }catch(error){
        if(o.signal?.aborted)return {...resultBase(),status:'CANCELLED'};
        diagnostics.groupedProbeFallback=error instanceof Budget?error.reason:String(error.message??error);
      }
      diagnostics.transitionsChecked+=stats.checks;
    }
  }
  const done=a=>model.bends.every(b=>a[b.id]>=1-EPS);
  function acceptComplete(candidate){
    if(!o.validateCompleteCandidate)return true;
    guard();
    const outcome=o.validateCompleteCandidate({...resultBase(),status:'CANDIDATE_SEQUENCE',
      stages:candidate.stages,cost:candidate.cost,allStagesSampleClear:true,allStagesOperatorSupportPass:true});
    if(outcome===true||outcome?.accepted===true)return true;
    diagnostics.completeCandidateRejections=(diagnostics.completeCandidateRejections??0)+1;
    const reason=outcome?.reason??'COMPLETE_ROUTE_REJECTED';
    diagnostics.rejections[reason]=(diagnostics.rejections[reason]??0)+1;
    emit({phase:'complete-candidate-rejected',reason});
    return false;
  }
  let compareUntil=Infinity,settleFrom=null;
  const primaryMinimum=()=>best&&best.cost.toolChangeEvents===0&&best.stages.every(s=>s.toolSupport.accepted);
  function guard(){
    if(o.signal?.aborted)throw new Budget('cancelled');
    // Optional: once a complete route exists, look for a cheaper one only this long.
    if(best&&o.settleAfterCompleteMs!=null){settleFrom??=performance.now();
      if(performance.now()-settleFrom>=o.settleAfterCompleteMs)throw new Budget('incumbent-comparison-budget');}
    if(performance.now()-started>=o.timeBudgetMs)throw new Budget('time-budget');
    if(diagnostics.transitionsChecked>=compareUntil)throw new Budget('incumbent-comparison-budget');
    if(diagnostics.transitionsChecked>o.maxTransitions)throw new Budget('transition-budget');
  }
  function reject(answer,stages){
    diagnostics.rejections[answer.reason]=(diagnostics.rejections[answer.reason]??0)+1;
    if(diagnostics.rejectionExamples.length<16)diagnostics.rejectionExamples.push(answer);
    const w=answer.witness;if(!w)return;
    const records=diagnostics.collisionWitnesses??=[];
    const existing=records.findIndex(s=>s.bendId===w.bendId&&s.phase===w.phase&&s.collision.bodyIds.join('|')===w.collision.bodyIds.join('|'));
    const depth=stages?.length??0;
    if(existing>=0&&depth<=(records[existing].prefix?.length??0)||existing<0&&records.length>=16)return;
    const entry={...w,prefix:stages?.map(s=>s.bendId)};
    if(existing>=0)records[existing]=entry;else records.push(entry);
  }
  const nodeCompare=(a,b)=>compareCost(a.cost,b.cost)||a.stages.length-b.stages.length||
    sum(model.bends.map(j=>b.amounts[j.id]))-sum(model.bends.map(j=>a.amounts[j.id]));
  // Find an incumbent by descending immediately after a feasible operation.
  // Breadth-only enumeration can consume the entire budget at the flat state.
  const operationOrder=(node,partial)=>operations(model,node,partial,o).sort((a,b)=>{
    if(partial){
      // Learn which full bend failed after other material had already formed.
      // Try its pre-bend early, then leave continuation until other fresh bends
      // have had a chance. This is a search heuristic, not a prescribed order.
      const deferred=Number(node.amounts[a.bend.id]>0)-Number(node.amounts[b.bend.id]>0);
      if(deferred)return deferred;
      const priority=id=>successfulFull.has(String(id))?0:difficulty.get(String(id))??0;
      const hard=priority(b.bend.id)-priority(a.bend.id);if(hard)return hard;
      if(String(a.bend.id)===String(b.bend.id)&&priority(a.bend.id)>0)return a.target-b.target;
    }else{const continuation=Number(node.amounts[b.bend.id]>0)-Number(node.amounts[a.bend.id]>0);if(continuation)return continuation;}
    if(o.candidateOrdering==='heuristic'){const h=heuristicBendCompare(a.bend.id,b.bend.id,bendFailures,id=>support.get(String(id)).minorBranch.areaMm2);if(h)return h;}
    return fullSpan(a.bend).width-fullSpan(b.bend).width||support.get(String(a.bend.id)).minorBranch.areaMm2-support.get(String(b.bend.id)).minorBranch.areaMm2||b.target-a.target;
  });
  async function seed(node,partial,depth,phaseBudget,seen){
    guard();
    const phaseEnded=()=>diagnostics.transitionsChecked>=phaseBudget.transitions||diagnostics.statesExpanded>=phaseBudget.states||performance.now()>=phaseBudget.until;
    if(phaseEnded()){diagnostics.phaseQuotaReached=true;return false;}
    const depthLimit=o.operationTemplates?.length??model.bends.length*(partial?2:1);
    if(depth>=depthLimit)return false;
    const nk=JSON.stringify([amountKey(model,node.amounts),node.stages.length?configKey(node.stages.at(-1).config):null,
      ...(o.preserveSearchHistory?[node.stages.map(s=>[s.id,configKey(s.config),s.flip])]:[])]);
    if(seen.has(nk)&&compareCost(seen.get(nk),node.cost)<=0)return false;seen.set(nk,node.cost);
    if(diagnostics.statesExpanded>=o.maxStates)throw new Budget('state-budget');
    diagnostics.statesExpanded++;
    let checks=0,successors=0;
    const groups=new Map();for(const op of operationOrder(node,partial)){const id=String(op.bend.id);if(!groups.has(id))groups.set(id,[]);groups.get(id).push(op);}
    for(const group of groups.values()){
      const last=node.stages.at(-1)?.config??o.initialSetup;
      // Round-robin over tool length layouts so the per-group quota reaches a
      // short segment even when long banks with many profiles come first; a
      // long bank can hit fixed material outside the active span.
      const base=setups.byBend.get(String(group[0].bend.id)),domains=group.map(configsAt);
      const mixed=domains.some(d=>d!==base),allowed=mixed?new Map(group.map((op,i)=>[op,new Set(domains[i])])):null;
      const byLayout=new Map();for(const cfg of mixed?new Set(domains.flat()):base){
        const k=(cfg.segments??[]).map(s=>`${s.kind}:${s.lengthMm}`).join('|');if(!byLayout.has(k))byLayout.set(k,[]);byLayout.get(k).push(cfg);}
      const list=[];for(let i=0,more=true;more;i++){more=false;for(const l of byLayout.values())if(i<l.length){list.push(l[i]);more=true;}}
      // Mounted setup first, then edits of only one kind's segments; the rest
      // keeps the round-robin order (profile diversity within the quota).
      const rank=cfg=>{const k=toolChangeKind(last,cfg),v={none:0,segments:1,profile:10};return Math.min(2,v[k.punch]+v[k.die]);};
      if(Array.isArray(last?.segments)){const ranks=new Map(list.map(c=>[c,rank(c)]));list.sort((a,b)=>ranks.get(a)-ranks.get(b));}
      let groupChecks=0;const perGroupLimit=Math.max(4,Math.floor(o.maxChecksPerState/groups.size));
      // If a complete bend is blocked, try its partial target at this same
      // setup/orientation before enumerating every other full-bend setup.
      groupLoop:for(const cfg of list)for(const flip of[false,true])for(const operation of group){
        if(allowed&&!allowed.get(operation).has(cfg))continue;
        guard();if(phaseEnded()||checks>=o.maxChecksPerState||successors>=o.maxSuccessorsPerState){diagnostics.seedPruned=true;return false;}
        if(groupChecks>=perGroupLimit){diagnostics.seedPruned=true;break groupLoop;}
        checks++;groupChecks++;
        const config={...cfg,flip,bedLengthMm:setups.bedLengthMm},key=transitionKey(node.amounts,operation.to,operation.bend.id,config,operation.id);
        let checked=cache.get(key);
        if(checked){
          diagnostics.cacheHits++;
          if(!checked.feasible)bendFailures.set(String(operation.bend.id),(bendFailures.get(String(operation.bend.id))??0)+1);
        }else{
          if(diagnostics.transitionsChecked>=o.maxTransitions)throw new Budget('transition-budget');
          diagnostics.transitionsChecked++;emit({phase:'checking-operation',search:'depth-first-incumbent',bendId:operation.bend.id,operationId:operation.id,depth,partialFallback:partial});
          await new Promise(resolve=>setTimeout(resolve,0));guard();
          checked=evaluateSequenceTransition({model,catalog,from:node.amounts,to:operation.to,bendId:operation.bend.id,config,operationId:operation.id,options:o,preparedSupport:support.get(String(operation.bend.id))},guard);
          cache.set(key,checked);
          if(!checked.feasible){reject(checked,node.stages);
            bendFailures.set(String(operation.bend.id),(bendFailures.get(String(operation.bend.id))??0)+1);
            if(!partial&&depth>0&&operation.target===1&&['FOLD_INTERFERENCE','NO_FRONT_ROUTE_IN_TESTED_LIFTS','ADAPTIVE_ACCESS_INTERFERENCE','RAM_SWEEP_INTERFERENCE'].includes(checked.reason)){
              const id=String(operation.bend.id);difficulty.set(id,Math.max(difficulty.get(id)??0,depth+1));
              // Keep the already checked prefix, rewind its last operation,
              // and insert the difficult bend's partial stroke before closure.
              // This candidate comes from this search's own collision evidence.
              const stages=node.stages.slice(0,-1),edges=node.edges.slice(0,-1),prefix={
                amounts:stages.at(-1)?.to??initial,stages,edges,cost:scoreToolSetupSequence(edges,scoreOptions).cost,
                difficultBendId:id,blockedDepth:depth};
              const previous=fallbackPrefixes.get(id);
              if(!previous||prefix.stages.length>previous.stages.length||prefix.stages.length===previous.stages.length&&compareCost(prefix.cost,previous.cost)<0)fallbackPrefixes.set(id,prefix);
            }}
        }
        if(checked.feasible&&!partial&&operation.target===1)successfulFull.add(String(operation.bend.id));
        // A wrong orientation says nothing about tool length: it must not use
        // up the quota that should reach other layouts.
        if(!checked.feasible&&checked.reason==='MAIN_BRANCH_NOT_FRONT'){groupChecks--;checks--;}
        if(!checked.feasible)continue;
        const stage={...checked.stage,id:operation.id},edges=[...node.edges,{id:stage.id,config:stage.config,flip,stationShiftMm:config.stationYmm??0,
          coverage:checked.coverage,uncoveredLengthMm:checked.metrics.uncoveredLengthMm,coverageUnresolved:checked.metrics.coverageUnresolved}];
        const candidate={amounts:operation.to,stages:[...node.stages,stage],edges,cost:scoreToolSetupSequence(edges,scoreOptions).cost};
        if(best&&compareCost(candidate.cost,best.cost)>0)continue;
        successors++;
        if(done(candidate.amounts)){
          if((!best||nodeCompare(candidate,best)<0)&&acceptComplete(candidate)){best=candidate;emit({phase:'candidate-found',depth:depth+1,cost:best.cost});}
          if(primaryMinimum()){diagnostics.stoppedAtPrimaryCostLowerBound=true;return true;}
        }else if(await seed(candidate,partial,depth+1,phaseBudget,seen))return true;
      }
    }return false;
  }
  // A route with a tool change may only need it because earlier bends were
  // generated with their own setups. Retry the whole route on the final setup:
  // first on its own stations, then with one extra separated station built
  // from spare inventory. The part is moved to a station by shifting every
  // segment by minus its centre (centres are relative to the active span).
  // Every transition repeats the full check; own small time budget.
  async function reuseFinalSetup(route,stock=catalog){
    const final=route.stages.at(-1).config,until=performance.now()+o.timeBudgetMs*.125;
    const stats=diagnostics.finalSetupReuseStats={layouts:0,checks:0,coverageRejectedLayouts:0,attempts:[]};
    const tryLayout=async(layout,extra)=>{
      stats.layouts++;
      const stations=[...new Set(layout.segments.filter(s=>s.kind==='punch').map(s=>Number(s.centerY.toFixed(4))))];
      // Moving to another existing station must select and verify that bank;
      // never silently turn an explicit active bank into union-of-stock support.
      const banks=layout.activeBankId==null?[undefined]:[...new Set([layout.activeBankId,...layout.segments.map(s=>s.bankId).filter(id=>id!=null)])];
      const candidates=[];
      for(const [i,stage] of route.stages.entries()){
        const flip=route.edges[i]?.flip??stage.config.flip??false;
        const own=configKey(stage.config)===configKey(final);
        const shifts=own?[0]:extra!=null?[extra,...stations.filter(c=>c!==extra)]:[0,...stations];
        const bend=model.bends.find(b=>String(b.id)===String(stage.bendId)),material=materialIntervals(bend);
        const intervals=material.intervals.map(([a,b])=>flip?[-b,-a]:[a,b])
          .map(([a,b])=>[a+(layout.stationYmm??0),b+(layout.stationYmm??0)]);
        const choices=[];
        for(const station of new Set(shifts))for(const activeBankId of banks){
          if(performance.now()>until||o.signal?.aborted)return null;
          const segments=layout.segments.map(s=>({...s,centerY:s.centerY-station}));
          const coverage=assessCoverage({materialIntervals:intervals,segments,activeBankId,exact:material.exact});
          if(assessToolSupport(coverage,o).accepted)choices.push({station,config:{...layout,activeBankId,segments,flip,bedLengthMm:setups.bedLengthMm}});
        }
        // A bank must support every stroke to replace every setup. Testing an
        // already impossible layout on its first, shorter bends wastes full
        // collision checks (including trials using hypothetical extra stock).
        if(!choices.length){stats.coverageRejectedLayouts++;return null;}
        candidates.push(choices);
      }
      const stages=[],edges=[];
      for(const [i,stage] of route.stages.entries()){
        const from=i?stages[i-1].to:initial,bendId=stage.bendId,flip=route.edges[i]?.flip??stage.config.flip??false;
        let found=null;
        for(const {station,config} of candidates[i]){
          if(performance.now()>until||o.signal?.aborted)return null;
          const key=transitionKey(from,stage.to,bendId,config,stage.id);
          let checked=cache.get(key);
          if(!checked){
            stats.checks++;await new Promise(resolve=>setTimeout(resolve,0));
            checked=evaluateSequenceTransition({model,catalog,from,to:stage.to,bendId,config,operationId:stage.id,options:o,preparedSupport:support.get(String(bendId))},()=>{if(o.signal?.aborted)throw new Budget('cancelled');});
            cache.set(key,checked);
          }else diagnostics.cacheHits++;
          const ok=checked.feasible&&checked.stage.toolSupport.accepted;
          if(stats.attempts.length<24)stats.attempts.push({bendId,extra,station,activeBankId:config.activeBankId,reason:ok?'OK':checked.feasible?'UNSUPPORTED':checked.reason,bodies:checked.hits?.[0]?.bodyIds});
          if(ok){found={checked,station};break;}
        }
        if(!found)return null;
        const st={...found.checked.stage,id:stage.id};stages.push(st);
        edges.push({id:st.id,config:st.config,flip,stationShiftMm:(layout.stationYmm??0)+found.station,coverage:found.checked.coverage,
          uncoveredLengthMm:found.checked.metrics.uncoveredLengthMm,coverageUnresolved:found.checked.metrics.coverageUnresolved});
      }
      const candidate={amounts:route.amounts,stages,edges,cost:scoreToolSetupSequence(edges,scoreOptions).cost};
      return compareCost(candidate.cost,route.cost)<0?candidate:null;
    };
    const direct=await tryLayout(final,null);if(direct)return direct;
    // Extra station: the shortest spare punch/die length that covers the
    // longest earlier punch, placed beyond the final layout with a free gap.
    const earlier=route.stages.filter(s=>configKey(s.config)!==configKey(final)&&s.config.punchId===final.punchId&&s.config.dieId===final.dieId);
    if(!earlier.length)return null;
    const need=Math.max(...earlier.flatMap(s=>s.config.segments.filter(x=>x.kind==='punch').map(x=>x.lengthMm)));
    const tool=kind=>stock.tools.find(t=>String(t.id)===String(final[kind+'Id']));
    const lengths=[...new Set((tool('punch')?.inventory??[]).map(i=>i.lengthMm))].filter(l=>l>=need-1e-6).sort((x,y)=>x-y);
    const half=setups.bedLengthMm/2;
    // If the final layout already holds the only piece of that length, also
    // try it with that piece swapped for the next longer one (inner edge kept),
    // freeing the short piece for the extra station.
    const bases=[final];
    for(const swap of lengths.slice(1)){
      const segs=final.segments.map(s=>{if(Math.abs(s.lengthMm-need)>1e-6)return s;
        const out=Math.sign(s.centerY||1),c=s.centerY+out*(swap-s.lengthMm)/2;return{...s,lengthMm:swap,centerY:c};});
      if(segs.some((s,i)=>s!==final.segments[i]))bases.push({...final,segments:segs});
    }
    // Farthest placement first (near the bed end): long parts need the most room.
    for(const base of bases)for(const length of lengths)for(const gap of [Infinity,250,150])for(const side of [1,-1]){
      const bl=Math.min(...base.segments.map(s=>s.centerY-s.lengthMm/2)),bh=Math.max(...base.segments.map(s=>s.centerY+s.lengthMm/2));
      const edge=half-length/2-10,centre=side>0?Math.min(edge,bh+gap+length/2):Math.max(-edge,bl-gap-length/2);
      if(side>0?centre-length/2<bh+50:centre+length/2>bl-50)continue;
      if(Math.abs(centre)+length/2>half||Math.max(Math.abs(bl),Math.abs(bh))>half)continue;
      let extraBankId='reuse-extra';while(base.segments.some(s=>s.bankId===extraBankId))extraBankId+='-next';
      const layout={...base,segments:[...base.segments,{id:`punch-${extraBankId}`,kind:'punch',bankId:extraBankId,lengthMm:length,centerY:centre,offsetX:0,offsetZ:0,punchId:final.punchId},
        {id:`die-${extraBankId}`,kind:'die',bankId:extraBankId,lengthMm:length,centerY:centre,offsetX:0,offsetZ:0}]};
      if(!fitsInventory(stock,layout))continue;
      const found=await tryLayout(layout,centre);if(found)return found;
      if(performance.now()>until)return null;
    }
    return null;
  }
  // When other punches already hit the finished shape, work back from a
  // promising final station. Validate each edge in its actual forward
  // direction, including both access paths and the ram. Only order is reversed.
  async function seedFromClosure(final,sharedKeys){
    const byWidth=[...model.bends].sort((a,b)=>fullSpan(b).width-fullSpan(a).width);
    const widths=c=>sum(c.segments.map(s=>s.lengthMm));
    const until=started+o.timeBudgetMs*.55,limit=o.reverseAllClosures?Math.floor(o.maxTransitions*.6):Math.min(o.maxTransitions,model.bends.length*12);
    const seen=new Set();
    async function visit(to,suffix){
      guard();
      if(performance.now()>until||diagnostics.transitionsChecked>=limit||diagnostics.statesExpanded>=o.maxStates)return null;
      if(!Object.values(to).some(Boolean)){
        const edges=suffix.map(s=>({id:s.id,config:s.config,flip:s.flip,stationShiftMm:s.config.stationYmm??0,
          coverage:s.coverage,uncoveredLengthMm:s.coverageMetrics.uncoveredLengthMm,coverageUnresolved:s.coverageMetrics.coverageUnresolved}));
        return acceptComplete({stages:suffix,cost:scoreToolSetupSequence(edges,scoreOptions).cost})?suffix:null;
      }
      const previous=suffix[0]?.config,order=previous||o.reverseAllClosures?byWidth:[model.bends.find(b=>b.id===final.bendId)];
      const state=JSON.stringify([amountKey(model,to),previous&&configKey(previous),
        ...(o.preserveSearchHistory?[suffix.map(s=>[s.id,configKey(s.config),s.flip])]:[])]);
      if(seen.has(state))return null;
      seen.add(state);diagnostics.statesExpanded++;
      for(const bend of order){
        if(!to[bend.id]||bend.strokeLeadId!=null)continue;
        const from=strokeAmounts(bend,{...to,[bend.id]:0});
        if(!strokeAllowed(model,bend,from))continue;
        const configs=[...setups.byBend.get(String(bend.id))];
        if(previous){
          configs.sort((a,b)=>Number(toolSetupKey(b)===toolSetupKey(previous))-Number(toolSetupKey(a)===toolSetupKey(previous))||
            Number(b.dieId===previous.dieId)-Number(a.dieId===previous.dieId)||
            Math.abs(widths(a)-widths(previous))-Math.abs(widths(b)-widths(previous)));
        }else{
          const sameProfile=c=>c.punchId===final.config.punchId&&c.dieId===final.config.dieId&&sharedKeys.has(toolSetupKey(c));
          configs.sort((a,b)=>Number(sameProfile(b))-Number(sameProfile(a))||widths(a)-widths(b));
        }
        for(const candidate of configs)for(const flip of [false,true]){
          guard();if(performance.now()>until||diagnostics.transitionsChecked>=limit)return null;
          const config={...candidate,flip,bedLengthMm:setups.bedLengthMm},id=String(bend.id)+':0.0000000>1.0000000';
          const key=transitionKey(from,to,bend.id,config,id);
          let checked=cache.get(key);
          if(!checked){
            diagnostics.transitionsChecked++;emit({phase:'checking-operation',search:'from-final-bend',bendId:bend.id,depth:suffix.length});
            await new Promise(resolve=>setTimeout(resolve,0));guard();
            checked=evaluateSequenceTransition({model,catalog,from,to,bendId:bend.id,config,operationId:id,options:o,
              preparedSupport:support.get(String(bend.id))},guard);
            cache.set(key,checked);if(!checked.feasible)reject(checked);
          }else diagnostics.cacheHits++;
          if(!checked.feasible)continue;
          const result=await visit(from,[checked.stage,...suffix]);if(result)return result;
        }
      }
      return null;
    }
    const stages=await visit(Object.fromEntries(model.bends.map(b=>[b.id,1])),[]);
    if(!stages)return;
    const edges=stages.map(s=>({id:s.id,config:s.config,flip:s.flip,stationShiftMm:s.config.stationYmm??0,
      coverage:s.coverage,uncoveredLengthMm:s.coverageMetrics.uncoveredLengthMm,coverageUnresolved:s.coverageMetrics.coverageUnresolved}));
    best={amounts:stages.at(-1).to,stages,edges,cost:scoreToolSetupSequence(edges,scoreOptions).cost};
    diagnostics.reverseClosureProgram=true;emit({phase:'candidate-found',depth:stages.length,cost:best.cost});
    // Allow a short forward comparison after finding an incumbent with a
    // setup change. Never report it as the global minimum.
    // Counted in transitions, not wall-clock, so the press (ARM), the PC and
    // the Fusion add-in compare the same candidates and return the same order.
    if(!primaryMinimum())compareUntil=diagnostics.transitionsChecked+(o.incumbentComparisonTransitions??64);
  }
  try{
    guard();emit({phase:'checking-final-setups'});
    const bankKeys=[...setups.byBend.values()].map(list=>new Set(list.map(toolSetupKey)));
    const sharedKeys=new Set([...bankKeys[0]].filter(key=>bankKeys.every(keys=>keys.has(key))));
    if(sharedKeys.size){
      const shared={...setups,byBend:new Map([...setups.byBend].map(([id,list])=>[id,list.filter(c=>sharedKeys.has(toolSetupKey(c)))]))};
      const checked=await checkFinalToolSetups({model,catalog,setups:shared,options:o,support},guard);
      diagnostics.reusableFinalSetupCheck={status:checked.status,checkedPoses:checked.checkedPoses};
      if(checked.clearSetup)diagnostics.finalSetupCheck=checked;
    }
    // Prefer finishing on a kept bank with only one kind's segments edited
    // (e.g. long bends first, then a punch-only edit for the short ones).
    diagnostics.finalSetupCheck??=await checkFinalToolSetups({model,catalog,setups,options:o,support},guard);
    if(diagnostics.finalSetupCheck.status==='NO_FINAL_TOOL_SETUP_IN_CANDIDATES')return{...resultBase(),
      status:'NO_FINAL_TOOL_SETUP_IN_CANDIDATES',
      reason:'Ninguno de los montajes probados permite terminar el último pliegue con la pieza al frente. Revisa los choques y las longitudes de herramienta; ampliar el tiempo no resuelve este bloqueo dentro de esos montajes.'};
    if(!o.operationTemplates&&!o.setups&&!o.initialSetup&&!o.initialSegments&&o.fixedStations==null){
      const conflict=diagnosePairedClosure({model,catalog,options:o},guard);
      // Forward and reverse search may reuse banks from other operations.
      // A wall measured against the active profile cannot reject a bank that
      // uses an offset segment or a second punch at a different clamping height.
      const usesOtherPunchPlacement=config=>config.segments.some(segment=>segment.kind==='punch'&&(
        (segment.offsetX??0)!==0||(segment.offsetZ??0)!==0||
        (segment.punchId!=null&&String(segment.punchId)!==String(config.punchId))));
      if(conflict&&conflict.pair.every(id=>!setups.byBend.get(String(id))?.some(usesOtherPunchPlacement))){
        diagnostics.pairedClosure=conflict;
        return{...resultBase(),status:conflict.status,reason:conflict.reason};
      }
    }
    // Reuse a bank that can finish the part from the first (shorter) bends.
    // A clear final pose only ranks candidates; every transition is still checked.
    const finalKey=diagnostics.finalSetupCheck.clearSetup?toolSetupKey(diagnostics.finalSetupCheck.clearSetup.config):null;
    if(finalKey)for(const list of setups.byBend.values())list.sort((a,b)=>Number(toolSetupKey(b)===finalKey)-Number(toolSetupKey(a)===finalKey));
    if(!o.operationTemplates&&(o.reverseAllClosures||(model.bends.every(b=>Math.abs(b.signedAngle)<=Math.PI/2+EPS)&&
      diagnostics.finalSetupCheck.byBend.some(row=>row.blocked.some(hit=>hit.collision.bodyKinds.includes('punch')))))){
      await seedFromClosure(diagnostics.finalSetupCheck.clearSetup,sharedKeys);
    }
    const fullFraction=o.allowPartialBends&&!o.operationTemplates ? .4 : .75;
    const seedLimit=Math.max(1,Math.floor(o.maxTransitions*fullFraction));
    // This historical screen proposes single-bend quantities. Joint strokes
    // use the general search, which synchronizes all their members.
    if(!o.operationTemplates&&o.allowPartialBends&&!model.bends.some(b=>b.strokeLeadId!=null)&&model.bends.some(b=>Math.abs(b.signedAngle)>Math.PI/2+1e-7)){
      const stats=diagnostics.fixedClosureScreen={configurations:0,poses:0,programs:0};
      const programs=probeFixedClosurePrograms({model,catalog,setups,options:o,support,initial,guard,finalKey,stats});
      for(const proposal of programs){
        let node={amounts:initial,stages:[],edges:[],cost:zero};
        for(const op of proposal){
          guard();const {bendId,target,config}=op,to={...node.amounts,[bendId]:target},id=`${bendId}:${node.amounts[bendId].toFixed(7)}>${target.toFixed(7)}`;
          const key=transitionKey(node.amounts,to,bendId,config,id);
          let checked=cache.get(key);
          if(!checked){
            if(diagnostics.statesExpanded>=o.maxStates||diagnostics.transitionsChecked>=o.maxTransitions)throw new Budget('fixed-program-budget');
            diagnostics.statesExpanded++;diagnostics.transitionsChecked++;
            emit({phase:'checking-operation',search:'fixed-stations-closure',bendId,operationId:id,depth:node.stages.length});
            await new Promise(resolve=>setTimeout(resolve,0));guard();
            checked=evaluateSequenceTransition({model,catalog,from:node.amounts,to,bendId,config,operationId:id,options:o,preparedSupport:support.get(String(bendId))},guard);
            cache.set(key,checked);if(!checked.feasible)reject(checked,node.stages);
          }else diagnostics.cacheHits++;
          if(!checked.feasible)break;
          const stage=checked.stage,edges=[...node.edges,{id,config:stage.config,flip:config.flip,stationShiftMm:config.stationYmm??0,
            coverage:checked.coverage,uncoveredLengthMm:checked.metrics.uncoveredLengthMm,coverageUnresolved:checked.metrics.coverageUnresolved}];
          node={amounts:to,stages:[...node.stages,stage],edges,cost:scoreToolSetupSequence(edges,scoreOptions).cost};
        }
        if(done(node.amounts)&&(!best||nodeCompare(node,best)<0)&&acceptComplete(node)){best=node;diagnostics.fixedClosureProgram=true;emit({phase:'candidate-found',depth:node.stages.length,cost:best.cost});}
        if(primaryMinimum()){diagnostics.stoppedAtPrimaryCostLowerBound=true;break;}
      }
    }
    const quota=fraction=>remainingPhaseQuota({transitions:diagnostics.transitionsChecked,states:diagnostics.statesExpanded,now:performance.now()},
      {transitions:o.maxTransitions,states:o.maxStates,until:started+o.timeBudgetMs},fraction);
    const fullPhaseBudget=o.relativePhaseBudgets?quota(fullFraction):{transitions:seedLimit,
      states:Math.max(1,Math.floor(o.maxStates*fullFraction)),until:started+o.timeBudgetMs*fullFraction};
    if(o.relativePhaseBudgets)diagnostics.relativePhaseBudgets=[{phase:'forward-full',startTransitions:diagnostics.transitionsChecked,
      startStates:diagnostics.statesExpanded,remainingMs:Math.max(0,started+o.timeBudgetMs-performance.now()),quota:fullPhaseBudget}];
    if(!primaryMinimum())await seed({amounts:initial,stages:[],edges:[],cost:zero},false,0,fullPhaseBudget,new Map());
    if(o.allowPartialBends&&!o.operationTemplates&&!primaryMinimum()){
      diagnostics.partialFallbackUsed=true;
      diagnostics.partialPrioritiesLearned=[...difficulty].filter(([id])=>!successfulFull.has(id)).map(([bendId,blockedDepth])=>({bendId,blockedDepth}));
      const phaseBudget=o.relativePhaseBudgets?quota(.75):{transitions:Math.max(seedLimit+1,Math.floor(o.maxTransitions*.85)),
        states:Math.max(2,Math.floor(o.maxStates*.85)),until:started+o.timeBudgetMs*.85},seen=new Map();
      if(o.relativePhaseBudgets)diagnostics.relativePhaseBudgets.push({phase:'forward-partial',startTransitions:diagnostics.transitionsChecked,
        startStates:diagnostics.statesExpanded,remainingMs:Math.max(0,started+o.timeBudgetMs-performance.now()),quota:phaseBudget});
      const prefixes=[...fallbackPrefixes.values()].filter(p=>!successfulFull.has(p.difficultBendId)).sort((a,b)=>b.stages.length-a.stages.length||compareCost(a.cost,b.cost)).slice(0,3);
      diagnostics.prebendInsertionPrefixes=prefixes.map(p=>({difficultBendId:p.difficultBendId,preservedOperationIds:p.stages.map(s=>s.id),blockedDepth:p.blockedDepth}));
      for(const prefix of [...prefixes,{amounts:initial,stages:[],edges:[],cost:zero}]){
        if(await seed(prefix,true,prefix.stages.length,phaseBudget,seen))break;
      }
    }
    for(const partial of [false,...(o.allowPartialBends&&!o.operationTemplates?[true]:[])]){
      if(primaryMinimum())break;
      diagnostics.partialFallbackUsed ||= partial;
      let frontier=[{amounts:initial,stages:[],edges:[],cost:zero}],localTransitions=0;
      const firstPhaseLimit=!partial&&o.allowPartialBends&&!o.operationTemplates?Math.max(1,Math.floor(o.maxTransitions*.55)):Infinity;
      for(let depth=0;frontier.length&&depth<(o.operationTemplates?.length??model.bends.length*(partial?2:1));depth++){
        const next=new Map();
        for(const node of frontier){
          guard();if(diagnostics.statesExpanded>=o.maxStates)throw new Budget('state-budget');diagnostics.statesExpanded++;
          for(const operation of operations(model,node,partial,o)){
            const list=[...configsAt(operation)];
            const last=node.stages.at(-1)?.config;if(last)list.sort((a,b)=>Number(toolSetupKey(b)===toolSetupKey(last))-Number(toolSetupKey(a)===toolSetupKey(last)));
            for(const cfg of list)for(const flip of[false,true]){
              guard();if(localTransitions>=firstPhaseLimit)break;
              const config={...cfg,flip,bedLengthMm:setups.bedLengthMm};
              const key=transitionKey(node.amounts,operation.to,operation.bend.id,config,operation.id);
              let checked=cache.get(key);
              if(checked)diagnostics.cacheHits++;
              else{
                if(diagnostics.transitionsChecked>=o.maxTransitions)throw new Budget('transition-budget');
                diagnostics.transitionsChecked++;localTransitions++;
                emit({phase:'checking-operation',bendId:operation.bend.id,operationId:operation.id,depth,partialFallback:partial});
                await new Promise(resolve=>setTimeout(resolve,0));guard();
                checked=evaluateSequenceTransition({model,catalog,from:node.amounts,to:operation.to,bendId:operation.bend.id,
                  config,operationId:operation.id,options:o,preparedSupport:support.get(String(operation.bend.id))},guard);
                cache.set(key,checked);if(!checked.feasible)reject(checked,node.stages);
              }
              if(!checked.feasible)continue;
              const stage={...checked.stage,id:operation.id},edge={id:stage.id,config:stage.config,flip,stationShiftMm:config.stationYmm??0,
                coverage:checked.coverage,uncoveredLengthMm:checked.metrics.uncoveredLengthMm,coverageUnresolved:checked.metrics.coverageUnresolved};
              const edges=[...node.edges,edge],cost=scoreToolSetupSequence(edges,scoreOptions).cost;
              const candidate={amounts:operation.to,stages:[...node.stages,stage],edges,cost};
              if(done(candidate.amounts)){if((!best||nodeCompare(candidate,best)<0)&&acceptComplete(candidate))best=candidate;continue;}
              const nk=JSON.stringify([amountKey(model,candidate.amounts),configKey(config),flip,
                ...(o.preserveSearchHistory?[candidate.stages.map(s=>[s.id,configKey(s.config),s.flip])]:[])]);
              if(!next.has(nk)||nodeCompare(candidate,next.get(nk))<0)next.set(nk,candidate);
            }
          }
        }
        const ordered=[...next.values()].sort(nodeCompare);diagnostics.beamPruned+=Math.max(0,ordered.length-o.beamWidth);
        // Preserve material-state diversity before filling duplicate states with
        // alternate tool setups; this avoids using the entire beam on one order.
        const chosen=[],seenAmounts=new Set();for(const node of ordered){const key=amountKey(model,node.amounts);
          if(!seenAmounts.has(key)){seenAmounts.add(key);chosen.push(node);if(chosen.length>=o.beamWidth)break;}}
        for(const node of ordered)if(chosen.length<o.beamWidth&&!chosen.includes(node))chosen.push(node);
        frontier=chosen;
        emit({phase:'search-layer',depth:depth+1,frontier:frontier.length,bestCompleteStages:best?.stages.length??0,partialFallback:partial});
        if(localTransitions>=firstPhaseLimit)break;
      }
    }
    diagnostics.searchComplete=!diagnostics.stoppedAtPrimaryCostLowerBound&&!diagnostics.phaseQuotaReached&&!diagnostics.seedPruned&&diagnostics.beamPruned===0&&!diagnostics.candidates.generatedByBend.some(b=>b.truncated);
  }catch(error){if(error instanceof Budget)budgetReason=error.reason;else return{...resultBase(),status:'ENGINE_ERROR',diagnostics:{...diagnostics,error:error.message}};}
  diagnostics.budgetReason=budgetReason;
  // Longest bends first on their bank, then shorter bends on setups that keep
  // that bank (no profile change). The free search can miss this route; it is
  // a short directed depth-first probe and every transition is fully checked.
  if(best&&!o.operationTemplates&&!o.signal?.aborted&&best.cost.fullChangeEvents+best.cost.partialChangeEvents>0&&
    [...setups.keptMixes.values()].some(list=>list.length)){
    const until=performance.now()+o.timeBudgetMs*.25,stats=diagnostics.keptBankProgram={transitions:0,result:'NOT_FOUND'};
    const order=[...model.bends].sort((a,b)=>fullSpan(b).width-fullSpan(a).width);
    const rankOf=(last,cfg)=>{if(!last)return 0;const k=toolChangeKind(last,cfg),v={none:0,segments:1,profile:10};return v[k.punch]+v[k.die];};
    const visit=async node=>{
      if(performance.now()>=until||stats.transitions>=120||o.signal?.aborted)return null;
      if(done(node.amounts))return node;
      const bend=order.find(b=>node.amounts[b.id]<1-EPS);if(!bend)return null;
      const op=operations(model,node,false,o).find(x=>String(x.bend.id)===String(bend.id)&&x.target===1);if(!op)return null;
      const last=node.stages.at(-1)?.config;
      const list=[...(setups.byBend.get(String(bend.id))??[]),...(setups.keptMixes.get(String(bend.id))??[])].filter(c=>rankOf(last,c)<=1).sort((a,b)=>rankOf(last,a)-rankOf(last,b)).slice(0,8);
      for(const cfg of list)for(const flip of[false,true]){
        const config={...cfg,flip,bedLengthMm:setups.bedLengthMm},key=transitionKey(node.amounts,op.to,bend.id,config,op.id);
        let checked=cache.get(key);
        if(!checked){stats.transitions++;await new Promise(resolve=>setTimeout(resolve,0));
          checked=evaluateSequenceTransition({model,catalog,from:node.amounts,to:op.to,bendId:bend.id,config,operationId:op.id,options:o,preparedSupport:support.get(String(bend.id))},()=>{});
          cache.set(key,checked);}
        if(!checked.feasible)continue;
        const stage={...checked.stage,id:op.id},edges=[...node.edges,{id:stage.id,config:stage.config,flip,stationShiftMm:config.stationYmm??0,
          coverage:checked.coverage,uncoveredLengthMm:checked.metrics.uncoveredLengthMm,coverageUnresolved:checked.metrics.coverageUnresolved}];
        const next={amounts:op.to,stages:[...node.stages,stage],edges,cost:scoreToolSetupSequence(edges,scoreOptions).cost};
        if(compareCost(next.cost,best.cost)>=0)continue;
        const found=await visit(next);if(found)return found;
      }
      return null;
    };
    try{const found=await visit({amounts:initial,stages:[],edges:[],cost:zero});
      if(found&&compareCost(found.cost,best.cost)<0&&acceptComplete(found)){best=found;stats.result='APPLIED';}}
    catch(error){stats.result='FAILED: '+error.message;}
  }
  if(best&&best.cost.toolChangeEvents>0&&!o.signal?.aborted){
      try{const reused=await reuseFinalSetup(best);if(reused){best=reused;diagnostics.finalSetupReuse='APPLIED';}
        else{
          // Advice only: verify whether one extra punch+die piece of a single
          // length would remove the change. The plan itself is not altered.
          const final=best.stages.at(-1).config;
          const lengths=[...new Set(final.segments.map(s=>s.lengthMm))].sort((a,b)=>a-b);
          for(const lengthMm of lengths){
            if(o.signal?.aborted)break;
            const extra=JSON.parse(JSON.stringify(catalog));
            for(const kind of['punch','die']){const t=extra.tools.find(x=>String(x.id)===String(final[kind+'Id']));
              const item=t?.inventory?.find(i=>Math.abs(i.lengthMm-lengthMm)<1e-5);if(item)item.count=(item.count??0)+1;else if(t)(t.inventory??=[]).push({lengthMm,count:1});}
            const trial=await reuseFinalSetup(best,extra);
            if(trial){const count=kind=>catalog.tools.find(x=>String(x.id)===String(final[kind+'Id']))?.inventory?.find(i=>Math.abs(i.lengthMm-lengthMm)<1e-5)?.count??0;
              diagnostics.finalSetupReuse='NEEDS_EXTRA_STOCK';
              diagnostics.extraStockAdvice={punchId:final.punchId,dieId:final.dieId,lengthMm,punchAvailable:count('punch'),dieAvailable:count('die'),
                currentChanges:best.cost.toolChangeEvents,verifiedChanges:trial.cost.toolChangeEvents,stationTravelMm:trial.cost.stationTravelMm};break;}
          }
        }}
    catch(error){diagnostics.finalSetupReuse='FAILED: '+error.message;}
  }
  // Compare with the unmodified ordinary search after all its lower-cost
  // reuse attempts. A failed probe therefore consumes none of its budget.
  if(family&&!o.signal?.aborted&&(!best||best.cost.toolChangeEvents>0)&&o.maxTransitions>=4){
    // Time caps are fractions of timeBudgetMs (ARM gets a 7.5x budget), so a
    // slower CPU checks the same transitions; transition caps bound the work.
    const time=performance.now(),limit=o.timeBudgetMs*.05,stats=diagnostics.groupedCompoundBanks={checks:0};
    const guardProbe=()=>{
      if(o.signal?.aborted)throw new Budget('cancelled');
      if(performance.now()-time>=limit)throw new Budget('probe-time-limit');
    };
    try{
      const grouped=await groupedRoute(stats,guardProbe,false);
      stats.result=grouped?'SELECTED':'NO_CLEAR_GROUPED_ROUTE';
      if(grouped){
        if(best){
          stats.ordinaryRoute={cost:best.cost,stages:best.stages.map(s=>({bendId:s.bendId,config:s.config,from:s.from,to:s.to}))};
          stats.uncoveredLengthDifferenceMm=grouped.cost.uncoveredLengthMm-best.cost.uncoveredLengthMm;
        }
        stats.selection='Both over-bed joint strokes on one mounted bank, then both perpendicular strokes on one shorter bank. Existing support limits apply; a route with fewer or lower-class changes is retained.';
        best=grouped;
      }
    }catch(error){stats.result=error instanceof Budget?error.reason:'PROBE_FAILED';if(!(error instanceof Budget))stats.error=error.message;}
    stats.elapsedMs=performance.now()-time;
  }
  if(!best)return{...resultBase(),status:budgetReason==='cancelled'?'CANCELLED':budgetReason?'SEARCH_BUDGET_EXHAUSTED':'NO_SEQUENCE_IN_SEARCHED_CANDIDATES'};
  const unresolved=best.stages.some(s=>!s.toolSupport.accepted);
  return{...resultBase(),status:unresolved?'DIAGNOSTIC_UNDERCOVERED':'CANDIDATE_SEQUENCE',stages:best.stages,cost:best.cost,costOrder:COST_ORDER,
    allStagesSampleClear:true,allStagesOperatorSupportPass:true,coverageUnresolved:unresolved,
    diagnostics:{...diagnostics,elapsedMs:performance.now()-started,
      optimality:'Lowest lexicographic cost among complete routes retained by this bounded search. No global manufacturing optimality is asserted.',
      pending:['Exterior transfer and reorientation between stages.','Physical tool-change motion for changed setups.','Machine frame/holders and actual travel limits.','Material forming, short-tool adequacy and springback.']}};
}
