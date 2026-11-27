// Bounded stock/layout search and sampled translational repositioning.
// No fabricated stock lengths, machine commands, material forming or CCD claims.
const POSITION_KEYS=['x','y','z','centerX','centerY','centerZ'];
const finite=(x,label)=>{if(!Number.isFinite(x))throw new TypeError(`${label} must be finite`);return x;};
const near=(a,b,t=1e-8)=>Math.abs(a-b)<=t;
const clone=p=>({...p});
function distance(a,b,keys) {return Math.hypot(...keys.map(k=>(b[k]??0)-(a[k]??0)));}
function interpolation(a,b,t,keys) {const p={...a};for(const k of keys)p[k]=(a[k]??0)+t*((b[k]??0)-(a[k]??0));return p;}
function collisionResult(raw,clearanceMm) {
  if(!raw||!['CLEAR','SAMPLE_CLEAR','INTERFERENCE'].includes(raw.status)) return {status:'INTERFERENCE',reason:'INVALID_COLLISION_RESULT',raw};
  const hasDistance=typeof raw.distanceMm==='number'&&!Number.isNaN(raw.distanceMm);
  const hit=raw.status==='INTERFERENCE'||(hasDistance&&raw.distanceMm<=clearanceMm);
  return {...raw,status:hit?'INTERFERENCE':'CLEAR'};
}

/**
 * A segment translates between poses; no orientation change is accepted.
 * evaluatePose is synchronous and returns {status:'CLEAR'|'INTERFERENCE',distanceMm?}.
 * Positive retraction uses +axis; use retractSign=-1 for downward die withdrawal.
 * Returned samples are observations, not a universal collision-free certificate.
 */
export function planSegmentMove({from,to,evaluatePose,retractAxis='z',retractSign=1,
  retractDistancesMm=[10,25,50,100],maxStepMm=2,minStepMm=.05,maxSamples=20000,
  clearanceMm=.2,underLoad=false,distanceIsConservative=false}={}) {
  if(!from||!to||typeof evaluatePose!=='function') throw new TypeError('from, to and evaluatePose are required');
  if(typeof retractAxis!=='string'||!retractAxis.length) throw new RangeError('retractAxis must name a translation coordinate');
  if(![1,-1].includes(retractSign))throw new RangeError('retractSign must be +1 or -1');
  if(!(maxStepMm>0)||!(minStepMm>0)||minStepMm>maxStepMm)throw new RangeError('Invalid sampling step');
  if(!Number.isInteger(maxSamples)||maxSamples<2)throw new RangeError('maxSamples must be at least 2');
  if(!Number.isFinite(clearanceMm)||clearanceMm<0)throw new RangeError('Invalid clearance');
  // The caller may name a relative translation coordinate (for example offsetZ).
  // Other changing properties remain rejected so angles are never interpolated as millimetres.
  const keys=[...new Set([...POSITION_KEYS.filter(k=>k in from||k in to),retractAxis])];
  for(const k of keys) {finite(from[k]??0,`from.${k}`);finite(to[k]??0,`to.${k}`);}
  const changes=Object.keys({...from,...to}).filter(k=>!keys.includes(k)&&JSON.stringify(from[k])!==JSON.stringify(to[k]));
  if(changes.length)throw new RangeError(`Only translation is supported; changing ${changes.join(', ')}`);
  const moved=distance(from,to,keys)>1e-10;
  const limits={method:'ADAPTIVE_SAMPLING',continuousCollisionDetection:false,maxStepMm,minStepMm,
    distanceBoundUsed:distanceIsConservative,assumptions:'Linear translation of one rigid body against fixed obstacles; collision callback owns all geometry and allowed contacts.'};
  if(underLoad&&moved)return {status:'BLOCKED',reason:'SEGMENT_UNDER_LOAD',keyframes:[],samples:[],attempts:[],limits};
  const offsets=[...new Set(retractDistancesMm.map(d=>finite(Number(d),'retraction')))];
  if(offsets.some(d=>d<0))throw new RangeError('Retraction distances must be nonnegative');
  const lateral=keys.some(k=>k!==retractAxis&&!near(from[k]??0,to[k]??0));
  const routes=lateral?offsets.map(offset=>{
    const retractValue=(retractSign>0?Math.max(from[retractAxis]??0,to[retractAxis]??0):Math.min(from[retractAxis]??0,to[retractAxis]??0))+retractSign*offset;
    return [{pose:clone(from),phase:'START'},{pose:{...from,[retractAxis]:retractValue},phase:'RETRACT'},
      {pose:{...to,[retractAxis]:retractValue},phase:'SLIDE'},{pose:clone(to),phase:'APPROACH'}];
  }):[[{pose:clone(from),phase:'START'},{pose:clone(to),phase:'APPROACH'}]];
  const cache=new Map(),attempts=[];let evaluations=0;
  function evaluate(pose,phase,samples) {
    const key=keys.map(k=>Number(pose[k]??0).toPrecision(15)).join('|');
    let result=cache.get(key);
    if(!result) {
      if(evaluations>=maxSamples)throw new Error('SAMPLE_BUDGET_EXHAUSTED');
      result=collisionResult(evaluatePose(clone(pose)),clearanceMm);cache.set(key,result);evaluations++;
    }
    const sample={pose:clone(pose),phase,result};samples.push(sample);return sample;
  }
  for(let routeIndex=0;routeIndex<routes.length;routeIndex++) {
    const keyframes=routes[routeIndex].filter((k,i,list)=>i===0||distance(k.pose,list[i-1].pose,keys)>1e-10);
    const samples=[];let blocked=null,boundedIntervals=0,maxObservedGapMm=0;
    function inspect(a,b,phase,sa,sb,depth=0) {
      if(sa.result.status==='INTERFERENCE')return sa;
      if(sb.result.status==='INTERFERENCE')return sb;
      const span=distance(a,b,keys),da=sa.result.distanceMm,db=sb.result.distanceMm;
      // Hausdorff displacement of a translated rigid body bounds separation change.
      // This optimization is opt-in because arbitrary callbacks may not report a true distance.
      if(distanceIsConservative&&Number.isFinite(da)&&Number.isFinite(db)&&Math.min(da,db)-span/2>clearanceMm) {
        boundedIntervals++;maxObservedGapMm=Math.max(maxObservedGapMm,span);return null;
      }
      const nearDistance=Math.min(Number.isFinite(da)?da:Infinity,Number.isFinite(db)?db:Infinity)-clearanceMm;
      const target=Math.max(minStepMm,Math.min(maxStepMm,nearDistance>0?nearDistance*.5:minStepMm));
      if(span<=target||depth>=40){maxObservedGapMm=Math.max(maxObservedGapMm,span);return null;}
      const mid=interpolation(a,b,.5,keys),sm=evaluate(mid,phase,samples);
      if(sm.result.status==='INTERFERENCE')return sm;
      return inspect(a,mid,phase,sa,sm,depth+1)||inspect(mid,b,phase,sm,sb,depth+1);
    }
    try {
      let sa=evaluate(keyframes[0].pose,'START',samples);
      if(sa.result.status==='INTERFERENCE')blocked=sa;
      for(let k=1;!blocked&&k<keyframes.length;k++) {
        const sb=evaluate(keyframes[k].pose,keyframes[k].phase,samples);
        blocked=inspect(keyframes[k-1].pose,keyframes[k].pose,keyframes[k].phase,sa,sb);sa=sb;
      }
    } catch(error) {
      if(error.message!=='SAMPLE_BUDGET_EXHAUSTED')throw error;
      return {status:'BLOCKED',reason:'SEARCH_BUDGET_EXHAUSTED',keyframes:[],samples,attempts,evaluations,limits};
    }
    const attempt={routeIndex,retractDistanceMm:lateral?offsets[routeIndex]:0,status:blocked?'INTERFERENCE':'SAMPLE_CLEAR',
      sampleCount:samples.length,blockedSample:blocked,boundedIntervals,maxObservedGapMm};
    attempts.push(attempt);
    if(!blocked)return {status:'SAMPLE_CLEAR',keyframes,samples,attempts,evaluations,
      totalTravelMm:keyframes.reduce((sum,k,i)=>sum+(i?distance(k.pose,keyframes[i-1].pose,keys):0),0),
      limits:{...limits,maxObservedGapMm,boundedIntervals}};
    // All candidate paths share endpoints. No retraction can cure an invalid endpoint.
    if(blocked.phase==='START'||(distance(blocked.pose,to,keys)<=1e-10))
      return {status:'BLOCKED',reason:blocked.phase==='START'?'START_INTERFERENCE':'TARGET_INTERFERENCE',keyframes:[],samples,attempts,evaluations,limits};
  }
  return {status:'BLOCKED',reason:'NO_SAMPLED_ROUTE',keyframes:[],samples:[],attempts,evaluations,limits};
}

function interval(value) {
  const a=Array.isArray(value)?value[0]:(value.minY??value.startY??value.start);
  const b=Array.isArray(value)?value[1]:(value.maxY??value.endY??value.end);
  finite(a,'interval minimum');finite(b,'interval maximum');
  if(b<a)throw new RangeError('Reversed material interval');
  return {minY:a,maxY:b,kind:value.kind};
}
function merged(values,tol=1e-9) {
  const result=[];
  for(const v of values.slice().sort((a,b)=>a.minY-b.minY)) {
    const last=result.at(-1);
    if(last&&v.minY<=last.maxY+tol)last.maxY=Math.max(last.maxY,v.maxY);
    else result.push({minY:v.minY,maxY:v.maxY});
  }
  return result;
}
function covered(placements,requirements,tolerance) {
  for(const req of requirements) {
    const union=merged(placements.filter(p=>p.kind===req.kind).map(p=>({minY:p.centerY-p.lengthMm/2,maxY:p.centerY+p.lengthMm/2})),tolerance);
    if(!union.some(u=>u.minY<=req.minY+tolerance&&u.maxY>=req.maxY-tolerance))return false;
  }
  return true;
}
function placementFits(candidate,placed,minGapMm) {
  const lo=candidate.centerY-candidate.lengthMm/2,hi=candidate.centerY+candidate.lengthMm/2;
  return placed.every(p=>p.kind!==candidate.kind||hi+minGapMm<=p.centerY-p.lengthMm/2+1e-8||lo>=p.centerY+p.lengthMm/2+minGapMm-1e-8);
}

/**
 * Search a finite set of stock subsets and Y translations. Segments remain their actual lengths.
 * requiredIntervals: [[minY,maxY],...] applies to every stock kind, or objects with kind.
 * evaluateLayout(placements) returns a collision result; omitted => NOT_EVALUATED.
 * Unselected stock must remain parked/removed; this function does not plan their motion.
 */
export function solveSegmentLayout({segments,bedMinY,bedMaxY,requiredIntervals,minGapMm=0,
  offsetStepMm=10,coverageToleranceMm=1e-7,evaluateLayout,collides,maxCandidates=30000,
  maxCentersPerSegment=160,clearanceMm=.2}={}) {
  if(!Array.isArray(segments)||!segments.length)throw new TypeError('Stock segments are required');
  finite(bedMinY,'bedMinY');finite(bedMaxY,'bedMaxY');
  if(bedMaxY<=bedMinY||!(offsetStepMm>0)||minGapMm<0||coverageToleranceMm<0)throw new RangeError('Invalid bed, step or gap');
  if(!Number.isInteger(maxCandidates)||maxCandidates<1||!Number.isInteger(maxCentersPerSegment)||maxCentersPerSegment<1)throw new RangeError('Invalid search budget');
  const ids=new Set(),kinds=[...new Set(segments.map(s=>s.kind))];
  for(const s of segments) {
    if(!s.id||ids.has(s.id))throw new RangeError('Stock IDs must be unique and nonempty');ids.add(s.id);
    if(!['punch','die'].includes(s.kind)||!Number.isFinite(s.lengthMm)||s.lengthMm<=0)throw new RangeError('Invalid stock kind/length');
    if(s.centerY!==undefined)finite(s.centerY,'centerY');
  }
  const requirements=(requiredIntervals??[]).map(interval).flatMap(r=>r.kind?[r]:kinds.map(kind=>({...r,kind})));
  if(!requirements.length)throw new RangeError('Required material intervals must not be empty');
  if(requirements.some(r=>!kinds.includes(r.kind)||r.minY<bedMinY-coverageToleranceMm||r.maxY>bedMaxY+coverageToleranceMm))
    return {status:'BLOCKED',reason:'REQUIRED_COVERAGE_OUTSIDE_STOCK_OR_BED',placements:[],search:{complete:true,evaluated:0}};
  // Required segments first; remaining stock largest first reduces impossible branches.
  const stock=segments.map(s=>({...s})).sort((a,b)=>Number(!!b.required)-Number(!!a.required)||b.lengthMm-a.lengthMm);
  let centerTruncated=false;
  const centers=stock.map(s=>{
    const low=bedMinY+s.lengthMm/2,high=bedMaxY-s.lengthMm/2;
    if(low>high)return [];
    if(s.fixed)return s.centerY>=low&&s.centerY<=high?[s.centerY]:[];
    const r=requirements.filter(r=>r.kind===s.kind);
    const candidates=[s.centerY,...r.flatMap(v=>[v.minY+s.lengthMm/2,v.maxY-s.lengthMm/2,(v.minY+v.maxY)/2]),low,high];
    const gridCount=Math.floor((high-low)/offsetStepMm)+1;
    for(let i=0;i<Math.min(gridCount,maxCentersPerSegment*4);i++)candidates.push(low+i*offsetStepMm);
    if(gridCount>maxCentersPerSegment*4)centerTruncated=true;
    const unique=[...new Set(candidates.filter(x=>Number.isFinite(x)&&x>=low-1e-8&&x<=high+1e-8).map(x=>Math.round(x*1e8)/1e8))];
    const origin=s.centerY??((r[0]?.minY??0)+(r[0]?.maxY??0))/2;
    unique.sort((a,b)=>Math.abs(a-origin)-Math.abs(b-origin));
    if(unique.length>maxCentersPerSegment)centerTruncated=true;
    return unique.slice(0,maxCentersPerSegment);
  });
  let visited=0,evaluated=0,coverageRejected=0,collisionRejected=0,budgetExhausted=false,best=null;
  const score=placed=>placed.reduce((sum,p)=>sum+Math.abs(p.centerY-(p.originalCenterY??p.centerY)),0)+placed.length*.001;
  function search(index,placed) {
    if(visited++>=maxCandidates){budgetExhausted=true;return;}
    if(index===stock.length) {
      if(!covered(placed,requirements,coverageToleranceMm)){coverageRejected++;return;}
      evaluated++;
      const clean=placed.map(({originalCenterY,...p})=>p);
      const raw=evaluateLayout?evaluateLayout(clean):collides?{status:collides(clean)?'INTERFERENCE':'CLEAR'}:null;
      const collision=raw?collisionResult(raw,clearanceMm):null;
      if(collision?.status==='INTERFERENCE'){collisionRejected++;return;}
      const cost=score(placed);
      if(!best||cost<best.cost)best={placements:clean,cost,collisionStatus:collision?'SAMPLE_CLEAR':'NOT_EVALUATED',collision};
      return;
    }
    const segment=stock[index];
    // Lower bound: remaining physical stock must be long enough for required material.
    for(const kind of kinds) {
      const required=merged(requirements.filter(r=>r.kind===kind)).reduce((n,r)=>n+r.maxY-r.minY,0);
      const available=placed.filter(s=>s.kind===kind).reduce((n,s)=>n+s.lengthMm,0)+stock.slice(index).filter(s=>s.kind===kind).reduce((n,s)=>n+s.lengthMm,0);
      if(available+coverageToleranceMm<required)return;
    }
    for(const centerY of centers[index]) {
      const candidate={...segment,centerY,originalCenterY:segment.centerY};
      if(!placementFits(candidate,placed,minGapMm))continue;
      if(best&&score([...placed,candidate])>best.cost)continue;
      search(index+1,[...placed,candidate]);if(budgetExhausted)break;
    }
    if(!segment.required&&!segment.fixed&&!budgetExhausted)search(index+1,placed);
  }
  search(0,[]);
  const stats={visited,evaluated,coverageRejected,collisionRejected,budgetExhausted,centerTruncated,
    complete:!budgetExhausted&&!centerTruncated,candidateGridStepMm:offsetStepMm,
    scope:'Finite candidate stock subsets and translations; not an exhaustive continuous/global optimizer.'};
  return best?{status:'LAYOUT_FOUND',...best,unusedSegmentIds:segments.filter(s=>!best.placements.some(p=>p.id===s.id)).map(s=>s.id),search:stats,
    motionPlanned:false}:{status:'BLOCKED',reason:budgetExhausted?'SEARCH_BUDGET_EXHAUSTED':'NO_CANDIDATE_LAYOUT',placements:[],search:stats,motionPlanned:false};
}
