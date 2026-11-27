/** Rank punch/die pairs by how closely they reproduce the CAD design (sheet-metal rule radius, thickness, angle). */
const DEG=180/Math.PI;
import {toolingSelectionDefaults} from './tooling-rules.mjs';
export {toolingSelectionDefaults};

const opening=die=>Number(die?.contact?.openingMm??die?.contact?.geometryOpeningMm??NaN);
const angle=tool=>Number(tool?.contact?.openingAngleDegrees??NaN);

/** Shortest flat distance from any bend centreline to the far edge of the panel on either side. */
export function shortestFlangeMm(model){
  const panels=new Map((model.panels||[]).map(p=>[p.id,p]));let best=Infinity;
  const extent=(id,b)=>{const fp=panels.get(id)?.flatPositions;if(!fp?.length)return Infinity;
    const o=b.origin,t=b.transverse;let mx=0;
    for(let i=0;i<fp.length;i+=3)mx=Math.max(mx,Math.abs((fp[i]-o[0])*t[0]+(fp[i+1]-o[1])*t[1]));return mx;};
  for(const b of model.bends||[]){if(!b.origin||!b.transverse)continue;
    best=Math.min(best,extent(b.childPanelId,b),extent(b.parentPanelId,b));}
  return best;
}

const inside=(tool,x,z)=>{const pts=tool?.vertices2D??tool?.profile;if(!pts?.length)return false;
  const loops=tool.loops??[pts.map((_,i)=>i)];let hit=false;
  for(const loop of loops)for(let i=0,j=loop.length-1;i<loop.length;j=i++){const a=pts[loop[i]],b=pts[loop[j]];
    if((a[1]>z)!==(b[1]>z)&&x<(b[0]-a[0])*(z-a[1])/(b[1]-a[1])+a[0])hit=!hit;}
  return hit;};

/** Section points of already formed return flanges next to each bend, in the press frame of that bend
 * (tip at origin, x across the die, z up; side +1/-1 is which side of the bend line they sit on). */
export function adjacentFlangeSections(model,samples=16){
  const panels=new Map((model.panels||[]).map(p=>[p.id,p])),bends=(model.bends||[]).filter(b=>b.origin&&b.transverse);
  const extent=(id,b)=>{const fp=panels.get(id)?.flatPositions;if(!fp?.length)return 0;let mx=0;
    for(let i=0;i<fp.length;i+=3)mx=Math.max(mx,Math.abs((fp[i]-b.origin[0])*b.transverse[0]+(fp[i+1]-b.origin[1])*b.transverse[1]));return mx;};
  return bends.map(b=>{const sections=[],a=Math.abs(b.signedAngle??0)/2;
    for(const n of bends){if(n===b)continue;
      const shared=[b.parentPanelId,b.childPanelId].find(p=>p!=null&&(p===n.parentPanelId||p===n.childPanelId));if(shared==null)continue;
      const along=(n.origin[0]-b.origin[0])*b.transverse[0]+(n.origin[1]-b.origin[1])*b.transverse[1],d=Math.abs(along);
      const far=n.parentPanelId===shared?n.childPanelId:n.parentPanelId,length=extent(far,n),phi=Math.abs(n.signedAngle??0);
      if(!(d>0&&length>0&&phi>0))continue;
      const same=Math.sign(n.signedAngle)===Math.sign(b.signedAngle),dir=same?a+phi:a-phi,points=[];
      for(let i=1;i<=samples;i++){const s=length*i/samples;points.push([d*Math.cos(a)+s*Math.cos(dir),d*Math.sin(a)+s*Math.sin(dir)]);}
      sections.push({bendId:b.id,neighbourId:n.id,side:Math.sign(along)||1,points});}
    return{bendId:b.id,sections};});
}

/** Count of formed-flange sample points inside a tool profile, taking the better of the two part
 * orientations per bend (front/back flip swaps sides). An estimate for ranking only. */
export function flangeClearancePenalty(tool,sections){
  let total=0;
  for(const bend of sections){const count=mirror=>bend.sections.reduce((n,s)=>n+s.points.filter(([x,z])=>inside(tool,x*s.side*mirror,z)).length,0);
    total+=Math.min(count(1),count(-1));}
  return total;
}

/** Pure ranking; never runs collision planning. */
export function rankToolingCandidates(model,catalog,settings={}){
  const s={...toolingSelectionDefaults,...settings},t=Number(model.thicknessMm);
  const radii=model.bends.map(b=>Number(b.insideRadius)).filter(Number.isFinite);
  const designRadius=radii.length?Math.min(...radii):t;
  const includedAngle=Math.min(...model.bends.map(b=>180-Math.abs(b.signedAngle)*DEG));
  const tolerance=Math.max(designRadius*s.radiusTolerance,t*s.radiusToleranceThickness);
  const ready=catalog.tools.filter(x=>x.status==='ready');
  const punches=ready.filter(x=>x.kind==='punch'),dies=ready.filter(x=>x.kind==='die');
  const punchOk=p=>{const r=[],tip=p.contact?.tipRadiusMm,a=angle(p);
    if(Number.isFinite(a)&&a>includedAngle+1e-6)r.push('PUNCH_ANGLE_TOO_OPEN');
    if(Number.isFinite(tip)&&radii.length&&tip>designRadius+1e-6)r.push('PUNCH_TIP_RADIUS_EXCEEDS_DESIGN');return r;};
  const candidates=[],flange=shortestFlangeMm(model),sections=s.flangeClearanceRanking?adjacentFlangeSections(model):[],clearance=new Map();
  const penalty=tool=>{if(!clearance.has(tool.id))clearance.set(tool.id,flangeClearancePenalty(tool,sections));return clearance.get(tool.id);};
  for(const die of dies){
    const v=opening(die),a=angle(die),dieReasons=[];
    if(v*s.minFlangeToOpeningRatio>flange)dieReasons.push('DIE_MIN_FLANGE_EXCEEDS_PART');
    if(!Number.isFinite(v))dieReasons.push('DIE_OPENING_UNKNOWN');
    if(v<s.minOpeningThickness*t)dieReasons.push('DIE_OPENING_BELOW_MIN_THICKNESS_RATIO');
    if(v>s.maxOpeningThickness*t)dieReasons.push('DIE_OPENING_ABOVE_MAX_THICKNESS_RATIO');
    if(Number.isFinite(a)&&a>includedAngle+1e-6)dieReasons.push('DIE_ANGLE_TOO_OPEN');
    const [lo,hi]=s.preferredOpeningThickness??[0,Infinity],outsideRule=v<lo*t||v>hi*t;
    const predicted=v*s.radiusToOpeningRatio,radiusError=predicted-designRadius;
    if(!(Math.abs(radiusError)<=tolerance))dieReasons.push('PREDICTED_RADIUS_OUTSIDE_TOLERANCE');
    const tonnage=s.tensileMpa>0&&v>0?1.42*s.tensileMpa*t*t/v:null;
    if(tonnage!=null&&s.maxTonnagePerMeterKn>0&&tonnage>s.maxTonnagePerMeterKn)dieReasons.push('TONNAGE_EXCEEDS_LIMIT');
    for(const punch of punches){
      const tip=punch.contact?.tipRadiusMm;
      const reasons=[...dieReasons,...punchOk(punch)];
      const preferred=String(punch.id)===String(s.preferredPunchId??'')&&String(die.id)===String(s.preferredDieId??'');
      candidates.push({punchId:punch.id,dieId:die.id,punchName:punch.name,dieName:die.name,openingMm:v,
        dieAngleDegrees:Number.isFinite(a)?a:null,punchAngleDegrees:Number.isFinite(angle(punch))?angle(punch):null,
        punchTipRadiusMm:Number.isFinite(tip)?tip:null,designInsideRadiusMm:designRadius,
        predictedInsideRadiusMm:Math.max(predicted,Number.isFinite(tip)?tip:0),radiusErrorMm:radiusError,radiusToleranceMm:tolerance,
        minFlangeMm:v*s.minFlangeToOpeningRatio,tonnagePerMeterKn:tonnage,preferred,viable:reasons.length===0,reasons,
        openingRatioOutsideRule:outsideRule,
        flangeClearancePenalty:penalty(punch)+penalty(die)});
    }
  }
  // A short flange forces a narrow die: when only the radius/opening rules block it, accept it as a compromise.
  const soft=new Set(['DIE_OPENING_BELOW_MIN_THICKNESS_RATIO','PREDICTED_RADIUS_OUTSIDE_TOLERANCE']);
  if(!candidates.some(c=>c.viable))for(const c of candidates)
    if((s.allowFlangeFallback===true||String(c.dieId)===String(s.preferredDieId))&&
      c.reasons.length&&c.reasons.every(r=>soft.has(r))&&c.radiusErrorMm<0){c.viable=true;c.compromise=c.reasons;c.reasons=[];}
  // Flange clearance only orders candidates; it never makes one non-viable.
  candidates.sort((a,b)=>(b.viable-a.viable)||(b.preferred-a.preferred)||(a.openingRatioOutsideRule-b.openingRatioOutsideRule)||
    (s.flangeClearanceRanking?a.flangeClearancePenalty-b.flangeClearancePenalty:0)||
    Math.abs(a.radiusErrorMm)-Math.abs(b.radiusErrorMm)||(a.punchTipRadiusMm??0)-(b.punchTipRadiusMm??0));
  // Rule-of-8 die (V closest to 8 t that fits the flange and angle): a recommendation only, the press preference still wins.
  const fits=c=>!c.reasons.some(r=>r==='DIE_MIN_FLANGE_EXCEEDS_PART'||r==='DIE_ANGLE_TOO_OPEN'||r==='DIE_OPENING_UNKNOWN');
  const best=candidates.filter(fits).sort((a,b)=>Math.abs(a.openingMm-8*t)-Math.abs(b.openingMm-8*t))[0];
  const [lo,hi]=s.preferredOpeningThickness??[6,10];
  const ruleOf8=best&&best.openingMm>=lo*t&&best.openingMm<=hi*t?{dieId:best.dieId,dieName:best.dieName,openingMm:best.openingMm,openingThicknessRatio:best.openingMm/t,
    bandThickness:[lo,hi]}:null;
  return{ruleOf8,designInsideRadiusMm:designRadius,thicknessMm:t,shortestFlangeMm:Number.isFinite(flange)?flange:null,includedAngleDegrees:includedAngle,radiusToleranceMm:tolerance,
    settings:s,candidates};
}

export {ruleAllowedDieIds,ruleDieDecision,resolveToolingRules} from './tooling-rules.mjs';

/** One plan per die keeps alternatives physically different; the engine still chooses the punch among viable ones. */
export function selectPlanGroups(ranking,maxPlans){
  const groups=new Map();
  for(const c of ranking.candidates.filter(c=>c.viable)){
    const key=String(c.dieId);if(!groups.has(key))groups.set(key,{dieId:c.dieId,punchIds:[],preferred:false,lead:c});
    const g=groups.get(key);g.punchIds.push(c.punchId);g.preferred||=c.preferred;
  }
  return[...groups.values()].sort((a,b)=>(b.preferred-a.preferred))
    .slice(0,Math.max(1,maxPlans));
}
