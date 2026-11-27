import {strokeMaterialIntervals} from './machine.mjs';
import {strokeAmounts,strokeSearchAllowed} from './sequence-stroke.mjs';
import {formingSegmentSeated,toolChangeKind,toolSetupKey} from './tool-setup-search.mjs';

const EPS=1e-6;
const dot=(a,b)=>a.reduce((s,x,i)=>s+x*b[i],0);
const width=intervals=>intervals.at(-1)[1]-intervals[0][0];

// Two opposed compound sides share the same still-flat carrier. The other
// two sides run perpendicular to them. Their stock can fit inside the bed.
export function longCompoundFamily(model,bed,options={}){
  if(model.bends.length!==6||!(bed>0)||options.operationTemplates||options.setups||options.initialSegments||
      options.initialSetup||options.fixedStations!=null||options.allowPartialCoverage)return null;
  const roots=model.bends.filter(b=>b.strokeLeadId==null);
  if(roots.length!==4||roots.some(b=>b.parentPanelId!==model.rootPanelId))return null;
  const longs=roots.filter(b=>b.strokeFollowerIds?.length===1),shorts=roots.filter(b=>!b.strokeFollowerIds?.length);
  if(longs.length!==2||shorts.length!==2)return null;
  const carrier=shorts.find(s=>longs.every(b=>b.strokeRequiresFlatBendIds?.length===1&&b.strokeRequiresFlatBendIds[0]===s.id));
  if(!carrier||!longs.every(b=>model.bends.some(f=>f.id===b.strokeFollowerIds[0]&&f.strokeLeadId===b.id&&f.parentPanelId===carrier.childPanelId)))return null;
  if(Math.abs(dot(longs[0].axis,longs[1].axis))<1-EPS||Math.abs(dot(shorts[0].axis,shorts[1].axis))<1-EPS||
      Math.abs(dot(longs[0].axis,shorts[0].axis))>EPS)return null;
  if(dot(longs[0].transverse,longs[1].transverse)>-1+EPS||dot(shorts[0].transverse,shorts[1].transverse)>-1+EPS)return null;
  const widths=new Map();
  for(const b of roots){
    const material=strokeMaterialIntervals(b),long=longs.includes(b);
    if(!material.exact||material.intervals.length!==(long?2:1))return null;
    const span=width(material.intervals);
    if(long?span<=bed+EPS:span>=bed-EPS)return null;
    widths.set(b.id,span);
  }
  const position=(a,b)=>{for(let i=0;i<3;i++){const d=a.origin[i]-b.origin[i];if(Math.abs(d)>EPS)return d;}return 0;};
  return {longs:longs.sort(position),shorts:shorts.sort(position),widths,bed};
}

// A single unchanged bank would have to span both outer material ends and
// also fit the final forming pose. Include the retained stock-reuse layouts.
export function spanCapableFinalSetups(family,setups,maxUnsupportedEndMm,configKey){
  const requiredSpan=Math.max(...family.widths.values())-2*maxUnsupportedEndMm;
  if(!(requiredSpan>setups.bedLengthMm+EPS))return null;
  const byBend=new Map();
  let profile=null;
  for(const [id,base] of setups.byBend){
    const seen=new Set(),wide=[];
    for(const config of [...base,...(setups.keptMixes.get(id)??[])]){
      const key=configKey(config);
      if(seen.has(key))continue;
      seen.add(key);
      if(config.mirror || config.activeBankId!=null || config.segments.some(s=>
        s.mirror || s.bankId!=null || (s.punchId && s.punchId!==config.punchId)))return null;
      const pair=JSON.stringify([config.punchId,config.dieId]);
      if(profile!==null && profile!==pair)return null;
      profile=pair;
      if(['punch','die'].every(kind=>{
        const segments=config.segments.filter(s=>s.kind===kind);
        const lo=Math.min(...segments.map(s=>s.centerY-s.lengthMm/2));
        const hi=Math.max(...segments.map(s=>s.centerY+s.lengthMm/2));
        return hi-lo>=requiredSpan-EPS;
      }))wide.push(config);
    }
    byBend.set(id,wide);
  }
  return {byBend,requiredSpan};
}

// Exactly one contiguous stack of stock for both forming profiles. Include
// mount positions so two equal stock lists mounted differently do not match.
function bank(config,bed,long){
  if(config.activeBankId!=null||config.segments.some(s=>s.bankId!=null||s.mirror||(s.offsetX??0)!==0||(s.offsetZ??0)!==0))return null;
  const lo=Math.min(...config.segments.map(s=>s.centerY-s.lengthMm/2));
  const hi=Math.max(...config.segments.map(s=>s.centerY+s.lengthMm/2));
  const origin=(lo+hi)/2;
  // Local stock generation centres the mounted bank on each bend. Place the
  // actual stack once, at the bed centre, and move only the part between sides.
  config={...config,segments:config.segments.map(s=>({...s,centerY:s.centerY-origin})),
    stationYmm:(config.stationYmm??0)-origin};
  const layout=[],widths=[];
  for(const kind of ['punch','die']){
    const segments=config.segments.filter(s=>s.kind===kind).slice().sort((a,b)=>a.centerY-b.centerY);
    if(!segments.length)return null;
    const spans=segments.map(s=>[s.centerY-s.lengthMm/2,s.centerY+s.lengthMm/2]);
    if(spans.some((s,i)=>i&&Math.abs(s[0]-spans[i-1][1])>EPS))return null;
    const lo=spans[0][0],hi=spans.at(-1)[1];
    if(long?(lo>=-bed/2-EPS||hi<=bed/2+EPS):lo< -bed/2-EPS||hi>bed/2+EPS)return null;
    if(!segments.every(s=>formingSegmentSeated(s,bed)))return null;
    widths.push(hi-lo);
    layout.push(segments.map(s=>[s.punchId??null,s.lengthMm,Math.round(s.centerY/EPS)]));
  }
  if(Math.abs(widths[0]-widths[1])>EPS)return null;
  return {key:JSON.stringify([toolSetupKey(config),layout]),config,width:widths[0],count:config.segments.length};
}

function commonBanks(bends,family,byBend,long){
  const candidates=bends.map(b=>{
    const map=new Map();
    for(const c of byBend.get(String(b.id))??[]){
      const value=bank(c,family.bed,long);
      if(!value||map.has(value.key))continue;
      map.set(value.key,value);
    }
    return map;
  });
  const target=Math.max(...bends.map(b=>family.widths.get(b.id)));
  return [...candidates[0]].filter(([key])=>candidates[1].has(key)).map(([key,item])=>({
    ...item,configs:new Map([[bends[0].id,item.config],[bends[1].id,candidates[1].get(key).config]])
  })).sort((a,b)=>Number(a.width+EPS<target)-Number(b.width+EPS<target)||
    Math.abs(a.width-target)-Math.abs(b.width-target)||a.count-b.count);
}

export function compareChangeClass(a,b){
  for(const k of ['toolChangeEvents','fullChangeEvents','partialChangeEvents','segmentEditEvents']){
    if(a[k]!==b[k])return a[k]-b[k];
  }
  return 0;
}

export async function probeLongCompound({model,family,setups,initial,check,complete,guard,warm}){
  const longs=commonBanks(family.longs,family,setups.byBend,true),shorts=commonBanks(family.shorts,family,setups.byBend,false);
  if(!longs.length||!shorts.length)return null;
  const trials=new Map();
  const orders=pair=>[pair,[pair[1],pair[0]]];
  async function visit(ops,index,from,stages){
    guard();
    if(index===ops.length)return complete(stages);
    const [bend,bank]=ops[index];
    if(!strokeSearchAllowed(model,bend,from))return null;
    const to=strokeAmounts(bend,{...from,[bend.id]:1});
    const candidate=bank.configs.get(bend.id);
    for(const flip of [false,true]){
      const config={...candidate,flip,bedLengthMm:family.bed};
      const key=JSON.stringify([Object.values(from),bend.id,bank.key,config.stationYmm??0,flip]);
      if(!trials.has(key)){guard();trials.set(key,await check({bend,from,to,config}));}
      const stage=trials.get(key);
      if(!stage)continue;
      const result=await visit(ops,index+1,to,[...stages,stage]);
      if(result)return result;
    }
    return null;
  }
  // A different order may clear where its opposite does not. Prefer keeping the
  // punch and die profiles; the only edit is the switch to the shorter stack.
  const target=Math.max(...family.shorts.map(b=>family.widths.get(b.id)));
  for(const a of longs){
    const compatible=shorts.filter(b=>{
      const change=toolChangeKind(a.config,b.config);
      return change.punch==='segments'&&change.die==='segments';
    });
    const nearestFull=compatible.find(b=>b.width+EPS>=target),nearestEnds=compatible.find(b=>b.width+EPS<target);
    const banks=[...new Set([nearestFull,nearestEnds,...compatible].filter(Boolean))];
    for(const longOrder of orders(family.longs))for(const shortOrder of orders(family.shorts))for(const b of banks){
      const ops=[...longOrder.map(bend=>[bend,a]),...shortOrder.map(bend=>[bend,b])];
      if(warm){
        const batch=[];let from=initial;
        for(const [bend,bank] of ops){
          if(!strokeSearchAllowed(model,bend,from))break;
          const to=strokeAmounts(bend,{...from,[bend.id]:1}),candidate=bank.configs.get(bend.id);
          for(const flip of [false,true])batch.push({bend,from,to,config:{...candidate,flip,bedLengthMm:family.bed}});
          from=to;
        }
        if(await warm(batch)===false)continue;
      }
      const route=await visit(ops,0,initial,[]);if(route)return route;
    }
  }
  return null;
}
