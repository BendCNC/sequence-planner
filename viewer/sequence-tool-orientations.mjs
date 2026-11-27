/** Additional mounting directions of existing prismatic stock, opt-in only.
 * Every original candidate stays first and keeps its object identity. Flipping
 * the section in X is the same solid as rotating uniform stock 180 degrees
 * around Z. Holder/clamp geometry and mounting permission are not modeled;
 * geometric acceptance is not a certificate that the mounting is permitted.
 */
export const TOOL_ORIENTATION_SCOPE='Original candidates preserved; added X-reversed punch, die and both profiles from the same inventory. Offsets, support and collision rules unchanged. Holder/clamp geometry and mounting permission are not verified.';

function placementKey(config){
  return JSON.stringify([config.punchId,config.dieId,config.stationYmm??0,config.bedLengthMm,config.activeBankId??null,
    Boolean(config.flip),config.contactGapMm??.3,config.ramGapMm??0,
    config.segments.map(s=>[s.id,s.kind,s.punchId??null,s.bankId??null,s.lengthMm,s.centerY,s.offsetX??0,s.offsetZ??0,Boolean(s.mirror)])]);
}

export function appendMirroredToolSetups(configs,{maxCandidates=configs.length*4}={}){
  if(!Number.isInteger(maxCandidates)||maxCandidates<configs.length)
    throw new RangeError('The orientation budget must preserve all original candidates.');
  const result=[...configs],seen=new Set(configs.map(placementKey));
  for(const [punch,die,label] of [[true,false,'PUNCH_REVERSED'],[false,true,'DIE_REVERSED'],[true,true,'PUNCH_AND_DIE_REVERSED']]){
    for(const config of configs){
      if(result.length>=maxCandidates)return result;
      const segments=config.segments.map(segment=>((segment.kind==='punch'&&punch)||(segment.kind==='die'&&die))?
        {...segment,mirror:!segment.mirror}:{...segment});
      const candidate={...config,segments,toolOrientationTrial:label,orientationMountingVerified:false},key=placementKey(candidate);
      if(seen.has(key))continue;seen.add(key);result.push(candidate);
    }
  }
  return result;
}
