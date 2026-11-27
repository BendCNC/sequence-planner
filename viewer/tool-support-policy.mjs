/** Planning allowance at outer bend ends; physical coverage remains unchanged. */
export const DEFAULT_UNSUPPORTED_END_MM=12.7;
export function toolSupportPolicy(options={}){
  const maxUnsupportedEndMm=options.maxUnsupportedEndMm??(options.allowPartialCoverage===false?0:DEFAULT_UNSUPPORTED_END_MM);
  if(!Number.isFinite(maxUnsupportedEndMm)||maxUnsupportedEndMm<0)throw new RangeError('maxUnsupportedEndMm must be finite and nonnegative.');
  return{maxUnsupportedEndMm};
}

export function assessToolSupport(coverage,options={}){
  const policy=toolSupportPolicy(options),eps=coverage.epsilonMm??1e-7,material=coverage.materialIntervals,byKind={};
  for(const kind of ['punch','die']){
    const data=coverage.byKind[kind],missing=data.missingIntervals;
    let leftMm=0,rightMm=0,internalMissing=false,unsupportedIsland=false;
    for(const [a,b]of missing){
      if(material.some(([lo,hi])=>a<=lo+eps&&b>=hi-eps))unsupportedIsland=true;
      if(Math.abs(a-material[0]?.[0])<=eps)leftMm+=b-a;
      else if(Math.abs(b-material.at(-1)?.[1])<=eps)rightMm+=b-a;
      else internalMissing=true;
    }
    const accepted=coverage.exact===true&&!internalMissing&&!unsupportedIsland&&
      leftMm<=policy.maxUnsupportedEndMm+eps&&rightMm<=policy.maxUnsupportedEndMm+eps;
    byKind[kind]={accepted,leftMm,rightMm,internalMissing,unsupportedIsland};
  }
  const accepted=Object.values(byKind).every(k=>k.accepted),fullCoverage=coverage.status==='COVERED';
  return{...policy,accepted,fullCoverage,usesEndAllowance:accepted&&!fullCoverage,byKind,
    status:!coverage.exact?'UNKNOWN':!accepted?'INSUFFICIENT_SUPPORT':fullCoverage?'FULL_COVERAGE':'ALLOWED_END_GAPS',
    formingVerified:false};
}
