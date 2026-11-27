/** Joint-stroke state contract shared by normal and advanced search.
 * Followers are quantities of the same physical stroke, never independent jobs.
 */
const EPS=1e-8;

export function strokeAllowed(model,bend,amounts){
  if(bend.strokeLeadId!=null)return false;
  if((bend.strokeRequiresFlatBendIds??[]).some(id=>amounts[id]>EPS))return false;
  return !model.bends.some(l=>(l.strokeRequiresFlatBendIds??[]).map(String).includes(String(bend.id))&&amounts[l.id]>EPS&&amounts[l.id]<1-EPS);
}

/** Monotone complete-plan search cannot close a carrier before its joint
 * stroke, because it cannot flatten that carrier again later. The physical
 * single-operation API retains the weaker strokeAllowed contract above. */
export function strokeSearchAllowed(model,bend,amounts){
  return strokeAllowed(model,bend,amounts)&&!model.bends.some(l=>
    (l.strokeRequiresFlatBendIds??[]).map(String).includes(String(bend.id))&&amounts[l.id]<1-EPS);
}

/** Update a newly allocated state just as the normal search has always done. */
export function strokeAmounts(bend,amounts){
  for(const id of bend.strokeFollowerIds??[])amounts[id]=amounts[bend.id];
  return amounts;
}

export function strokeStateError(model,amounts){
  const byId=new Map(model.bends.map(b=>[String(b.id),b]));
  for(const bend of model.bends){
    if(!Number.isFinite(amounts?.[bend.id])||amounts[bend.id]<0||amounts[bend.id]>1)return 'Invalid stroke amount';
    if(bend.strokeLeadId!=null){
      const lead=byId.get(String(bend.strokeLeadId));
      if(!lead||lead.strokeLeadId!=null||!(lead.strokeFollowerIds??[]).map(String).includes(String(bend.id)))return 'Invalid joint-stroke membership';
      if(Math.abs(amounts[bend.id]-amounts[lead.id])>EPS)return 'Joint-stroke followers must match their lead';
    }
    for(const id of bend.strokeFollowerIds??[]){
      const follower=byId.get(String(id));
      if(!follower||String(follower.strokeLeadId)!==String(bend.id)||bend.strokeLeadId!=null)return 'Invalid joint-stroke membership';
    }
    if((bend.strokeRequiresFlatBendIds??[]).some(id=>!byId.has(String(id))))return 'Unknown joint-stroke carrier';
  }
  return null;
}

/** Validate recorded amounts as well as the solver's effective geometry. */
export function strokeTransitionError(model,bend,from,to){
  if(!bend)return 'Unknown active bend';
  const stateError=strokeStateError(model,from)??strokeStateError(model,to);
  if(stateError)return stateError;
  if(bend.strokeLeadId!=null)return 'A joint-stroke follower cannot be operated independently';
  if(!(to[bend.id]>from[bend.id]))return 'A stroke must increase its active bend';
  const active=new Set([bend.id,...(bend.strokeFollowerIds??[])].map(String));
  if(model.bends.some(b=>!active.has(String(b.id))&&from[b.id]!==to[b.id]))return 'A stroke must change only its lead and followers';
  if(!strokeAllowed(model,bend,from)||!strokeAllowed(model,bend,to))return 'Joint-stroke carrier bends must remain flat until the stroke is complete';
  return null;
}
