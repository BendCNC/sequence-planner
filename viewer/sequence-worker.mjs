import {planCompleteSequence} from './sequence-plan.mjs';
import {planExperimentalSequence} from './sequence-portfolio.mjs';

self.onmessage=async event=>{
  const{id,payload}=event.data;
  try{
    const plan=payload.stationTrial?(await import('./examples/sm1-stations.mjs')).verifySM1Stations:
      payload.options?.experimentalFallback?planExperimentalSequence:planCompleteSequence;
    const result=await plan(payload,progress=>self.postMessage({id,type:'progress',result:progress}));
    self.postMessage({id,type:'result',result});
  }catch(error){self.postMessage({id,type:'error',error:error.message});}
};
