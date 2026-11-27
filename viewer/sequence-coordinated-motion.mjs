/** Playback of explicitly checked simultaneous ram / rigid-part keyframes.
 * This module does not validate a route or turn it into a machine command.
 */
const axes=['x','y','z','tiltDeg'];
export function coordinatedMotionAt(motion,q){
  if(!Array.isArray(motion?.frames)||motion.frames.length<2||!Array.isArray(motion.pivot)||motion.pivot.length!==3||!motion.pivot.every(Number.isFinite))
    throw new TypeError('Coordinated motion requires a pivot and at least two keyframes.');
  const frames=motion.frames;
  for(const f of frames)if(!Number.isFinite(f.ramGapMm)||f.ramGapMm<0||axes.some(k=>!Number.isFinite(f.pose?.[k]??0)))
    throw new TypeError('Invalid coordinated-motion keyframe.');
  if(!Number.isFinite(q))throw new TypeError('Invalid motion progress.');
  const lengths=frames.slice(1).map((b,i)=>{const a=frames[i];return Math.hypot(...axes.slice(0,3).map(k=>(b.pose[k]??0)-(a.pose[k]??0)))+
    2*Math.abs((b.pose.tiltDeg??0)-(a.pose.tiltDeg??0))+Math.abs(b.ramGapMm-a.ramGapMm);});
  let distance=Math.max(0,Math.min(1,q))*lengths.reduce((a,b)=>a+b,0);
  for(let i=0;i<lengths.length;i++){
    if(distance<=lengths[i]||i===lengths.length-1){const a=frames[i],b=frames[i+1],t=lengths[i]?Math.max(0,Math.min(1,distance/lengths[i])):1;
      return{ramGapMm:a.ramGapMm+(b.ramGapMm-a.ramGapMm)*t,pivot:motion.pivot,
        pose:Object.fromEntries(axes.map(k=>[k,(a.pose[k]??0)+((b.pose[k]??0)-(a.pose[k]??0))*t]))};}
    distance-=lengths[i];
  }
}
