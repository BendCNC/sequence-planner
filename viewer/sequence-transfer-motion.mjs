/** Browser-safe playback of the analytically bounded exterior transfer. */

function unitQuaternion(values){
  if(!Array.isArray(values)||values.length!==4||!values.every(Number.isFinite))throw new TypeError('Invalid XYZW quaternion');
  const length=Math.hypot(...values);if(length<1e-12)throw new RangeError('Zero quaternion');
  return values.map(v=>v/length);
}
function slerp(first,last,t){
  const a=unitQuaternion(first);let b=unitQuaternion(last),dot=a.reduce((sum,v,i)=>sum+v*b[i],0);
  if(dot<0){b=b.map(v=>-v);dot=-dot;}
  if(dot>.9995)return unitQuaternion(a.map((v,i)=>v+(b[i]-v)*t));
  const theta=Math.acos(Math.max(-1,Math.min(1,dot))),denominator=Math.sin(theta);
  return unitQuaternion(a.map((v,i)=>(Math.sin((1-t)*theta)*v+Math.sin(t*theta)*b[i])/denominator));
}
function finitePoint(point){return Array.isArray(point)&&point.length===3&&point.every(Number.isFinite);}

/**
 * Return canonical-part -> machine-world matrix, row-major, column-vector convention.
 * q is clamped to [0,1]. Three equal-duration legs connect the four stored frames:
 * retreat farther in front, reorient/change station outside, approach insertion start.
 * The geometry's canonical center is a vertex mean, not a material center of mass.
 */
export function poseTransferAt(transfer,q){
  if(!Number.isFinite(q))throw new TypeError('Transfer progress must be finite');
  const frames=transfer?.keyframes,canonicalCenter=transfer?.canonical?.vertexMean;
  if(!Array.isArray(frames)||frames.length!==4||!finitePoint(canonicalCenter))throw new TypeError('A four-frame transfer and canonical vertex mean are required');
  if(frames.some(f=>!finitePoint(f.vertexMeanWorld)||!Array.isArray(f.matrixRowMajor)||f.matrixRowMajor.length!==16||!f.matrixRowMajor.every(Number.isFinite)))
    throw new TypeError('Invalid transfer frame');
  const progress=Math.max(0,Math.min(1,q));
  if(progress===0)return [...frames[0].matrixRowMajor];
  if(progress===1)return [...frames[3].matrixRowMajor];
  const at=progress*3,index=Math.min(2,Math.floor(at)),t=at-index,a=frames[index],b=frames[index+1];
  const center=a.vertexMeanWorld.map((v,i)=>v+(b.vertexMeanWorld[i]-v)*t);
  const[x,y,z,w]=slerp(a.quaternionXYZW,b.quaternionXYZW,t);
  const matrix=[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w),0,
    2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w),0,
    2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y),0,0,0,0,1];
  for(let row=0;row<3;row++)matrix[row*4+3]=center[row]-canonicalCenter.reduce((sum,v,i)=>sum+matrix[row*4+i]*v,0);
  return matrix;
}
