/** Rigid manipulation about machine Y, followed by X/Z translation. */
export function transformFrontTiltPoint(point,pose,pivot=[0,0,0],inverse=false){
  const a=(pose.tiltDeg??0)*Math.PI/180,c=Math.cos(a),s=Math.sin(a),x=pose.x??0,z=pose.z??0;
  if(inverse){const px=point[0]-pivot[0]-x,pz=point[2]-pivot[2]-z;
    return[pivot[0]+c*px-s*pz,point[1],pivot[2]+s*px+c*pz];}
  const px=point[0]-pivot[0],pz=point[2]-pivot[2];
  return[pivot[0]+c*px+s*pz+x,point[1],pivot[2]-s*px+c*pz+z];
}
