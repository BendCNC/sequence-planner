/** Rigid backgauge fingers. mm; front −X, station Y, up Z. No renderer dependency. */
import {meshBounds} from './tool-geometry.mjs';

function finite(value,name){
  if(typeof value!=='number'||!Number.isFinite(value))throw new TypeError(`${name} must be a finite number.`);
  return value;
}

function placedFeatures(features,x,y,z){
  const placed={};
  for(const reference of ['lower','upper']){
    const feature=features?.[reference];
    if(!feature)continue;
    placed[reference]={...feature,planeX:feature.planeX+x,minY:feature.minY+y,maxY:feature.maxY+y,
      minZ:feature.minZ+z,maxZ:feature.maxZ+z,
      supportSurfaceZ:feature.supportSurfaceZ===null||feature.supportSurfaceZ===undefined?null:feature.supportSurfaceZ+z};
    if(feature.supportTerraceXY)placed[reference].supportTerraceXY=feature.supportTerraceXY.map(([px,py])=>[px+x,py+y]);
  }
  if(features?.hingeAxis)placed.hingeAxis={...features.hingeAxis,point:features.hingeAxis.point.map((v,i)=>v+[x,y,z][i])};
  return placed;
}

/**
 * Always returns both physically present fingers. Which one(s) must contact the
 * part is a separate planning requirement and never removes a collision body.
 * lowerFaceX and baseZ are common to both; stationY is the fixed two-finger setup.
 * Coordinates are geometric. They are not calibrated machine X/R command values.
 */
export function buildFingerBodies({definition,lowerFaceX,baseZ,stationY,collision=true}){
  if(!definition?.id)throw new TypeError('A finger definition is required.');
  finite(lowerFaceX,'lowerFaceX');finite(baseZ,'baseZ');
  if(!Array.isArray(stationY)||(stationY.length!==1&&stationY.length!==2))throw new TypeError('One or two stationY positions are required.');
  stationY.forEach((value,i)=>finite(value,`stationY[${i}]`));
  if(typeof collision!=='boolean')throw new TypeError('collision must be boolean.');
  const leftMesh=collision?definition.collisionMesh:definition.mesh,mesh=leftMesh;
  if(!mesh?.positions||!mesh?.triangles||mesh.positions.length<12||mesh.positions.length%3||mesh.triangles.length<12||mesh.triangles.length%3)
    throw new TypeError('Finger mesh is unavailable or malformed.');
  if(!mesh.positions.every(Number.isFinite)||!mesh.triangles.every(i=>Number.isInteger(i)&&i>=0&&i<mesh.positions.length/3))
    throw new TypeError('Finger mesh has invalid coordinates or indices.');
  // Handed fingers (original STEP finger): finger-right is the mirrored solid.
  const rightMesh=(collision?definition.rightCollisionMesh:definition.rightMesh)??mesh;
  return stationY.map((centerY,index)=>{
    const mesh=index===1?rightMesh:leftMesh;
    const positions=new Float64Array(mesh.positions.length);
    for(let i=0;i<positions.length;i+=3){positions[i]=mesh.positions[i]+lowerFaceX;positions[i+1]=mesh.positions[i+1]+centerY;positions[i+2]=mesh.positions[i+2]+baseZ;}
    return{id:index===0?'finger-left':'finger-right',kind:'finger',definitionId:definition.id,
      positions,triangles:[...mesh.triangles],bounds:meshBounds(positions),
      placement:{lowerFaceX,baseZ,stationY:centerY},collisionEnvelope:collision,
      contactFeatures:placedFeatures(definition.contactFeatures,lowerFaceX,centerY,baseZ)};
  });
}
