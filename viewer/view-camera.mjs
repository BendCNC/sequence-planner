import * as THREE from './vendor/three.module.js';
import {OrbitControls} from './vendor/OrbitControls.js';

/** Shared camera for the simulator and sequence reports. Z is machine height. */
export class ViewCamera {
  constructor(element,{direction=[-1,-1.15,.75],dampingFactor=.05,onChange=()=>{}}={}){
    this.element=element;this.defaultDirection=direction;this.dampingFactor=dampingFactor;
    this.onChange=onChange;this.mode='iso';this.aspect=1;this.orthoHeight=500;
    this.activate();
  }
  activate(){
    this.controls?.dispose();
    const orthographic=['side-right','side-left','top-ortho'].includes(this.mode);
    this.camera=orthographic?new THREE.OrthographicCamera(-250,250,250,-250,.1,30000):new THREE.PerspectiveCamera(36,this.aspect,.1,30000);
    // Feed is +X into the press: top views put +X up, hence the operator (−X) below.
    this.camera.up.set(...(this.mode.startsWith('top')?[1,0,0]:[0,0,1]));this.camera.position.set(600,-800,500);
    this.controls=new OrbitControls(this.camera,this.element);
    this.controls.enableDamping=true;this.controls.dampingFactor=this.dampingFactor;
    if(orthographic){
      this.controls.enableRotate=false;
      this.controls.mouseButtons.LEFT=THREE.MOUSE.PAN;
      this.controls.touches.ONE=THREE.TOUCH.PAN;
      this.controls.touches.TWO=THREE.TOUCH.DOLLY_PAN;
    }
    this.resize(this.element.clientWidth,this.element.clientHeight);
    this.onChange(this.camera,this.controls,this.mode);
  }
  resize(width,height){
    if(width<=0||height<=0)return;
    this.aspect=width/height;
    if(this.camera.isOrthographicCamera){
      this.camera.top=this.orthoHeight/2;this.camera.bottom=-this.camera.top;
      this.camera.right=this.camera.top*this.aspect;this.camera.left=-this.camera.right;
    }else this.camera.aspect=this.aspect;
    this.camera.updateProjectionMatrix();
  }
  setView(mode,box){
    if(mode==='side')mode='side-right'; // compatibility with existing callers
    if(!['side-right','side-left','iso','front','top','top-ortho'].includes(mode))throw new Error('Unknown view: '+mode);
    this.mode=mode;this.activate();this.fit(box);
  }
  fit(box){
    if(!box||box.isEmpty())return;
    const direction={'side-right':[0,-1,0],'side-left':[0,1,0],front:[-1,0,.15],top:[0,0,1],'top-ortho':[0,0,1],iso:this.defaultDirection}[this.mode];
    const back=new THREE.Vector3(...direction).normalize(),center=box.getCenter(new THREE.Vector3());
    const right=new THREE.Vector3().crossVectors(this.camera.up,back).normalize();
    const up=new THREE.Vector3().crossVectors(back,right).normalize();
    const tanV=Math.tan(36*Math.PI/360),tanH=tanV*this.aspect;
    let halfWidth=0,halfHeight=0,halfDepth=0,distance=1;
    for(const x of [box.min.x,box.max.x])for(const y of [box.min.y,box.max.y])for(const z of [box.min.z,box.max.z]){
      const p=new THREE.Vector3(x,y,z).sub(center),depth=p.dot(back);
      halfWidth=Math.max(halfWidth,Math.abs(p.dot(right)));
      halfHeight=Math.max(halfHeight,Math.abs(p.dot(up)));halfDepth=Math.max(halfDepth,Math.abs(depth));
      distance=Math.max(distance,Math.abs(p.dot(right))/tanH+depth,Math.abs(p.dot(up))/tanV+depth);
    }
    if(this.camera.isOrthographicCamera){
      this.orthoHeight=2*Math.max(halfHeight,halfWidth/this.aspect,1)*1.12;
      distance=Math.max(box.getSize(new THREE.Vector3()).length()*2,10);
    }else distance*=1.12;
    this.camera.zoom=1;this.camera.near=Math.max(.01,distance/10000);
    this.camera.far=Math.max(10000,(distance+halfDepth)*2);
    this.camera.position.copy(center).addScaledVector(back,distance);
    this.controls.target.copy(center);this.camera.lookAt(center);
    this.resize(this.element.clientWidth,this.element.clientHeight);
    this.controls.update();
  }
}
