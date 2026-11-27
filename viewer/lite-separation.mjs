const axes=[[1,1],[1,-1],[2,1],[2,-1],[1,2],[1,-2]];
export function shadow(body) {
 const out=new Float64Array(axes.length*2+1),p=body.positions,n=p.length;
 const packed=typeof p[0]==='number';
 for(let a=0;a<axes.length;a++){
  const [ax,az]=axes[a];let lo=Infinity,hi=-Infinity;
  for(let i=0;i<n;i+=packed?3:1){
   const sx=packed?ax*p[i]:ax*p[i][0],sz=packed?az*p[i+2]:az*p[i][2];out[axes.length*2]=Math.max(out[axes.length*2],Math.abs(sx)+Math.abs(sz));const t=sx+sz;lo=Math.min(lo,t);hi=Math.max(hi,t);
  }
  out[a*2]=lo;out[a*2+1]=hi;
 }
 return out;
}
export function certifiedSeparation(a,b,x,z,c,pivot=0){
 for(let k=0;k<axes.length;k++){
  const [ax,az]=axes[k], v=x*ax+z*az;
  const pad=(c+1e-7)*Math.hypot(ax,az)+1e-8*(1+Math.abs(pivot)+Math.abs(z)+Math.abs(x)+a[axes.length*2]+b[axes.length*2]);
  if(a[k*2]+v-b[k*2+1]>pad || b[k*2]-(a[k*2+1]+v)>pad)return true;
 }
 return false;
}