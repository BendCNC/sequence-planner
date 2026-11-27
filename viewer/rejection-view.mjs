import {rejectionReport,sectionForWitness} from './rejection-sections.mjs';

const fmt=(n,d=2)=>n.toLocaleString('es',{maximumFractionDigits:d});
const colors={panel:'#087f8b',band:'#087f8b',punch:'#536b80',die:'#ae8057',table:'#7d8995',ram:'#536b80',bed:'#7d8995',riser:'#7d8995',finger:'#a78329'};
const names={panel:'Panel',band:'Banda',punch:'Punzón',die:'Matriz',table:'Mesa',ram:'Carnero',bed:'Soporte de mesa',riser:'Elevador',finger:'Finger'};
const phases={entry:'Entrada frontal',exit:'Salida frontal',gauge:'Posición del backgauge'};
const meanings={contact:'Contacto o intersección',inside:'Un cuerpo dentro del otro',margin:'Holgura insuficiente'};
function html(tag,text){const e=document.createElement(tag);if(text!=null)e.textContent=text;return e;}
function svg(tag,attrs={},text){
  const e=document.createElementNS('http://www.w3.org/2000/svg',tag);
  for(const [key,v] of Object.entries(attrs))e.setAttribute(key,v);
  if(text!=null)e.textContent=text;return e;
}
function sectionPlot(section,thickness,index,description,showOverlap){
  const [x,,z]=section.point,scale= Math.min(320/Math.max(32,6*thickness,Math.min(110,Math.abs(x)*1.6+18)),
    215/Math.max(28,6*thickness,Math.min(95,Math.abs(z)*1.35+14)));
  const xCenter=Math.abs(x)<95/scale?x/2:x, zCenter=Math.abs(z)<155/scale?z/2:z;
  const X=v=>210+(v-xCenter)*scale,Z=v=>116-(v-zCenter)*scale;
  const plot=svg('svg',{viewBox:'0 0 390 275',role:'img','aria-label':description});
  const defs=svg('defs'),bounds=svg('clipPath',{id:'slice-frame-'+index});
  bounds.append(svg('rect',{x:47,y:8,width:325,height:216}));defs.append(bounds);plot.append(defs);
  const chart=svg('g',{'clip-path':'url(#slice-frame-'+index+')'});plot.append(chart);
  const desired=42/scale,power=10**Math.floor(Math.log10(desired)),step=([1,2,5,10].find(v=>v>=desired/power)??10)*power;
  const xmin=xCenter+(47-210)/scale,zmin=zCenter-(224-116)/scale;
  const tick=(tx,ty,label,anchor)=>plot.append(svg('text',{x:tx,y:ty,'text-anchor':anchor,class:'slice-tick'},fmt(label,3)));
  for(let i=Math.ceil(xmin/step);X(i*step)<=372;i++){
    const u=X(i*step);chart.append(svg('line',{x1:u,x2:u,y1:8,y2:224,class:'slice-grid'}));tick(u,242,i*step,'middle');
  }
  for(let i=Math.ceil(zmin/step);Z(i*step)>=8;i++){
    const v=Z(i*step);chart.append(svg('line',{x1:47,x2:372,y1:v,y2:v,class:'slice-grid'}));tick(40,v+4,i*step,'end');
  }
  const filled=new Map();
  for(const b of [...section.bodies].sort((a,b)=>Number(['panel','band'].includes(a.kind))-Number(['panel','band'].includes(b.kind)))){
    for(const closed of [true,false]){
      const path=b.contours.filter(c=>c.closed===closed).map(c=>c.points.map(([a,h],i)=>(i?'L':'M')+X(a).toFixed(3)+' '+Z(h).toFixed(3)).join('')+(closed?'Z':'')).join('');
      if(!path)continue;
      if(closed)filled.set(b.id,path);
      const piece=['panel','band'].includes(b.kind);
      chart.append(svg('path',{d:path,'fill-rule':'evenodd',fill:closed?colors[b.kind]:'none',
        'fill-opacity':piece?.12:.67,stroke:colors[b.kind]??'#666','stroke-width':piece?1.8:1.2}));
    }
  }
  const [a,b]=section.pair.map(id=>filled.get(id));
  if(a&&b&&showOverlap&&section.kind!=='margin'){
    const clip=svg('clipPath',{id:'pair-cut-'+index});clip.append(svg('path',{d:a,'clip-rule':'evenodd'}));defs.append(clip);
    chart.append(svg('path',{d:b,'fill-rule':'evenodd','clip-path':'url(#pair-cut-'+index+')',fill:'#cc3b34'}));
  }
  const color=section.kind==='margin'?'#ad6100':'#c73534';
  chart.append(svg('circle',{cx:X(x),cy:Z(z),r:14,fill:'none',stroke:color,'stroke-width':2.5,'stroke-dasharray':'6 4'}));
  if(section.kind==='margin'&&section.closestPoints?.length===2){
    for(const p of section.closestPoints)chart.append(svg('circle',{cx:X(p[0]),cy:Z(p[2]),r:2.6,fill:color}));
  }else chart.append(svg('circle',{cx:X(x),cy:Z(z),r:2.8,fill:color}));
  plot.append(svg('path',{d:'M47 8V224H372',stroke:'#8da1b0',fill:'none'}),
    svg('text',{x:206,y:265,'text-anchor':'middle',class:'slice-axis'},'← Frente del operador     ·     X (mm)'),
    svg('text',{transform:'translate(13 115) rotate(-90)','text-anchor':'middle',class:'slice-axis'},'Altura Z (mm)'));
  return plot;
}

function card(model,catalog,w,index,bendName){
  const section=sectionForWitness(model,catalog,w),cfg=w.config;
  const code=id=>catalog.tools.find(t=>t.id===id)?.vendorCode??id;
  const bend=model.bends.find(b=>b.id===w.bendId),angle=Math.abs(bend.signedAngle*w.amounts[w.bendId])*180/Math.PI;
  const phase=w.phase==='retract'?'Retracción de '+fmt(cfg.ramGapMm)+' mm':w.phase==='approach'?'Aproximación · apertura '+fmt(cfg.ramGapMm)+' mm':
    w.phase==='final'?'Cierre final a '+fmt(angle)+'°':w.phase==='fold'?'Plegado a '+fmt(angle)+'°':phases[w.phase];
  const sequence=[...(w.prefix??[]),w.bendId].map(bendName).join(' → '),title=sequence+' · '+phase;
  const wrap=html('article'),heading=html('h3',title);
  const bank=cfg.activeBankId??cfg.segments.find(s=>s.kind==='punch'&&Math.abs(s.centerY-(cfg.stationYmm??0))<=s.lengthMm/2+1e-7)?.bankId;
  const toolName=kind=>{
    const lengths=cfg.segments.filter(s=>s.kind===kind&&(bank==null||bank===s.bankId)).map(s=>fmt(s.lengthMm/25.4)+'″');
    return names[kind]+' '+code(cfg[kind+'Id'])+' · '+lengths.join(' + ');
  };
  wrap.append(heading,html('p',[toolName('punch'),toolName('die')].join('   /   ')));
  const pair=section.pair.map(id=>{
    const body=section.bodies.find(b=>b.id===id);
    if(!body)return id;
    if(body.kind==='panel')return 'Panel '+id.slice(6);
    if(body.kind==='band')return 'Banda '+bendName(id.slice(5));
    if(body.kind==='finger')return id==='finger-left'?'Finger 1':'Finger 2';
    return names[body.kind]+(body.toolId?' '+code(body.toolId):'');
  }).join(' ↔ ');
  wrap.append(sectionPlot(section,model.thicknessMm,index,title+'. '+pair,!w.collision.jointException));
  const caption=html('div');caption.className='slice-caption';
  const headingText=meanings[section.kind];caption.append(html('strong',headingText+' · '+pair));
  const [x,y,z]=section.point;
  caption.append(html('span','Corte Y = '+fmt(y)+' mm · X = '+fmt(x)+' mm · Z = '+fmt(z)+' mm'));
  if(section.kind==='margin'&&Number.isFinite(section.distance))
    caption.append(html('span','Distancia en los puntos señalados '+fmt(section.distance,3)+' mm'+(Number.isFinite(section.margin)?' · margen '+fmt(section.margin,3)+' mm':'')));
  if(w.fixedOnly)caption.append(html('span','Acceso comprobado contra los obstáculos fijos.'));
  wrap.append(caption);
  return {wrap,visible:new Set([section.kind,...section.bodies.filter(b=>b.contours.length).map(b=>b.kind)])};
}

export function clearRejectionView(){
  const dialog=document.getElementById('rejectionDialog'),button=document.getElementById('openRejection');
  if(dialog.open)dialog.close();
  button.hidden=true;button.onclick=null;
  document.getElementById('rejectionCases').replaceChildren();
  for(const id of ['previousCases','nextCases'])document.getElementById(id).onclick=null;
}

export function showRejectionView(model,catalog,result,bendName){
  clearRejectionView();
  const report=rejectionReport(model,result,bendName);if(!report)return;
  const get=id=>document.getElementById(id),dialog=get('rejectionDialog'),button=get('openRejection');
  const title=report.cases.length?'Interferencias de plegado':'Resultado de la búsqueda';
  get('rejectionTitle').textContent=title+' · '+(model.partName??model.metadata?.partName??'Pieza');
  get('rejectionSummary').textContent=[report.message,...report.notes].join(' ');
  let page=0;
  function draw(){
    const group=report.cases.slice(page*2,page*2+2),grid=get('rejectionCases');
    const cards=group.map((w,i)=>card(model,catalog,w,i,bendName));
    grid.replaceChildren(...cards.map(c=>c.wrap));
    const legend=get('rejectionLegend');legend.hidden=!group.length;
    const legendKinds={part:['panel','band'],fixture:['table','riser'],contact:['contact','inside']};
    for(const key of legend.children)key.hidden=!cards.some(c=>(legendKinds[key.className]??[key.className]).some(k=>c.visible.has(k)));
    get('rejectionNav').hidden=report.cases.length<=2;
    get('rejectionPage').textContent='Casos '+(page*2+1)+'–'+Math.min(page*2+2,report.cases.length)+' de '+report.cases.length;
    get('previousCases').disabled=page===0;get('nextCases').disabled=(page+1)*2>=report.cases.length;
  }
  get('previousCases').onclick=()=>{page--;draw();};get('nextCases').onclick=()=>{page++;draw();};
  button.hidden=false;button.textContent=report.cases.length?'Ver cortes de interferencia':'Ver motivo del rechazo';
  button.onclick=()=>{draw();dialog.showModal();dialog.scrollTop=0;};
  button.onclick();
}
