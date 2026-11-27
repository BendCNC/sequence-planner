import {unprotect} from './protected-json.mjs';
import {test} from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {toolById,buildPrism,profileArea,signedMeshVolume} from './tool-geometry.mjs';

const catalog=unprotect(readFileSync(new URL('./tooling.json.bct',import.meta.url)));
const ready=catalog.tools.filter(t=>t.status==='ready');

function closedOriented(mesh){
  const edges=new Map();
  for(let i=0;i<mesh.triangles.length;i+=3){
    const tri=mesh.triangles.slice(i,i+3);
    assert.equal(new Set(tri).size,3);
    for(let j=0;j<3;j++){
      const a=tri[j],b=tri[(j+1)%3],key=[Math.min(a,b),Math.max(a,b)].join(',');
      const info=edges.get(key)??{count:0,direction:0};info.count++;info.direction+=a<b?1:-1;edges.set(key,info);
    }
  }
  for(const entry of edges.values()){assert.equal(entry.count,2);assert.equal(entry.direction,0);}
}

test('catalog retains UUID identity, all real variants, and explicitly missing 2288',()=>{
  assert.equal(catalog.units,'mm');assert.equal(catalog.tools.filter(t=>t.kind==='punch').length,5);
  assert.equal(catalog.tools.filter(t=>t.kind==='die').length,12);assert.equal(ready.length,18);
  for(const tool of catalog.tools)assert.equal(toolById(catalog,tool.id),tool);
  const missing=catalog.tools.find(t=>t.vendorCode==='2288');assert.equal(missing.status,'missing');
  assert.throws(()=>buildPrism(missing,100),/unavailable/);
  assert.equal(missing.profile.length,0);assert.equal(missing.contact.tipRadiusMm,null);
});

test('four-way faces map to the physical 16/22/35/50 mm grooves, without UUID/name ambiguity',()=>{
  const byOpening={16:'TOP',22:'BOTTOM',35:'RIGHT',50:'LEFT'};
  for(const die of catalog.tools.filter(t=>t.vendorCode==='4387')){
    const opening=Math.round(die.contact.openingMm);
    assert.equal(die.variant.activeFace,byOpening[opening]);
    assert.ok(Math.abs(die.contact.geometryOpeningMm-opening)<1e-6);
    assert.ok(Math.abs(die.bounds.heightMm-60)<1e-6);
    assert.ok(Math.abs(die.contact.shoulderXs[0]+opening/2)<1e-6);
    assert.ok(Math.abs(die.contact.shoulderXs[1]-opening/2)<1e-6);
  }
});

test('bulge arcs are retained, extrema anchor the tip, and unknown radii remain unknown',()=>{
  const standard=catalog.tools.find(t=>t.vendorCode==='9302');
  assert.equal(standard.source.arcCount,2);
  assert.ok(standard.profile.length>12);
  assert.ok(Math.abs(standard.contact.tipRadiusMm-.9921875)<1e-7);
  const extreme=catalog.tools.find(t=>t.vendorCode==='3924');assert.ok(extreme.profile.length>16);
  for(const p of catalog.tools.filter(t=>t.kind==='punch')){
    assert.ok(Math.abs(p.bounds.minZ)<1e-7);
    assert.ok(p.profile.some(([x,z])=>Math.hypot(x,z)<1e-7));
  }
  assert.equal(catalog.tools.find(t=>t.vendorCode==='3169').contact.tipRadiusMm,null);
});

test('every ready profile extrudes to a closed consistently wound solid with correct volume',()=>{
  for(const tool of ready){
    assert.ok(profileArea(tool)>0);
    for(const length of [25.4,173.2]){
      const mesh=buildPrism(tool,length);closedOriented(mesh);
      assert.ok(mesh.positions.every(Number.isFinite));
      const expected=tool.areaMm2*length;
      assert.ok(Math.abs(signedMeshVolume(mesh)-expected)<Math.max(1e-6,expected*1e-10),tool.name);
      assert.ok(Math.abs(mesh.bounds.size[1]-length)<1e-9);
    }
  }
});

test('mirrored, rotated and translated instances keep positive volume and exact bed endpoints',()=>{
  const tool=catalog.tools.find(t=>t.vendorCode==='8392');
  const mesh=buildPrism(tool,127,{centerY:35,offsetX:18,offsetZ:80,rotationYRad:.7,mirrorX:true});
  closedOriented(mesh);
  assert.ok(Math.abs(signedMeshVolume(mesh)-tool.areaMm2*127)<1e-6);
  assert.equal(mesh.bounds.min[1],35-127/2);assert.equal(mesh.bounds.max[1],35+127/2);
  assert.throws(()=>buildPrism(tool,0),/greater/);assert.throws(()=>buildPrism(tool,NaN),/finite/);
});

test('holes are preserved by the generic prism builder',()=>{
  // Square ring, outer CCW and inner CW in x,z; triangulated annular cap.
  const ring={id:'test-ring',status:'ready',profile:[[0,0],[4,0],[4,4],[0,4]],
    vertices2D:[[0,0],[4,0],[4,4],[0,4],[1,1],[1,3],[3,3],[3,1]],
    loops:[[0,1,2,3],[4,5,6,7]],triangles2D:[0,1,7,0,7,4,1,2,6,1,6,7,2,3,5,2,5,6,3,0,4,3,4,5]};
  const mesh=buildPrism(ring,10);closedOriented(mesh);
  assert.equal(profileArea(ring),12);assert.ok(Math.abs(signedMeshVolume(mesh)-120)<1e-9);
});

test('mounting tangs are preserved, not stretched to catalog working heights',()=>{
  const punch=catalog.tools.find(t=>t.vendorCode==='9302');
  assert.ok(Math.abs(punch.bounds.heightMm-punch.heightMm-15.875)<1e-7);
  assert.ok(Math.abs(punch.contact.tangDepthMm-15.875)<1e-7);
  const die=catalog.tools.find(t=>t.vendorCode==='9858');assert.ok(Math.abs(die.contact.tangDepthMm-12.7)<1e-7);
  const table=catalog.tools.find(t=>t.kind==='table');assert.ok(Math.abs(table.bounds.maxZ)<1e-7);
  const riser=catalog.tools.find(t=>t.kind==='riser');assert.ok(riser.contact.seatingZMm>riser.contact.mountPlaneZMm);
});
