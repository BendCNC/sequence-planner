import test from 'node:test';
import assert from 'node:assert/strict';
import {meshCollider, meshColliderForScene, checkPair} from './collision.mjs';

test('large scene meshes preserve leaf order and exact collision witnesses across poses', () => {
  const vertices = [], faces = [];
  for (let i = 0; i < 512; ++i) {
    const x = i % 32, y = Math.floor(i / 32), j = vertices.length;
    vertices.push([x,y,0], [x+0.75,y,0], [x,y+0.75,0]);
    faces.push([j,j+1,j+2], [j,j+1,j+2]); // Equal centers and equal closest distances.
  }
  const controlFaces = faces.map(face => [...face]);
  const build = (builder, positions, triangles) => builder({id:'sheet', positions, triangles});
  build(meshCollider, vertices, controlFaces);
  build(meshColliderForScene, vertices, faces);

  function compareNodes(a, b) {
    assert.deepEqual(a.bounds, b.bounds);
    assert.equal(a.count, b.count);
    if (a.items) {
      assert.deepEqual(a.items.map(item => item.index), b.items.map(item => item.index));
    } else {
      compareNodes(a.left, b.left);
      compareNodes(a.right, b.right);
    }
  }

  for (const turn of [false, true]) {
    const positions = vertices.map(([x,y,z]) => turn ? [y+.25, 32-x, z+.5] : [x,y,z]);
    if (turn) positions[2] = positions[1]; // The first two faces collapse only in this pose.
    const old = build(meshCollider, positions, controlFaces);
    const scene = build(meshColliderForScene, positions, faces);
    assert.equal(scene.diagnostics.degenerateTriangles, turn ? 2 : 0);
    assert.deepEqual(scene.diagnostics, old.diagnostics);
    for (const gap of [0, .25]) {
      const target = meshCollider({id:'probe', triangles:[0,1,2],
        positions:positions.slice(3,6).map(([x,y,z]) => [x,y,z+gap])});
      for (const queryMode of ['distance', 'clearance']) {
        const options = {queryMode, clearanceMm:.125, toleranceMm:0};
        const result = checkPair(scene, target, options);
        assert.equal(result.status, gap === 0 ? 'INTERFERENCE' : 'CLEAR');
        assert.deepEqual(result, checkPair(old, target, options));
      }
    }
    compareNodes(old.bvh, scene.bvh);
    const firstLeaf = root => root.items ? root : firstLeaf(root.left);
    for (const triangle of firstLeaf(scene.bvh).items) assert.equal(triangle, scene.items[triangle.index]);
  }
});
