import test from 'node:test';
import assert from 'node:assert/strict';
import {overhangBankFits} from './tool-setup-search.mjs';

const IN=25.4,BED=838.2;

test('overhang bank 16+8+4+16 fits: more than half of each end section on the bed',()=>{
  assert.equal(overhangBankFits([16*IN,8*IN,4*IN,16*IN],BED),true);
});

test('overhang bank rejected when an end section would sit mostly off the bed',()=>{
  assert.equal(overhangBankFits([16*IN,16*IN,16*IN,16*IN],BED),false);
});

test('bank within the bed always fits',()=>{
  assert.equal(overhangBankFits([16*IN,8*IN,4*IN],BED),true);
});
