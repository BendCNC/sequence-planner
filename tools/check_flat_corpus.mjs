#!/usr/bin/env node
// Corpus regression: every imported model must have a self-contact-free flat blank.
// Usage: node tools/check_flat_corpus.mjs model1.json [model2.json ...]
// (import .bend packages first with engine/import_bend_package.py).
import {readFileSync} from 'node:fs';
import {basename} from 'node:path';
import {checkFlatSelfContact} from '../viewer/sequence-flat-self-check.mjs';

let failed=0;
for(const file of process.argv.slice(2).filter(f=>!f.endsWith('.verification.json'))){
  let line;
  try{
    const result=checkFlatSelfContact(JSON.parse(readFileSync(file,'utf8')));
    if(result.status!=='FLAT_CLEAR')failed++;
    line=`${result.status}\t${basename(file)}${result.hits.length?'\t'+JSON.stringify(result.hits[0]):''}`;
  }catch(error){failed++;line=`ERROR\t${basename(file)}\t${error.message}`;}
  console.log(line);
}
process.exitCode=failed?1:0;
