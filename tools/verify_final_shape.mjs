#!/usr/bin/env node
// Compare the plan's real final state (last stage `to` amounts, every span included)
// with a reference mesh of the folded CAD part.
// Usage: node tools/verify_final_shape.mjs --model m.json --reference ref.json [--plan plan.json] [--output out.json] [--tolerance mm]
import fs from 'node:fs';
import {compareWithReference} from '../viewer/reference-shape.mjs';
import {consistentInputFromComparison} from '../viewer/input-validation.mjs';

const args = {};
for (let i = 2; i < process.argv.length; i += 2) args[process.argv[i].replace(/^--/, '')] = process.argv[i + 1];
const model = JSON.parse(fs.readFileSync(args.model, 'utf8'));
const reference = args.reference && fs.existsSync(args.reference) ? JSON.parse(fs.readFileSync(args.reference, 'utf8')) : null;
let amounts = Object.fromEntries(model.bends.map(b => [b.id, 1])), source = 'allBendsComplete';
const notes = [];
if (args.plan) {
  const plan = JSON.parse(fs.readFileSync(args.plan, 'utf8'));
  const last = plan.stages?.at(-1);
  if (last?.to) { amounts = Object.fromEntries(model.bends.map(b => [b.id, last.to[b.id] ?? 0])); source = 'planFinalStage'; }
  else notes.push('plan has no stages: checked the fully folded model instead');
}
const incomplete = model.bends.filter(b => Math.abs((amounts[b.id] ?? 0) - 1) > 1e-6).map(b => b.label ?? b.id);
const result = {
  schemaVersion: 1, finalStateSource: source, finalStateComplete: incomplete.length === 0, incompleteBends: incomplete, notes,
  ...compareWithReference(model, amounts, reference, {toleranceMm: args.tolerance ? Number(args.tolerance) : undefined}),
};
if (result.suspectBends?.length) notes.push(`Final shape differs from the CAD after: `
  + result.suspectBends.map(b => `${b.label} (${b.signedAngleDeg} deg, child off by ${b.childMedianDeviationMm} mm)`).join(', ')
  + (result.alternativeSuspectBends?.length ? `; or, equivalently, the other root bends: ${result.alternativeSuspectBends.map(b => b.label).join(', ')}` : ''));
else if (result.localDeviations?.length) notes.push(`Final shape differs locally; every flange lands on the CAD. Largest deviation at: `
  + result.localDeviations.map(p => `${p.kind === 'bend' ? `bend zone ${p.label}` : `panel ${p.label}`} (${p.maxDeviationMm} mm)`).join(', ')
  + `. No bend is blamed: check cut-outs or reliefs near these zones (e.g. one ending inside a bend zone).`);
if (result.pass && incomplete.length) { result.pass = false; result.status = 'INCOMPLETE_FINAL_STATE'; }
if(args['include-input-consistency']==='yes'&&!args.plan&&!args.tolerance&&reference?.positions?.length&&
   model.bends.length&&result.pass===true&&result.status==='MATCH'){
  result.inputValidation=consistentInputFromComparison(model,result);
}
const text = JSON.stringify(result, null, 2);
if (args.output) fs.writeFileSync(args.output, text); else console.log(text);
process.exitCode = result.pass === false ? 2 : 0;
