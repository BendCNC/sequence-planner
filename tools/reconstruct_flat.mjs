#!/usr/bin/env node
// Rebuild the flat pattern's bend angles/directions/bend-line positions from the
// folded 3D reference body. Writes the corrected model only when it passes input
// validation (status RECONSTRUCTED). Exit codes: 0 RECONSTRUCTED/NOT_NEEDED,
// 4 PARTIAL/FAILED, 2 usage or input error.
import fs from 'node:fs/promises';
import {reconstructFlat, describeChanges} from '../viewer/flat-reconstruction.mjs';

const args = process.argv.slice(2), opt = {};
try {
  for (let i = 0; i < args.length; i += 2) {
    const key = args[i]?.replace(/^--/, '');
    if (!args[i]?.startsWith('--') || !['model', 'reference', 'output', 'report', 'max-offset-mm'].includes(key) || args[i+1] === undefined)
      throw new Error('Invalid argument: ' + args[i]);
    opt[key] = args[i+1];
  }
  if (!opt.model || !opt.reference || !opt.report)
    throw new Error('Use --model model.json --reference reference-mesh.json --report report.json [--output corrected-model.json] [--max-offset-mm 25]');
  const model = JSON.parse(await fs.readFile(opt.model, 'utf8')), reference = JSON.parse(await fs.readFile(opt.reference, 'utf8'));
  const r = reconstructFlat(model, reference, {maxOffsetMm: opt['max-offset-mm'] ? +opt['max-offset-mm'] : 25});
  const {model: corrected, ...report} = r;
  report.summary = describeChanges(r.changes ?? []);
  if (corrected && opt.output) { await fs.writeFile(opt.output, JSON.stringify(corrected)); report.output = opt.output; }
  await fs.writeFile(opt.report, JSON.stringify(report, null, 2));
  console.log(JSON.stringify({status: report.status, summary: report.summary, reason: report.reason ?? null,
    maxDeviationMmBefore: report.maxDeviationMmBefore, maxDeviationMmAfter: report.maxDeviationMmAfter}));
  process.exit(['RECONSTRUCTED', 'NOT_NEEDED'].includes(r.status) ? 0 : 4);
} catch (e) {
  console.error(e.message); process.exit(2);
}
