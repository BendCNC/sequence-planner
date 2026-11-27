#!/usr/bin/env node
// Resolve bend sides (and the counter-tab reading) that the flat data leaves open,
// against the folded CAD reference mesh. Each --model is one import variant of the
// same part. Source bends are flipped in subsets of growing size (fewest changes
// first, variants in the given order); the first fully folded trial within
// tolerance wins. Nothing is guessed: no match within the budget reports NO_MATCH.
// Trials run on worker threads; the result equals a sequential search.
// Usage: node tools/resolve_bend_sides.mjs --reference ref.json --model a.json [--model b.json]
//        [--output out.json] [--max-trials 80] [--tolerance mm] [--workers n]
import fs from 'node:fs';
import os from 'node:os';
import {Worker, isMainThread, parentPort, workerData} from 'node:worker_threads';
import {compareWithReference} from '../viewer/reference-shape.mjs';

const sourceOf = b => String(b.sourceBendId ?? String(b.id).split(':')[0]);

function* subsets(items, size, start = 0, acc = []) {
  if (acc.length === size) { yield acc.slice(); return; }
  for (let i = start; i < items.length; i++) { acc.push(items[i]); yield* subsets(items, size, i + 1, acc); acc.pop(); }
}

if (!isMainThread) {
  const {models, reference, toleranceMm, trials} = workerData;
  for (const {index, variant, flip} of trials) {
    const flipped = new Set(flip), model = models[variant];
    const trial = {...model, bends: model.bends.map(b => flipped.has(sourceOf(b)) ? {...b, signedAngle: -b.signedAngle} : b)};
    const r = compareWithReference(trial, Object.fromEntries(trial.bends.map(b => [b.id, 1])), reference, {toleranceMm});
    parentPort.postMessage({index, variant, flip, maxDeviationMm: r.maxDeviationMm, toleranceMm: r.toleranceMm, pass: r.pass});
    if (r.pass) break;
  }
} else {
  const paths = [], args = {};
  for (let i = 2; i < process.argv.length; i += 2) {
    const key = process.argv[i].replace(/^--/, '');
    if (key === 'model') paths.push(process.argv[i + 1]); else args[key] = process.argv[i + 1];
  }
  const reference = JSON.parse(fs.readFileSync(args.reference, 'utf8'));
  const toleranceMm = args.tolerance ? Number(args.tolerance) : undefined;
  const models = paths.map(p => JSON.parse(fs.readFileSync(p, 'utf8')));
  const ids = models.map(m => [...new Set(m.bends.map(sourceOf))].sort());
  const maxTrials = Number(args['max-trials'] ?? 80), trials = [];
  order: for (let size = 0; size <= Math.max(...ids.map(i => i.length)); size++)
    for (const [variant, list] of ids.entries())
      if (size <= list.length) for (const flip of subsets(list, size)) {
        if (trials.length >= maxTrials) break order;
        trials.push({index: trials.length, variant, flip});
      }
  const k = Math.max(1, Math.min(trials.length, Number(args.workers ?? Math.min(4, Math.max(1, os.cpus().length - 1)))));
  const results = new Map(), workers = [];
  let match = null;
  // Stop once every earlier trial is known, so the earliest match wins.
  const settled = () => match && [...Array(match.index).keys()].every(i => results.has(i));
  await new Promise(resolve => {
    let alive = k;
    for (let w = 0; w < k; w++) {
      const worker = new Worker(new URL(import.meta.url), {workerData: {models, reference, toleranceMm,
        trials: trials.filter(t => t.index % k === w)}});
      workers.push(worker);
      worker.on('message', m => {
        results.set(m.index, m);
        if (m.pass && (!match || m.index < match.index)) match = m;
        if (settled()) { workers.forEach(x => x.terminate()); resolve(); }
      });
      worker.on('error', e => { console.error(e); process.exitCode = 3; });
      worker.on('exit', () => { if (--alive === 0) resolve(); });
    }
  });
  const entry = m => ({variant: m.variant, model: paths[m.variant], flippedSourceBendIds: m.flip,
    maxDeviationMm: m.maxDeviationMm, toleranceMm: m.toleranceMm});
  const best = [...results.values()].sort((a, b) => a.maxDeviationMm - b.maxDeviationMm)[0];
  const result = match ? {status: 'MATCH', ...entry(match)} : {status: 'NO_MATCH', best: best ? entry(best) : null};
  result.trials = results.size;
  const text = JSON.stringify(result, null, 2);
  if (args.output) fs.writeFileSync(args.output, text); else console.log(text);
  process.exitCode ??= match ? 0 : 2;
}
