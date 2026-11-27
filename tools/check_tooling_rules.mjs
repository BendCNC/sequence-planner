#!/usr/bin/env node
// Lightweight necessary-condition check for the add-in. Exit 2 means a proven policy block.
import {readFile, writeFile, realpath} from 'node:fs/promises';
import {parseArgs} from 'node:util';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import {readCatalogJson} from '../viewer/protected-json.mjs';
import {resolveToolingRules} from '../viewer/tooling-rules.mjs';
import {diagnosePairedClosure} from '../viewer/sequence-paired-closure.mjs';

try {
  const {values} = parseArgs({options: Object.fromEntries(
    ['model', 'catalog', 'options', 'output'].map(key => [key, {type: 'string'}]))});
  if (!values.model || !values.options || !values.output) throw new Error('Expected --model, --options and --output');
  const catalogPath = values.catalog ?? fileURLToPath(new URL('../viewer/tooling.json', import.meta.url));
  const canonical = async p => {
    let name = await realpath(resolve(p)).catch(() => resolve(p));
    return process.platform === 'win32' ? name.toLowerCase() : name;
  };
  const out = await canonical(values.output);
  for (const path of [values.model, values.options, catalogPath, catalogPath + '.bct']) {
    if (out === await canonical(path)) throw new Error('Output must not replace an input file');
  }
  const [model, options, catalog] = await Promise.all([
    readFile(values.model, 'utf8').then(JSON.parse), readFile(values.options, 'utf8').then(JSON.parse), readCatalogJson(catalogPath)]);
  const {decision, blocked} = resolveToolingRules(model, catalog, options);
  let result = blocked ?? {status: 'CONTINUE', dieRuleDecision: decision ?? null};
  // A fixed/mixed mount may have a different punch height at the limiting
  // span. The analytic proof only covers the automatically generated uniform
  // banks. Retain all alternative models and all unfinished searches.
  if (!blocked && model.bends?.length > 1 && !options.seedSetups?.length &&
      !options.operationTemplates?.length && !options.toolingByBend) {
    const stop = Symbol('paired closure precheck budget'), until = performance.now() + 80;
    try {
      // Use the whole catalog plus explicit caller restrictions. Certifying
      // every such profile also certifies every narrower design-ranked pass.
      const proof = diagnosePairedClosure({model, catalog, options}, () => {
        if (performance.now() >= until) throw stop;
      });
      if (proof) result = {status: proof.status, reason: proof.reason, dieRuleDecision: decision ?? null, stages: [], transfers: [],
        allStagesSampleClear: false, completeGeometricRouteSampleClear: false, completeSequenceVerified: false,
        completeManufacturingVerified: false, diagnostics: {pairedClosure: proof, statesExpanded: 0,
          transitionsChecked: 0, endpointChecks: 0, foldChecks: 0}};
    } catch (err) {
      if (err !== stop) result.pairedClosureCheck = {status: 'INCONCLUSIVE', reason: String(err?.message ?? err)};
    }
  }
  if (result.status === 'CONTINUE' && options.probeRankedTerminal === true &&
      options.experimentalAdvancedSearch === false) {
    const {proveRankedTerminalBlock,rankedTerminalFingerprint} = await import('../viewer/sequence-ranked-terminal.mjs');
    const {EXPANDED_SEARCH_OPTIONS, ORIENTATION_SEARCH_OPTIONS} = await import('../viewer/sequence-portfolio.mjs');
    const angles = [...new Set([options.partialAngleDegrees ?? 30, ...(options.partialAnglesDegrees ?? []),
      ...EXPANDED_SEARCH_OPTIONS.partialAnglesDegrees, ...ORIENTATION_SEARCH_OPTIONS.partialAnglesDegrees])];
    const until = performance.now() + 5500, stop = Symbol('terminal proof budget');
    try {
      const proof = proveRankedTerminalBlock(model, catalog, options, angles, () => {
        if (performance.now() >= until) throw stop;
      });
      if (proof) result.rankedTerminalProof = {...proof,inputProofSha256:await rankedTerminalFingerprint(model,catalog,options,proof)};
    } catch (err) {
      if (err !== stop) result.rankedTerminalCheck = {status: 'INCONCLUSIVE', reason: String(err?.message ?? err)};
    }
  }
  await writeFile(values.output, JSON.stringify({schemaVersion: 1, units: 'mm', ...result}));
  process.exitCode = result.status === 'CONTINUE' ? 0 : 2;
} catch (error) {
  console.error(error.message);
  process.exitCode = 1;
}
