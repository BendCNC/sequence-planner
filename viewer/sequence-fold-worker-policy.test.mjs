import test from 'node:test';
import assert from 'node:assert/strict';
import {defaultFoldWorkerCount} from './sequence-fold-pool.mjs';

const MiB = 1024 * 1024;

test('four available CPUs cap only a single planner with enough memory for three workers', () => {
  for (const cpus of [1, 2, 3, 4, 5, 8, 16, 32]) {
    for (const processes of [1, 2, 3, 4, 5, 8]) {
      for (const freeMiB of [0, 319, 320, 639, 640, 959, 960, 2048, 8192]) {
        const cpuShare = Math.floor((cpus - 1) / processes);
        const memoryShare = Math.floor(freeMiB / 320);
        const previous = Math.min(cpuShare, memoryShare);
        const expected = previous < 2 ? 0 : previous;
        const count = defaultFoldWorkerCount({cpus, processes, free: freeMiB * MiB});
        assert.equal(count, cpus === 4 && expected === 3 ? 2 : expected,
          JSON.stringify({cpus, processes, freeMiB}));
      }
    }
  }
});

test('explicit worker limits still take precedence over CPU and process sharing', async () => {
  const previousLimit = process.env.UNFOLD_FOLD_WORKERS;
  const previousProcesses = process.env.UNFOLD_PLANNER_PROCESSES;
  try {
    process.env.UNFOLD_PLANNER_PROCESSES = '8';
    for (const limit of [0, 2, 3, 4]) {
      process.env.UNFOLD_FOLD_WORKERS = String(limit);
      const pool = await import(`./sequence-fold-pool.mjs?explicit-worker-limit=${limit}`);
      assert.equal(pool.foldWorkerCapacity, limit);
    }
  } finally {
    if (previousLimit === undefined) delete process.env.UNFOLD_FOLD_WORKERS;
    else process.env.UNFOLD_FOLD_WORKERS = previousLimit;
    if (previousProcesses === undefined) delete process.env.UNFOLD_PLANNER_PROCESSES;
    else process.env.UNFOLD_PLANNER_PROCESSES = previousProcesses;
  }
});
