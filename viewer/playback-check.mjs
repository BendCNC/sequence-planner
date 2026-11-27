/** Pure policy for a sampled cycle preflight; never a continuous-motion certificate. */
export function playbackSetupKey(config) {
  const setup = {...config};
  delete setup.progress; // A cycle scan checks the whole 0..1 interval.
  return JSON.stringify(setup);
}

export function machinePlaybackDecision(result) {
  if (!result || !['INTERFERENCE', 'SAMPLE_CLEAR'].includes(result.status)
      || !Array.isArray(result.samples) || !result.samples.length)
    throw new Error('La comprobación previa no devolvió un ciclo válido.');
  const samples = result.samples.slice().sort((a, b) => a.progress - b.progress);
  for (const sample of samples) {
    if (!Number.isFinite(sample.progress) || sample.progress < 0 || sample.progress > 1
        || !['CLEAR', 'INTERFERENCE'].includes(sample.status) || !Array.isArray(sample.pairs)
        || sample.pairs.some(pair => !Array.isArray(pair) || pair.length !== 2 || pair.some(id => typeof id !== 'string')))
      throw new Error('La comprobación previa contiene una posición inválida.');
  }
  const first = samples.find(sample => sample.status === 'INTERFERENCE' || sample.pairs.length);
  if (first) return {action: 'STOP_AT_SAMPLE', sampleCount: samples.length,
    firstCollision: {progress: first.progress, pairs: first.pairs.map(pair => pair.slice())},
    continuousCollisionDetection: false};
  if (result.status !== 'SAMPLE_CLEAR' || samples.length < 2 || samples[0].progress !== 0 || samples.at(-1).progress !== 1
      || samples.some((sample, i) => i && sample.progress <= samples[i - 1].progress))
    throw new Error('La comprobación previa no cubre el ciclo completo.');
  return {action: 'PLAY_SAMPLED', sampleCount: samples.length, continuousCollisionDetection: false};
}
