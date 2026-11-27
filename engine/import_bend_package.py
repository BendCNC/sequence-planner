"""Import BEND geometry, optionally associating its identifiers with embedded STEP CAD.

The attachment is optional. Only an unambiguous geometric association promotes
CAD to authority; a failed association falls back to the existing nominal BEND
importer without guessing legacy bend signs. Extracted STEP bytes are temporary.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import sys
import tempfile

from bcbend_embedded_step import read_bcbend_document, read_bcbend_with_step, extract_embedded_step, EmbeddedStepError, BCBendJsonError
from import_bcbend import import_bcbend, ImportBCBendError
from import_step import import_model, ImportStepError
from step_bend_dimensions import measure_split_bends

ROOT = Path(__file__).resolve().parents[1]


class ImportBendPackageError(ValueError):
    def __init__(self, code, message, **details):
        super().__init__(message)
        self.code, self.details = code, details

    def as_dict(self):
        return {'status': 'error', 'code': self.code, 'error': str(self), 'message': str(self), 'details': self.details}


def require(condition, code, message, **details):
    if not condition:
        raise ImportBendPackageError(code, message, **details)


def material_parameters(document):
    """Validate the unit/material contract before asking a CAD kernel to unfold."""
    require(isinstance(document, dict), 'invalid-schema', 'Se esperaba un documento BEND.')
    graph = document.get('sheetMetalGraph')
    require(isinstance(graph, dict) and str(graph.get('schemaVersion')) in ('5', '5.0'),
            'unsupported-schema', 'Se requiere sheetMetalGraph versión 5.')
    units = graph.get('units')
    require(units in ('mm', 'in'), 'unsupported-units', 'Las unidades del BEND deben ser mm o in.')
    material = graph.get('material')
    require(isinstance(material, dict), 'invalid-material', 'Falta el material del BEND.')
    thickness = next((material[key] for key in ('thickness_'+units, 'thicknessMm' if units == 'mm' else 'thicknessIn', 'thickness') if key in material), None)
    require(isinstance(thickness, (int, float)) and not isinstance(thickness, bool) and math.isfinite(thickness),
            'invalid-thickness', 'El espesor del BEND debe ser un número finito.')
    thickness *= 25.4 if units == 'in' else 1.
    require(0 < thickness < 1000, 'invalid-thickness', 'Espesor uniforme fuera de rango.')
    k = material.get('kFactor')
    require(isinstance(k, (int, float)) and not isinstance(k, bool) and math.isfinite(k) and 0 <= k <= 1,
            'invalid-k-factor', 'El factor K del BEND debe estar entre 0 y 1.')
    edges = graph.get('bendEdges')
    require(isinstance(edges, list), 'invalid-tree-size', 'Faltan las relaciones de pliegue del BEND.')
    varied_k = False
    for edge in edges:
        require(isinstance(edge, dict), 'invalid-bend-edge', 'Cada relación de pliegue debe ser un objeto.')
        bend_k = edge.get('kFactor', k)
        require(isinstance(bend_k, (int, float)) and not isinstance(bend_k, bool) and math.isfinite(bend_k) and 0 <= bend_k <= 1,
                'invalid-k-factor', 'Un pliegue contiene un factor K inválido.', bendId=edge.get('bendId'))
        varied_k |= abs(bend_k-k) > 1e-12
    return {'units': units, 'thicknessMm': thickness, 'kFactor': float(k), 'hasPerBendK': varied_k}


def association_record(status, message, **details):
    return {'status': status, 'message': message, 'mappingCount': 0, **details}


def match_bend_to_step(document, cad_model):
    # Lazy import keeps ignore/absent paths independent of the matcher and CAD runtime.
    from associate_step_bend import associate
    return associate(document, cad_model)


def _atomic_json(path, document):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=path.parent,
                                         prefix='.'+path.name+'-', suffix='.tmp', delete=False) as stream:
            temporary = Path(stream.name)
            json.dump(document, stream, ensure_ascii=False, allow_nan=False, separators=(',', ':'))
        temporary.replace(path)
        temporary = None
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def import_bend_package(source, output=None, embedded_step='auto', sign_convention='auto', merge_split_hinges=True,
                        counter_tabs=None):
    """Return a model; optionally write model and verification JSON, never the source."""
    require(embedded_step in ('auto', 'ignore'), 'invalid-embedded-step-mode', 'El modo STEP debe ser auto o ignore.')
    require(sign_convention in ('auto', 'directed-positive', 'direction-child'),
            'unknown-sign-convention', 'Convención de signo no admitida.')
    source = Path(source).resolve()
    output = Path(output).resolve() if output is not None else None
    if output:
        require(source not in (output, output.with_suffix('.verification.json')),
                'output-overwrites-source', 'El destino no puede sobrescribir el BEND original.')
    encoded_step = None
    try:
        if embedded_step == 'auto':
            info, encoded_step = read_bcbend_with_step(source)
        else:
            info = read_bcbend_document(source)
    except BCBendJsonError as exc:
        raise ImportBendPackageError('invalid-json', str(exc)) from exc
    document, attachment = info['document'], info['embeddedStep']
    parameters = material_parameters(document)
    hashes = {'bendSha256': info['sourceSha256'], 'embeddedStepSha256': None}
    association = association_record('ignored' if embedded_step == 'ignore' else 'unavailable',
                                    'STEP embebido ignorado por solicitud explícita.' if embedded_step == 'ignore' else 'El BEND no contiene STEP embebido.')
    model = verification = step_dimensions = measured_shape = None
    if embedded_step == 'auto' and attachment.get('present'):
        try:
            temporary_parent = output.parent if output else ROOT/'.work'
            temporary_parent.mkdir(parents=True, exist_ok=True)
            with tempfile.TemporaryDirectory(prefix='.bend-step-', dir=temporary_parent) as temporary:
                try:
                    extracted = extract_embedded_step(source, Path(temporary)/'step', _source_info=(info, encoded_step))
                finally:
                    encoded_step = None
                hashes['embeddedStepSha256'] = extracted['sha256']
                if parameters['hasPerBendK']:
                    association = association_record('unavailable', 'El STEP no se asoció porque el BEND usa distintos factores K por pliegue; el adaptador CAD actual sólo admite un K uniforme.',
                                                     code='per-bend-k-not-supported-by-cad')
                else:
                    path = Path(extracted['path'])
                    try:
                        cad_model, cad_verification = import_model(path, k_factor=parameters['kFactor'])
                    except ImportStepError as exc:
                        graph = document['sheetMetalGraph']
                        if (exc.code != 'bend-panel-count-unsupported' or not merge_split_hinges or
                                not any(e.get('operationRelationCount',1)>1 for e in graph['bendEdges'])):
                            raise
                        import Part
                        measured_shape = Part.read(str(path))
                        step_dimensions = measure_split_bends(measured_shape,graph,parameters['thicknessMm'])
                        if not step_dimensions:
                            raise
                        matched = {'status':'dimensions-matched','report':{'bends':step_dimensions}}
                    else:
                        matched = match_bend_to_step(document, cad_model)
                    status = matched.get('status')
                    require(status in ('matched', 'ambiguous', 'mismatch','dimensions-matched'), 'invalid-matcher-output', 'El asociador devolvió un estado inválido.')
                    report = matched.get('report', {})
                    require(isinstance(report, dict), 'invalid-matcher-output', 'El asociador no produjo un informe válido.')
                    if status == 'matched':
                        require(isinstance(matched.get('model'), dict), 'invalid-matcher-output', 'La asociación no produjo un modelo.')
                        model, verification = copy.deepcopy(matched['model']), copy.deepcopy(cad_verification)
                        association = association_record('matched', 'El STEP coincide geométricamente con el BEND; se usa CAD conservando las identidades BEND.',
                                                         mappingCount=len(model['bends']), panelMappingCount=len(model['panels']), report=report)
                    elif status == 'dimensions-matched' and step_dimensions:
                        association = association_record(status, 'Ángulos y radios medidos en las dos pieles del STEP. Los contornos y las relaciones de la pieza proceden del BEND.',
                                                         mappingCount=len(step_dimensions),report=report)
                    else:
                        association = association_record(status,
                            'No se logró demostrar una correspondencia única'+(' dentro del límite de búsqueda' if report.get('budgetExhausted') else '')+'; se conserva la geometría nominal.' if status == 'ambiguous' else
                            'El STEP no coincide de forma suficiente con el BEND; se conserva la geometría nominal.', report=report)
        except EmbeddedStepError as exc:
            association = association_record('corrupt' if exc.code != 'embedded-step-absent' else 'unavailable', str(exc), code=exc.code)
        except ImportStepError as exc:
            association = association_record('unavailable', 'El STEP embebido no se pudo usar: '+str(exc), code=exc.code, details=exc.details)
        except Exception as exc:
            association = association_record('unavailable', 'No se pudo asociar el STEP embebido; se intenta la geometría nominal.',
                                             code=getattr(exc, 'code', 'embedded-step-association-failed'),
                                             details={'reason': str(exc)[:500], 'exceptionType': type(exc).__name__})

    if model is None:
        try:
            nominal_options = dict(sign_convention=sign_convention, merge_split_hinges=merge_split_hinges,
                                   counter_tabs=counter_tabs, _source_info=info)
            if step_dimensions:
                try:
                    model = import_bcbend(source, **nominal_options, step_dimensions=step_dimensions)
                except ImportBCBendError as exc:
                    association = association_record('mismatch', 'Las dimensiones del STEP no son compatibles con el grafo BEND.',
                                                     code=exc.code, details=exc.details)
            if model is None:
                model = import_bcbend(source, **nominal_options)
            verification = copy.deepcopy(model['metadata']['validation'])
        except ImportBCBendError as exc:
            details = {**exc.details, 'stepAssociation': association, 'sourceHashes': hashes}
            message = str(exc)
            if exc.code in ('ambiguous-bend-sign', 'ambiguous-sign'):
                message += ' No hay una asociación STEP válida que resuelva la forma. Reintente la reconstrucción nominal declarando la convención de signo.'
                details['retry'] = {'embeddedStep': 'ignore', 'signConventionRequired': True,
                                    'allowedSignConventions': ['directed-positive', 'direction-child']}
            raise ImportBendPackageError(exc.code, message, **details) from exc

    # A source modified while decoding/importing cannot be paired with old hashes.
    with source.open('rb') as stream:
        current_hash = hashlib.sha256(stream.read()).hexdigest()
    require(current_hash == info['sourceSha256'], 'source-changed-during-import', 'El BEND cambió durante la importación; vuelva a cargarlo.')
    metadata = model.setdefault('metadata', {})
    metadata.update(embeddedStep=attachment, stepAssociation=association, sourceHashes=hashes,
                    embeddedStepMode=embedded_step, requestedSignConvention=sign_convention,
                    kFactorSource='BEND sheetMetalGraph.material.kFactor (ANSI)')
    metadata['sourceSha256'] = {source.name: hashes['bendSha256']}
    if hashes['embeddedStepSha256']:
        metadata['sourceSha256']['embedded-step:'+attachment['fileName']] = hashes['embeddedStepSha256']
    if association['status'] == 'matched':
        metadata.update(sourceKind='bend-step-associated', sourceUsesStep=True, geometryAuthority='embedded-step',
                        independentFinalShapeVerified=True, signConvention='step-geometry',
                        partName=str(document.get('part', {}).get('name', source.stem)) if isinstance(document.get('part'), dict) else source.stem)
        metadata['associationNote'] = 'CAD rooting and directed relations are preserved; BEND identities are assigned only after a unique geometric association.'
        verification.update(status='step-associated-geometry-verified', sourceKind='bend-step-associated',
                            sourceUsesStep=True, independentFinalShapeVerified=True,
                            kFactorSource='BEND sheetMetalGraph.material.kFactor (ANSI)',
                            cadChecksIdentitySpace='CAD identifiers before BEND association')
    elif association['status'] == 'dimensions-matched':
        metadata.update(sourceUsesStep=True, geometryAuthority='bend-contours-step-bend-dimensions')
        verification.update(sourceUsesStep=True,status='nominal-geometry-with-step-dimensions')
        metadata['validation'].update(sourceUsesStep=True,status=verification['status'])
        try:
            from folded_step_check import compare_folded_step
            audit, mesh = compare_folded_step(measured_shape, model, document['sheetMetalGraph'], step_dimensions)
            if mesh:
                model['stepReferenceMesh'] = mesh
        except Exception as exc:
            audit = {'status':'unavailable','message':str(exc)[:300]}
        verification['foldedStepCheck'] = metadata['foldedStepCheck'] = audit
        # The embedded STEP is the final-shape authority: a bend folded the wrong way
        # (or with the wrong angle) moves its flange far from the STEP surfaces.
        limit = max(1.0, parameters['thicknessMm'])
        require(audit.get('status') != 'sampled' or audit['maxMm'] <= limit, 'folded-step-mismatch',
                'La forma plegada no coincide con el STEP embebido (desviación %.2f mm > %.2f mm); revise el sentido de los pliegues.'
                % (audit.get('maxMm', 0), limit),
                worstPanels=sorted(audit.get('modelTriangleCentroidMaxMm', {}).items(), key=lambda kv: -kv[1])[:3])
    elif embedded_step == 'auto' and attachment.get('present'):
        warning = {'code': 'embedded-step-'+association['status'], 'message': association['message'], 'associationCode': association.get('code')}
        metadata.setdefault('warnings', []).append(warning)
        verification.setdefault('warnings', []).append(copy.deepcopy(warning))
    verification.update(stepAssociation=association, sourceHashes=hashes, embeddedStep=attachment)
    if output:
        _atomic_json(output, model)
        _atomic_json(output.with_suffix('.verification.json'), verification)
    return model


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bcbend', '--bend', dest='bcbend', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--embedded-step', choices=['auto', 'ignore'], default='auto')
    parser.add_argument('--sign-convention', choices=['auto', 'directed-positive', 'direction-child'], default='auto')
    parser.add_argument('--merge-split-hinges', action=argparse.BooleanOptionalAction, default=True, help='fold compatible coaxial spans as one stroke (default)')
    parser.add_argument('--counter-tabs', choices=['folded', 'parent'], default=None,
                        help="counter-span tabs ride with the folded span or stay with the stroke parent (default: the graph's counterTabs, else folded)")
    args = parser.parse_args(argv)
    try:
        model = import_bend_package(args.bcbend, args.output, args.embedded_step, args.sign_convention, args.merge_split_hinges,
                                    args.counter_tabs)
        print(json.dumps({'status': 'ok', 'output': str(args.output.resolve()),
                          'verification': str(args.output.with_suffix('.verification.json').resolve()),
                          'panels': len(model['panels']), 'bends': len(model['bends']), 'thicknessMm': model['thicknessMm'],
                          'stepAssociation': model['metadata']['stepAssociation'], 'warnings': model['metadata'].get('warnings', [])}, ensure_ascii=False))
        return 0
    except (ImportBendPackageError, ImportBCBendError) as exc:
        print(json.dumps(exc.as_dict(), ensure_ascii=False), file=sys.stderr)
        return 2
    except Exception as exc:
        print(json.dumps({'status': 'error', 'code': 'invalid-bend-package', 'message': str(exc), 'error': str(exc)}, ensure_ascii=False), file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
