"""Serve the simulator, import geometry and explicitly extract optional STEP attachments."""
from __future__ import annotations

import argparse
import importlib.util
import json
import os
import re
import shutil
import subprocess
import sys
import uuid
from urllib.parse import quote, unquote
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import threading
import webbrowser

ROOT = Path(__file__).resolve().parents[1]
JOBS = {}
IMPORT_LOCK = threading.Lock()
EXTRACTION_LOCKS = {}
EXTRACTION_LOCKS_GUARD = threading.Lock()
EMBEDDED_HELPER = None
EMBEDDED_HELPER_LOCK = threading.Lock()
MAX_UPLOAD_BYTES = 30 * 1024 * 1024
SIGN_CONVENTIONS = {'auto', 'directed-positive', 'direction-child'}
EMBEDDED_STEP_MODES = {'auto', 'ignore'}


class ImportFailure(ValueError):
    def __init__(self, code, message, **details):
        super().__init__(message)
        self.code, self.details = code, details


def subprocess_options(timeout):
    return dict(cwd=str(ROOT), capture_output=True, text=True, encoding='utf-8',
                errors='replace', timeout=timeout,
                creationflags=subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0,
                env={**os.environ, 'PYTHONUTF8': '1', 'PYTHONIOENCODING': 'utf-8',
                     'PYTHONDONTWRITEBYTECODE': '1'})


def bcbend_python():
    """Find a compatible installed Python; FreeCAD's Python is only a fallback runtime."""
    candidates = [os.environ.get('UNFOLD_BCBEND_PYTHON'), sys.executable,
                  shutil.which('python'), os.environ.get('UNFOLD_FREECAD_PYTHON'),
                  r'C:\Program Files\FreeCAD 1.1\bin\python.exe']
    checked, seen = [], set()
    probe = ('import json,shapely; '
             'assert hasattr(shapely,"constrained_delaunay_triangles") and hasattr(shapely,"set_precision"); '
             'print(json.dumps({"shapely":shapely.__version__}))')
    for candidate in candidates:
        if not candidate or candidate.lower() in seen:
            continue
        seen.add(candidate.lower())
        if not Path(candidate).is_file():
            checked.append({'python': candidate, 'reason': 'not-found'})
            continue
        try:
            result = subprocess.run([candidate, '-c', probe], **subprocess_options(12))
            if result.returncode == 0:
                return candidate
            checked.append({'python': candidate, 'reason': 'shapely-2.1-unavailable'})
        except (OSError, subprocess.TimeoutExpired) as error:
            checked.append({'python': candidate, 'reason': type(error).__name__})
    raise ImportFailure('bcbend-runtime-unavailable',
                        'No se encuentra Python con Shapely 2.1 o posterior. Configure UNFOLD_BCBEND_PYTHON con un entorno compatible. La importación .bcbend no requiere STEP ni FreeCAD.',
                        checked=checked)


def embedded_step_python():
    """Use CAD only when that interpreter can also validate the BEND graph."""
    candidate = os.environ.get('UNFOLD_FREECAD_PYTHON', r'C:\Program Files\FreeCAD 1.1\bin\python.exe')
    if Path(candidate).is_file():
        probe = ('import FreeCAD,Part,shapely; '
                 'assert hasattr(shapely,"constrained_delaunay_triangles") and hasattr(shapely,"set_precision")')
        try:
            if subprocess.run([candidate, '-c', probe], **subprocess_options(12)).returncode == 0:
                return candidate
        except (OSError, subprocess.TimeoutExpired):
            pass
    # The package importer records CAD unavailability and can still use the graph.
    return bcbend_python()


def last_structured_result(completed):
    for line in reversed((completed.stdout + '\n' + completed.stderr).splitlines()):
        try:
            result = json.loads(line)
            if isinstance(result, dict) and ('code' in result or 'status' in result):
                return result
        except ValueError:
            pass
    return {}


def finish_model_job(job, output):
    payload = json.loads(output.read_text(encoding='utf-8'))
    if not isinstance(payload, dict) or not isinstance(payload.get('panels'), list) or not isinstance(payload.get('bends'), list):
        raise ImportFailure('invalid-importer-output', 'El importador no produjo un modelo geométrico válido.')
    metadata = payload.get('metadata') if isinstance(payload.get('metadata'), dict) else {}
    result = dict(modelUrl='/' + output.relative_to(ROOT).as_posix(),
                  panelCount=len(payload['panels']), bendCount=len(payload['bends']),
                  warnings=payload.get('warnings', metadata.get('warnings', [])))
    for key in ('geometry', 'geometryQuality', 'geometryBasis', 'geometryAuthority', 'nominalGeometry',
                'geometryClass', 'independentFinalShapeVerified', 'sourceUsesStep', 'sourceKind',
                'signConvention', 'limitations', 'provenance', 'verification', 'validation', 'stepAssociation'):
        if key in payload or key in metadata:
            result[key] = payload.get(key, metadata.get(key))
    audit = output.with_suffix('.verification.json')
    if audit.is_file():
        result['verificationUrl'] = '/' + audit.relative_to(ROOT).as_posix()
    job.update(**result, status='done')


def run_import(job_id, source, output, kind, options, lock_acquired=False):
    job = JOBS[job_id]
    acquired = lock_acquired or IMPORT_LOCK.acquire(blocking=False)
    if not acquired:
        job.update(status='failed', errorCode='import-busy', error='Ya hay una importación en curso. Intente de nuevo al finalizar.')
        return
    try:
        if kind == 'step':
            runtime = os.environ.get('UNFOLD_FREECAD_PYTHON', r'C:\Program Files\FreeCAD 1.1\bin\python.exe')
            if not Path(runtime).is_file():
                raise ImportFailure('freecad-runtime-unavailable', 'No se encuentra Python de FreeCAD. Configure UNFOLD_FREECAD_PYTHON.')
            args = [runtime, str(ROOT/'engine/import_step.py'), '--step', str(source), '--output', str(output), '--k-factor', str(options['k_factor'])]
            if options['root_face']:
                args += ['--root-face', str(options['root_face'])]
        else:
            embedded_mode = options.get('embedded_step', 'auto')
            has_attachment = job.get('embeddedStep', {}).get('present') is True
            # An unavailable CAD runtime must not disable nominal BEND import.
            # The package importer reports the failed association and falls back.
            runtime = embedded_step_python() if has_attachment and embedded_mode == 'auto' else bcbend_python()
            importer = 'import_bend_package.py' if has_attachment else 'import_bcbend.py'
            args = [runtime, str(ROOT/'engine'/importer), '--bcbend', str(source), '--output', str(output),
                    '--sign-convention', options['sign_convention']]
            if has_attachment:
                args += ['--embedded-step', embedded_mode]
        completed = subprocess.run(args, **subprocess_options(240))
        (output.parent/'import.log').write_text(completed.stdout+'\n'+completed.stderr, encoding='utf-8')
        if completed.returncode or not output.exists():
            issue = last_structured_result(completed)
            lines = (completed.stderr or completed.stdout).strip().splitlines()
            fallback = lines[-1] if lines else 'No se pudo reconocer esta geometría como chapa desarrollable.'
            raise ImportFailure(issue.get('code', 'import-failed'), issue.get('message', issue.get('error', fallback)),
                                **(issue.get('details') if isinstance(issue.get('details'), dict) else {}))
        finish_model_job(job, output)
    except subprocess.TimeoutExpired:
        job.update(status='failed', errorCode='import-timeout', error='La importación superó cuatro minutos. Reduzca la complejidad de la pieza.')
    except Exception as error:
        result = dict(errorCode=getattr(error, 'code', 'import-failed'), error=str(error)[:1000])
        if getattr(error, 'details', None):
            result['errorDetails'] = error.details
        job.update(**result, status='failed')
    finally:
        IMPORT_LOCK.release()


def import_step(job_id, source, output, k_factor, root_face, lock_acquired=False):
    run_import(job_id, source, output, 'step', {'k_factor':k_factor, 'root_face':root_face}, lock_acquired)


def import_bcbend(job_id, source, output, sign_convention='auto', lock_acquired=False, embedded_step='auto'):
    run_import(job_id, source, output, 'bcbend', {'sign_convention':sign_convention, 'embedded_step':embedded_step}, lock_acquired)


def embedded_helper():
    """The attachment reader uses only the server's stdlib Python runtime."""
    global EMBEDDED_HELPER
    with EMBEDDED_HELPER_LOCK:
        if EMBEDDED_HELPER is None:
            spec = importlib.util.spec_from_file_location('unfold_embedded_step', ROOT/'engine/bcbend_embedded_step.py')
            module = importlib.util.module_from_spec(spec)
            sys.modules[spec.name] = module
            spec.loader.exec_module(module)
            EMBEDDED_HELPER = module
    return EMBEDDED_HELPER


def attachment_metadata(value):
    """Only small, explicitly selected metadata can enter a job response."""
    value = value if isinstance(value, dict) else {}
    result = {'present': value.get('present') is True}
    for key in ('fileName', 'sizeBytes', 'sha256', 'encoding', 'mediaType', 'format',
                'declaredSizeBytes', 'valid', 'validated', 'status', 'error', 'errorCode'):
        item = value.get(key)
        if item is None or isinstance(item, (bool, int, float)):
            if key in value:
                result[key] = item
        elif isinstance(item, str):
            result[key] = item[:300]
    return result


def validate_bcbend_bytes(payload, allow_prepared=False, source_name=None):
    try:
        parsed = embedded_helper().parse_bcbend_document(payload, max_bytes=MAX_UPLOAD_BYTES, source_name=source_name)
        data = parsed['document']
    except (ValueError, UnicodeError, RecursionError) as error:
        raise ImportFailure('invalid-bcbend-json', 'El archivo no contiene JSON .bcbend válido: ' + str(error)[:250]) from error
    if isinstance(data, dict) and isinstance(data.get('sheetMetalGraph'), dict) and ('bends' not in data or isinstance(data['bends'], list)):
        kind = 'bcbend'
    elif allow_prepared and isinstance(data, dict) and 'sheetMetalGraph' not in data and isinstance(data.get('panels'), list) and isinstance(data.get('bends'), list):
        kind = 'prepared-json'
    else:
        raise ImportFailure('unsupported-bcbend-schema', 'Se necesita sheetMetalGraph para .bcbend, o panels y bends para un modelo preparado enviado a /api/import-json.')
    return {'document': data, 'embeddedStep': attachment_metadata(parsed.get('embeddedStep')), 'kind': kind}


class Handler(SimpleHTTPRequestHandler):
    extensions_map = {**SimpleHTTPRequestHandler.extensions_map, '.mjs': 'text/javascript'}

    def end_headers(self):
        self.send_header('Cache-Control', 'no-store')
        super().end_headers()

    def send_json(self, status, payload):
        data = json.dumps(payload, ensure_ascii=False).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path.startswith('/api/import-status/'):
            job = JOBS.get(self.path.rsplit('/', 1)[-1])
            return self.send_json(200 if job else 404, dict(job) if job else {'error':'Importación no encontrada.', 'errorCode':'import-not-found'})
        if self.path == '/api/examples':
            examples = [{'name':'Complex · STEP auditado', 'url':'/viewer/model.json'}]
            for file in sorted((ROOT/'viewer/examples').glob('*.json')):
                try:
                    data = json.loads(file.read_text(encoding='utf-8'))
                    if isinstance(data.get('panels'), list):
                        examples.append({'name':data.get('partName',file.stem), 'url':'/'+file.relative_to(ROOT).as_posix()})
                except (ValueError, OSError):
                    continue
            return self.send_json(200, examples)
        super().do_GET()

    def do_POST(self):
        extraction = self.path.startswith('/api/extract-embedded-step/')
        if not extraction and self.path not in ('/api/import-step', '/api/import-bcbend', '/api/import-json'):
            return self.send_json(404, {'error':'Ruta no encontrada.', 'errorCode':'unknown-route'})
        origin = self.headers.get('Origin')
        allowed = {f'http://127.0.0.1:{self.server.server_port}', f'http://localhost:{self.server.server_port}'}
        if origin and origin not in allowed:
            return self.send_json(403, {'error':'Origen no permitido.', 'errorCode':'origin-not-allowed'})
        if extraction:
            return self.extract_embedded_step()
        kind = 'step' if self.path == '/api/import-step' else 'bcbend'
        owns_lock = False
        try:
            if self.headers.get('Transfer-Encoding'):
                raise ImportFailure('unsupported-transfer-encoding', 'Envíe el archivo con Content-Length, sin Transfer-Encoding.')
            length = int(self.headers.get('Content-Length', '0'))
            if not 0 < length <= MAX_UPLOAD_BYTES:
                raise ImportFailure('invalid-upload-size', 'Seleccione un archivo de hasta 30 MB.')
            if kind == 'step':
                k = float(self.headers.get('X-K-Factor', '.44'))
                face = int(self.headers.get('X-Root-Face', '0'))
                if not .05 <= k <= .95 or face < 0:
                    raise ImportFailure('invalid-step-options', 'K o cara base fuera de rango.')
            else:
                sign = self.headers.get('X-Bend-Sign-Convention', 'auto').strip().lower()
                if sign not in SIGN_CONVENTIONS:
                    raise ImportFailure('invalid-sign-convention', 'La convención debe ser auto, directed-positive o direction-child.')
                embedded_mode = self.headers.get('X-Embedded-Step-Mode', 'auto').strip().lower()
                if embedded_mode not in EMBEDDED_STEP_MODES:
                    raise ImportFailure('invalid-embedded-step-mode', 'El modo STEP embebido debe ser auto o ignore.')
            payload = self.rfile.read(length)
            if len(payload) != length:
                raise ImportFailure('incomplete-upload', 'El archivo recibido está incompleto.')
            name = unquote(self.headers.get('X-File-Name', 'pieza.'+kind))[:200]
            if kind == 'step':
                if b'ISO-10303-21' not in payload[:4096]:
                    raise ImportFailure('invalid-step-format', 'El archivo no parece un STEP de texto válido.')
            else:
                parsed = validate_bcbend_bytes(payload, allow_prepared=self.path == '/api/import-json', source_name=name)
                kind = parsed['kind']
            owns_lock = IMPORT_LOCK.acquire(blocking=False)
            if not owns_lock:
                return self.send_json(409, {'error':'Ya hay una importación en curso. Intente de nuevo al finalizar.', 'errorCode':'import-busy'})
            job_id = uuid.uuid4().hex
            folder = ROOT/'.work/imports'/job_id
            folder.mkdir(parents=True)
            # The uploaded document is the sole raw archive, including any attachment.
            source, output = folder/('source.step' if kind == 'step' else 'source.bcbend'), folder/'model.json'
            source.write_bytes(payload)
            JOBS[job_id] = {'id':job_id,'status':'running','name':name,'importKind':kind}
            if kind != 'step':
                JOBS[job_id]['embeddedStep'] = parsed['embeddedStep']
            if kind == 'prepared-json':
                output.write_text(json.dumps(parsed['document'], ensure_ascii=False, allow_nan=False), encoding='utf-8')
                finish_model_job(JOBS[job_id], output)
                return self.send_json(200, dict(JOBS[job_id]))
            if kind == 'bcbend':
                JOBS[job_id]['requestedSignConvention'] = sign
                JOBS[job_id]['requestedEmbeddedStepMode'] = embedded_mode
            target = import_step if kind == 'step' else import_bcbend
            args = (job_id,source,output,k,face) if kind == 'step' else (job_id,source,output,sign)
            worker_options = {'lock_acquired':True}
            if kind == 'bcbend':
                worker_options['embedded_step'] = embedded_mode
            threading.Thread(target=target,args=args,kwargs=worker_options,daemon=True).start()
            owns_lock = False  # The worker now owns release, including every failure path.
            return self.send_json(202, dict(JOBS[job_id]))
        except (ValueError, OSError) as error:
            issue = {'error':str(error)[:1000], 'errorCode':getattr(error,'code','invalid-import-request')}
            if 'job_id' in locals() and job_id in JOBS and owns_lock:
                JOBS[job_id].update(status='failed', **issue)
            return self.send_json(400, issue)
        except RuntimeError as error:
            if 'job_id' in locals() and job_id in JOBS:
                JOBS[job_id].update(status='failed', errorCode='import-start-failed', error=str(error)[:1000])
            return self.send_json(500, {'error':'No se pudo iniciar la importación.', 'errorCode':'import-start-failed'})
        finally:
            if owns_lock:
                IMPORT_LOCK.release()

    def extract_embedded_step(self):
        """Explicit extraction from one registered upload; no caller-provided paths."""
        empty = dict(ok=False, path=None, sizeBytes=None, sha256=None, fileName=None, url=None)
        lock = None
        acquired = False
        try:
            if self.headers.get('Transfer-Encoding'):
                raise ImportFailure('unsupported-transfer-encoding', 'Envíe la solicitud sin Transfer-Encoding.')
            if int(self.headers.get('Content-Length', '0')) != 0:
                raise ImportFailure('unexpected-extraction-payload', 'La extracción sólo recibe el identificador de importación, sin cuerpo ni rutas.')
            job_id = self.path[len('/api/extract-embedded-step/'):]
            job = JOBS.get(job_id) if re.fullmatch(r'[0-9a-f]{32}', job_id) else None
            if not job:
                return self.send_json(404, {**empty, 'error':'Importación no encontrada.', 'errorCode':'import-not-found'})
            if job.get('importKind') not in ('bcbend', 'prepared-json'):
                raise ImportFailure('embedded-step-unavailable', 'Esta importación no contiene un documento con STEP embebido.')
            with EXTRACTION_LOCKS_GUARD:
                lock = EXTRACTION_LOCKS.setdefault(job_id, threading.Lock())
            acquired = lock.acquire(blocking=False)
            if not acquired:
                return self.send_json(409, {**empty, 'error':'La extracción de este STEP ya está en curso.', 'errorCode':'extraction-busy'})
            imports = (ROOT/'.work/imports').resolve()
            folder = imports/job_id
            source, destination = folder/'source.bcbend', folder/'step'
            if (folder.resolve().parent != imports or source.resolve().parent != folder.resolve()
                    or destination.resolve().parent != folder.resolve()):
                raise ImportFailure('invalid-extraction-path', 'La ubicación registrada para esta importación no es válida.')
            extracted = embedded_helper().extract_embedded_step(source, destination, source_name=job.get('name'), max_bytes=MAX_UPLOAD_BYTES)
            path = Path(extracted['path']).resolve()
            if path.parent != destination.resolve() or not path.is_file():
                raise ImportFailure('invalid-extraction-result', 'La extracción no produjo un archivo en la carpeta prevista.')
            result = {key: extracted[key] for key in ('sizeBytes', 'sha256')}
            result.update(ok=True, path=str(path), fileName=path.name,
                          url='/' + quote(path.relative_to(ROOT.resolve()).as_posix(), safe='/'), error=None)
            return self.send_json(200, result)
        except (ValueError, OSError) as error:
            return self.send_json(400, {**empty, 'error':str(error)[:1000], 'errorCode':getattr(error, 'code', 'embedded-step-extraction-failed')})
        finally:
            if acquired:
                lock.release()


def main():
    parser = argparse.ArgumentParser(description='Abrir el simulador local de plegado.')
    parser.add_argument('--port', type=int, default=8786)
    parser.add_argument('--no-browser', action='store_true')
    args = parser.parse_args()
    handler = partial(Handler, directory=str(ROOT))
    try:
        server = ThreadingHTTPServer(('127.0.0.1', args.port), handler)
    except OSError as error:
        raise SystemExit(f'No se pudo abrir el puerto {args.port}: {error}. Use --port con otro numero.')
    url = f'http://127.0.0.1:{server.server_port}/viewer/'
    print(f'Simulacion local: {url}', flush=True)
    print('Para cerrar el servidor: Ctrl+C.', flush=True)
    if not args.no_browser:
        threading.Timer(0.3, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
