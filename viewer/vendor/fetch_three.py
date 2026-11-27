"""Vendor only the files used by this local CAD viewer, from the npm package."""
import hashlib
import io
import json
import tarfile
import urllib.request
from pathlib import Path

VERSION = '0.180.0'
ROOT = Path(__file__).resolve().parent
url = f'https://registry.npmjs.org/three/-/three-{VERSION}.tgz'
payload = urllib.request.urlopen(url, timeout=30).read()
files = {
    'package/build/three.module.js': 'three.module.js',
    'package/build/three.core.js': 'three.core.js',
    'package/examples/jsm/controls/OrbitControls.js': 'OrbitControls.js',
    'package/LICENSE': 'THREE-LICENSE.txt',
}
with tarfile.open(fileobj=io.BytesIO(payload), mode='r:gz') as archive:
    for member, output in files.items():
        (ROOT / output).write_bytes(archive.extractfile(member).read())
(ROOT / 'manifest.json').write_text(json.dumps({'package': 'three', 'version': VERSION,
    'source': url, 'archiveSHA256': hashlib.sha256(payload).hexdigest(),
    'files': list(files.values())}, indent=2))
print('Vendored Three.js', VERSION)
