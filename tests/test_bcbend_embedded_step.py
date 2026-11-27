"""Standard-library-only tests of streaming metadata and safe on-demand extraction."""
import base64
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile
import tracemalloc
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'engine'))
import bcbend_embedded_step as embedded

PAYLOAD = b'ISO-10303-21;\nHEADER;ENDSEC;\nDATA;ENDSEC;\nEND-ISO-10303-21;\n'


def envelope(payload=PAYLOAD, **overrides):
    return {'fileName': 'part.step', 'mediaType': 'model/step', 'encoding': 'base64', 'source': 'component',
            'sizeBytes': len(payload), 'sha256': hashlib.sha256(payload).hexdigest(),
            'data': base64.b64encode(payload).decode(), **overrides}


class EmbeddedStepTests(unittest.TestCase):
    def setUp(self):
        root = ROOT/'.work/test-embedded-step'; root.mkdir(parents=True, exist_ok=True)
        self.temp = tempfile.TemporaryDirectory(dir=root)
        self.directory = Path(self.temp.name)
        self.source = self.directory/'source.bcbend'
        self.out = self.directory/'not-created-until-valid'

    def tearDown(self): self.temp.cleanup()

    def write(self, step):
        data = {'sheetMetalGraph': {'schemaVersion': 5, 'arbitrary': ['héllo', 1, None]}, 'keep': 'unchanged'}
        if step is not None: data['stepModel'] = step
        raw = json.dumps(data, ensure_ascii=False).encode()
        self.source.write_bytes(raw)
        return raw

    def test_absent_legacy_document_still_reads(self):
        raw = self.write(None); read = embedded.read_bcbend_document(self.source)
        self.assertEqual(read['document'], json.loads(raw))
        self.assertEqual(read['embeddedStep'], {'present': False, 'status': 'absent'})
        self.assertEqual(read['sourceSha256'], hashlib.sha256(raw).hexdigest())
        with self.assertRaises(embedded.EmbeddedStepError) as caught:
            embedded.extract_embedded_step(self.source, self.out)
        self.assertEqual(caught.exception.code, 'embedded-step-absent')
        self.assertFalse(self.out.exists())

    def test_valid_reader_skips_payload_and_retains_original_hash(self):
        raw = self.write(envelope()); read = embedded.read_bcbend_document(self.source)
        self.assertNotIn('stepModel', read['document'])
        self.assertNotIn('data', read['embeddedStep'])
        self.assertEqual(read['embeddedStep']['status'], 'unverified')
        self.assertEqual(read['sizeBytes'], len(raw))
        self.assertEqual(read['sourceSha256'], hashlib.sha256(raw).hexdigest())
        self.assertEqual(read, embedded.read_bcbend_bytes(raw))
        self.assertFalse(self.out.exists())

    def test_valid_extract_only_after_verification(self):
        self.write(envelope()); result = embedded.extract_embedded_step(self.source, self.out)
        self.assertEqual(Path(result['path']).read_bytes(), PAYLOAD)
        self.assertEqual(result['sha256'], hashlib.sha256(PAYLOAD).hexdigest())
        self.assertEqual(result['fileName'], 'part.step')
        self.assertEqual(list(self.out.glob('*.tmp')), [])

    def test_corrupt_payload_does_not_invalidate_document_or_create_destination(self):
        cases = [dict(data='%%%%'), dict(data=base64.b64encode(PAYLOAD).decode()+'===='),
                 dict(data=base64.b64encode(PAYLOAD).decode()+'\n'), dict(data='AB==',sizeBytes=1,sha256=hashlib.sha256(b'\0').hexdigest()),
                 dict(sizeBytes=len(PAYLOAD)+1), dict(sha256='0'*64), dict(sha256=hashlib.sha256(PAYLOAD).hexdigest().upper()),
                 dict(encoding='hex'), dict(mediaType='application/pdf'), dict(sizeBytes=True), dict(data=42),
                 dict(data={'bad': 'wrong type'}), dict(fileName='ignored.step',sha256=None)]
        for changes in cases:
            with self.subTest(changes={k:v for k,v in changes.items() if k!='data'}):
                raw = self.write(envelope(**changes))
                read = embedded.parse_bcbend_document(raw)
                self.assertIn('sheetMetalGraph', read['document'])
                self.assertNotIn('stepModel', read['document'])
                with self.assertRaisesRegex(embedded.EmbeddedStepError, '^STEP embebido corrupto$'):
                    embedded.extract_embedded_step(self.source, self.out)
                self.assertFalse(self.out.exists())

    def test_semantically_bad_metadata_is_bounded_and_nonblocking(self):
        for value in [None, [], 'invalid', {'data': 'not decoded'}, envelope(fileName='X'*100000, encoding='X'*100000)]:
            raw = json.dumps({'sheetMetalGraph': {'schemaVersion': 5}, 'stepModel': value}).encode()
            result = embedded.parse_bcbend_document(raw)
            self.assertEqual(result['embeddedStep']['status'], 'invalid')
            self.assertLess(len(json.dumps(result['embeddedStep'])), 1024)
        raw = b'{"sheetMetalGraph":{"schemaVersion":5},"stepModel":{"sizeBytes":1e999,"data":"bad"}}'
        self.assertEqual(embedded.parse_bcbend_document(raw)['embeddedStep']['status'], 'invalid')
        raw = b'{"sheetMetalGraph":{"schemaVersion":5},"stepModel":{"sizeBytes":'+b'9'*10000+b',"data":"bad"}}'
        self.assertEqual(embedded.parse_bcbend_document(raw)['embeddedStep']['status'], 'invalid')

    def test_strict_json_outside_embedded_metadata(self):
        for raw in [b'{"x":1,"x":2}', b'{"x":NaN}', b'{"x":Infinity}', b'{"x":1e999}', b'{"x":01}',
                    b'{"x":"\xff"}', b'{"x":"\\q"}', b'{"x":1}garbage', b'{"x":[1,]}']:
            with self.subTest(raw=raw):
                with self.assertRaises(embedded.BCBendJsonError): embedded.parse_bcbend_document(raw)

    def test_cap_applies_to_complete_original_input(self):
        raw = self.write(envelope())
        with self.assertRaises(embedded.BCBendJsonError): embedded.parse_bcbend_document(raw, len(raw)-1)
        with self.assertRaises(embedded.BCBendJsonError): embedded.read_bcbend_document(self.source, len(raw)-1)
        self.assertFalse(self.out.exists())

    def test_normal_streaming_read_does_not_materialize_eight_megabyte_data(self):
        # Build the input before tracking memory. The JSON graph is deliberately
        # tiny so a full parsed base64 string would be observable in the peak.
        self.source.write_bytes(b'{"sheetMetalGraph":{"schemaVersion":5},"stepModel":{"data":"'+b'A'*(8*1024*1024)+b'"}}')
        tracemalloc.start()
        result = embedded.read_bcbend_document(self.source)
        _, peak = tracemalloc.get_traced_memory(); tracemalloc.stop()
        self.assertLess(peak, 2*1024*1024)
        self.assertNotIn('stepModel', result['document'])

    def test_safe_basename_for_both_separators_and_display_name_fallback(self):
        for index, (name, expected) in enumerate([('C:\\outside\\other.step','other.step'), ('../../outside/other.step','other.step'),
                               ('..','complex2.step'), ('bad..name.step','complex2.step'), ('CON.step','complex2.step'),
                               ('bad:name.step','complex2.step'), ('program.exe','complex2.step'), ('trailing.step ','complex2.step')]):
            with self.subTest(name=name):
                self.write(envelope(fileName=name))
                folder = self.directory/str(index)
                result = embedded.extract_embedded_step(self.source, folder, source_name='complex2.bcbend')
                self.assertEqual(result['fileName'], expected)
                self.assertEqual(Path(result['path']).parent, folder)

    def test_existing_files_and_concurrent_calls_are_never_overwritten(self):
        self.write(envelope()); self.out.mkdir()
        original = self.out/'part.step'; original.write_bytes(b'KEEP')
        with ThreadPoolExecutor(max_workers=8) as pool:
            results = list(pool.map(lambda _: embedded.extract_embedded_step(self.source, self.out), range(8)))
        self.assertEqual(original.read_bytes(), b'KEEP')
        self.assertEqual(len({r['path'] for r in results}), 8)
        self.assertTrue(all(Path(r['path']).read_bytes() == PAYLOAD for r in results))
        self.assertEqual(len(list(self.out.iterdir())), 9)

    def test_existing_symlink_is_not_followed_or_replaced(self):
        self.write(envelope()); self.out.mkdir()
        sentinel = self.directory/'sentinel.step'; sentinel.write_bytes(b'KEEP')
        link = self.out/'part.step'
        try: link.symlink_to(sentinel)
        except OSError as exc: self.skipTest('Symlink creation unavailable on this host: '+str(exc))
        result = embedded.extract_embedded_step(self.source, self.out)
        self.assertEqual(result['fileName'], 'part-1.step')
        self.assertTrue(link.is_symlink()); self.assertEqual(sentinel.read_bytes(), b'KEEP')

    def test_publish_failure_cleans_same_directory_temporary(self):
        self.write(envelope())
        publish = 'rename' if os.name == 'nt' else 'link'
        def fail(source, target):
            self.assertEqual(Path(source).parent, Path(target).parent)
            self.assertEqual(Path(source).read_bytes(), PAYLOAD)
            self.assertFalse(Path(target).exists())
            raise OSError('simulated publish failure')
        with mock.patch.object(embedded.os, publish, side_effect=fail):
            with self.assertRaisesRegex(OSError, 'simulated publish failure'):
                embedded.extract_embedded_step(self.source, self.out)
        self.assertEqual(list(self.out.iterdir()), [])


if __name__ == '__main__': unittest.main(verbosity=2)
