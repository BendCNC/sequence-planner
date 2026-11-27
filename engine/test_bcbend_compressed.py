import hashlib
import io
import json
import struct
import tempfile
import unittest
import zlib
from pathlib import Path

from bcbend_embedded_step import BCBendJsonError, _read, parse_bcbend_document, read_bcbend_document


def compressed(text: bytes) -> bytes:
    return b'BCZ1' + struct.pack('>I', len(text)) + zlib.compress(text, 6)


class CompressedBendTest(unittest.TestCase):
    DOC = {'part': {'name': 'x'}, 'bends': [{'id': 1, 'angle': 90.5}], 'note': 'ñ'}

    def test_compressed_and_plain_give_same_document(self):
        text = json.dumps(self.DOC, ensure_ascii=False).encode('utf-8')
        plain = parse_bcbend_document(text)
        packed = compressed(text)
        result = parse_bcbend_document(packed)
        self.assertEqual(result['document'], plain['document'])
        self.assertTrue(result['compressed'])
        self.assertNotIn('compressed', plain)
        # Hash and size describe the bytes on disk.
        self.assertEqual(result['sourceSha256'], hashlib.sha256(packed).hexdigest())
        self.assertEqual(result['sizeBytes'], len(packed))
        self.assertEqual(plain['sourceSha256'], hashlib.sha256(text).hexdigest())

    def test_file_path_reader(self):
        text = json.dumps(self.DOC).encode('utf-8')
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / 'a.bend'
            path.write_bytes(compressed(text))
            self.assertEqual(read_bcbend_document(path)['document'], self.DOC)
            path.write_bytes(text)
            self.assertEqual(read_bcbend_document(path)['document'], self.DOC)

    def test_explicit_step_reader_matches_streamed_metadata_and_payload(self):
        cases = [br'{"note":"\u00f1","stepModel":{"data":"c3RlcA==","sizeBytes":4}}',
                 br'{"step\u004dodel":{"data":"ab\/c","unknown":[1,2]}}',
                 b'{"stepModel":{"data":"first","data":"last"}}',
                 b'{"stepModel":{"data":2,"sizeBytes":1e999}}',
                 b'{"stepModel":null}', b'{"stepModel":{"data":"", "source":{}}}']
        for raw in cases:
            with self.subTest(raw=raw):
                def extracted(data):
                    result, reader = _read(io.BytesIO(data), 30*1024*1024, 'sample.bend', True)
                    return result['document'], result['embeddedStep'], reader.data
                self.assertEqual(extracted(compressed(raw)), extracted(raw))

    def test_native_geometry_reader_keeps_streamed_limits_and_errors(self):
        invalid = [b'{"x":1,"x":2}', b'{"x":1e999}', b'{"x":NaN}', b'{"x":-' + b'9'*1024 + b'}',
                   b'{"' + b'a'*4097 + b'":0}', b'['*129 + b'0' + b']'*129]
        for text in invalid:
            with self.subTest(text=text[:25]):
                with self.assertRaises(BCBendJsonError) as expected:
                    parse_bcbend_document(text)
                with self.assertRaises(BCBendJsonError) as actual:
                    parse_bcbend_document(compressed(text))
                self.assertEqual(str(actual.exception), str(expected.exception))
        for text in [b'['*128 + b'0' + b']'*128, b'{"' + b'a'*4096 + b'":0}',
                     b'{"steps":[{"x":-0.0,"y":1e-3}]}', br'{"step\u004dodel":{"data":"abc"}}',
                     b'{"stepModel":{"data":"abc"}}']:
            with self.subTest(text=text[:25]):
                normal, packed = parse_bcbend_document(text), parse_bcbend_document(compressed(text))
                self.assertEqual(packed['document'], normal['document'])
                self.assertEqual(packed['embeddedStep'], normal['embeddedStep'])

    def test_damaged_or_oversized_rejected(self):
        text = json.dumps(self.DOC).encode('utf-8')
        packed = compressed(text)
        for bad in (packed[:-3], packed[:6], b'BCZ1' + struct.pack('>I', len(text) + 1) + packed[8:],
                    packed + b'x'):
            with self.assertRaises(BCBendJsonError):
                parse_bcbend_document(bad)
        with self.assertRaises(BCBendJsonError):
            parse_bcbend_document(compressed(b'{"a":"' + b'0' * 5000 + b'"}'), max_bytes=1000)

    def test_decode_uses_bounded_file_reads_for_small_compressed_parts(self):
        class MeasuredStream(io.BytesIO):
            def read(self, n=-1):
                self.largest_read = max(getattr(self, 'largest_read', 0), n)
                return super().read(n)

        stream = MeasuredStream(compressed(b'{"piece":"small"}'))
        info, _ = _read(stream, 30*1024*1024, 'small.bend')
        self.assertEqual(info['document'], {'piece':'small'})
        self.assertLessEqual(stream.largest_read, 65536)


if __name__ == '__main__':
    unittest.main()
