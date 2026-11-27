"""Bounded BCBEND JSON reading and optional verified STEP extraction.

The normal reader never builds a Python string for stepModel.data. The source hash
still covers every original byte. Extraction is explicit and validates the complete
decoded payload before creating a directory or temporary output file.
"""
from __future__ import annotations

import base64
import binascii
import codecs
import hashlib
import io
import json
import math
import os
from pathlib import Path
import re
import tempfile
import zlib

MAX_BYTES = 30*1024*1024
_SPECIAL = re.compile(r'["\\\x00-\x1f]')
_NUMBER = re.compile(r'-?(?:0|[1-9][0-9]*)(?:\.[0-9]+)?(?:[eE][+-]?[0-9]+)?\Z')
_DELIMITED_NUMBER = re.compile(r'-?(?:0|[1-9][0-9]*)(?:\.[0-9]+)?(?:[eE][+-]?[0-9]+)?(?=[,\]} \t\r\n])')
_SHA = re.compile(r'[0-9a-f]{64}\Z')
_META_KEYS = {'fileName', 'mediaType', 'encoding', 'source', 'sizeBytes', 'sha256'}
_OVERSIZE = object()
_USE_STREAM_READER = object()


class EmbeddedStepError(ValueError):
    def __init__(self, code='embedded-step-corrupt'):
        self.code = code
        super().__init__('El archivo no contiene STEP embebido' if code == 'embedded-step-absent' else 'STEP embebido corrupto')


class BCBendJsonError(ValueError):
    code = 'invalid-json'


class _Reader:
    def __init__(self, stream, max_bytes, capture_step=False):
        if not isinstance(max_bytes, int) or isinstance(max_bytes, bool) or max_bytes < 1:
            raise ValueError('max_bytes must be a positive integer')
        self.stream, self.max_bytes, self.capture_step = stream, max_bytes, capture_step
        self.decoder = codecs.getincrementaldecoder('utf-8-sig')('strict')
        self.buffer, self.pos, self.eof = '', 0, False
        self.hash, self.size = hashlib.sha256(), 0
        self.present, self.meta, self.data, self.has_data, self.invalid_embedded = False, {}, None, False, False

    def fill(self):
        while self.pos >= len(self.buffer) and not self.eof:
            raw = self.stream.read(65536)
            self.size += len(raw)
            if self.size > self.max_bytes:
                raise BCBendJsonError('El archivo supera el límite de tamaño.')
            self.hash.update(raw)
            try: self.buffer = self.decoder.decode(raw, final=not raw)
            except UnicodeDecodeError as exc: raise BCBendJsonError('El archivo no contiene UTF-8 válido.') from exc
            self.pos = 0
            if not raw: self.eof = True

    def peek(self):
        self.fill()
        return self.buffer[self.pos] if self.pos < len(self.buffer) else ''

    def take(self):
        c = self.peek()
        if not c: raise BCBendJsonError('JSON incompleto.')
        self.pos += 1
        return c

    def whitespace(self):
        while self.peek() and self.peek() in ' \t\r\n': self.pos += 1

    def expect(self, c):
        if self.take() != c: raise BCBendJsonError('JSON inválido.')

    def string(self, keep=True, limit=None):
        self.expect('"')
        self.fill()
        match = _SPECIAL.search(self.buffer, self.pos)
        if match and match.group() == '"':
            start, end = self.pos, match.start()
            self.pos = end + 1
            if not keep:
                return None
            return _OVERSIZE if limit is not None and end - start > limit else self.buffer[start:end]
        tokens = ['"'] if keep else None
        captured, oversized = 0, False

        def append(text):
            nonlocal captured, oversized, tokens
            if tokens is None: return
            captured += len(text)
            if limit is not None and captured > limit:
                oversized = True; tokens = None
            else: tokens.append(text)

        while True:
            self.fill()
            if self.pos >= len(self.buffer): raise BCBendJsonError('Cadena JSON incompleta.')
            match = _SPECIAL.search(self.buffer, self.pos)
            end = match.start() if match else len(self.buffer)
            if tokens is not None: append(self.buffer[self.pos:end])
            self.pos = end
            if match is None: continue
            c = self.take()
            if c == '"':
                if oversized: return _OVERSIZE
                if tokens is None: return None
                tokens.append('"')
                return json.loads(''.join(tokens))
            if c != '\\': raise BCBendJsonError('Carácter de control en cadena JSON.')
            escape = self.take()
            if escape in '"\\/bfnrt': append('\\'+escape)
            elif escape == 'u':
                digits = ''.join(self.take() for _ in range(4))
                if not re.fullmatch('[0-9A-Fa-f]{4}', digits): raise BCBendJsonError('Escape Unicode inválido.')
                append('\\u'+digits)
            else: raise BCBendJsonError('Escape JSON inválido.')

    def number(self, keep, allow_nonfinite=False):
        # The common case is a finite coordinate entirely inside the current
        # UTF-8 chunk. The delimiter proves we have the whole JSON token.
        match = _DELIMITED_NUMBER.match(self.buffer, self.pos)
        if match:
            self.pos = match.end()
            if not keep:
                return None
            if match.end() - match.start() > 1024:
                if allow_nonfinite: return _OVERSIZE
                raise BCBendJsonError('Número JSON fuera del límite.')
            return self.numeric_value(match.group(), allow_nonfinite)
        token, count = [], 0
        def consume():
            nonlocal count
            c = self.take(); count += 1
            if count <= 1024: token.append(c)
        def digits():
            if not self.peek() or self.peek() not in '0123456789': raise BCBendJsonError('Número JSON inválido.')
            while self.peek() and self.peek() in '0123456789': consume()
        if self.peek() == '-': consume()
        if self.peek() == '0':
            consume()
            if self.peek() and self.peek() in '0123456789': raise BCBendJsonError('Número JSON inválido.')
        else: digits()
        if self.peek() == '.': consume(); digits()
        if self.peek() and self.peek() in 'eE':
            consume()
            if self.peek() and self.peek() in '+-': consume()
            digits()
        if not keep: return None
        if count > 1024:
            if allow_nonfinite: return _OVERSIZE
            raise BCBendJsonError('Número JSON fuera del límite.')
        token = ''.join(token)
        if not _NUMBER.fullmatch(token): raise BCBendJsonError('Número JSON inválido.')
        return self.numeric_value(token, allow_nonfinite)

    @staticmethod
    def numeric_value(token, allow_nonfinite):
        value = float(token) if '.' in token or 'e' in token or 'E' in token else int(token)
        try: finite = math.isfinite(value)
        except OverflowError: finite = False
        if not finite:
            if allow_nonfinite: return _OVERSIZE
            raise BCBendJsonError('Número JSON no finito.')
        return value

    def value(self, depth=0, keep=True, strict=True):
        if depth > 128: raise BCBendJsonError('JSON demasiado anidado.')
        self.whitespace(); c = self.peek()
        if c == '"': return self.string(keep)
        if c and c in '-0123456789': return self.number(keep)
        if c == '{':
            self.take(); self.whitespace(); result = {} if keep else None; keys = set()
            if self.peek() == '}': self.take(); return result
            while True:
                self.whitespace()
                if self.peek() != '"': raise BCBendJsonError('Clave JSON inválida.')
                key = self.string(True, 4096)
                if key is _OVERSIZE: raise BCBendJsonError('Clave JSON demasiado larga.')
                if key in keys and strict: raise BCBendJsonError('Clave JSON duplicada: '+key[:80])
                keys.add(key); self.whitespace(); self.expect(':')
                if depth == 0 and key == 'stepModel':
                    self.embedded(depth+1)
                else:
                    item = self.value(depth+1, keep, strict)
                    if keep: result[key] = item
                self.whitespace(); separator = self.take()
                if separator == '}': return result
                if separator != ',': raise BCBendJsonError('Separador JSON inválido.')
        if c == '[':
            self.take(); self.whitespace(); result = [] if keep else None
            if self.peek() == ']': self.take(); return result
            while True:
                item = self.value(depth+1, keep, strict)
                if keep: result.append(item)
                self.whitespace(); separator = self.take()
                if separator == ']': return result
                if separator != ',': raise BCBendJsonError('Separador JSON inválido.')
        for literal, value in [('true', True), ('false', False), ('null', None)]:
            if c == literal[0]:
                for letter in literal: self.expect(letter)
                return value if keep else None
        raise BCBendJsonError('Valor JSON inválido.')

    def embedded(self, depth):
        self.present = True; self.whitespace()
        if self.peek() != '{':
            self.invalid_embedded = True; self.value(depth, False, False); return
        self.take(); self.whitespace(); keys = set()
        if self.peek() == '}': self.take(); return
        while True:
            self.whitespace()
            if self.peek() != '"': raise BCBendJsonError('Clave JSON inválida.')
            key = self.string(True, 4096)
            if key is _OVERSIZE: key = None; self.invalid_embedded = True
            if key in keys: self.invalid_embedded = True
            keys.add(key); self.whitespace(); self.expect(':'); self.whitespace()
            if key == 'data':
                self.has_data = True
                if self.peek() == '"': self.data = self.string(self.capture_step)
                else: self.invalid_embedded = True; self.value(depth+1, False, False)
            elif key in _META_KEYS:
                if self.peek() == '"': value = self.string(True, 4096)
                elif self.peek() in '{[': self.value(depth+1, False, False); value = _OVERSIZE
                elif self.peek() and self.peek() in '-0123456789': value = self.number(True, allow_nonfinite=True)
                else:
                    try: value = self.value(depth+1, True, False)
                    except BCBendJsonError:
                        # Invalid JSON syntax still fails; only semantically bad
                        # embedded metadata is tolerated, never a malformed file.
                        raise
                if value is _OVERSIZE: self.invalid_embedded = True; value = None
                self.meta[key] = value
            else: self.value(depth+1, False, False)
            self.whitespace(); separator = self.take()
            if separator == '}': return
            if separator != ',': raise BCBendJsonError('Separador JSON inválido.')

    def read(self):
        document = self.value(); self.whitespace()
        if self.peek(): raise BCBendJsonError('Contenido después del objeto JSON.')
        return document


def _safe_name(value, source_name):
    """Basename under either separator convention; never a path or device name."""
    reserved = {'CON', 'PRN', 'AUX', 'NUL', 'CONIN$', 'CONOUT$',
                *('COM'+str(i) for i in range(1, 10)), *('LPT'+str(i) for i in range(1, 10))}

    def valid(name):
        return bool(name and len(name) <= 120 and '..' not in name and name not in ['.', '..']
                    and name == name.rstrip(' .') and all(ord(c) >= 32 and ord(c) != 127 and not 0xD800 <= ord(c) <= 0xDFFF
                        and c not in '<>:"/\\|?*' for c in name)
                    and name.split('.')[0].upper() not in reserved)

    candidate = value.replace('\\', '/').rsplit('/', 1)[-1] if isinstance(value, str) else ''
    if valid(candidate) and Path(candidate).suffix.lower() in ['.step', '.stp']: return candidate
    original = str(source_name or 'bend').replace('\\', '/').rsplit('/', 1)[-1]
    stem = Path(original).stem
    stem = ''.join(c for c in stem if ord(c) >= 32 and ord(c) != 127 and not 0xD800 <= ord(c) <= 0xDFFF and c not in '<>:"/\\|?*').strip(' .')[:80]
    while '..' in stem: stem = stem.replace('..', '_')
    fallback = (stem or 'bend')+'.step'
    return fallback if valid(fallback) else 'bend.step'


def _metadata(reader, source_name):
    if not reader.present: return {'present': False, 'status': 'absent'}
    meta = reader.meta
    valid = (not reader.invalid_embedded and reader.has_data and meta.get('encoding') == 'base64'
             and meta.get('mediaType') == 'model/step' and isinstance(meta.get('sizeBytes'), int)
             and not isinstance(meta.get('sizeBytes'), bool) and 0 < meta['sizeBytes'] <= MAX_BYTES
             and isinstance(meta.get('sha256'), str) and bool(_SHA.fullmatch(meta['sha256'])))
    result = {'present': True, 'status': 'unverified' if valid else 'invalid',
              'fileName': _safe_name(meta.get('fileName'), source_name),
              'sizeBytes': meta.get('sizeBytes') if isinstance(meta.get('sizeBytes'), int) and not isinstance(meta.get('sizeBytes'), bool) and 0 <= meta['sizeBytes'] <= MAX_BYTES else None,
              'encoding': meta.get('encoding') if isinstance(meta.get('encoding'), str) and len(meta['encoding']) <= 32 else None,
              'mediaType': meta.get('mediaType') if isinstance(meta.get('mediaType'), str) and len(meta['mediaType']) <= 128 else None}
    if isinstance(meta.get('sha256'), str) and _SHA.fullmatch(meta['sha256']): result['sha256'] = meta['sha256']
    return result


# Compressed BEND container: b'BCZ1' + 4-byte big-endian length + zlib stream
# (Qt qCompress layout). Plain JSON text is still accepted.
COMPRESSED_MAGIC = b'BCZ1'


def _decompress(stream, max_bytes):
    # Allocating the full 30 MiB input limit for a small file raises the peak
    # on a board processing several parts. The magic has already been read.
    size, chunks = len(COMPRESSED_MAGIC), [COMPRESSED_MAGIC]
    while size <= max_bytes:
        chunk = stream.read(min(65536, max_bytes + 1 - size))
        if not chunk:
            break
        chunks.append(chunk)
        size += len(chunk)
    if size > max_bytes: raise BCBendJsonError('El archivo supera el límite de tamaño.')
    raw = b''.join(chunks)
    if len(raw) < 8: raise BCBendJsonError('El BEND comprimido está truncado.')
    expected = int.from_bytes(raw[4:8], 'big')
    if expected > max_bytes: raise BCBendJsonError('El archivo supera el límite de tamaño.')
    inflater = zlib.decompressobj()
    try:
        data = inflater.decompress(raw[8:], max_bytes + 1)
    except zlib.error as exc: raise BCBendJsonError('El BEND comprimido está dañado.') from exc
    if len(data) > max_bytes: raise BCBendJsonError('El archivo supera el límite de tamaño.')
    if not inflater.eof or inflater.unused_data or len(data) != expected:
        raise BCBendJsonError('El BEND comprimido está dañado.')
    return raw, data


def _small_document(data, reader):
    # The geometry-only reader must skip the STEP string. Explicit extraction
    # already needs that string, so modest compressed exports can use the C
    # JSON decoder, including exports that also contain a previous plan.
    limit = (2 if reader.capture_step else 1) * 1024*1024
    escaped = b'\\u' in data
    if len(data) > limit or not reader.capture_step and (b'"stepModel"' in data or escaped):
        return _USE_STREAM_READER

    def number(token):
        if len(token) > 1024:
            raise ValueError
        return _Reader.numeric_value(token, False)

    def mapping(pairs):
        obj = dict(pairs)
        # At most two source characters per decoded character with no Unicode
        # escapes. Longer keys can still be legal; let the streamed reader decide.
        if len(obj) != len(pairs) or any(len(k) > (320 if escaped else 2048) for k in obj):
            raise ValueError
        return obj

    def allowed_depth(value, depth):
        if isinstance(value, dict):
            children = value.values()
        elif isinstance(value, list):
            children = value
        else:
            return True
        if depth == 128:
            return not value
        return all(allowed_depth(child, depth+1) for child in children
                   if isinstance(child, (list, dict)))

    try:
        document = json.loads(data.decode('utf-8-sig'), object_pairs_hook=mapping,
                              parse_int=number, parse_float=number, parse_constant=number)
        if not allowed_depth(document, 0):
            return _USE_STREAM_READER
        if isinstance(document, dict) and 'stepModel' in document:
            step = document['stepModel']
            if (not reader.capture_step or not isinstance(step, dict)
                    or not isinstance(step.get('data'), str)
                    or not step.keys() <= _META_KEYS | {'data'}):
                return _USE_STREAM_READER
            meta = {k: v for k, v in step.items() if k != 'data'}
            # Longer/complex header fields use the streaming reader's exact
            # token-length and malformed-attachment rules.
            if any(isinstance(v, (dict, list)) or isinstance(v, str) and len(v) > 320
                   for v in meta.values()):
                return _USE_STREAM_READER
            reader.present = reader.has_data = True
            reader.meta, reader.data = meta, step['data']
            del document['stepModel']
        return document
    except (ValueError, RecursionError, OverflowError):
        pass
    return _USE_STREAM_READER


def _read(stream, max_bytes, source_name, capture_step=False):
    head = stream.read(len(COMPRESSED_MAGIC))
    compressed = None
    if head == COMPRESSED_MAGIC:
        compressed, data = _decompress(stream, max_bytes)
        stream = io.BytesIO(data)
    else:
        stream = io.BufferedReader(_Prefixed(head, stream))
    reader = _Reader(stream, max_bytes, capture_step)
    document = _small_document(data, reader) if compressed is not None else _USE_STREAM_READER
    if document is _USE_STREAM_READER:
        document = reader.read()
    # Hash and size always describe the file bytes on disk.
    result = {'document': document,
              'sourceSha256': hashlib.sha256(compressed).hexdigest() if compressed is not None else reader.hash.hexdigest(),
              'sizeBytes': len(compressed) if compressed is not None else reader.size,
              'embeddedStep': _metadata(reader, source_name)}
    if compressed is not None:
        result['compressed'] = True
    return result, reader


class _Prefixed(io.RawIOBase):
    """Re-emits already-consumed leading bytes before the rest of the stream."""
    def __init__(self, head, stream):
        self.head, self.stream = head, stream
    def readable(self):
        return True
    def readinto(self, buffer):
        if self.head:
            n = min(len(buffer), len(self.head))
            buffer[:n] = self.head[:n]; self.head = self.head[n:]
            return n
        data = self.stream.read(len(buffer))
        buffer[:len(data)] = data
        return len(data)


def read_bcbend_document(path, max_bytes=MAX_BYTES, *, source_name=None):
    path = Path(path)
    with path.open('rb') as stream:
        return _read(stream, max_bytes, source_name or path.name)[0]


def parse_bcbend_document(raw: bytes, max_bytes=MAX_BYTES, *, source_name=None):
    if not isinstance(raw, bytes): raise TypeError('raw must be bytes')
    if len(raw) > max_bytes: raise BCBendJsonError('El archivo supera el límite de tamaño.')
    return _read(io.BytesIO(raw), max_bytes, source_name or 'bend.bcbend')[0]


read_bcbend_bytes = parse_bcbend_document


def read_bcbend_with_step(source, *, source_name=None, max_bytes=MAX_BYTES):
    """Read the geometry and retain optional encoded data for a requested extraction."""
    source = Path(source)
    with source.open('rb') as stream:
        result, reader = _read(stream, max_bytes, source_name or source.name, True)
    return result, reader.data


def extract_embedded_step(source, directory, *, source_name=None, max_bytes=MAX_BYTES, _source_info=None):
    if _source_info is None:
        _source_info = read_bcbend_with_step(source, source_name=source_name, max_bytes=max_bytes)
    result, encoded = _source_info
    attachment = result['embeddedStep']
    if not attachment['present']: raise EmbeddedStepError('embedded-step-absent')
    if attachment['status'] != 'unverified' or not isinstance(encoded, str): raise EmbeddedStepError()
    try:
        payload = base64.b64decode(encoded, validate=True)
    except (ValueError, binascii.Error): raise EmbeddedStepError() from None
    if (base64.b64encode(payload).decode('ascii') != encoded
            or len(payload) != attachment['sizeBytes'] or len(payload) > max_bytes
            or hashlib.sha256(payload).hexdigest() != attachment['sha256']):
        raise EmbeddedStepError()
    # NOTHING above this line creates any directory or output file.
    destination = Path(directory)
    destination.mkdir(parents=True, exist_ok=True)
    filename = _safe_name(attachment['fileName'], source_name or Path(source).name)
    stem, suffix = Path(filename).stem, Path(filename).suffix
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='wb', dir=destination, prefix='.embedded-step-', suffix='.tmp', delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(payload); stream.flush(); os.fsync(stream.fileno())
        for index in range(10000):
            target = destination/(filename if index == 0 else f'{stem}-{index}{suffix}')
            try:
                if os.name == 'nt':
                    # Windows rename is atomic and fails if the target exists.
                    os.rename(temporary, target)
                else:
                    # POSIX rename would replace the target. Exclusive hard-link
                    # publication provides the same atomic visibility without it.
                    os.link(temporary, target)
                    temporary.unlink()
                temporary = None
                return {'path': str(target.resolve()), 'fileName': target.name, 'sizeBytes': len(payload),
                        'sha256': attachment['sha256'], 'status': 'extracted'}
            except FileExistsError:
                continue
            except PermissionError:
                if target.exists() or target.is_symlink(): continue
                raise
        raise OSError('No existe un nombre libre para extraer el STEP.')
    finally:
        if temporary is not None:
            try: temporary.unlink()
            except FileNotFoundError: pass


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description='Extrae el STEP embebido tras validar base64, tamaño y SHA-256; nunca reemplaza archivos.')
    parser.add_argument('--bend', type=Path, required=True)
    parser.add_argument('--out-dir', type=Path, required=True)
    parser.add_argument('--source-name')
    args = parser.parse_args(argv)
    try:
        result = extract_embedded_step(args.bend, args.out_dir, source_name=args.source_name)
        print(json.dumps({'ok': True, 'path': result['path'], 'sizeBytes': result['sizeBytes'], 'sha256': result['sha256']}, ensure_ascii=True))
        return 0
    except (ValueError, OSError) as exc:
        print(json.dumps({'ok': False, 'error': str(exc)}, ensure_ascii=True))
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
