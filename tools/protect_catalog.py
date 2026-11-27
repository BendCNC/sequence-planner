"""Write protected catalogs (.bct) for distribution; see viewer/protected-json.mjs for the format.

Usage: protect_catalog.py <in.json> <out.bct> [...pairs]
"""
import hashlib
import hmac
import os
import sys
import zlib

MAGIC = b"BCTE1\n"
KEY = hashlib.sha256("BendCNC tooling catalog v1 \u00b7 7f3c91e2a4d84b0c".encode("utf-8")).digest()


def protect(plain_json_bytes, nonce=None):
    nonce = nonce or os.urandom(16)
    data = zlib.compress(plain_json_bytes, 9)
    out = bytearray(len(data))
    for block, i in enumerate(range(0, len(data), 32)):
        ks = hashlib.sha256(KEY + nonce + block.to_bytes(4, "big")).digest()
        chunk = data[i:i + 32]
        out[i:i + len(chunk)] = bytes(a ^ b for a, b in zip(chunk, ks))
    tag = hmac.new(KEY, nonce + bytes(out), hashlib.sha256).digest()
    return MAGIC + nonce + tag + bytes(out)


def main(argv):
    if len(argv) < 2 or len(argv) % 2:
        raise SystemExit(__doc__)
    for src, dst in zip(argv[::2], argv[1::2]):
        with open(src, "rb") as f:
            payload = protect(f.read())
        with open(dst, "wb") as f:
            f.write(payload)
        print(f"{dst}: {len(payload)} bytes")


if __name__ == "__main__":
    main(sys.argv[1:])
