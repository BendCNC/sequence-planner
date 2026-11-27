"""Identify a staged planner without requiring Git on the client.

Run after copying planner code, assets and hmi_materials.json, before archiving.
Only the staged copy receives ENGINE_REVISION and ENGINE_MANIFEST.json.
The content identity excludes platform runtimes, caches and the stamps themselves.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import tempfile


STAMP = "ENGINE_REVISION"
MANIFEST = "ENGINE_MANIFEST.json"
SKIP_DIRS = {".git", "__pycache__", ".pytest_cache"}
SKIP_SUFFIXES = {".pyc", ".pyo", ".log"}


def _is_link(path):
    return path.is_symlink() or getattr(path, "is_junction", lambda: False)()


def _planner_root(path):
    path = Path(path).absolute()
    if any(_is_link(p) for p in (path, *path.parents)):
        raise ValueError("Planner paths cannot traverse symlinks or junctions")
    path = path.resolve(strict=True)
    if not ((path / "tools/plan_sequence.mjs").is_file()
            and (path / "engine").is_dir() and (path / "viewer").is_dir()):
        raise ValueError("Expected tools/plan_sequence.mjs, engine/ and viewer/")
    return path


def _snapshot(package):
    files = []
    for here, dirs, names in os.walk(package, followlinks=False):
        parent = Path(here)
        dirs[:] = sorted(d for d in dirs if d not in SKIP_DIRS
                         and not (parent == package and d == "runtime"))
        for name in dirs:
            if _is_link(parent / name):
                raise ValueError("Payload contains a linked directory: " + name)
        for name in sorted(names):
            path = parent / name
            if ((parent == package and name in {STAMP, MANIFEST})
                    or path.suffix.lower() in SKIP_SUFFIXES):
                continue
            if _is_link(path) or not path.is_file():
                raise ValueError("Payload contains a linked/nonregular file: " + name)
            before = path.stat()
            digest = hashlib.sha256()
            with path.open("rb") as stream:
                for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                    digest.update(chunk)
            after = path.stat()
            if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
                raise RuntimeError("Payload changed while hashing: " + str(path))
            files.append({"path": path.relative_to(package).as_posix(),
                          "bytes": after.st_size, "sha256": digest.hexdigest()})
    return sorted(files, key=lambda entry: entry["path"])


def _source_commit(source):
    """Avoid accidentally reporting the Git revision of an enclosing repository."""
    try:
        top = subprocess.check_output(
            ["git", "-C", str(source), "rev-parse", "--show-toplevel"],
            stderr=subprocess.DEVNULL, text=True, timeout=10).strip()
        if Path(top).resolve() != source:
            return None
        commit = subprocess.check_output(
            ["git", "-C", str(source), "rev-parse", "--verify", "HEAD"],
            stderr=subprocess.DEVNULL, text=True, timeout=10).strip()
        return commit.lower() if re.fullmatch(r"[0-9a-fA-F]{40,64}", commit) else None
    except (OSError, subprocess.SubprocessError):
        return None


def _write_atomic(package, name, data):
    # Temporary files and replacement stay within the validated staging root.
    with tempfile.NamedTemporaryFile(dir=package, prefix="." + name + ".",
                                     suffix=".tmp", delete=False) as stream:
        temporary = Path(stream.name)
        stream.write(data)
    try:
        temporary.replace(package / name)
    finally:
        if temporary.exists():
            temporary.unlink()


def stamp_engine(source, package):
    """Return the manifest and write a one-line UTF-8/no-BOM revision to staging."""
    source = _planner_root(source)
    package = _planner_root(package)
    if package == source or package in source.parents or (package / ".git").exists():
        raise ValueError("Write stamps only to a separate packaged copy, never a checkout")
    for name in (STAMP, MANIFEST):
        if _is_link(package / name):
            raise ValueError("Stamp output cannot be a link")
    files = _snapshot(package)
    commit = _source_commit(source)
    if files != _snapshot(package):
        raise RuntimeError("Payload changed during stamping; package it again")
    canonical = json.dumps(files, sort_keys=True, ensure_ascii=True,
                           separators=(",", ":")).encode("utf-8")
    digest = hashlib.sha256(canonical).hexdigest()
    revision = "v6-sha256:" + digest
    manifest = {
        "schemaVersion": 1,
        "revision": revision,
        "contentSha256": digest,
        "sourceGitCommit": commit,
        "sourceGitCommitScope": "Source checkout base, not a claim that packaged files are committed or clean.",
        "contentScope": "Actual staged planner files, including catalogs, tooling assets and materials; not platform runtimes.",
        "excluded": ["ENGINE_REVISION", "ENGINE_MANIFEST.json", "runtime/",
                     "**/.git/", "**/__pycache__/", "**/.pytest_cache/", "*.pyc", "*.pyo", "*.log"],
        "hashAlgorithm": "SHA256 of UTF-8 JSON files array: sorted by POSIX path, sorted keys, ASCII escaping, no whitespace.",
        "files": files,
    }
    _write_atomic(package, MANIFEST,
                  (json.dumps(manifest, indent=2, ensure_ascii=True) + "\n").encode("utf-8"))
    # Publish the client-readable marker last, after the manifest is complete.
    _write_atomic(package, STAMP, (revision + "\n").encode("ascii"))
    return manifest


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", required=True, type=Path, help="Source planner checkout or archive")
    parser.add_argument("--package", required=True, type=Path, help="Prepared distribution planner directory")
    args = parser.parse_args(argv)
    try:
        manifest = stamp_engine(args.source, args.package)
    except (OSError, ValueError, RuntimeError) as error:
        parser.exit(1, "ENGINE_REVISION: " + str(error) + "\n")
    print(manifest["revision"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
