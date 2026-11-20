"""Tiny on-disk cache so repeated runs don't hammer the public APIs."""
import hashlib
import os
from pathlib import Path

CACHE_DIR = Path(os.environ.get("TRACKGEN_CACHE", Path.home() / ".cache" / "trackgen"))


def cache_path(namespace, key, suffix):
    h = hashlib.sha1(key.encode("utf-8")).hexdigest()[:20]
    d = CACHE_DIR / namespace
    d.mkdir(parents=True, exist_ok=True)
    return d / f"{h}{suffix}"
