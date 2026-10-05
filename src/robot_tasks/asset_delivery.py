"""Portable access to the version-locked, privately delivered Tuojing assets."""

import hashlib
import json

from .paths import PROJECT_ROOT

ROOT = PROJECT_ROOT / "assets/tuojing"


def verified_package(package_id):
    lock = json.loads((ROOT / "source.lock.json").read_text(encoding="utf-8"))
    package = next(p for p in lock["packages"] if p["id"] == package_id)
    directory = ROOT / "source" / package["directory"]
    for name, expected in package["files"].items():
        path = directory / name
        if not path.is_file():
            raise ValueError("Missing Tuojing delivery; run python tools/tuojing_assets.py "
                             f"--archives .cache/tuojing_assets. Missing: {path}")
        if path.stat().st_size != expected["bytes"] or hashlib.sha256(path.read_bytes()).hexdigest() != expected["sha256"]:
            raise ValueError(f"Tuojing delivery has changed: {path}; do not mix asset/calibration versions")
    return directory, package


def verified_prop(name):
    catalog = json.loads((ROOT / "catalog.json").read_text(encoding="utf-8"))
    entry = next((e for e in catalog["entries"] if e["id"] == name and e["role"] == "prop"), None)
    if entry is None:
        raise ValueError(f"Unregistered Tuojing prop: {name}")
    directory, package = verified_package(entry["package"])
    if entry["path"] not in package["files"]:
        raise ValueError(f"Prop is not in the locked delivery: {name}")
    return directory / entry["path"], {"delivery": "2026-10-05", "prop": name,
                                       "package": entry["package"], "path": entry["path"],
                                       "sha256": package["files"][entry["path"]]["sha256"]}
