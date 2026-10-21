#!/usr/bin/env python3
"""Verify portable runtime inputs without starting Isaac or calling a model.

--write-lock explicitly refreshes the inventory after an intentional asset edit.
--usd additionally needs usd-core (or Isaac's Python) and resolves USD references.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
LOCK = ROOT / "assets/runtime.lock.json"
BUILTIN_MDL = {"OmniPBR.mdl", "OmniGlass.mdl", "OmniSurface.mdl"}


def inventory():
    return {
        p.relative_to(ROOT).as_posix(): {
            "bytes": p.stat().st_size,
            "sha256": hashlib.sha256(p.read_bytes()).hexdigest(),
        }
        for p in sorted((ROOT / "assets").rglob("*"))
        if p.is_file() and p != LOCK and "__pycache__" not in p.parts
        and p.suffix not in {".pyc", ".tmp"}
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--write-lock", action="store_true")
    parser.add_argument("--usd", action="store_true")
    parser.add_argument("--check-git", action="store_true")
    args = parser.parse_args()
    actual = inventory()
    if args.write_lock:
        LOCK.write_text(json.dumps({
            "schema_version": 1,
            "scope": "Combined checkout assets; historical deliveries and caches excluded",
            "external_runtime": ["Isaac Sim 5.1", "cuRobo planner environment"],
            "isaac_builtin_mdl": sorted(BUILTIN_MDL),
            "files": actual,
        }, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    expected = json.loads(LOCK.read_text(encoding="utf-8"))["files"]
    differences = [name for name in sorted(set(actual) | set(expected))
                   if actual.get(name) != expected.get(name)]
    if differences:
        raise ValueError("Missing, changed, or unlisted assets:\n" + "\n".join(differences))
    if args.check_git:
        result = subprocess.run(
            ["git", "check-ignore", "--no-index", "--stdin"],
            input="\n".join(expected) + "\n", text=True, capture_output=True, cwd=ROOT)
        if result.returncode != 1:
            raise ValueError("Git ignores runtime assets, or Git check failed:\n"
                             + result.stdout + result.stderr)
    if args.usd:
        sys.path.insert(0, str(ROOT / "src"))
        from pxr import Ar, UsdUtils
        from robot_tasks.asset_delivery import verified_shared
        from robot_tasks.video_delivery import scenes, verified_scene

        entries = [(name, verified_scene(name)) for name in scenes()]
        entries.append(("laboratory engineering check", verified_shared("lab")))
        for name, path in entries:
            layers, assets, unresolved = UsdUtils.ComputeAllDependencies(str(path))
            missing = sorted(set(unresolved) - BUILTIN_MDL)
            external = []
            for item in [*assets, *(layer.realPath for layer in layers)]:
                outer = Ar.SplitPackageRelativePathOuter(item)[0] if Ar.IsPackageRelativePath(item) else item
                if not Path(outer).resolve().is_relative_to(ROOT):
                    external.append(item)
            if missing or external:
                raise ValueError(f"{name}: unresolved={missing}, outside checkout={external}")
        print(f"USD references verified: {len(entries)} scenes; built-in MDL supplied by Isaac Sim 5.1")
    print(f"Runtime inventory verified: {len(expected)} files, "
          f"{sum(row['bytes'] for row in expected.values()) / 2**20:.1f} MiB")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
