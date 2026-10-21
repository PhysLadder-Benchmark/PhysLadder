"""Check asset restore boundaries and preserved per-task reproduction budgets."""
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import tarfile
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


def module(name, relative):
    spec = importlib.util.spec_from_file_location(name, ROOT / relative)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


installer = module("install_assets", "tools/release/install_assets.py")
reproduce = module("reproduce", "tools/reproduce.py")


class ReleaseToolTests(unittest.TestCase):
    def test_reproduction_preserves_nonuniform_budgets(self):
        rows = reproduce.plan(["L2-04_variant01", "L3-14", "L3-14_variant01"],
                              ROOT / "configs/models/gpt.toml")
        self.assertEqual([(r["max_calls"], r["wall_time_s"]) for r in rows],
                         [(100, 0), (0, 18000), (150, 18000)])
        self.assertEqual(rows[0]["record_id"], "L2-04_v3")
        self.assertEqual(rows[2]["model_budget_basis"], "completed_generations")

    def test_asset_delivery_is_checked_before_overwrite(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / "assets").mkdir()
            original = root / "assets/scene.usda"
            original.write_bytes(b"old")
            data = b"new"
            lock = {"files": {"assets/scene.usda": {
                "bytes": 3, "sha256": hashlib.sha256(data).hexdigest()}}}
            (root / "assets/runtime.lock.json").write_text(json.dumps(lock))
            archive = root / "assets.tar"

            def write(names):
                with tarfile.open(archive, "w") as bundle:
                    for name, content in names:
                        info = tarfile.TarInfo(name)
                        info.size = len(content)
                        bundle.addfile(info, io.BytesIO(content))

            for names in ([("assets/scene.usda", b"bad")],
                          [("assets/scene.usda", data), ("../escape", data)], []):
                write(names)
                with self.assertRaises(ValueError):
                    installer.install(archive, root)
                self.assertEqual(original.read_bytes(), b"old")
            write([("assets/scene.usda", data)])
            self.assertEqual(installer.install(archive, root), 1)
            self.assertEqual(original.read_bytes(), data)
