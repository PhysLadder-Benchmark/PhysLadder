"""Task copies remain independent, portable, pinned and complete; no simulator/API."""

import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from robot_tasks import asset_delivery
from robot_tasks.asset_layout import bundles, find_bundle, install_layout, verify_files
from tools.tuojing_assets import install_package, organize


class AssetLayoutTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        contents = {
            "assets/marker/asset.usda": b'@../body/body.usda@',
            "assets/body/body.usda": b'@label.png@',
            "assets/body/label.png": b'texture fixture',
            "unrelated.txt": b'not part of the task copy',
        }
        files = {}
        for name, payload in contents.items():
            path = self.root / "packages/vendor" / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(payload)
            files[name] = {"bytes": len(payload), "sha256": hashlib.sha256(payload).hexdigest()}
        self.write_json("source.lock.json", {"packages": [
            {"id": "vendor", "directory": "vendor", "files": files}]})
        self.write_json("catalog.json", {"entries": [
            {"id": "marker", "package": "vendor", "path": "assets/marker/asset.usda", "role": "prop"}]})
        self.layout = {"schema_version": 1, "shared": {}, "tasks": {
            "L1-01": {"name": "First", "props": {"pen": "marker"}},
            "L2-01": {"name": "Second", "props": {"pen": "marker"}}},
            "bundles": {"marker": {"root": "assets", "include": ["assets/marker", "assets/body"]}}}
        self.write_json("task_assets.json", self.layout)

    def write_json(self, name, value):
        (self.root / name).write_text(json.dumps(value), encoding="utf-8")

    def test_task_copies_keep_relative_dependencies_without_sharing_files(self):
        install_layout(self.root)
        first = find_bundle(self.root, "L1-01", "marker")
        second = find_bundle(self.root, "L2-01", "marker")
        self.assertEqual(first.entry.as_posix(), "marker/asset.usda")
        self.assertTrue((first.destination / "body/label.png").is_file())
        self.assertFalse((first.destination / "unrelated.txt").exists())
        self.assertFalse((first.destination / first.entry).samefile(second.destination / second.entry))
        original = (second.destination / first.entry).read_bytes()
        (first.destination / first.entry).write_bytes(b'local edit')
        self.assertEqual((second.destination / second.entry).read_bytes(), original)
        self.assertEqual((first.source / first.entry).read_bytes(), original)
        with self.assertRaises(ValueError):
            install_layout(self.root)
        self.assertEqual((first.destination / first.entry).read_bytes(), b'local edit')

    def test_runtime_reads_requested_copy_and_does_not_fall_back(self):
        install_layout(self.root)
        with patch.object(asset_delivery, "ROOT", self.root):
            path, evidence = asset_delivery.verified_prop("marker", task_id="L2-01")
            self.assertEqual(path, self.root / "source/L2-01/pen/marker/asset.usda")
            self.assertEqual(evidence["task_id"], "L2-01")
            (path.parent.parent / "body/label.png").unlink()
            with self.assertRaises(ValueError):
                asset_delivery.verified_prop("marker", task_id="L2-01")
            asset_delivery.verified_prop("marker", task_id="L1-01")
            with self.assertRaises(ValueError):
                asset_delivery.verified_prop("marker", task_id="L3-99")

    def test_old_delivery_is_moved_intact_and_organizing_is_repeatable(self):
        (self.root / "source").mkdir()
        (self.root / "packages/vendor").rename(self.root / "source/vendor")
        organize(self.root)
        self.assertFalse((self.root / "source/vendor").exists())
        self.assertTrue((self.root / "packages/vendor/unrelated.txt").exists())
        organize(self.root)
        for item in bundles(self.root):
            verify_files(item.destination, item.files)

    def test_modified_delivery_is_not_moved(self):
        (self.root / "source").mkdir()
        (self.root / "packages/vendor").rename(self.root / "source/vendor")
        (self.root / "source/vendor/unrelated.txt").write_text("modified")
        with self.assertRaises(ValueError):
            organize(self.root)
        self.assertTrue((self.root / "source/vendor").exists())
        self.assertFalse((self.root / "source/L1-01").exists())

    def test_installer_reuses_old_verified_delivery_without_archive(self):
        (self.root / "source").mkdir()
        (self.root / "packages/vendor").rename(self.root / "source/vendor")
        package = json.loads((self.root / "source.lock.json").read_text())["packages"][0]
        install_package(self.root, self.root / "no_archives", package)
        install_layout(self.root)
        self.assertTrue((self.root / "packages/vendor/unrelated.txt").exists())
        self.assertTrue((self.root / "source/L2-01/pen/body/label.png").exists())

    def test_paths_cannot_escape_task_directories(self):
        self.layout["tasks"]["L1-01"]["props"] = {"../escape": "marker"}
        self.write_json("task_assets.json", self.layout)
        with self.assertRaises(ValueError):
            list(bundles(self.root))

    def test_partial_install_does_not_install_unselected_packages(self):
        self.assertEqual(install_layout(self.root, []), [])
        self.assertFalse((self.root / "source/L1-01").exists())

    def test_all_current_task_assets_are_mapped(self):
        from robot_tasks.paths import PROJECT_ROOT
        from robot_tasks.tasks.bench.arm_selection import resolve_bench_spec
        from robot_tasks.config import load_config
        from robot_tasks.tasks.screw.nut import nut_spec
        import tomllib

        root = PROJECT_ROOT / "assets/tuojing"
        mapped = {}
        for bundle in bundles(root):
            mapped.setdefault(bundle.destination.parent.name, set()).add(bundle.asset)
        for config in (PROJECT_ROOT / "configs/tasks").glob("*.toml"):
            if config.name.endswith(".local.toml"):
                continue  # Untracked personal diagnostics are not the task catalog.
            task = tomllib.loads(config.read_text(encoding="utf-8"))["task"]["id"]
            cfg = load_config(config)
            spec = nut_spec() if task == "L3-05" else resolve_bench_spec(cfg.raw["task"], cfg.robot)
            assets = {a.asset.split(":", 1)[1] for a in spec.design.articles
                      if a.asset.startswith("tuojing:")}
            if spec.design.fixture_asset.startswith("tuojing:"):
                assets.add(spec.design.fixture_asset.split(":", 1)[1])
            assets.update(i["asset"] for i in spec.parameters.get("native_instances", []))
            if "pedestal_asset" in spec.parameters:
                assets.add(spec.parameters["pedestal_asset"])
            self.assertEqual(assets, mapped.get(task, set()), task)


if __name__ == "__main__":
    unittest.main()
