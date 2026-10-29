"""Check asset restore boundaries and preserved per-task reproduction budgets."""
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import tarfile
import tempfile
import unittest
import subprocess
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]


def module(name, relative):
    spec = importlib.util.spec_from_file_location(name, ROOT / relative)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


installer = module("install_assets", "tools/release/install_assets.py")
reproduce = module("reproduce", "tools/reproduce.py")
asset_verifier = module("asset_verifier", "tools/verify_runtime_assets.py")


class ReleaseToolTests(unittest.TestCase):
    def test_asset_inventory_excludes_submodule_git_metadata_only(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            assets = root / "assets"
            assets.mkdir()
            (assets / "scene.usda").write_bytes(b"scene")
            (assets / "runtime.lock.json").write_text("{}")
            with patch.object(asset_verifier, "ROOT", root), patch.object(
                    asset_verifier, "LOCK", assets / "runtime.lock.json"):
                expected = asset_verifier.inventory()
                (assets / ".gitattributes").write_text("* -text -whitespace\n")
                self.assertEqual(asset_verifier.inventory(), expected)
                (assets / ".git").write_text("gitdir: ../.git/modules/assets")
                self.assertEqual(asset_verifier.inventory(), expected)
                (assets / ".git").unlink()
                (assets / ".git" / "objects").mkdir(parents=True)
                (assets / ".git" / "objects" / "metadata").write_text("git object")
                self.assertEqual(asset_verifier.inventory(), expected)
                (assets / "unexpected.usda").write_bytes(b"unlisted input")
                self.assertIn("assets/unexpected.usda", asset_verifier.inventory())

    def test_git_ignore_check_runs_inside_asset_repository(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            assets = root / "assets"
            assets.mkdir()
            subprocess.run(["git", "init", "-q", str(assets)], check=True)
            (assets / "scene.usda").write_bytes(b"scene")
            with patch.object(asset_verifier, "ROOT", root):
                asset_verifier.check_git_ignore({"assets/scene.usda": {}})
                (assets / ".gitignore").write_text("scene.usda\n")
                with self.assertRaisesRegex(ValueError, "Git ignores runtime assets"):
                    asset_verifier.check_git_ignore({"assets/scene.usda": {}})

    def test_uninitialized_assets_report_download_command(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            errors = io.StringIO()
            with patch.object(asset_verifier, "ROOT", root), patch.object(
                    asset_verifier, "LOCK", root / "assets/runtime.lock.json"), patch(
                    "sys.argv", ["verify_runtime_assets.py"]), patch("sys.stderr", errors):
                with self.assertRaises(SystemExit):
                    asset_verifier.main()
            self.assertIn("git submodule update --init assets", errors.getvalue())

    def test_source_delivery_includes_optional_skill_and_excludes_private_files(self):
        packager = module('release_packager_test', 'tools/release/package.py')
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            keep = ['skills/r5-remote-deploy/SKILL.md',
                    'skills/r5-remote-deploy/scripts/fleet.py',
                    'skills/r5-remote-deploy/assets/fleet.example.json']
            private = ['configs/.env', 'skills/r5-remote-deploy/.env.remote',
                       'skills/r5-remote-deploy/assets/fleet.local.json',
                       'tools/local.key', 'tools/.venv/example.py']
            for name in keep + private:
                path = root / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text('synthetic fixture')
            with patch.object(packager, 'ROOT', root):
                selected = set(packager.source_files())
            self.assertTrue(set(keep) <= selected)
            self.assertFalse(set(private) & selected)

    def test_reproduction_preserves_nonuniform_budgets(self):
        rows = reproduce.plan(["L2-03_variant01", "L3-03", "L3-03_variant01"],
                              ROOT / "configs/models/gpt.toml")
        self.assertEqual([(r["max_calls"], r["wall_time_s"]) for r in rows],
                         [(100, 0), (0, 18000), (150, 18000)])
        self.assertEqual(rows[0]["record_id"], "L2-03_v3")
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
