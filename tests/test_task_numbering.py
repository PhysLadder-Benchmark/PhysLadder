"""Sequential identities must not dispatch to the task with the old same ID."""
import hashlib
import json
from pathlib import Path
import unittest

from robot_tasks.registry import TASKS, resolve_task
from robot_tasks.task_numbering import from_legacy, numbering, previous_task_id, to_legacy
from robot_tasks.video_delivery import scenes, setup_compatible

ROOT = Path(__file__).resolve().parents[1]


class TaskNumberingTests(unittest.TestCase):
    def test_current_catalogue_and_explicit_historical_mapping(self):
        expected = ({f"L1-{i:02}" for i in range(1, 6)} |
                    {f"L2-{i:02}" for i in range(1, 8)} |
                    {"L3-01", "L3-02", "L3-04", "L3-05"})
        self.assertEqual({i for i in TASKS if i.startswith("L") and "_" not in i}, expected)
        self.assertEqual(len(numbering()["tasks"]), 16)
        for row in numbering()["tasks"]:
            for suffix in ("", "_v3", "_variant01"):
                self.assertEqual(from_legacy(row["legacy_id"] + suffix), row["id"] + suffix)
                self.assertEqual(to_legacy(row["id"] + suffix), row["legacy_id"] + suffix)
                self.assertEqual(resolve_task(row["id"] + suffix)[1].task_id, row["id"])
            self.assertIn("/" + row["legacy_id"] + "/", scenes()[row["id"]]["runtime_path"])
        self.assertEqual(from_legacy("L3-03"), "L3-01")
        self.assertEqual(from_legacy("L2-07"), "L2-05")
        self.assertEqual(from_legacy("L3-11"), "L2-07")
        for suffix in ("", "_v3", "_variant01"):
            self.assertEqual(previous_task_id("L2-07" + suffix), "L3-03" + suffix)
            with self.assertRaises((KeyError, ValueError)):
                resolve_task("L3-03" + suffix)
        self.assertIn("whiteboard", resolve_task("L2-07")[1].instruction)
        self.assertIn("bottle", resolve_task("L3-01")[1].instruction)

    def test_previous_visual_setup_is_reused_for_the_same_physical_task(self):
        digest = "acda1a821f894899b862d23fcf3c09814cab31df670815c5229739446c6372fa"
        path = ROOT / "configs/task_references/video_visual_setup_history" / (digest + ".json")
        self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), digest)
        for row in numbering()["tasks"]:
            self.assertTrue(setup_compatible(row["id"], digest), row["id"])

    def test_website_configs_and_images_match_current_task_identity(self):
        rows = json.loads((ROOT / "site/tasks.json").read_text(encoding="utf-8"))
        self.assertEqual({r["id"] for r in rows}, {r["id"] for r in numbering()["tasks"]})
        for row in rows:
            self.assertEqual(row["level"], row["id"][:2])
            for setting in row["settings"].values():
                spec, config = resolve_task(setting["id"])
                self.assertEqual(config.source, ROOT / setting["config"])
                self.assertEqual(setting["record_id"], config.experiment_id)
                for image in setting["images"].values():
                    self.assertTrue((ROOT / "site" / image).is_file(), image)
        html = (ROOT / "site/index.html").read_text(encoding="utf-8")
        self.assertIn('id="task-list"', html)
        self.assertIn('src="tasks.js"', html)
        self.assertIn('src="app.js"', html)
        exported = (ROOT / "site/tasks.js").read_text(encoding="utf-8")
        prefix = "window.BENCHMARK_TASKS = "
        self.assertTrue(exported.startswith(prefix))
        self.assertEqual(json.loads(exported[len(prefix):].strip().removesuffix(";")), rows)

    def test_pre_reclassification_visual_setup_is_still_compatible(self):
        digest = "e9fcc542e1686b725f1e74f7e572e314982f0aed83dd0bb5d876e57dda61f03c"
        path = ROOT / "configs/task_references/video_visual_setup_history" / (digest + ".json")
        self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), digest)
        for row in numbering()["tasks"]:
            for suffix in ("", "_v3"):
                _, cfg = resolve_task(row["id"] + suffix)
                self.assertTrue(setup_compatible(cfg.scene_id, digest), row["id"] + suffix)
