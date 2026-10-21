"""Sequential identities must not dispatch to the task with the old same ID."""
import hashlib
import json
from pathlib import Path
import unittest

from robot_tasks.registry import TASKS, resolve_task
from robot_tasks.task_numbering import from_legacy, numbering, to_legacy
from robot_tasks.video_delivery import scenes, setup_compatible

ROOT = Path(__file__).resolve().parents[1]


class TaskNumberingTests(unittest.TestCase):
    def test_contiguous_catalogue_and_explicit_historical_mapping(self):
        expected = {f"L{level}-{i:02}" for level, count in ((1, 5), (2, 6), (3, 5))
                    for i in range(1, count + 1)}
        self.assertEqual({i for i in TASKS if i.startswith("L") and "_" not in i}, expected)
        self.assertEqual(len(numbering()["tasks"]), 16)
        for row in numbering()["tasks"]:
            for suffix in ("", "_v3", "_variant01"):
                self.assertEqual(from_legacy(row["legacy_id"] + suffix), row["id"] + suffix)
                self.assertEqual(to_legacy(row["id"] + suffix), row["legacy_id"] + suffix)
                self.assertEqual(resolve_task(row["id"] + suffix)[1].task_id, row["id"])
            self.assertIn("/" + row["legacy_id"] + "/", scenes()[row["id"]]["runtime_path"])
        self.assertEqual(from_legacy("L3-03"), "L3-01")
        self.assertIn("whiteboard", resolve_task("L3-03")[1].instruction)
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
            for setting in row["settings"].values():
                spec, config = resolve_task(setting["id"])
                self.assertEqual(config.source, ROOT / setting["config"])
                self.assertEqual(setting["record_id"], config.experiment_id)
                for image in setting["images"].values():
                    self.assertTrue((ROOT / "site" / image).is_file(), image)
        html = (ROOT / "site/index.html").read_text(encoding="utf-8")
        self.assertIn('data-select="L3-05"', html)
        self.assertIn('media/L3-05-original-overview.webp', html)
