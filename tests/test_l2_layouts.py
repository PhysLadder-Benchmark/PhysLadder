"""Arm-facing layouts, contained ball packing and L2 process contracts."""

import math
import unittest

import numpy as np

from robot_tasks.planning.scene import article_surface
from robot_tasks.registry import resolve_task
from robot_tasks.robots.transforms import pose_transform
from robot_tasks.tasks.bench.evaluation import BenchEvaluator
from robot_tasks.tasks.bench.scoring_l2 import ball_contained
from robot_tasks.tasks.bench.spec import build_spec
from robot_tasks.tasks.l1.specs import local_corners
from test_household_tasks import measurement


class L2LayoutTests(unittest.TestCase):
    def test_single_arm_roles_and_arm_facing_extraction_axes(self):
        for task in ("L2-03", "L2-05", "L2-07"):
            with self.subTest(task=task):
                _, config = resolve_task(task)
                spec = build_spec(task)
                self.assertEqual(config.arms, ("left",))
                self.assertEqual(config.observation_arms, ("right",))
                spec_task = resolve_task(task)[0]
                spec_task.factory.prepare_config(config)
                article = spec.design.articles[0]
                if task == "L2-05":
                    center = np.array(article.initial[:2])
                    mount = next(m for m in config.robot.mounts if m.name == "left")
                    toward_arm = np.array(mount.base_pose_wxyz[:2]) - center
                    self.assertGreater(toward_arm @ [-1, 0] / np.linalg.norm(toward_arm), .95)
                else:
                    outward = spec.parameters["outward_axis_world"]
                    np.testing.assert_allclose(outward, [-1, 0, 0])
                    # Local front, scoring direction and target must all rotate.
                    np.testing.assert_allclose(-pose_transform(article.initial)[:3, 0], outward, atol=1e-12)
                    travel = np.array(article.goal[:3]) - article.initial[:3]
                    self.assertGreater(travel @ outward, .065)
                    np.testing.assert_allclose(travel[[1, 2]], 0, atol=1e-12)

    def test_middle_book_has_exposed_grip_and_free_extraction_corridor(self):
        spec = build_spec("L2-03")
        book = spec.design.articles[0]
        fixtures = {p.name: p for p in spec.design.fixtures}
        front = spec.parameters["stack_front_world"][0]
        self.assertLess(book.parts[0].size[1], .070)
        self.assertGreater(front - (book.initial[0] - .060), .050)
        self.assertLess(fixtures["lower_book"].position[2] + .013, book.initial[2] - .012)
        self.assertGreater(fixtures["upper_book"].position[2] - .013, book.initial[2] + .012)
        self.assertLess(book.goal[0] + .060, front - .004)
        for name in ("lower_book", "upper_book"):
            part = fixtures[name]
            self.assertAlmostEqual(part.rotation[2], 0.)
            self.assertAlmostEqual(part.position[0] - part.size[0] / 2, front)
        self.assertEqual(set(a.name for a in spec.design.articles), {"middle_book"})

    def test_sideways_motion_cannot_count_as_front_extraction(self):
        for task, object_name in (("L2-03", "middle_book"), ("L2-07", "drawer")):
            with self.subTest(task=task):
                spec = build_spec(task)
                rows = measurement(spec)
                row = rows[object_name]
                row["pose"][1] -= .11
                row["corners"] = (np.asarray(row["corners"]) - [0, .11, 0]).tolist()
                row["fingers"]["left"] = [1, 1]
                e = BenchEvaluator(spec, {a.name: a.initial for a in spec.design.articles})
                for i in range(1, 15):
                    result = e.update(i * .1, rows)
                    if task == "L2-07" and i >= 4:
                        # A persistently detached drawer violates its physical rail;
                        # a free book may instead be moved back and extracted correctly.
                        self.assertEqual(result.status, "failed")
                        self.assertIn("guide", result.reason)
                        break
                    self.assertEqual(result.status, "running")
                self.assertNotIn("middle_extracted_while_held", e.flags)
                self.assertNotIn("opened_while_held", e.flags)

    def test_partly_extracted_book_is_not_complete(self):
        spec = build_spec("L2-03")
        rows = measurement(spec)
        row = rows["middle_book"]
        row["pose"][0] -= .04
        row["corners"] = (np.asarray(row["corners"]) - [.04, 0, 0]).tolist()
        row["fingers"]["left"] = [1, 1]
        e = BenchEvaluator(spec, {a.name: a.initial for a in spec.design.articles})
        for i in range(1, 15):
            result = e.update(i * .1, rows)
            self.assertEqual(result.status, "running")
        self.assertLess(result.metrics["criteria"]["rear_clearance_m"], 0)

    def test_loaded_bowl_contains_separate_nonintersecting_real_spheres(self):
        spec = build_spec("L2-04")
        p = spec.parameters
        balls = spec.design.articles[1:]
        self.assertEqual(len(balls), 50)
        self.assertFalse(spec.joints)
        self.assertFalse(any("feed" in a.name for a in spec.design.articles))
        points = np.array([a.initial[:3] for a in balls])
        for i, ball in enumerate(balls):
            self.assertEqual(ball.parts[0].shape, "sphere")
            self.assertTrue(ball_contained(points[i] - spec.design.articles[0].initial[:3], p))
            if i:
                self.assertTrue(np.all(np.linalg.norm(points[:i] - points[i], axis=1) >= 2 * p["ball_radius_m"]))
        self.assertLess(p["ball_friction"], .1)
        surface = article_surface(balls[0])
        np.testing.assert_allclose(np.linalg.norm(surface, axis=1), p["ball_radius_m"])
        np.testing.assert_allclose(np.ptp(local_corners(balls[0]), axis=0), [0.020] * 3)

    def test_ball_on_rim_or_outside_is_not_contained(self):
        p = build_spec("L2-04").parameters
        self.assertTrue(ball_contained((0, 0, .02), p))
        self.assertFalse(ball_contained((0, 0, .066), p))
        self.assertFalse(ball_contained((.068, 0, .055), p))
        self.assertFalse(ball_contained((0, 0, -.01), p))

    def test_latch_requires_unlock_open_removal_and_reclosure(self):
        spec = build_spec("L2-05")
        rows = measurement(spec)
        initial = {a.name: a.initial for a in spec.design.articles}
        e = BenchEvaluator(spec, initial)
        rows["latch"]["pose"][0] -= .017
        rows["latch"]["fingers"]["left"] = [1, 1]
        e.update(.1, rows)
        self.assertIn("latch_retracted", e.flags)
        # Angle is measured relative to the initially reversed lid, not world yaw.
        # Compose world Y opening with the original orientation.
        from robot_tasks.robots.transforms import origin_transform, transform_pose
        transform = pose_transform(initial["lid"])
        transform[:3, :3] = origin_transform((0, 0, 0), (0, math.radians(110), 0))[:3, :3] @ transform[:3, :3]
        rows["lid"]["pose"] = list(transform_pose(transform))
        e.update(.2, rows)
        self.assertIn("box_opened", e.flags)
        rows["eraser"]["pose"][:3] = [.28, .48, .08]
        rows["eraser"]["fingers"]["left"] = [1, 1]
        e.update(.3, rows)
        e.update(.5, rows)
        self.assertIn("item_removed", e.flags)
        for name in ("lid", "latch"):
            rows[name]["pose"] = list(initial[name])
            rows[name]["fingers"]["left"] = [0, 0]
        rows["eraser"]["pose"][2] = .009
        rows["eraser"]["bench_force_n"] = .25
        rows["eraser"]["fingers"]["left"] = [0, 0]
        for i in range(6, 18):
            result = e.update(i * .1, rows)
        self.assertEqual(result.status, "succeeded")
        self.assertNotEqual(BenchEvaluator(spec, initial).update(.1, rows).status, "succeeded")

    def test_drawer_has_roof_bottom_outside_handle_and_negative_x_stroke(self):
        spec = build_spec("L2-07")
        fixtures = {p.name: p for p in spec.design.fixtures}
        self.assertIn("cabinet_roof", fixtures)
        self.assertIn("rail_bottom", fixtures)
        drawer = spec.design.articles[0]
        parts = {p.name: p for p in drawer.parts}
        self.assertLess(parts["handle"].position[0] + parts["handle"].size[0] / 2,
                        parts["front"].position[0] - parts["front"].size[0] / 2)
        self.assertLess(parts["handle"].size[1], .070)
        tray_top = max(p.position[2] + p.size[2] / 2 for p in drawer.parts if not p.name.startswith("handle"))
        self.assertLess(drawer.initial[2] + tray_top,
                        fixtures["cabinet_roof"].position[2] - fixtures["cabinet_roof"].size[2] / 2)
        self.assertEqual(spec.joints[0].axes["transX"][:2], (-.13, 0))
        self.assertNotIn("transY", spec.joints[0].axes)
        self.assertAlmostEqual(fixtures["cabinet_roof"].rotation[2], 0.)
        # The rail surface must carry gravity before the vertical joint limit.
        rail_top = fixtures["rail_bottom"].position[2] + fixtures["rail_bottom"].size[2] / 2
        self.assertLess(drawer.initial[2] + spec.joints[0].axes["transZ"][0], rail_top)


if __name__ == "__main__":
    unittest.main()
