"""D-recess geometry and measured-state scoring; no GPT or robot solver."""

import math
import unittest

import numpy as np

from robot_tasks.planning.scene import article_surface
from robot_tasks.registry import resolve_task
from robot_tasks.robots.transforms import pose_transform
from robot_tasks.tasks.l1.evaluation import L1Evaluator, L1Settings
from robot_tasks.tasks.l1.shaped_recess import offset_polygon
from robot_tasks.tasks.l1.specs import build_design, local_corners, pose
from robot_tasks.tasks.l1.task import L1Task


class ShapedRecessTests(unittest.TestCase):
    def setUp(self):
        self.design = build_design("L1-02")
        self.article = self.design.articles[0]
        self.geometry = self.design.insertion

    def row(self, p, held=False, supported=True):
        t = pose_transform(p)
        return {
            self.article.name: {
                "pose": list(p),
                "linear_velocity": [0, 0, 0],
                "angular_velocity": [0, 0, 0],
                "finger_forces": [1, 1] if held else [0, 0],
                "fixture_force_n": self.article.mass * 9.81 if supported else 0,
                "bench_force_n": 0,
                "corners": (
                    local_corners(self.article) @ t[:3, :3].T + t[:3, 3]
                ).tolist(),
            }
        }

    def test_shape_fits_gripper_and_has_exposed_grip_space(self):
        # This geometry remains a library regression, outside the active task pool.
        from dataclasses import replace
        _, cfg = resolve_task("L1-01")
        cfg = replace(cfg, task_id="L1-02", instruction="Fit and release the D block.",
                      arms=("left",), observation_arms=("right",))
        width = np.ptp(self.geometry.profile, axis=0).max()
        self.assertLess(width + 0.015, cfg.robot.gripper.width_limits_m[1])
        self.assertAlmostEqual(
            self.geometry.height_m - self.geometry.hole_depth_m, 0.028
        )
        self.assertEqual(self.design.fixture_friction, 0)
        prepared = L1Task.prepare_config(cfg)
        self.assertIn("D-shaped", prepared.instruction)
        self.assertAlmostEqual(
            pose_transform(self.article.initial)[0, 0], math.sqrt(0.5)
        )

    def test_closed_convex_meshes_and_open_recess(self):
        for vertices, faces in [
            self.geometry.block_mesh(),
            *self.geometry.wall_meshes(),
        ]:
            vertices, faces = np.array(vertices), np.array(faces)
            counts = {}
            for face in faces:
                for a, b in zip(face, np.roll(face, -1)):
                    edge = tuple(sorted((int(a), int(b))))
                    counts[edge] = counts.get(edge, 0) + 1
            self.assertTrue(all(n == 2 for n in counts.values()))
            a, b, c = (vertices[faces[:, i]] for i in range(3))
            normals = np.cross(b - a, c - a)
            self.assertGreater(np.einsum("ij,ij->i", a, np.cross(b, c)).sum(), 0)
            # Every vertex is on or inside every face's outward plane.
            signed = np.einsum(
                "fvd,fd->fv", vertices[None, :, :] - a[:, None, :], normals
            )
            self.assertLessEqual(signed.max(), 1e-12)
        inner = offset_polygon(self.geometry.profile, self.geometry.clearance_m)
        for vertices, _ in self.geometry.wall_meshes():
            # Wall pieces must not seal the profile's interior.
            points = np.asarray(vertices)[:, :2]
            edges = np.roll(inner, -1, axis=0) - inner
            cross = edges[:, 0, None] * (points[:, 1] - inner[:, 1, None]) - edges[
                :, 1, None
            ] * (points[:, 0] - inner[:, 0, None])
            self.assertFalse(np.any(np.all(cross > 1e-10, axis=0)))

    def test_containment_allows_offset_but_rejects_wrong_shape_pose(self):
        x, y, z = self.article.goal[:3]
        for p, expected in (
            (pose(x, y + 0.002, z, yaw=0.05), True),
            (pose(x, y, z, yaw=math.pi), False),
            (pose(x, y + 0.008, z), False),
            (pose(x, y, z + 0.010), False),
            (pose(x, y, z - 0.002), False),
            (pose(x, y, z, roll=0.3), False),
        ):
            with self.subTest(p=p):
                self.assertEqual(
                    self.geometry.measure(p, (x, y), 0.14)["insertion_geometry_met"],
                    expected,
                )

    def test_planner_surface_matches_actual_convex_block(self):
        points = article_surface(self.article)
        expected = local_corners(self.article)
        np.testing.assert_allclose(points.min(axis=0), expected.min(axis=0))
        np.testing.assert_allclose(points.max(axis=0), expected.max(axis=0))
        self.assertGreater(len(points), len(expected))

    def evaluator(self):
        return L1Evaluator(
            self.design, L1Settings(), {self.article.name: self.article.initial}
        )

    def update(self, evaluator, t, row):
        # Close hand nearby but no actual contact: no arbitrary 65 mm retreat gate.
        return evaluator.update(
            t, row, tcp=(*self.article.goal[:3], 1, 0, 0, 0), gripper_width=0.055
        )

    def grasp(self, evaluator):
        p = pose(*self.article.initial[:2], 0.12, yaw=math.pi / 4)
        self.update(evaluator, 0, self.row(p, held=True, supported=False))
        self.update(evaluator, 0.3, self.row(p, held=True, supported=False))

    def test_guided_seating_then_release_finishes_without_extra_retreat(self):
        e = self.evaluator()
        self.grasp(e)
        self.update(e, 0.5, self.row(self.article.goal, held=True))
        self.assertEqual(
            self.update(e, 0.6, self.row(self.article.goal)).status, "running"
        )
        self.assertEqual(
            self.update(e, 1.6, self.row(self.article.goal)).status, "succeeded"
        )

    def test_no_grasp_no_guidance_no_support_and_still_held_cannot_pass(self):
        for case in ("no_grasp", "no_guidance", "no_support", "still_held"):
            with self.subTest(case=case):
                e = self.evaluator()
                if case != "no_grasp":
                    self.grasp(e)
                if case not in {"no_grasp", "no_guidance"}:
                    self.update(e, 0.5, self.row(self.article.goal, held=True))
                for t in (0.6, 1.0, 1.6):
                    result = self.update(
                        e,
                        t,
                        self.row(
                            self.article.goal,
                            held=case == "still_held",
                            supported=case != "no_support",
                        ),
                    )
                self.assertNotEqual(result.status, "succeeded")

    def test_dropped_block_ends_attempt_for_shared_reset(self):
        e = self.evaluator()
        self.grasp(e)
        self.update(e, 0.5, self.row(self.article.initial, supported=False))
        result = self.update(e, 0.8, self.row(self.article.initial, supported=False))
        self.assertEqual(result.status, "failed")
        self.assertTrue(result.retryable)


if __name__ == "__main__":
    unittest.main()
