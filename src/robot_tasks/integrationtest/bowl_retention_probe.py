"""Isolated final-state scoring check with real pearl contact; no GPT success claim.

Uses scene_smoke startup, then places a pearl over the settled central heap. The
fixture is at the goal with a declared synthetic lift flag only for scorer
verification. It is not robot transport or historical episode rescoring.
"""
import argparse
import json
from pathlib import Path

import numpy as np


def probe(backend, task, output):
    from PIL import Image
    from ..robots.transforms import pose_transform
    from ..tasks.bench.evaluation import BenchEvaluator

    scene = task.scene
    rows = scene.measure()
    old = np.array(rows["bowl"]["pose"][:3])
    target = np.array([*scene.spec.parameters["target_xy"], old[2]])
    delta = target - old
    b = scene.bodies["bowl"]
    b.set_world_pose(target, np.array(rows["bowl"]["pose"][3:]))
    positions, orientations = scene.load_view.get_world_poses()
    positions = np.array(positions) + delta
    # Put one central pearl atop the highest central bead, with a small gap.
    centre_indices = [i for i,p in enumerate(positions)
                      if np.linalg.norm(p[:2] - target[:2]) < .020]
    highest = max(centre_indices, key=lambda i: positions[i,2])
    chosen = next(i for i in centre_indices if i != highest)
    positions[chosen] = positions[highest] + [0., 0., .0082]
    scene.load_view.set_world_poses(positions, orientations)
    scene.load_view.set_velocities(np.zeros((len(positions), 6)))
    scene.cached_time = None
    for _ in range(36):
        backend.step(task.scene.config.control_dt_s)
    e = BenchEvaluator(scene.spec, scene.initial)
    e.flags.add("loaded_bowl_lifted")  # Declared final-state fixture only.
    samples = []
    for _ in range(18):
        backend.step(task.scene.config.control_dt_s)
        a = e.update(backend.time_s(), scene.measure())
        samples.append({"time_s": backend.time_s(), "status": a.status,
                        "criteria": a.metrics["criteria"]})
    for f in backend.capture_rgb():
        Image.frombytes("RGB", (f.width, f.height), f.data).save(output / ("heap_"+f.camera+".png"))
    # Exercise the above-rim exception with real measured support/speed but an
    # explicitly synthetic position; this is scorer input injection, not proof
    # that the physical heap itself protruded in this particular settling trial.
    from ..tasks.bench.scoring_l2 import retained_pearl
    measured = scene.measure()
    candidates = [r for n,r in measured.items() if n.startswith("load_")
                  and r.get("net_contact_force_world_n", [0,0,0])[2] > .0007
                  and np.linalg.norm(r["linear_velocity"]) < .025]
    if not candidates:
        raise RuntimeError("No settled pearl with measured upward support")
    row = candidates[0]
    retained, reason = retained_pearl(np.array([.006179, .014521, .054260]), row, scene.spec.parameters)
    result = {"above_rim_sensor_check": {"retained": retained, "reason": reason,
              "synthetic_local_position": True, "measured_force": row["net_contact_force_world_n"]},
              "gpt_called": False, "scope": __doc__, "moved_pearl": scene.load_names[chosen],
              "samples": samples, "passed": retained and any(s["status"] == "succeeded" for s in samples)}
    (output / "heap_result.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    # Leave scene_smoke's evaluator unmodified: its no-lift initial-state check
    # should still report running even though this diagnostic fixture is at goal.
    if not result["passed"]:
        raise RuntimeError("Settled loaded-bowl fixture did not pass; see heap_result.json")
    return {"passed": True, "fixture_only": True}


if __name__ == "__main__":
    from .scene_smoke import check
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    raise SystemExit(0 if check("L2-04", args.output, prepare=False, capture_hook=probe) else 2)
