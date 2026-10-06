"""Isolated R5 contact/short-motion diagnostic, never a scripted task baseline.

An explicit sampled-IK fixture starts OPEN fingers around the prop. Subsequent
closing and motion use the production adapter/planner. Props are never attached
or teleported. This does not validate visual grasp selection or full task success.
Run in Isaac Python, with --poses pointing to l2_clearance's diagnostic JSON.
"""

import argparse
import json
import os
from dataclasses import asdict, replace
from pathlib import Path

import numpy as np


def check(task_id, poses, output, width=None, velocity_iterations=None, travel=0.020):
    from ..backends.isaacsim import IsaacSimBackend
    from ..registry import resolve_task
    from ..robots.adapters import R5Adapter
    from ..robots.control import width_to_joints
    from ..types import Action, ArmCommand, EvaluationState

    if not np.isfinite(travel) or not 0 < travel <= 0.05:
        raise ValueError("A local probe must travel between 0 and 50 mm")
    os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
    output.mkdir(parents=True, exist_ok=True)
    spec, config = resolve_task(task_id)
    config = spec.factory.prepare_config(config)
    if velocity_iterations is not None:
        config = replace(
            config,
            raw={
                **config.raw,
                "simulation": {
                    **config.raw["simulation"],
                    "solver_velocity_iterations": velocity_iterations,
                },
            },
        )
    fixture = next(
        r for r in json.loads(poses.read_text())["results"] if r["task"] == task_id
    )
    active = ("left", "right") if task_id == "L2-04" else ("left",)
    initial = {a: next(s for s in fixture["samples"] if s["arm"] == a) for a in active}
    config = replace(
        config,
        robot=replace(
            config.robot,
            mounts=tuple(
                replace(m, home_positions_rad=tuple(initial[m.name]["joints"]))
                if m.name in initial
                else m
                for m in config.robot.mounts
            ),
        ),
    )
    backend, robot, task = IsaacSimBackend(), R5Adapter(), spec.factory()
    backend.run_directory = output
    result = {
        "task": task_id,
        "passed": False,
        "gpt_called": False,
        "scope": "open-finger fixture, actual R5 grip and short motion; not full task or GPT success",
        "fixture": initial,
        "checkpoints": [],
    }

    last_time, last_assessment = None, None

    def assess():
        nonlocal last_time, last_assessment
        now = backend.time_s()
        if now != last_time:
            last_assessment = task.evaluate(
                EvaluationState({"simulation_time_s": now}), robot.feedback()
            )
            last_time = now
        return last_assessment

    def capture(label):
        from PIL import Image

        rows = task.scene.measure()
        assessment = assess()
        objects = {
            n: {k: v for k, v in row.items() if k != "corners"}
            for n, row in rows.items()
            if not n.startswith("load_")
        }
        result["checkpoints"].append(
            {
                "label": label,
                "simulation_time_s": backend.time_s(),
                "objects": objects,
                "criteria": assessment.metrics.get("criteria"),
                "assessment_status": assessment.status,
                "assessment_reason": assessment.reason,
                "observer_contact": assessment.metrics.get("observer_contact"),
                "robot": {a: asdict(s) for a, s in robot.state().items()},
                "feedback": asdict(robot.feedback()),
                "planning": robot.last_plan,
            }
        )
        for f in backend.capture_rgb():
            Image.frombytes("RGB", (f.width, f.height), f.data).save(
                output / f"{label}_{f.camera}.png"
            )
        (output / "progress.json").write_text(
            json.dumps(result, ensure_ascii=False), encoding="utf-8"
        )
        print("PROBE_CHECKPOINT", label, assessment.status, flush=True)
        if assessment.status == "invalid":
            raise RuntimeError(assessment.reason)
        return rows

    def execute(action):
        robot.apply(action)
        completed = 0
        for tick in range(120):
            backend.step(config.control_dt_s)
            assessment = assess()
            if assessment.status in {"failed", "invalid"}:
                capture("terminal")
                raise RuntimeError(assessment.reason)
            completed = completed + 1 if robot.feedback().status == "completed" else 0
            if tick % 24 == 23:
                print("PROBE_ACTION", tick + 1, robot.feedback().status, flush=True)
            if completed >= 3:
                return
        capture("timeout")
        robot.on_action_timeout()
        raise RuntimeError("R5 action timeout: " + str(asdict(robot.feedback())))

    try:
        backend.open(config)
        result["solver_velocity_iterations"] = (
            backend.settings.solver_velocity_iterations
        )
        result["finger_collision_overrides"] = backend.environment_report.get(
            "finger_collision_overrides", []
        )
        robot.initialize(backend, config)
        task.build_scene(backend, config)
        result["task_version"] = task.spec.design.version
        result["scoring_protocol"] = config.raw["task"].get("scoring_protocol")
        result["physics_dt_s"] = config.physics_dt_s
        backend.reset(config.seed)
        task.reset(backend, config.seed)
        robot.reset(config.seed)
        # Fixture assignment only, before physics advances; actual close is later.
        fixture_targets = dict(robot.targets)
        for arm in active:
            q = robot.targets[arm][:6] + width_to_joints(config.robot, 0.07)
            robot.instances[arm].set_joint_positions(
                np.asarray(q), joint_indices=robot.indices[arm]
            )
            fixture_targets[arm] = q
        robot._send_targets(fixture_targets)
        for _ in range(12):
            backend.step(config.control_dt_s)
        before = capture("open")
        close = (
            width
            if width is not None
            else {
                "L2-01": 0.012,
                "L2-03": 0.015,
                "L2-04": 0.040,
                "L2-06": 0.023,
                "L2-07": 0.035,
            }[task_id]
        )
        result["close_width_m"] = close
        widths = [close]
        if task_id == "L2-06":
            widths = [w for w in (0.030, 0.026, 0.023, 0.021, 0.019) if w > close] + [
                close
            ]
        result["close_sequence_m"] = widths
        for opening in widths:
            execute(
                Action(
                    tuple(ArmCommand(a, "gripper", (opening,)) for a in active),
                    config.control_dt_s,
                )
            )
        capture("close_command_finished")
        # Deliberately local, not a task-solving trajectory or goal-position helper.
        delta = (
            np.array([0.0, travel, 0.0])
            if task_id in {"L2-03", "L2-07"}
            else np.array([0.0, 0.0, travel])
        )
        result["requested_displacement_m"] = delta.tolist()
        for i in range(2):
            commands = []
            for arm in active:
                state = robot.state()[arm]
                pose = list(state.tcp_pose_world_wxyz)
                pose[:3] = (np.asarray(pose[:3]) + delta / 2).tolist()
                commands.append(
                    ArmCommand(arm, "cartesian_pose", tuple(pose), frame="world")
                )
            execute(Action(tuple(commands), config.control_dt_s))
            after = capture(f"move_{i + 1}")
        name = {
            "L2-01": "body",
            "L2-03": "middle_book",
            "L2-04": "bowl",
            "L2-06": "orange",
            "L2-07": "drawer",
        }[task_id]
        key = "centroid" if task_id == "L2-06" else "pose"
        actual = np.asarray(after[name][key][:3]) - before[name][key][:3]
        result["measured_object_displacement_m"] = actual.tolist()
        result["passed"] = bool(np.linalg.norm(actual - delta) < 0.01)
        if any(
            c["assessment_status"] in {"invalid", "failed"}
            for c in result["checkpoints"]
        ):
            result["passed"] = False
    except Exception as exc:  # noqa: BLE001 - diagnostics must retain simulator/planner failures
        result["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        (output / "result.json").write_text(
            json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        robot.close()
        backend.close()
    print(
        "L2_EXECUTION_PROBE",
        task_id,
        result["passed"],
        result.get("error", ""),
        flush=True,
    )
    return result["passed"]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("task", choices=["L2-01", "L2-03", "L2-04", "L2-06", "L2-07"])
    p.add_argument("--poses", required=True, type=Path)
    p.add_argument("--output", required=True, type=Path)
    p.add_argument("--width", type=float)
    p.add_argument("--velocity-iterations", type=int)
    p.add_argument(
        "--travel",
        type=float,
        default=0.020,
        help="Local motion in metres, at most 0.05",
    )
    args = p.parse_args()
    return (
        0
        if check(
            args.task,
            args.poses,
            args.output,
            args.width,
            args.velocity_iterations,
            args.travel,
        )
        else 2
    )


if __name__ == "__main__":
    raise SystemExit(main())
