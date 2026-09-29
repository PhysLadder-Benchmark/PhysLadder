"""Summarize recorded attempts without rerunning models or selecting only successes."""

import argparse
import hashlib
import json
from pathlib import Path
import statistics
import tomllib


def summarize(directory: Path) -> dict:
    result = json.loads((directory / "result.json").read_text(encoding="utf-8"))
    manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
    with (directory / "config.toml").open("rb") as stream:
        config = tomllib.load(stream)
    events = [json.loads(line) for line in (directory / "episode.jsonl").read_text(encoding="utf-8").splitlines()]
    policy = next((e["configuration"] for e in events if e["event"] == "policy_configuration"), {})
    scene_path = directory / "scene.json"
    scene = json.loads(scene_path.read_text(encoding="utf-8")) if scene_path.is_file() else {}
    errors, calls = [], []
    for event in events:
        if event["event"] == "transition":
            for error in event["observation"]["feedback"].get("metrics", {}).get("tcp", {}).values():
                errors.append(error["position_error_m"])
        if event["event"] == "model_exchange":
            calls.append(event["exchange"])
    return {"directory": str(directory), "task": result["task_id"], "seed": manifest["seed"],
            "success": result.get("success"), "status": result["status"], "termination_reason": result["termination_reason"],
            "model": policy.get("provider", {}).get("model"),
            "reasoning_effort": policy.get("provider", {}).get("reasoning_effort"),
            "options": policy.get("options", {}),
            "gripper_timing": config.get("execution", {}).get("gripper_timing", "simultaneous"),
            "ik_solver": scene.get("ik_solver", config.get("execution", {}).get("ik_solver", "not_recorded")),
            "budgets": {"calls": config.get("execution", {}).get("max_decisions"),
                        "control_ticks": config["runtime"]["max_steps"], "wall_s": config["runtime"]["max_wall_time_s"]},
            "robot_profile_sha256": manifest["robot_profile_sha256"], "object_and_scoring": config.get("interaction", {}),
            "camera_profile_sha256": (hashlib.sha256((directory / "camera_profile.toml").read_bytes()).hexdigest()
                                      if (directory / "camera_profile.toml").exists() else None),
            "source_sha256": manifest.get("source_sha256", {}),
            "model_calls": result["model_calls"], "action_rejections": result["rejected_actions"],
            "action_timeouts": result["action_timeouts"], "false_completion": result["false_completion"],
            "wall_time_s": result["wall_time_s"], "simulation_time_s": result["simulation_time_s"],
            "mean_observed_tcp_goal_error_m": statistics.mean(errors) if errors else None,
            "input_tokens": sum(e.get("usage", {}).get("input_tokens", 0) for e in calls),
            "output_tokens": sum(e.get("usage", {}).get("output_tokens", 0) for e in calls)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("runs", type=Path, nargs="+")
    args = parser.parse_args()
    rows = [summarize(path) for path in args.runs]
    matching_fields = ("task", "seed", "model", "budgets", "robot_profile_sha256", "camera_profile_sha256", "object_and_scoring")
    differences = [name for name in matching_fields if any(row[name] != rows[0][name] for row in rows[1:])]
    for row in rows:
        hashable_source = json.dumps(row.pop("source_sha256"), sort_keys=True)
        row["source_version"] = hashlib.sha256(hashable_source.encode()).hexdigest()
    print(json.dumps({"runs": rows, "unmatched_fields": differences,
                      "interpretation": "Engineering attempts, not an estimated success rate. Match initial states, sensor profiles, budgets and conditions; repeat layouts before claiming improvement."},
                     ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
