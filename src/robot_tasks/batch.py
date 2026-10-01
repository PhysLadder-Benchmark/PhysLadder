"""Serial, resumable GPT experiments; existing CLI/runner remain authoritative."""

import argparse
from contextlib import contextmanager
from copy import deepcopy
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shlex
import shutil
import signal
import subprocess
import sys
import time
from uuid import uuid4

from .config import load_config
from .paths import PROJECT_ROOT
from .policies.model_config import load_gpt_settings
from .registry import TASKS, resolve_task
from .runtime_config import load_runtime


def write_json(path, value):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temporary.replace(path)


def load_env(path, environment):
    """Parse assignments as data, never execute shell code or overwrite exports."""
    if not path.is_file():
        return environment
    result = dict(environment)
    for number, line in enumerate(path.read_text(encoding="utf-8-sig").splitlines(), 1):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        match = re.fullmatch(r"(?:export\s+)?([A-Za-z_][A-Za-z0-9_]*)\s*=\s*(.*)", line)
        if not match:
            raise ValueError(f"Invalid .env assignment at line {number}; values are not displayed")
        key, value = match.groups()
        if value.startswith(('"', "'")):
            try:
                parts = shlex.split(value, comments=True, posix=True)
            except ValueError:
                raise ValueError(f"Invalid .env quoting at line {number}") from None
            if len(parts) != 1:
                raise ValueError(f"Invalid .env value at line {number}")
            value = parts[0]
        else:
            value = re.split(r"\s+#", value, maxsplit=1)[0].strip()
        result.setdefault(key, value)
    return result


def toml_text(data):
    """Small writer for the existing scalar/list/table configuration schema."""
    def value(item):
        if isinstance(item, bool):
            return str(item).lower()
        if isinstance(item, str):
            return json.dumps(item, ensure_ascii=False)
        if isinstance(item, (int, float)) and math.isfinite(item):
            return str(item)
        if isinstance(item, (list, tuple)):
            return "[" + ", ".join(value(v) for v in item) + "]"
        raise ValueError("Unsupported value in experiment configuration")
    lines = []
    def table(items, prefix=()):
        if prefix:
            lines.append("[" + ".".join(json.dumps(k) for k in prefix) + "]")
        lines.extend(f"{json.dumps(k)} = {value(v)}" for k, v in items.items() if not isinstance(v, dict))
        lines.append("")
        for key, item in items.items():
            if isinstance(item, dict):
                table(item, prefix + (key,))
    table(data)
    return "\n".join(lines)


def freeze_config(task_id, destination, model, workflow, options):
    _, config = resolve_task(task_id)
    raw = deepcopy(config.raw)
    raw["workflow_profile"] = str(workflow)
    for section, key in (("robot", "profile"), ("simulation", "camera_profile"),
                         ("simulation", "asset_directory")):
        if raw.get(section, {}).get(key):
            raw[section][key] = str((config.source.parent / raw[section][key]).resolve())
    raw["policy"].update(model_config=str(model), reasoning_effort=options["reasoning_effort"])
    raw["runtime"].update(seed=options["seed"], max_wall_time_s=options["wall_time_s"])
    raw["execution"]["max_decisions"] = options["max_calls"]
    raw["recording"].update(save_rgb=True, save_depth=True, save_video=True)
    destination.write_text(toml_text(raw), encoding="utf-8")
    load_config(destination)  # Validate relocated paths and merged overrides before any API call.


def fingerprint(paths):
    return {str(p.resolve()): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(set(paths))}


@contextmanager
def batch_lock(root):
    """OS locks are released after crashes, unlike sentinel PID files."""
    with (root / ".batch.lock").open("a+b") as stream:
        try:
            if os.name == "nt":
                import msvcrt
                stream.write(b"0")
                stream.flush()
                stream.seek(0)
                msvcrt.locking(stream.fileno(), msvcrt.LK_NBLCK, 1)
            else:
                import fcntl
                fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            raise ValueError("This batch is already running") from None
        yield


def process_identity(pid):
    try:
        fields = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
        return fields[19] if fields[0] != "Z" else None  # Linux starttime, PID reuse protection.
    except OSError:
        return None


def descendants(pid):
    """Include cuRobo workers which launch their own POSIX sessions."""
    parents, identities = {}, {}
    for stat in Path("/proc").glob("[0-9]*/stat"):
        try:
            fields = stat.read_text().rsplit(")", 1)[1].split()
            child = int(stat.parent.name)
            if fields[0] != "Z":
                parents[child], identities[child] = int(fields[1]), fields[19]
        except (OSError, ValueError):
            continue
    owned, frontier = {}, {pid}
    while frontier:
        frontier = {child for child, parent in parents.items() if parent in frontier and child not in owned}
        owned.update({child: identities[child] for child in frontier})
    return owned


def pid_alive(pid):
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


def stop_process(process, owned):
    from .integrationtest.simulation_process import _stop_worker, _uninterrupted_cleanup
    with _uninterrupted_cleanup():
        forced, complete = _stop_worker(process, grace_s=30, kill_timeout_s=5)
        # The ordinary runner closes its planner. If forced, reclaim separately
        # sessioned descendants observed while the worker was still alive.
        for pid, identity in owned.items():
            if process_identity(pid) == identity:
                try:
                    os.kill(pid, signal.SIGKILL)
                    forced = True
                except ProcessLookupError:
                    pass
        deadline = time.monotonic() + 5
        while any(process_identity(pid) == identity for pid, identity in owned.items()):
            if time.monotonic() >= deadline:
                complete = False
                break
            time.sleep(.1)
    return forced, complete


def run_job(command, directory, environment, timeout, started):
    directory.mkdir(parents=True)
    options = {"creationflags": subprocess.CREATE_NEW_PROCESS_GROUP} if os.name == "nt" else {"start_new_session": True}
    start, owned = time.monotonic(), {}
    next_progress = start + 30
    record = {"returncode": None, "timed_out": False, "interrupted": False, "cleanup_complete": True}
    with (directory / "process.log").open("w", encoding="utf-8") as log:
        process = subprocess.Popen(command, cwd=PROJECT_ROOT, env=environment, stdout=log, stderr=subprocess.STDOUT, **options)
        record.update(worker_pid=process.pid, worker_identity=process_identity(process.pid))
        try:
            started(record)
            while True:
                if time.monotonic() >= next_progress:
                    print(f"BATCH_PROGRESS task={directory.parent.name} elapsed_s={time.monotonic() - start:.0f}", flush=True)
                    next_progress = time.monotonic() + 30
                found = descendants(process.pid)
                if any(owned.get(pid) != identity for pid, identity in found.items()):
                    owned.update(found)
                    record["owned_processes"] = owned.copy()
                    started(record)
                remaining = timeout - (time.monotonic() - start)
                if remaining <= 0:
                    raise subprocess.TimeoutExpired(command[0], timeout)
                try:
                    record["returncode"] = process.wait(timeout=min(1, remaining))
                    break
                except subprocess.TimeoutExpired:
                    continue
        except (KeyboardInterrupt, subprocess.TimeoutExpired) as exc:
            record["interrupted"] = isinstance(exc, KeyboardInterrupt)
            record["timed_out"] = not record["interrupted"]
            record["returncode"] = 130 if record["interrupted"] else 124
            record["forced_termination"], record["cleanup_complete"] = stop_process(process, owned)
        except BaseException:
            stop_process(process, owned)
            raise
        else:
            # Also reclaim leaked descendants after a native crash/abrupt exit.
            if any(process_identity(pid) == identity for pid, identity in owned.items()):
                record["forced_termination"], record["cleanup_complete"] = stop_process(process, owned)
    record["wall_time_s"] = time.monotonic() - start
    write_json(directory / "process.json", record)
    return record


def inspect_video(path):
    if not path.is_file() or not path.stat().st_size:
        return {"valid": False, "reason": "missing_or_empty"}
    probe = shutil.which("ffprobe")
    if not probe:
        return {"valid": None, "reason": "ffprobe_unavailable"}
    try:
        result = subprocess.run([probe, "-v", "error", "-select_streams", "v:0", "-show_entries",
                                 "stream=width,height,nb_frames,duration", "-of", "json", str(path)],
                                capture_output=True, text=True, timeout=15, check=False)
        stream = json.loads(result.stdout).get("streams", [{}])[0]
        valid = (result.returncode == 0 and stream.get("width", 0) > 0
                 and stream.get("height", 0) > 0 and float(stream.get("duration", 0)) > 0)
        return {"valid": valid, **stream}
    except (OSError, ValueError, IndexError, subprocess.TimeoutExpired):
        return {"valid": False, "reason": "unreadable_video"}


def collect_result(directory, process, dry_run, cameras):
    episodes = sorted(directory.glob("episodes/*/result.json"))
    result, episode = {}, None
    if len(episodes) == 1:
        try:
            result = json.loads(episodes[0].read_text(encoding="utf-8"))
            if not isinstance(result, dict):
                result = {}
            episode = episodes[0].parent
        except (OSError, ValueError):
            pass
    if process.get("interrupted"):
        outcome = "interrupted"
    elif process.get("timed_out"):
        outcome = "timeout"
    elif not process.get("cleanup_complete", True) or process.get("returncode") != 0:
        outcome = "infrastructure_error"
    elif dry_run and result.get("status") == "dry_run_complete":
        outcome = "dry_run"
    elif result.get("status") == "finished" and result.get("task_evaluated") is True and type(result.get("success")) is bool:
        outcome = "succeeded" if result["success"] else "failed"
    else:
        outcome = "infrastructure_error"
    tokens, exchanges, latency, usage_samples, latency_samples = {}, 0, 0., 0, 0
    if episode and (episode / "episode.jsonl").exists():
        with (episode / "episode.jsonl").open(encoding="utf-8") as stream:
            for line in stream:
                try:
                    event = json.loads(line)
                except ValueError:
                    continue
                if event.get("event") == "model_exchange":
                    exchange = event.get("exchange", {})
                    exchanges += 1
                    usage = exchange.get("usage", {})
                    usage_samples += bool(usage)
                    for key, value in usage.items():
                        if type(value) is int:
                            tokens[key] = tokens.get(key, 0) + value
                    duration = exchange.get("latency_s")
                    if isinstance(duration, (int, float)) and not isinstance(duration, bool):
                        latency += duration
                        latency_samples += 1
    videos = {name: {"path": str(episode / "video" / name / "motion.mp4"),
                     **inspect_video(episode / "video" / name / "motion.mp4")}
              for name in cameras} if episode and not dry_run else {}
    return {"outcome": outcome, "episode": str(episode) if episode else None,
            "success": result.get("success") if outcome in {"succeeded", "failed"} else None,
            "first_attempt_success": result.get("first_attempt_success"),
            "model_calls": result.get("model_calls", exchanges if exchanges else None), "usage_samples": usage_samples,
            "tokens": tokens, **{key: tokens.get(key) for key in ("input_tokens", "output_tokens", "total_tokens", "cached_tokens", "reasoning_tokens")},
            "api_latency_s": latency if latency_samples else None,
            "termination_reason": result.get("termination_reason", outcome),
            "attempt_count": result.get("attempt_count"), "reset_count": result.get("reset_count"),
            "action_timeouts": result.get("action_timeouts"), "rejected_actions": result.get("rejected_actions"),
            "invalid_model_actions": result.get("invalid_model_actions"),
            "wall_time_s": process.get("wall_time_s"), "simulation_time_s": result.get("simulation_time_s"),
            "error": result.get("error") if result else "No unique readable episode result.json was saved",
            "videos": videos,
            "video_complete": all(v["valid"] is True for v in videos.values()) if videos else None,
            "assessment": result.get("assessment"), "process": process}


def summarize(root, manifest, state):
    rows = []
    for task in manifest["tasks"]:
        job = state.get(task["id"], {})
        row = {"task_id": task["id"], "name": task["name"], "outcome": "pending", **job.get("summary", {})}
        if task.get("skip_reason"):
            row.update(outcome="skipped", termination_reason=task["skip_reason"])
        rows.append(row)
    counts = {key: sum(r["outcome"] == key for r in rows)
              for key in ("succeeded", "failed", "infrastructure_error", "timeout", "interrupted", "dry_run", "skipped", "pending")}
    counts["video_incomplete"] = sum(r["outcome"] not in {"pending", "skipped", "dry_run"}
                                     and r.get("video_complete") is not True for r in rows)
    write_json(root / "summary.json", {"counts": counts, "results": rows})
    fields = ["task_id", "name", "outcome", "success", "first_attempt_success", "model_calls", "usage_samples",
              "wall_time_s", "simulation_time_s", "attempt_count", "reset_count", "action_timeouts",
              "rejected_actions", "invalid_model_actions", "input_tokens", "output_tokens", "total_tokens",
              "cached_tokens", "reasoning_tokens", "api_latency_s", "video_complete", "termination_reason", "episode"]
    with (root / "summary.csv").open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    lines = ["# GPT 批量测评", "", f"模型：{manifest['model']}；每任务最多 {manifest['options']['max_calls']} 次调用。",
             "", "dry_run 不调用 GPT、不运行物理，也不代表任务成功。Token 为已返回 usage 的请求之和；缺失用量不填零。",
             "", "| 任务 | 结果 | 首次成功 | 调用 | 视频 | 报告 |", "| --- | --- | --- | --- | --- | --- |"]
    for row in rows:
        episode = row.get("episode")
        report = f"[报告]({Path(episode).relative_to(root).as_posix()}/report.md)" if episode else "—"
        links = " ".join(f"[{name}]({Path(v['path']).relative_to(root).as_posix()})" for name, v in row.get("videos", {}).items()) or "—"
        lines.append(f"| {row['task_id']} {row['name']} | {row['outcome']} | {row.get('first_attempt_success', '—')} | {row.get('model_calls', '—')} | {links} | {report} |")
    (root / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return counts


def create_batch(root, args, model, runtime):
    ids = args.tasks or sorted(key for key in TASKS if key.startswith("L"))
    if len(set(ids)) != len(ids) or any(key not in TASKS for key in ids):
        raise ValueError("Task IDs must be registered and unique")
    options = {"max_calls": args.max_calls or 100, "wall_time_s": args.wall_time_s or 3600.,
               "startup_grace_s": args.startup_grace_s if args.startup_grace_s is not None else 300.,
               "seed": args.seed if args.seed is not None else 0, "reasoning_effort": args.reasoning_effort or "medium",
               "dry_run": bool(args.dry_run), "gui": bool(args.gui)}
    root.mkdir(parents=True, exist_ok=False)
    configs = root / "configs"
    configs.mkdir()
    model_copy, workflow = configs / "model.toml", configs / "workflow.toml"
    shutil.copyfile(model, model_copy)
    shutil.copyfile(PROJECT_ROOT / "configs/workflows/gpt_default.toml", workflow)
    inputs = list((PROJECT_ROOT / "src/robot_tasks").rglob("*.py"))
    inputs += list((PROJECT_ROOT / "configs/robots").glob("*.toml")) + list((PROJECT_ROOT / "configs/cameras").glob("*.toml"))
    inputs += [model, PROJECT_ROOT / "configs/workflows/gpt_default.toml"]
    tasks = []
    for key in ids:
        spec = TASKS[key]
        task = {"id": key, "name": spec.name, "cameras": []}
        inputs.append(spec.default_config)
        if not spec.implemented:
            task["skip_reason"] = spec.evidence
        else:
            path = configs / f"{key}.toml"
            freeze_config(key, path, model_copy, workflow, options)
            task.update(config=str(path), cameras=list(load_config(path).cameras))
        tasks.append(task)
    inputs += list(configs.glob("*.toml"))
    manifest = {"version": 1, "created_at": datetime.now(timezone.utc).isoformat(),
                "model": load_gpt_settings(model).model, "python": runtime.python,
                "transport": load_gpt_settings(model).transport,
                "model_config": str(model_copy), "options": options, "tasks": tasks, "inputs": fingerprint(inputs)}
    write_json(root / "batch.json", manifest)
    write_json(root / "state.json", {})
    return manifest, {}


def execute_batch(root, manifest, state, environment):
    options = manifest["options"]
    for task in manifest["tasks"]:
        if task.get("skip_reason"):
            continue
        previous = state.get(task["id"], {})
        recorded_process = previous.get("summary", {}).get("process", previous)
        if previous.get("status") != "done" or recorded_process.get("cleanup_complete") is False:
            pid, identity = recorded_process.get("worker_pid"), recorded_process.get("worker_identity")
            if pid and pid_alive(pid) and (identity is None or process_identity(pid) == identity):
                raise ValueError(f"{task['id']} worker is still alive; stop it before resuming")
            for child, identity in recorded_process.get("owned_processes", {}).items():
                if process_identity(int(child)) == identity:
                    raise ValueError(f"{task['id']} planner/child is still alive; stop it before resuming")
        if previous.get("status") == "done":
            continue  # Failures also count as completed observations; never cherry-pick.
        # Recover a completed result after a supervisor crash before state was saved.
        if previous.get("directory") and (Path(previous["directory"]) / "process.json").is_file():
            directory = Path(previous["directory"])
            process = json.loads((directory / "process.json").read_text())
            if not process.get("interrupted"):
                previous.update(status="done", summary=collect_result(directory, process, options["dry_run"], task["cameras"]))
                state[task["id"]] = previous
                write_json(root / "state.json", state)
                summarize(root, manifest, state)
                continue
        directory = root / "tasks" / task["id"] / ("run_" + uuid4().hex[:8])
        command = [manifest["python"], "-m", "robot_tasks", "dry-run" if options["dry_run"] else "run",
                   task["id"], "--config", task["config"], "--output", str(directory / "episodes")]
        if not options["dry_run"] and options["gui"]:
            command.append("--gui")
        print(f"BATCH_START {task['id']} log={directory / 'process.log'}", flush=True)
        def started(process):
            state[task["id"]] = {"status": "running", "directory": str(directory), **process}
            write_json(root / "state.json", state)
        process = run_job(command, directory, environment, options["wall_time_s"] + options["startup_grace_s"], started)
        summary = collect_result(directory, process, options["dry_run"], task["cameras"])
        state[task["id"]].update(status="interrupted" if process["interrupted"] else "done", summary=summary)
        write_json(root / "state.json", state)
        summarize(root, manifest, state)
        print(f"BATCH_RESULT {task['id']} outcome={summary['outcome']} calls={summary['model_calls']} video_complete={summary['video_complete']}", flush=True)
        if process["interrupted"]:
            return 130
        if not process["cleanup_complete"]:
            print("Batch stopped: worker cleanup could not be confirmed", flush=True)
            return 2
        error = (summary.get("error") or "").lower()
        if any(code in error for code in ("responses http 401", "responses http 403", "invalid_api_key",
                                         "insufficient_quota", "quota_exceeded", "billing_hard_limit", "额度已耗尽", "限额已用完",
                                         "codex chatgpt call failed", "codex chatgpt decision timed out",
                                         "subscription_sharing_usage")):
            print("Batch stopped: provider authentication/permission/quota error; fix it before --resume", flush=True)
            return 2
    counts = summarize(root, manifest, state)
    return 2 if any(counts[key] for key in ("infrastructure_error", "timeout", "interrupted", "pending", "video_incomplete")) else 0


def main(argv=None):
    parser = argparse.ArgumentParser(description="串行运行 GPT 任务并录制视频；默认覆盖全部仿真任务")
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument("--all", action="store_true", help="全部任务；仅真机任务记录为 skipped（默认）")
    selection.add_argument("--tasks", nargs="+", help="指定任务，如 L1-01 L2-01")
    action = parser.add_mutually_exclusive_group()
    action.add_argument("--resume", type=Path, help="续跑已有批次；保留原预算，跳过已结束任务")
    action.add_argument("--summarize", type=Path, help="重新检查结果/录像并汇总，不启动任务")
    parser.add_argument("--output", type=Path, help="新批次目录，必须尚不存在")
    parser.add_argument("--env-file", type=Path, default=PROJECT_ROOT / ".env")
    parser.add_argument("--model-config", type=Path)
    parser.add_argument("--max-calls", type=int)
    parser.add_argument("--wall-time-s", type=float)
    parser.add_argument("--startup-grace-s", type=float)
    parser.add_argument("--seed", type=int)
    parser.add_argument("--reasoning-effort", choices=("low", "medium", "high", "xhigh", "max"))
    parser.add_argument("--dry-run", action="store_true", default=None, help="仅验证批量流程；不调用 API、不运行 Isaac")
    parser.add_argument("--gui", action="store_true", default=None)
    args = parser.parse_args(argv)
    try:
        if args.max_calls is not None and not 1 <= args.max_calls <= 1000:
            raise ValueError("--max-calls must be in [1, 1000]")
        for name in ("wall_time_s", "startup_grace_s"):
            value = getattr(args, name)
            if value is not None and (not math.isfinite(value) or value < (0 if name == "startup_grace_s" else .001)):
                raise ValueError(f"--{name.replace('_', '-')} must be finite and positive (grace may be zero)")
        if args.seed is not None and args.seed < 0:
            raise ValueError("--seed must be nonnegative")
        resuming = args.resume or args.summarize
        if resuming:
            if any(getattr(args, key) is not None for key in ("tasks", "output", "model_config", "max_calls", "wall_time_s", "startup_grace_s", "seed", "reasoning_effort", "dry_run", "gui")) or args.all:
                raise ValueError("Resume/summarize uses the saved experiment; do not override its options")
            root = resuming.expanduser().resolve()
            manifest = json.loads((root / "batch.json").read_text(encoding="utf-8"))
            with batch_lock(root):
                state = json.loads((root / "state.json").read_text(encoding="utf-8"))
                if args.summarize:
                    for task in manifest["tasks"]:
                        job = state.get(task["id"], {})
                        if job.get("directory") and job.get("summary"):
                            job["summary"] = collect_result(Path(job["directory"]), job["summary"]["process"], manifest["options"]["dry_run"], task["cameras"])
                    write_json(root / "state.json", state)
                    summarize(root, manifest, state)
                    return 0
                for path, digest in manifest["inputs"].items():
                    if not Path(path).is_file() or hashlib.sha256(Path(path).read_bytes()).hexdigest() != digest:
                        raise ValueError(f"Code/config changed since batch creation: {path}; start a new batch")
                return execute_batch(root, manifest, state, worker_environment(args.env_file, manifest))
        runtime = load_runtime()
        model = (args.model_config or PROJECT_ROOT / "configs/models/gpt.toml").expanduser().resolve()
        settings = load_gpt_settings(model)
        if not Path(runtime.python).is_file():
            raise ValueError("Configured Isaac Python does not exist")
        # Fail before creating a batch when credentials are absent.
        environment = worker_environment(args.env_file, {"options": {"dry_run": bool(args.dry_run)}, "model_config": str(model)})
        name = "batch_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ_") + uuid4().hex[:8]
        root = (args.output or runtime.output_dir / name).expanduser().resolve()
        manifest, state = create_batch(root, args, model, runtime)
        with batch_lock(root):
            summarize(root, manifest, state)
            print(f"BATCH_OUTPUT {root} model={settings.model} dry_run={bool(args.dry_run)}", flush=True)
            return execute_batch(root, manifest, state, environment)
    except KeyboardInterrupt:
        return 130
    except (OSError, ValueError, KeyError, RuntimeError) as exc:
        print(f"Batch error: {exc}", file=sys.stderr)
        return 2


def worker_environment(env_file, manifest):
    environment = dict(os.environ)
    if not manifest["options"]["dry_run"]:
        settings = load_gpt_settings(Path(manifest["model_config"]))
        if settings.transport == "codex_cli":
            from .policies.codex_cli import CodexCLIClient
            CodexCLIClient(settings).require_chatgpt_login()
        else:
            environment = load_env(env_file.expanduser().resolve(), environment)
            # Validate without changing the supervisor's environment or writing the key.
            if not environment.get(settings.api_key_env, "").strip():
                raise ValueError(f"Missing {settings.api_key_env}; export it or provide --env-file")
            if any(c.isspace() for c in environment[settings.api_key_env].strip()):
                raise ValueError(f"Invalid whitespace in {settings.api_key_env}")
    environment.update(PYTHONPATH=str(PROJECT_ROOT / "src") + os.pathsep + environment.get("PYTHONPATH", ""),
                       PYTHONUTF8="1", PYTHONUNBUFFERED="1")
    return environment


if __name__ == "__main__":
    raise SystemExit(main())
