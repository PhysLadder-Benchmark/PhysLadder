"""Run a registered check in an isolated process with a wall-time watchdog."""

from datetime import datetime, timezone
import json
import os
import signal
from pathlib import Path
import subprocess
import sys
import threading
from uuid import uuid4

from ..paths import PROJECT_ROOT
from ..recording import write_json
from .registry import get_check
from .scene_preview import PREVIEW_READY


def completed_check(output: Path, returncode: int) -> bool:
    """An exit code of zero alone is not evidence that a physical check passed."""
    try:
        result = json.loads((output / "result.json").read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return False
    return (isinstance(result, dict) and returncode == 0
            and result.get("status") == "completed" and result.get("passed") is True)


def run_check_process(name: str, output_root: Path, *, config_path: Path | None = None,
                      gui: bool = False, model_config: Path | None = None) -> tuple[Path, bool]:
    spec = get_check(name)
    spec.validate_options(gui=gui, model_config=model_config)
    config = spec.load_config(config_path)
    if spec.preflight is not None:
        spec.preflight(config.robot)
    output = output_root / (spec.output_prefix + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ_") + uuid4().hex[:8])
    output.mkdir(parents=True, exist_ok=False)
    command = [sys.executable, "-m", "robot_tasks.integrationtest.simulation_worker", "--config", str(config.source),
               "--run-directory", str(output.resolve()), "--check", spec.name]
    if gui:
        command.append("--gui")
    if model_config is not None:
        command.extend(("--model-config", str(model_config.resolve())))
    environment = dict(os.environ, PYTHONUTF8="1", PYTHONUNBUFFERED="1")
    timed_out = threading.Event()
    interactive_scene = spec.name == "l1-scene" and gui
    preview_started = False
    process_options = {}
    if interactive_scene:
        # Forward Ctrl+C once to the viewer, instead of interrupting both its
        # render loop and cleanup via the shared terminal process group.
        if os.name == "nt":
            process_options["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP
        else:
            process_options["start_new_session"] = True
    with (output / "process.log").open("w", encoding="utf-8") as log:
        with subprocess.Popen(command, cwd=PROJECT_ROOT, env=environment,
                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                              text=True, encoding="utf-8", errors="replace", **process_options) as process:
            def terminate_on_timeout():
                timed_out.set()
                process.kill()
            timer = threading.Timer(config.max_wall_time_s + 60, terminate_on_timeout)
            timer.daemon = True
            timer.start()
            try:
                for line in process.stdout:
                    log.write(line)
                    log.flush()
                    if interactive_scene and line.strip() == PREVIEW_READY:
                        preview_started = True
                        timer.cancel()
                    print(line, end="", flush=True)
                returncode = process.wait()
            except KeyboardInterrupt:
                if not preview_started:
                    process.kill()
                    process.wait()
                    raise
                ignored = {s: signal.signal(s, signal.SIG_IGN) for s in
                           (signal.SIGINT, *([signal.SIGBREAK] if hasattr(signal, "SIGBREAK") else []))}
                try:
                    if process.poll() is None:
                        process.send_signal(signal.CTRL_BREAK_EVENT if os.name == "nt" else signal.SIGINT)
                    try:
                        returncode = process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        returncode = process.wait()
                finally:
                    for signum, handler in ignored.items():
                        signal.signal(signum, handler)
            except BaseException:
                process.kill()
                process.wait()
                raise
            finally:
                timer.cancel()
    from ..runtime import snapshot_runtime_config
    snapshot_runtime_config(output)
    passed = completed_check(output, returncode) and not timed_out.is_set()
    write_json(output / "process.json", {"returncode": returncode, "passed": passed,
                                        "timed_out": timed_out.is_set(), "worker_pid": process.pid})
    from ..reporting import write_run_report
    write_run_report(output)
    print("R5_PROCESS_RESULT " + json.dumps({"output": str(output), "passed": passed,
                                            "returncode": returncode}), flush=True)
    return output, passed
