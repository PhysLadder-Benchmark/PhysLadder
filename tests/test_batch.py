"""Batch lifecycle tests use mock episodes/short subprocesses, never GPT or Isaac."""

from contextlib import redirect_stdout
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

from robot_tasks import batch
from robot_tasks.config import load_config
from robot_tasks.runtime_config import RuntimeConfig


class BatchTests(unittest.TestCase):
    def setUp(self):
        # Reproduce Windows CI's legacy text-file default on every platform.
        # Explicit UTF-8 must work without relying on PYTHONUTF8 or the OS locale.
        original_open = Path.open

        def legacy_locale_open(path, mode="r", buffering=-1, encoding=None, errors=None, newline=None):
            if "b" not in mode and encoding in (None, "locale"):
                encoding = "cp1252"
            return original_open(path, mode, buffering, encoding, errors, newline)

        self.enterContext(patch.object(Path, "open", legacy_locale_open))
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name).resolve()
        self.model = self.root / "model.toml"
        self.model.write_text('[gpt]\nbase_url="https://example.org/v1"\nmodel="offline-fixture"\n', encoding="utf-8")
        self.runtime = RuntimeConfig(sys.executable, self.root, self.root, self.root / "runtime.toml", True)

    def main(self, *args):
        with patch.object(batch, "load_runtime", return_value=self.runtime), redirect_stdout(io.StringIO()):
            return batch.main(list(args))

    def dry_batch(self, name="batch", tasks=("L1-01", "L3-11")):
        root = self.root / name
        self.assertEqual(self.main("--tasks", *tasks, "--output", str(root), "--model-config", str(self.model),
                                  "--dry-run", "--max-calls", "7", "--wall-time-s", "30"), 0)
        return root

    def test_env_never_executes_substitutions_and_exports_take_precedence(self):
        env = self.root / ".env"
        env.write_text('export OPENAI_API_KEY="file-secret" # comment\n'
                       'LITERAL=$(touch unexpected) # stays literal\nSINGLE=\'hello # world\'\n', encoding="utf-8")
        loaded = batch.load_env(env, {"OPENAI_API_KEY": "exported-secret"})
        self.assertEqual(loaded["OPENAI_API_KEY"], "exported-secret")
        self.assertEqual(loaded["LITERAL"], "$(touch unexpected)")
        self.assertEqual(loaded["SINGLE"], "hello # world")
        self.assertFalse((self.root / "unexpected").exists())
        env.write_text('SECRET_VALUE_WITHOUT_ASSIGNMENT', encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "line 1") as error:
            batch.load_env(env, {})
        self.assertNotIn("SECRET_VALUE", str(error.exception))

    def test_dry_batch_snapshots_overrides_and_skips_real_only(self):
        root = self.dry_batch()
        config = load_config(root / "configs/L1-01.toml")
        self.assertEqual(config.max_wall_time_s, 30)
        self.assertEqual(config.raw["execution"]["max_decisions"], 7)
        self.assertTrue(config.raw["recording"]["save_video"])
        self.assertEqual(config.raw["policy"]["reasoning_effort"], "medium")
        self.assertTrue(config.robot.asset_path.is_absolute())
        result = json.loads((root / "summary.json").read_text(encoding="utf-8"))
        self.assertEqual(result["counts"]["dry_run"], 1)
        self.assertEqual(result["counts"]["skipped"], 1)
        self.assertIsNone(result["results"][0]["success"])
        self.assertTrue((root / "summary.csv").exists())
        self.assertTrue((root / "summary.md").exists())

    def test_resume_skips_completed_jobs_and_does_not_allow_changed_conditions(self):
        root = self.dry_batch()
        with patch.object(batch, "run_job", side_effect=AssertionError("must not rerun")):
            self.assertEqual(self.main("--resume", str(root)), 0)
        with redirect_stdout(io.StringIO()), patch.object(batch.sys, "stderr", io.StringIO()):
            self.assertEqual(self.main("--resume", str(root), "--max-calls", "8"), 2)
            self.model.write_text(self.model.read_text(encoding="utf-8") + "# changed\n", encoding="utf-8")
            self.assertEqual(self.main("--resume", str(root)), 2)
        self.assertEqual(self.main("--summarize", str(root)), 0)  # Offline reporting is still possible.

    def evidence(self, name, result):
        directory = self.root / name
        episode = directory / "episodes/run"
        episode.mkdir(parents=True)
        (episode / "result.json").write_text(json.dumps(result), encoding="utf-8")
        return directory, episode

    def test_interrupted_job_restarts_in_new_directory_and_preserves_old_evidence(self):
        root = self.dry_batch(tasks=("L1-01",))
        state = json.loads((root / "state.json").read_text(encoding="utf-8"))
        job = state["L1-01"]
        old = Path(job["directory"])
        process = json.loads((old / "process.json").read_text(encoding="utf-8"))
        process.update(interrupted=True, returncode=130)
        batch.write_json(old / "process.json", process)
        job.update(status="interrupted")
        batch.write_json(root / "state.json", state)
        self.assertEqual(self.main("--resume", str(root)), 0)
        new = json.loads((root / "state.json").read_text(encoding="utf-8"))["L1-01"]
        self.assertNotEqual(new["directory"], str(old))
        self.assertTrue(list((old / "episodes").glob("*/result.json")))
        self.assertEqual(len(list((root / "tasks/L1-01").glob("run_*"))), 2)

    def test_saved_process_result_is_recovered_without_duplicate_calls(self):
        root = self.dry_batch(tasks=("L1-01",))
        state = json.loads((root / "state.json").read_text(encoding="utf-8"))
        process_path = Path(state["L1-01"]["directory"]) / "process.json"
        process = json.loads(process_path.read_text(encoding="utf-8"))
        process["note"] = "保留中文记录"
        batch.write_json(process_path, process)
        state["L1-01"].update(status="running")
        state["L1-01"].pop("summary")
        batch.write_json(root / "state.json", state)
        with patch.object(batch, "run_job", side_effect=AssertionError("must recover")):
            self.assertEqual(self.main("--resume", str(root)), 0)
        recovered = json.loads((root / "state.json").read_text(encoding="utf-8"))["L1-01"]
        self.assertEqual(recovered["status"], "done")
        self.assertEqual(recovered["summary"]["process"]["note"], "保留中文记录")

    def test_process_liveness_probe_preserves_live_process_and_accepts_exited_pid(self):
        process = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)"])
        try:
            self.assertTrue(batch.pid_alive(process.pid))
            self.assertIsNone(process.poll())
        finally:
            process.terminate()
            process.wait(timeout=5)
        self.assertFalse(batch.pid_alive(process.pid))

    def test_missing_key_fails_before_creating_batch_or_launching_worker(self):
        root = self.root / "no_key"
        with patch.dict(os.environ, {}, clear=True), patch.object(batch, "run_job") as run, \
                patch.object(batch.sys, "stderr", io.StringIO()):
            self.assertEqual(self.main("--tasks", "L1-01", "--output", str(root), "--model-config", str(self.model),
                                       "--env-file", str(self.root / "missing.env")), 2)
        run.assert_not_called()
        self.assertFalse(root.exists())

    def test_exit_zero_is_not_task_success_and_unknown_usage_is_not_zero(self):
        directory, episode = self.evidence("failed", {"status": "finished", "task_evaluated": True,
                                                     "success": False, "model_calls": 2})
        events = [
            {"event": "model_exchange", "exchange": {"usage": {"input_tokens": 10, "output_tokens": 5, "total_tokens": 15}, "latency_s": 2.}},
            {"event": "model_exchange", "exchange": {"status": "rejected"}},
        ]
        (episode / "episode.jsonl").write_text("\n".join(map(json.dumps, events)) + '\n{"partial":', encoding="utf-8")
        row = batch.collect_result(directory, {"returncode": 0}, False, ["overview"])
        self.assertEqual(row["outcome"], "failed")
        self.assertFalse(row["success"])
        self.assertEqual(row["total_tokens"], 15)
        self.assertEqual(row["usage_samples"], 1)
        self.assertEqual(row["api_latency_s"], 2.)
        self.assertFalse(row["video_complete"])
        self.assertIsNone(row["reasoning_tokens"])
        self.assertEqual(batch.collect_result(directory, {"returncode": 124, "timed_out": True}, False, [])['outcome'], "timeout")
        row = batch.collect_result(self.root / "missing", {"returncode": 0}, False, [])
        self.assertEqual(row["outcome"], "infrastructure_error")
        self.assertIsNone(row["model_calls"])
        self.assertIsNone(row["api_latency_s"])

    def test_video_metadata_is_checked_instead_of_trusting_result_flag(self):
        video = self.root / "motion.mp4"
        video.write_bytes(b"fixture")
        with patch.object(batch.shutil, "which", return_value="ffprobe"), \
                patch.object(batch.subprocess, "run", return_value=subprocess.CompletedProcess([], 0, json.dumps({"streams": [{"width": 640, "height": 480, "duration": "1.2"}]}))):
            self.assertTrue(batch.inspect_video(video)["valid"])
        with patch.object(batch.shutil, "which", return_value=None):
            self.assertIsNone(batch.inspect_video(video)["valid"])
        self.assertFalse(batch.inspect_video(self.root / "absent.mp4")["valid"])

    def test_provider_quota_error_stops_before_next_job(self):
        root = self.dry_batch(tasks=("L1-01", "L1-03"))
        manifest = json.loads((root / "batch.json").read_text(encoding="utf-8"))
        manifest["options"]["dry_run"] = False
        def failed_job(command, directory, environment, timeout, started):
            directory.mkdir(parents=True)
            started({"worker_pid": 99999999})
            episode = directory / "episodes/run"
            episode.mkdir(parents=True)
            (episode / "result.json").write_text(json.dumps({"status": "error", "error": "insufficient_quota"}), encoding="utf-8")
            return {"returncode": 2, "interrupted": False, "cleanup_complete": True}
        with patch.object(batch, "run_job", side_effect=failed_job) as run, redirect_stdout(io.StringIO()):
            self.assertEqual(batch.execute_batch(root, manifest, {}, {}), 2)
        self.assertEqual(run.call_count, 1)
        self.assertEqual(json.loads((root / "summary.json").read_text(encoding="utf-8"))["counts"]["pending"], 1)

    @unittest.skipUnless(sys.platform.startswith("linux"), "Linux process-session ownership fixture")
    def test_timeout_reclaims_separate_session_planner_child(self):
        pidfile = self.root / "child.pid"
        script = ("import subprocess,sys,time; from pathlib import Path; "
                  "p=subprocess.Popen([sys.executable,'-c','import time; time.sleep(60)'],start_new_session=True); "
                  f"Path({str(pidfile)!r}).write_text(str(p.pid),encoding='utf-8'); time.sleep(60)")
        process = batch.run_job([sys.executable, "-c", script], self.root / "timeout", dict(os.environ), 1.5, lambda _: None)
        self.assertTrue(process["timed_out"])
        self.assertTrue(process["cleanup_complete"])
        self.assertIsNone(batch.process_identity(int(pidfile.read_text(encoding="utf-8"))))


if __name__ == "__main__":
    unittest.main()
