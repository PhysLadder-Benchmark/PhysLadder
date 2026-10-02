"""Offline native/WSL process lifecycle checks. Never call a model or account."""

import os
import hashlib
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

from robot_tasks.policies import codex_process


class CodexProcessTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        settings = patch.object(codex_process, "_local_settings", return_value={})
        settings.start()
        self.addCleanup(settings.stop)

    def test_windows_without_native_codex_selects_wsl_and_translates_paths(self):
        def which(name):
            return "wsl.exe" if name == "wsl.exe" else None
        converted = subprocess.CompletedProcess([], 0, b"/mnt/d/a space/worker.py\n", b"")
        with patch.object(codex_process, "_WINDOWS", True), \
                patch.object(codex_process.shutil, "which", side_effect=which), \
                patch.object(codex_process.subprocess, "run", return_value=converted):
            host = codex_process.CodexProcess("codex")
            command = host.command(["login", "status"], self.root, 15)
        self.assertEqual(host.mode, "wsl")
        self.assertEqual(command[:4], ["wsl.exe", "--exec", "python3", "/mnt/d/a space/worker.py"])
        self.assertEqual(command[-3:], ["--", "login", "status"])

    def test_explicit_wsl_account_takes_precedence_over_windows_install(self):
        with patch.object(codex_process, "_WINDOWS", True), \
                patch.object(codex_process, "_local_settings", return_value={
                    "host": "wsl", "wsl_distribution": "Ubuntu", "wsl_user": "researcher"}), \
                patch.object(codex_process.shutil, "which", side_effect=lambda name: name), \
                patch.object(codex_process.CodexProcess, "path", return_value=Path("/worker.py")):
            host = codex_process.CodexProcess("codex")
        self.assertEqual(host.prefix, ["wsl.exe", "--distribution", "Ubuntu", "--user", "researcher", "--exec"])

    def test_native_missing_executable_does_not_silently_switch_provider(self):
        with patch.object(codex_process, "_WINDOWS", False), \
                patch.object(codex_process.shutil, "which", return_value=None):
            with self.assertRaisesRegex(RuntimeError, "Codex CLI not found"):
                codex_process.CodexProcess("missing")

    def delayed_input_fixture(self):
        # Larger than a pipe buffer, with a reader delayed beyond the 100 ms
        # polling interval. Python 3.10 used to stop sending after that timeout.
        payload = ("observation: \u6293\u53d6\u628a\u624b\n" * 32768).encode("utf-8")
        script = ("import hashlib,sys,time\n"
                  "time.sleep(.35)\n"
                  "data=sys.stdin.buffer.read()\n"
                  "print(str(len(data)) + ':' + hashlib.sha256(data).hexdigest(),flush=True)\n")
        expected = f"{len(payload)}:{hashlib.sha256(payload).hexdigest()}".encode()
        return payload, script, expected

    def test_native_large_prompt_reaches_eof_without_truncation_or_duplication(self):
        payload, script, expected = self.delayed_input_fixture()
        with patch.object(codex_process, "_local_settings", return_value={"host": "native"}):
            host = codex_process.CodexProcess(sys.executable)
        lines = []
        result = host.run(["-c", script], directory=self.root, data=payload, timeout_s=3,
                          on_stdout_line=lines.append)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), expected)
        self.assertEqual([line.strip() for line in lines], [expected])

    @unittest.skipIf(os.name == "nt", "WSL worker requires POSIX")
    def test_wsl_worker_large_prompt_reaches_eof_after_slow_reader_startup(self):
        payload, script, expected = self.delayed_input_fixture()
        result = subprocess.run(self.worker_command(sys.executable, 3, "-c", script),
                                input=payload, capture_output=True, timeout=5)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), expected)

    def test_wsl_proxy_is_explicitly_forwarded_to_the_supervisor(self):
        with patch.object(codex_process, "_WINDOWS", True), \
                patch.object(codex_process, "_local_settings", return_value={
                    "host": "wsl", "wsl_proxy": "http://172.20.0.1:7899"}), \
                patch.object(codex_process.shutil, "which", side_effect=lambda name: name), \
                patch.object(codex_process.CodexProcess, "path", return_value=Path("/worker.py")):
            host = codex_process.CodexProcess("codex")
            command = host.command(["exec"], self.root, 10)
        self.assertEqual(command[command.index("--proxy") + 1], "http://172.20.0.1:7899")

    def test_keyboard_interrupt_requests_wsl_cleanup(self):
        with patch.object(codex_process, "_WINDOWS", False), \
                patch.object(codex_process.shutil, "which", return_value="fixture"):
            host = codex_process.CodexProcess("codex")
        host.mode = "wsl"
        with patch.object(host, "command", return_value=["fixture"]), \
                patch.object(codex_process.subprocess, "Popen") as popen:
            process = popen.return_value
            process.wait.side_effect = [KeyboardInterrupt, None]
            process.poll.return_value = None
            with self.assertRaises(KeyboardInterrupt):
                host.run(["exec"], directory=self.root, timeout_s=30)
        self.assertTrue((self.root / "cancel").is_file())
        self.assertEqual(process.wait.call_count, 2)
        self.assertEqual(process.wait.call_args.kwargs, {"timeout": 3})

    @unittest.skipIf(os.name == "nt", "POSIX subprocess fixture")
    def test_wsl_worker_forwards_input_and_filters_api_keys(self):
        executable = self.root / "codex-fixture"
        executable.write_text(f"#!{sys.executable}\nimport os,sys\n"
                              "assert 'OPENAI_API_KEY' not in os.environ\n"
                              "assert sys.argv[1:]==['exec','argument with spaces']\n"
                              "print(sys.stdin.read())\n")
        executable.chmod(0o700)
        result = subprocess.run(self.worker_command(executable, 3, "exec", "argument with spaces"),
                                input=b"Chinese instruction: \xe6\x8a\x93\xe5\x8f\x96", capture_output=True,
                                env={**os.environ, "OPENAI_API_KEY": "must-not-forward"}, timeout=5)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(b"\xe6\x8a\x93\xe5\x8f\x96", result.stdout)

    def worker_command(self, executable, timeout, *arguments):
        return [sys.executable, codex_process.__file__, "--wsl-worker", "--directory", str(self.root),
                "--codex-bin", str(executable), "--timeout", str(timeout), "--", *arguments]

    @unittest.skipIf(os.name == "nt", "POSIX subprocess fixture")
    def test_wsl_worker_timeout_reaps_its_codex_process(self):
        executable, pidfile = self.slow_fixture()
        # Timeout must also work when the child never reads a large prompt.
        result = subprocess.run(self.worker_command(executable, .4), input=b"x" * (256 * 1024),
                                capture_output=True, timeout=5)
        self.assertEqual(result.returncode, 124, result.stderr)
        with self.assertRaises(ProcessLookupError):
            os.kill(int(pidfile.read_text()), 0)

    @unittest.skipIf(os.name == "nt", "POSIX subprocess fixture")
    def test_wsl_cancel_marker_stops_codex_before_decision_timeout(self):
        executable, pidfile = self.slow_fixture()
        process = subprocess.Popen(self.worker_command(executable, 30), stdin=subprocess.DEVNULL,
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            deadline = time.monotonic() + 3
            while not pidfile.exists() and time.monotonic() < deadline:
                time.sleep(.02)
            self.assertTrue(pidfile.exists())
            (self.root / "cancel").touch()
            self.assertEqual(process.wait(timeout=3), 130)
            with self.assertRaises(ProcessLookupError):
                os.kill(int(pidfile.read_text()), 0)
        finally:
            if process.poll() is None:
                process.kill()
                process.wait(timeout=3)

    def slow_fixture(self):
        pidfile = self.root / "pid"
        executable = self.root / "codex-slow"
        executable.write_text(f"#!{sys.executable}\nimport os,pathlib,time\n"
                              f"pathlib.Path({str(pidfile)!r}).write_text(str(os.getpid()))\n"
                              "time.sleep(30)\n")
        executable.chmod(0o700)
        return executable, pidfile


if __name__ == "__main__":
    unittest.main()
