"""Real subprocess cancellation, using silent fixtures; no Isaac or model calls."""

import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest

from robot_tasks.paths import PROJECT_ROOT


def alive(pid):
    if os.name == "nt":
        import ctypes
        kernel = ctypes.WinDLL("kernel32")
        kernel.OpenProcess.restype = ctypes.c_void_p
        handle = kernel.OpenProcess(0x1000, False, pid)
        if not handle:
            return False
        try:
            code = ctypes.c_ulong()
            return bool(kernel.GetExitCodeProcess(ctypes.c_void_p(handle), ctypes.byref(code))) and code.value == 259
        finally:
            kernel.CloseHandle(ctypes.c_void_p(handle))
    try:
        os.kill(pid, 0)
        # A reaped-by-init descendant can briefly remain as an inert zombie.
        status = Path(f"/proc/{pid}/stat")
        return not status.exists() or status.read_text().split(")", 1)[1].split()[0] != "Z"
    except ProcessLookupError:
        return False


class ProcessCancellationTests(unittest.TestCase):
    def test_silent_worker_and_stubborn_child_are_stopped_and_interruption_is_recorded(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary).resolve()
            pidfile = root / "pids.json"
            worker = root / "worker.py"
            # Both processes refuse graceful shutdown, approximating a blocked
            # network/native call. The supervisor must terminate only this tree.
            ignore = "import signal,time; signal.signal(signal.SIGINT,signal.SIG_IGN); "
            if os.name == "nt":
                ignore += "signal.signal(signal.SIGBREAK,signal.SIG_IGN); "
            ignore += "time.sleep(60)"
            worker.write_text(f'''import json,os,signal,subprocess,sys,time
from pathlib import Path
signal.signal(signal.SIGINT,signal.SIG_IGN)
if hasattr(signal,'SIGBREAK'): signal.signal(signal.SIGBREAK,signal.SIG_IGN)
child=subprocess.Popen([sys.executable,'-c',{ignore!r}])
Path({str(pidfile)!r}).write_text(json.dumps([os.getpid(),child.pid]))
time.sleep(60)
''')
            supervisor = root / "supervisor.py"
            supervisor.write_text(f'''import signal,subprocess,sys
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,{str(PROJECT_ROOT / 'src')!r})
from robot_tasks.integrationtest import simulation_process as sp
original=subprocess.Popen
def spawn(command,**kwargs):
    if 'robot_tasks.integrationtest.simulation_worker' in command:
        command=[sys.executable,{str(worker)!r}]
    return original(command,**kwargs)
def stop(*args): raise KeyboardInterrupt
signal.signal(signal.SIGINT,stop)
if hasattr(signal,'SIGBREAK'): signal.signal(signal.SIGBREAK,stop)
try:
    with patch.object(sp.subprocess,'Popen',spawn):
        sp.run_check_process('gpt-pick-place',Path({str(root / 'runs')!r}))
except KeyboardInterrupt:
    raise SystemExit(130)
''')
            options = {"creationflags": subprocess.CREATE_NEW_PROCESS_GROUP} if os.name == "nt" else {"start_new_session": True}
            process = subprocess.Popen([sys.executable, str(supervisor)], stdout=subprocess.DEVNULL,
                                       stderr=subprocess.DEVNULL, **options)
            owned = []
            try:
                deadline = time.monotonic() + 10
                while time.monotonic() < deadline:
                    try:
                        owned = json.loads(pidfile.read_text())
                        break
                    except (OSError, ValueError):
                        self.assertIsNone(process.poll(), "Fixture supervisor exited before worker readiness")
                        time.sleep(.02)
                self.assertEqual(len(owned), 2, "Fixture worker did not become ready")
                started = time.monotonic()
                process.send_signal(signal.CTRL_BREAK_EVENT if os.name == "nt" else signal.SIGINT)
                self.assertEqual(process.wait(timeout=12), 130)
                self.assertLess(time.monotonic() - started, 12)
                for pid in owned:
                    self.assertFalse(alive(pid), f"Owned process {pid} survived cancellation")
                report, = (root / "runs").glob("*/process.json")
                result = json.loads(report.read_text())
                self.assertTrue(result["interrupted"])
                self.assertFalse(result["timed_out"])
                self.assertTrue(result["forced_termination"])
                self.assertTrue(result["cleanup_complete"])
                self.assertFalse(result["passed"])
                task_result = json.loads(report.with_name("result.json").read_text())
                self.assertEqual(task_result["status"], "cancelled")
                self.assertIsNone(task_result["success"])
                self.assertTrue(report.with_name("report.md").exists())
            finally:
                for pid in [*reversed(owned), process.pid]:
                    if alive(pid):
                        if os.name == "nt":
                            subprocess.run(["taskkill", "/PID", str(pid), "/T", "/F"],
                                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=3)
                        else:
                            try:
                                os.kill(pid, signal.SIGKILL)
                            except ProcessLookupError:
                                pass
                process.wait(timeout=3)


if __name__ == "__main__":
    unittest.main()
