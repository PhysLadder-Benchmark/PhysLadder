"""Interactive inspection must not time out or invalidate completed checks."""

from contextlib import redirect_stdout
import io
import json
import os
from pathlib import Path
import signal
import tempfile
import unittest
from unittest.mock import Mock, patch

from robot_tasks.integrationtest.scene_preview import PREVIEW_READY, hold_scene_preview
from robot_tasks.integrationtest.simulation_process import run_check_process


class ScenePreviewTests(unittest.TestCase):
    def test_viewer_updates_until_window_closes_without_physics_steps(self):
        backend = Mock()
        backend.app.is_running.side_effect = [True, True, False]
        with redirect_stdout(io.StringIO()) as out, patch("robot_tasks.integrationtest.scene_preview.time.sleep"):
            hold_scene_preview(backend)
        backend.world.pause.assert_called_once()
        backend.world.step.assert_not_called()
        self.assertEqual(backend.app.update.call_count, 2)
        self.assertIn(PREVIEW_READY, out.getvalue())

    def test_ctrl_c_leaves_viewer_without_raising(self):
        backend = Mock()
        backend.app.update.side_effect = KeyboardInterrupt
        with redirect_stdout(io.StringIO()):
            hold_scene_preview(backend)
        backend.app.update.assert_called_once()

    def supervisor(self, gui, interrupt=False):
        with tempfile.TemporaryDirectory() as tmp:
            timer = Mock()
            process = Mock(pid=123)
            process.wait.return_value = 0
            process.poll.return_value = None

            def output_lines():
                # Watchdog protects the actual check even in GUI mode.
                timer.cancel.assert_not_called()
                yield PREVIEW_READY + "\n"
                if gui:
                    timer.cancel.assert_called_once()
                else:
                    timer.cancel.assert_not_called()
                if interrupt:
                    raise KeyboardInterrupt

            process.stdout = output_lines()

            def spawn(command, **kwargs):
                path = Path(command[command.index("--run-directory") + 1])
                (path / "result.json").write_text('{"status":"completed","passed":true}')
                context = Mock()
                context.__enter__ = Mock(return_value=process)
                context.__exit__ = Mock(return_value=False)
                return context

            with patch("robot_tasks.integrationtest.simulation_process.subprocess.Popen", side_effect=spawn) as popen, \
                    patch("robot_tasks.integrationtest.simulation_process.threading.Timer", return_value=timer), \
                    redirect_stdout(io.StringIO()):
                output, passed = run_check_process("l1-scene", Path(tmp), gui=gui)
            self.assertTrue(passed)
            self.assertEqual(json.loads((output / "result.json").read_text()), {"status": "completed", "passed": True})
            self.assertFalse(json.loads((output / "process.json").read_text())["timed_out"])
            if interrupt:
                process.send_signal.assert_called_once_with(signal.CTRL_BREAK_EVENT if os.name == "nt" else signal.SIGINT)
                process.kill.assert_not_called()
                process.wait.assert_called_with(timeout=5)
            if gui:
                self.assertTrue(popen.call_args.kwargs.get("creationflags") if os.name == "nt" else
                                popen.call_args.kwargs.get("start_new_session"))

    def test_gui_watchdog_ends_only_after_saved_result_and_ctrl_c_preserves_pass(self):
        self.supervisor(gui=True, interrupt=True)

    def test_headless_watchdog_is_not_disabled_by_preview_marker(self):
        self.supervisor(gui=False)


if __name__ == "__main__":
    unittest.main()
