"""Robot/Codex boundary fixtures; no account, network, model or Isaac required."""

import base64
from dataclasses import replace
from io import BytesIO
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

from PIL import Image

from robot_tasks import batch
from robot_tasks.execution import _gpt_policy
from robot_tasks.policies.codex_cli import CodexCLIClient, codex_environment, parse_events, serialize_observations
from robot_tasks.policies.model_config import GPTSettings, load_gpt_settings
from robot_tasks.policies.responses import ModelError, RetryableModelError
from robot_tasks.registry import resolve_task


SETTINGS = GPTSettings(base_url="https://api.openai.com/v1", model="gpt-6-astra", transport="codex_cli",
                       reasoning_effort="medium", timeout_s=5)


def events(*items, complete=True, text='{"commands": []}'):
    rows = [{"type": "thread.started", "thread_id": "fixture-thread"}, {"type": "turn.started"}, *items]
    if text is not None:
        rows.append({"type": "item.completed", "item": {"type": "agent_message", "text": text}})
    if complete:
        rows.append({"type": "turn.completed", "usage": {"input_tokens": 12, "cached_input_tokens": 4,
                                                         "output_tokens": 3, "reasoning_output_tokens": 2}})
    return "\n".join(map(json.dumps, rows))


class CodexBoundaryTests(unittest.TestCase):
    def setUp(self):
        settings = patch("robot_tasks.policies.codex_process._local_settings", return_value={})
        settings.start()
        self.addCleanup(settings.stop)
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)

    def png_url(self):
        output = BytesIO()
        Image.new("RGB", (4, 4), (255, 0, 0)).save(output, format="PNG")
        return "data:image/png;base64," + base64.b64encode(output.getvalue()).decode()

    def test_images_stay_ordered_and_labelled_across_history(self):
        messages = [{"role": "user", "content": [
            {"type": "input_text", "text": "before: RGB left wrist"},
            {"type": "input_image", "image_url": self.png_url()}]},
            {"role": "assistant", "content": '{"commands":[]}'},
            {"role": "user", "content": [
                {"type": "input_text", "text": "current: depth overview"},
                {"type": "input_image", "image_url": self.png_url()}]}]
        prompt, images = serialize_observations(messages, self.root)
        self.assertEqual(len(images), 2)
        self.assertLess(prompt.index("before:"), prompt.index("Attached image #1"))
        self.assertLess(prompt.index("current:"), prompt.index("Attached image #2"))
        self.assertIn("Record 3, role=user", prompt)
        with Image.open(images[1]) as image:
            self.assertEqual(image.size, (4, 4))

    def test_remote_images_and_malformed_inline_data_are_rejected(self):
        for url in ("https://untrusted/image.png", "data:image/png;base64,@@@",
                    "data:image/png;base64," + base64.b64encode(b"not PNG").decode()):
            with self.subTest(url=url), self.assertRaises(ModelError):
                serialize_observations([{"role": "user", "content": [{"type": "input_image", "image_url": url}]}], self.root)

    def test_only_completed_json_decisions_are_accepted_and_usage_preserved(self):
        result = parse_events(events(), model=SETTINGS.model, latency_s=1.5)
        self.assertEqual(result.response_id, "fixture-thread")
        self.assertEqual(result.usage, {"input_tokens": 12, "output_tokens": 3, "cached_tokens": 4,
                                       "reasoning_tokens": 2, "total_tokens": 15})
        for stream in (events(complete=False), events(text="```json\n{}\n```"), events(text="[]"), "malformed"):
            with self.subTest(stream=stream), self.assertRaises(ModelError):
                parse_events(stream, model=SETTINGS.model, latency_s=1)

    def test_completed_turn_with_tool_activity_is_rejected(self):
        for kind in ("command_execution", "mcp_tool_call", "file_change", "web_search"):
            with self.subTest(kind=kind), self.assertRaisesRegex(ModelError, "tool"):
                parse_events(events({"type": "item.started", "item": {"type": kind}}), model=SETTINGS.model, latency_s=1)

    def test_failure_redacts_bearer_api_key_and_jwt(self):
        stream = events({"type": "turn.failed", "error": {
            "message": "Bearer secret sk-private eyJtest.payload.signature quota exceeded"}}, complete=False, text=None)
        with self.assertRaises(ModelError) as caught:
            parse_events(stream, model=SETTINGS.model, latency_s=1)
        self.assertIn("quota exceeded", str(caught.exception))
        for private in ("secret", "sk-private", "eyJtest"):
            self.assertNotIn(private, str(caught.exception))

    def test_environment_drops_api_keys_and_application_secrets(self):
        result = codex_environment({"HOME": "/identity", "PATH": "/bin", "CODEX_HOME": "/auth",
                                    "HTTPS_PROXY": "http://localhost:7897", "OPENAI_API_KEY": "private",
                                    "CODEX_API_KEY": "private", "OPENAI_BASE_URL": "https://relay",
                                    "OTHER_SECRET": "private"})
        self.assertEqual(set(result), {"HOME", "PATH", "CODEX_HOME", "HTTPS_PROXY"})

    def test_login_rejects_api_auth_even_if_cli_exits_zero(self):
        with patch("robot_tasks.policies.codex_process.shutil.which", return_value="fixture"):
            client = CodexCLIClient(SETTINGS)
        for message in ("Logged in using an API key", "Not logged in"):
            with patch.object(client.host, "run", return_value=subprocess.CompletedProcess([], 0, message.encode(), b"")):
                with self.assertRaisesRegex(ModelError, "ChatGPT login required"):
                    client.require_chatgpt_login()

    def test_codex_task_and_batch_do_not_require_or_load_api_key(self):
        model = self.root / "model.toml"
        model.write_text('[gpt]\nbase_url="https://api.openai.com/v1"\nmodel="gpt-6-astra"\ntransport="codex_cli"\n')
        _, config = resolve_task("L1-01")
        with patch("robot_tasks.policies.codex_process.shutil.which", return_value="fixture"), \
                patch.object(CodexCLIClient, "require_chatgpt_login") as login, \
                patch("robot_tasks.execution.read_api_key", side_effect=AssertionError("API credentials must not be used")), \
                patch.object(batch, "load_env", side_effect=AssertionError(".env must not be loaded")):
            policy = _gpt_policy(config, model)
            self.assertIsInstance(policy.client, CodexCLIClient)
            environment = batch.worker_environment(self.root / ".env", {"model_config": str(model), "options": {"dry_run": False}})
            self.assertIn("PYTHONPATH", environment)
            self.assertEqual(login.call_count, 2)

    @unittest.skipIf(os.name == "nt", "POSIX mock executable")
    def test_real_cli_boundary_gets_images_schema_model_and_clean_env(self):
        executable = self.root / "codex-fixture"
        executable.write_text(f"#!{sys.executable}\n" +
            "import json,os,pathlib,sys\n"
            "a=sys.argv[1:]\n"
            "assert a[a.index('--model')+1]=='gpt-6-astra'\n"
            "assert 'model_reasoning_effort=\"medium\"' in a\n"
            "assert 'forced_login_method=\"chatgpt\"' in a\n"
            "assert 'OPENAI_API_KEY' not in os.environ\n"
            "assert json.loads(pathlib.Path(a[a.index('--output-schema')+1]).read_text())['type']=='object'\n"
            "assert pathlib.Path(a[a.index('--image')+1]).read_bytes().startswith(b'\\x89PNG')\n"
            "assert 'Attached image #1' in sys.stdin.read()\n"
            f"print({events()!r})\n")
        executable.chmod(0o700)
        with patch.dict(os.environ, {"OPENAI_API_KEY": "must-not-forward"}):
            client = CodexCLIClient(replace(SETTINGS, codex_bin=str(executable)))
            result = client.generate(instructions="Robot contract", schema={"type": "object"}, messages=[{
                "role": "user", "content": [{"type": "input_text", "text": "RGB label"},
                                             {"type": "input_image", "image_url": self.png_url()}]}])
        self.assertEqual(result.usage["total_tokens"], 15)

    @unittest.skipIf(os.name == "nt", "POSIX mock executable")
    def test_timeout_discards_partial_reply_and_stops_process(self):
        executable = self.root / "codex-slow"
        pidfile = self.root / "pid"
        executable.write_text(f"#!{sys.executable}\nimport os,time,pathlib\n"
                              f"pathlib.Path({str(pidfile)!r}).write_text(str(os.getpid()))\n"
                              f"print({events(complete=False)!r},flush=True)\ntime.sleep(30)\n")
        executable.chmod(0o700)
        client = CodexCLIClient(replace(SETTINGS, codex_bin=str(executable), timeout_s=.3))
        with self.assertRaisesRegex(RetryableModelError, "timed out"):
            client.generate(instructions="contract", messages=[], schema={})
        with self.assertRaises(ProcessLookupError):
            os.kill(int(pidfile.read_text()), 0)

    def test_model_config_stays_explicit_and_rejects_unsupported_transport(self):
        with self.assertRaises(ValueError):
            replace(SETTINGS, transport="implicit_fallback")
        with self.assertRaises(ValueError):
            replace(SETTINGS, max_output_tokens=100)


if __name__ == "__main__":
    unittest.main()
