"""Use the user's ChatGPT login through Codex for bounded robot decisions.

Codex owns credentials and refresh; this adapter never reads auth.json. Each
invocation receives only disclosed observations/history and returns one JSON
decision. Physics, validation, evaluation and video stay in the existing runner.
CLI usage describes Codex turns, not a promise of one HTTP request per decision.
"""

import base64
import binascii
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import tempfile
import time

from .model_config import GPTSettings
from .responses import ModelError, ModelReply, _safe_error_field


def codex_environment(environment=None):
    """Keep desktop/network/auth paths, without forwarding API/application keys."""
    source = os.environ if environment is None else environment
    allowed = {"PATH", "HOME", "USER", "LOGNAME", "USERPROFILE", "APPDATA", "LOCALAPPDATA",
               "SYSTEMROOT", "SystemRoot", "COMSPEC", "PATHEXT", "TEMP", "TMP", "TMPDIR",
               "CODEX_HOME", "LANG", "LC_ALL", "XDG_CONFIG_HOME", "XDG_DATA_HOME", "XDG_CACHE_HOME",
               "XDG_RUNTIME_DIR", "HTTP_PROXY", "HTTPS_PROXY", "ALL_PROXY", "NO_PROXY",
               "http_proxy", "https_proxy", "all_proxy", "no_proxy", "SSL_CERT_FILE", "SSL_CERT_DIR"}
    return {key: value for key, value in source.items() if key in allowed}


def safe_diagnostic(value):
    text = _safe_error_field(value, "") or "No diagnostic available"
    return re.sub(r"\beyJ[A-Za-z0-9_.-]+", "[REDACTED]", text)


def serialize_observations(messages, directory):
    """Map each attached PNG to an explicit, ordered label in the text prompt."""
    parts, images = [], []
    for number, message in enumerate(messages, 1):
        role = message.get("role")
        if role not in {"user", "assistant"}:
            raise ModelError("Unsupported Codex observation role")
        parts.append(f"Record {number}, role={role}:")
        content = message.get("content")
        if isinstance(content, str):
            parts.append(content)
            continue
        if not isinstance(content, list):
            raise ModelError("Invalid Codex observation content")
        for item in content:
            if item.get("type") == "input_text" and isinstance(item.get("text"), str):
                parts.append(item["text"])
            elif item.get("type") == "input_image":
                url = item.get("image_url", "")
                prefix = "data:image/png;base64,"
                if not isinstance(url, str) or not url.startswith(prefix):
                    raise ModelError("Codex observations must be inline PNGs")
                try:
                    png = base64.b64decode(url[len(prefix):], validate=True)
                except (ValueError, binascii.Error):
                    raise ModelError("Invalid observation PNG encoding") from None
                if not png.startswith(b"\x89PNG\r\n\x1a\n") or len(png) > 16 * 1024 * 1024:
                    raise ModelError("Invalid or oversized observation PNG")
                path = directory / f"image_{len(images) + 1:03d}.png"
                path.write_bytes(png)
                images.append(path)
                parts.append(f"[Attached image #{len(images)} belongs to the immediately preceding label.]")
            else:
                raise ModelError("Unsupported Codex observation item")
    return "\n".join(parts), images


def parse_events(stdout, *, model, latency_s):
    thread, texts, completed, usage = None, [], False, {}
    for line in stdout.splitlines():
        try:
            event = json.loads(line)
        except (ValueError, TypeError):
            raise ModelError("Malformed Codex event stream; no action accepted") from None
        if not isinstance(event, dict):
            raise ModelError("Malformed Codex event; no action accepted")
        if event.get("type") == "thread.started":
            thread = event.get("thread_id")
        elif event.get("type") == "turn.completed":
            if completed:
                raise ModelError("Multiple Codex turns; no action accepted")
            completed = True
            counts = event.get("usage") or {}
            usage = {k: counts[k] for k in ("input_tokens", "output_tokens") if type(counts.get(k)) is int}
            if type(counts.get("cached_input_tokens")) is int:
                usage["cached_tokens"] = counts["cached_input_tokens"]
            if type(counts.get("reasoning_output_tokens")) is int:
                usage["reasoning_tokens"] = counts["reasoning_output_tokens"]
            if "input_tokens" in usage and "output_tokens" in usage:
                usage["total_tokens"] = usage["input_tokens"] + usage["output_tokens"]
        elif event.get("type") == "turn.failed":
            error = event.get("error") or {}
            raise ModelError("Codex ChatGPT call failed: " + safe_diagnostic(error.get("message") if isinstance(error, dict) else error))
        elif event.get("type") in {"item.started", "item.updated", "item.completed"}:
            item = event.get("item") or {}
            kind = item.get("type")
            if kind not in {"agent_message", "reasoning", "error"}:
                raise ModelError("Codex attempted a tool or unsupported item; no action accepted")
            if event["type"] == "item.completed" and kind == "agent_message":
                texts.append(item.get("text"))
    if not completed or not isinstance(thread, str) or len(texts) != 1 or not isinstance(texts[0], str):
        raise ModelError("Codex did not return one completed decision; no action accepted")
    try:
        document = json.loads(texts[0])
    except ValueError:
        raise ModelError("Codex final decision is not JSON; no action accepted") from None
    if not isinstance(document, dict):
        raise ModelError("Codex final decision must be a JSON object")
    # CLI JSONL doesn't disclose the server-returned model ID. Record the exact
    # configured model honestly; do not claim independent response-ID verification.
    return ModelReply(texts[0], model, thread, usage, latency_s)


class CodexCLIClient:
    provider_metadata = {"authentication": "chatgpt", "model_verification": "explicit_cli_configuration",
                         "response_id_kind": "codex_thread_id"}

    def __init__(self, settings: GPTSettings):
        self.settings = settings
        self.executable = shutil.which(settings.codex_bin)
        if not self.executable:
            raise ModelError("Codex CLI not found; install Codex or configure codex_bin")

    def require_chatgpt_login(self):
        try:
            status = subprocess.run([self.executable, "login", "status"], capture_output=True, text=True,
                                    timeout=15, env=codex_environment(), check=False)
        except (OSError, subprocess.TimeoutExpired):
            raise ModelError("Cannot check Codex ChatGPT login; run codex login status") from None
        if status.returncode != 0 or "Logged in using ChatGPT" not in status.stdout + status.stderr:
            raise ModelError("Codex ChatGPT login required; run codex login, then codex login status")

    def generate(self, *, instructions, messages, schema):
        settings = self.settings
        start = time.monotonic()
        with tempfile.TemporaryDirectory(prefix="gpt-benchmark-codex-") as temporary:
            directory = Path(temporary)
            observations, images = serialize_observations(messages, directory)
            schema_file = directory / "action.schema.json"
            schema_file.write_text(json.dumps(schema, allow_nan=False), encoding="utf-8")
            instructions_file = directory / "robot.instructions.txt"
            instructions_file.write_text("You are a robot decision policy, not a coding agent. "
                                         "Use only supplied sensor evidence. Do not call tools or access files. "
                                         "Return one decision matching the output schema.\n" + instructions,
                                         encoding="utf-8")
            prompt = ("You are the robot decision model for a benchmark. Return exactly one JSON object. "
                      "Do not use tools, read files, inspect the repository, or execute commands. "
                      "All permitted evidence is supplied below and in the attached images. "
                      "Image numbers in the records identify the ordered attachments. "
                      "Use the latest user record as the current observation; earlier records are history.\n"
                      + "\nOBSERVATION HISTORY:\n" + observations)
            command = [self.executable, "exec", "--ignore-user-config", "--ephemeral", "--skip-git-repo-check",
                       "--cd", temporary, "--sandbox", "read-only", "--model", settings.model,
                       "--json", "--color", "never", "--output-schema", str(schema_file)]
            overrides = {"model_provider": "openai", "forced_login_method": "chatgpt",
                         "model_reasoning_effort": settings.reasoning_effort, "web_search": "disabled",
                         "features.shell_tool": False, "features.unified_exec": False,
                         "features.multi_agent": False, "features.apps": False, "features.plugins": False,
                         "features.browser_use": False, "features.computer_use": False,
                         "features.image_generation": False, "features.hooks": False,
                         "features.skip_host_skill_discovery": True, "suppress_unstable_features_warning": True,
                         "features.unbounded_connection_retries": False,
                         "model_instructions_file": str(instructions_file)}
            for key, value in overrides.items():
                command.extend(["-c", key + "=" + json.dumps(value)])
            for path in images:
                command.extend(["--image", str(path)])
            command.append("-")
            with (directory / "events.jsonl").open("w+b") as output, (directory / "stderr.log").open("w+b") as errors:
                try:
                    process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=output, stderr=errors,
                                               env=codex_environment(), start_new_session=(os.name != "nt"))
                except OSError:
                    raise ModelError("Unable to start Codex CLI") from None
                try:
                    process.communicate(prompt.encode("utf-8"), timeout=settings.timeout_s)
                except (subprocess.TimeoutExpired, KeyboardInterrupt) as exc:
                    self._terminate(process)
                    if isinstance(exc, KeyboardInterrupt):
                        raise
                    raise ModelError("Codex ChatGPT decision timed out; no action accepted") from None
                output.seek(0)
                stdout = output.read(4 * 1024 * 1024 + 1)
                if len(stdout) > 4 * 1024 * 1024:
                    raise ModelError("Codex event stream exceeds size limit")
                stdout = stdout.decode("utf-8", errors="replace")
                if process.returncode:
                    # Prefer a bounded structured failure to arbitrary stderr (which
                    # can contain local environment or provider details).
                    for line in stdout.splitlines():
                        try:
                            event = json.loads(line)
                        except ValueError:
                            continue
                        if event.get("type") == "turn.failed":
                            error = event.get("error") or {}
                            raise ModelError("Codex ChatGPT call failed: " + safe_diagnostic(error.get("message")))
                    raise ModelError(f"Codex ChatGPT call failed (exit {process.returncode}); no action accepted")
                return parse_events(stdout, model=settings.model, latency_s=time.monotonic() - start)

    @staticmethod
    def _terminate(process):
        if process.poll() is not None:
            return
        if os.name == "nt":
            subprocess.run(["taskkill", "/PID", str(process.pid), "/T", "/F"],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10, check=False)
        else:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        process.wait(timeout=10)
