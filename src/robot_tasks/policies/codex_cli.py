"""Use the user's ChatGPT login through Codex for bounded robot decisions.

Codex owns credentials and refresh; this adapter never reads auth.json. Each
invocation receives only disclosed observations/history and returns one JSON
decision. Physics, validation, evaluation and video stay in the existing runner.
CLI usage describes Codex turns, not a promise of one HTTP request per decision.
"""

import base64
import binascii
import json
from pathlib import Path
import re
import subprocess
import tempfile
import time

from ..paths import PROJECT_ROOT
from .codex_process import CodexProcess, codex_environment
from .model_config import GPTSettings
from .responses import ModelError, ModelReply, RetryableModelError, _safe_error_field


def _temporary_directory():
    root = PROJECT_ROOT / "build/codex"
    root.mkdir(parents=True, exist_ok=True)
    return tempfile.TemporaryDirectory(prefix="decision-", dir=root)


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
        try:
            self.host = CodexProcess(settings.codex_bin)
        except RuntimeError as exc:
            raise ModelError(str(exc)) from None
        self.provider_metadata = {**self.provider_metadata, "codex_host": self.host.mode}

    def require_chatgpt_login(self):
        try:
            with _temporary_directory() as temporary:
                status = self.host.run(["login", "status"], directory=Path(temporary), timeout_s=15)
        except (OSError, subprocess.TimeoutExpired):
            raise ModelError("Cannot check Codex ChatGPT login; run codex login status") from None
        except RuntimeError as exc:
            raise ModelError(str(exc)) from None
        if status.returncode == 127:
            raise ModelError("Codex CLI not found in WSL; check configs/runtime/codex.local.toml and codex_bin")
        if status.returncode != 0 or b"Logged in using ChatGPT" not in status.stdout + status.stderr:
            raise ModelError("Codex ChatGPT login required; run codex login, then codex login status")

    def generate(self, *, instructions, messages, schema):
        settings = self.settings
        start = time.monotonic()
        with _temporary_directory() as temporary:
            directory = Path(temporary)
            remote = self.host.path(directory)
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
            command = ["exec", "--ignore-user-config", "--ephemeral", "--skip-git-repo-check",
                       "--cd", str(remote), "--sandbox", "read-only", "--model", settings.model,
                       "--json", "--color", "never", "--output-schema", str(remote / schema_file.name)]
            overrides = {"model_provider": "openai", "forced_login_method": "chatgpt",
                         "model_reasoning_effort": settings.reasoning_effort, "web_search": "disabled",
                         "features.shell_tool": False, "features.unified_exec": False,
                         "features.multi_agent": False, "features.apps": False, "features.plugins": False,
                         "features.browser_use": False, "features.computer_use": False,
                         "features.image_generation": False, "features.hooks": False,
                         "features.skip_host_skill_discovery": True, "suppress_unstable_features_warning": True,
                         "features.unbounded_connection_retries": False,
                         "model_instructions_file": str(remote / instructions_file.name)}
            for key, value in overrides.items():
                command.extend(["-c", key + "=" + json.dumps(value)])
            for path in images:
                command.extend(["--image", str(remote / path.name)])
            command.append("-")
            try:
                result = self.host.run(command, directory=directory, data=prompt.encode("utf-8"),
                                       timeout_s=settings.timeout_s)
            except subprocess.TimeoutExpired:
                raise RetryableModelError("Codex ChatGPT decision timed out; no action accepted") from None
            except OSError:
                raise ModelError("Unable to start Codex CLI") from None
            except RuntimeError as exc:
                raise ModelError(str(exc)) from None
            stdout = result.stdout.decode("utf-8", errors="replace")
            if result.returncode:
                # Prefer structured failures; stderr may contain private paths.
                for line in stdout.splitlines():
                    try:
                        event = json.loads(line)
                    except ValueError:
                        continue
                    if isinstance(event, dict) and event.get("type") == "turn.failed":
                        error = event.get("error") or {}
                        message = error.get("message") if isinstance(error, dict) else error
                        raise ModelError("Codex ChatGPT call failed: " + safe_diagnostic(message))
                raise ModelError(f"Codex ChatGPT call failed (exit {result.returncode}); no action accepted")
            return parse_events(stdout, model=settings.model, latency_s=time.monotonic() - start)
