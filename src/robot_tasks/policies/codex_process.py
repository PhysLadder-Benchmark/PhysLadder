"""Run Codex natively or through WSL, with an owned, cancellable process tree.

The WSL worker in this file uses only Python 3.10's standard library. It can run
without installing the robot project or Isaac in WSL, and never reads tokens.
"""

import argparse
import os
from pathlib import Path, PurePosixPath
import shutil
import signal
import subprocess
import sys
import time
from urllib.parse import urlsplit

_WINDOWS = sys.platform == "win32"


def codex_environment(environment=None):
    """Keep desktop/network/auth paths, without forwarding API/application keys."""
    source = os.environ if environment is None else environment
    allowed = {"PATH", "HOME", "USER", "LOGNAME", "USERPROFILE", "APPDATA", "LOCALAPPDATA",
               "SYSTEMROOT", "SystemRoot", "COMSPEC", "PATHEXT", "TEMP", "TMP", "TMPDIR",
               "CODEX_HOME", "LANG", "LC_ALL", "XDG_CONFIG_HOME", "XDG_DATA_HOME", "XDG_CACHE_HOME",
               "XDG_RUNTIME_DIR", "HTTP_PROXY", "HTTPS_PROXY", "ALL_PROXY", "NO_PROXY",
               "http_proxy", "https_proxy", "all_proxy", "no_proxy", "SSL_CERT_FILE", "SSL_CERT_DIR"}
    return {key: value for key, value in source.items() if key in allowed}


def _local_settings():
    import tomllib
    path = Path(__file__).resolve().parents[3] / "configs/runtime/codex.local.toml"
    data = tomllib.loads(path.read_text(encoding="utf-8")) if path.is_file() else {}
    if set(data) - {"host", "wsl_distribution", "wsl_user", "wsl_python", "wsl_proxy"}:
        raise ValueError("Unknown Codex runtime setting")
    if any(not isinstance(value, str) or not value.strip() or any(c in value for c in "\r\n\x00")
           for value in data.values()):
        raise ValueError("Codex runtime settings must be non-empty single-line strings")
    if data.get("host", "auto") not in {"auto", "native", "wsl"}:
        raise ValueError("Codex host must be auto, native or wsl")
    if "wsl_proxy" in data:
        proxy = urlsplit(data["wsl_proxy"])
        if proxy.scheme not in {"http", "https", "socks5", "socks5h"} or not proxy.hostname or proxy.username or proxy.password:
            raise ValueError("wsl_proxy must be a proxy URL without credentials")
    return data


class CodexProcess:
    def __init__(self, codex_bin):
        config = _local_settings()
        self.codex_bin = codex_bin
        self.executable = shutil.which(codex_bin)
        preference = config.get("host", "auto")
        self.mode = "wsl" if _WINDOWS and (
            preference == "wsl" or (preference == "auto" and not self.executable)
        ) else "native"
        self.prefix = []
        if self.mode == "native":
            if not self.executable:
                raise RuntimeError("Codex CLI not found; install Codex or configure codex_bin")
            return
        wsl = shutil.which("wsl.exe")
        if not wsl:
            raise RuntimeError("WSL not found; configure a native Codex executable")
        self.prefix = [wsl]
        for field, flag in (("wsl_distribution", "--distribution"), ("wsl_user", "--user")):
            if config.get(field):
                self.prefix.extend([flag, config[field]])
        self.prefix.append("--exec")
        self.python = config.get("wsl_python", "python3")
        self.proxy = config.get("wsl_proxy")
        self.worker = str(self.path(Path(__file__).resolve()))

    def path(self, path):
        if self.mode == "native":
            return path
        try:
            result = subprocess.run([*self.prefix, "wslpath", "-a", "-u", str(path)],
                                    capture_output=True, timeout=10, env=codex_environment())
        except (OSError, subprocess.TimeoutExpired):
            raise RuntimeError("Cannot translate project paths into WSL") from None
        value = result.stdout.decode("utf-8", errors="replace").strip()
        if result.returncode or not value.startswith("/") or "\n" in value:
            raise RuntimeError("Cannot translate project paths into WSL")
        return PurePosixPath(value)

    def command(self, arguments, directory, timeout_s):
        if self.mode == "native":
            return [self.executable, *arguments]
        return [*self.prefix, self.python, self.worker, "--wsl-worker",
                "--directory", str(self.path(directory)), "--codex-bin", self.codex_bin,
                "--timeout", str(timeout_s),
                *(["--proxy", self.proxy] if self.proxy else []), "--", *arguments]

    def run(self, arguments, *, directory, data=b"", timeout_s):
        command = self.command(arguments, directory, timeout_s)
        with (directory / "events.jsonl").open("w+b") as output, (directory / "stderr.log").open("w+b") as errors:
            process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=output, stderr=errors,
                                       env=codex_environment(), start_new_session=(os.name != "nt"),
                                       creationflags=subprocess.CREATE_NEW_PROCESS_GROUP if os.name == "nt" else 0)
            try:
                # WSL enforces the same deadline itself; allow it time to reap
                # Linux descendants before terminating the Windows launcher.
                deadline = time.monotonic() + timeout_s + (3 if self.mode == "wsl" else 0)
                while True:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        raise subprocess.TimeoutExpired(command, timeout_s)
                    try:
                        # Short waits let Python handle Ctrl+C/CTRL_BREAK on
                        # Windows even when the provider has not returned yet.
                        process.communicate(data, timeout=min(.1, remaining))
                        break
                    except subprocess.TimeoutExpired:
                        data = None
            except (subprocess.TimeoutExpired, KeyboardInterrupt):
                self.terminate(process, directory)
                raise
            output.seek(0)
            stdout = output.read(4 * 1024 * 1024 + 1)
            errors.seek(0)
            stderr = errors.read(8192)
        if len(stdout) > 4 * 1024 * 1024:
            raise RuntimeError("Codex event stream exceeds size limit")
        if self.mode == "wsl" and process.returncode == 124:
            raise subprocess.TimeoutExpired(command, timeout_s)
        return subprocess.CompletedProcess(command, process.returncode, stdout, stderr)

    def terminate(self, process, directory):
        if process.poll() is not None:
            return
        if self.mode == "wsl":
            # taskkill cannot reliably reap Linux descendants. Ask their Linux
            # supervisor to kill its own process group before killing wsl.exe.
            (directory / "cancel").touch()
            try:
                process.wait(timeout=3)
                return
            except subprocess.TimeoutExpired:
                pass
        if os.name == "nt":
            subprocess.run(["taskkill", "/PID", str(process.pid), "/T", "/F"],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10, check=False)
        else:
            _kill_group(process.pid)
        process.wait(timeout=10)


def _kill_group(pid):
    try:
        os.killpg(pid, signal.SIGKILL)
    except ProcessLookupError:
        pass


def wsl_worker(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--wsl-worker", action="store_true", required=True)
    parser.add_argument("--directory", type=Path, required=True)
    parser.add_argument("--codex-bin", required=True)
    parser.add_argument("--timeout", type=float, required=True)
    parser.add_argument("--proxy")
    parser.add_argument("arguments", nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    executable = shutil.which(args.codex_bin)
    if not executable and args.codex_bin == "codex":
        # Codex's standalone installer uses this location; non-login WSL shells
        # commonly omit it from PATH. Use the selected WSL user's installation.
        candidate = Path.home() / ".local/bin/codex"
        if candidate.is_file() and os.access(candidate, os.X_OK):
            executable = str(candidate)
    if not executable:
        print("Codex CLI not found for the selected WSL user", file=sys.stderr)
        return 127
    arguments = args.arguments[1:] if args.arguments[:1] == ["--"] else args.arguments
    data = sys.stdin.buffer.read()
    environment = codex_environment()
    if args.proxy:
        for key in ("HTTP_PROXY", "HTTPS_PROXY", "http_proxy", "https_proxy"):
            environment[key] = args.proxy
    process = subprocess.Popen([executable, *arguments], stdin=subprocess.PIPE,
                               env=environment, start_new_session=True)
    deadline = time.monotonic() + args.timeout
    try:
        while True:
            if not args.directory.exists() or (args.directory / "cancel").exists():
                return 130
            if time.monotonic() >= deadline:
                return 124
            try:
                process.communicate(data, timeout=0.1)
                return process.returncode
            except subprocess.TimeoutExpired:
                data = None
    finally:
        _kill_group(process.pid)
        process.wait(timeout=3)


if __name__ == "__main__":
    raise SystemExit(wsl_worker())
