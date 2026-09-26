"""Process-local research storage. Never replace HOME, USERPROFILE or AppData.

Windows entry points share the same D-drive manifest as the PowerShell launcher.
WSL already lives in D:\\wsl\\Ubuntu\\ext4.vhdx; Linux paths are left intact.
"""

from dataclasses import dataclass
import json
import os
from pathlib import Path
import sys
import tempfile

from .paths import PROJECT_ROOT


STORAGE_CONFIG = PROJECT_ROOT / "configs/runtime/windows.json"


@dataclass(frozen=True)
class RuntimeStorage:
    root: Path
    kit_root: Path
    environment: dict[str, str]

    def kit_arguments(self) -> list[str]:
        return ["--portable-root", str(self.kit_root)]


def configure_windows_storage() -> RuntimeStorage | None:
    """Configure before importing native libraries/debugpy or starting Kit."""
    if sys.platform != "win32":
        return None
    data = json.loads(STORAGE_CONFIG.read_text(encoding="utf-8"))
    root = Path(data["root"]).resolve()
    if root.drive.upper() != "D:":
        raise ValueError("Research runtime storage must be on D:")

    def directory(relative: str) -> Path:
        path = (root / relative).resolve()
        if not path.is_relative_to(root):
            raise ValueError(f"Runtime path escapes configured root: {relative}")
        path.mkdir(parents=True, exist_ok=True)
        return path

    forbidden = {"HOME", "USERPROFILE", "APPDATA", "LOCALAPPDATA", "PATH", "CODEX_HOME"}
    if data["environment"].keys() & forbidden:
        raise ValueError("Storage configuration must not replace identity or system search paths")
    environment = {name: str(directory(relative)) for name, relative in data["environment"].items()}
    os.environ.update(environment)
    # These may already have been cached by Python before main() was entered.
    tempfile.tempdir = environment["TEMP"]
    sys.pycache_prefix = environment["PYTHONPYCACHEPREFIX"]
    return RuntimeStorage(root, directory(data["kit_root"]), environment)


def storage_report(storage: RuntimeStorage | None, *, kit_started: bool = False) -> dict:
    """Report resolved paths, including junction targets, rather than C: aliases."""
    if storage is None:
        return {"platform": sys.platform, "windows_storage_configured": False}
    paths = {"project": str(PROJECT_ROOT.resolve()), "python": str(Path(sys.executable).resolve()),
             "tempfile": str(Path(tempfile.gettempdir()).resolve()), "kit_root": str(storage.kit_root),
             **{name: str(Path(value).resolve()) for name, value in storage.environment.items()}}
    data = json.loads(STORAGE_CONFIG.read_text(encoding="utf-8"))
    for entry in data["junctions"]:
        alias = Path(os.environ[entry["base"]]) / entry["source"]
        paths[entry["source"]] = str(alias.resolve())
    if kit_started:
        import carb.tokens
        import warp
        tokens = carb.tokens.get_tokens_interface()
        for name in ("cache", "data", "logs"):
            resolved = tokens.resolve("${" + name + "}")
            if "${" in resolved:
                raise RuntimeError(f"Kit storage token was not resolved: {name}")
            paths[f"kit_{name}"] = str(Path(resolved).resolve())
        if warp.config.kernel_cache_dir is not None:
            paths["actual_warp_cache"] = str(Path(warp.config.kernel_cache_dir).resolve())
    off_drive = {name: value for name, value in paths.items() if Path(value).drive.upper() != "D:"}
    exceptions = {entry["source"]: {"resolved_path": str((Path(os.environ[entry["base"]]) / entry["source"]).resolve()),
                                    "reason": entry["reason"]}
                  for entry in data.get("shared_system_cache_exceptions", [])}
    return {"platform": sys.platform, "windows_storage_configured": True,
            "runtime_root": str(storage.root), "paths": paths, "all_research_paths_on_d": not off_drive,
            "off_drive_research_paths": off_drive, "shared_system_cache_exceptions": exceptions}


if __name__ == "__main__":
    configured = configure_windows_storage()
    print(json.dumps(storage_report(configured), indent=2, ensure_ascii=False))
