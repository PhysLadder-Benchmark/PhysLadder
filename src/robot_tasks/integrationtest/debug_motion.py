"""Debug the real motion check in one Windows process from a WSL editor.

The normal check-r5 command and its subprocess watchdog remain unchanged.
This development entry waits for attachment before opening Isaac Sim.
"""

import argparse
import ipaddress
from pathlib import Path
import sys

from ..paths import PROJECT_ROOT


def main() -> int:
    from ..runtime import configure_windows_storage
    configure_windows_storage()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=PROJECT_ROOT / "configs/checks/r5_motion_debug.toml")
    parser.add_argument("--output", type=Path, default=PROJECT_ROOT / "runs/debug")
    parser.add_argument("--listen-host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5678)
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--check", choices=("joint", "cartesian", "interaction", "gpt", "cameras"), default="joint")
    args = parser.parse_args()
    if sys.platform != "win32":
        parser.error("Use the Windows isaac51 Python through tools/run_r5_check.ps1 -Debugger")
    address = ipaddress.ip_address(args.listen_host)
    if address.is_unspecified or not (address.is_private or address.is_loopback):
        parser.error("Bind only to loopback or the Windows WSL virtual adapter address")
    if not 1 <= args.port <= 65535:
        parser.error("port must be between 1 and 65535")
    if not args.config.is_file():
        parser.error(f"Configuration not found: {args.config}")

    print("R5_DEBUG_STARTING", flush=True)
    try:
        import debugpy
    except ImportError:
        parser.error("Install debugpy in the Windows isaac51 environment first")
    debugpy.configure(subProcess=False)
    debugpy.listen((str(address), args.port))
    # VS Code's background task becomes ready only after the socket is listening.
    print(f"R5_DEBUG_READY {address}:{args.port}", flush=True)
    debugpy.wait_for_client()

    from .motion_check import run_motion_check
    check = run_motion_check
    if args.check == "cartesian":
        from .cartesian_check import run_cartesian_check
        check = run_cartesian_check
    elif args.check == "interaction":
        from .interaction_check import run_interaction_check
        check = run_interaction_check
    elif args.check == "gpt":
        from .gpt_check import run_gpt_check
        check = run_gpt_check
    elif args.check == "cameras":
        from .camera_check import run_camera_check
        check = run_camera_check

    # Stops here automatically. F11 on the following call enters the real check.
    debugpy.breakpoint()
    _, result = check(args.config, args.output, gui=args.gui)
    return 0 if result["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
