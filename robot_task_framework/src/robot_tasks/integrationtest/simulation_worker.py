"""Private subprocess entry point; all evidence is flushed before native shutdown."""

import argparse
from pathlib import Path

from .motion_check import run_motion_check


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--run-directory", type=Path, required=True)
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--check", choices=("joint", "cartesian", "interaction", "gpt", "cameras"), default="joint")
    args = parser.parse_args()
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
    _, result = check(args.config, args.run_directory.parent, gui=args.gui, run_directory=args.run_directory)
    return 0 if result["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
