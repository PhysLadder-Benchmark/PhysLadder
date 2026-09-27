"""Private subprocess entry point; all evidence is flushed before native shutdown."""

import argparse
from pathlib import Path

from .registry import CHECKS, get_check


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--run-directory", type=Path, required=True)
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--check", choices=tuple(CHECKS), required=True)
    parser.add_argument("--model-config", type=Path)
    args = parser.parse_args(argv)
    _, result = get_check(args.check).run(args.config, args.run_directory.parent, gui=args.gui,
                                        model_config=args.model_config, run_directory=args.run_directory)
    return 0 if result["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
