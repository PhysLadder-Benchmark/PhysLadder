"""Compatibility entry for the current L3-05 nut clearance diagnostic."""

from .nut_clearance import check, main

if __name__ == "__main__":
    raise SystemExit(main())
