"""Maintained L2 tasks use the version-locked L20 deliveries."""

from .native_l2_specs import books, bowl, drawer, marker, orange

BUILDERS = {
    "L2-01": marker,
    "L2-03": books,
    "L2-04": bowl,
    "L2-06": orange,
    "L2-07": drawer,
}
