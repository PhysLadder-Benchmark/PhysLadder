"""Maintained L2 tasks use the version-locked L20 deliveries."""

from .native_l2_specs import books, drawer, marker, orange
from .pearl_pouring import pouring
from .jelly import jelly

BUILDERS = {
    "L2-01": marker,
    "L2-03": books,
    "L2-04": pouring,
    "L2-06": orange,
    "L2-07": drawer,
    "L2-08": jelly,
}
