"""Native L3 task props."""

from .native_l3_specs import bottle, sweeping, wiping
from .push_triangle import build as pushing

BUILDERS = {
    "L3-01": bottle,
    "L3-03": wiping,
    "L3-04": pushing,
    "L3-05": sweeping,
}
