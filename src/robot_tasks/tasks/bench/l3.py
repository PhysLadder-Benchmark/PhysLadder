"""Native L3 task props."""

from .native_l3_specs import bottle, sweeping, wiping
from .push_triangle import build as pushing

BUILDERS = {
    "L3-03": bottle,
    "L3-11": wiping,
    "L3-14": pushing,
    "L3-15": sweeping,
}
