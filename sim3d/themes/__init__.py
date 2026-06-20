"""Theme registry — maps a theme name to a generator function.

Each generator is `generate(seed: int, difficulty: int) -> list[Rect3D]`,
pure (no global state, no I/O). Themes that ignore parameters accept them
and discard them.

Other themes (forest, warehouse, canyon, city) are added to the registry as
they land in their own tasks.
"""

from typing import Callable

from sim3d.world import Rect3D
from sim3d.themes import demo

THEME_REGISTRY: dict[str, Callable[[int, int], list[Rect3D]]] = {
    "demo": demo.generate,
}

THEMES: tuple[str, ...] = tuple(THEME_REGISTRY.keys())
