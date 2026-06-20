"""Theme registry — maps a theme name to a generator function.

Each generator is `generate(seed: int, difficulty: int) -> list[Rect3D]`,
pure (no global state, no I/O). Themes that ignore parameters accept them
and discard them.
"""

from typing import Callable

from sim3d.world import Rect3D
from sim3d.themes import demo, slalom, arena

THEME_REGISTRY: dict[str, Callable[[int, int], list[Rect3D]]] = {
    "demo": demo.generate,
    "slalom": slalom.generate,
    "arena": arena.generate,
}

THEMES: tuple[str, ...] = tuple(THEME_REGISTRY.keys())
