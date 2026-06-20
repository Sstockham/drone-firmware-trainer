"""The hand-crafted seed-42 three-set-piece layout from the original 3D demo.

This theme exists so that the v1_3d-vs-v2_3d demo narrative (tall wall +
short bar + hanging overhang) keeps working byte-identically after build_world3d
gains its theme dispatch. Ignores `seed` and `difficulty`.
"""

from sim3d.world import Rect3D


def generate(seed: int, difficulty: int) -> list[Rect3D]:
    del seed, difficulty
    return [
        Rect3D(6.0, 6.0, 0.0, 4.0, 0.6, 4.0),    # tall wall, full height
        Rect3D(11.0, 9.0, 0.0, 3.0, 0.6, 1.0),   # short bar, 1.0 m
        Rect3D(14.0, 11.5, 1.0, 0.6, 3.0, 3.0),  # hanging overhang, bottom z=1.0 to ceiling
    ]
