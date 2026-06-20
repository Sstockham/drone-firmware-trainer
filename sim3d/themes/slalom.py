"""Slalom — tall walls with right-side gaps + a short wall to fly over.

Hand-crafted. Ignores seed and difficulty. The drone routes RIGHT past the
first tall wall, flies OVER the short full-width wall in the middle, then
routes RIGHT past the second tall wall to reach the goal at (18, 13). Mixes
horizontal routing with altitude awareness — same skills the v2_3d demo
firmware exercises in the demo theme.
"""

from sim3d.world import Rect3D


def generate(seed: int, difficulty: int) -> list[Rect3D]:
    del seed, difficulty
    return [
        Rect3D(1.0,  4.0, 0.0, 13.0, 0.5, 4.0),   # tall wall y=4, gap on the right (x in [14, 19])
        Rect3D(1.0,  8.0, 0.0, 18.0, 0.5, 1.0),   # short wall y=8, full-width, fly OVER (drone at z=1.5 clears z=1.0)
        Rect3D(1.0, 11.0, 0.0, 13.0, 0.5, 4.0),   # tall wall y=11, gap on the right (x in [14, 19])
    ]
