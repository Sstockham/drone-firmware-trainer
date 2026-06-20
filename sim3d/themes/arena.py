"""Arena — a central tall tower flanked by four short bars.

Hand-crafted. Ignores seed and difficulty. The drone has to route around the
central tower (left or right). The short bars are 1.0 m tall — v2_3d can fly
over them with its ceiling/floor repulsion, v1_3d's plain altitude hold
either skirts or clips depending on path.
"""

from sim3d.world import Rect3D


def generate(seed: int, difficulty: int) -> list[Rect3D]:
    del seed, difficulty
    return [
        # Central tall tower (3 m x 3 m footprint, full ceiling height) blocks the direct path.
        Rect3D(8.5, 6.0, 0.0, 3.0, 3.0, 4.0),
        # Four short floor-standing bars to either side, low enough to fly over.
        Rect3D(3.0,  4.0, 0.0, 2.0, 0.5, 1.0),   # south-west low horizontal bar
        Rect3D(15.0, 4.0, 0.0, 2.0, 0.5, 1.0),   # south-east low horizontal bar
        Rect3D(3.0, 10.5, 0.0, 0.5, 2.0, 1.0),   # north-west low vertical bar
        Rect3D(15.0,10.5, 0.0, 0.5, 2.0, 1.0),   # north-east low vertical bar
    ]
