import random
from dataclasses import dataclass

ARENA_W = 20.0
ARENA_D = 15.0
CEILING = 4.0
SPAWN = (1.0, 1.0, 1.5)
GOAL_CENTER = (18.0, 13.0, 1.5)
GOAL_RADIUS = 0.75

@dataclass(frozen=True)
class Rect3D:
    x: float
    y: float
    z: float
    w: float
    d: float
    h: float

    def contains_point3d(self, px: float, py: float, pz: float) -> bool:
        return (
            self.x <= px <= self.x + self.w
            and self.y <= py <= self.y + self.d
            and self.z <= pz <= self.z + self.h
        )

@dataclass
class World3D:
    arena_w: float
    arena_d: float
    ceiling: float
    obstacles: list[Rect3D]
    spawn: tuple[float, float, float]
    goal_center: tuple[float, float, float]
    goal_radius: float

def _hand_crafted_seed_42() -> list[Rect3D]:
    # Three set-pieces: tall wall, short bar to fly over, hanging overhang to duck under.
    return [
        Rect3D(6.0, 6.0, 0.0, 4.0, 0.6, 4.0),    # tall wall, full height
        Rect3D(11.0, 9.0, 0.0, 3.0, 0.6, 1.0),   # short bar, 1.0 m
        Rect3D(14.0, 11.5, 1.0, 0.6, 3.0, 3.0),  # hanging overhang, bottom z=1.0 to ceiling
    ]

def _random_obstacles3d(seed: int) -> list[Rect3D]:
    rng = random.Random(seed)
    rects: list[Rect3D] = []
    attempts = 0
    while len(rects) < 8 and attempts < 200:
        attempts += 1
        x = rng.uniform(3.0, 16.0)
        y = rng.uniform(2.0, 12.0)
        h = rng.uniform(0.8, 4.0)
        # 50/50 floor-standing vs ceiling-hung
        z = 0.0 if rng.random() < 0.5 else max(0.0, CEILING - h)
        if rng.random() < 0.5:
            w, d = rng.uniform(0.8, 2.5), 0.6
        else:
            w, d = 0.6, rng.uniform(0.8, 2.5)
        cand = Rect3D(x, y, z, w, d, h)
        if cand.contains_point3d(*SPAWN) or cand.contains_point3d(*GOAL_CENTER):
            continue
        rects.append(cand)
    return rects

def build_world3d(seed: int) -> World3D:
    obstacles = _hand_crafted_seed_42() if seed == 42 else _random_obstacles3d(seed)
    return World3D(
        arena_w=ARENA_W,
        arena_d=ARENA_D,
        ceiling=CEILING,
        obstacles=obstacles,
        spawn=SPAWN,
        goal_center=GOAL_CENTER,
        goal_radius=GOAL_RADIUS,
    )
