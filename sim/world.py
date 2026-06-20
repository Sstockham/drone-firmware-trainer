import random
from dataclasses import dataclass, field

ARENA_W = 20.0
ARENA_H = 15.0
SPAWN = (1.0, 1.0)
GOAL_CENTER = (18.0, 13.0)
GOAL_RADIUS = 0.75

@dataclass(frozen=True)
class Rect:
    x: float
    y: float
    w: float
    h: float

    def contains_point(self, px: float, py: float) -> bool:
        return self.x <= px <= self.x + self.w and self.y <= py <= self.y + self.h

@dataclass
class World:
    arena_w: float
    arena_h: float
    obstacles: list[Rect]
    spawn: tuple[float, float]
    goal_center: tuple[float, float]
    goal_radius: float

def _hand_crafted_seed_42() -> list[Rect]:
    # Diagonal wall with a gap that requires steering off the direct goal line.
    # firmware_v1 only reads forward ray and will smack into rect (6,6,4,0.6).
    return [
        Rect(6.0, 6.0, 4.0, 0.6),   # horizontal wall blocking direct path
        Rect(11.0, 8.5, 0.6, 4.5),  # vertical wall after the gap
        Rect(3.0, 10.0, 2.0, 0.6),
        Rect(14.0, 4.0, 0.6, 3.0),
        Rect(8.5, 2.0, 0.6, 3.0),
    ]

def _random_obstacles(seed: int) -> list[Rect]:
    rng = random.Random(seed)
    rects: list[Rect] = []
    attempts = 0
    while len(rects) < 9 and attempts < 200:
        attempts += 1
        x = rng.uniform(3.0, 16.0)
        y = rng.uniform(2.0, 12.0)
        if rng.random() < 0.5:
            w, h = rng.uniform(1.0, 3.0), 0.6
        else:
            w, h = 0.6, rng.uniform(1.0, 3.0)
        cand = Rect(x, y, w, h)
        if cand.contains_point(*SPAWN) or cand.contains_point(*GOAL_CENTER):
            continue
        rects.append(cand)
    return rects

def build_world(seed: int) -> World:
    obstacles = _hand_crafted_seed_42() if seed == 42 else _random_obstacles(seed)
    return World(
        arena_w=ARENA_W,
        arena_h=ARENA_H,
        obstacles=obstacles,
        spawn=SPAWN,
        goal_center=GOAL_CENTER,
        goal_radius=GOAL_RADIUS,
    )
