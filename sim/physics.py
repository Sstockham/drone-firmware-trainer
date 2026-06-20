from dataclasses import dataclass
from sim.world import Rect

MASS = 0.5
DRAG = 0.4

@dataclass
class DroneState:
    x: float
    y: float
    vx: float
    vy: float
    t: float

def make_drone(spawn: tuple[float, float]) -> DroneState:
    return DroneState(x=spawn[0], y=spawn[1], vx=0.0, vy=0.0, t=0.0)

def step_physics(state: DroneState, thrust: tuple[float, float], dt: float) -> None:
    fx = max(-1.0, min(1.0, thrust[0]))
    fy = max(-1.0, min(1.0, thrust[1]))
    ax = (fx - DRAG * state.vx) / MASS
    ay = (fy - DRAG * state.vy) / MASS
    state.vx += ax * dt
    state.vy += ay * dt
    state.x += state.vx * dt
    state.y += state.vy * dt
    state.t += dt

def is_inside_any(rects: list[Rect], px: float, py: float) -> bool:
    return any(r.contains_point(px, py) for r in rects)

def min_clearance(rects: list[Rect], px: float, py: float) -> float:
    if not rects:
        return float("inf")
    best = float("inf")
    for r in rects:
        cx = max(r.x, min(px, r.x + r.w))
        cy = max(r.y, min(py, r.y + r.h))
        d = ((px - cx) ** 2 + (py - cy) ** 2) ** 0.5
        if r.contains_point(px, py):
            d = -d
        if d < best:
            best = d
    return best
