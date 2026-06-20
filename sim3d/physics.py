from dataclasses import dataclass
from sim3d.world import Rect3D, CEILING

MASS = 0.5
DRAG = 0.4

@dataclass
class DroneState3D:
    x: float
    y: float
    z: float
    vx: float
    vy: float
    vz: float
    t: float

def make_drone3d(spawn: tuple[float, float, float]) -> DroneState3D:
    return DroneState3D(x=spawn[0], y=spawn[1], z=spawn[2], vx=0.0, vy=0.0, vz=0.0, t=0.0)

def step_physics3d(state: DroneState3D, thrust: tuple[float, float, float], dt: float) -> None:
    fx = max(-1.0, min(1.0, thrust[0]))
    fy = max(-1.0, min(1.0, thrust[1]))
    fz = max(-1.0, min(1.0, thrust[2]))
    ax = (fx - DRAG * state.vx) / MASS
    ay = (fy - DRAG * state.vy) / MASS
    az = (fz - DRAG * state.vz) / MASS
    state.vx += ax * dt
    state.vy += ay * dt
    state.vz += az * dt
    state.x += state.vx * dt
    state.y += state.vy * dt
    state.z += state.vz * dt
    if state.z < 0.0:
        state.z = 0.0
        if state.vz < 0.0:
            state.vz = 0.0
    elif state.z > CEILING:
        state.z = CEILING
        if state.vz > 0.0:
            state.vz = 0.0
    state.t += dt

def is_inside_any3d(rects: list[Rect3D], px: float, py: float, pz: float) -> bool:
    return any(r.contains_point3d(px, py, pz) for r in rects)

def min_clearance3d(rects: list[Rect3D], px: float, py: float, pz: float) -> float:
    if not rects:
        return float("inf")
    best = float("inf")
    for r in rects:
        cx = max(r.x, min(px, r.x + r.w))
        cy = max(r.y, min(py, r.y + r.d))
        cz = max(r.z, min(pz, r.z + r.h))
        d = ((px - cx) ** 2 + (py - cy) ** 2 + (pz - cz) ** 2) ** 0.5
        if r.contains_point3d(px, py, pz):
            d = -d
        if d < best:
            best = d
    return best
