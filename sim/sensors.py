import math
import numpy as np
from firmware.contract import SensorPacket
from sim.physics import DroneState
from sim.world import Rect

RAY_COUNT = 8
RAY_MAX = 4.0
RAY_STEP = 0.05  # meters per step in the dumb march

RAY_NOISE = 0.03
ACCEL_NOISE = 0.05
POS_NOISE = 0.05
YAW_NOISE = 0.02
QUANT = 0.01

def _q(v: float) -> float:
    return round(v / QUANT) * QUANT

def _ray_cast(px: float, py: float, ang: float, rects: list[Rect]) -> float:
    cx, cy = math.cos(ang), math.sin(ang)
    d = 0.0
    while d < RAY_MAX:
        d += RAY_STEP
        x = px + cx * d
        y = py + cy * d
        for r in rects:
            if r.contains_point(x, y):
                return d
    return RAY_MAX

def build_sensor_packet(
    state: DroneState,
    rects: list[Rect],
    dt: float,
    rng: np.random.Generator,
) -> SensorPacket:
    yaw_true = math.atan2(state.vy, state.vx) if (state.vx or state.vy) else 0.0
    rays_clean = []
    for i in range(RAY_COUNT):
        ang = yaw_true + i * (2 * math.pi / RAY_COUNT)
        rays_clean.append(_ray_cast(state.x, state.y, ang, rects))
    rays_noisy = tuple(
        _q(max(0.0, min(RAY_MAX, d + float(rng.normal(0.0, RAY_NOISE)))))
        for d in rays_clean
    )
    accel = (
        _q(float(rng.normal(0.0, ACCEL_NOISE))),
        _q(float(rng.normal(0.0, ACCEL_NOISE))),
    )
    yaw = _q(yaw_true + float(rng.normal(0.0, YAW_NOISE)))
    pos = (
        _q(state.x + float(rng.normal(0.0, POS_NOISE))),
        _q(state.y + float(rng.normal(0.0, POS_NOISE))),
    )
    return SensorPacket(
        rays=rays_noisy,
        imu_accel=accel,
        yaw=yaw,
        pos_estimate=pos,
        dt=dt,
    )
