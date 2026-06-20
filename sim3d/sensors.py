import math
import numpy as np

from firmware.contract3d import SensorPacket3D
from sim3d.physics import DroneState3D
from sim3d.world import Rect3D, CEILING

RAY_COUNT_H = 8
RAY_MAX = 4.0
RAY_STEP = 0.05

RAY_NOISE = 0.03
ACCEL_NOISE = 0.05
POS_NOISE = 0.05
YAW_NOISE = 0.02
QUANT = 0.01

def _q(v: float) -> float:
    return round(v / QUANT) * QUANT

def _ray_cast_h(px: float, py: float, pz: float, ang: float, rects: list[Rect3D]) -> float:
    cx, cy = math.cos(ang), math.sin(ang)
    d = 0.0
    while d < RAY_MAX:
        d += RAY_STEP
        x = px + cx * d
        y = py + cy * d
        for r in rects:
            if r.contains_point3d(x, y, pz):
                return d
    return RAY_MAX

def _ray_cast_up(px: float, py: float, pz: float, rects: list[Rect3D]) -> float:
    ceiling_d = max(0.0, CEILING - pz)
    limit = min(RAY_MAX, ceiling_d)
    d = 0.0
    while d < limit:
        d += RAY_STEP
        for r in rects:
            if r.contains_point3d(px, py, pz + d):
                return d
    return limit

def _ray_cast_down(px: float, py: float, pz: float, rects: list[Rect3D]) -> float:
    limit = min(RAY_MAX, max(0.0, pz))
    d = 0.0
    while d < limit:
        d += RAY_STEP
        for r in rects:
            if r.contains_point3d(px, py, pz - d):
                return d
    return limit

def build_sensor_packet3d(
    state: DroneState3D,
    rects: list[Rect3D],
    dt: float,
    rng: np.random.Generator,
) -> SensorPacket3D:
    yaw_true = math.atan2(state.vy, state.vx) if (state.vx or state.vy) else 0.0

    rays_clean_h = []
    for i in range(RAY_COUNT_H):
        ang = yaw_true + i * (2 * math.pi / RAY_COUNT_H)
        rays_clean_h.append(_ray_cast_h(state.x, state.y, state.z, ang, rects))
    rays_h = tuple(
        _q(max(0.0, min(RAY_MAX, d + float(rng.normal(0.0, RAY_NOISE)))))
        for d in rays_clean_h
    )

    ray_up_clean = _ray_cast_up(state.x, state.y, state.z, rects)
    ray_down_clean = _ray_cast_down(state.x, state.y, state.z, rects)
    ray_up = _q(max(0.0, min(RAY_MAX, ray_up_clean + float(rng.normal(0.0, RAY_NOISE)))))
    ray_down = _q(max(0.0, min(RAY_MAX, ray_down_clean + float(rng.normal(0.0, RAY_NOISE)))))

    accel = (
        _q(float(rng.normal(0.0, ACCEL_NOISE))),
        _q(float(rng.normal(0.0, ACCEL_NOISE))),
        _q(float(rng.normal(0.0, ACCEL_NOISE))),
    )
    yaw = _q(yaw_true + float(rng.normal(0.0, YAW_NOISE)))
    pos = (
        _q(state.x + float(rng.normal(0.0, POS_NOISE))),
        _q(state.y + float(rng.normal(0.0, POS_NOISE))),
        _q(state.z + float(rng.normal(0.0, POS_NOISE))),
    )
    return SensorPacket3D(
        rays_h=rays_h,
        ray_up=ray_up,
        ray_down=ray_down,
        imu_accel=accel,
        yaw=yaw,
        pos_estimate=pos,
        dt=dt,
    )
