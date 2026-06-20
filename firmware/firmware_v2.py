import math
from firmware.contract import SensorPacket, MotorCommand

GOAL_X, GOAL_Y = 18.0, 13.0
RAY_MAX = 4.0
REPULSION_GAIN = 1.5
ATTRACTION_GAIN = 0.8

class Firmware:
    def __init__(self) -> None:
        self._last_thrust = (0.0, 0.0)

    def step(self, sensors: SensorPacket) -> MotorCommand:
        px, py = sensors.pos_estimate
        yaw = sensors.yaw

        # attraction toward goal
        dx, dy = GOAL_X - px, GOAL_Y - py
        norm = math.hypot(dx, dy) or 1.0
        ax = ATTRACTION_GAIN * dx / norm
        ay = ATTRACTION_GAIN * dy / norm

        # repulsion from all 8 rays
        rx = ry = 0.0
        n = len(sensors.rays)
        for i, ray in enumerate(sensors.rays):
            if ray >= RAY_MAX * 0.95:
                continue
            ang = yaw + i * (2 * math.pi / n)
            strength = REPULSION_GAIN * (1.0 - ray / RAY_MAX) ** 2
            rx -= math.cos(ang) * strength
            ry -= math.sin(ang) * strength

        # low-pass smoothing
        fx_raw = ax + rx
        fy_raw = ay + ry
        fx = 0.7 * fx_raw + 0.3 * self._last_thrust[0]
        fy = 0.7 * fy_raw + 0.3 * self._last_thrust[1]
        self._last_thrust = (fx, fy)

        return MotorCommand(thrust=(fx, fy)).clipped()
