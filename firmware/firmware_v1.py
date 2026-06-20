import math
from firmware.contract import SensorPacket, MotorCommand

GOAL_X, GOAL_Y = 18.0, 13.0

class Firmware:
    def __init__(self) -> None:
        self._last_forward = 4.0

    def step(self, sensors: SensorPacket) -> MotorCommand:
        px, py = sensors.pos_estimate
        dx, dy = GOAL_X - px, GOAL_Y - py
        norm = math.hypot(dx, dy) or 1.0
        ux, uy = dx / norm, dy / norm

        forward = sensors.rays[0]
        self._last_forward = forward

        if forward < 1.0:
            # naive brake — does NOT steer around diagonal obstacles
            fx, fy = -ux * 0.3, -uy * 0.3
        else:
            fx, fy = ux * 0.8, uy * 0.8

        return MotorCommand(thrust=(fx, fy)).clipped()
