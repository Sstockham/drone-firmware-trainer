import math
from firmware.contract3d import SensorPacket3D, MotorCommand3D

GOAL_X, GOAL_Y, GOAL_Z = 18.0, 13.0, 1.5
ALT_HOLD = 1.5
K_Z = 0.5

class Firmware:
    def __init__(self) -> None:
        pass

    def step(self, sensors: SensorPacket3D) -> MotorCommand3D:
        px, py, pz = sensors.pos_estimate
        dx, dy = GOAL_X - px, GOAL_Y - py
        norm = math.hypot(dx, dy) or 1.0
        ux, uy = dx / norm, dy / norm

        forward = sensors.rays_h[0]
        if forward < 1.0:
            fx, fy = -ux * 0.3, -uy * 0.3
        else:
            fx, fy = ux * 0.8, uy * 0.8

        fz = K_Z * (ALT_HOLD - pz)
        return MotorCommand3D(thrust=(fx, fy, fz)).clipped()
