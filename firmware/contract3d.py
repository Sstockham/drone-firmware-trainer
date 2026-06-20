from dataclasses import dataclass

@dataclass(frozen=True)
class SensorPacket3D:
    rays_h: tuple[float, float, float, float, float, float, float, float]
    ray_up: float
    ray_down: float
    imu_accel: tuple[float, float, float]
    yaw: float
    pos_estimate: tuple[float, float, float]
    dt: float

@dataclass(frozen=True)
class MotorCommand3D:
    thrust: tuple[float, float, float]

    @staticmethod
    def zero() -> "MotorCommand3D":
        return MotorCommand3D(thrust=(0.0, 0.0, 0.0))

    def clipped(self) -> "MotorCommand3D":
        fx, fy, fz = self.thrust
        return MotorCommand3D(thrust=(
            max(-1.0, min(1.0, fx)),
            max(-1.0, min(1.0, fy)),
            max(-1.0, min(1.0, fz)),
        ))
