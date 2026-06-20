from dataclasses import dataclass

@dataclass(frozen=True)
class SensorPacket:
    rays: tuple[float, float, float, float, float, float, float, float]
    imu_accel: tuple[float, float]
    yaw: float
    pos_estimate: tuple[float, float]
    dt: float

@dataclass(frozen=True)
class MotorCommand:
    thrust: tuple[float, float]

    @staticmethod
    def zero() -> "MotorCommand":
        return MotorCommand(thrust=(0.0, 0.0))

    def clipped(self) -> "MotorCommand":
        fx, fy = self.thrust
        return MotorCommand(thrust=(max(-1.0, min(1.0, fx)), max(-1.0, min(1.0, fy))))
